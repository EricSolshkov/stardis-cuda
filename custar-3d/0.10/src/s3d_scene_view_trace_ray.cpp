/* Copyright (C) 2015-2023 |Méso|Star> (contact@meso-star.com)
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>. */

/*
 * cuBQL backend for s3d_scene_view_trace_ray / trace_rays.
 *
 * Replaces the Embree-based implementation.  Uses Top-K multi-hit GPU trace
 * (cus3d_trace_ray_single_multi) with CPU-side filter evaluation.  If all K
 * candidates are rejected by the filter, falls back to re-tracing with an
 * advanced range (scheme A).
 *
 * Key conventions:
 *   - Moller-Trumbore (u,v) -> s3d (w=1-u-v, u) for triangle UV
 *   - Triangle normals negated (cross(e1,e2) vs s3d CW winding)
 *   - flip_surface already applied in the GPU kernel
 */

#include "s3d.h"
#include "s3d_c.h"
#include "s3d_device_c.h"
#include "s3d_scene_view_c.h"
#include "cus3d_trace.h"
#include "cus3d_prim.h"
#include "cus3d_geom_store.h"
#include "cus3d_trace_util.h"

#include <rsys/float3.h>
#include <float.h>

/* Number of Top-K candidates returned by a single GPU trace.
 * 4 is sufficient for the typical self-intersection-avoidance case
 * (1 self-hit + 1 valid hit), with margin for multi-layer scenes.
 * K=8 covers worst-case dense meshes: 1 self-hit + ~2 shared-edge
 * hits + ~1 boundary enclosure hit + margin. */
#define TOPK_COUNT CUS3D_MAX_MULTI_HITS

/* Maximum recursive fallback depth when all K candidates are rejected.
 * Prevents infinite recursion in degenerate scenes. */
#define MAX_FALLBACK_DEPTH 4

/*******************************************************************************
 * Internal: recursive trace with fallback
 ******************************************************************************/
static res_T
trace_ray_impl
  (struct s3d_scene_view* scnview,
   const float org[3],
   const float dir[3],
   const float range[2],
   void* ray_data,
   struct s3d_hit* hit,
   int depth)
{
  struct cus3d_multi_hit_result multi;
  int i;
  res_T res;

  /* Guard against infinite recursion */
  if(depth >= MAX_FALLBACK_DEPTH) {
    *hit = S3D_HIT_NULL;
    return RES_OK;
  }

  /* One GPU call: get Top-K nearest candidates */
  res = cus3d_trace_ray_single_multi(
    scnview->bvh,
    scnview->geom_store,
    scnview->scn->dev->gpu,
    org, dir, range,
    TOPK_COUNT,
    &multi);
  if(res != RES_OK) return res;

  if(multi.count == 0) {
    *hit = S3D_HIT_NULL;
    return RES_OK;
  }

  /* Iterate candidates in ascending distance order */
  for(i = 0; i < multi.count; i++) {
    const struct cus3d_hit_result* candidate = &multi.hits[i];
    const struct geom_entry* ge;

    /* Resolve the correct geometry store for this hit.
     * For instanced hits, geom_idx is relative to the child store;
     * for pseudo-instance or direct hits, it's the parent store. */
    const struct cus3d_geom_store* hit_store = scnview->geom_store;
    if (candidate->inst_id >= 0) {
        const struct cus3d_geom_store* cs =
            cus3d_bvh_get_instance_store(scnview->bvh,
                                         (unsigned)candidate->inst_id);
        if (cs) hit_store = cs;
    }

    /* Convert GPU hit to s3d_hit */
    cus3d_hit_to_s3d_hit(hit_store, scnview->bvh, candidate, hit);

    /* Look up geometry entry for UV fixup and filter check */
    ge = cus3d_geom_store_get_entry(
      hit_store, (uint32_t)candidate->geom_idx);
    if(!ge) continue; /* Should not happen; defensive */

    /* Apply UV/normal convention conversion */
    trace_hit_fixup(hit, ge);

    /* Check hit filter function */
    if(ge->filter_func) {
      int rejected = ge->filter_func(
        hit, org, dir, range, ray_data, ge->filter_data);
      if(rejected)
        continue; /* Try next candidate */
    }

    /* Hit accepted */
    return RES_OK;
  }

  /* All K candidates were rejected by the filter.
   * Fall back to scheme A: re-trace from beyond the last candidate. */
  if(multi.count == TOPK_COUNT) {
    float fallback_range[2];
    fallback_range[0] = multi.hits[TOPK_COUNT - 1].distance + 1e-6f;
    fallback_range[1] = range[1];
    if(fallback_range[0] < fallback_range[1]) {
      return trace_ray_impl(scnview, org, dir,
                            fallback_range, ray_data, hit,
                            depth + 1);
    }
  }

  /* No valid hit found */
  *hit = S3D_HIT_NULL;
  return RES_OK;
}

/*******************************************************************************
 * Exported functions
 ******************************************************************************/
res_T
s3d_scene_view_trace_ray
  (struct s3d_scene_view* scnview,
   const float org[3],
   const float dir[3],
   const float range[2],
   void* ray_data,
   struct s3d_hit* hit)
{
  if(!scnview || !org || !dir || !range || !hit)
    return RES_BAD_ARG;
  if(!f3_is_normalized(dir)) {
    log_error(scnview->scn->dev,
      "%s: unnormalized ray direction {%g, %g, %g}.\n",
      FUNC_NAME, SPLIT3(dir));
    return RES_BAD_ARG;
  }
  if(range[0] < 0) {
    log_error(scnview->scn->dev,
      "%s: invalid ray range [%g, %g] - it must be in [0, INF).\n",
      FUNC_NAME, range[0], range[1]);
    return RES_BAD_ARG;
  }
  if((scnview->mask & S3D_TRACE) == 0) {
    log_error(scnview->scn->dev,
      "%s: the S3D_TRACE flag is not active onto the submitted scene view.\n",
      FUNC_NAME);
    return RES_BAD_OP;
  }
  if(range[0] > range[1]) { /* Degenerated range <=> disabled ray */
    *hit = S3D_HIT_NULL;
    return RES_OK;
  }

  return trace_ray_impl(scnview, org, dir, range, ray_data, hit, 0);
}

res_T
s3d_scene_view_trace_rays
  (struct s3d_scene_view* scnview,
   const size_t nrays,
   const int mask,
   const float* origins,
   const float* directions,
   const float* ranges,
   void* rays_data,
   const size_t sizeof_ray_data,
   struct s3d_hit* hits)
{
  size_t iray;
  size_t iorg, idir, irange, idata;
  size_t org_step, dir_step, range_step, data_step;
  res_T res = RES_OK;

  if(!scnview) return RES_BAD_ARG;
  if(!nrays) return RES_OK;

  org_step = mask & S3D_RAYS_SINGLE_ORIGIN ? 0 : 3;
  dir_step = mask & S3D_RAYS_SINGLE_DIRECTION ? 0 : 3;
  range_step = mask & S3D_RAYS_SINGLE_RANGE ? 0 : 2;
  data_step = (mask & S3D_RAYS_SINGLE_DATA) || !rays_data ? 0 : sizeof_ray_data;
  iorg = idir = irange = idata = 0;

  FOR_EACH(iray, 0, nrays) {
    res = s3d_scene_view_trace_ray(scnview, origins+iorg, directions+idir,
      ranges+irange, (char*)rays_data+idata, hits+iray);
    if(UNLIKELY(res != RES_OK)) break;
    iorg += org_step;
    idir += dir_step;
    irange += range_step;
    idata += data_step;
  }
  return res;
}
