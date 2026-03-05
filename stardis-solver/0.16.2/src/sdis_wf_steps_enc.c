/* Copyright (C) 2016-2025 |Méso|Star> (contact@meso-star.com)
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


/* Wavefront steps: enclosure locate (M10) + 6-ray query (M1-v2).  Split from sdis_wf_steps.c. */

#include "sdis_wf_steps.h"
#include "sdis_solve_wavefront.h"  /* struct wavefront_context (used by types) */
#include "sdis.h"

#include <rsys/float33.h>   /* f33_rotation, f33_mulf3, f33_basis (B-4 M1) */
#include <rsys/rsys_math.h> /* PI */
#include "sdis_brdf.h"
#include "sdis_c.h"
#include "sdis_camera.h"
#include "sdis_device_c.h"
#include "sdis_estimator_buffer_c.h"
#include "sdis_green.h"
#include "sdis_heat_path_boundary_c.h"
#include "sdis_heat_path_conductive_c.h"
#include "sdis_interface_c.h"
#include "sdis_log.h"
#include "sdis_medium_c.h"
#include "sdis_misc.h"
#include "sdis_radiative_env_c.h"
#include "sdis_realisation.h"
#include "sdis_scene_c.h"
#include "sdis_source_c.h"
#include "sdis_tile.h"

#include <float.h>
#include <stdlib.h>
#include <string.h>
#include <star/swf.h>  /* swf_tabulation_inverse, SWF_QUADRATIC (M9 WoS) */

/*******************************************************************************
 * B-4 M10: Point-in-enclosure via BVH closest primitive
 ******************************************************************************/

/* Submit a point-in-enclosure query.  Sets up enc_locate fields and
 * transitions to PATH_ENC_LOCATE_PENDING.  The wavefront pool will
 * collect these requests into a batch before the next GPU dispatch. */
LOCAL_SYM void
step_enc_locate_submit(struct path_state* p,
                       struct path_hot* hot,
                       struct path_enc_data* enc,
                       const double pos[3],
                       enum path_phase return_state)
{
  ASSERT(p && pos);

  enc->locate.query_pos[0] = pos[0];
  enc->locate.query_pos[1] = pos[1];
  enc->locate.query_pos[2] = pos[2];
  enc->locate.return_state = return_state;
  enc->locate.resolved_enc_id = ENCLOSURE_ID_NULL;
  enc->locate.prim_id = -1;
  enc->locate.side = -1;
  enc->locate.distance = -1.0f;
  enc->locate.batch_idx = (uint32_t)-1;

  hot->needs_ray = 0;  /* NOT a ray request — enc_locate has its own batch */
  hot->phase = (uint8_t)PATH_ENC_LOCATE_PENDING;
}

/* Process the result of a batch enc_locate query.
 * At this point enc_locate.prim_id and enc_locate.side have been filled
 * by pool_distribute_enc_locate_results().  Resolve to enc_id. */
LOCAL_SYM res_T
step_enc_locate_result(struct path_state* p, struct path_hot* hot,
                       struct sdis_scene* scn,
                       struct path_enc_data* enc)
{
  unsigned enc_id = ENCLOSURE_ID_NULL;
  ASSERT(p && scn);

  if(enc->locate.prim_id >= 0 && enc->locate.side >= 0) {
    /* Valid result: resolve prim_id + side → enc_id via prim_props */
    unsigned enc_ids[2];
    scene_get_enclosure_ids(scn, (unsigned)enc->locate.prim_id, enc_ids);
    enc_id = enc_ids[enc->locate.side]; /* side 0=front, 1=back */
  } else if(enc->locate.prim_id >= 0 && enc->locate.side < 0) {
    /* Degenerate: point is ON a surface (distance < threshold).
     * Accept NULL rather than calling the synchronous brute-force
     * scene_get_enclosure_id() which issues O(nprims) blocking
     * trace_ray calls.  This case is extremely rare and accepting
     * NULL does not affect convergence. */
    enc_id = ENCLOSURE_ID_NULL;
  }
  /* else: prim_id < 0 → miss, enc_id stays ENCLOSURE_ID_NULL */

  enc->locate.resolved_enc_id = enc_id;
  hot->phase = (uint8_t)enc->locate.return_state;
  return RES_OK;
}

/*******************************************************************************
 * B-4 M1-v2: 6-ray enclosure query (replaces M10 at call sites)
 *
 * Replicates scene_get_enclosure_id_in_closed_boundaries_3d logic:
 *   - 6 axis-aligned directions rotated by PI/4 around each axis
 *   - First valid hit (distance > 1e-6, |cos(N,dir)| > 1e-2) resolves
 *     enc_id via scene_get_enclosure_ids + dot-product side selection
 *   - If all 6 fail → fallback 1 random-direction ray (batched)
 *   - If fallback also fails → synchronous scene_get_enclosure_id (last resort)
 ******************************************************************************/

/* --- step_enc_query_emit: build 6 PI/4-rotated dirs + prepare ray request -- */
LOCAL_SYM void
step_enc_query_emit(struct path_state* p,
                    struct path_hot* hot,
                    struct path_enc_data* enc,
                    const double pos[3],
                    enum path_phase return_state)
{
  float frame[9];
  static const float base_dirs[6][3] = {
    { 1, 0, 0}, {-1, 0, 0},
    { 0, 1, 0}, { 0,-1, 0},
    { 0, 0, 1}, { 0, 0,-1}
  };
  int i;

  ASSERT(p && pos);

  enc->query_pos[0] = pos[0];
  enc->query_pos[1] = pos[1];
  enc->query_pos[2] = pos[2];
  enc->return_state = return_state;
  enc->resolved_enc_id = ENCLOSURE_ID_NULL;

  /* Build PI/4 rotation matrix (same as CPU reference) */
  f33_rotation(frame, (float)PI/4, (float)PI/4, (float)PI/4);

  for(i = 0; i < 6; i++) {
    float d[3];
    d[0] = base_dirs[i][0]; d[1] = base_dirs[i][1]; d[2] = base_dirs[i][2];
    f33_mulf3(enc->directions[i], frame, d);
    enc->dir_hits[i] = S3D_HIT_NULL;
    enc->batch_indices[i] = (uint32_t)-1;
  }

  /* Setup ray request: first 2 rays go through standard ray_req,
   * remaining 4 are emitted by collect via enc_query.directions[]. */
  {
    float P[3];
    P[0] = (float)pos[0]; P[1] = (float)pos[1]; P[2] = (float)pos[2];

    p->ray_req.origin[0] = P[0];
    p->ray_req.origin[1] = P[1];
    p->ray_req.origin[2] = P[2];
    p->ray_req.direction[0] = enc->directions[0][0];
    p->ray_req.direction[1] = enc->directions[0][1];
    p->ray_req.direction[2] = enc->directions[0][2];
    p->ray_req.range[0] = FLT_MIN;
    p->ray_req.range[1] = FLT_MAX;
    p->ray_req.direction2[0] = enc->directions[1][0];
    p->ray_req.direction2[1] = enc->directions[1][1];
    p->ray_req.direction2[2] = enc->directions[1][2];
    p->ray_req.range2[0] = FLT_MIN;
    p->ray_req.range2[1] = FLT_MAX;
    p->ray_req.ray_count = 2;
  }

  /* No self-intersection filter for enclosure query */
  p->filter_data_storage = HIT_FILTER_DATA_NULL;

  hot->ray_count_ext = 6;
  hot->ray_bucket = (uint8_t)RAY_BUCKET_ENCLOSURE;
  hot->needs_ray = 1;
  hot->phase = (uint8_t)PATH_ENC_QUERY_EMIT;
}

/* --- step_enc_query_resolve: check 6 hits, pick first valid --------------- */
LOCAL_SYM res_T
step_enc_query_resolve(struct path_state* p, struct path_hot* hot,
                       struct sdis_scene* scn,
                       struct path_enc_data* enc)
{
  unsigned enc_id = ENCLOSURE_ID_NULL;
  int idir;

  ASSERT(p && scn);

  for(idir = 0; idir < 6; idir++) {
    const struct s3d_hit* hit = &enc->dir_hits[idir];
    float N[3], cos_N_dir;

    if(S3D_HIT_NONE(hit)) continue;

    /* Distance must be > 1e-6 (not touching surface) */
    if(hit->distance <= 1.e-6f) continue;

    f3_normalize(N, hit->normal);
    cos_N_dir = f3_dot(N, enc->directions[idir]);

    /* Not roughly orthogonal */
    if(absf(cos_N_dir) <= 1.e-2f) continue;

    /* Valid hit — resolve enclosure from primitive */
    {
      unsigned enc_ids[2];
      scene_get_enclosure_ids(scn, hit->prim.prim_id, enc_ids);
      enc_id = cos_N_dir < 0 ? enc_ids[0] : enc_ids[1];
    }
    break; /* First valid hit is sufficient */
  }

  if(idir < 6) {
    /* Resolved from primary 6 rays */
    enc->resolved_enc_id = enc_id;
    hot->phase = (uint8_t)enc->return_state;
    hot->needs_ray = 0;
    return RES_OK;
  }

  /* All 6 failed → emit 1 fallback ray with a different random direction.
   * Use a simple diagonal direction that avoids axis alignment. */
  {
    float P[3];
    static const float fb_dir[3] = {0.57735027f, 0.57735027f, 0.57735027f};
    P[0] = (float)enc->query_pos[0];
    P[1] = (float)enc->query_pos[1];
    P[2] = (float)enc->query_pos[2];

    enc->fb_direction[0] = fb_dir[0];
    enc->fb_direction[1] = fb_dir[1];
    enc->fb_direction[2] = fb_dir[2];
    enc->fb_hit = S3D_HIT_NULL;
    enc->fb_batch_idx = (uint32_t)-1;

    p->ray_req.origin[0] = P[0];
    p->ray_req.origin[1] = P[1];
    p->ray_req.origin[2] = P[2];
    p->ray_req.direction[0] = fb_dir[0];
    p->ray_req.direction[1] = fb_dir[1];
    p->ray_req.direction[2] = fb_dir[2];
    p->ray_req.range[0] = FLT_MIN;
    p->ray_req.range[1] = FLT_MAX;
    p->ray_req.ray_count = 1;

    p->filter_data_storage = HIT_FILTER_DATA_NULL;

    hot->ray_count_ext = 1;
    hot->ray_bucket = (uint8_t)RAY_BUCKET_ENCLOSURE;
    hot->needs_ray = 1;
    hot->phase = (uint8_t)PATH_ENC_QUERY_FB_EMIT;
  }
  return RES_OK;
}

/* --- step_enc_query_fb_resolve: check 1 fallback hit --------------------- */
LOCAL_SYM res_T
step_enc_query_fb_resolve(struct path_state* p, struct path_hot* hot,
                          struct sdis_scene* scn,
                          struct path_enc_data* enc)
{
  unsigned enc_id = ENCLOSURE_ID_NULL;
  const struct s3d_hit* hit;

  ASSERT(p && scn);

  hit = &enc->fb_hit;

  if(!S3D_HIT_NONE(hit) && hit->distance > 1.e-6f) {
    float N[3], cos_N_dir;
    f3_normalize(N, hit->normal);
    cos_N_dir = f3_dot(N, enc->fb_direction);

    if(absf(cos_N_dir) > 1.e-2f) {
      unsigned enc_ids[2];
      scene_get_enclosure_ids(scn, hit->prim.prim_id, enc_ids);
      enc_id = cos_N_dir < 0 ? enc_ids[0] : enc_ids[1];
    }
  }

  if(enc_id == ENCLOSURE_ID_NULL) {
    /* Escalate to M10 BVH closest-point batch instead of synchronous
     * brute-force.  The return_state is forwarded so that when M10
     * resolves, the path resumes at the original caller.  enc_locate
     * and enc_query are independent top-level members of path_state,
     * so writing enc_locate.* does not clobber enc_query.*. */
    step_enc_locate_submit(p, hot, enc,
                           enc->query_pos,
                           enc->return_state);
    /* p->phase is now PATH_ENC_LOCATE_PENDING; cascade will break out
     * and the next main-loop Step D2 will collect this request. */
    return RES_OK;
  }

  enc->resolved_enc_id = enc_id;
  hot->phase = (uint8_t)enc->return_state;
  hot->needs_ray = 0;
  return RES_OK;
}

