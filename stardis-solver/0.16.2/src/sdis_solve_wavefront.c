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

/* Phase B-2: Wavefront path stepping for solve_tile.
 *
 * This file implements the wavefront execution model for the stardis-solver.
 * Instead of running each path to completion (depth-first), all active
 * paths advance one step in lockstep.  After each step the ray requests are
 * collected into a batch, traced via s3d_scene_view_trace_rays_batch_ctx(),
 * and results distributed back.
 *
 * The physical algorithms are *not* changed — only the execution order.
 * Because each path now has its own RNG stream the per-pixel random
 * sequences differ from serial mode, but the Monte-Carlo estimator's
 * expected value is unchanged.
 *
 * Reference: guide/upper-parallelization/phase_b2_implementation.md
 */

#include "sdis_solve_wavefront.h"
#include "sdis_wf_steps.h"
#include "sdis.h"

#include <rsys/float33.h>   /* f33_rotation, f33_mulf3 (B-4 M1 ENC query) */
#include <rsys/rsys_math.h> /* PI */
#include "sdis_brdf.h"
#include "sdis_c.h"
#include "sdis_camera.h"
#include "sdis_device_c.h"
#include "sdis_estimator_buffer_c.h"
#include "sdis_heat_path_boundary_c.h"
#include "sdis_heat_path_conductive_c.h"
#include "sdis_interface_c.h"
#include "sdis_log.h"
#include "sdis_medium_c.h"
#include "sdis_misc.h"
#include "sdis_radiative_env_c.h"
#include "sdis_realisation.h"
#include "sdis_scene_c.h"
#include "sdis_tile.h"

#include <rsys/clock_time.h>
#include <rsys/morton.h>

#include <float.h>
#include <stdlib.h>
#include <string.h>

/*******************************************************************************
 * Wavefront context — allocation / deallocation
 ******************************************************************************/
static res_T
wf_context_create(struct wavefront_context* wf, size_t total_paths)
{
  ASSERT(wf && total_paths > 0);
  memset(wf, 0, sizeof(*wf));
  wf->total_paths = total_paths;

  wf->paths = (struct path_state*)calloc(total_paths, sizeof(struct path_state));
  if(!wf->paths) return RES_MEM_ERR;

  /* Each path can request up to 6 rays per step (B-4 M1: ENC query) */
  wf->max_rays = total_paths * 6;
  wf->ray_requests = (struct s3d_ray_request*)malloc(
    wf->max_rays * sizeof(struct s3d_ray_request));
  wf->ray_to_path = (uint32_t*)malloc(wf->max_rays * sizeof(uint32_t));
  wf->ray_slot    = (uint32_t*)malloc(wf->max_rays * sizeof(uint32_t));
  wf->ray_hits    = (struct s3d_hit*)malloc(wf->max_rays * sizeof(struct s3d_hit));

  if(!wf->ray_requests || !wf->ray_to_path
  || !wf->ray_slot     || !wf->ray_hits) {
    return RES_MEM_ERR;
  }
  return RES_OK;
}

static void
wf_context_destroy(struct wavefront_context* wf)
{
  if(!wf) return;
  if(wf->batch_ctx)    s3d_batch_trace_context_destroy(wf->batch_ctx);
  free(wf->paths);
  free(wf->ray_requests);
  free(wf->ray_to_path);
  free(wf->ray_slot);
  free(wf->ray_hits);
  memset(wf, 0, sizeof(*wf));
}

/*******************************************************************************
 * Path initialisation — mirrors solve_pixel / ray_realisation_3d setup
 *
 * Morton order + SPP loop matches the original solve_tile traversal.
 ******************************************************************************/
static res_T
init_all_paths(
  struct wavefront_context* wf,
  struct sdis_scene* scn,
  struct ssp_rng* base_rng,
  const unsigned enc_id,
  const struct sdis_camera* cam,
  const double time_range[2],
  const size_t tile_org[2],
  const size_t tile_size[2],
  const size_t spp,
  const int register_paths,
  const double pix_sz[2],
  const size_t picard_order,
  const enum sdis_diffusion_algorithm diff_algo)
{
  size_t npixels;
  size_t mcode;
  size_t path_idx = 0;

  ASSERT(wf && scn && base_rng && cam && tile_size && pix_sz && time_range);

  /* Adjust pixel count for Morton-order traversal (same as solve_tile) */
  npixels = round_up_pow2(MMAX(tile_size[0], tile_size[1]));
  npixels *= npixels;

  for(mcode = 0; mcode < npixels; mcode++) {
    uint16_t ipix_tile[2];
    size_t ipix_image[2];
    size_t irealisation;

    ipix_tile[0] = morton2D_decode_u16((uint32_t)(mcode >> 0));
    if(ipix_tile[0] >= tile_size[0]) continue;
    ipix_tile[1] = morton2D_decode_u16((uint32_t)(mcode >> 1));
    if(ipix_tile[1] >= tile_size[1]) continue;

    ipix_image[0] = ipix_tile[0] + tile_org[0];
    ipix_image[1] = ipix_tile[1] + tile_org[1];

    for(irealisation = 0; irealisation < spp; irealisation++) {
      struct path_state* p = &wf->paths[path_idx];
      double samp[2];
      double ray_pos[3], ray_dir[3];

      /* Identity */
      p->path_id = (uint32_t)path_idx;
      p->pixel_x = ipix_tile[0];
      p->pixel_y = ipix_tile[1];
      p->realisation_idx = (uint32_t)irealisation;
      p->ipix_image[0] = ipix_image[0];
      p->ipix_image[1] = ipix_image[1];

      /* Share the per-thread RNG (same as original solve_tile) */
      p->rng = base_rng;

      /* Sample time (same RNG call order as solve_pixel) */
      {
        double time = sample_time(p->rng, time_range);

        /* Pixel sub-sample */
        samp[0] = ((double)ipix_image[0]
                  + ssp_rng_canonical(p->rng)) * pix_sz[0];
        samp[1] = ((double)ipix_image[1]
                  + ssp_rng_canonical(p->rng)) * pix_sz[1];

        /* Camera ray */
        camera_ray(cam, samp, ray_pos, ray_dir);

        /* Initialise random walk */
        p->rwalk = RWALK_NULL;
        d3_set(p->rwalk.vtx.P, ray_pos);
        p->rwalk.vtx.time = time;
        p->rwalk.hit_3d = S3D_HIT_NULL;
        p->rwalk.hit_side = SDIS_SIDE_NULL__;
        p->rwalk.enc_id = enc_id;
      }

      /* Initialise rwalk_context */
      p->ctx = RWALK_CONTEXT_NULL;
      p->ctx.heat_path   = NULL; /* wavefront does not track heat paths yet */
      p->ctx.green_path  = NULL;
      p->ctx.Tmin   = scn->tmin;
      p->ctx.Tmin2  = scn->tmin * scn->tmin;
      p->ctx.Tmin3  = scn->tmin * scn->tmin * scn->tmin;
      p->ctx.That   = scn->tmax;
      p->ctx.That2  = scn->tmax * scn->tmax;
      p->ctx.That3  = scn->tmax * scn->tmax * scn->tmax;
      p->ctx.max_branchings = picard_order - 1;
      p->ctx.nbranchings    = SIZE_MAX; /* beginning of realisation */
      p->ctx.irealisation   = irealisation;
      p->ctx.diff_algo      = diff_algo;

      /* Temperature accumulator */
      p->T = TEMPERATURE_NULL;

      /* Set initial radiative direction from camera ray */
      f3_set_d3(p->rad_direction, ray_dir);
      p->rad_bounce_count = 0;
      p->rad_retry_count  = 0;

      /* Set lifecycle */
      p->phase   = PATH_INIT;
      p->active  = 1;
      p->needs_ray = 0;

      /* Zero the scratch areas */
      p->ds_delta_solid = 0;
      p->ds_initialized = 0;
      p->ds_enc_id = ENCLOSURE_ID_NULL;
      p->ds_medium = NULL;
      p->ds_props_ref = SOLID_PROPS_NULL;
      p->ds_green_power_term = 0;
      memset(p->ds_position_start, 0, sizeof(p->ds_position_start));
      p->ds_robust_attempt = 0;
      p->ds_delta = 0;
      p->ds_delta_solid_param = 0;
      p->bnd_reinject_distance = 0;
      p->bnd_solid_enc_id = ENCLOSURE_ID_NULL;
      p->bnd_retry_count = 0;
      p->coupled_nbranchings = -1; /* sentinel: not yet entered sample_coupled_path */
      p->steps_taken = 0;
      p->done_reason = 0;

      path_idx++;
    }
  }

  wf->active_count = path_idx;
  return RES_OK;
}

/* Collect all pending ray requests into the batch arrays */
LOCAL_SYM res_T
collect_ray_requests(struct wavefront_context* wf)
{
  size_t ray_idx = 0;
  size_t i;

  for(i = 0; i < wf->total_paths; i++) {
    struct path_state* p = &wf->paths[i];
    if(!p->active || !p->needs_ray) continue;
    if(p->ray_req.ray_count < 1) continue;

    /* Detailed stats: count by phase type */
    if(p->phase == PATH_RAD_TRACE_PENDING)
      wf->rays_radiative += (size_t)p->ray_req.ray_count;
    else if(p->phase == PATH_COUPLED_COND_DS_PENDING) {
      wf->conductive_steps++;
      if(p->ds_robust_attempt > 0)
        wf->rays_conductive_ds_retry += (size_t)p->ray_req.ray_count;
      else
        wf->rays_conductive_ds += (size_t)p->ray_req.ray_count;
    }

    /* Ray 0 */
    {
      struct s3d_ray_request* rr = &wf->ray_requests[ray_idx];
      rr->origin[0]    = p->ray_req.origin[0];
      rr->origin[1]    = p->ray_req.origin[1];
      rr->origin[2]    = p->ray_req.origin[2];
      rr->direction[0] = p->ray_req.direction[0];
      rr->direction[1] = p->ray_req.direction[1];
      rr->direction[2] = p->ray_req.direction[2];
      rr->range[0]     = p->ray_req.range[0];
      rr->range[1]     = p->ray_req.range[1];
      rr->user_id      = (uint32_t)i;

      /* Filter data: for radiative trace we need the filter;
       * for delta_sphere / convective startup we pass NULL */
      if(p->phase == PATH_RAD_TRACE_PENDING
      && !S3D_HIT_NONE(&p->filter_data_storage.hit_3d)) {
        rr->filter_data = &p->filter_data_storage;
      } else {
        rr->filter_data = NULL;
      }

      wf->ray_to_path[ray_idx] = (uint32_t)i;
      wf->ray_slot[ray_idx]    = 0;
      p->ray_req.batch_idx     = (uint32_t)ray_idx;
      ray_idx++;
    }

    /* Ray 1 (if 2-ray request) */
    if(p->ray_req.ray_count >= 2 && p->ray_count_ext != 6) {
      struct s3d_ray_request* rr = &wf->ray_requests[ray_idx];
      rr->origin[0]    = p->ray_req.origin[0];
      rr->origin[1]    = p->ray_req.origin[1];
      rr->origin[2]    = p->ray_req.origin[2];
      rr->direction[0] = p->ray_req.direction2[0];
      rr->direction[1] = p->ray_req.direction2[1];
      rr->direction[2] = p->ray_req.direction2[2];
      rr->range[0]     = p->ray_req.range2[0];
      rr->range[1]     = p->ray_req.range2[1];
      rr->filter_data  = NULL; /* delta_sphere / reinject: no filter */
      rr->user_id      = (uint32_t)i;

      wf->ray_to_path[ray_idx] = (uint32_t)i;
      wf->ray_slot[ray_idx]    = 1;
      p->ray_req.batch_idx2    = (uint32_t)ray_idx;
      ray_idx++;
    }

    /* B-4 M3: 4-ray solid/solid reinjection.
     * ray_req holds front dirs (2 rays); we emit back dirs (2 more). */
    if(p->ray_count_ext == 4 && p->phase == PATH_BND_SS_REINJECT_SAMPLE) {
      int j;
      /* Store batch indices for ray 0 and ray 1 (already emitted above) */
      p->locals.bnd_ss.batch_idx_frt0 = p->ray_req.batch_idx;
      p->locals.bnd_ss.batch_idx_frt1 = p->ray_req.batch_idx2;

      /* Emit back dir0 (ray 2) */
      {
        struct s3d_ray_request* rr = &wf->ray_requests[ray_idx];
        rr->origin[0]    = p->ray_req.origin[0];
        rr->origin[1]    = p->ray_req.origin[1];
        rr->origin[2]    = p->ray_req.origin[2];
        rr->direction[0] = p->locals.bnd_ss.dir_bck[0][0];
        rr->direction[1] = p->locals.bnd_ss.dir_bck[0][1];
        rr->direction[2] = p->locals.bnd_ss.dir_bck[0][2];
        rr->range[0]     = p->ray_req.range[0];
        rr->range[1]     = p->ray_req.range[1];
        rr->filter_data  = NULL;
        rr->user_id      = (uint32_t)i;

        wf->ray_to_path[ray_idx] = (uint32_t)i;
        wf->ray_slot[ray_idx]    = 2;
        p->locals.bnd_ss.batch_idx_bck0 = (uint32_t)ray_idx;
        ray_idx++;
      }

      /* Emit back dir1 (ray 3) */
      {
        struct s3d_ray_request* rr = &wf->ray_requests[ray_idx];
        rr->origin[0]    = p->ray_req.origin[0];
        rr->origin[1]    = p->ray_req.origin[1];
        rr->origin[2]    = p->ray_req.origin[2];
        rr->direction[0] = p->locals.bnd_ss.dir_bck[1][0];
        rr->direction[1] = p->locals.bnd_ss.dir_bck[1][1];
        rr->direction[2] = p->locals.bnd_ss.dir_bck[1][2];
        rr->range[0]     = p->ray_req.range[0];
        rr->range[1]     = p->ray_req.range[1];
        rr->filter_data  = NULL;
        rr->user_id      = (uint32_t)i;

        wf->ray_to_path[ray_idx] = (uint32_t)i;
        wf->ray_slot[ray_idx]    = 3;
        p->locals.bnd_ss.batch_idx_bck1 = (uint32_t)ray_idx;
        ray_idx++;
      }
      (void)j;
    }

    /* B-4 M1: 6-ray enclosure query.
     * Emit all 6 directional rays from enc_query.directions[].
     * Ray 0 was already emitted above (using direction[0] from ray_req);
     * we replace it and emit all 6 here for clarity. */
    if(p->ray_count_ext == 6 && p->phase == PATH_ENC_QUERY_EMIT) {
      int j;
      /* Overwrite ray 0 with enc direction 0 (ray_req.direction may differ) */
      {
        struct s3d_ray_request* rr = &wf->ray_requests[p->ray_req.batch_idx];
        rr->direction[0] = p->enc_query.directions[0][0];
        rr->direction[1] = p->enc_query.directions[0][1];
        rr->direction[2] = p->enc_query.directions[0][2];
        rr->range[0]     = p->ray_req.range[0];
        rr->range[1]     = p->ray_req.range[1];
        rr->filter_data  = NULL;
      }
      p->enc_query.batch_indices[0] = p->ray_req.batch_idx;

      /* Emit directions 1..5 */
      for(j = 1; j < 6; j++) {
        struct s3d_ray_request* rr = &wf->ray_requests[ray_idx];
        rr->origin[0]    = p->ray_req.origin[0];
        rr->origin[1]    = p->ray_req.origin[1];
        rr->origin[2]    = p->ray_req.origin[2];
        rr->direction[0] = p->enc_query.directions[j][0];
        rr->direction[1] = p->enc_query.directions[j][1];
        rr->direction[2] = p->enc_query.directions[j][2];
        rr->range[0]     = p->ray_req.range[0];
        rr->range[1]     = p->ray_req.range[1];
        rr->filter_data  = NULL;
        rr->user_id      = (uint32_t)i;

        wf->ray_to_path[ray_idx] = (uint32_t)i;
        wf->ray_slot[ray_idx]    = (uint32_t)j;
        p->enc_query.batch_indices[j] = (uint32_t)ray_idx;
        ray_idx++;
      }
    }
  }

  wf->ray_count = ray_idx;
  return RES_OK;
}

/* Distribute batch trace results back to paths and advance them */
static res_T
distribute_and_advance(struct wavefront_context* wf, struct sdis_scene* scn)
{
  size_t r;
  res_T res = RES_OK;

  /* First, deliver ray results to every path that requested rays.
   * B-4 M1: pre-deliver 6-ray ENC results into enc_query.dir_hits[]. */
  for(r = 0; r < wf->ray_count; r++) {
    uint32_t pid = wf->ray_to_path[r];
    uint32_t slot = wf->ray_slot[r];
    struct path_state* p = &wf->paths[pid];

    /* B-4 M1: ENC 6-ray pre-delivery */
    if(p->phase == PATH_ENC_QUERY_EMIT && slot < 6) {
      p->enc_query.dir_hits[slot] = wf->ray_hits[r];
      continue;
    }

    /* B-4 M3: SS 4-ray pre-delivery */
    if(p->phase == PATH_BND_SS_REINJECT_SAMPLE && slot < 4) {
      switch(slot) {
      case 0: p->locals.bnd_ss.ray_frt[0] = wf->ray_hits[r]; break;
      case 1: p->locals.bnd_ss.ray_frt[1] = wf->ray_hits[r]; break;
      case 2: p->locals.bnd_ss.ray_bck[0] = wf->ray_hits[r]; break;
      case 3: p->locals.bnd_ss.ray_bck[1] = wf->ray_hits[r]; break;
      }
      continue;
    }

    if(slot == 0) {
      /* This is hit0 for the path */
      /* We'll dispatch after all results are placed */
    }
    /* Results are in wf->ray_hits[r], indexed by ray order */
    (void)slot;
  }

  /* Now advance each path that had a ray pending */
  for(r = 0; r < wf->ray_count; r++) {
    uint32_t pid = wf->ray_to_path[r];
    uint32_t slot = wf->ray_slot[r];
    struct path_state* p = &wf->paths[pid];

    /* Only process when we see slot 0 (avoids double-dispatch for 2-ray) */
    if(slot != 0) continue;

    {
      const struct s3d_hit* h0 = &wf->ray_hits[p->ray_req.batch_idx];
      const struct s3d_hit* h1 = NULL;
      if(p->ray_req.ray_count >= 2) {
        h1 = &wf->ray_hits[p->ray_req.batch_idx2];
      }

      res = advance_one_step_with_ray(p, scn, h0, h1);
      if(res != RES_OK && res != RES_BAD_OP
      && res != RES_BAD_OP_IRRECOVERABLE) return res;
      if(res == RES_BAD_OP || res == RES_BAD_OP_IRRECOVERABLE) {
        p->phase = PATH_DONE;
        p->active = 0;
        p->done_reason = -1;
        wf->paths_failed++;
        res = RES_OK; /* treated as path failure, not fatal */
      }
      p->steps_taken++;
    }
  }

  /* After distributing ray results, cascade non-ray steps for all paths */
  {
    size_t i;
    for(i = 0; i < wf->total_paths; i++) {
      struct path_state* p = &wf->paths[i];
      if(!p->active) continue;

      /* Run non-ray steps until the path needs a ray or finishes */
      for(;;) {
        int advanced = 0;
        if(p->needs_ray) break;
        if(p->phase == PATH_DONE || p->phase == PATH_ERROR) break;

        /* Skip ray-waiting phases */
        if(path_phase_is_ray_pending(p->phase)) break;

        res = advance_one_step_no_ray(p, scn, &advanced);
        if(res != RES_OK && res != RES_BAD_OP
        && res != RES_BAD_OP_IRRECOVERABLE) return res;
        if(res == RES_BAD_OP || res == RES_BAD_OP_IRRECOVERABLE) {
          p->phase = PATH_DONE;
          p->active = 0;
          p->done_reason = -1;
          wf->paths_failed++;
          res = RES_OK;
          break;
        }
        if(!advanced) break;
        p->steps_taken++;
      }
    }
  }

  return RES_OK;
}

/* Count active paths and compute per-path diagnostics */
static void
update_active_count(struct wavefront_context* wf)
{
  size_t i, count = 0;
  size_t max_depth = 0;
  for(i = 0; i < wf->total_paths; i++) {
    if(wf->paths[i].active) count++;
    if(wf->paths[i].steps_taken > max_depth)
      max_depth = wf->paths[i].steps_taken;
  }
  wf->active_count = count;
  if(max_depth > wf->max_wavefront_depth)
    wf->max_wavefront_depth = max_depth;
}

/* Collect results from completed paths into tile pixels */
static res_T
collect_results(
  struct wavefront_context* wf,
  const size_t tile_size[2],
  struct tile* tile)
{
  size_t i;
  ASSERT(wf && tile_size && tile);

  for(i = 0; i < wf->total_paths; i++) {
    const struct path_state* p = &wf->paths[i];
    struct pixel* pix;

    ASSERT(!p->active); /* all paths should be done */

    pix = tile_at(tile, p->pixel_x, p->pixel_y);

    if(p->T.done) {
      pix->acc_temp.sum  += p->T.value;
      pix->acc_temp.sum2 += p->T.value * p->T.value;
      pix->acc_temp.count += 1;
      /* We don't track time per-realisation in wavefront mode yet */
      pix->acc_time.count += 1;
    }

    /* Classify path termination for statistics */
    switch(p->done_reason) {
    case 1:  wf->paths_done_radiative++;   break; /* radiative miss     */
    case 2:  wf->paths_done_temperature++; break; /* temperature known  */
    case 3:  wf->paths_done_boundary++;    break; /* boundary done      */
    case 4:  wf->paths_done_temperature++; break; /* time rewind        */
    case -1: wf->paths_failed++;           break; /* error              */
    default: break;
    }
  }

  return RES_OK;
}

/*******************************************************************************
 * Public entry point
 ******************************************************************************/
#ifndef SDIS_SOLVE_WAVEFRONT_SKIP_PUBLIC_API
res_T
solve_tile_wavefront(
  struct sdis_scene* scn,
  struct ssp_rng*    base_rng,
  const unsigned     enc_id,
  const struct sdis_camera* cam,
  const double       time_range[2],
  const size_t       tile_org[2],
  const size_t       tile_size[2],
  const size_t       spp,
  const int          register_paths,
  const double       pix_sz[2],
  const size_t       picard_order,
  const enum sdis_diffusion_algorithm diff_algo,
  struct sdis_estimator_buffer* buf,
  struct tile*       tile)
{
  struct wavefront_context wf;
  const size_t npixels = tile_size[0] * tile_size[1];
  const size_t total_paths = npixels * spp;
  res_T res = RES_OK;
  struct time t_start, t_end, t_elapsed;

  ASSERT(scn && base_rng && cam && spp);
  ASSERT(tile_size && tile_size[0] && tile_size[1]);
  ASSERT(pix_sz && pix_sz[0] > 0 && pix_sz[1] > 0 && time_range && tile);

  (void)register_paths; /* heat path tracking not yet supported in wavefront */
  (void)buf;            /* estimator buffer written via write_tile in caller */

  /* ====== Allocate wavefront context ====== */
  res = wf_context_create(&wf, total_paths);
  if(res != RES_OK) goto cleanup;

  /* Create batch trace context (pre-allocated GPU buffers) */
  res = s3d_batch_trace_context_create(&wf.batch_ctx, wf.max_rays);
  if(res != RES_OK) goto cleanup;

  /* ====== Initialise all paths ====== */
  res = init_all_paths(&wf, scn, base_rng, enc_id, cam, time_range,
                       tile_org, tile_size, spp, register_paths,
                       pix_sz, picard_order, diff_algo);
  if(res != RES_OK) goto cleanup;

  /* ====== Initial step: advance all paths from PATH_INIT ====== */
  {
    size_t i;
    for(i = 0; i < wf.total_paths; i++) {
      int advanced = 0;
      struct path_state* p = &wf.paths[i];

      /* Run non-ray steps to get to the first ray request */
      while(p->active && !p->needs_ray
          && p->phase != PATH_DONE && p->phase != PATH_ERROR) {
        if(path_phase_is_ray_pending(p->phase)) break;

        res = advance_one_step_no_ray(p, scn, &advanced);
        if(res != RES_OK && res != RES_BAD_OP
        && res != RES_BAD_OP_IRRECOVERABLE)
            goto cleanup;
        if(res == RES_BAD_OP || res == RES_BAD_OP_IRRECOVERABLE) {
          p->phase = PATH_DONE;
          p->active = 0;
          p->done_reason = -1;
          res = RES_OK;
          break;
        }
        if(!advanced) break;
        p->steps_taken++;
      }
    }
  }

  /* ====== Wavefront main loop ====== */
  time_current(&t_start);
  while(wf.active_count > 0) {
    wf.total_steps++;

    /* Step A: Collect ray requests from all active paths */
    wf.ray_count = 0;
    res = collect_ray_requests(&wf);
    if(res != RES_OK) 
        goto cleanup;

    /* Step B: Batch trace via Phase B-1 */
    if(wf.ray_count > 0) {
      struct s3d_batch_trace_stats stats;
      memset(&stats, 0, sizeof(stats));

      /* Use the global scene view for batch trace, matching the original
       * depth-first trace_ray which always uses scn->s3d_view (the full
       * scene BVH).  Per-path enc_id filtering is handled by
       * filter_data_storage inside each ray_request. */
      res = s3d_scene_view_trace_rays_batch_ctx(
        scn->s3d_view,
        wf.batch_ctx,
        wf.ray_requests,
        wf.ray_count,
        wf.ray_hits,
        &stats);
      if(res != RES_OK) 
          goto cleanup;

      wf.total_rays_traced += wf.ray_count;
    }

    /* Step C: Distribute results + advance paths */
    res = distribute_and_advance(&wf, scn);
    if(res != RES_OK) 
        goto cleanup;

    /* Step D: Update active count */
    update_active_count(&wf);

    /* Safety: prevent infinite loops */
    if(wf.total_steps > total_paths * 1000) {
      log_err(scn->dev,
        "wavefront: exceeded maximum step count (%lu). "
        "Tile (%lu,%lu), %lu paths still active.\n",
        (unsigned long)wf.total_steps,
        (unsigned long)tile_org[0], (unsigned long)tile_org[1],
        (unsigned long)wf.active_count);
      res = RES_BAD_OP;
      goto cleanup;
    }
  }

  /* ====== Collect results into tile ====== */
  res = collect_results(&wf, tile_size, tile);

  /* ====== Summary logging ====== */
  time_current(&t_end);
  time_sub(&t_elapsed, &t_end, &t_start);
  {
    char time_str[128];
    time_dump(&t_elapsed, TIME_ALL, NULL, time_str, sizeof(time_str));
    log_info(scn->dev,
      "wavefront tile (%lu,%lu) %lux%lu spp=%lu:  "
      "elapsed=%s  steps=%lu  rays=%lu  "
      "(rad=%lu cond_ds=%lu ds_retry=%lu)  "
      "done: rad=%lu temp=%lu bnd=%lu fail=%lu  "
      "max_depth=%lu\n",
      (unsigned long)tile_org[0], (unsigned long)tile_org[1],
      (unsigned long)tile_size[0], (unsigned long)tile_size[1],
      (unsigned long)spp,
      time_str,
      (unsigned long)wf.total_steps,
      (unsigned long)wf.total_rays_traced,
      (unsigned long)wf.rays_radiative,
      (unsigned long)wf.rays_conductive_ds,
      (unsigned long)wf.rays_conductive_ds_retry,
      (unsigned long)wf.paths_done_radiative,
      (unsigned long)wf.paths_done_temperature,
      (unsigned long)wf.paths_done_boundary,
      (unsigned long)wf.paths_failed,
      (unsigned long)wf.max_wavefront_depth);
  }

cleanup:
  wf_context_destroy(&wf);
  return res;
}
#endif /* SDIS_SOLVE_WAVEFRONT_SKIP_PUBLIC_API */
