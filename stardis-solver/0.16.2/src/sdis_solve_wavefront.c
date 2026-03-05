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

#ifdef SDIS_P0_OPT
/* P0_OPT: old per-tile wavefront disabled — hot fields (phase, active,
 * needs_ray, ray_bucket, ray_count_ext) moved to path_hot and the legacy
 * code that reads them from path_state cannot compile.  The persistent
 * wavefront (sdis_solve_persistent_wavefront.c) is the only active path. */
#else  /* !SDIS_P0_OPT */

#include "sdis_solve_persistent_wavefront.h"  /* P1: wavefront_pool for cold-block proxy */
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

  /* P1: Allocate cold-block SoA arrays + lightweight pool proxy */
  wf->sfn_arr = (struct path_sfn_data*)calloc(total_paths, sizeof(struct path_sfn_data));
  wf->enc_arr = (struct path_enc_data*)calloc(total_paths, sizeof(struct path_enc_data));
  wf->ext_arr = (struct path_ext_data*)calloc(total_paths, sizeof(struct path_ext_data));
  wf->cold_pool = (struct wavefront_pool*)calloc(1, sizeof(struct wavefront_pool));
  if(!wf->sfn_arr || !wf->enc_arr || !wf->ext_arr || !wf->cold_pool)
    return RES_MEM_ERR;
  wf->cold_pool->sfn_arr = wf->sfn_arr;
  wf->cold_pool->enc_arr = wf->enc_arr;
  wf->cold_pool->ext_arr = wf->ext_arr;

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

  /* B-4 M10: enc_locate batch buffers */
  wf->max_enc_locates = total_paths;
  wf->enc_locate_requests = (struct s3d_enc_locate_request*)malloc(
    total_paths * sizeof(struct s3d_enc_locate_request));
  wf->enc_locate_results = (struct s3d_enc_locate_result*)malloc(
    total_paths * sizeof(struct s3d_enc_locate_result));
  wf->enc_locate_to_path = (uint32_t*)malloc(
    total_paths * sizeof(uint32_t));
  if(!wf->enc_locate_requests || !wf->enc_locate_results
  || !wf->enc_locate_to_path) {
    return RES_MEM_ERR;
  }

  /* B-4 M9: closest_point (WoS) batch buffers */
  wf->max_cps = total_paths;
  wf->cp_requests = (struct s3d_cp_request*)malloc(
    total_paths * sizeof(struct s3d_cp_request));
  wf->cp_hits = (struct s3d_hit*)malloc(
    total_paths * sizeof(struct s3d_hit));
  wf->cp_to_path = (uint32_t*)malloc(
    total_paths * sizeof(uint32_t));
  if(!wf->cp_requests || !wf->cp_hits || !wf->cp_to_path) {
    return RES_MEM_ERR;
  }

  return RES_OK;
}

static void
wf_context_destroy(struct wavefront_context* wf)
{
  if(!wf) return;
  if(wf->batch_ctx)    s3d_batch_trace_context_destroy(wf->batch_ctx);
  if(wf->enc_batch_ctx) s3d_batch_enc_context_destroy(wf->enc_batch_ctx);
  if(wf->cp_batch_ctx)  s3d_batch_cp_context_destroy(wf->cp_batch_ctx);
  free(wf->cold_pool);
  free(wf->sfn_arr);
  free(wf->enc_arr);
  free(wf->ext_arr);
  free(wf->paths);
  free(wf->ray_requests);
  free(wf->ray_to_path);
  free(wf->ray_slot);
  free(wf->ray_hits);
  free(wf->enc_locate_requests);
  free(wf->enc_locate_results);
  free(wf->enc_locate_to_path);
  free(wf->cp_requests);
  free(wf->cp_hits);
  free(wf->cp_to_path);
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

      /* Filter data: pass filter whenever the step function set up a
       * valid hit_3d (radiative, reinjection, etc.).  Only phases that
       * intentionally leave filter_data_storage as HIT_NONE (delta-sphere,
       * convective startup, enc-query) will get NULL here. */
      if(!S3D_HIT_NONE(&p->filter_data_storage.hit_3d)) {
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
    if(p->ray_req.ray_count >= 2) {
      struct s3d_ray_request* rr = &wf->ray_requests[ray_idx];
      rr->origin[0]    = p->ray_req.origin[0];
      rr->origin[1]    = p->ray_req.origin[1];
      rr->origin[2]    = p->ray_req.origin[2];
      rr->direction[0] = p->ray_req.direction2[0];
      rr->direction[1] = p->ray_req.direction2[1];
      rr->direction[2] = p->ray_req.direction2[2];
      rr->range[0]     = p->ray_req.range2[0];
      rr->range[1]     = p->ray_req.range2[1];
      if(!S3D_HIT_NONE(&p->filter_data_storage.hit_3d)) {
        rr->filter_data = &p->filter_data_storage;
      } else {
        rr->filter_data = NULL;
      }
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
        if(!S3D_HIT_NONE(&p->filter_data_storage.hit_3d)) {
          rr->filter_data = &p->filter_data_storage;
        } else {
          rr->filter_data = NULL;
        }
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
        if(!S3D_HIT_NONE(&p->filter_data_storage.hit_3d)) {
          rr->filter_data = &p->filter_data_storage;
        } else {
          rr->filter_data = NULL;
        }
        rr->user_id      = (uint32_t)i;

        wf->ray_to_path[ray_idx] = (uint32_t)i;
        wf->ray_slot[ray_idx]    = 3;
        p->locals.bnd_ss.batch_idx_bck1 = (uint32_t)ray_idx;
        ray_idx++;
      }
      (void)j;
    }

    /* B-4 M1-v2: 6-ray enclosure query.
     * ray_req holds first 2 dirs; we emit remaining 4 from enc_arr. */
    if(p->ray_count_ext == 6 && p->phase == PATH_ENC_QUERY_EMIT) {
      int j;
      /* Store batch indices for ray 0 and ray 1 (already emitted) */
      wf->enc_arr[i].batch_indices[0] = p->ray_req.batch_idx;
      wf->enc_arr[i].batch_indices[1] = p->ray_req.batch_idx2;

      /* Emit rays 2..5 */
      for(j = 2; j < 6; j++) {
        struct s3d_ray_request* rr = &wf->ray_requests[ray_idx];
        rr->origin[0]    = p->ray_req.origin[0];
        rr->origin[1]    = p->ray_req.origin[1];
        rr->origin[2]    = p->ray_req.origin[2];
        rr->direction[0] = wf->enc_arr[i].directions[j][0];
        rr->direction[1] = wf->enc_arr[i].directions[j][1];
        rr->direction[2] = wf->enc_arr[i].directions[j][2];
        rr->range[0]     = p->ray_req.range[0];
        rr->range[1]     = p->ray_req.range[1];
        rr->filter_data  = NULL;  /* no self-intersection filter */
        rr->user_id      = (uint32_t)i;

        wf->ray_to_path[ray_idx] = (uint32_t)i;
        wf->ray_slot[ray_idx]    = (uint32_t)j;
        wf->enc_arr[i].batch_indices[j] = (uint32_t)ray_idx;
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

  /* First, deliver ray results to every path that requested rays. */
  for(r = 0; r < wf->ray_count; r++) {
    uint32_t pid = wf->ray_to_path[r];
    uint32_t slot = wf->ray_slot[r];
    struct path_state* p = &wf->paths[pid];

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

    /* B-4 M1-v2: 6-ray enc_query pre-delivery */
    if(p->phase == PATH_ENC_QUERY_EMIT && slot < 6) {
      wf->enc_arr[pid].dir_hits[slot] = wf->ray_hits[r];
      continue;
    }

    /* B-4 M1-v2: 1-ray enc_query fallback pre-delivery */
    if(p->phase == PATH_ENC_QUERY_FB_EMIT && slot == 0) {
      wf->enc_arr[pid].fb_hit = wf->ray_hits[r];
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

      res = advance_one_step_with_ray(p, scn, h0, h1,
                                      wf->cold_pool, (size_t)pid);
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
        if(p->phase == PATH_DONE || p->phase == PATH_ERROR) {
          /* M8: intercept PATH_DONE when sfn_stack_depth > 0. */
          if(p->phase == PATH_DONE && wf->sfn_arr[i].depth > 0) {
            p->phase = PATH_BND_SFN_COMPUTE_Ti_RESUME;
            p->active = 1;
            continue;
          }
          break;
        }

        /* Skip ray-waiting phases */
        if(path_phase_is_ray_pending(p->phase)) break;
        /* Skip enc_locate-waiting phases (M10) */
        if(path_phase_is_enc_locate_pending(p->phase)) break;
        /* Skip closest_point-waiting phases (M9: WoS) */
        if(path_phase_is_cp_pending(p->phase)) break;

        res = advance_one_step_no_ray(p, scn, &advanced,
                                      wf->cold_pool, i);
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

/* ---- Step C3 helper: batch closest_point for all CP-pending WoS paths ---- */
static res_T
wf_batch_closest_point(struct wavefront_context* wf,
                       struct sdis_scene* scn)
{
  size_t i, n = 0;
  res_T res = RES_OK;

  /* Collect CP requests from all CP-pending paths */
  for(i = 0; i < wf->total_paths; i++) {
    struct path_state* p = &wf->paths[i];
    if(p->active && path_phase_is_cp_pending(p->phase)) {
      struct s3d_cp_request* req = &wf->cp_requests[n];
      req->pos[0] = (float)p->locals.cnd_wos.query_pos[0];
      req->pos[1] = (float)p->locals.cnd_wos.query_pos[1];
      req->pos[2] = (float)p->locals.cnd_wos.query_pos[2];
      req->radius = p->locals.cnd_wos.query_radius;
      req->query_data = NULL;
      req->user_id = (uint32_t)i;
      wf->cp_to_path[n] = (uint32_t)i;
      if(p->phase == PATH_CND_WOS_CLOSEST)
        p->locals.cnd_wos.batch_cp_idx = (uint32_t)n;
      else
        p->locals.cnd_wos.batch_cp2_idx = (uint32_t)n;
      n++;
    }
  }
  wf->cp_count = n;

  if(n > 0) {
    struct s3d_batch_cp_stats cp_stats;
    memset(&cp_stats, 0, sizeof(cp_stats));

    res = s3d_scene_view_closest_point_batch_ctx(
      scn->s3d_view, wf->cp_batch_ctx,
      wf->cp_requests, n,
      wf->cp_hits, &cp_stats);
    if(res != RES_OK) return res;

    /* Distribute results to paths, advancing phase */
    for(i = 0; i < n; i++) {
      uint32_t pid = wf->cp_to_path[i];
      struct path_state* p = &wf->paths[pid];
      p->locals.cnd_wos.cached_hit = wf->cp_hits[i];
      if(p->phase == PATH_CND_WOS_CLOSEST)
        p->phase = PATH_CND_WOS_CLOSEST_RESULT;
      else if(p->phase == PATH_CND_WOS_DIFFUSION_CHECK)
        p->phase = PATH_CND_WOS_DIFFUSION_CHECK_RESULT;
    }

    /* Cascade non-ray steps for paths that got CP results */
    for(i = 0; i < n; i++) {
      uint32_t pid = wf->cp_to_path[i];
      struct path_state* p = &wf->paths[pid];
      while(p->active && !p->needs_ray
          && p->phase != PATH_DONE && p->phase != PATH_ERROR) {
        int advanced = 0;
        if(path_phase_is_ray_pending(p->phase)) break;
        if(path_phase_is_enc_locate_pending(p->phase)) break;
        if(path_phase_is_cp_pending(p->phase)) break;
        res = advance_one_step_no_ray(p, scn, &advanced,
                                      wf->cold_pool, (size_t)pid);
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
        if(p->phase == PATH_DONE && wf->sfn_arr[pid].depth > 0) {
          p->phase = PATH_BND_SFN_COMPUTE_Ti_RESUME;
          p->active = 1;
        }
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

  /* Create batch enc_locate context (M10) */
  res = s3d_batch_enc_context_create(&wf.enc_batch_ctx, wf.max_enc_locates);
  if(res != RES_OK) goto cleanup;

  /* Create batch closest_point context (M9: WoS) */
  res = s3d_batch_cp_context_create(&wf.cp_batch_ctx, wf.max_cps);
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
        if(path_phase_is_enc_locate_pending(p->phase)) break;
        if(path_phase_is_cp_pending(p->phase)) break;

        res = advance_one_step_no_ray(p, scn, &advanced,
                                      wf.cold_pool, i);
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
        /* M8: intercept PATH_DONE when sfn_stack_depth > 0 */
        if(p->phase == PATH_DONE && wf.sfn_arr[i].depth > 0) {
          p->phase = PATH_BND_SFN_COMPUTE_Ti_RESUME;
          p->active = 1;
        }
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

    /* Step C2 (M10): Batch enc_locate for paths in PATH_ENC_LOCATE_PENDING */
    {
      size_t i, n = 0;
      /* Collect enc_locate requests */
      for(i = 0; i < wf.total_paths; i++) {
        struct path_state* p = &wf.paths[i];
        if(p->active && p->phase == PATH_ENC_LOCATE_PENDING) {
          struct s3d_enc_locate_request* req = &wf.enc_locate_requests[n];
          req->pos[0] = (float)wf.enc_arr[i].locate.query_pos[0];
          req->pos[1] = (float)wf.enc_arr[i].locate.query_pos[1];
          req->pos[2] = (float)wf.enc_arr[i].locate.query_pos[2];
          req->user_id = (uint32_t)i;
          wf.enc_locate_to_path[n] = (uint32_t)i;
          wf.enc_arr[i].locate.batch_idx = (uint32_t)n;
          n++;
        }
      }
      wf.enc_locate_count = n;

      if(n > 0) {
        struct s3d_batch_enc_stats enc_stats;
        memset(&enc_stats, 0, sizeof(enc_stats));

        res = s3d_scene_view_find_enclosure_batch_ctx(
          scn->s3d_view, wf.enc_batch_ctx,
          wf.enc_locate_requests, n,
          wf.enc_locate_results, &enc_stats);
        if(res != RES_OK) goto cleanup;

        /* Distribute prim_id + side to paths; enc_id resolution happens
         * in step_enc_locate_result() during cascade (handles degenerate). */
        for(i = 0; i < n; i++) {
          uint32_t pid = wf.enc_locate_to_path[i];
          struct path_state* p = &wf.paths[pid];
          struct s3d_enc_locate_result* r = &wf.enc_locate_results[i];

          wf.enc_arr[pid].locate.prim_id  = r->prim_id;
          wf.enc_arr[pid].locate.side     = r->side;
          wf.enc_arr[pid].locate.distance = r->distance;
          p->phase = PATH_ENC_LOCATE_RESULT;
        }

        /* Cascade non-ray steps for paths that just got enc_locate results */
        for(i = 0; i < n; i++) {
          uint32_t pid = wf.enc_locate_to_path[i];
          struct path_state* p = &wf.paths[pid];
          while(p->active && !p->needs_ray
              && p->phase != PATH_DONE && p->phase != PATH_ERROR) {
            int advanced = 0;
            if(path_phase_is_ray_pending(p->phase)) break;
            if(path_phase_is_enc_locate_pending(p->phase)) break;
            if(path_phase_is_cp_pending(p->phase)) break;
            res = advance_one_step_no_ray(p, scn, &advanced,
                                          wf.cold_pool, (size_t)pid);
            if(res != RES_OK && res != RES_BAD_OP
            && res != RES_BAD_OP_IRRECOVERABLE) goto cleanup;
            if(res == RES_BAD_OP || res == RES_BAD_OP_IRRECOVERABLE) {
              p->phase = PATH_DONE;
              p->active = 0;
              p->done_reason = -1;
              wf.paths_failed++;
              res = RES_OK;
              break;
            }
            if(!advanced) break;
            if(p->phase == PATH_DONE && wf.sfn_arr[pid].depth > 0) {
              p->phase = PATH_BND_SFN_COMPUTE_Ti_RESUME;
              p->active = 1;
            }
            p->steps_taken++;
          }
        }
      }
    }

    /* Step C3 (M9): Batch closest_point for all CP-pending WoS paths */
    res = wf_batch_closest_point(&wf, scn);
    if(res != RES_OK) goto cleanup;

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

/*******************************************************************************
 * Probe-mode wavefront: init_paths_from_probe
 *
 * Initialise all paths from a single spatial position (probe mode).
 * Each path corresponds to one MC realisation at the given position.
 * No camera ray is involved — rwalk.vtx.P is set directly from `position`.
 ******************************************************************************/
static res_T
init_paths_from_probe(
  struct wavefront_context* wf,
  struct sdis_scene*        scn,
  struct ssp_rng*           base_rng,
  const unsigned            enc_id,
  const double              position[3],
  const double              time_range[2],
  const size_t              nrealisations,
  const size_t              picard_order,
  const enum sdis_diffusion_algorithm diff_algo)
{
  size_t i;
  const struct enclosure* enc = NULL;
  struct sdis_medium* mdm = NULL;
  enum sdis_medium_type mdm_type;
  res_T rr;
  ASSERT(wf && scn && base_rng && position && time_range);

  /* Resolve medium type from enclosure (SOLID or FLUID) */
  enc = scene_get_enclosure(scn, enc_id);
  rr = scene_get_enclosure_medium(scn, enc, &mdm);
  if(rr != RES_OK) return rr;
  mdm_type = sdis_medium_get_type(mdm);

  for(i = 0; i < nrealisations; i++) {
    struct path_state* p = &wf->paths[i];

    /* Identity */
    p->path_id = (uint32_t)i;
    p->pixel_x = 0;   /* probe mode: virtual pixel (0,0) */
    p->pixel_y = 0;
    p->realisation_idx = (uint32_t)i;
    p->ipix_image[0] = 0;
    p->ipix_image[1] = 0;

    /* Share per-thread RNG */
    p->rng = base_rng;

    /* Sample time (same RNG call order as depth-first solve_one_probe) */
    {
      double time = sample_time(p->rng, time_range);

      /* Directly set probe position — no camera_ray */
      p->rwalk = RWALK_NULL;
      d3_set(p->rwalk.vtx.P, position);
      p->rwalk.vtx.time = time;
      p->rwalk.hit_3d = S3D_HIT_NULL;
      p->rwalk.hit_side = SDIS_SIDE_NULL__;
      p->rwalk.enc_id = enc_id;
    }

    /* Initialise rwalk_context (matches init_all_paths exactly) */
    p->ctx = RWALK_CONTEXT_NULL;
    p->ctx.heat_path   = NULL;
    p->ctx.green_path  = NULL;
    p->ctx.Tmin   = scn->tmin;
    p->ctx.Tmin2  = scn->tmin * scn->tmin;
    p->ctx.Tmin3  = scn->tmin * scn->tmin * scn->tmin;
    p->ctx.That   = scn->tmax;
    p->ctx.That2  = scn->tmax * scn->tmax;
    p->ctx.That3  = scn->tmax * scn->tmax * scn->tmax;
    p->ctx.max_branchings = picard_order - 1;
    /* Probe paths start directly at COUPLED_CONDUCTIVE/CONVECTIVE,
     * bypassing step_boundary's coupled_nbranchings sentinel logic.
     * In depth-first, sample_coupled_path does nbranchings = SIZE_MAX + 1 = 0
     * at entry.  We replicate that here. */
    p->ctx.nbranchings    = 0;
    p->ctx.irealisation   = i;
    p->ctx.diff_algo      = diff_algo;

    /* Temperature accumulator — set T.func based on medium type.
     * This mirrors probe_realisation_Xd.h: SOLID -> conductive_path,
     * FLUID -> convective_path. */
    p->T = TEMPERATURE_NULL;
    if(mdm_type == SDIS_SOLID) {
      p->T.func = conductive_path_3d;
    } else {
      p->T.func = convective_path_3d;
    }

    /* Probe has no camera ray direction — zero the radiative state */
    memset(p->rad_direction, 0, sizeof(p->rad_direction));
    p->rad_bounce_count = 0;
    p->rad_retry_count  = 0;

    /* Lifecycle — start at conductive/convective state, NOT PATH_INIT
     * (PATH_INIT always emits a radiative trace which is wrong for
     * probe mode where the path starts inside the volume). */
    if(mdm_type == SDIS_SOLID) {
      p->phase = PATH_COUPLED_CONDUCTIVE;
    } else {
      p->phase = PATH_COUPLED_CONVECTIVE;
    }
    p->active  = 1;
    p->needs_ray = 0;

    /* Zero scratch areas (same as init_all_paths) */
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
    /* Probe mode: nbranchings already set to 0 above, so sync the
     * coupled_nbranchings to 0 (not -1 sentinel) to match. */
    p->coupled_nbranchings = 0;
    p->steps_taken = 0;
    p->done_reason = 0;
  }

  wf->active_count = nrealisations;
  return RES_OK;
}

/*******************************************************************************
 * Probe-mode wavefront: collect_results_probe
 *
 * Aggregate completed paths into a temperature accumulator (sum/sum2/count).
 * This replaces collect_results() which writes to tile pixels.
 ******************************************************************************/
static res_T
collect_results_probe(
  struct wavefront_context* wf,
  struct accum*             acc_temp)
{
  size_t i;
  ASSERT(wf && acc_temp);

  *acc_temp = ACCUM_NULL;

  for(i = 0; i < wf->total_paths; i++) {
    const struct path_state* p = &wf->paths[i];
    ASSERT(!p->active); /* all paths should be done */

    if(p->T.done) {
      acc_temp->sum  += p->T.value;
      acc_temp->sum2 += p->T.value * p->T.value;
      acc_temp->count += 1;
    }

    /* Classify path termination for statistics */
    switch(p->done_reason) {
    case 1:  wf->paths_done_radiative++;   break;
    case 2:  wf->paths_done_temperature++; break;
    case 3:  wf->paths_done_boundary++;    break;
    case 4:  wf->paths_done_temperature++; break;
    case -1: wf->paths_failed++;           break;
    default: break;
    }
  }

  return RES_OK;
}

/*******************************************************************************
 * Probe-mode wavefront: solve_wavefront_probe
 *
 * Wavefront main loop for probe mode.  Identical to solve_tile_wavefront
 * except:
 *   - paths are initialised from a single position (init_paths_from_probe)
 *   - results are collected into an accumulator (collect_results_probe)
 *   - no camera, no tile, no pixel grid
 ******************************************************************************/
LOCAL_SYM res_T
solve_wavefront_probe(
  struct sdis_scene*                   scn,
  struct ssp_rng*                      base_rng,
  const unsigned                       enc_id,
  const double                         position[3],
  const double                         time_range[2],
  const size_t                         nrealisations,
  const size_t                         picard_order,
  const enum sdis_diffusion_algorithm  diff_algo,
  struct accum*                        out_acc_temp)
{
  struct wavefront_context wf;
  const size_t total_paths = nrealisations;
  res_T res = RES_OK;
  struct time t_start, t_end, t_elapsed;
#ifdef SDIS_WF_DIAG
  FILE* diag = fopen("wf_probe_diag.log", "w");
#else
  FILE* diag = NULL;
#endif

  ASSERT(scn && base_rng && position && time_range && nrealisations > 0);
  ASSERT(out_acc_temp);

  /* ====== Allocate wavefront context ====== */
  res = wf_context_create(&wf, total_paths);
  if(res != RES_OK) goto cleanup;

  /* Create batch trace context (pre-allocated GPU buffers) */
  res = s3d_batch_trace_context_create(&wf.batch_ctx, wf.max_rays);
  if(res != RES_OK) goto cleanup;

  /* Create batch enc_locate context (M10) */
  res = s3d_batch_enc_context_create(&wf.enc_batch_ctx, wf.max_enc_locates);
  if(res != RES_OK) goto cleanup;

  /* Create batch closest_point context (M9: WoS) */
  res = s3d_batch_cp_context_create(&wf.cp_batch_ctx, wf.max_cps);
  if(res != RES_OK) goto cleanup;

  /* ====== Initialise all paths from probe position ====== */
  if(diag) { fprintf(diag, "[WF-DIAG] probe: entering init_paths_from_probe "
    "nreals=%lu enc_id=%u diff_algo=%d\n",
    (unsigned long)nrealisations, enc_id, (int)diff_algo); fflush(diag); }
  res = init_paths_from_probe(&wf, scn, base_rng, enc_id,
                              position, time_range, nrealisations,
                              picard_order, diff_algo);
  if(res != RES_OK) goto cleanup;
  if(diag) { fprintf(diag, "[WF-DIAG] probe: init_paths_from_probe OK\n"); fflush(diag); }

  /* ====== Initial step: advance all paths from PATH_INIT ====== */
  {
    size_t i;
    size_t init_ph[PATH_PHASE_COUNT];
    memset(init_ph, 0, sizeof(init_ph));
    for(i = 0; i < wf.total_paths; i++) {
      int advanced = 0;
      struct path_state* p = &wf.paths[i];

      while(p->active && !p->needs_ray
          && p->phase != PATH_DONE && p->phase != PATH_ERROR) {
        if(path_phase_is_ray_pending(p->phase)) break;
        if(path_phase_is_enc_locate_pending(p->phase)) break;
        if(path_phase_is_cp_pending(p->phase)) break;

        res = advance_one_step_no_ray(p, scn, &advanced,
                                      wf.cold_pool, i);
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
        if(p->phase == PATH_DONE && wf.sfn_arr[i].depth > 0) {
          p->phase = PATH_BND_SFN_COMPUTE_Ti_RESUME;
          p->active = 1;
        }
        p->steps_taken++;
      }
      if((int)p->phase >= 0 && (int)p->phase < PATH_PHASE_COUNT)
        init_ph[(int)p->phase]++;
    }
    if(diag) {
      fprintf(diag, "[WF-DIAG] probe: initial step done. Phase histogram:\n");
      for(i = 0; i < PATH_PHASE_COUNT; i++) {
        if(init_ph[i] > 0)
          fprintf(diag, "  phase[%lu]=%lu\n",
            (unsigned long)i, (unsigned long)init_ph[i]);
      }
      fflush(diag);
    }
  }

  /* ====== Wavefront main loop ====== */
  time_current(&t_start);
  while(wf.active_count > 0) {
    wf.total_steps++;

    /* ---- Diagnostic: phase histogram (first 20 steps + every 500) ---- */
    if(diag && (wf.total_steps <= 20
    || (wf.total_steps % 500) == 0
    || wf.total_steps == total_paths * 1000)) {
      size_t ph_counts[PATH_PHASE_COUNT];
      size_t di, n_needs_ray = 0, n_active = 0, n_done = 0;
      memset(ph_counts, 0, sizeof(ph_counts));
      for(di = 0; di < wf.total_paths; di++) {
        const struct path_state* dp = &wf.paths[di];
        if(dp->active) {
          n_active++;
          if((int)dp->phase >= 0 && (int)dp->phase < PATH_PHASE_COUNT)
            ph_counts[(int)dp->phase]++;
          if(dp->needs_ray) n_needs_ray++;
        }
        if(dp->phase == PATH_DONE) n_done++;
      }
      fprintf(diag,
        "[WF-DIAG] step=%lu active=%lu done=%lu needs_ray=%lu rays=%lu "
        "cp_count=%lu enc_count=%lu\n",
        (unsigned long)wf.total_steps, (unsigned long)n_active,
        (unsigned long)n_done, (unsigned long)n_needs_ray,
        (unsigned long)wf.ray_count,
        (unsigned long)wf.cp_count, (unsigned long)wf.enc_locate_count);
      for(di = 0; di < PATH_PHASE_COUNT; di++) {
        if(ph_counts[di] > 0)
          fprintf(diag, "  phase[%lu]=%lu\n",
            (unsigned long)di, (unsigned long)ph_counts[di]);
      }
      /* Extra: log first active path's WoS state for first 20 steps */
      if(wf.total_steps <= 20) {
        for(di = 0; di < wf.total_paths; di++) {
          const struct path_state* dp = &wf.paths[di];
          if(dp->active) {
            fprintf(diag, "  path[%lu]: phase=%d pos=(%g,%g,%g) "
              "wos_dist=%g delta=%g steps=%lu\n",
              (unsigned long)di, (int)dp->phase,
              dp->rwalk.vtx.P[0], dp->rwalk.vtx.P[1], dp->rwalk.vtx.P[2],
              dp->locals.cnd_wos.last_distance,
              dp->locals.cnd_wos.delta,
              (unsigned long)dp->steps_taken);
            break; /* only first active path */
          }
        }
      }
      fflush(diag);
    }

    /* Step A: Collect ray requests from all active paths */
    wf.ray_count = 0;
    res = collect_ray_requests(&wf);
    if(res != RES_OK) goto cleanup;

    /* Step B: Batch trace */
    if(wf.ray_count > 0) {
      struct s3d_batch_trace_stats stats;
      memset(&stats, 0, sizeof(stats));

      res = s3d_scene_view_trace_rays_batch_ctx(
        scn->s3d_view,
        wf.batch_ctx,
        wf.ray_requests,
        wf.ray_count,
        wf.ray_hits,
        &stats);
      if(res != RES_OK) goto cleanup;

      wf.total_rays_traced += wf.ray_count;
    }

    /* Step C: Distribute results + advance paths */
    res = distribute_and_advance(&wf, scn);
    if(res != RES_OK) goto cleanup;

    /* Step C2 (M10): Batch enc_locate */
    {
      size_t i, n = 0;
      for(i = 0; i < wf.total_paths; i++) {
        struct path_state* p = &wf.paths[i];
        if(p->active && p->phase == PATH_ENC_LOCATE_PENDING) {
          struct s3d_enc_locate_request* req = &wf.enc_locate_requests[n];
          req->pos[0] = (float)wf.enc_arr[i].locate.query_pos[0];
          req->pos[1] = (float)wf.enc_arr[i].locate.query_pos[1];
          req->pos[2] = (float)wf.enc_arr[i].locate.query_pos[2];
          req->user_id = (uint32_t)i;
          wf.enc_locate_to_path[n] = (uint32_t)i;
          wf.enc_arr[i].locate.batch_idx = (uint32_t)n;
          n++;
        }
      }
      wf.enc_locate_count = n;

      if(n > 0) {
        struct s3d_batch_enc_stats enc_stats;
        memset(&enc_stats, 0, sizeof(enc_stats));

        res = s3d_scene_view_find_enclosure_batch_ctx(
          scn->s3d_view, wf.enc_batch_ctx,
          wf.enc_locate_requests, n,
          wf.enc_locate_results, &enc_stats);
        if(res != RES_OK) goto cleanup;

        for(i = 0; i < n; i++) {
          uint32_t pid = wf.enc_locate_to_path[i];
          struct path_state* p = &wf.paths[pid];
          struct s3d_enc_locate_result* r = &wf.enc_locate_results[i];

          wf.enc_arr[pid].locate.prim_id  = r->prim_id;
          wf.enc_arr[pid].locate.side     = r->side;
          wf.enc_arr[pid].locate.distance = r->distance;
          p->phase = PATH_ENC_LOCATE_RESULT;
        }

        for(i = 0; i < n; i++) {
          uint32_t pid = wf.enc_locate_to_path[i];
          struct path_state* p = &wf.paths[pid];
          while(p->active && !p->needs_ray
              && p->phase != PATH_DONE && p->phase != PATH_ERROR) {
            int advanced = 0;
            if(path_phase_is_ray_pending(p->phase)) break;
            if(path_phase_is_enc_locate_pending(p->phase)) break;
            if(path_phase_is_cp_pending(p->phase)) break;
            res = advance_one_step_no_ray(p, scn, &advanced,
                                          wf.cold_pool, (size_t)pid);
            if(res != RES_OK && res != RES_BAD_OP
            && res != RES_BAD_OP_IRRECOVERABLE) goto cleanup;
            if(res == RES_BAD_OP || res == RES_BAD_OP_IRRECOVERABLE) {
              p->phase = PATH_DONE;
              p->active = 0;
              p->done_reason = -1;
              wf.paths_failed++;
              res = RES_OK;
              break;
            }
            if(!advanced) break;
            if(p->phase == PATH_DONE && wf.sfn_arr[pid].depth > 0) {
              p->phase = PATH_BND_SFN_COMPUTE_Ti_RESUME;
              p->active = 1;
            }
            p->steps_taken++;
          }
        }
      }
    }

    /* Step C3 (M9): Batch closest_point for all CP-pending WoS paths */
    res = wf_batch_closest_point(&wf, scn);
    if(res != RES_OK) goto cleanup;

    /* Step D: Update active count */
    update_active_count(&wf);

    /* Safety: prevent infinite loops */
    if(wf.total_steps > total_paths * 1000) {
      log_err(scn->dev,
        "wavefront probe: exceeded maximum step count (%lu). "
        "%lu paths still active.\n",
        (unsigned long)wf.total_steps,
        (unsigned long)wf.active_count);
      res = RES_BAD_OP;
      goto cleanup;
    }
  }

  /* ====== Collect results into accumulator ====== */
  res = collect_results_probe(&wf, out_acc_temp);

  /* ====== Summary logging ====== */
  time_current(&t_end);
  time_sub(&t_elapsed, &t_end, &t_start);
  {
    char time_str[128];
    time_dump(&t_elapsed, TIME_ALL, NULL, time_str, sizeof(time_str));
    log_info(scn->dev,
      "wavefront probe: nrealisations=%lu  "
      "elapsed=%s  steps=%lu  rays=%lu  "
      "(rad=%lu cond_ds=%lu ds_retry=%lu)  "
      "done: rad=%lu temp=%lu bnd=%lu fail=%lu  "
      "max_depth=%lu\n",
      (unsigned long)nrealisations,
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
  if(diag) { fprintf(diag, "[WF-DIAG] cleanup: res=%d total_steps=%lu active=%lu\n",
    (int)res, (unsigned long)wf.total_steps, (unsigned long)wf.active_count);
    fclose(diag); diag = NULL; }
  wf_context_destroy(&wf);
  return res;
}

/*******************************************************************************
 * Public API: sdis_solve_wavefront_probe
 *
 * Public entry point matching sdis_solve_probe semantics exactly.
 * Delegates to solve_wavefront_probe() after scene validation, enclosure
 * lookup, RNG creation, and estimator setup.
 ******************************************************************************/
res_T
sdis_solve_wavefront_probe(
  struct sdis_scene*                   scn,
  const struct sdis_solve_probe_args*  args,
  struct sdis_estimator**              out_estimator)
{
  struct ssp_rng* rng = NULL;
  struct sdis_estimator* estimator = NULL;
  unsigned enc_id = ENCLOSURE_ID_NULL;
  struct accum acc_temp = ACCUM_NULL;
  res_T res = RES_OK;

  if(!scn || !args || !out_estimator) return RES_BAD_ARG;
  if(args->nrealisations == 0) return RES_BAD_ARG;

  *out_estimator = NULL;

  /* Retrieve enclosure at probe position (one-time brute-force lookup,
   * same as CPU solve_probe entry).  The authoritative enc_id for the
   * conductive path is established later by the M1 6-ray enc_query
   * (step_enc_query_emit/resolve) during the first wavefront iteration,
   * which mirrors CPU's scene_get_enclosure_id_in_closed_boundaries. */
  res = scene_get_enclosure_id(scn, args->position, &enc_id);
  if(res != RES_OK) goto error;

  /* Create RNG */
  if(args->rng_state) {
    rng = args->rng_state;
  } else {
    res = ssp_rng_create(NULL, args->rng_type, &rng);
    if(res != RES_OK) goto error;
  }

  /* Create estimator */
  res = estimator_create(scn->dev, SDIS_ESTIMATOR_TEMPERATURE, &estimator);
  if(res != RES_OK) goto error;

  /* Run wavefront probe solver */
  res = solve_wavefront_probe(
    scn, rng, enc_id,
    args->position, args->time_range,
    args->nrealisations,
    args->picard_order,
    args->diff_algo,
    &acc_temp);
  if(res != RES_OK) goto error;

  /* Setup estimator from accumulator */
  estimator_setup_realisations_count(estimator,
    args->nrealisations, acc_temp.count);
  estimator_setup_temperature(estimator, acc_temp.sum, acc_temp.sum2);

  *out_estimator = estimator;
  goto exit;

error:
  if(estimator) { sdis_estimator_ref_put(estimator); estimator = NULL; }
exit:
  if(rng && !args->rng_state) ssp_rng_ref_put(rng);
  return res;
}

/*******************************************************************************
 * Public API: sdis_solve_persistent_wavefront_probe
 *
 * Same as sdis_solve_wavefront_probe but uses the persistent pool main loop
 * (solve_persistent_wavefront_probe) shared with the camera solver.
 ******************************************************************************/
res_T
sdis_solve_persistent_wavefront_probe(
  struct sdis_scene*                   scn,
  const struct sdis_solve_probe_args*  args,
  struct sdis_estimator**              out_estimator)
{
  struct ssp_rng* rng = NULL;
  struct sdis_estimator* estimator = NULL;
  unsigned enc_id = ENCLOSURE_ID_NULL;
  struct accum acc_temp = ACCUM_NULL;
  res_T res = RES_OK;

  if(!scn || !args || !out_estimator) return RES_BAD_ARG;
  if(args->nrealisations == 0) return RES_BAD_ARG;

  *out_estimator = NULL;

  /* Retrieve enclosure at probe position */
  res = scene_get_enclosure_id(scn, args->position, &enc_id);
  if(res != RES_OK) goto error_p;

  /* Create RNG */
  if(args->rng_state) {
    rng = args->rng_state;
  } else {
    res = ssp_rng_create(NULL, args->rng_type, &rng);
    if(res != RES_OK) goto error_p;
  }

  /* Create estimator */
  res = estimator_create(scn->dev, SDIS_ESTIMATOR_TEMPERATURE, &estimator);
  if(res != RES_OK) goto error_p;

  /* Run persistent wavefront probe solver */
  res = solve_persistent_wavefront_probe(
    scn, rng, enc_id,
    args->position, args->time_range,
    args->nrealisations,
    args->picard_order,
    args->diff_algo,
    &acc_temp);
  if(res != RES_OK) goto error_p;

  /* Setup estimator from accumulator */
  estimator_setup_realisations_count(estimator,
    args->nrealisations, acc_temp.count);
  estimator_setup_temperature(estimator, acc_temp.sum, acc_temp.sum2);

  *out_estimator = estimator;
  goto exit_p;

error_p:
  if(estimator) { sdis_estimator_ref_put(estimator); estimator = NULL; }
exit_p:
  if(rng && !args->rng_state) ssp_rng_ref_put(rng);
  return res;
}

/*******************************************************************************
 * Public API: sdis_solve_persistent_wavefront_probe_batch
 *
 * Multi-probe variant — runs K probes in a single persistent pool pass.
 * Each probe gets its own estimator with independent sum/sum2/count.
 ******************************************************************************/
res_T
sdis_solve_persistent_wavefront_probe_batch(
  struct sdis_scene*                   scn,
  const size_t                         nprobes,
  const struct sdis_solve_probe_args*  args_array,
  struct sdis_estimator**              out_estimators)
{
  struct ssp_rng* rng = NULL;
  unsigned* enc_ids = NULL;
  double* positions = NULL;
  enum sdis_medium_type* mdm_types = NULL;
  struct accum* accums = NULL;
  size_t* valid_map = NULL; /* compact index -> original index */
  size_t nvalid = 0;
  res_T res = RES_OK;
  size_t i;

  if(!scn || !args_array || !out_estimators || nprobes == 0)
    return RES_BAD_ARG;

  for(i = 0; i < nprobes; i++)
    out_estimators[i] = NULL;

  /* ---- Allocate per-probe arrays ---- */
  enc_ids   = (unsigned*)malloc(nprobes * sizeof(unsigned));
  positions = (double*)malloc(nprobes * 3 * sizeof(double));
  mdm_types = (enum sdis_medium_type*)malloc(
    nprobes * sizeof(enum sdis_medium_type));
  accums    = (struct accum*)malloc(nprobes * sizeof(struct accum));
  valid_map = (size_t*)malloc(nprobes * sizeof(size_t));

  if(!enc_ids || !positions || !mdm_types || !accums || !valid_map) {
    res = RES_MEM_ERR;
    goto error_batch;
  }

  /* ---- Resolve per-probe enclosures + medium types ---- */
  /* Probes that fail enclosure/medium resolution are silently skipped
   * (estimator stays NULL), matching the serial API where each probe may
   * independently return RES_BAD_OP for fluid/multi-material enclosures.
   * Only truly fatal errors (bad args, OOM) abort the whole batch. */
  for(i = 0; i < nprobes; i++) {
    const struct sdis_solve_probe_args* a = &args_array[i];
    res_T pres; /* per-probe result */
    if(a->nrealisations == 0) { res = RES_BAD_ARG; goto error_batch; }

    pres = scene_get_enclosure_id(scn, a->position, &enc_ids[nvalid]);
    if(pres != RES_OK) continue; /* skip — out_estimators[i] stays NULL */

    { const struct enclosure* enc = scene_get_enclosure(scn, enc_ids[nvalid]);
      struct sdis_medium* mdm = NULL;
      pres = scene_get_enclosure_medium(scn, enc, &mdm);
      if(pres != RES_OK) continue; /* skip — fluid/multi-material enclosure */
      mdm_types[nvalid] = sdis_medium_get_type(mdm);
    }

    positions[nvalid*3+0] = a->position[0];
    positions[nvalid*3+1] = a->position[1];
    positions[nvalid*3+2] = a->position[2];
    valid_map[nvalid] = i;
    nvalid++;
  }

  /* All probes skipped — nothing to solve, return OK with all NULL ests */
  if(nvalid == 0) goto exit_batch;

  /* ---- Create RNG (from first args entry) ---- */
  if(args_array[0].rng_state) {
    rng = args_array[0].rng_state;
  } else {
    res = ssp_rng_create(NULL, args_array[0].rng_type, &rng);
    if(res != RES_OK) goto error_batch;
  }

  /* ---- Create estimators for valid probes only ---- */
  for(i = 0; i < nvalid; i++) {
    res = estimator_create(scn->dev, SDIS_ESTIMATOR_TEMPERATURE,
                           &out_estimators[valid_map[i]]);
    if(res != RES_OK) goto error_batch;
  }

  /* ---- Run batch solver (compact arrays, nvalid probes) ---- */
  res = solve_persistent_wavefront_probe_batch(
    scn, rng, nvalid, enc_ids, positions,
    args_array[0].time_range,
    args_array[0].nrealisations,
    args_array[0].picard_order,
    args_array[0].diff_algo,
    mdm_types, accums);
  if(res != RES_OK) goto error_batch;

  /* ---- Setup estimators from accumulators ---- */
  for(i = 0; i < nvalid; i++) {
    size_t orig = valid_map[i];
    estimator_setup_realisations_count(out_estimators[orig],
      args_array[orig].nrealisations, accums[i].count);
    estimator_setup_temperature(out_estimators[orig],
      accums[i].sum, accums[i].sum2);
  }

  goto exit_batch;

error_batch:
  for(i = 0; i < nprobes; i++) {
    if(out_estimators[i]) {
      sdis_estimator_ref_put(out_estimators[i]);
      out_estimators[i] = NULL;
    }
  }
exit_batch:
  free(enc_ids);
  free(positions);
  free(mdm_types);
  free(accums);
  free(valid_map);
  if(rng && !args_array[0].rng_state) ssp_rng_ref_put(rng);
  return res;
}

#endif  /* !SDIS_P0_OPT */