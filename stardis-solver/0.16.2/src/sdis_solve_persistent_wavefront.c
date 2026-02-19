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

/* Phase B-3 M3: Persistent Wavefront Pool with refill, stream
 * compaction, path-type bucketed dispatch, adaptive pool sizing,
 * and enhanced diagnostics.
 *
 * Builds on M2+M2.5:
 *   M2:   Fixed-size pool with refill (wavefront width constant).
 *   M2.5: Stream compaction + bucket index arrays.
 *   M3:   Bucketed step dispatch (per-type loops for radiative /
 *         conductive), adaptive pool_size based on GPU SM count,
 *         per-phase wall-clock timing, average wavefront width tracking.
 *
 * Reference: guide/upper-parallelization/phase_b3_persistent_wavefront.md
 */

#include "sdis_solve_persistent_wavefront.h"
#include "sdis_wf_rng.h"
#include "sdis_wf_rng_adapter.h"
#include "sdis.h"
#include "sdis_brdf.h"
#include "sdis_c.h"
#include "sdis_camera.h"
#include "sdis_device_c.h"
#include "sdis_estimator_buffer_c.h"
#include "sdis_estimator_c.h"
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
#include <rsys/cstr.h>
#include <rsys/morton.h>

#include <omp.h>
#include <float.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>  /* pixel trace */

/* Per-path temperature trace (set STARDIS_PIXEL_TRACE=gpu_trace.csv) */
static FILE* s_pixel_trace_fp = NULL;
static int   s_pixel_trace_checked = 0;
static FILE* pixel_trace_file(void) {
  if(!s_pixel_trace_checked) {
    const char* path = getenv("STARDIS_PIXEL_TRACE");
    if(path && path[0]) s_pixel_trace_fp = fopen(path, "w");
    if(s_pixel_trace_fp)
      fprintf(s_pixel_trace_fp,
        "path_id,px,py,spp,T_value,T_done,done_reason,steps,phase\n");
    s_pixel_trace_checked = 1;
  }
  return s_pixel_trace_fp;
}

/*******************************************************************************
 * Step functions are now in sdis_wf_steps.c with LOCAL_SYM linkage.
 * Include the header to access them.
 ******************************************************************************/
#include "sdis_wf_steps.h"

/*******************************************************************************
 * Pool allocation / deallocation
 ******************************************************************************/
static res_T
pool_create(struct wavefront_pool* pool, size_t pool_size)
{
  ASSERT(pool && pool_size > 0);
  memset(pool, 0, sizeof(*pool));
  pool->pool_size = pool_size;

  pool->slots = (struct path_state*)calloc(pool_size, sizeof(struct path_state));
  if(!pool->slots) return RES_MEM_ERR;

  /* Each path can request up to 6 rays per step (B-4 M1: ENC query) */
  pool->max_rays = pool_size * 6;
  pool->ray_requests = (struct s3d_ray_request*)malloc(
    pool->max_rays * sizeof(struct s3d_ray_request));
  pool->ray_to_slot = (uint32_t*)malloc(pool->max_rays * sizeof(uint32_t));
  pool->ray_slot_sub = (uint32_t*)malloc(pool->max_rays * sizeof(uint32_t));
  pool->ray_hits = (struct s3d_hit*)malloc(pool->max_rays * sizeof(struct s3d_hit));

  if(!pool->ray_requests || !pool->ray_to_slot
  || !pool->ray_slot_sub || !pool->ray_hits) {
    return RES_MEM_ERR;
  }

  /* Per-slot RNG array (pointers only; assigned later) */
  pool->slot_rngs = (struct ssp_rng**)calloc(pool_size, sizeof(struct ssp_rng*));
  if(!pool->slot_rngs) return RES_MEM_ERR;

  /* Stream compaction index arrays (M2.5) */
  pool->active_indices   = (uint32_t*)malloc(pool_size * sizeof(uint32_t));
  pool->need_ray_indices = (uint32_t*)malloc(pool_size * sizeof(uint32_t));
  pool->done_indices     = (uint32_t*)malloc(pool_size * sizeof(uint32_t));

  /* Path type buckets (M2.5) */
  pool->bucket_radiative  = (uint32_t*)malloc(pool_size * sizeof(uint32_t));
  pool->bucket_conductive = (uint32_t*)malloc(pool_size * sizeof(uint32_t));

  if(!pool->active_indices || !pool->need_ray_indices || !pool->done_indices
  || !pool->bucket_radiative || !pool->bucket_conductive) {
    return RES_MEM_ERR;
  }

  /* B-4 M10: enc_locate batch buffers */
  pool->max_enc_locates = pool_size;
  pool->enc_locate_requests = (struct s3d_enc_locate_request*)malloc(
    pool_size * sizeof(struct s3d_enc_locate_request));
  pool->enc_locate_results = (struct s3d_enc_locate_result*)malloc(
    pool_size * sizeof(struct s3d_enc_locate_result));
  pool->enc_locate_to_slot = (uint32_t*)malloc(
    pool_size * sizeof(uint32_t));

  if(!pool->enc_locate_requests || !pool->enc_locate_results
  || !pool->enc_locate_to_slot) {
    return RES_MEM_ERR;
  }

  /* Initialise diagnostics */
  pool->diag_min_batch = (size_t)-1; /* updated on first step with rays */
  pool->trace_batch_size_min = (size_t)-1;

  return RES_OK;
}

static void
pool_destroy(struct wavefront_pool* pool)
{
  if(!pool) return;

  if(pool->batch_ctx) s3d_batch_trace_context_destroy(pool->batch_ctx);
  if(pool->enc_batch_ctx) s3d_batch_enc_context_destroy(pool->enc_batch_ctx);

  /* Release per-slot RNGs */
  if(pool->thin_rng_storage) {
    /* Per-path CBRNG mode: thin wrappers own no heap state, free array */
    wf_rng_destroy_thin_ssp_rngs(pool->thin_rng_storage);
    pool->thin_rng_storage = NULL;
  } else if(pool->slot_rngs) {
    /* Legacy round-robin mode: release ref-counted shared RNGs */
    size_t i;
    for(i = 0; i < pool->pool_size; i++) {
      if(pool->slot_rngs[i]) SSP(rng_ref_put(pool->slot_rngs[i]));
    }
  }
  free(pool->slot_rngs);

  free(pool->task_queue);
  free(pool->slots);
  free(pool->ray_requests);
  free(pool->ray_to_slot);
  free(pool->ray_slot_sub);
  free(pool->ray_hits);

  /* Stream compaction arrays */
  free(pool->active_indices);
  free(pool->need_ray_indices);
  free(pool->done_indices);

  /* Bucket arrays */
  free(pool->bucket_radiative);
  free(pool->bucket_conductive);

  /* B-4 M10: enc_locate arrays */
  free(pool->enc_locate_requests);
  free(pool->enc_locate_results);
  free(pool->enc_locate_to_slot);

  memset(pool, 0, sizeof(*pool));
}

/*******************************************************************************
 * Task queue generation
 *
 * Tasks are generated in the same traversal order as the original code:
 *   tile Morton order -> pixel Morton order within tile -> SPP
 ******************************************************************************/
static res_T
generate_task_queue(
  struct wavefront_pool* pool,
  const size_t image_def[2],
  const size_t spp)
{
  size_t ntiles_x, ntiles_y, ntiles_adjusted;
  size_t tile_mcode, pix_mcode;
  size_t task_idx = 0;
  size_t total_tasks = image_def[0] * image_def[1] * spp;

  ASSERT(pool && image_def && spp > 0);

  pool->task_queue = (struct pixel_task*)malloc(
    total_tasks * sizeof(struct pixel_task));
  if(!pool->task_queue) return RES_MEM_ERR;

  pool->task_count = total_tasks;
  pool->task_next = 0;

  /* Tile grid dimensions (same as sdis_solve_camera) */
  ntiles_x = (image_def[0] + (TILE_SIZE - 1)) / TILE_SIZE;
  ntiles_y = (image_def[1] + (TILE_SIZE - 1)) / TILE_SIZE;
  ntiles_adjusted = round_up_pow2(MMAX(ntiles_x, ntiles_y));
  ntiles_adjusted *= ntiles_adjusted;

  /* Iterate tiles in Morton order */
  for(tile_mcode = 0; tile_mcode < ntiles_adjusted; tile_mcode++) {
    size_t tile_x, tile_y;
    size_t tile_org[2], tile_sz[2];
    size_t npixels_morton;

    tile_x = morton2D_decode_u16((uint32_t)(tile_mcode >> 0));
    if(tile_x >= ntiles_x) continue;
    tile_y = morton2D_decode_u16((uint32_t)(tile_mcode >> 1));
    if(tile_y >= ntiles_y) continue;

    tile_org[0] = tile_x * TILE_SIZE;
    tile_org[1] = tile_y * TILE_SIZE;
    tile_sz[0] = MMIN(TILE_SIZE, image_def[0] - tile_org[0]);
    tile_sz[1] = MMIN(TILE_SIZE, image_def[1] - tile_org[1]);

    /* Iterate pixels within tile in Morton order */
    npixels_morton = round_up_pow2(MMAX(tile_sz[0], tile_sz[1]));
    npixels_morton *= npixels_morton;

    for(pix_mcode = 0; pix_mcode < npixels_morton; pix_mcode++) {
      uint16_t ipix_tile[2];
      size_t ipix_image[2];
      size_t irealisation;

      ipix_tile[0] = morton2D_decode_u16((uint32_t)(pix_mcode >> 0));
      if(ipix_tile[0] >= tile_sz[0]) continue;
      ipix_tile[1] = morton2D_decode_u16((uint32_t)(pix_mcode >> 1));
      if(ipix_tile[1] >= tile_sz[1]) continue;

      ipix_image[0] = ipix_tile[0] + tile_org[0];
      ipix_image[1] = ipix_tile[1] + tile_org[1];

      for(irealisation = 0; irealisation < spp; irealisation++) {
        struct pixel_task* t = &pool->task_queue[task_idx];
        t->ipix_image[0] = ipix_image[0];
        t->ipix_image[1] = ipix_image[1];
        t->spp_idx = (uint32_t)irealisation;
        task_idx++;
      }
    }
  }

  ASSERT(task_idx == total_tasks);
  return RES_OK;
}

/*******************************************************************************
 * Path initialisation -- initialise a single path from a pixel_task
 *
 * Mirrors init_all_paths from sdis_solve_wavefront.c but operates on a
 * single slot.
 ******************************************************************************/
static res_T
init_single_path(
  struct path_state* p,
  struct ssp_rng* rng,
  const struct pixel_task* task,
  struct sdis_scene* scn,
  const unsigned enc_id,
  const struct sdis_camera* cam,
  const double time_range[2],
  const double pix_sz[2],
  const size_t picard_order,
  const enum sdis_diffusion_algorithm diff_algo,
  uint32_t path_id,
  uint64_t global_seed)
{
  double samp[2];
  double ray_pos[3], ray_dir[3];
  double time;

  ASSERT(p && rng && task && scn && cam && pix_sz && time_range);

  memset(p, 0, sizeof(*p));

  /* Identity */
  p->path_id = path_id;
  p->pixel_x = (uint16_t)task->ipix_image[0]; /* absolute image coords */
  p->pixel_y = (uint16_t)task->ipix_image[1];
  p->realisation_idx = task->spp_idx;
  p->ipix_image[0] = task->ipix_image[0];
  p->ipix_image[1] = task->ipix_image[1];

  /* Use per-slot RNG (thin wrapper pointing to p->rng_state) */
  p->rng = rng;

  /* Seed per-path CBRNG with (pixel_x, pixel_y, spp_idx, global_seed).
   * The thin ssp_rng wrapper's state already points to &p->rng_state,
   * so all ssp_rng_canonical(p->rng) calls will use this seeded state. */
  wf_rng_seed(&p->rng_state,
              (uint32_t)task->ipix_image[0],
              (uint32_t)task->ipix_image[1],
              task->spp_idx,
              global_seed);

  /* Sample time (same RNG call order as solve_pixel) */
  time = sample_time(p->rng, time_range);

  /* Pixel sub-sample */
  samp[0] = ((double)task->ipix_image[0]
            + ssp_rng_canonical(p->rng)) * pix_sz[0];
  samp[1] = ((double)task->ipix_image[1]
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

  /* Initialise rwalk_context */
  p->ctx = RWALK_CONTEXT_NULL;
  p->ctx.heat_path   = NULL;
  p->ctx.green_path  = NULL;
  p->ctx.Tmin  = scn->tmin;
  p->ctx.Tmin2 = scn->tmin * scn->tmin;
  p->ctx.Tmin3 = scn->tmin * scn->tmin * scn->tmin;
  p->ctx.That  = scn->tmax;
  p->ctx.That2 = scn->tmax * scn->tmax;
  p->ctx.That3 = scn->tmax * scn->tmax * scn->tmax;
  p->ctx.max_branchings = picard_order - 1;
  p->ctx.nbranchings    = SIZE_MAX;
  p->ctx.irealisation   = task->spp_idx;
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

  /* Zero scratch areas */
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
  p->coupled_nbranchings = -1;
  p->steps_taken = 0;
  p->done_reason = 0;

  return RES_OK;
}

/*******************************************************************************
 * Advance a single path past PATH_INIT to its first ray-needed phase (or
 * PATH_DONE).  Shared by fill_pool() and refill_pool().
 ******************************************************************************/
static res_T
advance_path_to_first_ray(struct path_state* p,
                          struct sdis_scene* scn,
                          struct wavefront_pool* pool)
{
  res_T res = RES_OK;
  while(p->active && !p->needs_ray && p->phase != PATH_DONE
      && p->phase != PATH_ERROR) {
    int advanced = 0;
    if(path_phase_is_ray_pending(p->phase)) break;

    res = advance_one_step_no_ray(p, scn, &advanced);
    if(res != RES_OK && res != RES_BAD_OP
    && res != RES_BAD_OP_IRRECOVERABLE) return res;
    if(res == RES_BAD_OP || res == RES_BAD_OP_IRRECOVERABLE) {
      p->phase = PATH_DONE;
      p->active = 0;
      p->done_reason = -1;
      pool->paths_failed++;
      res = RES_OK;
      break;
    }
    if(!advanced) break;
    /* M8: intercept PATH_DONE when sfn_stack_depth > 0 */
    if(p->phase == PATH_DONE && p->sfn_stack_depth > 0) {
      p->phase = PATH_BND_SFN_COMPUTE_Ti_RESUME;
      p->active = 1;
    }
    p->steps_taken++;
  }
  return RES_OK;
}

/*******************************************************************************
 * Fill pool -- load the initial batch of tasks
 ******************************************************************************/
static res_T
fill_pool(struct wavefront_pool* pool)
{
  size_t i;
  res_T res = RES_OK;

  ASSERT(pool && pool->task_next == 0);

  for(i = 0; i < pool->pool_size && pool->task_next < pool->task_count; i++) {
    const struct pixel_task* task = &pool->task_queue[pool->task_next];

    res = init_single_path(
      &pool->slots[i], pool->slot_rngs[i], task,
      pool->scn, pool->enc_id, pool->cam,
      pool->time_range, pool->pix_sz,
      pool->picard_order, pool->diff_algo,
      pool->next_path_id++, pool->global_seed);
    if(res != RES_OK) return res;

    /* Advance past PATH_INIT to first ray request */
    res = advance_path_to_first_ray(&pool->slots[i], pool->scn, pool);
    if(res != RES_OK) return res;

    pool->task_next++;
  }

  /* Count active paths */
  pool->active_count = 0;
  for(i = 0; i < pool->pool_size; i++) {
    if(pool->slots[i].active) pool->active_count++;
  }

  return RES_OK;
}

/*******************************************************************************
 * Stream Compaction (M2.5) -- rebuild compact index arrays each step.
 *
 * After this function:
 *   active_indices[0..active_compact-1]   = indices of active, non-done slots
 *   need_ray_indices[0..need_ray_count-1] = active slots needing ray trace
 *   done_indices[0..done_count-1]         = slots with PATH_DONE
 *   bucket_radiative[0..n-1]              = need_ray + radiative phase
 *   bucket_conductive[0..n-1]             = need_ray + conductive phase
 ******************************************************************************/
static void
compact_active_paths(struct wavefront_pool* pool)
{
  size_t i;
  pool->active_compact      = 0;
  pool->need_ray_count      = 0;
  pool->done_count          = 0;
  pool->bucket_radiative_n  = 0;
  pool->bucket_conductive_n = 0;

  for(i = 0; i < pool->pool_size; i++) {
    struct path_state* p = &pool->slots[i];

    if(p->phase == PATH_DONE || p->phase == PATH_ERROR
    || p->phase == PATH_HARVESTED) {
      /* M8: if sfn_stack_depth > 0, the sub-path finished but the
       * parent picardN frame still needs to resume.  Re-activate. */
      if(p->phase == PATH_DONE && p->sfn_stack_depth > 0) {
        p->phase = PATH_BND_SFN_COMPUTE_Ti_RESUME;
        p->active = 1;
        /* Fall through to treat as active path */
      } else {
        pool->done_indices[pool->done_count++] = (uint32_t)i;
        continue;
      }
    }
    if(!p->active) continue;

    pool->active_indices[pool->active_compact++] = (uint32_t)i;

    if(p->needs_ray && p->ray_req.ray_count > 0) {
      pool->need_ray_indices[pool->need_ray_count++] = (uint32_t)i;

      /* Bucket by phase type */
      if(p->phase == PATH_RAD_TRACE_PENDING) {
        pool->bucket_radiative[pool->bucket_radiative_n++] = (uint32_t)i;
      } else if(p->phase == PATH_COUPLED_COND_DS_PENDING
             || p->phase == PATH_CND_DS_STEP_TRACE) {
        pool->bucket_conductive[pool->bucket_conductive_n++] = (uint32_t)i;
      }
      /* Other ray-pending phases (boundary reinject) go into
       * need_ray_indices but not into type-specific buckets. */
    }
  }
}

/*******************************************************************************
 * Collect ray requests -- compact version (M2.5)
 *
 * Only iterates over need_ray_indices instead of the full pool.
 ******************************************************************************/
static res_T
pool_collect_ray_requests_compact(struct wavefront_pool* pool)
{
  size_t ray_idx = 0;
  size_t k;

  for(k = 0; k < pool->need_ray_count; k++) {
    uint32_t i = pool->need_ray_indices[k];
    struct path_state* p = &pool->slots[i];

    ASSERT(p->active && p->needs_ray && p->ray_req.ray_count >= 1);

    /* Detailed stats by phase type */
    if(p->phase == PATH_RAD_TRACE_PENDING)
      pool->rays_radiative += (size_t)p->ray_req.ray_count;
    else if(p->phase == PATH_COUPLED_COND_DS_PENDING
         || p->phase == PATH_CND_DS_STEP_TRACE) {
      if(p->ds_robust_attempt > 0)
        pool->rays_conductive_ds_retry += (size_t)p->ray_req.ray_count;
      else
        pool->rays_conductive_ds += (size_t)p->ray_req.ray_count;
    }

    /* Ray 0 */
    {
      struct s3d_ray_request* rr = &pool->ray_requests[ray_idx];
      rr->origin[0]    = p->ray_req.origin[0];
      rr->origin[1]    = p->ray_req.origin[1];
      rr->origin[2]    = p->ray_req.origin[2];
      rr->direction[0] = p->ray_req.direction[0];
      rr->direction[1] = p->ray_req.direction[1];
      rr->direction[2] = p->ray_req.direction[2];
      rr->range[0]     = p->ray_req.range[0];
      rr->range[1]     = p->ray_req.range[1];
      rr->user_id      = i;

      if(!S3D_HIT_NONE(&p->filter_data_storage.hit_3d)) {
        rr->filter_data = &p->filter_data_storage;
      } else {
        rr->filter_data = NULL;
      }

      pool->ray_to_slot[ray_idx] = i;
      pool->ray_slot_sub[ray_idx] = 0;
      p->ray_req.batch_idx = (uint32_t)ray_idx;
      ray_idx++;
    }

    /* Ray 1 (if 2-ray request: delta_sphere or reinjection) */
    if(p->ray_req.ray_count >= 2) {
      struct s3d_ray_request* rr = &pool->ray_requests[ray_idx];
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
      rr->user_id      = i;

      pool->ray_to_slot[ray_idx] = i;
      pool->ray_slot_sub[ray_idx] = 1;
      p->ray_req.batch_idx2 = (uint32_t)ray_idx;
      ray_idx++;
    }
  }

  pool->ray_count = ray_idx;
  return RES_OK;
}

/*******************************************************************************
 * Distribute batch trace results — bucketed dispatch (M3)
 *
 * Instead of a generic switch over p->phase via advance_one_step_with_ray,
 * iterate per-type buckets: all radiative paths first, then all conductive
 * paths.  This eliminates branch misprediction (~50% → ~99% hit rate) and
 * improves cache locality since each step function accesses a distinct
 * subset of scene data.
 *
 * Any ray-pending paths NOT in either bucket (e.g. future boundary reinject)
 * fall through to a generic fallback loop.
 ******************************************************************************/

/* Helper: mark a failed path as done */
static INLINE void
mark_path_failed(struct path_state* p, struct wavefront_pool* pool)
{
  p->phase = PATH_DONE;
  p->active = 0;
  p->done_reason = -1;
  pool->paths_failed++;
}

/* Helper: elapsed time in seconds between two struct time values (M3) */
static INLINE double
time_elapsed_sec(const struct time* t0, const struct time* t1)
{
  struct time dt;
  time_sub(&dt, t1, t0);
  return (double)dt.sec + (double)dt.nsec * 1e-9;
}

/*******************************************************************************
 * Bucketed ray request collection (B-4 M2)
 *
 * 2-pass radix scatter: sorts rays by ray_bucket_type so that same-type
 * rays occupy contiguous segments in the ray_requests[] array.  This
 * improves GPU warp coherence during BVH traversal since rays in the
 * same warp share similar traversal patterns.
 *
 * Pass 1: Count rays per bucket (iterate need_ray_indices once).
 * Pass 2: Compute bucket offsets, then scatter rays into final positions.
 *
 * The output is still a single contiguous ray_requests[] array.  Bucket
 * boundaries are recorded in pool->bucket_offsets[RAY_BUCKET_COUNT+1].
 ******************************************************************************/

/* count_path_rays is now static INLINE in persistent_wavefront.h */

LOCAL_SYM res_T
pool_collect_ray_requests_bucketed(struct wavefront_pool* pool)
{
  size_t k, b;
  /* Per-bucket write cursors */
  size_t cursor[RAY_BUCKET_COUNT];

  /* ---- Pass 1: Count rays per bucket ---- */
  memset(pool->bucket_counts, 0, sizeof(pool->bucket_counts));

  for(k = 0; k < pool->need_ray_count; k++) {
    uint32_t i = pool->need_ray_indices[k];
    struct path_state* p = &pool->slots[i];
    size_t nrays = count_path_rays(p);
    int bkt = (int)p->ray_bucket;

    ASSERT(bkt >= 0 && bkt < RAY_BUCKET_COUNT);
    pool->bucket_counts[bkt] += nrays;
  }

  /* ---- Compute bucket offsets (prefix sum) ---- */
  pool->bucket_offsets[0] = 0;
  for(b = 0; b < RAY_BUCKET_COUNT; b++) {
    pool->bucket_offsets[b + 1] = pool->bucket_offsets[b]
                                + pool->bucket_counts[b];
  }

  /* Initialise write cursors to bucket starts */
  for(b = 0; b < RAY_BUCKET_COUNT; b++) {
    cursor[b] = pool->bucket_offsets[b];
  }

  /* ---- Pass 2: Scatter rays into bucketed positions ---- */
  for(k = 0; k < pool->need_ray_count; k++) {
    uint32_t i = pool->need_ray_indices[k];
    struct path_state* p = &pool->slots[i];
    int bkt = (int)p->ray_bucket;

    /* Detailed stats by phase type */
    if(p->phase == PATH_RAD_TRACE_PENDING)
      pool->rays_radiative += (size_t)p->ray_req.ray_count;
    else if(p->phase == PATH_COUPLED_COND_DS_PENDING
         || p->phase == PATH_CND_DS_STEP_TRACE) {
      if(p->ds_robust_attempt > 0)
        pool->rays_conductive_ds_retry += (size_t)p->ray_req.ray_count;
      else
        pool->rays_conductive_ds += (size_t)p->ray_req.ray_count;
    }
    /* B-4 M2: additional per-bucket stats */
    if(bkt == RAY_BUCKET_SHADOW)
      pool->rays_shadow += count_path_rays(p);
    else if(bkt == RAY_BUCKET_STARTUP)
      pool->rays_startup += count_path_rays(p);

    /* --- Emit ray 0 --- */
    {
      size_t ray_idx = cursor[bkt]++;
      struct s3d_ray_request* rr = &pool->ray_requests[ray_idx];
      rr->origin[0]    = p->ray_req.origin[0];
      rr->origin[1]    = p->ray_req.origin[1];
      rr->origin[2]    = p->ray_req.origin[2];
      rr->direction[0] = p->ray_req.direction[0];
      rr->direction[1] = p->ray_req.direction[1];
      rr->direction[2] = p->ray_req.direction[2];
      rr->range[0]     = p->ray_req.range[0];
      rr->range[1]     = p->ray_req.range[1];
      rr->user_id      = i;

      if(!S3D_HIT_NONE(&p->filter_data_storage.hit_3d)) {
        rr->filter_data = &p->filter_data_storage;
      } else {
        rr->filter_data = NULL;
      }

      pool->ray_to_slot[ray_idx] = i;
      pool->ray_slot_sub[ray_idx] = 0;
      p->ray_req.batch_idx = (uint32_t)ray_idx;
    }

    /* --- Ray 1 (2-ray request: delta_sphere or reinjection) --- */
    if(p->ray_req.ray_count >= 2) {
      size_t ray_idx = cursor[bkt]++;
      struct s3d_ray_request* rr = &pool->ray_requests[ray_idx];
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
      rr->user_id      = i;

      pool->ray_to_slot[ray_idx] = i;
      pool->ray_slot_sub[ray_idx] = 1;
      p->ray_req.batch_idx2 = (uint32_t)ray_idx;
    }

    /* --- B-4 M1-v2: Rays 2..5 for 6-ray enclosure query --- */
    if(p->ray_count_ext == 6 && p->phase == PATH_ENC_QUERY_EMIT) {
      int j;
      /* Record batch indices for ray 0 and ray 1 (already emitted) */
      p->enc_query.batch_indices[0] = p->ray_req.batch_idx;
      p->enc_query.batch_indices[1] = p->ray_req.batch_idx2;

      for(j = 2; j < 6; j++) {
        size_t ray_idx = cursor[bkt]++;
        struct s3d_ray_request* rr = &pool->ray_requests[ray_idx];
        rr->origin[0]    = p->ray_req.origin[0];
        rr->origin[1]    = p->ray_req.origin[1];
        rr->origin[2]    = p->ray_req.origin[2];
        rr->direction[0] = p->enc_query.directions[j][0];
        rr->direction[1] = p->enc_query.directions[j][1];
        rr->direction[2] = p->enc_query.directions[j][2];
        rr->range[0]     = p->ray_req.range[0];
        rr->range[1]     = p->ray_req.range[1];
        rr->filter_data  = NULL;  /* no self-intersection filter */
        rr->user_id      = i;

        pool->ray_to_slot[ray_idx] = i;
        pool->ray_slot_sub[ray_idx] = (uint32_t)j;
        p->enc_query.batch_indices[j] = (uint32_t)ray_idx;
      }
    }
  }

  /* Total ray count = end of last bucket */
  pool->ray_count = pool->bucket_offsets[RAY_BUCKET_COUNT];
  return RES_OK;
}

/*******************************************************************************
 * B-4 M10: Collect enc_locate requests from paths in PATH_ENC_LOCATE_PENDING
 *
 * Scans active paths for PATH_ENC_LOCATE_PENDING and fills the
 * enc_locate_requests array.  The requests are then dispatched as a
 * single GPU batch via s3d_scene_view_find_enclosure_batch_ctx.
 ******************************************************************************/
LOCAL_SYM res_T
pool_collect_enc_locate_requests(struct wavefront_pool* pool)
{
  size_t k, n = 0;

  for(k = 0; k < pool->active_compact; k++) {
    uint32_t i = pool->active_indices[k];
    struct path_state* p = &pool->slots[i];

    if(p->phase == PATH_ENC_LOCATE_PENDING) {
      struct s3d_enc_locate_request* req = &pool->enc_locate_requests[n];
      req->pos[0] = (float)p->enc_locate.query_pos[0];
      req->pos[1] = (float)p->enc_locate.query_pos[1];
      req->pos[2] = (float)p->enc_locate.query_pos[2];
      req->user_id = i;
      pool->enc_locate_to_slot[n] = i;
      p->enc_locate.batch_idx = (uint32_t)n;
      n++;
    }
  }

  pool->enc_locate_count = n;
  return RES_OK;
}

/*******************************************************************************
 * B-4 M10: Distribute enc_locate results back to paths
 *
 * For each enc_locate result, write prim_id/side/distance into the path's
 * enc_locate struct, then transition to PATH_ENC_LOCATE_RESULT so the
 * cascade loop can resolve enc_id via step_enc_locate_result.
 ******************************************************************************/
static res_T
pool_distribute_enc_locate_results(struct wavefront_pool* pool)
{
  size_t k;

  for(k = 0; k < pool->enc_locate_count; k++) {
    uint32_t slot_id = pool->enc_locate_to_slot[k];
    struct path_state* p = &pool->slots[slot_id];
    const struct s3d_enc_locate_result* r = &pool->enc_locate_results[k];

    p->enc_locate.prim_id  = r->prim_id;
    p->enc_locate.side     = r->side;
    p->enc_locate.distance = r->distance;
    p->phase = PATH_ENC_LOCATE_RESULT;
  }

  return RES_OK;
}

static res_T
pool_distribute_ray_results(struct wavefront_pool* pool, struct sdis_scene* scn)
{
  size_t k;
  res_T res = RES_OK;
  size_t bucket_other_n = 0;

  /* ---- Phase 1: Radiative paths (1 ray each) ---- */
  for(k = 0; k < pool->bucket_radiative_n; k++) {
    uint32_t i = pool->bucket_radiative[k];
    struct path_state* p = &pool->slots[i];
    const struct s3d_hit* h0 = &pool->ray_hits[p->ray_req.batch_idx];

    p->needs_ray = 0;
    res = step_radiative_trace(p, scn, h0);
    if(res != RES_OK && res != RES_BAD_OP
    && res != RES_BAD_OP_IRRECOVERABLE) return res;
    if(res == RES_BAD_OP || res == RES_BAD_OP_IRRECOVERABLE) {
      mark_path_failed(p, pool);
      res = RES_OK;
    }
    p->steps_taken++;
  }

  /* ---- Phase 2: Conductive / delta-sphere paths (2 rays each) ---- */
  for(k = 0; k < pool->bucket_conductive_n; k++) {
    uint32_t i = pool->bucket_conductive[k];
    struct path_state* p = &pool->slots[i];
    const struct s3d_hit* h0 = &pool->ray_hits[p->ray_req.batch_idx];
    const struct s3d_hit* h1 = NULL;
    if(p->ray_req.ray_count >= 2)
      h1 = &pool->ray_hits[p->ray_req.batch_idx2];

    p->needs_ray = 0;
    res = step_conductive_ds_process(p, scn, h0, h1);
    if(res != RES_OK && res != RES_BAD_OP
    && res != RES_BAD_OP_IRRECOVERABLE) return res;
    if(res == RES_BAD_OP || res == RES_BAD_OP_IRRECOVERABLE) {
      mark_path_failed(p, pool);
      res = RES_OK;
    }
    p->steps_taken++;
  }

  /* ---- Phase 3: Fallback — unbucketed ray paths (boundary reinject etc.) ---- */
  bucket_other_n = pool->need_ray_count
                 - pool->bucket_radiative_n
                 - pool->bucket_conductive_n;
  if(bucket_other_n > 0) {
    for(k = 0; k < pool->need_ray_count; k++) {
      uint32_t i = pool->need_ray_indices[k];
      struct path_state* p = &pool->slots[i];

      /* Skip already-processed buckets */
      if(p->phase == PATH_RAD_TRACE_PENDING) continue;
      if(p->phase == PATH_COUPLED_COND_DS_PENDING) continue;
      if(p->phase == PATH_CND_DS_STEP_TRACE) continue;
      /* These paths have already been advanced by the bucketed loops above,
       * so their phase has changed.  Check needs_ray to find remaining ones. */
      if(!p->needs_ray) continue;

      {
        const struct s3d_hit* h0 = &pool->ray_hits[p->ray_req.batch_idx];
        const struct s3d_hit* h1 = NULL;
        if(p->ray_req.ray_count >= 2)
          h1 = &pool->ray_hits[p->ray_req.batch_idx2];

        /* B-4 M1-v2: Pre-deliver 6-ray enc_query hits before advance */
        if(p->phase == PATH_ENC_QUERY_EMIT && p->ray_count_ext == 6) {
          int j;
          for(j = 0; j < 6; j++) {
            p->enc_query.dir_hits[j] =
              pool->ray_hits[p->enc_query.batch_indices[j]];
          }
        }
        /* B-4 M1-v2: Pre-deliver fallback enc_query hit */
        if(p->phase == PATH_ENC_QUERY_FB_EMIT) {
          p->enc_query.fb_hit = pool->ray_hits[p->ray_req.batch_idx];
        }

        {
          enum path_phase phase_before = p->phase;
          p->needs_ray = 0;
          res = advance_one_step_with_ray(p, scn, h0, h1);
          if(res != RES_OK && res != RES_BAD_OP
          && res != RES_BAD_OP_IRRECOVERABLE) return res;
          if(res == RES_BAD_OP || res == RES_BAD_OP_IRRECOVERABLE) {
            mark_path_failed(p, pool);
            res = RES_OK;
          }
          /* Track M1-v2 → M10 escalation */
          if(phase_before == PATH_ENC_QUERY_FB_EMIT
          && p->phase == PATH_ENC_LOCATE_PENDING) {
            pool->enc_query_escalated_to_m10++;
          }
          p->steps_taken++;
        }
      }
    }
  }

  return RES_OK;
}

/*******************************************************************************
 * Cascade non-ray steps -- serial inner kernel
 *
 * Advances a single path through non-ray phases until a ray is needed,
 * the path completes, or no progress can be made.  Accumulates statistics
 * into caller-provided local counters (thread-safe for OMP).
 *
 * Returns: 0 = ok, 1 = fatal error (propagate to caller).
 ******************************************************************************/
static int
cascade_advance_single_path(
  struct path_state* p,
  struct sdis_scene* scn,
  /* --- thread-local accumulators --- */
  size_t* local_iterations,
  size_t* local_advances,
  size_t* local_paths_failed,
  size_t* local_enc_degenerate_null,
  size_t  local_phase_count[],
  double  local_phase_time[])
{
  for(;;) {
    int advanced = 0;
    enum path_phase phase_before;
    struct time t_step0, t_step1;
    res_T res;

    if(p->needs_ray) break;
    if(p->phase == PATH_DONE || p->phase == PATH_ERROR) {
      /* M8: intercept PATH_DONE when sfn_stack_depth > 0. */
      if(p->phase == PATH_DONE && p->sfn_stack_depth > 0) {
        p->phase = PATH_BND_SFN_COMPUTE_Ti_RESUME;
        p->active = 1;
        continue;
      }
      break;
    }
    if(path_phase_is_ray_pending(p->phase)) break;
    if(path_phase_is_enc_locate_pending(p->phase)) break;

    (*local_iterations)++;
    phase_before = p->phase;
    time_current(&t_step0);

    res = advance_one_step_no_ray(p, scn, &advanced);

    time_current(&t_step1);
    if((int)phase_before >= 0 && (int)phase_before < PATH_PHASE_COUNT) {
      local_phase_count[(int)phase_before]++;
      local_phase_time[(int)phase_before]
        += time_elapsed_sec(&t_step0, &t_step1);
    }
    if(phase_before == PATH_ENC_LOCATE_RESULT
    && p->enc_locate.prim_id >= 0 && p->enc_locate.side < 0) {
      (*local_enc_degenerate_null)++;
    }

    if(res != RES_OK && res != RES_BAD_OP
    && res != RES_BAD_OP_IRRECOVERABLE) return 1; /* fatal */
    if(res == RES_BAD_OP || res == RES_BAD_OP_IRRECOVERABLE) {
      p->phase = PATH_DONE;
      p->active = 0;
      p->done_reason = -1;
      (*local_paths_failed)++;
      break;
    }
    if(!advanced) break;
    (*local_advances)++;
    p->steps_taken++;
  }

  return 0; /* ok */
}

/*******************************************************************************
 * Cascade non-ray steps -- compact version (M2.5)
 *
 * For each active path, advance through non-ray phases until a ray is
 * needed, the path completes, or no progress can be made.
 *
 * OMP parallelization:
 *   - Controlled by STARDIS_CASCADE_OMP env var (default=1=on).
 *   - Thread count from scn->dev->nthreads (matches CPU version's -t flag).
 *   - schedule(dynamic, 64) for load balancing (variable cascade depth).
 *   - Per-thread local accumulators, reduced after the parallel region.
 ******************************************************************************/
static res_T
pool_cascade_non_ray_steps_compact(struct wavefront_pool* pool,
                                   struct sdis_scene* scn)
{
  const size_t n = pool->active_compact;
  int use_omp = 1;
  int omp_nthreads;
  int had_fatal = 0;

  /* Check env var for cascade OMP toggle (default = on) */
  {
    const char* env = getenv("STARDIS_CASCADE_OMP");
    if(env && env[0] == '0') use_omp = 0;
  }

  /* Determine thread count: use scn->dev->nthreads, same as CPU -t flag */
  omp_nthreads = (scn && scn->dev) ? (int)scn->dev->nthreads : 1;
  if(omp_nthreads < 1) omp_nthreads = 1;

  /* For very small wavefronts, serial is faster than OMP overhead */
  if(n < 64) use_omp = 0;

  if(use_omp) {
    /* === OMP parallel cascade === */
    #pragma omp parallel num_threads(omp_nthreads)
    {
      /* Per-thread local accumulators */
      size_t tl_iterations = 0;
      size_t tl_advances = 0;
      size_t tl_paths_failed = 0;
      size_t tl_enc_degenerate_null = 0;
      size_t tl_phase_count[PATH_PHASE_COUNT];
      double tl_phase_time[PATH_PHASE_COUNT];
      int ph;
      for(ph = 0; ph < PATH_PHASE_COUNT; ph++) {
        tl_phase_count[ph] = 0;
        tl_phase_time[ph] = 0.0;
      }

      #pragma omp for schedule(dynamic, 64)
      for(ph = 0; ph < (int)n; ph++) {
        uint32_t idx = pool->active_indices[ph];
        struct path_state* p = &pool->slots[idx];
        if(!p->active) continue;

        if(cascade_advance_single_path(
              p, scn,
              &tl_iterations, &tl_advances,
              &tl_paths_failed, &tl_enc_degenerate_null,
              tl_phase_count, tl_phase_time)) {
          /* Fatal error — signal, but can't break from OMP for.
           * Other threads will finish their current work items.
           * Plain write is safe: x86 aligned-int stores are atomic,
           * and we only ever set to 1 (monotonic flag). */
          had_fatal = 1;
        }
      }

      /* Reduction: merge thread-local counters into pool (critical) */
      #pragma omp critical
      {
        int j;
        pool->cascade_total_iterations += tl_iterations;
        pool->cascade_total_advances   += tl_advances;
        pool->paths_failed             += tl_paths_failed;
        pool->enc_locate_degenerate_null += tl_enc_degenerate_null;
        for(j = 0; j < PATH_PHASE_COUNT; j++) {
          pool->cascade_phase_count[j] += tl_phase_count[j];
          pool->cascade_phase_time[j]  += tl_phase_time[j];
        }
      }
    } /* end omp parallel */

    return had_fatal ? RES_UNKNOWN_ERR : RES_OK;
  }

  /* === Serial fallback (STARDIS_CASCADE_OMP=0 or small wavefront) === */
  {
    size_t k;
    size_t tl_iterations = 0;
    size_t tl_advances = 0;
    size_t tl_paths_failed = 0;
    size_t tl_enc_degenerate_null = 0;
    size_t tl_phase_count[PATH_PHASE_COUNT];
    double tl_phase_time[PATH_PHASE_COUNT];
    int ph;
    for(ph = 0; ph < PATH_PHASE_COUNT; ph++) {
      tl_phase_count[ph] = 0;
      tl_phase_time[ph] = 0.0;
    }

    for(k = 0; k < n; k++) {
      uint32_t idx = pool->active_indices[k];
      struct path_state* p = &pool->slots[idx];
      if(!p->active) continue;

      if(cascade_advance_single_path(
            p, scn,
            &tl_iterations, &tl_advances,
            &tl_paths_failed, &tl_enc_degenerate_null,
            tl_phase_count, tl_phase_time)) {
        return RES_UNKNOWN_ERR;
      }
    }

    pool->cascade_total_iterations += tl_iterations;
    pool->cascade_total_advances   += tl_advances;
    pool->paths_failed             += tl_paths_failed;
    pool->enc_locate_degenerate_null += tl_enc_degenerate_null;
    {
      int j;
      for(j = 0; j < PATH_PHASE_COUNT; j++) {
        pool->cascade_phase_count[j] += tl_phase_count[j];
        pool->cascade_phase_time[j]  += tl_phase_time[j];
      }
    }
  }

  return RES_OK;
}

/*******************************************************************************
 * Harvest completed paths -- write directly to estimator_buffer (M2)
 *
 * Called each wavefront step.  Marks completed slots as available for
 * refill.
 ******************************************************************************/
static res_T
harvest_completed_paths(
  struct wavefront_pool* pool,
  struct sdis_estimator_buffer* buf)
{
  size_t k;

  ASSERT(pool && buf);

  for(k = 0; k < pool->done_count; k++) {
    uint32_t i = pool->done_indices[k];
    struct path_state* p = &pool->slots[i];

    /* Skip if already harvested */
    if(p->phase == PATH_HARVESTED)
      continue;
    if(!p->active && p->phase != PATH_DONE && p->phase != PATH_ERROR)
      continue;

    {
      struct sdis_estimator* estimator =
        estimator_buffer_grab(buf, p->ipix_image[0], p->ipix_image[1]);

      if(p->T.done) {
        estimator->temperature.sum  += p->T.value;
        estimator->temperature.sum2 += p->T.value * p->T.value;
        estimator->temperature.count += 1;
        estimator->realisation_time.count += 1;
        estimator->nrealisations += 1;
      }
    }

    /* Per-path temperature trace */
    { FILE* tf = pixel_trace_file();
      if(tf) {
        fprintf(tf, "%u,%u,%u,%u,%.17g,%d,%d,%lu,%d\n",
          (unsigned)p->path_id,
          (unsigned)p->ipix_image[0], (unsigned)p->ipix_image[1],
          (unsigned)p->realisation_idx,
          p->T.value, (int)p->T.done, p->done_reason,
          (unsigned long)p->steps_taken, (int)p->phase);
      }
    }

    /* Track max depth */
    if(p->steps_taken > pool->max_path_depth)
      pool->max_path_depth = p->steps_taken;

    /* Classify done reason for statistics */
    pool->paths_completed++;
    switch(p->done_reason) {
    case 1:  pool->paths_done_radiative++;   break;
    case 2:  pool->paths_done_temperature++; break;
    case 3:  pool->paths_done_boundary++;    break;
    case 4:  pool->paths_done_temperature++; break;
    case -1: /* already counted in paths_failed */ break;
    default: break;
    }

    /* Mark slot as harvested -- prevents re-accumulation if refill
     * cannot replace this slot (task queue exhausted).  The new
     * PATH_HARVESTED state is a terminal state distinct from PATH_DONE
     * that compact_active_paths and refill_pool recognise. */
    p->active = 0;
    p->phase  = PATH_HARVESTED;
  }

  return RES_OK;
}

/*******************************************************************************
 * Refill pool -- replace completed slots with new tasks from the queue (M2)
 *
 * Scans done_indices for PATH_DONE, inactive slots and fills them with new
 * tasks.  Returns refill count.
 ******************************************************************************/
static res_T
refill_pool(struct wavefront_pool* pool, size_t* out_refill_count)
{
  size_t k, refill_count = 0;
  res_T res = RES_OK;

  if(pool->task_next >= pool->task_count) {
    *out_refill_count = 0;
    return RES_OK;
  }

  for(k = 0; k < pool->done_count; k++) {
    uint32_t i = pool->done_indices[k];
    struct path_state* p = &pool->slots[i];

    /* Only refill slots that have been harvested */
    if(p->active) continue;
    if(p->phase != PATH_DONE && p->phase != PATH_ERROR
    && p->phase != PATH_HARVESTED) continue;
    if(pool->task_next >= pool->task_count) break;

    {
      const struct pixel_task* task = &pool->task_queue[pool->task_next];

      res = init_single_path(
        p, pool->slot_rngs[i], task,
        pool->scn, pool->enc_id, pool->cam,
        pool->time_range, pool->pix_sz,
        pool->picard_order, pool->diff_algo,
        pool->next_path_id++, pool->global_seed);
      if(res != RES_OK) return res;

      /* Advance past PATH_INIT */
      res = advance_path_to_first_ray(p, pool->scn, pool);
      if(res != RES_OK) return res;

      pool->task_next++;
      refill_count++;
    }
  }

  pool->diag_refill_count += refill_count;
  *out_refill_count = refill_count;
  return RES_OK;
}

/*******************************************************************************
 * Update active count
 ******************************************************************************/
static void
pool_update_active_count(struct wavefront_pool* pool)
{
  size_t i, count = 0;
  for(i = 0; i < pool->pool_size; i++) {
    if(pool->slots[i].active) count++;
  }
  pool->active_count = count;
}

/*******************************************************************************
 * Update diagnostics -- track batch size statistics + wavefront width (M3)
 ******************************************************************************/
static void
pool_update_diagnostics(struct wavefront_pool* pool)
{
  if(pool->ray_count > 0) {
    if(pool->ray_count < pool->diag_min_batch)
      pool->diag_min_batch = pool->ray_count;
    if(pool->ray_count > pool->diag_max_batch)
      pool->diag_max_batch = pool->ray_count;
  }

  if(pool->in_drain_phase) {
    pool->diag_drain_rays += pool->ray_count;
    pool->drain_step_count++;
  } else {
    pool->diag_refill_rays += pool->ray_count;
  }

  /* M3: accumulate active_count for average wavefront width */
  pool->diag_total_active += pool->active_count;
}

/*******************************************************************************
 * Log drain phase report — enhanced with M3 per-phase timing + wavefront width
 ******************************************************************************/
static void
log_drain_phase_report(struct sdis_device* dev, struct wavefront_pool* pool)
{
  double drain_ray_pct = 0;
  double avg_width = 0;
  if(pool->total_rays_traced > 0) {
    drain_ray_pct = (double)pool->diag_drain_rays * 100.0
                  / (double)pool->total_rays_traced;
  }
  if(pool->total_steps > 0) {
    avg_width = (double)pool->diag_total_active / (double)pool->total_steps;
  }
  log_info(dev,
    "persistent wavefront summary:\n"
    "  total_steps=%lu  total_rays=%lu  avg_wavefront_width=%.1f\n"
    "  refill_phase: rays=%lu (%.1f%%)  wall=%.3fs\n"
    "  drain_phase:  rays=%lu (%.1f%%)  steps=%lu  wall=%.3fs\n"
    "  batch_size: min=%lu, max=%lu\n"
    "  ray_buckets: radiative=%lu, step_pair=%lu, "
    "shadow=%lu, startup=%lu\n"
    "  paths: completed=%lu, failed=%lu, truncated=%lu, max_depth=%lu\n"
    "  refills=%lu\n"
    "  timing: compact=%.3fs  collect=%.3fs  trace=%.3fs  "
    "distribute=%.3fs  cascade=%.3fs  harvest+refill=%.3fs\n",
    (unsigned long)pool->total_steps,
    (unsigned long)pool->total_rays_traced,
    avg_width,
    (unsigned long)pool->diag_refill_rays,
    100.0 - drain_ray_pct,
    pool->time_refill_phase_s,
    (unsigned long)pool->diag_drain_rays,
    drain_ray_pct,
    (unsigned long)pool->drain_step_count,
    pool->time_drain_phase_s,
    (unsigned long)(pool->diag_min_batch == (size_t)-1
                    ? 0 : pool->diag_min_batch),
    (unsigned long)pool->diag_max_batch,
    (unsigned long)pool->rays_radiative,
    (unsigned long)pool->rays_conductive_ds,
    (unsigned long)pool->rays_shadow,
    (unsigned long)pool->rays_startup,
    (unsigned long)pool->paths_completed,
    (unsigned long)pool->paths_failed,
    (unsigned long)pool->paths_truncated,
    (unsigned long)pool->max_path_depth,
    (unsigned long)pool->diag_refill_count,
    pool->time_compact_s,
    pool->time_collect_s,
    pool->time_trace_s,
    pool->time_distribute_s,
    pool->time_cascade_s,
    pool->time_harvest_s);

  /* Enclosure query escalation stats */
  if(pool->enc_query_escalated_to_m10 > 0
  || pool->enc_locate_degenerate_null > 0) {
    log_info(dev,
      "  enc_escalation: query_fb->m10=%lu  m10_degenerate_null=%lu\n",
      (unsigned long)pool->enc_query_escalated_to_m10,
      (unsigned long)pool->enc_locate_degenerate_null);
  }

  /* --- Experiment 3: Cascade per-phase top-N hotspots --- */
  if(pool->cascade_total_advances > 0) {
    /* Collect phase indices sorted by time (simple selection of top 10) */
    int top[10];
    int ti, tj;
    int used[PATH_PHASE_COUNT];
    memset(used, 0, sizeof(used));

    for(ti = 0; ti < 10; ti++) {
      int best = -1;
      double best_t = 0;
      for(tj = 0; tj < PATH_PHASE_COUNT; tj++) {
        if(used[tj]) continue;
        if(pool->cascade_phase_time[tj] > best_t) {
          best_t = pool->cascade_phase_time[tj];
          best = tj;
        }
      }
      top[ti] = best;
      if(best >= 0) used[best] = 1;
    }

    log_info(dev,
      "cascade profiling: total_iterations=%lu  total_advances=%lu\n",
      (unsigned long)pool->cascade_total_iterations,
      (unsigned long)pool->cascade_total_advances);

    for(ti = 0; ti < 10; ti++) {
      int ph = top[ti];
      if(ph < 0 || pool->cascade_phase_count[ph] == 0) break;
      log_info(dev,
        "  cascade phase[%2d]: count=%10lu  time=%8.3fs  avg=%.3fus  "
        "(%.1f%% of cascade)\n",
        ph,
        (unsigned long)pool->cascade_phase_count[ph],
        pool->cascade_phase_time[ph],
        pool->cascade_phase_time[ph] * 1e6
          / (double)pool->cascade_phase_count[ph],
        pool->cascade_phase_time[ph] * 100.0 / pool->time_cascade_s);
    }
  }

  /* --- Experiment 7+8: Batch trace per-call summary --- */
  if(pool->trace_call_count > 0) {
    double avg_batch = (double)pool->trace_batch_size_sum
                     / (double)pool->trace_call_count;
    double avg_batch_ms = pool->trace_batch_time_ms_sum
                        / (double)pool->trace_call_count;
    double avg_post_ms  = pool->trace_post_time_ms_sum
                        / (double)pool->trace_call_count;
    double total_trace_ms = pool->trace_batch_time_ms_sum
                          + pool->trace_post_time_ms_sum
                          + pool->trace_retrace_time_ms_sum;
    double gpu_pct  = pool->trace_batch_time_ms_sum * 100.0 / total_trace_ms;
    double post_pct = pool->trace_post_time_ms_sum * 100.0 / total_trace_ms;
    double rtrc_pct = pool->trace_retrace_time_ms_sum * 100.0 / total_trace_ms;
    double throughput = (double)pool->total_rays_traced
                      / (pool->trace_batch_time_ms_sum * 1e-3) / 1e6;

    log_info(dev,
      "batch trace profiling: calls=%lu  avg_batch=%.0f  "
      "min=%lu  max=%lu\n"
      "  gpu_kernel+upload: total=%.1fms (%.1f%%)  avg=%.2fms/call\n"
      "  cpu_postprocess:   total=%.1fms (%.1f%%)  avg=%.2fms/call\n"
      "  fallback_retrace:  total=%.1fms (%.1f%%)  "
      "accepted=%lu  missed=%lu  rejected=%lu\n"
      "  gpu_throughput: %.1f Mrays/s  (kernel+upload only)\n",
      (unsigned long)pool->trace_call_count,
      avg_batch,
      (unsigned long)(pool->trace_batch_size_min == (size_t)-1
                      ? 0 : pool->trace_batch_size_min),
      (unsigned long)pool->trace_batch_size_max,
      pool->trace_batch_time_ms_sum, gpu_pct, avg_batch_ms,
      pool->trace_post_time_ms_sum, post_pct, avg_post_ms,
      pool->trace_retrace_time_ms_sum, rtrc_pct,
      (unsigned long)pool->trace_retrace_accepted_sum,
      (unsigned long)pool->trace_retrace_missed_sum,
      (unsigned long)pool->trace_filter_rejected_sum,
      throughput);
  }
}

/*******************************************************************************
 * Determine pool size — adaptive based on GPU SM count (M3)
 *
 * Formula: pool_size = sm_count × WARPS_PER_SM × 32
 *   - Ensures enough threads to fill all SMs with reasonable occupancy.
 *   - Falls back to PERSISTENT_WF_DEFAULT_POOL_SIZE if SM count unavailable.
 *   - Capped at total_tasks (no point allocating more slots than tasks).
 *   - Overridable via STARDIS_POOL_SIZE environment variable.
 ******************************************************************************/
#define WARPS_PER_SM  8  /* Conservative occupancy target per SM */

static size_t
determine_pool_size(size_t total_tasks, struct sdis_scene* scn)
{
  size_t pool_size = PERSISTENT_WF_DEFAULT_POOL_SIZE;
  const char* env = getenv("STARDIS_POOL_SIZE");

  /* Priority 1: Environment variable override */
  if(env) {
    long val = atol(env);
    if(val > 0) {
      pool_size = (size_t)val;
      goto cap;
    }
  }

  /* Priority 2: Adaptive sizing from GPU SM count */
  if(scn && scn->dev && scn->dev->s3d_dev) {
    int sm_count = s3d_device_get_gpu_sm_count(scn->dev->s3d_dev);
    if(sm_count > 0) {
      /* sm_count × warps_per_SM × 32 threads/warp */
      pool_size = (size_t)sm_count * WARPS_PER_SM * 32;
    }
  }

cap:
  if(pool_size > total_tasks) pool_size = total_tasks;
  if(pool_size == 0) pool_size = 1;

  return pool_size;
}

/*******************************************************************************
 * Public entry point
 ******************************************************************************/
#ifndef SDIS_SOLVE_PERSISTENT_WAVEFRONT_SKIP_PUBLIC_API
res_T
solve_camera_persistent_wavefront(
  struct sdis_scene*          scn,
  struct ssp_rng**            per_thread_rng,
  const size_t                nthreads,
  const unsigned              enc_id,
  const struct sdis_camera*   cam,
  const double                time_range[2],
  const size_t                image_def[2],
  const size_t                spp,
  const int                   register_paths,
  const double                pix_sz[2],
  const size_t                picard_order,
  const enum sdis_diffusion_algorithm diff_algo,
  struct sdis_estimator_buffer* buf,
  int32_t                     progress[],
  const int                   pcent_progress,
  const char*                 progress_label)
{
  struct wavefront_pool pool;
  size_t total_tasks;
  size_t pool_size;
  size_t i;
  res_T res = RES_OK;
  struct time t_start, t_end, t_elapsed;
  int last_pcent = 0;

  ASSERT(scn && per_thread_rng && nthreads > 0 && cam && spp);
  ASSERT(image_def && image_def[0] > 0 && image_def[1] > 0);
  ASSERT(pix_sz && pix_sz[0] > 0 && pix_sz[1] > 0 && time_range && buf);

  (void)register_paths; /* heat path tracking not yet supported */

  total_tasks = image_def[0] * image_def[1] * spp;
  pool_size = determine_pool_size(total_tasks, scn);

  {
    int sm = (scn->dev && scn->dev->s3d_dev)
           ? s3d_device_get_gpu_sm_count(scn->dev->s3d_dev) : 0;
    log_info(scn->dev,
      "Persistent wavefront (Phase B-3 M3): %lux%lu spp=%lu, "
      "pool_size=%lu (SM=%d), total_tasks=%lu\n",
      (unsigned long)image_def[0], (unsigned long)image_def[1],
      (unsigned long)spp, (unsigned long)pool_size,
      sm, (unsigned long)total_tasks);
  }

  /* Log cascade OMP configuration */
  {
    const char* env = getenv("STARDIS_CASCADE_OMP");
    int cascade_omp = (env && env[0] == '0') ? 0 : 1;
    int omp_nt = (scn && scn->dev) ? (int)scn->dev->nthreads : 1;
    if(omp_nt < 1) omp_nt = 1;
    log_info(scn->dev,
      "Cascade OMP: %s, threads=%d (STARDIS_CASCADE_OMP=%s)\n",
      cascade_omp ? "ENABLED" : "DISABLED",
      cascade_omp ? omp_nt : 1,
      env ? env : "<unset,default=1>");
  }

  /* ====== 1. Allocate pool ====== */
  res = pool_create(&pool, pool_size);
  if(res != RES_OK) {
    log_err(scn->dev,
      "persistent_wavefront: pool_create failed for %lu paths -- %s\n",
      (unsigned long)pool_size, res_to_cstr(res));
    goto cleanup;
  }

  /* ====== 2. Create per-path CBRNG thin wrappers ====== */
  /* Each slot gets its own ssp_rng whose desc.get delegates to wf_rng_get()
   * on the embedded path_state.rng_state.  This eliminates the round-robin
   * RNG sharing that caused 32x32-pixel block noise patterns. */
  {
    struct ssp_rng* thin_storage = NULL;
    /* Build array of wf_rng pointers (one per slot) for the C++ adapter */
    struct wf_rng** rng_ptrs = (struct wf_rng**)malloc(
        pool.pool_size * sizeof(struct wf_rng*));
    if(!rng_ptrs) {
      res = RES_MEM_ERR;
      log_err(scn->dev,
        "persistent_wavefront: rng_ptrs alloc failed\n");
      goto cleanup;
    }
    { size_t ri;
      for(ri = 0; ri < pool.pool_size; ri++)
        rng_ptrs[ri] = &pool.slots[ri].rng_state;
    }
    res = wf_rng_create_thin_ssp_rngs(pool.pool_size, rng_ptrs,
                                       pool.slot_rngs, &thin_storage);
    free(rng_ptrs);
    if(res != RES_OK) {
      log_err(scn->dev,
        "persistent_wavefront: wf_rng thin wrapper creation failed -- %s\n",
        res_to_cstr(res));
      goto cleanup;
    }
    pool.thin_rng_storage = thin_storage;
  }

  /* Store global seed for per-path CBRNG seeding.
   * Derived from the first per_thread_rng to ensure deterministic
   * behaviour for a given input seed (user's -s parameter). */
  {
    double r0 = ssp_rng_canonical(per_thread_rng[0]);
    double r1 = ssp_rng_canonical(per_thread_rng[0]);
    uint32_t hi = (uint32_t)(r0 * 4294967296.0);  /* [0, 2^32) */
    uint32_t lo = (uint32_t)(r1 * 4294967296.0);  /* [0, 2^32) */
    pool.global_seed = ((uint64_t)hi << 32) | (uint64_t)lo;
  }

  /* ====== 3. Generate task queue ====== */
  res = generate_task_queue(&pool, image_def, spp);
  if(res != RES_OK) {
    log_err(scn->dev,
      "persistent_wavefront: task queue generation failed -- %s\n",
      res_to_cstr(res));
    goto cleanup;
  }

  /* ====== 4. Create batch trace context ====== */
  res = s3d_batch_trace_context_create(&pool.batch_ctx, pool.max_rays);
  if(res != RES_OK) {
    log_err(scn->dev,
      "persistent_wavefront: batch_ctx creation failed for %lu rays -- %s\n",
      (unsigned long)pool.max_rays, res_to_cstr(res));
    goto cleanup;
  }

  /* ====== 4b. Create batch enc_locate context (M10) ====== */
  res = s3d_batch_enc_context_create(&pool.enc_batch_ctx, pool.max_enc_locates);
  if(res != RES_OK) {
    log_err(scn->dev,
      "persistent_wavefront: enc_batch_ctx creation failed for %lu queries -- %s\n",
      (unsigned long)pool.max_enc_locates, res_to_cstr(res));
    goto cleanup;
  }

  /* ====== 5. Cache scene params for refill ====== */
  pool.scn          = scn;
  pool.enc_id       = enc_id;
  pool.cam          = cam;
  pool.time_range   = time_range;
  pool.pix_sz       = pix_sz;
  pool.picard_order = picard_order;
  pool.diff_algo    = diff_algo;

  /* ====== 6. Fill pool with initial tasks ====== */
  res = fill_pool(&pool);
  if(res != RES_OK) goto cleanup;

  log_info(scn->dev,
    "persistent_wavefront: %lu paths initialised (%lu/%lu tasks consumed), "
    "starting wavefront loop\n",
    (unsigned long)pool.active_count,
    (unsigned long)pool.task_next,
    (unsigned long)pool.task_count);

  /* ====== 7. Wavefront main loop ====== */
  time_current(&t_start);

  while(pool.active_count > 0 || pool.task_next < pool.task_count) {
    size_t refill_count = 0;
    struct time t_phase0, t_phase1; /* M3 per-phase timing */
    pool.total_steps++;

    /* Step A: Stream compaction (M2.5) */
    time_current(&t_phase0);
    compact_active_paths(&pool);
    time_current(&t_phase1);
    pool.time_compact_s += time_elapsed_sec(&t_phase0, &t_phase1);

    /* Step B: Collect ray requests — bucketed (B-4 M2) */
    time_current(&t_phase0);
    pool.ray_count = 0;
    res = pool_collect_ray_requests_bucketed(&pool);
    if(res != RES_OK) goto cleanup;
    time_current(&t_phase1);
    pool.time_collect_s += time_elapsed_sec(&t_phase0, &t_phase1);

    /* Step C: Batch trace via Phase B-1 */
    time_current(&t_phase0);
    if(pool.ray_count > 0) {
      struct s3d_batch_trace_stats stats;
      memset(&stats, 0, sizeof(stats));

      res = s3d_scene_view_trace_rays_batch_ctx(
        scn->s3d_view, pool.batch_ctx,
        pool.ray_requests, pool.ray_count,
        pool.ray_hits, &stats);
      if(res != RES_OK) goto cleanup;

      pool.total_rays_traced += pool.ray_count;

      /* Experiment 7+8: accumulate per-call batch trace stats */
      pool.trace_call_count++;
      pool.trace_batch_size_sum += pool.ray_count;
      if(pool.ray_count < pool.trace_batch_size_min)
        pool.trace_batch_size_min = pool.ray_count;
      if(pool.ray_count > pool.trace_batch_size_max)
        pool.trace_batch_size_max = pool.ray_count;
      pool.trace_batch_time_ms_sum   += stats.batch_time_ms;
      pool.trace_post_time_ms_sum    += stats.postprocess_time_ms;
      pool.trace_retrace_time_ms_sum += stats.retrace_time_ms;
      pool.trace_retrace_accepted_sum += stats.retrace_accepted;
      pool.trace_retrace_missed_sum   += stats.retrace_missed;
      pool.trace_filter_rejected_sum  += stats.filter_rejected;
    }
    time_current(&t_phase1);
    pool.time_trace_s += time_elapsed_sec(&t_phase0, &t_phase1);

    /* Step D: Distribute ray results — bucketed dispatch (M3) */
    time_current(&t_phase0);
    res = pool_distribute_ray_results(&pool, scn);
    if(res != RES_OK) goto cleanup;
    time_current(&t_phase1);
    pool.time_distribute_s += time_elapsed_sec(&t_phase0, &t_phase1);

    /* Step D2 (M10): Collect + dispatch + distribute enc_locate batch */
    time_current(&t_phase0);
    res = pool_collect_enc_locate_requests(&pool);
    if(res != RES_OK) goto cleanup;

    if(pool.enc_locate_count > 0) {
      struct s3d_batch_enc_stats enc_stats;
      memset(&enc_stats, 0, sizeof(enc_stats));

      res = s3d_scene_view_find_enclosure_batch_ctx(
        scn->s3d_view, pool.enc_batch_ctx,
        pool.enc_locate_requests, pool.enc_locate_count,
        pool.enc_locate_results, &enc_stats);
      if(res != RES_OK) goto cleanup;

      /* Distribute prim_id + side to paths; enc_id resolution happens
       * in step_enc_locate_result() during cascade (handles degenerate). */
      res = pool_distribute_enc_locate_results(&pool);
      if(res != RES_OK) goto cleanup;

      pool.enc_locates_total += pool.enc_locate_count;
      pool.enc_locates_resolved += enc_stats.resolved;
      pool.enc_locates_degenerate += enc_stats.degenerate;
    }
    time_current(&t_phase1);
    pool.time_enc_locate_s += time_elapsed_sec(&t_phase0, &t_phase1);

    /* Step E: Cascade non-ray steps (compact) */
    time_current(&t_phase0);
    res = pool_cascade_non_ray_steps_compact(&pool, scn);
    if(res != RES_OK) goto cleanup;
    time_current(&t_phase1);
    pool.time_cascade_s += time_elapsed_sec(&t_phase0, &t_phase1);

    /* Step F+G: Harvest completed paths + refill (timed together) */
    time_current(&t_phase0);
    compact_active_paths(&pool); /* rebuild done_indices after cascade */
    res = harvest_completed_paths(&pool, buf);
    if(res != RES_OK) goto cleanup;

    /* Step G: Refill pool with new tasks (M2) */
    res = refill_pool(&pool, &refill_count);
    if(res != RES_OK) goto cleanup;
    time_current(&t_phase1);
    pool.time_harvest_s += time_elapsed_sec(&t_phase0, &t_phase1);

    /* Step H: Update active count */
    pool_update_active_count(&pool);

    /* Step I: Detect drain phase transition */
    if(!pool.in_drain_phase && pool.task_next >= pool.task_count) {
      pool.in_drain_phase = 1;
      /* Record refill phase total wall-clock */
      {
        struct time t_now;
        time_current(&t_now);
        pool.time_refill_phase_s = time_elapsed_sec(&t_start, &t_now);
      }
      log_info(scn->dev,
        "persistent_wavefront: entering drain phase at step %lu, "
        "%lu active paths remain, refill_wall=%.3fs\n",
        (unsigned long)pool.total_steps,
        (unsigned long)pool.active_count,
        pool.time_refill_phase_s);
    }

    /* Step J: Update diagnostics (M2.5 + M3 wavefront width) */
    pool_update_diagnostics(&pool);

    /* Step K: Progress reporting */
    {
      size_t tasks_done = pool.paths_completed + pool.paths_failed;
      int pcent = (int)((double)tasks_done * 100.0
                      / (double)total_tasks + 0.5);
      if(pcent > 100) pcent = 100;

      if(pcent / pcent_progress > last_pcent / pcent_progress) {
        last_pcent = pcent;
        if(progress) {
          progress[0] = pcent;
          print_progress_update(scn->dev, progress, progress_label);
        }
      }
    }

    /* Step L: Periodic status logging */
    if(pool.total_steps % 1000 == 0
    || (pool.in_drain_phase && pool.drain_step_count % 500 == 0
     && pool.active_count > 0)) {
      log_info(scn->dev,
        "persistent_wavefront step %lu: %lu/%lu active, "
        "%lu rays this step (rad=%lu ds=%lu shd=%lu st=%lu), "
        "%lu/%lu tasks done, %s\n",
        (unsigned long)pool.total_steps,
        (unsigned long)pool.active_count,
        (unsigned long)pool.pool_size,
        (unsigned long)pool.ray_count,
        (unsigned long)pool.bucket_counts[RAY_BUCKET_RADIATIVE],
        (unsigned long)pool.bucket_counts[RAY_BUCKET_STEP_PAIR],
        (unsigned long)pool.bucket_counts[RAY_BUCKET_SHADOW],
        (unsigned long)pool.bucket_counts[RAY_BUCKET_STARTUP],
        (unsigned long)(pool.paths_completed + pool.paths_failed),
        (unsigned long)total_tasks,
        pool.in_drain_phase ? "DRAIN" : "refill");
    }

    /* Safety: prevent infinite loops */
    if(pool.total_steps > total_tasks * 1000) {
      log_err(scn->dev,
        "persistent_wavefront: exceeded maximum step count (%lu). "
        "%lu paths still active.\n",
        (unsigned long)pool.total_steps,
        (unsigned long)pool.active_count);
      res = RES_BAD_OP;
      goto cleanup;
    }
  }

  /* ====== 8. Summary logging ====== */
  time_current(&t_end);
  time_sub(&t_elapsed, &t_end, &t_start);

  /* Record drain phase wall-clock (total - refill phase) */
  if(pool.in_drain_phase) {
    double total_wall = time_elapsed_sec(&t_start, &t_end);
    pool.time_drain_phase_s = total_wall - pool.time_refill_phase_s;
  }

  {
    double avg_width = 0;
    char time_str[128];
    time_dump(&t_elapsed, TIME_ALL, NULL, time_str, sizeof(time_str));
    if(pool.total_steps > 0)
      avg_width = (double)pool.diag_total_active / (double)pool.total_steps;
    log_info(scn->dev,
      "persistent_wavefront DONE: %lux%lu spp=%lu pool=%lu "
      "elapsed=%s  steps=%lu  rays=%lu  avg_width=%.1f  "
      "(rad=%lu cond_ds=%lu ds_retry=%lu)  "
      "done: rad=%lu temp=%lu bnd=%lu fail=%lu  "
      "max_depth=%lu\n",
      (unsigned long)image_def[0], (unsigned long)image_def[1],
      (unsigned long)spp, (unsigned long)pool.pool_size,
      time_str,
      (unsigned long)pool.total_steps,
      (unsigned long)pool.total_rays_traced,
      avg_width,
      (unsigned long)pool.rays_radiative,
      (unsigned long)pool.rays_conductive_ds,
      (unsigned long)pool.rays_conductive_ds_retry,
      (unsigned long)pool.paths_done_radiative,
      (unsigned long)pool.paths_done_temperature,
      (unsigned long)pool.paths_done_boundary,
      (unsigned long)pool.paths_failed,
      (unsigned long)pool.max_path_depth);
  }

  log_drain_phase_report(scn->dev, &pool);

cleanup:
  /* Flush and close pixel trace file */
  if(s_pixel_trace_fp) { fclose(s_pixel_trace_fp); s_pixel_trace_fp = NULL; s_pixel_trace_checked = 0; }

  pool_destroy(&pool);
  return res;
}
#endif /* !SDIS_SOLVE_PERSISTENT_WAVEFRONT_SKIP_PUBLIC_API */
