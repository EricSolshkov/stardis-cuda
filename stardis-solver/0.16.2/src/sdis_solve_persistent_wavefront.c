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

#include <float.h>
#include <stdlib.h>
#include <string.h>

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

  return RES_OK;
}

static void
pool_destroy(struct wavefront_pool* pool)
{
  if(!pool) return;

  if(pool->batch_ctx) s3d_batch_trace_context_destroy(pool->batch_ctx);
  if(pool->enc_batch_ctx) s3d_batch_enc_context_destroy(pool->enc_batch_ctx);

  /* Release per-slot RNGs */
  if(pool->slot_rngs) {
    size_t i;
    for(i = 0; i < pool->pool_size; i++) {
      if(pool->slot_rngs[i]) SSP(rng_ref_put(pool->slot_rngs[i]));
    }
    free(pool->slot_rngs);
  }

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
  uint32_t path_id)
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

  /* Use per-slot RNG */
  p->rng = rng;

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
      pool->next_path_id++);
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

    if(p->phase == PATH_DONE || p->phase == PATH_ERROR) {
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

        p->needs_ray = 0;
        res = advance_one_step_with_ray(p, scn, h0, h1);
        if(res != RES_OK && res != RES_BAD_OP
        && res != RES_BAD_OP_IRRECOVERABLE) return res;
        if(res == RES_BAD_OP || res == RES_BAD_OP_IRRECOVERABLE) {
          mark_path_failed(p, pool);
          res = RES_OK;
        }
        p->steps_taken++;
      }
    }
  }

  return RES_OK;
}

/*******************************************************************************
 * Cascade non-ray steps -- compact version (M2.5)
 *
 * For each active path, advance through non-ray phases until a ray is
 * needed, the path completes, or no progress can be made.
 ******************************************************************************/
static res_T
pool_cascade_non_ray_steps_compact(struct wavefront_pool* pool,
                                   struct sdis_scene* scn)
{
  size_t k;
  res_T res = RES_OK;

  for(k = 0; k < pool->active_compact; k++) {
    uint32_t i = pool->active_indices[k];
    struct path_state* p = &pool->slots[i];
    if(!p->active) continue;

    for(;;) {
      int advanced = 0;
      if(p->needs_ray) break;
      if(p->phase == PATH_DONE || p->phase == PATH_ERROR) {
        /* M8: intercept PATH_DONE when sfn_stack_depth > 0.
         * The sub-path finished — resume the parent picardN frame. */
        if(p->phase == PATH_DONE && p->sfn_stack_depth > 0) {
          p->phase = PATH_BND_SFN_COMPUTE_Ti_RESUME;
          p->active = 1;
          continue;
        }
        break;
      }
      if(path_phase_is_ray_pending(p->phase)) break;
      /* M10: also break on enc_locate-pending phases */
      if(path_phase_is_enc_locate_pending(p->phase)) break;

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
      p->steps_taken++;
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

    /* Skip if already harvested (active==0 AND phase reset) */
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

    /* Mark slot as harvestable for refill.
     * We set active=0 (done by step functions) and clear phase so that
     * refill_pool can identify available slots. */
    p->active = 0;
    /* phase stays PATH_DONE until refill overwrites with a new task */
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
    if(p->phase != PATH_DONE && p->phase != PATH_ERROR) continue;
    if(pool->task_next >= pool->task_count) break;

    {
      const struct pixel_task* task = &pool->task_queue[pool->task_next];

      res = init_single_path(
        p, pool->slot_rngs[i], task,
        pool->scn, pool->enc_id, pool->cam,
        pool->time_range, pool->pix_sz,
        pool->picard_order, pool->diff_algo,
        pool->next_path_id++);
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

  /* ====== 1. Allocate pool ====== */
  res = pool_create(&pool, pool_size);
  if(res != RES_OK) {
    log_err(scn->dev,
      "persistent_wavefront: pool_create failed for %lu paths -- %s\n",
      (unsigned long)pool_size, res_to_cstr(res));
    goto cleanup;
  }

  /* ====== 2. Assign per-slot RNGs (round-robin from per_thread_rng) ====== */
  for(i = 0; i < pool.pool_size; i++) {
    pool.slot_rngs[i] = per_thread_rng[i % nthreads];
    SSP(rng_ref_get(pool.slot_rngs[i]));
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
  pool_destroy(&pool);
  return res;
}
#endif /* !SDIS_SOLVE_PERSISTENT_WAVEFRONT_SKIP_PUBLIC_API */
