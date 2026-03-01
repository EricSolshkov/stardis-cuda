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
 * P2: pool_view — Unified pool view init / destroy / merge / split
 *
 * A pool_view represents a view over slots[base..base+view_size).
 * Single-buffer: views[0] covers the entire pool.
 * Dual-buffer:   views[0] front half, views[1] back half.
 *
 * All pool operation functions will be unified to accept (pool, pv) in P3.
 * Until then, the old pool-level fields remain and are used by existing code.
 ******************************************************************************/

/**
 * pool_view_init — Allocate and initialise a single pool view.
 *
 * @param pv         Target view struct (caller-owned, e.g. &pool->views[i])
 * @param base       Slot start offset (0 or view_size)
 * @param view_size  Number of slots this view covers
 * @param capacity   Allocation capacity (>= view_size; views[0] may use
 *                   pool_size to support merge)
 */
static res_T
pool_view_init(struct pool_view* pv, size_t base, size_t view_size,
               size_t capacity)
{
  size_t max_rays = capacity * 6;  /* each slot can request up to 6 rays */

  memset(pv, 0, sizeof(*pv));
  pv->base      = base;
  pv->view_size = view_size;
  pv->capacity  = capacity;
  pv->max_rays  = max_rays;

  /* Index arrays — allocated by capacity */
  pv->active_indices   = (uint32_t*)calloc(capacity, sizeof(uint32_t));
  pv->need_ray_indices = (uint32_t*)calloc(capacity, sizeof(uint32_t));
  pv->done_indices     = (uint32_t*)calloc(capacity, sizeof(uint32_t));
  pv->bucket_radiative = (uint32_t*)calloc(capacity, sizeof(uint32_t));
  pv->bucket_conductive= (uint32_t*)calloc(capacity, sizeof(uint32_t));

  if(!pv->active_indices || !pv->need_ray_indices || !pv->done_indices
  || !pv->bucket_radiative || !pv->bucket_conductive)
    return RES_MEM_ERR;

  /* Ray buffers — allocated by max_rays */
  pv->ray_requests = (struct s3d_ray_request*)calloc(
    max_rays, sizeof(struct s3d_ray_request));
  pv->ray_to_slot  = (uint32_t*)calloc(max_rays, sizeof(uint32_t));
  pv->ray_slot_sub = (uint32_t*)calloc(max_rays, sizeof(uint32_t));
  pv->ray_hits     = (struct s3d_hit*)calloc(max_rays, sizeof(struct s3d_hit));

  if(!pv->ray_requests || !pv->ray_to_slot
  || !pv->ray_slot_sub || !pv->ray_hits)
    return RES_MEM_ERR;

  /* GPU batch trace context (will hold independent CUDA stream + params) */
  {
    res_T rc = s3d_batch_trace_context_create(&pv->batch_ctx, max_rays);
    if(rc != RES_OK) return rc;
  }

  /* enc_locate buffers */
  pv->max_enc_locates     = capacity;
  pv->enc_locate_requests = (struct s3d_enc_locate_request*)calloc(
    capacity, sizeof(struct s3d_enc_locate_request));
  pv->enc_locate_results  = (struct s3d_enc_locate_result*)calloc(
    capacity, sizeof(struct s3d_enc_locate_result));
  pv->enc_locate_to_slot  = (uint32_t*)calloc(capacity, sizeof(uint32_t));

  if(!pv->enc_locate_requests || !pv->enc_locate_results
  || !pv->enc_locate_to_slot)
    return RES_MEM_ERR;

  {
    res_T rc = s3d_batch_enc_context_create(&pv->enc_batch_ctx, capacity);
    if(rc != RES_OK) return rc;
  }

  /* closest_point buffers */
  pv->max_cps     = capacity;
  pv->cp_requests = (struct s3d_cp_request*)calloc(
    capacity, sizeof(struct s3d_cp_request));
  pv->cp_hits     = (struct s3d_hit*)calloc(
    capacity, sizeof(struct s3d_hit));
  pv->cp_to_slot  = (uint32_t*)calloc(capacity, sizeof(uint32_t));

  if(!pv->cp_requests || !pv->cp_hits || !pv->cp_to_slot)
    return RES_MEM_ERR;

  {
    res_T rc = s3d_batch_cp_context_create(&pv->cp_batch_ctx, capacity);
    if(rc != RES_OK) return rc;
  }

  return RES_OK;
}

/**
 * pool_view_destroy — Free all resources held by a pool view.
 */
static void
pool_view_destroy(struct pool_view* pv)
{
  if(!pv) return;

  free(pv->active_indices);
  free(pv->need_ray_indices);
  free(pv->done_indices);
  free(pv->bucket_radiative);
  free(pv->bucket_conductive);

  free(pv->ray_requests);
  free(pv->ray_to_slot);
  free(pv->ray_slot_sub);
  free(pv->ray_hits);

  if(pv->batch_ctx) s3d_batch_trace_context_destroy(pv->batch_ctx);

  free(pv->enc_locate_requests);
  free(pv->enc_locate_results);
  free(pv->enc_locate_to_slot);
  if(pv->enc_batch_ctx) s3d_batch_enc_context_destroy(pv->enc_batch_ctx);

  free(pv->cp_requests);
  free(pv->cp_hits);
  free(pv->cp_to_slot);
  if(pv->cp_batch_ctx) s3d_batch_cp_context_destroy(pv->cp_batch_ctx);

  memset(pv, 0, sizeof(*pv));
}

/*******************************************************************************
 * P2: Dynamic merge / split — dual-buffer <-> single-buffer transitions
 ******************************************************************************/

/**
 * should_merge — Check if dual-buffer should collapse to single-buffer.
 *
 * Condition: either half's active count drops below 12.5% of its view_size.
 */
static int
should_merge(const struct wavefront_pool* pool)
{
  size_t thresh;
  if(pool->num_active_views != 2) return 0;

  thresh = pool->views[0].view_size / 8;     /* 12.5% */

  return (pool->views[0].active_compact < thresh)
      || (pool->views[1].active_compact < thresh);
}

/**
 * should_split — Check if single-buffer should split to dual-buffer.
 *
 * Condition: active paths > 75% pool capacity, and undistributed tasks remain.
 */
static int
should_split(const struct wavefront_pool* pool)
{
  if(pool->num_active_views != 1) return 0;
  if(pool->task_next >= pool->task_count) return 0;

  return pool->views[0].active_compact > (pool->pool_size * 3 / 4);
}

/**
 * merge_to_single_pool — Collapse dual-buffer views into a single full-range view.
 *
 * Precondition: both views' GPU traces have completed (no async pending).
 * views[0].capacity was allocated as pool_size, so no reallocation needed.
 * views[1] resources are NOT freed (may split back later).
 */
static void
merge_to_single_pool(struct wavefront_pool* pool)
{
  /* Expand views[0] to cover the full pool */
  pool->views[0].base      = 0;
  pool->views[0].view_size = pool->pool_size;

  /* Clear runtime counts — next compact pass will rebuild them */
  pool->views[0].active_compact     = 0;
  pool->views[0].need_ray_count     = 0;
  pool->views[0].done_count         = 0;
  pool->views[0].bucket_radiative_n = 0;
  pool->views[0].bucket_conductive_n= 0;
  pool->views[0].ray_count          = 0;
  pool->views[0].enc_locate_count   = 0;
  pool->views[0].cp_count           = 0;

  pool->num_active_views = 1;

  /* views[1] resources retained for potential future split */
}

/**
 * split_to_dual_pool — Split single-buffer view into two half-range views.
 *
 * Precondition: views[1] was previously initialised by pool_view_init
 * (only valid if initially started in dual mode; single-mode start does
 * not initialise views[1], and should_split returns 0 in that case).
 *
 * Must be called before compact, so the next compact pass works on
 * the new view ranges.
 */
static void
split_to_dual_pool(struct wavefront_pool* pool)
{
  size_t half = pool->pool_size / 2;

  pool->views[0].base      = 0;
  pool->views[0].view_size = half;

  pool->views[1].base      = half;
  pool->views[1].view_size = half;

  /* Clear both views' runtime counts */
  pool->views[0].active_compact     = 0;
  pool->views[0].need_ray_count     = 0;
  pool->views[0].done_count         = 0;
  pool->views[0].bucket_radiative_n = 0;
  pool->views[0].bucket_conductive_n= 0;
  pool->views[0].ray_count          = 0;
  pool->views[0].enc_locate_count   = 0;
  pool->views[0].cp_count           = 0;

  pool->views[1].active_compact     = 0;
  pool->views[1].need_ray_count     = 0;
  pool->views[1].done_count         = 0;
  pool->views[1].bucket_radiative_n = 0;
  pool->views[1].bucket_conductive_n= 0;
  pool->views[1].ray_count          = 0;
  pool->views[1].enc_locate_count   = 0;
  pool->views[1].cp_count           = 0;

  pool->num_active_views = 2;
}

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

  /* B-4 M9: closest_point (WoS) batch buffers */
  pool->max_cps = pool_size;
  pool->cp_requests = (struct s3d_cp_request*)malloc(
    pool_size * sizeof(struct s3d_cp_request));
  pool->cp_hits = (struct s3d_hit*)malloc(
    pool_size * sizeof(struct s3d_hit));
  pool->cp_to_slot = (uint32_t*)malloc(
    pool_size * sizeof(uint32_t));

  if(!pool->cp_requests || !pool->cp_hits || !pool->cp_to_slot) {
    return RES_MEM_ERR;
  }

  /* P1: Dispatch-layer SoA arrays */
  {
    res_T soa_res = dispatch_soa_alloc(&pool->dsoa, pool_size);
    if(soa_res != RES_OK) return soa_res;
  }

  /* P1: Cold-block SoA arrays (separated from path_state slots) */
  pool->sfn_arr = (struct path_sfn_data*)calloc(pool_size, sizeof(struct path_sfn_data));
  pool->enc_arr = (struct path_enc_data*)calloc(pool_size, sizeof(struct path_enc_data));
  pool->ext_arr = (struct path_ext_data*)calloc(pool_size, sizeof(struct path_ext_data));
  if(!pool->sfn_arr || !pool->enc_arr || !pool->ext_arr) return RES_MEM_ERR;

  /* Initialise diagnostics */
  pool->diag_min_batch = (size_t)-1; /* updated on first step with rays */
  pool->trace_batch_size_min = (size_t)-1;

  /* P2: Initialise unified pool views */
  {
    const char* env = getenv("STARDIS_PIPELINE");
    int dual = (env && env[0] == '0') ? 0 : 1;
    res_T vrc;

    if(dual) {
      /* Dual-buffer pipeline mode */
      size_t half = pool_size / 2;

      /* views[0]: capacity = pool_size (supports merge to full range)
       *           initial view_size = half (front half only) */
      vrc = pool_view_init(&pool->views[0], 0, half, pool_size);
      if(vrc != RES_OK) return vrc;

      /* views[1]: capacity = half (back half only) */
      vrc = pool_view_init(&pool->views[1], half, half, half);
      if(vrc != RES_OK) return vrc;

      pool->num_active_views = 2;
    } else {
      /* Single-buffer mode: views[0] covers entire pool */
      vrc = pool_view_init(&pool->views[0], 0, pool_size, pool_size);
      if(vrc != RES_OK) return vrc;

      pool->num_active_views = 1;
    }
  }

  return RES_OK;
}

static void
pool_destroy(struct wavefront_pool* pool)
{
  if(!pool) return;

  /* P2: Destroy pool views (GPU batch contexts + view buffers) */
  pool_view_destroy(&pool->views[0]);
  pool_view_destroy(&pool->views[1]);  /* safe even if not initialised (zeroed) */

  if(pool->batch_ctx) s3d_batch_trace_context_destroy(pool->batch_ctx);
  if(pool->enc_batch_ctx) s3d_batch_enc_context_destroy(pool->enc_batch_ctx);
  if(pool->cp_batch_ctx) s3d_batch_cp_context_destroy(pool->cp_batch_ctx);

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

  /* B-4 M9: closest_point (WoS) arrays */
  free(pool->cp_requests);
  free(pool->cp_hits);
  free(pool->cp_to_slot);

  /* P1: Dispatch-layer SoA arrays */
  dispatch_soa_free(&pool->dsoa);

  /* P1: Cold-block SoA arrays */
  free(pool->sfn_arr);
  free(pool->enc_arr);
  free(pool->ext_arr);

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
                          struct wavefront_pool* pool,
                          size_t slot_idx)
{
  res_T res = RES_OK;
  while(p->active && !p->needs_ray && p->phase != PATH_DONE
      && p->phase != PATH_ERROR) {
    int advanced = 0;
    if(path_phase_is_ray_pending(p->phase)) break;

    res = advance_one_step_no_ray(p, scn, &advanced, pool, slot_idx);
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
    if(p->phase == PATH_DONE && pool->sfn_arr[slot_idx].depth > 0) {
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
    res = advance_path_to_first_ray(&pool->slots[i], pool->scn, pool, i);
    if(res != RES_OK) return res;

    /* P1 SYNC POINT C: sync new path to SoA */
    dispatch_soa_sync_from_path(&pool->dsoa, (uint32_t)i, &pool->slots[i]);

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
 * Stream Compaction (M2.5 + P1 SoA) -- rebuild compact index arrays each step.
 *
 * P1: reads phase/active/needs_ray from dsoa (SoA) instead of path_state
 * (AoS), reducing cache footprint from ~4KB/path to 12B/path for the
 * common case (needs_ray==0).
 *
 * After this function:
 *   active_indices[0..active_compact-1]   = indices of active, non-done slots
 *   need_ray_indices[0..need_ray_count-1] = active slots needing ray trace
 *   done_indices[0..done_count-1]         = slots with PATH_DONE
 *   bucket_radiative[0..n-1]              = need_ray + radiative phase
 *   bucket_conductive[0..n-1]             = need_ray + conductive phase
 ******************************************************************************/
static void
compact_active_paths(struct wavefront_pool* pool, struct pool_view* pv)
{
  size_t base = pv->base;
  size_t end  = base + pv->view_size;
  size_t i;
  struct dispatch_soa* dsoa = &pool->dsoa;

  pv->active_compact      = 0;
  pv->need_ray_count      = 0;
  pv->done_count          = 0;
  pv->bucket_radiative_n  = 0;
  pv->bucket_conductive_n = 0;

  for(i = base; i < end; i++) {
    enum path_phase ph = dsoa->phase[i];
    int act = dsoa->active[i];
    int nr  = dsoa->needs_ray[i];

    if(ph == PATH_DONE || ph == PATH_ERROR
    || ph == PATH_HARVESTED) {
      /* M8: if sfn_stack_depth > 0, the sub-path finished but the
       * parent picardN frame still needs to resume.  Re-activate.
       * Need to access AoS for sfn_stack_depth (rare path). */
      if(ph == PATH_DONE && pool->sfn_arr[i].depth > 0) {
        pool->slots[i].phase = PATH_BND_SFN_COMPUTE_Ti_RESUME;
        pool->slots[i].active = 1;
        dsoa->phase[i] = PATH_BND_SFN_COMPUTE_Ti_RESUME;
        dsoa->active[i] = 1;
        ph = PATH_BND_SFN_COMPUTE_Ti_RESUME;
        act = 1;
        /* Fall through to treat as active path */
      } else {
        pv->done_indices[pv->done_count++] = (uint32_t)i;
        continue;
      }
    }
    if(!act) continue;

    pv->active_indices[pv->active_compact++] = (uint32_t)i;

    if(nr) {
      /* Only touch AoS when needs_ray is set (minority of paths).
       * Still need ray_req.ray_count from AoS (P2 scope). */
      struct path_state* p = &pool->slots[i];
      if(p->ray_req.ray_count > 0) {
        pv->need_ray_indices[pv->need_ray_count++] = (uint32_t)i;

        /* Bucket by phase type */
        if(ph == PATH_RAD_TRACE_PENDING) {
          pv->bucket_radiative[pv->bucket_radiative_n++] = (uint32_t)i;
        } else if(ph == PATH_COUPLED_COND_DS_PENDING
               || ph == PATH_CND_DS_STEP_TRACE) {
          pv->bucket_conductive[pv->bucket_conductive_n++] = (uint32_t)i;
        }
        /* Other ray-pending phases (boundary reinject) go into
         * need_ray_indices but not into type-specific buckets. */
      }
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

    /* Detailed stats by phase — semantic layer + per-phase layer.
     * NOTE: compact collect lacks SoA; inline count_path_rays logic. */
    {
      size_t nrays = (p->phase == PATH_ENC_QUERY_EMIT
                   && p->ray_req.ray_count >= 2) ? 6
                   : (size_t)p->ray_req.ray_count;
      switch(p->phase) {
      case PATH_RAD_TRACE_PENDING:
      case PATH_BND_SF_NULLCOLL_RAD_TRACE:
      case PATH_BND_SFN_RAD_TRACE:
      case PATH_BND_EXT_DIFFUSE_TRACE:
      case PATH_CND_WOS_FALLBACK_TRACE:
      case PATH_COUPLED_BOUNDARY_REINJECT:
      case PATH_BND_SS_REINJECT_SAMPLE:
      case PATH_BND_SF_REINJECT_SAMPLE:
        pool->rays_radiative += nrays; break;
      case PATH_COUPLED_COND_DS_PENDING:
      case PATH_CND_DS_STEP_TRACE:
        if(p->ds_robust_attempt > 0) pool->rays_conductive_ds_retry += nrays;
        else                         pool->rays_conductive_ds += nrays;
        break;
      case PATH_BND_EXT_DIRECT_TRACE:
      case PATH_BND_EXT_DIFFUSE_SHADOW_TRACE:
        pool->rays_shadow += nrays; break;
      case PATH_ENC_QUERY_EMIT:
      case PATH_ENC_QUERY_FB_EMIT:
      case PATH_CND_INIT_ENC:
        pool->rays_enclosure += nrays; break;
      case PATH_CNV_STARTUP_TRACE:
        pool->rays_startup += nrays; break;
      default:
        pool->rays_other += nrays; break;
      }
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
pool_collect_ray_requests_bucketed(struct wavefront_pool* pool,
                                    struct pool_view* pv)
{
  size_t b;
  int omp_nthreads;
  int use_omp = 1;

  /* Max OMP threads for stack-allocated per-thread arrays */
  #define COLLECT_MAX_THREADS 32

  /* Zero per-view pending stats (promoted to pool after trace completes) */
  pv->pending_rays_radiative           = 0;
  pv->pending_rays_conductive_ds       = 0;
  pv->pending_rays_conductive_ds_retry = 0;
  pv->pending_rays_shadow              = 0;
  pv->pending_rays_enclosure           = 0;
  pv->pending_rays_startup             = 0;
  pv->pending_rays_other               = 0;

  /* OMP toggle: STARDIS_COLLECT_OMP=0 disables */
  {
    const char* env = getenv("STARDIS_COLLECT_OMP");
    if(env && env[0] == '0') use_omp = 0;
  }
  omp_nthreads = (pool->scn && pool->scn->dev)
               ? (int)pool->scn->dev->nthreads : 1;
  if(omp_nthreads < 1) omp_nthreads = 1;
  if(omp_nthreads > COLLECT_MAX_THREADS) omp_nthreads = COLLECT_MAX_THREADS;
  if(pv->need_ray_count < 128) use_omp = 0;

  if(use_omp) {
    /* ════════════ OMP 3-pass bucketed collect (zero atomic) ════════════
     *
     * Pass 1: Per-thread bucket counting.  Each thread counts rays per
     *         bucket for its chunk.  Same schedule(static) as Pass 2 so
     *         thread t processes identical index range in both passes.
     *
     * Prefix sum: Compute global bucket_offsets, then per-thread per-bucket
     *             write offsets.  Thread t's rays for bucket b start at
     *             tl_write_base[t][b].  No overlap -> zero contention.
     *
     * Pass 2: Scatter.  Each thread writes rays using a local cursor
     *         initialised from tl_write_base.  No atomic, no sharing.
     * ================================================================== */

    /* Per-thread per-bucket ray counts (padded to avoid false sharing) */
    size_t tl_ray_counts[COLLECT_MAX_THREADS][RAY_BUCKET_COUNT];
    size_t tl_write_base[COLLECT_MAX_THREADS][RAY_BUCKET_COUNT];

    memset(tl_ray_counts, 0, sizeof(tl_ray_counts));

    /* ---- Pass 1: Per-thread bucket counting (P1: from SoA) ---- */
    #pragma omp parallel num_threads(omp_nthreads)
    {
      int tid = omp_get_thread_num();
      int kk;

      #pragma omp for schedule(static)
      for(kk = 0; kk < (int)pv->need_ray_count; kk++) {
        uint32_t i = pv->need_ray_indices[kk];
        struct path_state* p = &pool->slots[i];
        size_t nrays = count_path_rays_soa(
          pool->dsoa.phase[i], pool->dsoa.ray_count_ext[i], p);
        int bkt = (int)pool->dsoa.ray_bucket[i];
        tl_ray_counts[tid][bkt] += nrays;
      }
    }

    /* ---- Aggregate bucket totals ---- */
    memset(pv->bucket_counts, 0, sizeof(pv->bucket_counts));
    {
      int t;
      for(t = 0; t < omp_nthreads; t++) {
        for(b = 0; b < RAY_BUCKET_COUNT; b++) {
          pv->bucket_counts[b] += tl_ray_counts[t][b];
        }
      }
    }

    /* ---- Bucket offsets (global prefix sum) ---- */
    pv->bucket_offsets[0] = 0;
    for(b = 0; b < RAY_BUCKET_COUNT; b++) {
      pv->bucket_offsets[b + 1] = pv->bucket_offsets[b]
                                 + pv->bucket_counts[b];
    }

    /* ---- Per-thread per-bucket write bases ---- */
    for(b = 0; b < RAY_BUCKET_COUNT; b++) {
      size_t running = pv->bucket_offsets[b];
      int t;
      for(t = 0; t < omp_nthreads; t++) {
        tl_write_base[t][b] = running;
        running += tl_ray_counts[t][b];
      }
    }

    /* ---- Pass 2: Scatter rays (no atomic, each thread owns its region) ---- */
    #pragma omp parallel num_threads(omp_nthreads)
    {
      int tid = omp_get_thread_num();
      int kk, bb;
      size_t my_cursor[RAY_BUCKET_COUNT];
      size_t tl_rays_radiative = 0;
      size_t tl_rays_cond_ds = 0;
      size_t tl_rays_cond_ds_retry = 0;
      size_t tl_rays_shadow = 0;
      size_t tl_rays_enclosure = 0;
      size_t tl_rays_startup = 0;
      size_t tl_rays_other = 0;

      for(bb = 0; bb < RAY_BUCKET_COUNT; bb++)
        my_cursor[bb] = tl_write_base[tid][bb];

      /* MUST use identical schedule(static) as Pass 1 */
      #pragma omp for schedule(static)
      for(kk = 0; kk < (int)pv->need_ray_count; kk++) {
        uint32_t i = pv->need_ray_indices[kk];
        struct path_state* p = &pool->slots[i];
        int bkt = (int)pool->dsoa.ray_bucket[i];  /* P1: from SoA */
        enum path_phase ph_i = pool->dsoa.phase[i]; /* P1: from SoA */

        /* Thread-local stats accumulation — semantic + per-phase */
        {
          size_t nrays = count_path_rays_soa(
            ph_i, pool->dsoa.ray_count_ext[i], p);
          switch(ph_i) {
          case PATH_RAD_TRACE_PENDING:
          case PATH_BND_SF_NULLCOLL_RAD_TRACE:
          case PATH_BND_SFN_RAD_TRACE:
          case PATH_BND_EXT_DIFFUSE_TRACE:
          case PATH_CND_WOS_FALLBACK_TRACE:
          case PATH_COUPLED_BOUNDARY_REINJECT:
          case PATH_BND_SS_REINJECT_SAMPLE:
          case PATH_BND_SF_REINJECT_SAMPLE:
            tl_rays_radiative += nrays; break;
          case PATH_COUPLED_COND_DS_PENDING:
          case PATH_CND_DS_STEP_TRACE:
            if(p->ds_robust_attempt > 0) tl_rays_cond_ds_retry += nrays;
            else                         tl_rays_cond_ds += nrays;
            break;
          case PATH_BND_EXT_DIRECT_TRACE:
          case PATH_BND_EXT_DIFFUSE_SHADOW_TRACE:
            tl_rays_shadow += nrays; break;
          case PATH_ENC_QUERY_EMIT:
          case PATH_ENC_QUERY_FB_EMIT:
          case PATH_CND_INIT_ENC:
            tl_rays_enclosure += nrays; break;
          case PATH_CNV_STARTUP_TRACE:
            tl_rays_startup += nrays; break;
          default:
            tl_rays_other += nrays; break;
          }
        }

        /* --- Emit ray 0 --- */
        {
          size_t ray_idx = my_cursor[bkt]++;
          struct s3d_ray_request* rr = &pv->ray_requests[ray_idx];
          rr->origin[0]    = p->ray_req.origin[0];
          rr->origin[1]    = p->ray_req.origin[1];
          rr->origin[2]    = p->ray_req.origin[2];
          rr->direction[0] = p->ray_req.direction[0];
          rr->direction[1] = p->ray_req.direction[1];
          rr->direction[2] = p->ray_req.direction[2];
          rr->range[0]     = p->ray_req.range[0];
          rr->range[1]     = p->ray_req.range[1];
          rr->user_id      = i;

          if(!S3D_HIT_NONE(&p->filter_data_storage.hit_3d))
            rr->filter_data = &p->filter_data_storage;
          else
            rr->filter_data = NULL;

          pv->ray_to_slot[ray_idx] = i;
          pv->ray_slot_sub[ray_idx] = 0;
          p->ray_req.batch_idx = (uint32_t)ray_idx;
        }

        /* --- Ray 1 (2-ray request) --- */
        if(p->ray_req.ray_count >= 2) {
          size_t ray_idx = my_cursor[bkt]++;
          struct s3d_ray_request* rr = &pv->ray_requests[ray_idx];
          rr->origin[0]    = p->ray_req.origin[0];
          rr->origin[1]    = p->ray_req.origin[1];
          rr->origin[2]    = p->ray_req.origin[2];
          rr->direction[0] = p->ray_req.direction2[0];
          rr->direction[1] = p->ray_req.direction2[1];
          rr->direction[2] = p->ray_req.direction2[2];
          rr->range[0]     = p->ray_req.range2[0];
          rr->range[1]     = p->ray_req.range2[1];
          if(!S3D_HIT_NONE(&p->filter_data_storage.hit_3d))
            rr->filter_data = &p->filter_data_storage;
          else
            rr->filter_data = NULL;
          rr->user_id      = i;

          pv->ray_to_slot[ray_idx] = i;
          pv->ray_slot_sub[ray_idx] = 1;
          p->ray_req.batch_idx2 = (uint32_t)ray_idx;
        }

        /* --- Rays 2..5 for 6-ray enclosure query (P1: phase from SoA) --- */
        if(pool->dsoa.ray_count_ext[i] == 6
        && ph_i == PATH_ENC_QUERY_EMIT) {
          int j;
          pool->enc_arr[i].batch_indices[0] = p->ray_req.batch_idx;
          pool->enc_arr[i].batch_indices[1] = p->ray_req.batch_idx2;

          for(j = 2; j < 6; j++) {
            size_t ray_idx = my_cursor[bkt]++;
            struct s3d_ray_request* rr = &pv->ray_requests[ray_idx];
            rr->origin[0]    = p->ray_req.origin[0];
            rr->origin[1]    = p->ray_req.origin[1];
            rr->origin[2]    = p->ray_req.origin[2];
            rr->direction[0] = pool->enc_arr[i].directions[j][0];
            rr->direction[1] = pool->enc_arr[i].directions[j][1];
            rr->direction[2] = pool->enc_arr[i].directions[j][2];
            rr->range[0]     = p->ray_req.range[0];
            rr->range[1]     = p->ray_req.range[1];
            rr->filter_data  = NULL;
            rr->user_id      = i;

            pv->ray_to_slot[ray_idx] = i;
            pv->ray_slot_sub[ray_idx] = (uint32_t)j;
            pool->enc_arr[i].batch_indices[j] = (uint32_t)ray_idx;
          }
        }
      } /* end omp for */

      #pragma omp critical
      {
        pv->pending_rays_radiative           += tl_rays_radiative;
        pv->pending_rays_conductive_ds       += tl_rays_cond_ds;
        pv->pending_rays_conductive_ds_retry += tl_rays_cond_ds_retry;
        pv->pending_rays_shadow              += tl_rays_shadow;
        pv->pending_rays_enclosure           += tl_rays_enclosure;
        pv->pending_rays_startup             += tl_rays_startup;
        pv->pending_rays_other               += tl_rays_other;
      }
    } /* end omp parallel */

    pv->ray_count = pv->bucket_offsets[RAY_BUCKET_COUNT];
    return RES_OK;

    #undef COLLECT_MAX_THREADS
  }

  /* ════════════ Serial fallback ════════════ */
  {
    size_t k;
    size_t ser_cursor[RAY_BUCKET_COUNT];

    memset(pv->bucket_counts, 0, sizeof(pv->bucket_counts));

    /* P1: Serial fallback Pass 1 — read bucket + ray_count from SoA */
    for(k = 0; k < pv->need_ray_count; k++) {
      uint32_t i = pv->need_ray_indices[k];
      struct path_state* p = &pool->slots[i];
      size_t nrays = count_path_rays_soa(
        pool->dsoa.phase[i], pool->dsoa.ray_count_ext[i], p);
      int bkt = (int)pool->dsoa.ray_bucket[i];
      ASSERT(bkt >= 0 && bkt < RAY_BUCKET_COUNT);
      pv->bucket_counts[bkt] += nrays;
    }

    pv->bucket_offsets[0] = 0;
    for(b = 0; b < RAY_BUCKET_COUNT; b++) {
      pv->bucket_offsets[b + 1] = pv->bucket_offsets[b]
                                 + pv->bucket_counts[b];
    }
    for(b = 0; b < RAY_BUCKET_COUNT; b++) {
      ser_cursor[b] = pv->bucket_offsets[b];
    }

    for(k = 0; k < pv->need_ray_count; k++) {
      uint32_t i = pv->need_ray_indices[k];
      struct path_state* p = &pool->slots[i];
      int bkt = (int)pool->dsoa.ray_bucket[i];   /* P1: from SoA */
      enum path_phase ph_i = pool->dsoa.phase[i]; /* P1: from SoA */

      /* Semantic + per-phase stats (written to pv->pending_*,
       * promoted to pool after trace completes) */
      {
        size_t nrays = count_path_rays_soa(
          ph_i, pool->dsoa.ray_count_ext[i], p);
        switch(ph_i) {
        case PATH_RAD_TRACE_PENDING:
        case PATH_BND_SF_NULLCOLL_RAD_TRACE:
        case PATH_BND_SFN_RAD_TRACE:
        case PATH_BND_EXT_DIFFUSE_TRACE:
        case PATH_CND_WOS_FALLBACK_TRACE:
        case PATH_COUPLED_BOUNDARY_REINJECT:
        case PATH_BND_SS_REINJECT_SAMPLE:
        case PATH_BND_SF_REINJECT_SAMPLE:
          pv->pending_rays_radiative += nrays; break;
        case PATH_COUPLED_COND_DS_PENDING:
        case PATH_CND_DS_STEP_TRACE:
          if(p->ds_robust_attempt > 0) pv->pending_rays_conductive_ds_retry += nrays;
          else                         pv->pending_rays_conductive_ds += nrays;
          break;
        case PATH_BND_EXT_DIRECT_TRACE:
        case PATH_BND_EXT_DIFFUSE_SHADOW_TRACE:
          pv->pending_rays_shadow += nrays; break;
        case PATH_ENC_QUERY_EMIT:
        case PATH_ENC_QUERY_FB_EMIT:
        case PATH_CND_INIT_ENC:
          pv->pending_rays_enclosure += nrays; break;
        case PATH_CNV_STARTUP_TRACE:
          pv->pending_rays_startup += nrays; break;
        default:
          pv->pending_rays_other += nrays; break;
        }
      }

      {
        size_t ray_idx = ser_cursor[bkt]++;
        struct s3d_ray_request* rr = &pv->ray_requests[ray_idx];
        rr->origin[0]    = p->ray_req.origin[0];
        rr->origin[1]    = p->ray_req.origin[1];
        rr->origin[2]    = p->ray_req.origin[2];
        rr->direction[0] = p->ray_req.direction[0];
        rr->direction[1] = p->ray_req.direction[1];
        rr->direction[2] = p->ray_req.direction[2];
        rr->range[0]     = p->ray_req.range[0];
        rr->range[1]     = p->ray_req.range[1];
        rr->user_id      = i;

        if(!S3D_HIT_NONE(&p->filter_data_storage.hit_3d))
          rr->filter_data = &p->filter_data_storage;
        else
          rr->filter_data = NULL;

        pv->ray_to_slot[ray_idx] = i;
        pv->ray_slot_sub[ray_idx] = 0;
        p->ray_req.batch_idx = (uint32_t)ray_idx;
      }

      if(p->ray_req.ray_count >= 2) {
        size_t ray_idx = ser_cursor[bkt]++;
        struct s3d_ray_request* rr = &pv->ray_requests[ray_idx];
        rr->origin[0]    = p->ray_req.origin[0];
        rr->origin[1]    = p->ray_req.origin[1];
        rr->origin[2]    = p->ray_req.origin[2];
        rr->direction[0] = p->ray_req.direction2[0];
        rr->direction[1] = p->ray_req.direction2[1];
        rr->direction[2] = p->ray_req.direction2[2];
        rr->range[0]     = p->ray_req.range2[0];
        rr->range[1]     = p->ray_req.range2[1];
        if(!S3D_HIT_NONE(&p->filter_data_storage.hit_3d))
          rr->filter_data = &p->filter_data_storage;
        else
          rr->filter_data = NULL;
        rr->user_id      = i;

        pv->ray_to_slot[ray_idx] = i;
        pv->ray_slot_sub[ray_idx] = 1;
        p->ray_req.batch_idx2 = (uint32_t)ray_idx;
      }

      /* P1: read phase + ray_count_ext from SoA */
      if(pool->dsoa.ray_count_ext[i] == 6
      && ph_i == PATH_ENC_QUERY_EMIT) {
        int j;
        pool->enc_arr[i].batch_indices[0] = p->ray_req.batch_idx;
        pool->enc_arr[i].batch_indices[1] = p->ray_req.batch_idx2;

        for(j = 2; j < 6; j++) {
          size_t ray_idx = ser_cursor[bkt]++;
          struct s3d_ray_request* rr = &pv->ray_requests[ray_idx];
          rr->origin[0]    = p->ray_req.origin[0];
          rr->origin[1]    = p->ray_req.origin[1];
          rr->origin[2]    = p->ray_req.origin[2];
          rr->direction[0] = pool->enc_arr[i].directions[j][0];
          rr->direction[1] = pool->enc_arr[i].directions[j][1];
          rr->direction[2] = pool->enc_arr[i].directions[j][2];
          rr->range[0]     = p->ray_req.range[0];
          rr->range[1]     = p->ray_req.range[1];
          rr->filter_data  = NULL;
          rr->user_id      = i;

          pv->ray_to_slot[ray_idx] = i;
          pv->ray_slot_sub[ray_idx] = (uint32_t)j;
          pool->enc_arr[i].batch_indices[j] = (uint32_t)ray_idx;
        }
      }
    }

    pv->ray_count = pv->bucket_offsets[RAY_BUCKET_COUNT];
    return RES_OK;
  }
}

/*******************************************************************************
 * B-4 M10: Collect enc_locate requests from paths in PATH_ENC_LOCATE_PENDING
 *
 * Scans active paths for PATH_ENC_LOCATE_PENDING and fills the
 * enc_locate_requests array.  The requests are then dispatched as a
 * single GPU batch via s3d_scene_view_find_enclosure_batch_ctx.
 ******************************************************************************/
LOCAL_SYM res_T
pool_collect_enc_locate_requests(struct wavefront_pool* pool,
                                  struct pool_view* pv)
{
  size_t k, n = 0;

  for(k = 0; k < pv->active_compact; k++) {
    uint32_t i = pv->active_indices[k];
    struct path_state* p = &pool->slots[i];

    if(p->phase == PATH_ENC_LOCATE_PENDING) {
      struct s3d_enc_locate_request* req = &pv->enc_locate_requests[n];
      req->pos[0] = (float)pool->enc_arr[i].locate.query_pos[0];
      req->pos[1] = (float)pool->enc_arr[i].locate.query_pos[1];
      req->pos[2] = (float)pool->enc_arr[i].locate.query_pos[2];
      req->user_id = i;
      pv->enc_locate_to_slot[n] = i;
      pool->enc_arr[i].locate.batch_idx = (uint32_t)n;
      n++;
    }
  }

  pv->enc_locate_count = n;
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
pool_distribute_enc_locate_results(struct wavefront_pool* pool,
                                    struct pool_view* pv)
{
  size_t k;

  for(k = 0; k < pv->enc_locate_count; k++) {
    uint32_t slot_id = pv->enc_locate_to_slot[k];
    struct path_state* p = &pool->slots[slot_id];
    const struct s3d_enc_locate_result* r = &pv->enc_locate_results[k];

    pool->enc_arr[slot_id].locate.prim_id  = r->prim_id;
    pool->enc_arr[slot_id].locate.side     = r->side;
    pool->enc_arr[slot_id].locate.distance = r->distance;
    p->phase = PATH_ENC_LOCATE_RESULT;
  }

  return RES_OK;
}

/*******************************************************************************
 * B-4 M9: Collect closest_point requests from WoS paths in PATH_CND_WOS_CLOSEST
 *
 * Scans active paths for PATH_CND_WOS_CLOSEST and fills the cp_requests
 * array.  The requests are then dispatched as a single GPU batch via
 * s3d_scene_view_closest_point_batch_ctx.
 ******************************************************************************/
LOCAL_SYM res_T
pool_collect_cp_requests(struct wavefront_pool* pool,
                          struct pool_view* pv)
{
  size_t k, n = 0;

  for(k = 0; k < pv->active_compact; k++) {
    uint32_t i = pv->active_indices[k];
    struct path_state* p = &pool->slots[i];

    if(p->phase == PATH_CND_WOS_CLOSEST) {
      struct s3d_cp_request* req = &pv->cp_requests[n];
      req->pos[0] = (float)p->locals.cnd_wos.query_pos[0];
      req->pos[1] = (float)p->locals.cnd_wos.query_pos[1];
      req->pos[2] = (float)p->locals.cnd_wos.query_pos[2];
      req->radius = p->locals.cnd_wos.query_radius;
      req->query_data = NULL; /* no filter */
      req->user_id = i;
      pv->cp_to_slot[n] = i;
      p->locals.cnd_wos.batch_cp_idx = (uint32_t)n;
      n++;
    }
  }

  pv->cp_count = n;
  return RES_OK;
}

/*******************************************************************************
 * B-4 M9: Distribute closest_point results back to WoS paths
 *
 * For each CP result, write the hit into the path's cached_hit field,
 * then transition to PATH_CND_WOS_CLOSEST_RESULT so the cascade loop
 * can process the result via step_cnd_wos_closest_result.
 ******************************************************************************/
LOCAL_SYM res_T
pool_distribute_cp_results(struct wavefront_pool* pool,
                            struct pool_view* pv)
{
  size_t k;

  for(k = 0; k < pv->cp_count; k++) {
    uint32_t slot_id = pv->cp_to_slot[k];
    struct path_state* p = &pool->slots[slot_id];

    p->locals.cnd_wos.cached_hit = pv->cp_hits[k];
    p->phase = PATH_CND_WOS_CLOSEST_RESULT;
  }

  return RES_OK;
}

static res_T
pool_distribute_ray_results(struct wavefront_pool* pool,
                             struct pool_view* pv,
                             struct sdis_scene* scn)
{
  size_t bucket_other_n = 0;
  int omp_nthreads;
  int use_omp = 1;
  int had_fatal = 0;

  /* OMP toggle: STARDIS_DISTRIBUTE_OMP=0 disables */
  {
    const char* env = getenv("STARDIS_DISTRIBUTE_OMP");
    if(env && env[0] == '0') use_omp = 0;
  }
  omp_nthreads = (scn && scn->dev) ? (int)scn->dev->nthreads : 1;
  if(omp_nthreads < 1) omp_nthreads = 1;
  if(pv->bucket_radiative_n + pv->bucket_conductive_n < 128) use_omp = 0;

  if(use_omp) {
    /* ════════════ OMP parallel distribute ════════════ */

    /* ---- Phase 1: Radiative paths (1 ray each) ---- */
    #pragma omp parallel num_threads(omp_nthreads)
    {
      int kk;
      size_t tl_paths_failed = 0;
      int tl_fatal = 0;

      #pragma omp for schedule(static)
      for(kk = 0; kk < (int)pv->bucket_radiative_n; kk++) {
        uint32_t i = pv->bucket_radiative[kk];
        struct path_state* p = &pool->slots[i];
        const struct s3d_hit* h0 = &pv->ray_hits[p->ray_req.batch_idx];
        res_T lr;

        p->needs_ray = 0;
        lr = step_radiative_trace(p, scn, h0);
        if(lr != RES_OK && lr != RES_BAD_OP
        && lr != RES_BAD_OP_IRRECOVERABLE) { tl_fatal = 1; }
        else if(lr == RES_BAD_OP || lr == RES_BAD_OP_IRRECOVERABLE) {
          p->phase = PATH_DONE; p->active = 0; p->done_reason = -1;
          tl_paths_failed++;
        }
        p->steps_taken++;
      }

      #pragma omp critical
      {
        pool->paths_failed += tl_paths_failed;
        if(tl_fatal) had_fatal = 1;
      }
    } /* end omp parallel Phase 1 */
    if(had_fatal) return RES_UNKNOWN_ERR;

    /* ---- Phase 2: Conductive / delta-sphere paths (2 rays each) ---- */
    had_fatal = 0;
    #pragma omp parallel num_threads(omp_nthreads)
    {
      int kk;
      size_t tl_paths_failed = 0;
      int tl_fatal = 0;

      #pragma omp for schedule(static)
      for(kk = 0; kk < (int)pv->bucket_conductive_n; kk++) {
        uint32_t i = pv->bucket_conductive[kk];
        struct path_state* p = &pool->slots[i];
        const struct s3d_hit* h0 = &pv->ray_hits[p->ray_req.batch_idx];
        const struct s3d_hit* h1 = NULL;
        res_T lr;
        if(p->ray_req.ray_count >= 2)
          h1 = &pv->ray_hits[p->ray_req.batch_idx2];

        p->needs_ray = 0;
        lr = step_conductive_ds_process(p, scn, h0, h1,
                                         &pool->enc_arr[i]);
        if(lr != RES_OK && lr != RES_BAD_OP
        && lr != RES_BAD_OP_IRRECOVERABLE) { tl_fatal = 1; }
        else if(lr == RES_BAD_OP || lr == RES_BAD_OP_IRRECOVERABLE) {
          p->phase = PATH_DONE; p->active = 0; p->done_reason = -1;
          tl_paths_failed++;
        }
        p->steps_taken++;
      }

      #pragma omp critical
      {
        pool->paths_failed += tl_paths_failed;
        if(tl_fatal) had_fatal = 1;
      }
    } /* end omp parallel Phase 2 */
    if(had_fatal) return RES_UNKNOWN_ERR;

    /* ---- Phase 3: Fallback — unbucketed ray paths ---- */
    bucket_other_n = pv->need_ray_count
                   - pv->bucket_radiative_n
                   - pv->bucket_conductive_n;
    if(bucket_other_n > 0) {
      had_fatal = 0;
      #pragma omp parallel num_threads(omp_nthreads)
      {
        int kk;
        size_t tl_paths_failed = 0;
        size_t tl_enc_escalated = 0;
        int tl_fatal = 0;

        #pragma omp for schedule(static)
        for(kk = 0; kk < (int)pv->need_ray_count; kk++) {
          uint32_t i = pv->need_ray_indices[kk];
          struct path_state* p = &pool->slots[i];

          if(p->phase == PATH_RAD_TRACE_PENDING) continue;
          if(p->phase == PATH_COUPLED_COND_DS_PENDING) continue;
          if(p->phase == PATH_CND_DS_STEP_TRACE) continue;
          if(!p->needs_ray) continue;

          {
            const struct s3d_hit* h0 = &pv->ray_hits[p->ray_req.batch_idx];
            const struct s3d_hit* h1 = NULL;
            res_T lr;
            if(p->ray_req.ray_count >= 2)
              h1 = &pv->ray_hits[p->ray_req.batch_idx2];

            if(p->phase == PATH_ENC_QUERY_EMIT && p->ray_count_ext == 6) {
              int j;
              for(j = 0; j < 6; j++) {
                pool->enc_arr[i].dir_hits[j] =
                  pv->ray_hits[pool->enc_arr[i].batch_indices[j]];
              }
            }
            if(p->phase == PATH_ENC_QUERY_FB_EMIT) {
              pool->enc_arr[i].fb_hit = pv->ray_hits[p->ray_req.batch_idx];
            }

            {
              enum path_phase phase_before = p->phase;
              p->needs_ray = 0;
              lr = advance_one_step_with_ray(p, scn, h0, h1,
                                             pool, (size_t)i);
              if(lr != RES_OK && lr != RES_BAD_OP
              && lr != RES_BAD_OP_IRRECOVERABLE) { tl_fatal = 1; }
              else if(lr == RES_BAD_OP || lr == RES_BAD_OP_IRRECOVERABLE) {
                p->phase = PATH_DONE; p->active = 0; p->done_reason = -1;
                tl_paths_failed++;
              }
              if(phase_before == PATH_ENC_QUERY_FB_EMIT
              && p->phase == PATH_ENC_LOCATE_PENDING) {
                tl_enc_escalated++;
              }
              p->steps_taken++;
            }
          }
        }

        #pragma omp critical
        {
          pool->paths_failed += tl_paths_failed;
          pool->enc_query_escalated_to_m10 += tl_enc_escalated;
          if(tl_fatal) had_fatal = 1;
        }
      } /* end omp parallel Phase 3 */
      if(had_fatal) return RES_UNKNOWN_ERR;
    }

    return RES_OK;
  }

  /* ════════════ Serial fallback ════════════ */
  {
    size_t k;
    res_T res = RES_OK;

    /* ---- Phase 1: Radiative paths ---- */
    for(k = 0; k < pv->bucket_radiative_n; k++) {
      uint32_t i = pv->bucket_radiative[k];
      struct path_state* p = &pool->slots[i];
      const struct s3d_hit* h0 = &pv->ray_hits[p->ray_req.batch_idx];

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

    /* ---- Phase 2: Conductive / delta-sphere paths ---- */
    for(k = 0; k < pv->bucket_conductive_n; k++) {
      uint32_t i = pv->bucket_conductive[k];
      struct path_state* p = &pool->slots[i];
      const struct s3d_hit* h0 = &pv->ray_hits[p->ray_req.batch_idx];
      const struct s3d_hit* h1 = NULL;
      if(p->ray_req.ray_count >= 2)
        h1 = &pv->ray_hits[p->ray_req.batch_idx2];

      p->needs_ray = 0;
      res = step_conductive_ds_process(p, scn, h0, h1,
                                       &pool->enc_arr[i]);
      if(res != RES_OK && res != RES_BAD_OP
      && res != RES_BAD_OP_IRRECOVERABLE) return res;
      if(res == RES_BAD_OP || res == RES_BAD_OP_IRRECOVERABLE) {
        mark_path_failed(p, pool);
        res = RES_OK;
      }
      p->steps_taken++;
    }

    /* ---- Phase 3: Fallback — unbucketed ray paths ---- */
    bucket_other_n = pv->need_ray_count
                   - pv->bucket_radiative_n
                   - pv->bucket_conductive_n;
    if(bucket_other_n > 0) {
      for(k = 0; k < pv->need_ray_count; k++) {
        uint32_t i = pv->need_ray_indices[k];
        struct path_state* p = &pool->slots[i];

        if(p->phase == PATH_RAD_TRACE_PENDING) continue;
        if(p->phase == PATH_COUPLED_COND_DS_PENDING) continue;
        if(p->phase == PATH_CND_DS_STEP_TRACE) continue;
        if(!p->needs_ray) continue;

        {
          const struct s3d_hit* h0 = &pv->ray_hits[p->ray_req.batch_idx];
          const struct s3d_hit* h1 = NULL;
          if(p->ray_req.ray_count >= 2)
            h1 = &pv->ray_hits[p->ray_req.batch_idx2];

          if(p->phase == PATH_ENC_QUERY_EMIT && p->ray_count_ext == 6) {
            int j;
            for(j = 0; j < 6; j++) {
              pool->enc_arr[i].dir_hits[j] =
                pv->ray_hits[pool->enc_arr[i].batch_indices[j]];
            }
          }
          if(p->phase == PATH_ENC_QUERY_FB_EMIT) {
            pool->enc_arr[i].fb_hit = pv->ray_hits[p->ray_req.batch_idx];
          }

          {
            enum path_phase phase_before = p->phase;
            p->needs_ray = 0;
            res = advance_one_step_with_ray(p, scn, h0, h1,
                                            pool, (size_t)i);
            if(res != RES_OK && res != RES_BAD_OP
            && res != RES_BAD_OP_IRRECOVERABLE) return res;
            if(res == RES_BAD_OP || res == RES_BAD_OP_IRRECOVERABLE) {
              mark_path_failed(p, pool);
              res = RES_OK;
            }
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
  struct wavefront_pool* pool, size_t slot_idx,
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
      if(p->phase == PATH_DONE && pool->sfn_arr[slot_idx].depth > 0) {
        p->phase = PATH_BND_SFN_COMPUTE_Ti_RESUME;
        p->active = 1;
        continue;
      }
      break;
    }
    if(path_phase_is_ray_pending(p->phase)) break;
    if(path_phase_is_enc_locate_pending(p->phase)) break;
    if(path_phase_is_cp_pending(p->phase)) break;

    (*local_iterations)++;
    phase_before = p->phase;
    time_current(&t_step0);

    res = advance_one_step_no_ray(p, scn, &advanced, pool, slot_idx);

    time_current(&t_step1);
    if((int)phase_before >= 0 && (int)phase_before < PATH_PHASE_COUNT) {
      local_phase_count[(int)phase_before]++;
      local_phase_time[(int)phase_before]
        += time_elapsed_sec(&t_step0, &t_step1);
    }
    if(phase_before == PATH_ENC_LOCATE_RESULT
    && pool->enc_arr[slot_idx].locate.prim_id >= 0 && pool->enc_arr[slot_idx].locate.side < 0) {
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
                                   struct pool_view* pv,
                                   struct sdis_scene* scn)
{
  const size_t n = pv->active_compact;
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
        uint32_t idx = pv->active_indices[ph];
        struct path_state* p = &pool->slots[idx];
        if(!p->active) continue;

        if(cascade_advance_single_path(
              p, scn, pool, (size_t)idx,
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
      uint32_t idx = pv->active_indices[k];
      struct path_state* p = &pool->slots[idx];
      if(!p->active) continue;

      if(cascade_advance_single_path(
            p, scn, pool, (size_t)idx,
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
  struct pool_view* pv,
  struct sdis_estimator_buffer* buf)
{
  size_t k;

  ASSERT(pool && buf);

  for(k = 0; k < pv->done_count; k++) {
    uint32_t i = pv->done_indices[k];
    struct path_state* p = &pool->slots[i];
    enum path_phase ph = pool->dsoa.phase[i]; /* P1: from SoA */

    /* Skip if already harvested */
    if(ph == PATH_HARVESTED)
      continue;
    if(!pool->dsoa.active[i] && ph != PATH_DONE && ph != PATH_ERROR)
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
        fprintf(tf, "%u,%u,%u,%u,%.17g,%d,%d,%llu,%d\n",
          (unsigned)p->path_id,
          (unsigned)p->ipix_image[0], (unsigned)p->ipix_image[1],
          (unsigned)p->realisation_idx,
          p->T.value, (int)p->T.done, p->done_reason,
          (unsigned long long)p->steps_taken, (int)p->phase);
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
    /* P1: sync harvest state to SoA */
    pool->dsoa.active[i] = 0;
    pool->dsoa.phase[i]  = PATH_HARVESTED;
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
refill_pool(struct wavefront_pool* pool, struct pool_view* pv,
            size_t* out_refill_count)
{
  size_t k, refill_count = 0;
  res_T res = RES_OK;

  if(pool->task_next >= pool->task_count) {
    *out_refill_count = 0;
    return RES_OK;
  }

  for(k = 0; k < pv->done_count; k++) {
    uint32_t i = pv->done_indices[k];
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
      res = advance_path_to_first_ray(p, pool->scn, pool, i);
      if(res != RES_OK) return res;

      /* P1 SYNC POINT C: sync new path to SoA */
      dispatch_soa_sync_from_path(&pool->dsoa, i, p);

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
  if(pool->num_active_views == 1) {
    pool->active_count = pool->views[0].active_compact;
  } else {
    pool->active_count = pool->views[0].active_compact
                       + pool->views[1].active_compact;
  }
}

/*******************************************************************************
 * Update diagnostics -- track batch size statistics + wavefront width (M3)
 ******************************************************************************/
static void
pool_update_diagnostics(struct wavefront_pool* pool, struct pool_view* pv)
{
  if(pv->ray_count > 0) {
    if(pv->ray_count < pool->diag_min_batch)
      pool->diag_min_batch = pv->ray_count;
    if(pv->ray_count > pool->diag_max_batch)
      pool->diag_max_batch = pv->ray_count;
  }

  if(pool->in_drain_phase) {
    pool->diag_drain_rays += pv->ray_count;
    pool->drain_step_count++;
  } else {
    pool->diag_refill_rays += pv->ray_count;
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
    "  total_steps=%llu  total_rays=%llu  avg_wavefront_width=%.1f\n"
    "  refill_phase: rays=%llu (%.1f%%)  wall=%.3fs\n"
    "  drain_phase:  rays=%llu (%.1f%%)  steps=%llu  wall=%.3fs\n"
    "  batch_size: min=%llu, max=%llu\n"
    "  rays: radiative=%llu  cond_ds=%llu(retry=%llu)  "
    "shadow=%llu  enclosure=%llu  startup=%llu  other=%llu\n"
    "  paths: completed=%llu, failed=%llu, truncated=%llu, max_depth=%llu\n"
    "  refills=%llu\n"
    "  timing: compact=%.3fs  collect=%.3fs  trace=%.3fs  "
    "distribute=%.3fs  cascade=%.3fs  harvest+refill=%.3fs\n",
    (unsigned long long)pool->total_steps,
    (unsigned long long)pool->total_rays_traced,
    avg_width,
    (unsigned long long)pool->diag_refill_rays,
    100.0 - drain_ray_pct,
    pool->time_refill_phase_s,
    (unsigned long long)pool->diag_drain_rays,
    drain_ray_pct,
    (unsigned long long)pool->drain_step_count,
    pool->time_drain_phase_s,
    (unsigned long long)(pool->diag_min_batch == (size_t)-1
                    ? 0 : pool->diag_min_batch),
    (unsigned long long)pool->diag_max_batch,
    (unsigned long long)pool->rays_radiative,
    (unsigned long long)pool->rays_conductive_ds,
    (unsigned long long)pool->rays_conductive_ds_retry,
    (unsigned long long)pool->rays_shadow,
    (unsigned long long)pool->rays_enclosure,
    (unsigned long long)pool->rays_startup,
    (unsigned long long)pool->rays_other,
    (unsigned long long)pool->paths_completed,
    (unsigned long long)pool->paths_failed,
    (unsigned long long)pool->paths_truncated,
    (unsigned long long)pool->max_path_depth,
    (unsigned long long)pool->diag_refill_count,
    pool->time_compact_s,
    pool->time_collect_s,
    pool->time_trace_s,
    pool->time_distribute_s,
    pool->time_cascade_s,
    pool->time_harvest_s);

  /* Verify ray stats sum == total_rays_traced */
  {
    size_t sum = pool->rays_radiative
               + pool->rays_conductive_ds
               + pool->rays_conductive_ds_retry
               + pool->rays_shadow
               + pool->rays_enclosure
               + pool->rays_startup
               + pool->rays_other;
    if(sum != pool->total_rays_traced) {
      log_info(dev,
        "  WARNING: ray stats sum=%llu != total_rays_traced=%llu "
        "(delta=%lld)\n",
        (unsigned long long)sum,
        (unsigned long long)pool->total_rays_traced,
        (long long)((long long)sum - (long long)pool->total_rays_traced));
    }
    ASSERT(sum == pool->total_rays_traced);
  }

  /* Enclosure query escalation stats */
  if(pool->enc_query_escalated_to_m10 > 0
  || pool->enc_locate_degenerate_null > 0) {
    log_info(dev,
      "  enc_escalation: query_fb->m10=%llu  m10_degenerate_null=%llu\n",
      (unsigned long long)pool->enc_query_escalated_to_m10,
      (unsigned long long)pool->enc_locate_degenerate_null);
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
      "cascade profiling: total_iterations=%llu  total_advances=%llu\n",
      (unsigned long long)pool->cascade_total_iterations,
      (unsigned long long)pool->cascade_total_advances);

    for(ti = 0; ti < 10; ti++) {
      int ph = top[ti];
      if(ph < 0 || pool->cascade_phase_count[ph] == 0) break;
      log_info(dev,
        "  cascade phase[%2d]: count=%10llu  time=%8.3fs  avg=%.3fus  "
        "(%.1f%% of cascade)\n",
        ph,
        (unsigned long long)pool->cascade_phase_count[ph],
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
      "batch trace profiling: calls=%llu  avg_batch=%.0f  "
      "min=%llu  max=%llu\n"
      "  gpu_kernel+upload: total=%.1fms (%.1f%%)  avg=%.2fms/call\n"
      "  cpu_postprocess:   total=%.1fms (%.1f%%)  avg=%.2fms/call\n"
      "  fallback_retrace:  total=%.1fms (%.1f%%)  "
      "accepted=%llu  missed=%llu  rejected=%llu\n"
      "  gpu_throughput: %.1f Mrays/s  (kernel+upload only)\n",
      (unsigned long long)pool->trace_call_count,
      avg_batch,
      (unsigned long long)(pool->trace_batch_size_min == (size_t)-1
                      ? 0 : pool->trace_batch_size_min),
      (unsigned long long)pool->trace_batch_size_max,
      pool->trace_batch_time_ms_sum, gpu_pct, avg_batch_ms,
      pool->trace_post_time_ms_sum, post_pct, avg_post_ms,
      pool->trace_retrace_time_ms_sum, rtrc_pct,
      (unsigned long long)pool->trace_retrace_accepted_sum,
      (unsigned long long)pool->trace_retrace_missed_sum,
      (unsigned long long)pool->trace_filter_rejected_sum,
      throughput);
  }
}

/*******************************************************************************
 * P4: Pipeline helper functions
 *
 * Four stage functions that wrap the existing P3 pool operations into
 * pipeline-friendly units.  Used by both single-pool and dual-pool loops.
 *
 *   cpu_pre_gpu             — compact + collect (prepare rays for GPU)
 *   gpu_launch_async        — launch GPU trace asynchronously
 *   gpu_wait_and_postprocess — wait GPU + distribute + enc/cp batches
 *   cpu_between             — cascade + harvest + refill
 ******************************************************************************/

#define VIEW_TAG(pv, pool) ((pv) == &(pool)->views[0] ? "[A]" : "[B]")

/**
 * cpu_pre_gpu — compact active paths + collect ray requests.
 */
static res_T
cpu_pre_gpu(struct wavefront_pool* pool, struct pool_view* pv)
{
  struct time t0, t1;
  res_T res;

  time_current(&t0);
  compact_active_paths(pool, pv);
  time_current(&t1);
  pool->time_compact_s += time_elapsed_sec(&t0, &t1);

  time_current(&t0);
  pv->ray_count = 0;
  res = pool_collect_ray_requests_bucketed(pool, pv);
  if(res != RES_OK) return res;
  time_current(&t1);
  pool->time_collect_s += time_elapsed_sec(&t0, &t1);

  return RES_OK;
}

/**
 * gpu_launch_async — launch GPU trace asynchronously.
 * Sets pv->gpu_pending = 1 on success, 0 if no rays.
 */
static res_T
gpu_launch_async(struct wavefront_pool* pool, struct pool_view* pv,
                 struct s3d_scene_view* sv)
{
  res_T res;
  (void)pool;

  if(pv->ray_count == 0) {
    pv->gpu_pending = 0;
    return RES_OK;
  }

  res = s3d_scene_view_trace_rays_batch_ctx_async(
    sv, pv->batch_ctx, pv->ray_requests, pv->ray_count);
  if(res != RES_OK) return res;
  pv->gpu_pending = 1;
  return RES_OK;
}

/**
 * gpu_wait_and_postprocess — wait for async GPU trace, then distribute
 * ray results, enc_locate, and closest_point batches.
 */
static res_T
gpu_wait_and_postprocess(struct wavefront_pool* pool,
                         struct pool_view* pv,
                         struct s3d_scene_view* sv,
                         struct sdis_scene* scn)
{
  struct time t0, t1;
  res_T res;

  /* ---- batch trace wait + postprocess ---- */
  time_current(&t0);
  if(pv->ray_count > 0 && pv->gpu_pending) {
    struct s3d_batch_trace_stats stats;
    memset(&stats, 0, sizeof(stats));

    res = s3d_scene_view_trace_rays_batch_ctx_wait(
      sv, pv->batch_ctx,
      pv->ray_requests, pv->ray_count,
      pv->ray_hits, &stats);
    if(res != RES_OK) return res;
    pv->gpu_pending = 0;

    pool->total_rays_traced += pv->ray_count;

    /* Promote per-view pending ray stats to pool-level counters.
     * Stats were accumulated during collect; they are only committed
     * here after the GPU trace actually completes, so untrace'd views
     * (e.g. last pipeline iteration that exits before launch) never
     * pollute the pool totals. */
    {
      pool->rays_radiative           += pv->pending_rays_radiative;
      pool->rays_conductive_ds       += pv->pending_rays_conductive_ds;
      pool->rays_conductive_ds_retry += pv->pending_rays_conductive_ds_retry;
      pool->rays_shadow              += pv->pending_rays_shadow;
      pool->rays_enclosure           += pv->pending_rays_enclosure;
      pool->rays_startup             += pv->pending_rays_startup;
      pool->rays_other               += pv->pending_rays_other;
    }

    pool->trace_call_count++;
    pool->trace_batch_size_sum   += pv->ray_count;
    pool->trace_batch_time_ms_sum  += stats.batch_time_ms;
    pool->trace_post_time_ms_sum   += stats.postprocess_time_ms;
    pool->trace_retrace_time_ms_sum += stats.retrace_time_ms;
    pool->trace_retrace_accepted_sum += stats.retrace_accepted;
    pool->trace_retrace_missed_sum   += stats.retrace_missed;
    pool->trace_filter_rejected_sum  += stats.filter_rejected;

    if(pv->ray_count < pool->trace_batch_size_min)
      pool->trace_batch_size_min = pv->ray_count;
    if(pv->ray_count > pool->trace_batch_size_max)
      pool->trace_batch_size_max = pv->ray_count;
  }
  time_current(&t1);
  pool->time_trace_s += time_elapsed_sec(&t0, &t1);

  /* ---- distribute ray results ---- */
  time_current(&t0);
  res = pool_distribute_ray_results(pool, pv, scn);
  if(res != RES_OK) return res;
  time_current(&t1);
  pool->time_distribute_s += time_elapsed_sec(&t0, &t1);

  /* ---- enc_locate batch ---- */
  time_current(&t0);
  res = pool_collect_enc_locate_requests(pool, pv);
  if(res != RES_OK) return res;

  if(pv->enc_locate_count > 0) {
    struct s3d_batch_enc_stats enc_stats;
    memset(&enc_stats, 0, sizeof(enc_stats));

    res = s3d_scene_view_find_enclosure_batch_ctx(
      scn->s3d_view, pv->enc_batch_ctx,
      pv->enc_locate_requests, pv->enc_locate_count,
      pv->enc_locate_results, &enc_stats);
    if(res != RES_OK) return res;

    res = pool_distribute_enc_locate_results(pool, pv);
    if(res != RES_OK) return res;

    pool->enc_locates_total     += pv->enc_locate_count;
    pool->enc_locates_resolved  += enc_stats.resolved;
    pool->enc_locates_degenerate += enc_stats.degenerate;
  }
  time_current(&t1);
  pool->time_enc_locate_s += time_elapsed_sec(&t0, &t1);

  /* ---- closest_point batch ---- */
  time_current(&t0);
  res = pool_collect_cp_requests(pool, pv);
  if(res != RES_OK) return res;

  if(pv->cp_count > 0) {
    struct s3d_batch_cp_stats cp_stats;
    memset(&cp_stats, 0, sizeof(cp_stats));

    res = s3d_scene_view_closest_point_batch_ctx(
      scn->s3d_view, pv->cp_batch_ctx,
      pv->cp_requests, pv->cp_count,
      pv->cp_hits, &cp_stats);
    if(res != RES_OK) return res;

    res = pool_distribute_cp_results(pool, pv);
    if(res != RES_OK) return res;

    pool->cp_total     += pv->cp_count;
    pool->cp_accepted  += cp_stats.batch_accepted;
    pool->cp_requeried += cp_stats.requery_accepted;
  }
  time_current(&t1);
  pool->time_cp_s += time_elapsed_sec(&t0, &t1);

  /* ---- P1 SYNC POINT A: distribute finished — refresh dsoa for slots
   *      that received ray/enc_locate/cp results.  Sync all slots that
   *      had pending rays (need_ray_indices from the preceding compact). */
  {
    size_t k;
    for(k = 0; k < pv->need_ray_count; k++) {
      uint32_t idx = pv->need_ray_indices[k];
      dispatch_soa_sync_from_path(&pool->dsoa, idx, &pool->slots[idx]);
    }
    /* Also sync enc_locate slots (may have transitioned phase) */
    for(k = 0; k < pv->enc_locate_count; k++) {
      uint32_t idx = pv->enc_locate_to_slot[k];
      dispatch_soa_sync_from_path(&pool->dsoa, idx, &pool->slots[idx]);
    }
    /* Also sync closest_point slots */
    for(k = 0; k < pv->cp_count; k++) {
      uint32_t idx = pv->cp_to_slot[k];
      dispatch_soa_sync_from_path(&pool->dsoa, idx, &pool->slots[idx]);
    }
  }

  return RES_OK;
}

/**
 * cpu_between — cascade non-ray steps + harvest completed + refill pool.
 */
static res_T
cpu_between(struct wavefront_pool* pool, struct pool_view* pv,
            struct sdis_scene* scn,
            struct sdis_estimator_buffer* buf)
{
  size_t refill_count = 0;
  struct time t0, t1;
  res_T res;

  /* cascade non-ray steps */
  time_current(&t0);
  res = pool_cascade_non_ray_steps_compact(pool, pv, scn);
  if(res != RES_OK) return res;
  time_current(&t1);
  pool->time_cascade_s += time_elapsed_sec(&t0, &t1);

  /* P1 SYNC POINT B: cascade finished — refresh dsoa for all active slots.
   * cascade modifies phase/active/needs_ray in AoS; bulk-sync to SoA
   * before the upcoming compact_active_paths reads from dsoa. */
  {
    size_t k;
    for(k = 0; k < pv->active_compact; k++) {
      uint32_t idx = pv->active_indices[k];
      dispatch_soa_sync_from_path(&pool->dsoa, idx, &pool->slots[idx]);
    }
  }

  /* rebuild done_indices after cascade + harvest + refill */
  time_current(&t0);
  compact_active_paths(pool, pv);
  res = harvest_completed_paths(pool, pv, buf);
  if(res != RES_OK) return res;
  res = refill_pool(pool, pv, &refill_count);
  if(res != RES_OK) return res;
  time_current(&t1);
  pool->time_harvest_s += time_elapsed_sec(&t0, &t1);

  return RES_OK;
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
  res_T res = RES_OK;
  struct time t_start, t_end, t_elapsed;
  int last_pcent = 0;

  ASSERT(scn && per_thread_rng && nthreads > 0 && cam && spp);
  ASSERT(image_def && image_def[0] > 0 && image_def[1] > 0);
  ASSERT(pix_sz && pix_sz[0] > 0 && pix_sz[1] > 0 && time_range && buf);

  (void)register_paths; /* heat path tracking not yet supported */

  total_tasks = image_def[0] * image_def[1] * spp;
  pool_size = determine_pool_size(total_tasks, scn);

  /* P4: In dual-buffer pipeline mode each view sees pool_size/2 slots.
   * Double pool_size so that each half matches the single-buffer width.
   * STARDIS_POOL_SIZE env var still controls the *per-view* width. */
  {
    const char* env_pipe = getenv("STARDIS_PIPELINE");
    int dual = (env_pipe && env_pipe[0] == '0') ? 0 : 1;
    if(dual) {
      size_t doubled = pool_size * 2;
      if(doubled > total_tasks) doubled = total_tasks;
      /* Ensure even split */
      doubled = (doubled / 2) * 2;
      if(doubled < 2) doubled = 2;
      log_info(scn->dev,
        "pipeline: dual-buffer auto-double pool_size %llu -> %llu "
        "(per-view=%llu)\n",
        (unsigned long long)pool_size, (unsigned long long)doubled,
        (unsigned long long)(doubled / 2));
      pool_size = doubled;
    }
  }

  {
    int sm = (scn->dev && scn->dev->s3d_dev)
           ? s3d_device_get_gpu_sm_count(scn->dev->s3d_dev) : 0;
    log_info(scn->dev,
      "Persistent wavefront (Phase B-3 M3): %llux%llu spp=%llu, "
      "pool_size=%llu (SM=%d), total_tasks=%llu\n",
      (unsigned long long)image_def[0], (unsigned long long)image_def[1],
      (unsigned long long)spp, (unsigned long long)pool_size,
      sm, (unsigned long long)total_tasks);
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
      "persistent_wavefront: pool_create failed for %llu paths -- %s\n",
      (unsigned long long)pool_size, res_to_cstr(res));
    goto cleanup;
  }

  /* P2: Log pool view configuration */
  if(pool.num_active_views == 2) {
    log_info(scn->dev,
      "persistent_wavefront: dual-buffer pipeline, "
      "view_size=%llu, views[0].capacity=%llu, views[1].capacity=%llu\n",
      (unsigned long long)pool.views[0].view_size,
      (unsigned long long)pool.views[0].capacity,
      (unsigned long long)pool.views[1].capacity);
  } else {
    log_info(scn->dev,
      "persistent_wavefront: single-buffer mode, pool_size=%llu\n",
      (unsigned long long)pool.pool_size);
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
      "persistent_wavefront: batch_ctx creation failed for %llu rays -- %s\n",
      (unsigned long long)pool.max_rays, res_to_cstr(res));
    goto cleanup;
  }

  /* ====== 4b. Create batch enc_locate context (M10) ====== */
  res = s3d_batch_enc_context_create(&pool.enc_batch_ctx, pool.max_enc_locates);
  if(res != RES_OK) {
    log_err(scn->dev,
      "persistent_wavefront: enc_batch_ctx creation failed for %llu queries -- %s\n",
      (unsigned long long)pool.max_enc_locates, res_to_cstr(res));
    goto cleanup;
  }

  /* ====== 4c. Create batch closest_point context (M9: WoS) ====== */
  res = s3d_batch_cp_context_create(&pool.cp_batch_ctx, pool.max_cps);
  if(res != RES_OK) {
    log_err(scn->dev,
      "persistent_wavefront: cp_batch_ctx creation failed for %llu queries -- %s\n",
      (unsigned long long)pool.max_cps, res_to_cstr(res));
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
    "persistent_wavefront: %llu paths initialised (%llu/%llu tasks consumed), "
    "starting wavefront loop\n",
    (unsigned long long)pool.active_count,
    (unsigned long long)pool.task_next,
    (unsigned long long)pool.task_count);

  /* ====== 7. Wavefront main loop ====== */
  time_current(&t_start);

  /* ═══════════════════════════════════════════════════════ */
  /*  P4: Dual-buffer pipeline (num_active_views == 2)       */
  /* ═══════════════════════════════════════════════════════ */
  if(pool.num_active_views == 2) {
    struct pool_view* pv_a = &pool.views[0];
    struct pool_view* pv_b = &pool.views[1];

    log_info(scn->dev,
      "pipeline: starting dual-buffer mode, half=%llu\n",
      (unsigned long long)pv_a->view_size);

    /* ---- Startup: prepare A and launch first GPU ---- */
    res = cpu_pre_gpu(&pool, pv_a);
    if(res != RES_OK) goto cleanup;
    res = gpu_launch_async(&pool, pv_a, scn->s3d_view);
    if(res != RES_OK) goto cleanup;

    /* ---- Prepare B (first cycle: no cascade yet, just pre) ---- */
    res = cpu_pre_gpu(&pool, pv_b);
    if(res != RES_OK) goto cleanup;

    while(pool.active_count > 0 || pool.task_next < pool.task_count) {
      struct time t_pl0, t_pl1;
      /* P4 timeline: 7 timestamps spanning one full A+B cycle
       * [0]=start [1]=waitA [2]=launchB [3]=cpuA [4]=waitB [5]=launchA [6]=cpuB */
      struct time t_cy[7];

      /* ════════ Phase 1: wait GPU(A) → launch GPU(B) → CPU on A ════════ */
      time_current(&t_cy[0]);
      time_current(&t_pl0);
      res = gpu_wait_and_postprocess(&pool, pv_a, scn->s3d_view, scn);
      if(res != RES_OK) goto cleanup;
      time_current(&t_pl1);
      pool.time_pipeline_wait_s += time_elapsed_sec(&t_pl0, &t_pl1);
      t_cy[1] = t_pl1;  /* after wait(A) */

      res = gpu_launch_async(&pool, pv_b, scn->s3d_view);
      if(res != RES_OK) goto cleanup;
      time_current(&t_cy[2]);  /* after launch(B) */

      pool.total_steps++;  /* view A completed one step */

      time_current(&t_pl0);
      res = cpu_between(&pool, pv_a, scn, buf);
      if(res != RES_OK) goto cleanup;
      res = cpu_pre_gpu(&pool, pv_a);
      if(res != RES_OK) goto cleanup;
      time_current(&t_pl1);
      pool.time_pipeline_cpu_between_s += time_elapsed_sec(&t_pl0, &t_pl1);
      t_cy[3] = t_pl1;  /* after cpu(A) */

      /* -- Housekeeping after A's step -- */
      pool_update_active_count(&pool);

      if(!pool.in_drain_phase && pool.task_next >= pool.task_count) {
        struct time t_now;
        pool.in_drain_phase = 1;
        time_current(&t_now);
        pool.time_refill_phase_s = time_elapsed_sec(&t_start, &t_now);
        log_info(scn->dev,
          "persistent_wavefront: entering drain phase at step %llu, "
          "%llu active paths remain, refill_wall=%.3fs\n",
          (unsigned long long)pool.total_steps,
          (unsigned long long)pool.active_count,
          pool.time_refill_phase_s);
      }
      pool_update_diagnostics(&pool, pv_a);
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
      if(pool.total_steps % 1000 == 1
      || (pool.in_drain_phase && pool.drain_step_count % 500 == 0
       && pool.active_count > 0)) {
        log_info(scn->dev,
          "persistent_wavefront pipeline step %llu [A]: %llu/%llu active, "
          "%llu rays (rad=%llu ds=%llu shd=%llu enc=%llu st=%llu), "
          "%llu/%llu tasks done, %s\n",
          (unsigned long long)pool.total_steps,
          (unsigned long long)pool.active_count,
          (unsigned long long)pool.pool_size,
          (unsigned long long)pv_a->ray_count,
          (unsigned long long)pv_a->bucket_counts[RAY_BUCKET_RADIATIVE],
          (unsigned long long)pv_a->bucket_counts[RAY_BUCKET_STEP_PAIR],
          (unsigned long long)pv_a->bucket_counts[RAY_BUCKET_SHADOW],
          (unsigned long long)pv_a->bucket_counts[RAY_BUCKET_ENCLOSURE],
          (unsigned long long)pv_a->bucket_counts[RAY_BUCKET_STARTUP],
          (unsigned long long)(pool.paths_completed + pool.paths_failed),
          (unsigned long long)total_tasks,
          pool.in_drain_phase ? "DRAIN" : "refill");
      }
      { /* STARDIS_PIPELINE_LOG */
        const char* env_log = getenv("STARDIS_PIPELINE_LOG");
        if(env_log && env_log[0] == '1') {
          log_info(scn->dev,
            "[A] step %llu: rays=%llu, active=%llu\n",
            (unsigned long long)pool.total_steps,
            (unsigned long long)pv_a->ray_count,
            (unsigned long long)pv_a->active_compact);
        }
      }
      if(pool.total_steps > total_tasks * 1000) {
        log_err(scn->dev,
          "pipeline: infinite loop safety break at step %llu\n",
          (unsigned long long)pool.total_steps);
        res = RES_BAD_OP;
        goto cleanup;
      }

      /* ════════ Phase 2: wait GPU(B) → launch GPU(A) → CPU on B ════════ */
      time_current(&t_pl0);
      res = gpu_wait_and_postprocess(&pool, pv_b, scn->s3d_view, scn);
      if(res != RES_OK) goto cleanup;
      time_current(&t_pl1);
      pool.time_pipeline_wait_s += time_elapsed_sec(&t_pl0, &t_pl1);
      t_cy[4] = t_pl1;  /* after wait(B) */

      res = gpu_launch_async(&pool, pv_a, scn->s3d_view);
      if(res != RES_OK) goto cleanup;
      time_current(&t_cy[5]);  /* after launch(A) */

      pool.total_steps++;  /* view B completed one step */

      time_current(&t_pl0);
      res = cpu_between(&pool, pv_b, scn, buf);
      if(res != RES_OK) goto cleanup;
      res = cpu_pre_gpu(&pool, pv_b);
      if(res != RES_OK) goto cleanup;
      time_current(&t_pl1);
      pool.time_pipeline_cpu_between_s += time_elapsed_sec(&t_pl0, &t_pl1);
      t_cy[6] = t_pl1;  /* after cpu(B) */

      /* -- Housekeeping after B's step -- */
      pool_update_active_count(&pool);

      if(!pool.in_drain_phase && pool.task_next >= pool.task_count) {
        struct time t_now;
        pool.in_drain_phase = 1;
        time_current(&t_now);
        pool.time_refill_phase_s = time_elapsed_sec(&t_start, &t_now);
        log_info(scn->dev,
          "persistent_wavefront: entering drain phase at step %llu, "
          "%llu active paths remain, refill_wall=%.3fs\n",
          (unsigned long long)pool.total_steps,
          (unsigned long long)pool.active_count,
          pool.time_refill_phase_s);
      }
      pool_update_diagnostics(&pool, pv_b);
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
      if(pool.total_steps % 1000 == 0
      || (pool.in_drain_phase && pool.drain_step_count % 500 == 0
       && pool.active_count > 0)) {
        log_info(scn->dev,
          "persistent_wavefront pipeline step %llu [B]: %llu/%llu active, "
          "%llu rays (rad=%llu ds=%llu shd=%llu enc=%llu st=%llu), "
          "%llu/%llu tasks done, %s\n",
          (unsigned long long)pool.total_steps,
          (unsigned long long)pool.active_count,
          (unsigned long long)pool.pool_size,
          (unsigned long long)pv_b->ray_count,
          (unsigned long long)pv_b->bucket_counts[RAY_BUCKET_RADIATIVE],
          (unsigned long long)pv_b->bucket_counts[RAY_BUCKET_STEP_PAIR],
          (unsigned long long)pv_b->bucket_counts[RAY_BUCKET_SHADOW],
          (unsigned long long)pv_b->bucket_counts[RAY_BUCKET_ENCLOSURE],
          (unsigned long long)pv_b->bucket_counts[RAY_BUCKET_STARTUP],
          (unsigned long long)(pool.paths_completed + pool.paths_failed),
          (unsigned long long)total_tasks,
          pool.in_drain_phase ? "DRAIN" : "refill");
      }
      { /* STARDIS_PIPELINE_LOG */
        const char* env_log = getenv("STARDIS_PIPELINE_LOG");
        if(env_log && env_log[0] == '1') {
          log_info(scn->dev,
            "[B] step %llu: rays=%llu, active=%llu\n",
            (unsigned long long)pool.total_steps,
            (unsigned long long)pv_b->ray_count,
            (unsigned long long)pv_b->active_compact);
        }
      }
      if(pool.total_steps > total_tasks * 1000) {
        log_err(scn->dev,
          "pipeline: infinite loop safety break at step %llu\n",
          (unsigned long long)pool.total_steps);
        res = RES_BAD_OP;
        goto cleanup;
      }

      /* ════════ P4 Timeline output (STARDIS_PIPELINE_LOG=2) ════════ */
      {
        const char* env_tl = getenv("STARDIS_PIPELINE_LOG");
        env_tl="2";
        if(env_tl && env_tl[0] == '2' && pool.total_steps % 500 == 0) {
          double waitA   = time_elapsed_sec(&t_cy[0], &t_cy[1]) * 1000.0;
          double launchB = time_elapsed_sec(&t_cy[1], &t_cy[2]) * 1000.0;
          double cpuA    = time_elapsed_sec(&t_cy[2], &t_cy[3]) * 1000.0;
          double waitB   = time_elapsed_sec(&t_cy[3], &t_cy[4]) * 1000.0;
          double launchA = time_elapsed_sec(&t_cy[4], &t_cy[5]) * 1000.0;
          double cpuB    = time_elapsed_sec(&t_cy[5], &t_cy[6]) * 1000.0;
          double cycle   = time_elapsed_sec(&t_cy[0], &t_cy[6]) * 1000.0;
          log_info(scn->dev,
            "[TIMELINE] step=%llu "
            "|waitA=%.2fms|launchB=%.2fms|cpuA=%.2fms"
            "|waitB=%.2fms|launchA=%.2fms|cpuB=%.2fms"
            "|cycle=%.2fms raysA=%llu raysB=%llu\n",
            (unsigned long long)pool.total_steps,
            waitA, launchB, cpuA,
            waitB, launchA, cpuB,
            cycle,
            (unsigned long long)pv_a->ray_count,
            (unsigned long long)pv_b->ray_count);
        }
      }

      /* ════════ Dynamic merge check ════════ */
      if(should_merge(&pool)) {
        /* Wait for all pending GPU calls */
        if(pv_a->gpu_pending) {
          res = gpu_wait_and_postprocess(&pool, pv_a, scn->s3d_view, scn);
          if(res != RES_OK) goto cleanup;
        }
        if(pv_b->gpu_pending) {
          res = gpu_wait_and_postprocess(&pool, pv_b, scn->s3d_view, scn);
          if(res != RES_OK) goto cleanup;
        }
        log_info(scn->dev,
          "pipeline: merging to single-pool at step %llu "
          "(A.active=%llu, B.active=%llu)\n",
          (unsigned long long)pool.total_steps,
          (unsigned long long)pv_a->active_compact,
          (unsigned long long)pv_b->active_compact);
        merge_to_single_pool(&pool);
        break;  /* fall through to single-pool loop */
      }
    } /* end dual-pool while */

    /* ---- Drain: wait for last pending GPU calls ---- */
    if(pool.num_active_views == 2) {
      if(pv_a->gpu_pending) {
        res = gpu_wait_and_postprocess(&pool, pv_a, scn->s3d_view, scn);
        if(res != RES_OK) goto cleanup;
        pool.total_steps++;
        res = cpu_between(&pool, pv_a, scn, buf);
        if(res != RES_OK) goto cleanup;
      }
      if(pv_b->gpu_pending) {
        res = gpu_wait_and_postprocess(&pool, pv_b, scn->s3d_view, scn);
        if(res != RES_OK) goto cleanup;
        pool.total_steps++;
        res = cpu_between(&pool, pv_b, scn, buf);
        if(res != RES_OK) goto cleanup;
      }
    }

    /* Log pipeline stats */
    {
      struct time t_now;
      double pipeline_wall;
      time_current(&t_now);
      pipeline_wall = time_elapsed_sec(&t_start, &t_now);
      if(pipeline_wall <= 0) pipeline_wall = 1e-9;
      log_info(scn->dev,
        "pipeline stats: wall=%.3fs, gpu_wait=%.3fs, cpu_between=%.3fs, "
        "gpu_util=%.1f%%, steps=%llu\n",
        pipeline_wall,
        pool.time_pipeline_wait_s,
        pool.time_pipeline_cpu_between_s,
        100.0 * (1.0 - pool.time_pipeline_gpu_idle_s / pipeline_wall),
        (unsigned long long)pool.total_steps);
    }
  } /* end if(num_active_views == 2) */

  /* ═══════════════════════════════════════════════════════════ */
  /*  Single-pool main loop (num_active_views == 1)              */
  /*  - STARDIS_PIPELINE=0 enters directly                      */
  /*  - Or merge_to_single_pool above falls through              */
  /* ═══════════════════════════════════════════════════════════ */
  if(pool.num_active_views == 1) {

  while(pool.active_count > 0 || pool.task_next < pool.task_count) {
    size_t refill_count = 0;
    struct time t_phase0, t_phase1; /* M3 per-phase timing */
    struct pool_view* pv = &pool.views[0]; /* single-pool view */
    pool.total_steps++;

    /* Step A: Stream compaction (M2.5) */
    time_current(&t_phase0);
    compact_active_paths(&pool, pv);
    time_current(&t_phase1);
    pool.time_compact_s += time_elapsed_sec(&t_phase0, &t_phase1);

    /* Step B: Collect ray requests — bucketed (B-4 M2) */
    time_current(&t_phase0);
    pv->ray_count = 0;
    res = pool_collect_ray_requests_bucketed(&pool, pv);
    if(res != RES_OK) goto cleanup;
    time_current(&t_phase1);
    pool.time_collect_s += time_elapsed_sec(&t_phase0, &t_phase1);

    /* Step C: Batch trace via Phase B-1 */
    time_current(&t_phase0);
    if(pv->ray_count > 0) {
      struct s3d_batch_trace_stats stats;
      memset(&stats, 0, sizeof(stats));

      res = s3d_scene_view_trace_rays_batch_ctx(
        scn->s3d_view, pv->batch_ctx,
        pv->ray_requests, pv->ray_count,
        pv->ray_hits, &stats);
      if(res != RES_OK) goto cleanup;

      pool.total_rays_traced += pv->ray_count;

      /* Promote per-view pending ray stats to pool (single-pool path) */
      {
        pool.rays_radiative           += pv->pending_rays_radiative;
        pool.rays_conductive_ds       += pv->pending_rays_conductive_ds;
        pool.rays_conductive_ds_retry += pv->pending_rays_conductive_ds_retry;
        pool.rays_shadow              += pv->pending_rays_shadow;
        pool.rays_enclosure           += pv->pending_rays_enclosure;
        pool.rays_startup             += pv->pending_rays_startup;
        pool.rays_other               += pv->pending_rays_other;
      }

      /* Experiment 7+8: accumulate per-call batch trace stats */
      pool.trace_call_count++;
      pool.trace_batch_size_sum += pv->ray_count;
      if(pv->ray_count < pool.trace_batch_size_min)
        pool.trace_batch_size_min = pv->ray_count;
      if(pv->ray_count > pool.trace_batch_size_max)
        pool.trace_batch_size_max = pv->ray_count;
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
    res = pool_distribute_ray_results(&pool, pv, scn);
    if(res != RES_OK) goto cleanup;
    time_current(&t_phase1);
    pool.time_distribute_s += time_elapsed_sec(&t_phase0, &t_phase1);

    /* Step D2 (M10): Collect + dispatch + distribute enc_locate batch */
    time_current(&t_phase0);
    res = pool_collect_enc_locate_requests(&pool, pv);
    if(res != RES_OK) goto cleanup;

    if(pv->enc_locate_count > 0) {
      struct s3d_batch_enc_stats enc_stats;
      memset(&enc_stats, 0, sizeof(enc_stats));

      res = s3d_scene_view_find_enclosure_batch_ctx(
        scn->s3d_view, pv->enc_batch_ctx,
        pv->enc_locate_requests, pv->enc_locate_count,
        pv->enc_locate_results, &enc_stats);
      if(res != RES_OK) goto cleanup;

      /* Distribute prim_id + side to paths; enc_id resolution happens
       * in step_enc_locate_result() during cascade (handles degenerate). */
      res = pool_distribute_enc_locate_results(&pool, pv);
      if(res != RES_OK) goto cleanup;

      pool.enc_locates_total += pv->enc_locate_count;
      pool.enc_locates_resolved += enc_stats.resolved;
      pool.enc_locates_degenerate += enc_stats.degenerate;
    }
    time_current(&t_phase1);
    pool.time_enc_locate_s += time_elapsed_sec(&t_phase0, &t_phase1);

    /* Step D3 (M9): Collect + dispatch + distribute closest_point batch */
    time_current(&t_phase0);
    res = pool_collect_cp_requests(&pool, pv);
    if(res != RES_OK) goto cleanup;

    if(pv->cp_count > 0) {
      struct s3d_batch_cp_stats cp_stats;
      memset(&cp_stats, 0, sizeof(cp_stats));

      res = s3d_scene_view_closest_point_batch_ctx(
        scn->s3d_view, pv->cp_batch_ctx,
        pv->cp_requests, pv->cp_count,
        pv->cp_hits, &cp_stats);
      if(res != RES_OK) goto cleanup;

      res = pool_distribute_cp_results(&pool, pv);
      if(res != RES_OK) goto cleanup;

      pool.cp_total    += pv->cp_count;
      pool.cp_accepted += cp_stats.batch_accepted;
      pool.cp_requeried += cp_stats.requery_accepted;
    }
    time_current(&t_phase1);
    pool.time_cp_s += time_elapsed_sec(&t_phase0, &t_phase1);

    /* P1 SYNC POINT A (single-pool): distribute + enc_locate + cp finished
     * — refresh dsoa for all modified slots before cascade and next cycle's
     * compact/collect which read from dsoa. */
    {
      size_t k;
      for(k = 0; k < pv->need_ray_count; k++) {
        uint32_t idx = pv->need_ray_indices[k];
        dispatch_soa_sync_from_path(&pool.dsoa, idx, &pool.slots[idx]);
      }
      for(k = 0; k < pv->enc_locate_count; k++) {
        uint32_t idx = pv->enc_locate_to_slot[k];
        dispatch_soa_sync_from_path(&pool.dsoa, idx, &pool.slots[idx]);
      }
      for(k = 0; k < pv->cp_count; k++) {
        uint32_t idx = pv->cp_to_slot[k];
        dispatch_soa_sync_from_path(&pool.dsoa, idx, &pool.slots[idx]);
      }
    }

    /* Step E: Cascade non-ray steps (compact) */
    time_current(&t_phase0);
    res = pool_cascade_non_ray_steps_compact(&pool, pv, scn);
    if(res != RES_OK) goto cleanup;
    time_current(&t_phase1);
    pool.time_cascade_s += time_elapsed_sec(&t_phase0, &t_phase1);

    /* P1 SYNC POINT B (single-pool): cascade finished — refresh dsoa for
     * all active slots before compact + harvest read from dsoa. */
    {
      size_t k;
      for(k = 0; k < pv->active_compact; k++) {
        uint32_t idx = pv->active_indices[k];
        dispatch_soa_sync_from_path(&pool.dsoa, idx, &pool.slots[idx]);
      }
    }

    /* Step F+G: Harvest completed paths + refill (timed together) */
    time_current(&t_phase0);
    compact_active_paths(&pool, pv); /* rebuild done_indices after cascade */
    res = harvest_completed_paths(&pool, pv, buf);
    if(res != RES_OK) goto cleanup;

    /* Step G: Refill pool with new tasks (M2) */
    res = refill_pool(&pool, pv, &refill_count);
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
        "persistent_wavefront: entering drain phase at step %llu, "
        "%llu active paths remain, refill_wall=%.3fs\n",
        (unsigned long long)pool.total_steps,
        (unsigned long long)pool.active_count,
        pool.time_refill_phase_s);
    }

    /* Step J: Update diagnostics (M2.5 + M3 wavefront width) */
    pool_update_diagnostics(&pool, pv);

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
        "persistent_wavefront step %llu: %llu/%llu active, "
        "%llu rays this step (rad=%llu ds=%llu shd=%llu enc=%llu st=%llu), "
        "%llu/%llu tasks done, %s\n",
        (unsigned long long)pool.total_steps,
        (unsigned long long)pool.active_count,
        (unsigned long long)pool.pool_size,
        (unsigned long long)pv->ray_count,
        (unsigned long long)pv->bucket_counts[RAY_BUCKET_RADIATIVE],
        (unsigned long long)pv->bucket_counts[RAY_BUCKET_STEP_PAIR],
        (unsigned long long)pv->bucket_counts[RAY_BUCKET_SHADOW],
        (unsigned long long)pv->bucket_counts[RAY_BUCKET_ENCLOSURE],
        (unsigned long long)pv->bucket_counts[RAY_BUCKET_STARTUP],
        (unsigned long long)(pool.paths_completed + pool.paths_failed),
        (unsigned long long)total_tasks,
        pool.in_drain_phase ? "DRAIN" : "refill");
    }

    /* Safety: prevent infinite loops */
    if(pool.total_steps > total_tasks * 1000) {
      log_err(scn->dev,
        "persistent_wavefront: exceeded maximum step count (%llu). "
        "%llu paths still active.\n",
        (unsigned long long)pool.total_steps,
        (unsigned long long)pool.active_count);
      res = RES_BAD_OP;
      goto cleanup;
    }
  } /* end single-pool while */
  } /* end if(pool.num_active_views == 1) */

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
      "persistent_wavefront DONE: %llux%llu spp=%llu pool=%llu "
      "elapsed=%s  steps=%llu  rays=%llu  avg_width=%.1f  "
      "(rad=%llu cond_ds=%llu(retry=%llu) shadow=%llu enc=%llu "
      "startup=%llu other=%llu)  "
      "done: rad=%llu temp=%llu bnd=%llu fail=%llu  "
      "max_depth=%llu\n",
      (unsigned long long)image_def[0], (unsigned long long)image_def[1],
      (unsigned long long)spp, (unsigned long long)pool.pool_size,
      time_str,
      (unsigned long long)pool.total_steps,
      (unsigned long long)pool.total_rays_traced,
      avg_width,
      (unsigned long long)pool.rays_radiative,
      (unsigned long long)pool.rays_conductive_ds,
      (unsigned long long)pool.rays_conductive_ds_retry,
      (unsigned long long)pool.rays_shadow,
      (unsigned long long)pool.rays_enclosure,
      (unsigned long long)pool.rays_startup,
      (unsigned long long)pool.rays_other,
      (unsigned long long)pool.paths_done_radiative,
      (unsigned long long)pool.paths_done_temperature,
      (unsigned long long)pool.paths_done_boundary,
      (unsigned long long)pool.paths_failed,
      (unsigned long long)pool.max_path_depth);
  }

  /* M9: WoS closest_point batch summary */
  if(pool.cp_total > 0) {
    log_info(scn->dev,
      "persistent_wavefront CP (WoS): %llu queries, "
      "%llu gpu-accepted, %llu requeried, %.3fs total\n",
      (unsigned long long)pool.cp_total,
      (unsigned long long)pool.cp_accepted,
      (unsigned long long)pool.cp_requeried,
      pool.time_cp_s);
  }

  log_drain_phase_report(scn->dev, &pool);

cleanup:
  /* Flush and close pixel trace file */
  if(s_pixel_trace_fp) { fclose(s_pixel_trace_fp); s_pixel_trace_fp = NULL; s_pixel_trace_checked = 0; }

  pool_destroy(&pool);
  return res;
}
#endif /* !SDIS_SOLVE_PERSISTENT_WAVEFRONT_SKIP_PUBLIC_API */
