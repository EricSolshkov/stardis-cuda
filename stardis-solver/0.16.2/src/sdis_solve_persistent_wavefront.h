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

/* Phase B-3: Persistent Wavefront Pool for solve_camera.
 *
 * Instead of per-tile wavefront isolation (Phase B-2), this module pools ALL
 * pixel×SPP tasks from the entire image into a single persistent wavefront.
 * This eliminates the GPU utilisation bottleneck caused by TILE_SIZE=4
 * (only 512 paths/tile → <3% GPU occupancy on RTX 4090).
 *
 * M2: Fixed-size pool with refill — completed paths are immediately replaced
 * with new tasks from the global queue, keeping the wavefront width constant
 * at pool_size until the task queue is exhausted (drain phase).
 *
 * M2.5: Stream compaction + type bucketing — every step rebuilds compact
 * index arrays (active_indices, need_ray_indices, done_indices) to avoid
 * scanning the full pool.  Paths are also bucketed by type (radiative vs
 * conductive) for cache-friendly step dispatch.
 *
 * M3: Bucketed step dispatch + adaptive pool_size + enhanced diagnostics.
 * Ray results are distributed via per-type loops (radiative then conductive)
 * instead of a generic switch dispatch, improving branch prediction and
 * cache locality.  pool_size is computed from GPU SM count.  Per-phase
 * timing and average wavefront width are tracked for performance analysis.
 *
 * Key design points:
 *   - Single-threaded CPU scheduling (no OMP) — one CUDA stream, no races.
 *   - Per-slot RNG shared from solve_camera's per_thread_rng via ref_get().
 *   - All step functions from sdis_solve_wavefront.c are fully reused.
 *   - Results written directly to estimator_buffer (bypass tile layer).
 *
 * Reference: guide/upper-parallelization/phase_b3_persistent_wavefront.md
 */

#ifndef SDIS_SOLVE_PERSISTENT_WAVEFRONT_H
#define SDIS_SOLVE_PERSISTENT_WAVEFRONT_H

#include "sdis.h"
#include "sdis_solve_wavefront.h"  /* path_state, path_phase, ray_request */

#include <star/s3d.h>
#include <star/ssp.h>
#include <stdint.h>

/* Forward declarations */
struct sdis_estimator_buffer;
struct sdis_scene;
struct sdis_camera;
struct ssp_rng;

/*******************************************************************************
 * pixel_task — one pixel×realisation work item
 ******************************************************************************/
struct pixel_task {
  size_t   ipix_image[2];  /* image-space pixel coordinates */
  uint32_t spp_idx;        /* realisation index (0..spp-1)  */
};

/*******************************************************************************
 * Default pool size — tuned for RTX 4090 (128 SM × 256 threads/SM).
 * Overridable via STARDIS_POOL_SIZE environment variable.
 ******************************************************************************/
#define PERSISTENT_WF_DEFAULT_POOL_SIZE  32768

/*******************************************************************************
 * wavefront_pool — persistent pool holding all active paths
 ******************************************************************************/
struct wavefront_pool {
  /* --- Path pool (fixed size) --- */
  struct path_state*  slots;
  size_t              pool_size;       /* number of path slots */
  size_t              active_count;    /* currently active paths */

  /* --- Global task queue --- */
  struct pixel_task*  task_queue;
  size_t              task_count;      /* image_w × image_h × spp */
  size_t              task_next;       /* next unassigned task index */

  /* --- Scene / camera params (cached for refill) --- */
  struct sdis_scene*  scn;
  unsigned            enc_id;
  const struct sdis_camera* cam;
  const double*       time_range;
  const double*       pix_sz;
  size_t              picard_order;
  enum sdis_diffusion_algorithm diff_algo;
  uint32_t            next_path_id;    /* monotonically increasing path id */

  /* --- Per-slot RNG (independent streams) --- */
  struct ssp_rng**    slot_rngs;

  /* --- Stream Compaction indices (M2.5) --- */
  uint32_t*           active_indices;  /* compact array of active slot ids  */
  size_t              active_compact;  /* length of active_indices          */

  uint32_t*           need_ray_indices;/* slots that need ray trace         */
  size_t              need_ray_count;  /* length of need_ray_indices        */

  uint32_t*           done_indices;    /* slots with PATH_DONE, !active     */
  size_t              done_count;      /* length of done_indices            */

  /* --- Path type buckets (M2.5) --- */
  uint32_t*           bucket_radiative;   /* radiative slot indices         */
  size_t              bucket_radiative_n;
  uint32_t*           bucket_conductive;  /* conductive slot indices        */
  size_t              bucket_conductive_n;

  /* --- Ray request buffers --- */
  struct s3d_ray_request* ray_requests;
  uint32_t*           ray_to_slot;     /* ray → slot index */
  uint32_t*           ray_slot_sub;    /* ray → sub-ray (0 or 1) */
  struct s3d_hit*     ray_hits;
  size_t              ray_count;
  size_t              max_rays;        /* pool_size × 6 */

  /* --- Ray bucket offsets (B-4 M2) --- */
  size_t              bucket_offsets[RAY_BUCKET_COUNT + 1];
  size_t              bucket_counts[RAY_BUCKET_COUNT];

  /* --- Batch trace context (Phase B-1) --- */
  struct s3d_batch_trace_context* batch_ctx;

  /* --- Drain phase control (M2) --- */
  int                 in_drain_phase;  /* 1 = task_queue exhausted          */
  size_t              drain_step_count;/* steps taken in drain phase        */
  size_t              drain_fallback_threshold; /* batch < this → CPU (M3.5)*/

  /* --- Statistics --- */
  size_t total_steps;
  size_t total_rays_traced;
  size_t paths_completed;
  size_t paths_failed;
  size_t max_path_depth;

  /* Detailed ray statistics */
  size_t rays_radiative;
  size_t rays_conductive_ds;
  size_t rays_conductive_ds_retry;
  size_t rays_enclosure;
  size_t rays_shadow;
  size_t rays_startup;
  size_t paths_done_radiative;
  size_t paths_done_temperature;
  size_t paths_done_boundary;

  /* Diagnostics (M2.5 + M3) */
  size_t diag_min_batch;    /* smallest batch size seen              */
  size_t diag_max_batch;    /* largest batch size seen               */
  size_t diag_refill_count; /* total refill operations               */
  size_t diag_drain_rays;   /* rays traced during drain phase        */
  size_t diag_refill_rays;  /* rays traced during refill phase       */

  /* M3: Per-phase wall-clock timing (seconds) */
  double time_collect_s;    /* cumulative collect_ray_requests time  */
  double time_trace_s;      /* cumulative GPU batch trace time       */
  double time_distribute_s; /* cumulative distribute+advance time    */
  double time_cascade_s;    /* cumulative cascade non-ray time       */
  double time_compact_s;    /* cumulative stream compaction time     */
  double time_harvest_s;    /* cumulative harvest+refill time        */
  double time_refill_phase_s; /* wall-clock of refill phase total    */
  double time_drain_phase_s;  /* wall-clock of drain phase total     */

  /* M3: Wavefront width tracking */
  size_t diag_total_active; /* sum of active_count across all steps  */
  size_t paths_truncated;   /* paths force-terminated in drain (M3.5)*/
};

/*******************************************************************************
 * Public API — called from sdis_solve_camera
 ******************************************************************************/

/* Persistent wavefront replacement of the OMP tile loop.
/* ============================================================================
 * Internal helpers — LOCAL_SYM, accessible by unit tests via OBJECT lib
 * ============================================================================ */

static INLINE size_t
count_path_rays(const struct path_state* p)
{
  if(p->ray_count_ext == 6 && p->phase == PATH_ENC_QUERY_EMIT)
    return 6;
  return (size_t)p->ray_req.ray_count;
}

extern LOCAL_SYM res_T
pool_collect_ray_requests_bucketed(struct wavefront_pool* pool);

/* ============================================================================
 * Public API
 * ============================================================================ */

/* Processes the entire image in a single wavefront pool.
 * Receives the already-created per-thread RNG array (nthreads entries)
 * from solve_camera.  The buckets are already locked by create_per_thread_rng
 * so we must not call ssp_rng_proxy_create_rng again. */
extern LOCAL_SYM res_T
solve_camera_persistent_wavefront
  (struct sdis_scene*          scn,
   struct ssp_rng**            per_thread_rng,
   const size_t                nthreads,
   const unsigned              enc_id,
   const struct sdis_camera*   cam,
   const double                time_range[2],
   const size_t                image_def[2],  /* image width, height */
   const size_t                spp,
   const int                   register_paths,
   const double                pix_sz[2],
   const size_t                picard_order,
   const enum sdis_diffusion_algorithm diff_algo,
   struct sdis_estimator_buffer* buf,
   int32_t                     progress[],
   const int                   pcent_progress,
   const char*                 progress_label);

#endif /* SDIS_SOLVE_PERSISTENT_WAVEFRONT_H */
