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
 * Instead of running each pixel's path to completion (depth-first), all active
 * paths advance one step in lockstep.  Ray requests emitted during a step are
 * collected into a batch and traced via Phase B-1's
 * s3d_scene_view_trace_rays_batch_ctx(), then results are distributed back to
 * the paths.
 *
 * Key design points:
 *   - Each OMP thread runs its own independent wavefront (no cross-thread
 *     sharing).
 *   - Per-path independent RNG (seeded from base_seed + path_id) so that
 *     interleaving does not change results beyond the expected RNG-stream
 *     reordering.  The Monte Carlo estimator's expected value is unchanged.
 *   - All stack-local state from the original recursive path code is
 *     externalised into struct path_state.
 */

#ifndef SDIS_SOLVE_WAVEFRONT_H
#define SDIS_SOLVE_WAVEFRONT_H

/* Pull in extracted type and state headers */
#include "sdis_wf_types.h"  /* enum path_phase, enum ray_bucket_type, etc. */
#include "sdis_wf_state.h"  /* struct path_ray_request, struct path_state  */

#include "sdis.h"
#include "sdis_misc.h"  /* struct accum (for probe-mode output) */
#include "sdis_tile.h"

#include <star/s3d.h>
#include <stdint.h>

/* Forward declarations */
struct sdis_estimator_buffer;

/*******************************************************************************
 * struct wavefront_context — per-tile wavefront scheduler
 ******************************************************************************/
struct wavefront_context {
  /* Path pool */
  struct path_state*  paths;
  size_t              total_paths;
  size_t              active_count;

  /* Ray request collection */
  struct s3d_ray_request*  ray_requests;   /* indexed by ray_count           */
  uint32_t*               ray_to_path;    /* ray→path mapping               */
  uint32_t*               ray_slot;       /* ray→slot (0 or 1) in path      */
  size_t                  ray_count;
  size_t                  max_rays;

  /* Trace results */
  struct s3d_hit*         ray_hits;

  /* Pre-allocated batch trace context (Phase B-1) */
  struct s3d_batch_trace_context* batch_ctx;

  /* B-4 M10: enc_locate batch buffers */
  struct s3d_batch_enc_context*  enc_batch_ctx;
  struct s3d_enc_locate_request* enc_locate_requests;
  struct s3d_enc_locate_result*  enc_locate_results;
  uint32_t*                      enc_locate_to_path;
  size_t                         enc_locate_count;
  size_t                         max_enc_locates;

  /* B-4 M9: closest_point (WoS) batch buffers */
  struct s3d_batch_cp_context*   cp_batch_ctx;
  struct s3d_cp_request*         cp_requests;
  struct s3d_hit*                cp_hits;
  uint32_t*                      cp_to_path;
  size_t                         cp_count;
  size_t                         max_cps;

  /* Per-path RNG array (allocated once) */
  /* (each path stores a non-owning pointer to the shared per-thread RNG) */

  /* Statistics */
  size_t  total_steps;
  size_t  total_rays_traced;

  /* Detailed statistics (M4/M5 diagnostics) */
  size_t  rays_radiative;          /* rays from radiative path traces       */
  size_t  rays_conductive_ds;      /* rays from delta-sphere (2 per step)   */
  size_t  rays_conductive_ds_retry;/* robust retry re-traces                */
  size_t  paths_done_radiative;    /* paths terminated in radiative miss    */
  size_t  paths_done_temperature;  /* paths terminated by known temperature */
  size_t  paths_done_boundary;     /* paths terminated by boundary/absorb   */
  size_t  paths_failed;            /* paths that errored out                */
  size_t  conductive_steps;        /* delta-sphere iteration count          */
  size_t  max_wavefront_depth;     /* max steps any single path took        */
};

/*******************************************************************************
 * Public API — called from solve_camera
 ******************************************************************************/

/* Wavefront replacement of solve_tile.  Signature matches the original
 * solve_tile() parameters exactly so the caller can switch between the two
 * with a simple flag. */
extern LOCAL_SYM res_T
solve_tile_wavefront
  (struct sdis_scene* scn,
   struct ssp_rng*    base_rng,      /* per-thread RNG (used for seeding)   */
   const unsigned     enc_id,
   const struct sdis_camera* cam,
   const double       time_range[2],
   const size_t       tile_org[2],   /* origin in pixel space               */
   const size_t       tile_size[2],  /* #pixels in X and Y                  */
   const size_t       spp,
   const int          register_paths,
   const double       pix_sz[2],
   const size_t       picard_order,
   const enum sdis_diffusion_algorithm diff_algo,
   struct sdis_estimator_buffer* buf,
   struct tile*       tile);

/* Internal (LOCAL_SYM) — exposed so that unit tests can call it directly. */
extern LOCAL_SYM res_T
collect_ray_requests(struct wavefront_context* wf);

/*******************************************************************************
 * Probe-mode wavefront solver
 *
 * Wavefront variant of sdis_solve_probe: all realisations originate from the
 * same spatial position and advance in lockstep with batched ray-tracing.
 ******************************************************************************/
extern LOCAL_SYM res_T
solve_wavefront_probe
  (struct sdis_scene* scn,
   struct ssp_rng*    base_rng,
   const unsigned     enc_id,
   const double       position[3],
   const double       time_range[2],
   const size_t       nrealisations,
   const size_t       picard_order,
   const enum sdis_diffusion_algorithm diff_algo,
   struct accum*      out_acc_temp);

#endif /* SDIS_SOLVE_WAVEFRONT_H */
