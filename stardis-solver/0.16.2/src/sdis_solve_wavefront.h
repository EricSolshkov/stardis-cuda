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

#include "sdis.h"
#include "sdis_heat_path.h"
#include "sdis_scene_c.h"
#include "sdis_tile.h"

#include <star/s3d.h>
#include <star/ssp.h>
#include <stdint.h>

/* Forward declarations */
struct sdis_estimator_buffer;

/*******************************************************************************
 * Path phase — models the state machine of a single Monte-Carlo path.
 ******************************************************************************/
enum path_phase {
  /* === Initialisation === */
  PATH_INIT,                          /* Not yet started                      */

  /* === Radiative path phases === */
  PATH_RAD_TRACE_PENDING,             /* trace_ray request submitted          */
  PATH_RAD_PROCESS_HIT,               /* hit obtained, processing BRDF/emit  */

  /* === Coupled path state machine === */
  PATH_COUPLED_BOUNDARY,              /* boundary_path decision               */
  PATH_COUPLED_BOUNDARY_REINJECT,     /* find_reinjection_ray: wait for trace */
  PATH_COUPLED_CONDUCTIVE,            /* conductive_path entry                */
  PATH_COUPLED_COND_DS_PENDING,       /* delta_sphere 2-ray wait             */
  PATH_COUPLED_CONVECTIVE,            /* convective_path                      */
  PATH_COUPLED_RADIATIVE,             /* bounce into radiative from boundary  */

  /* === Terminal === */
  PATH_DONE                           /* path finished; weight is valid       */
};

/*******************************************************************************
 * Per-ray request descriptor (wavefront internal)
 ******************************************************************************/
struct path_ray_request {
  float    origin[3];
  float    direction[3];
  float    range[2];

  /* Second ray (delta_sphere / find_reinjection_ray can emit 2 rays) */
  float    direction2[3];
  float    range2[2];
  int      ray_count;        /* 1 or 2                                       */

  /* Batch dispatch indices (set by collect_ray_requests) */
  uint32_t batch_idx;        /* index of ray 0 in the batch                  */
  uint32_t batch_idx2;       /* index of ray 1 in the batch (if ray_count==2)*/
};

/*******************************************************************************
 * struct path_state — fully externalised state of one MC path
 *
 * Approximately 600 bytes including padding.  For a tile of 4×4 pixels at
 * spp = 256, that is 4096 paths × 600 B ≈ 2.4 MB, well within stack / heap
 * budgets.
 ******************************************************************************/
struct path_state {
  /* --- Identity --- */
  uint32_t  path_id;                   /* global unique id within the tile   */
  uint16_t  pixel_x, pixel_y;         /* pixel coords (tile-local)          */
  uint32_t  realisation_idx;           /* SPP index                          */

  /* --- Lifecycle --- */
  enum path_phase  phase;
  int              active;             /* 1 = still running                  */

  /* --- Random walk core --- */
  struct rwalk          rwalk;         /* position, time, hit, enc_id …      */
  struct rwalk_context  ctx;           /* Tmin/That/branchings …             */

  /* --- Temperature / weight --- */
  struct temperature T;                /* func ptr + value + done            */

  /* --- Radiative path scratch --- */
  float   rad_direction[3];            /* current radiative direction        */
  int     rad_bounce_count;
  int     rad_retry_count;             /* find_next_fragment retries         */

  /* --- Coupled path scratch --- */
  int     coupled_nbranchings;

  /* --- Conductive delta-sphere scratch --- */
  float   ds_dir0[3], ds_dir1[3];
  struct s3d_hit ds_hit0, ds_hit1;
  double  ds_delta_solid;

  /* --- Boundary reinjection scratch --- */
  struct s3d_hit bnd_hit0, bnd_hit1;
  double  bnd_reinject_distance;
  unsigned bnd_solid_enc_id;
  int     bnd_retry_count;

  /* --- Filter data for current ray request --- */
  struct hit_filter_data  filter_data_storage;

  /* --- Ray request output (set by step functions) --- */
  struct path_ray_request ray_req;
  int    needs_ray;                    /* does this step need a trace?        */

  /* --- RNG (shared per-thread, non-owning pointer) --- */
  struct ssp_rng* rng;

  /* --- Image-space pixel coords (in image plane) --- */
  size_t  ipix_image[2];
};

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

  /* Per-path RNG array (allocated once) */
  /* (each path stores a non-owning pointer to the shared per-thread RNG) */

  /* Statistics */
  size_t  total_steps;
  size_t  total_rays_traced;
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

#endif /* SDIS_SOLVE_WAVEFRONT_H */
