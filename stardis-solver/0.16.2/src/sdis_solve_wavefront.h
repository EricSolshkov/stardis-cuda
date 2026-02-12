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
#include "sdis_medium_c.h"
#include "sdis_scene_c.h"
#include "sdis_tile.h"

#include <star/s3d.h>
#include <star/ssp.h>
#include <stdint.h>

/* Forward declarations */
struct sdis_estimator_buffer;

/*******************************************************************************
 * Path phase — models the state machine of a single Monte-Carlo path.
 *
 * Phase B-4: Fine-grained explicit state machine (~45 states).
 * Each trace_ray call site becomes a wavefront suspend/resume point.
 *
 * Legend:
 *   🔴 = ray-pending (needs batch trace result before resuming)
 *   ✅ = pure-compute (advances without ray)
 *
 * States marked [B4-FUTURE] are defined here for completeness but currently
 * fall through to the original synchronous code path.  They will be activated
 * in subsequent B-4 milestones.
 ******************************************************************************/
enum path_phase {
  /* === Initialisation === */
  PATH_INIT,                              /* ✅ Not yet started               */

  /* === Radiative path phases === */
  PATH_RAD_TRACE_PENDING,                 /* 🔴 trace_ray submitted           */
  PATH_RAD_PROCESS_HIT,                   /* ✅ hit: BRDF/emit [B4-FUTURE]    */

  /* === Coupled path state machine (B-2/B-3 legacy) === */
  PATH_COUPLED_BOUNDARY,                  /* ✅ boundary_path decision         */
  PATH_COUPLED_BOUNDARY_REINJECT,         /* 🔴 find_reinjection_ray wait     */
  PATH_COUPLED_CONDUCTIVE,                /* ✅ conductive_path entry          */
  PATH_COUPLED_COND_DS_PENDING,           /* 🔴 delta_sphere 2-ray wait       */
  PATH_COUPLED_CONVECTIVE,                /* ✅ convective_path                */
  PATH_COUPLED_RADIATIVE,                 /* ✅ bounce into radiative          */

  /* === B-4 Fine-grained: Boundary dispatch === */
  PATH_BND_DISPATCH,                      /* ✅ type → 3-way dispatch  [B4-FUTURE] */
  PATH_BND_POST_ROBIN_CHECK,              /* ✅ Robin post-check       [B4-FUTURE] */

  /* === B-4: Solid/Solid boundary === */
  PATH_BND_SS_REINJECT_SAMPLE,            /* 🔴 4 reinjection rays     [B4-FUTURE] */
  PATH_BND_SS_REINJECT_ENC,               /* 🔴 miss → ENC sub-query   [B4-FUTURE] */
  PATH_BND_SS_REINJECT_DECIDE,            /* ✅ probability decision    [B4-FUTURE] */

  /* === B-4: Solid/Fluid picard1 boundary === */
  PATH_BND_SF_REINJECT_SAMPLE,            /* 🔴 2 reinjection rays     [B4-FUTURE] */
  PATH_BND_SF_REINJECT_ENC,               /* 🔴 miss → ENC sub-query   [B4-FUTURE] */
  PATH_BND_SF_PROB_DISPATCH,              /* ✅ prob dispatch           [B4-FUTURE] */
  PATH_BND_SF_NULLCOLL_RAD_TRACE,         /* 🔴 null-collision rad ray [B4-FUTURE] */
  PATH_BND_SF_NULLCOLL_DECIDE,            /* ✅ accept/reject          [B4-FUTURE] */

  /* === B-4: Solid/Fluid picardN boundary === */
  PATH_BND_SFN_PROB_DISPATCH,             /* ✅ prob + stack mgmt      [B4-FUTURE] */
  PATH_BND_SFN_RAD_TRACE,                 /* 🔴 rad sub-path ray      [B4-FUTURE] */
  PATH_BND_SFN_RAD_DONE,                  /* ✅ rad sub-path done      [B4-FUTURE] */
  PATH_BND_SFN_COMPUTE_Ti,                /* ✅ push sub-path          [B4-FUTURE] */
  PATH_BND_SFN_COMPUTE_Ti_RESUME,         /* ✅ pop sub-path           [B4-FUTURE] */
  PATH_BND_SFN_CHECK_PMIN_PMAX,           /* ✅ early accept/reject    [B4-FUTURE] */

  /* === B-4: External net flux (picard sub-process) === */
  PATH_BND_EXT_CHECK,                     /* ✅ need ext flux?         [B4-FUTURE] */
  PATH_BND_EXT_DIRECT_TRACE,              /* 🔴 shadow ray             [B4-FUTURE] */
  PATH_BND_EXT_DIRECT_RESULT,             /* ✅ shadow result          [B4-FUTURE] */
  PATH_BND_EXT_DIFFUSE_TRACE,             /* 🔴 diffuse bounce ray    [B4-FUTURE] */
  PATH_BND_EXT_DIFFUSE_RESULT,            /* ✅ bounce result          [B4-FUTURE] */
  PATH_BND_EXT_DIFFUSE_SHADOW_TRACE,      /* 🔴 bounce shadow ray     [B4-FUTURE] */
  PATH_BND_EXT_DIFFUSE_SHADOW_RESULT,     /* ✅ bounce shadow result  [B4-FUTURE] */
  PATH_BND_EXT_FINALIZE,                  /* ✅ sum flux → return      [B4-FUTURE] */

  /* === B-4: Conductive path === */
  PATH_CND_INIT_ENC,                      /* 🔴 init ENC query         [B4-FUTURE] */

  /* --- Delta-sphere conductive --- */
  PATH_CND_DS_CHECK_TEMP,                 /* ✅ check known temp        [B4-FUTURE] */
  PATH_CND_DS_STEP_TRACE,                 /* 🔴 2 step rays (dir0+dir1)[B4-FUTURE] */
  PATH_CND_DS_STEP_PROCESS,               /* ✅ process hit0/hit1       [B4-FUTURE] */
  PATH_CND_DS_STEP_ENC_VERIFY,            /* 🔴 ENC verify             [B4-FUTURE] */
  PATH_CND_DS_STEP_ADVANCE,               /* ✅ pos update + loop      [B4-FUTURE] */

  /* --- Walk on Spheres (WoS) conductive --- */
  PATH_CND_WOS_CHECK_TEMP,                /* ✅ check known temp        [B4-FUTURE] */
  PATH_CND_WOS_CLOSEST,                   /* 🔴 closest_point query    [B4-FUTURE] */
  PATH_CND_WOS_CLOSEST_RESULT,            /* ✅ ε-shell / diffuse      [B4-FUTURE] */
  PATH_CND_WOS_FALLBACK_TRACE,            /* 🔴 fallback trace_ray    [B4-FUTURE] */
  PATH_CND_WOS_FALLBACK_RESULT,           /* ✅ fallback result        [B4-FUTURE] */
  PATH_CND_WOS_TIME_TRAVEL,               /* ✅ time rewind + loop     [B4-FUTURE] */

  /* --- Custom conductive (Phase 7 plugin) --- */
  PATH_CND_CUSTOM,                        /* ✅ custom callback         [B4-FUTURE] */

  /* === B-4: Convective path === */
  PATH_CNV_INIT,                           /* ✅ fluid temp check        [B4-FUTURE] */
  PATH_CNV_STARTUP_TRACE,                 /* 🔴 1 startup ray          [B4-FUTURE] */
  PATH_CNV_STARTUP_RESULT,                /* ✅ startup result          [B4-FUTURE] */
  PATH_CNV_SAMPLE_LOOP,                   /* ✅ null-collision loop     [B4-FUTURE] */

  /* === B-4: Enclosure query sub-state machine === */
  PATH_ENC_QUERY_EMIT,                    /* 🔴 emit 6 directional rays[B4-FUTURE] */
  PATH_ENC_QUERY_RESOLVE,                 /* ✅ resolve 6 hits → enc_id[B4-FUTURE] */

  /* === Terminal === */
  PATH_DONE,                               /* path finished              */
  PATH_ERROR,                              /* error termination          */

  PATH_PHASE_COUNT                         /* total state count          */
};

/*******************************************************************************
 * Ray bucket type — classifies rays by BVH traversal pattern for
 * warp-coherent GPU batching (Phase B-4).
 ******************************************************************************/
enum ray_bucket_type {
  RAY_BUCKET_RADIATIVE,       /* long-range random direction, range=[ε,∞)    */
  RAY_BUCKET_STEP_PAIR,       /* short-range opposing rays (delta-sphere)     */
  RAY_BUCKET_ENCLOSURE,       /* 6 fixed-direction family, range=[ε,∞)       */
  RAY_BUCKET_SHADOW,          /* fixed-distance shadow ray, range=[0,dist]   */
  RAY_BUCKET_STARTUP,         /* single-direction probe ray                  */
  RAY_BUCKET_COUNT
};

/*******************************************************************************
 * path_phase_is_ray_pending — returns 1 if the phase requires a ray trace
 * result before the path can advance.  Used by cascade loops and stream
 * compaction to avoid attempting no-ray advance on these phases.
 ******************************************************************************/
static INLINE int
path_phase_is_ray_pending(enum path_phase ph)
{
  switch(ph) {
  /* B-2/B-3 legacy ray-pending */
  case PATH_RAD_TRACE_PENDING:
  case PATH_COUPLED_BOUNDARY_REINJECT:
  case PATH_COUPLED_COND_DS_PENDING:
  /* B-4 fine-grained ray-pending */
  case PATH_BND_SS_REINJECT_SAMPLE:
  case PATH_BND_SS_REINJECT_ENC:
  case PATH_BND_SF_REINJECT_SAMPLE:
  case PATH_BND_SF_REINJECT_ENC:
  case PATH_BND_SF_NULLCOLL_RAD_TRACE:
  case PATH_BND_SFN_RAD_TRACE:
  case PATH_BND_EXT_DIRECT_TRACE:
  case PATH_BND_EXT_DIFFUSE_TRACE:
  case PATH_BND_EXT_DIFFUSE_SHADOW_TRACE:
  case PATH_CND_INIT_ENC:
  case PATH_CND_DS_STEP_TRACE:
  case PATH_CND_DS_STEP_ENC_VERIFY:
  case PATH_CND_WOS_CLOSEST:
  case PATH_CND_WOS_FALLBACK_TRACE:
  case PATH_CNV_STARTUP_TRACE:
  case PATH_ENC_QUERY_EMIT:
    return 1;
  default:
    return 0;
  }
}

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
 * B-4 expansion: ~600 B (original) + ~600 B (locals union) + ~200 B
 * (ext_flux) + ~200 B (enc_query) + ~600 B (sfn_stack×3) ≈ 2.2 KB/path.
 * 32K paths × 2.2 KB = 70 MB, well within CPU memory budget.
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

  /* --- Conductive delta-sphere persistent state (wavefront M3) --- */
  int     ds_initialized;         /* 1 = init phase done                   */
  unsigned ds_enc_id;             /* enclosure id for conductive walk       */
  struct sdis_medium* ds_medium;  /* solid medium pointer                   */
  struct solid_props  ds_props_ref; /* reference properties at start        */
  double  ds_green_power_term;    /* accumulated green function power       */
  double  ds_position_start[3];   /* starting position backup              */
  int     ds_robust_attempt;      /* robust retry counter                   */
  float   ds_delta;               /* computed step distance                 */
  float   ds_delta_solid_param;   /* props.delta (medium step parameter)    */

  /* --- Boundary reinjection scratch --- */
  struct s3d_hit bnd_hit0, bnd_hit1;
  double  bnd_reinject_distance;
  unsigned bnd_solid_enc_id;
  int     bnd_retry_count;

  /* --- Filter data for current ray request --- */
  struct hit_filter_data  filter_data_storage;

  /* --- Diagnostics --- */
  size_t  steps_taken;                 /* total steps this path has taken     */
  int     done_reason;                 /* 0=none, 1=rad_miss, 2=temp_known,
                                          3=boundary_done, 4=time_rewind,
                                          -1=failed                          */

  /* --- Ray request output (set by step functions) --- */
  struct path_ray_request ray_req;
  int    needs_ray;                    /* does this step need a trace?        */

  /* --- RNG (shared per-thread, non-owning pointer) --- */
  struct ssp_rng* rng;

  /* --- Image-space pixel coords (in image plane) --- */
  size_t  ipix_image[2];

  /* ===================================================================== */
  /* B-4: Fine-grained state machine extensions                            */
  /* ===================================================================== */

  /* --- B-4: Fine-grained locale variables (union, one branch active) --- */
  union {
    struct {                            /* solid/solid reinjection         */
      float   dir_frt[2][3];           /* front side 2 directions         */
      float   dir_bck[2][3];           /* back side 2 directions          */
      struct s3d_hit ray_frt[2];       /* hit results for front dir0/dir1 */
      struct s3d_hit ray_bck[2];       /* hit results for back dir0/dir1  */
      unsigned enc_ids[2];             /* [FRONT] and [BACK] enclosure ids*/
      double  lambda_frt, lambda_bck;
      double  delta_boundary_frt;      /* reinject distance front side    */
      double  delta_boundary_bck;      /* reinject distance back side     */
      double  tcr;                     /* thermal contact resistance      */
      double  proba;
      int     multi_frt, multi_bck;    /* 1 if enclosure is MEDIUM_ID_MULTI */

      /* Reinjection ray results (mirrors find_reinjection_ray output) */
      float   reinject_dir_frt[3];     /* chosen front reinjection dir    */
      float   reinject_dst_frt;        /* chosen front reinjection dist   */
      struct s3d_hit reinject_hit_frt; /* chosen front reinjection hit    */
      float   reinject_dir_bck[3];     /* chosen back reinjection dir     */
      float   reinject_dst_bck;        /* chosen back reinjection dist    */
      struct s3d_hit reinject_hit_bck; /* chosen back reinjection hit     */

      /* Enclosure ids at reinjection endpoints (from trace or ENC query) */
      unsigned enc0_frt, enc1_frt;     /* enc_id along frt dir0, dir1     */
      unsigned enc0_bck, enc1_bck;     /* enc_id along bck dir0, dir1     */
      int      need_enc_frt, need_enc_bck; /* which side needs ENC query  */
      int      enc_side;               /* 0=frt,1=bck for current ENC    */

      /* Position backup for retry logic */
      double  rwalk_pos_backup[3];
      int     position_was_moved;      /* did find_reinjection_ray move?  */
      int     retry_count;             /* attempt counter (MAX=10)        */

      /* Batch indices for 4-ray collect/distribute */
      uint32_t batch_idx_frt0, batch_idx_frt1;
      uint32_t batch_idx_bck0, batch_idx_bck1;
    } bnd_ss;

    struct {                            /* solid/fluid picard1/N           */
      double  p_conv, p_cond, p_radi;
      double  h_hat, h_conv, h_cond;
      double  epsilon, Tref;
      float   reinject_dir[2][3];
      struct s3d_hit reinject_hit[2];
      unsigned solid_enc_id;
      double  r;                       /* saved random number              */
      /* null-collision sub-path snapshot */
      struct rwalk      rwalk_snapshot;
      struct temperature T_snapshot;
    } bnd_sf;

    struct {                            /* WoS conductive                  */
      double  query_pos[3];
      double  query_radius;
      float   new_pos[3];
      float   dir[3];
      struct s3d_hit cached_hit;
      double  delta;
      double  last_distance;
    } cnd_wos;

    struct {                            /* convective path                 */
      unsigned enc_id;
      double  hc_upper_bound;
      double  rho_cp;
      double  S_over_V;
    } cnv;
  } locals;

  /* --- B-4: External net flux (independent, co-active with picard) --- */
  struct {
    float   pos[3], dir[3];
    float   range;
    struct s3d_hit hit;
    double  flux_direct;
    double  flux_diffuse_reflected;
    double  flux_scattered;
    int     nbounces;
    enum path_phase return_state;      /* resume state after flux done     */
  } ext_flux;

  /* --- B-4: Enclosure query sub-state (M1) --- */
  struct {
    float   directions[6][3];          /* 6 rotated axis directions        */
    struct s3d_hit dir_hits[6];        /* 6 directional ray results        */
    uint32_t batch_indices[6];         /* batch array index per direction   */
    enum path_phase return_state;      /* resume state after ENC done      */
    unsigned resolved_enc_id;          /* result enclosure id              */
    double   query_pos[3];            /* position used for query           */
  } enc_query;

  /* --- B-4: PicardN recursive stack --- */
  struct {
    enum path_phase return_state;
    double  partial_temperature;
    struct rwalk rwalk_saved;
    struct temperature T_saved;
    double  T_values[6];
    int     T_count;
    double  r, p_conv, p_cond, h_hat;
  } sfn_stack[3];                      /* MAX_PICARD_DEPTH = 3             */
  int sfn_stack_depth;

  /* --- B-4: Ray type tag --- */
  enum ray_bucket_type ray_bucket;     /* bucket for current ray request   */
  int  ray_count_ext;                  /* extended ray count (ENC=6, etc.) */
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

#endif /* SDIS_SOLVE_WAVEFRONT_H */
