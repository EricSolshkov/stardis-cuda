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

/* Wavefront per-path state structures.
 *
 * Extracted from sdis_solve_wavefront.h.  Contains the fully externalised
 * state of one Monte-Carlo path (struct path_state) and its ray request
 * descriptor (struct path_ray_request).
 */

#ifndef SDIS_WF_STATE_H
#define SDIS_WF_STATE_H

#include "sdis_wf_types.h"  /* enum path_phase, enum ray_bucket_type        */
#include "sdis_wf_rng.h"   /* struct wf_rng, per-path CBRNG                */
#include "sdis_heat_path.h" /* struct rwalk, rwalk_context, temperature      */
#include "sdis_medium_c.h"  /* struct solid_props                            */
#include "sdis_scene_c.h"   /* struct hit_filter_data                        */
#include "sdis_source_c.h"  /* struct source_props, source_sample (M7)       */

#include <star/s3d.h>        /* struct s3d_hit                                */
#include <star/ssp.h>        /* struct ssp_rng                                */
#include <stdint.h>          /* uint32_t, uint16_t                            */

/* Forward declarations (pointer-only usage in path_state) */
struct sdis_medium;
struct green_path_handle;

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
 * (ext_flux) + ~200 B (enc_query) + ~600 B (sfn_stack x 3) ~ 2.2 KB/path.
 * 32K paths x 2.2 KB = 70 MB, well within CPU memory budget.
 ******************************************************************************/

/* Maximum recursion depth for picardN COMPUTE_TEMPERATURE stack.
 * 3 = up to 3 nested boundary sub-paths.  If the depth is exceeded during
 * a recursive Ti sampling, the path falls back to synchronous execution. */
#define MAX_PICARD_DEPTH 3
struct path_state {
  /* --- Identity --- */
  uint32_t  path_id;                   /* global unique id within the tile   */
  uint16_t  pixel_x, pixel_y;         /* pixel coords (tile-local)          */
  uint32_t  realisation_idx;           /* SPP index                          */

  /* --- Lifecycle --- */
  enum path_phase  phase;
  int              active;             /* 1 = still running                  */

  /* --- Random walk core --- */
  struct rwalk          rwalk;         /* position, time, hit, enc_id ...    */
  struct rwalk_context  ctx;           /* Tmin/That/branchings ...           */

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

  /* --- Per-path CBRNG state (inline, keyed by pixel+spp+seed) --- */
  struct wf_rng   rng_state;

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

    struct path_bnd_sf_locals {         /* solid/fluid picard1/N           */
      double  p_conv, p_cond, p_radi;
      double  h_hat, h_conv, h_cond;
      double  h_radi_hat;              /* radiative coeff upper bound      */
      double  epsilon, Tref;
      double  lambda;                  /* solid thermal conductivity       */
      double  delta_boundary;          /* sqrt(DIM)*delta                  */
      double  delta_m;                 /* delta in meters                  */
      float   reinject_dir[2][3];      /* 2 candidate directions           */
      struct s3d_hit reinject_hit[2];  /* 2 hit results                    */
      float   chosen_dir[3];          /* resolved reinjection direction    */
      float   chosen_dst;             /* resolved reinjection distance     */
      struct s3d_hit chosen_hit;      /* resolved reinjection hit          */
      unsigned solid_enc_id;
      unsigned enc_ids[2];            /* [FRONT] and [BACK] enclosure ids  */
      unsigned enc0_id, enc1_id;      /* enclosure ids at ray endpoints    */
      enum sdis_side solid_side;      /* which side is solid               */
      enum sdis_side fluid_side;      /* which side is fluid               */
      int     need_enc;               /* whether ENC query is needed       */
      int     retry_count;            /* reinjection retry counter         */
      double  rwalk_pos_backup[3];    /* position backup for retry logic   */
      double  r;                       /* saved random number              */
      /* null-collision radiative sub-path state */
      float   rad_sub_direction[3];   /* current radiative direction       */
      int     rad_sub_bounce_count;   /* bounce counter                    */
      int     rad_sub_retry_count;    /* find_next_fragment retries        */
      struct rwalk      rwalk_snapshot;
      struct temperature T_snapshot;
      /* heat path restart vertex saved before null-collision loop */
      struct sdis_heat_vertex hvtx_saved;
      size_t  ihvtx_radi_begin;       /* radiative sub-path vertex start   */

      /* === PicardN-specific fields (M8) === */
      int     is_picardn;             /* 1 when in picardN mode            */
      struct rwalk rwalk_s;            /* radiative endpoint rwalk (SFN)    */
      struct temperature T_s;          /* radiative endpoint temperature    */
      struct sdis_heat_vertex hvtx_s;  /* heat vertex at radiative endpoint */
      size_t  ihvtx_radi_end;          /* vertex index after rad sub-path   */
      int     coupled_nbranchings_saved; /* saved nbranchings at push       */
    } bnd_sf;

    struct {                            /* WoS conductive                  */
      double  query_pos[3];
      double  query_radius;
      float   new_pos[3];
      float   dir[3];
      struct s3d_hit cached_hit;        /* closest_point hit (for snap)    */
      double  delta;                    /* material delta parameter        */
      double  last_distance;            /* wos_distance from closest query */
      double  alpha;                    /* diffusivity = lambda/(rho*cp)   */
      unsigned enc_id;                  /* enclosure id for WoS walk       */
      struct sdis_medium* medium;       /* solid medium pointer            */
      struct solid_props props_ref;     /* reference properties at start   */
      struct solid_props props;         /* current position properties     */
      double  green_power_term;         /* accumulated green function pwr  */
      double  position_start[3];        /* pos before diffusion step       */
      int     wos_initialized;          /* 1 = init phase done             */
      uint32_t batch_cp_idx;            /* index in closest_point batch    */
    } cnd_wos;

    struct {                            /* convective path                 */
      unsigned enc_id;
      double  hc_upper_bound;
      double  rho_cp;
      double  S_over_V;
    } cnv;
  } locals;

  /* --- B-4 M7: External net flux (independent, co-active with picard) --- */
  struct {
    /* Ray data for current shadow / diffuse ray */
    float   pos[3];                    /* current bounce position (float)  */
    float   dir[3];                    /* current diffuse bounce direction  */
    float   range;                     /* ray range (shadow: src dist)     */
    struct s3d_hit hit;                /* current hit from boundary/bounce  */

    /* Accumulated flux components */
    double  flux_direct;               /* incident_flux_direct [W/m^2]     */
    double  flux_diffuse_reflected;    /* sum of L contributions [W/m^2/sr]*/
    double  flux_scattered;            /* PI if miss (scattered component) */
    double  scattered_dir[3];          /* direction of scattered component  */
    int     nbounces;                  /* bounce counter                    */

    /* Persistent state across suspend points */
    double  emissivity;                /* interface emissivity for source   */
    double  sum_h;                     /* h_cond + h_conv + h_radi          */
    double  cos_theta;                 /* N . src_sample.dir                */
    double  N[3];                      /* outward normal (toward fluid)     */
    unsigned enc_id_fluid;             /* enclosure id on fluid side        */
    struct source_props  src_props;    /* source properties at frag.time    */
    struct source_sample src_sample;   /* sampled direction toward source   */

    /* Fragment info for BRDF at bounce */
    double  frag_time;                 /* fragment time [s]                 */
    double  frag_P[3];                 /* original fragment position        */

    /* Green function handle (non-owning pointer) */
    struct green_path_handle* green_path;

    /* Return state after flux computation is done */
    enum path_phase return_state;      /* resume state after flux done     */
  } ext_flux;

  /* --- B-4 M10: Point-in-enclosure via BVH closest primitive --- */
  struct {
    double   query_pos[3];            /* position for enclosure locate     */
    enum path_phase return_state;     /* resume state after locate done    */
    unsigned resolved_enc_id;         /* result enclosure id               */
    int32_t  prim_id;                 /* closest prim from GPU kernel      */
    int32_t  side;                    /* 0=front, 1=back, -1=degenerate    */
    float    distance;                /* distance to closest surface       */
    uint32_t batch_idx;               /* index in enc_locate batch         */
  } enc_locate;

  /* --- B-4 M1-v2: 6-ray enclosure query (replaces M10 at call sites) --- */
  struct {
    double   query_pos[3];            /* query position                    */
    enum path_phase return_state;     /* resume state after resolve done   */
    unsigned resolved_enc_id;         /* result enclosure id               */
    float    directions[6][3];        /* PI/4 rotated axis-aligned dirs    */
    struct s3d_hit dir_hits[6];       /* hit results for 6 rays            */
    uint32_t batch_indices[6];        /* batch indices for each ray        */
    /* Fallback ray (when all 6 primary rays invalid) */
    float    fb_direction[3];         /* fallback ray direction            */
    struct s3d_hit fb_hit;            /* fallback ray hit result           */
    uint32_t fb_batch_idx;            /* fallback ray batch index          */
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
    struct path_bnd_sf_locals bnd_sf_backup;
  } sfn_stack[MAX_PICARD_DEPTH];        /* recursive COMPUTE_TEMPERATURE    */
  int sfn_stack_depth;

  /* --- B-4: Ray type tag --- */
  enum ray_bucket_type ray_bucket;     /* bucket for current ray request   */
  int  ray_count_ext;                  /* extended ray count (ENC=6, etc.) */
};

#endif /* SDIS_WF_STATE_H */
