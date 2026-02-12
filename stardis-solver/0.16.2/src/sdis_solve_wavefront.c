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
#include "sdis.h"
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
 * check_interface — copied from sdis_heat_path_radiative_Xd.h because it is
 * static in that header and not accessible from this translation unit.
 * This function is DIM-independent.
 ******************************************************************************/
static res_T
wf_check_interface
  (const struct sdis_interface* interf,
   const struct sdis_interface_fragment* frag,
   const int verbose)
{
  enum sdis_medium_type mdm_frt_type = SDIS_MEDIUM_TYPES_COUNT__;
  enum sdis_medium_type mdm_bck_type = SDIS_MEDIUM_TYPES_COUNT__;
  enum sdis_side fluid_side = SDIS_SIDE_NULL__;
  res_T res = RES_OK;

  mdm_frt_type = sdis_medium_get_type(interf->medium_front);
  mdm_bck_type = sdis_medium_get_type(interf->medium_back);

  if(mdm_frt_type == SDIS_SOLID && mdm_bck_type == SDIS_SOLID) {
    if(verbose) {
      log_err(interf->dev,
        "Error when sampling the radiative path. The trajectory reaches a "
        "solid/solid interface, whereas this is supposed to be impossible "
        "(path position: %g, %g, %g).\n",
        SPLIT3(frag->P));
    }
    res = RES_BAD_OP;
    goto error;
  }

  if(mdm_frt_type == SDIS_FLUID) {
    fluid_side = SDIS_FRONT;
  } else if(mdm_bck_type == SDIS_FLUID) {
    fluid_side = SDIS_BACK;
  } else {
    FATAL("Unreachable code\n");
  }

  if(frag->side != fluid_side) {
    if(verbose) {
      log_err(interf->dev,
        "Inconsistent intersection when sampling the radiative path. "
        "The path reaches an interface on its solid side, whereas this is "
        "supposed to be impossible (path position: %g, %g, %g).\n",
        SPLIT3(frag->P));
    }
    res = RES_BAD_OP;
    goto error;
  }

exit:
  return res;
error:
  goto exit;
}

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

  /* Each path can request up to 2 rays per step */
  wf->max_rays = total_paths * 2;
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

/*******************************************************************************
 * Ray-request setup helpers (matches trace_ray / sample_next_step API)
 ******************************************************************************/
static void
setup_radiative_trace_ray(struct path_state* p, struct sdis_scene* scn)
{
  float pos[3];
  ASSERT(p && scn);

  f3_set_d3(pos, p->rwalk.vtx.P);

  p->ray_req.origin[0] = pos[0];
  p->ray_req.origin[1] = pos[1];
  p->ray_req.origin[2] = pos[2];
  p->ray_req.direction[0] = p->rad_direction[0];
  p->ray_req.direction[1] = p->rad_direction[1];
  p->ray_req.direction[2] = p->rad_direction[2];
  p->ray_req.range[0] = 0.0f;
  p->ray_req.range[1] = FLT_MAX;
  p->ray_req.ray_count = 1;

  /* Filter: self-intersection avoidance + enclosure matching */
  p->filter_data_storage = HIT_FILTER_DATA_NULL;
  p->filter_data_storage.hit_3d = p->rwalk.hit_3d;
  p->filter_data_storage.epsilon = 1.e-6;
  p->filter_data_storage.scn = scn;
  p->filter_data_storage.enc_id = p->rwalk.enc_id;

  p->needs_ray = 1;
}

static void
setup_delta_sphere_rays(struct path_state* p, struct sdis_scene* scn)
{
  float pos[3];
  float dir[3];
  ASSERT(p && scn);
  (void)scn;

  f3_set_d3(pos, p->rwalk.vtx.P);

  /* Random sphere direction — matches sample_next_step's RNG call */
  ssp_ran_sphere_uniform_float(p->rng, dir, NULL);

  p->ds_dir0[0] = dir[0];
  p->ds_dir0[1] = dir[1];
  p->ds_dir0[2] = dir[2];
  p->ds_dir1[0] = -dir[0];
  p->ds_dir1[1] = -dir[1];
  p->ds_dir1[2] = -dir[2];

  /* Ray 0: forward */
  p->ray_req.origin[0] = pos[0];
  p->ray_req.origin[1] = pos[1];
  p->ray_req.origin[2] = pos[2];
  p->ray_req.direction[0] = p->ds_dir0[0];
  p->ray_req.direction[1] = p->ds_dir0[1];
  p->ray_req.direction[2] = p->ds_dir0[2];
  p->ray_req.range[0] = FLT_MIN;
  p->ray_req.range[1] = p->ds_delta_solid_param * RAY_RANGE_MAX_SCALE;

  /* Ray 1: backward */
  p->ray_req.direction2[0] = p->ds_dir1[0];
  p->ray_req.direction2[1] = p->ds_dir1[1];
  p->ray_req.direction2[2] = p->ds_dir1[2];
  p->ray_req.range2[0] = FLT_MIN;
  p->ray_req.range2[1] = p->ds_delta_solid_param * RAY_RANGE_MAX_SCALE;

  p->ray_req.ray_count = 2;
  p->needs_ray = 1;
}

static void
setup_convective_startup_ray(struct path_state* p)
{
  float pos[3];
  ASSERT(p);

  f3_set_d3(pos, p->rwalk.vtx.P);

  p->ray_req.origin[0] = pos[0];
  p->ray_req.origin[1] = pos[1];
  p->ray_req.origin[2] = pos[2];
  /* Shoot along +Z for startup hit (same as handle_convective_path_startup) */
  p->ray_req.direction[0] = 0.0f;
  p->ray_req.direction[1] = 0.0f;
  p->ray_req.direction[2] = 1.0f;
  p->ray_req.range[0] = FLT_MIN;
  p->ray_req.range[1] = FLT_MAX;
  p->ray_req.ray_count = 1;
  p->needs_ray = 1;
}

/*******************************************************************************
 * Step functions — each one advances a path by exactly one logical step.
 *
 * If the step produces a ray request, it sets p->needs_ray = 1 and
 * populates p->ray_req.  On the next wavefront iteration the batch trace
 * result is delivered to the path and the appropriate step function is
 * called again.
 *
 * Steps that do NOT require ray tracing may cascade: the wavefront loop
 * calls them repeatedly in the same iteration until either a ray is needed
 * or the path finishes.
 ******************************************************************************/

/* --- PATH_INIT: set up first radiative trace ----------------------------- */
static res_T
step_init(struct path_state* p, struct sdis_scene* scn)
{
  ASSERT(p && scn);

  /* Register starting vertex (skip heat_path for now) */

  /* Emit the first radiative trace ray */
  setup_radiative_trace_ray(p, scn);
  p->phase = PATH_RAD_TRACE_PENDING;
  return RES_OK;
}

/* --- PATH_RAD_TRACE_PENDING → process hit / BRDF decision ---------------- */
static res_T
step_radiative_trace(
  struct path_state* p,
  struct sdis_scene* scn,
  const struct s3d_hit* trace_hit)
{
  res_T res = RES_OK;
  ASSERT(p && scn && trace_hit);

  /* --- Miss → radiative environment temperature → done --- */
  if(S3D_HIT_NONE(trace_hit)) {
    struct sdis_radiative_ray ray = SDIS_RADIATIVE_RAY_NULL;
    double dir_d[3];
    double trad;

    d3_set_f3(dir_d, p->rad_direction);
    d3_normalize(dir_d, dir_d);

    d3_set(ray.dir, dir_d);
    ray.time = p->rwalk.vtx.time;

    trad = radiative_env_get_temperature(scn->radenv, &ray);
    if(SDIS_TEMPERATURE_IS_UNKNOWN(trad)) {
      log_err(scn->dev,
        "wavefront: radiative path reached unknown environment from "
        "(%g, %g, %g) along (%g, %g, %g)\n",
        SPLIT3(p->rwalk.vtx.P), SPLIT3(dir_d));
      res = RES_BAD_OP;
      goto error;
    }

    p->T.value += trad;
    p->T.done = 1;
    p->phase = PATH_DONE;
    p->active = 0;
    p->needs_ray = 0;
    p->done_reason = 1; /* radiative miss */
    goto exit;
  }

  /* --- Hit handling --- */
  {
    struct sdis_interface_fragment frag = SDIS_INTERFACE_FRAGMENT_NULL;
    struct sdis_interface* interf = NULL;
    struct sdis_medium* chk_mdm = NULL;
    struct brdf brdf = BRDF_NULL;
    struct brdf_sample bounce = BRDF_SAMPLE_NULL;
    struct brdf_setup_args brdf_args = BRDF_SETUP_ARGS_NULL;
    double dir_d[3], pos_d[3], N[3], wi[3];

    /* Update hit */
    p->rwalk.hit_3d = *trace_hit;

    /* Compute new position from fragment */
    d3_set_f3(dir_d, p->rad_direction);
    d3_normalize(dir_d, dir_d);
    d3_set(pos_d, p->rwalk.vtx.P);

    /* Retrieve position, normal, interface */
    {
      double vec[3];
      d3_add(pos_d, pos_d, d3_muld(vec, dir_d, trace_hit->distance));
    }
    d3_set(p->rwalk.vtx.P, pos_d);

    /* Normal */
    d3_set_f3(N, trace_hit->normal);
    d3_normalize(N, N);

    /* Determine hit side */
    p->rwalk.hit_side = d3_dot(dir_d, N) < 0 ? SDIS_FRONT : SDIS_BACK;

    /* Get interface */
    interf = scene_get_interface(scn, trace_hit->prim.prim_id);

    /* Setup fragment */
    {
      struct sdis_rwalk_vertex vtx = SDIS_RWALK_VERTEX_NULL;
      d3_set(vtx.P, pos_d);
      vtx.time = p->rwalk.vtx.time;
      setup_interface_fragment_3d(&frag, &vtx, &p->rwalk.hit_3d,
                                  p->rwalk.hit_side);
    }

    /* Check interface validity (fluid/solid boundary from fluid side) */
    res = wf_check_interface(interf, &frag, p->rad_retry_count >= 9);
    if(res != RES_OK) {
      if(p->rad_retry_count < 9) {
        struct sdis_medium* solid;
        double delta;

        p->rad_retry_count++;

        /* Determine which side is solid */
        if(sdis_medium_get_type(interf->medium_front) == SDIS_SOLID)
          solid = interf->medium_front;
        else
          solid = interf->medium_back;

        delta = solid_get_delta(solid, &p->rwalk.vtx);
        move_away_primitive_boundaries_3d(&p->rwalk.hit_3d, delta,
                                          p->rwalk.vtx.P);

        /* Re-trace from adjusted position */
        f3_set_d3(p->rad_direction, dir_d);
        setup_radiative_trace_ray(p, scn);
        p->phase = PATH_RAD_TRACE_PENDING;
        goto exit;
      }
      goto error;
    }
    p->rad_retry_count = 0;

    /* Check medium consistency: the path must be in a fluid */
    chk_mdm = p->rwalk.hit_side == SDIS_FRONT
      ? interf->medium_front : interf->medium_back;
    if(sdis_medium_get_type(chk_mdm) == SDIS_SOLID) {
      log_err(scn->dev,
        "wavefront: radiative path in solid -- pos=(%g, %g, %g)\n",
        SPLIT3(p->rwalk.vtx.P));
      res = RES_BAD_OP_IRRECOVERABLE;
      goto error;
    }

    /* BRDF at intersection */
    brdf_args.interf = interf;
    brdf_args.frag = &frag;
    brdf_args.source_id = SDIS_INTERN_SOURCE_ID;
    res = brdf_setup(scn->dev, &brdf_args, &brdf);
    if(res != RES_OK) goto error;

    /* Absorption test: switch to boundary_path? */
    if(ssp_rng_canonical(p->rng) < brdf.emissivity) {
      /* Absorbed → enter boundary path */
      p->T.func = boundary_path_3d;
      p->rwalk.enc_id = ENCLOSURE_ID_NULL;
      p->phase = PATH_COUPLED_BOUNDARY;
      p->needs_ray = 0;
      goto exit;
    }

    /* BRDF reflection: sample new direction */
    p->rad_bounce_count++;
    d3_minus(wi, dir_d);
    /* Ensure N points toward current medium */
    switch(p->rwalk.hit_side) {
    case SDIS_FRONT:
      /* N already correct */
      break;
    case SDIS_BACK:
      d3_minus(N, N);
      break;
    default:
      FATAL("Unreachable code\n");
      break;
    }
    brdf_sample(&brdf, p->rng, wi, N, &bounce);
    f3_set_d3(p->rad_direction, bounce.dir);

    /* Set up next trace */
    setup_radiative_trace_ray(p, scn);
    p->phase = PATH_RAD_TRACE_PENDING;
  }

exit:
  return res;
error:
  goto exit;
}

/* --- PATH_COUPLED_BOUNDARY: run boundary_path logic (no ray needed) ------ */
static res_T
step_boundary(struct path_state* p, struct sdis_scene* scn)
{
  res_T res = RES_OK;
  ASSERT(p && scn);

  /* Delegate to original boundary_path via the temperature function pointer.
   * boundary_path itself does not call trace_ray — the sub-functions
   * (solid_solid, solid_fluid_picard1/N) eventually set T.func to conductive,
   * convective, or radiative, or T.done = 1.
   *
   * Because boundary_path is self-contained (no trace_ray call), we can
   * execute it synchronously in a single step. */

  /* Replicate sample_coupled_path's nbranchings management.
   *
   * In the original code, sample_coupled_path increments nbranchings ONCE
   * at entry, then the while(!T->done) loop calls T->func (boundary_path,
   * conductive_path, etc.) repeatedly WITHOUT changing nbranchings.
   * Deeper recursive calls (picardN → sample_coupled_path) handle their
   * own increment/decrement on ctx->nbranchings via the call stack.
   *
   * In wavefront mode, each call to step_boundary / step_conductive /
   * step_convective corresponds to one iteration of that while loop.
   * We only increment on the FIRST entry (sentinel -1 → 0), and simply
   * restore the saved value on re-entries within the same level. */
  if(p->coupled_nbranchings < 0) {
    /* First entry into coupled path: simulate SIZE_MAX + 1 = 0 */
    p->ctx.nbranchings = 0;
    p->coupled_nbranchings = 0;
  } else {
    /* Re-entry within same sample_coupled_path level — restore, no increment */
    p->ctx.nbranchings = (size_t)p->coupled_nbranchings;
  }

  if(p->ctx.nbranchings > p->ctx.max_branchings) {
    /* Exceeded picard recursion limit (should not happen with correct logic) */
    log_err(scn->dev,
      "wavefront: exceeded max_branchings (%lu) at pixel (%u,%u)\n",
      (unsigned long)p->ctx.max_branchings, p->pixel_x, p->pixel_y);
    res = RES_BAD_OP;
    p->phase = PATH_DONE;
    p->active = 0;
    goto error;
  }

  /* Call boundary_path (synchronous: it does not call trace_ray).
   * Internal recursion (picardN → sample_coupled_path) may temporarily
   * modify ctx->nbranchings but restores it before returning. */
  res = boundary_path_3d(scn, &p->ctx, &p->rwalk, p->rng, &p->T);
  /* Persist nbranchings (in case internal recursion changed our level) */
  p->coupled_nbranchings = (int)p->ctx.nbranchings;
  if(res != RES_OK && res != RES_BAD_OP) goto error;
  if(res == RES_BAD_OP) {
    /* Path failure — mark as done */
    p->phase = PATH_DONE;
    p->active = 0;
    goto exit;
  }

  /* Check where boundary_path sent us */
  if(p->T.done) {
    p->phase = PATH_DONE;
    p->active = 0;
    p->done_reason = 3; /* boundary done */
    goto exit;
  }
  if(p->T.func == conductive_path_3d) {
    p->phase = PATH_COUPLED_CONDUCTIVE;
  } else if(p->T.func == convective_path_3d) {
    p->phase = PATH_COUPLED_CONVECTIVE;
  } else if(p->T.func == radiative_path_3d) {
    p->phase = PATH_COUPLED_RADIATIVE;
  } else if(p->T.func == boundary_path_3d) {
    /* Still boundary → re-enter */
    p->phase = PATH_COUPLED_BOUNDARY;
  } else {
    FATAL("wavefront: unexpected T.func after boundary_path\n");
  }

exit:
  return res;
error:
  goto exit;
}

/* --- PATH_COUPLED_CONDUCTIVE: wavefront delta_sphere entry/loop --------- */
static res_T
step_conductive(struct path_state* p, struct sdis_scene* scn)
{
  res_T res = RES_OK;
  ASSERT(p && scn);

  /* Custom paths and WoS: still delegate to the synchronous implementation.
   * Only delta_sphere is wavefront-ized. */
  if(p->ctx.diff_algo != SDIS_DIFFUSION_DELTA_SPHERE) {
    /* WoS or custom — synchronous fallback */
    res = conductive_path_3d(scn, &p->ctx, &p->rwalk, p->rng, &p->T);
    if(res != RES_OK && res != RES_BAD_OP) goto error;
    if(res == RES_BAD_OP) {
      p->phase = PATH_DONE;
      p->active = 0;
      goto exit;
    }
    if(p->T.done) {
      p->phase = PATH_DONE;
      p->active = 0;
    } else if(p->T.func == boundary_path_3d) {
      p->phase = PATH_COUPLED_BOUNDARY;
    } else if(p->T.func == convective_path_3d) {
      p->phase = PATH_COUPLED_CONVECTIVE;
    } else if(p->T.func == radiative_path_3d) {
      p->phase = PATH_COUPLED_RADIATIVE;
    } else {
      FATAL("wavefront: unexpected T.func after conductive_path (fallback)\n");
    }
    goto exit;
  }

  /* ----- Delta-sphere wavefront path ----- */

  /* Initialization (run once per conductive entry) */
  if(!p->ds_initialized) {
    unsigned enc_id = ENCLOSURE_ID_NULL;
    struct sdis_medium* mdm = NULL;

    res = scene_get_enclosure_id_in_closed_boundaries(
      scn, p->rwalk.vtx.P, &enc_id);
    if(res != RES_OK) goto error;
    res = scene_get_enclosure_medium(
      scn, scene_get_enclosure(scn, enc_id), &mdm);
    if(res != RES_OK) goto error;

    /* Must be a solid medium */
    if(sdis_medium_get_type(mdm) != SDIS_SOLID) {
      log_err(scn->dev,
        "wavefront: conductive_path in non-solid medium at "
        "(%g, %g, %g)\n", SPLIT3(p->rwalk.vtx.P));
      res = RES_BAD_OP;
      goto error;
    }

    /* Check enclosure consistency */
    if(enc_id != p->rwalk.enc_id) {
      log_err(scn->dev,
        "wavefront: conductive_path enclosure mismatch at "
        "(%g, %g, %g)\n", SPLIT3(p->rwalk.vtx.P));
      res = RES_BAD_OP_IRRECOVERABLE;
      goto error;
    }

    p->ds_enc_id = enc_id;
    p->ds_medium = mdm;
    d3_set(p->ds_position_start, p->rwalk.vtx.P);
    solid_get_properties(mdm, &p->rwalk.vtx, &p->ds_props_ref);
    p->ds_green_power_term = 0;
    p->ds_initialized = 1;
    p->ds_robust_attempt = 0;
  }

  /* --- Each loop iteration of the do-while --- */
  {
    struct solid_props props = SOLID_PROPS_NULL;

    /* Fetch current properties */
    res = solid_get_properties(p->ds_medium, &p->rwalk.vtx, &props);
    if(res != RES_OK) goto error;
    res = check_solid_constant_properties(
      scn->dev, p->ctx.green_path != NULL, 0, &p->ds_props_ref, &props);
    if(res != RES_OK) goto error;

    /* Check temperature limit condition */
    if(SDIS_TEMPERATURE_IS_KNOWN(props.temperature)) {
      p->T.value += props.temperature;
      p->T.done = 1;
      if(p->ctx.heat_path) {
        heat_path_get_last_vertex(p->ctx.heat_path)->weight = p->T.value;
      }
      p->phase = PATH_DONE;
      p->active = 0;
      p->done_reason = 2; /* temperature known */
      goto exit;
    }

    /* Store delta_solid parameter for ray setup */
    p->ds_delta_solid_param = (float)props.delta;
    p->ds_robust_attempt = 0;

    /* Emit 2 rays (forward + backward) */
    setup_delta_sphere_rays(p, scn);
    p->phase = PATH_COUPLED_COND_DS_PENDING;
  }

exit:
  return res;
error:
  /* Mark path as failed */
  p->phase = PATH_DONE;
  p->active = 0;
  p->done_reason = -1;
  goto exit;
}

/* --- PATH_COUPLED_COND_DS_PENDING: process 2-ray results (delta sphere) -- */
static res_T
step_conductive_ds_process(
  struct path_state* p,
  struct sdis_scene* scn,
  const struct s3d_hit* hit0,
  const struct s3d_hit* hit1)
{
  res_T res = RES_OK;
  float delta;
  double delta_m, mu;
  struct solid_props props = SOLID_PROPS_NULL;
  const float delta_solid = p->ds_delta_solid_param;

  ASSERT(p && scn && hit0 && hit1);

  p->ds_hit0 = *hit0;
  p->ds_hit1 = *hit1;

  /* ------ Replicate sample_next_step logic ------ */
  /* Compute delta = min of the two hit distances */
  if(S3D_HIT_NONE(hit0) && S3D_HIT_NONE(hit1)) {
    delta = delta_solid;
  } else {
    float d0 = S3D_HIT_NONE(hit0) ? delta_solid * RAY_RANGE_MAX_SCALE
                                   : hit0->distance;
    float d1 = S3D_HIT_NONE(hit1) ? delta_solid * RAY_RANGE_MAX_SCALE
                                   : hit1->distance;
    delta = MMIN(d0, d1);
  }

  /* Fix delta ≈ hit0.distance edge case */
  if(!S3D_HIT_NONE(hit0)
  && delta != hit0->distance
  && fabs(hit0->distance - delta) < delta_solid * 0.1f) {
    delta = hit0->distance;
  }

  /* Handle snap-to-boundary: if delta <= delta_solid*0.1 and hit1 closer */
  if(delta <= delta_solid * 0.1f
  && !S3D_HIT_NONE(hit1) && hit1->distance == delta) {
    /* Swap: walk toward hit1 (the backward direction) */
    float tmp[3];
    struct s3d_hit tmp_hit;
    f3_set(tmp, p->ds_dir0);
    f3_set(p->ds_dir0, p->ds_dir1);
    f3_set(p->ds_dir1, tmp);
    tmp_hit = p->ds_hit0;
    p->ds_hit0 = p->ds_hit1;
    p->ds_hit1 = tmp_hit;
    /* Recompute hit0 reference */
    hit0 = &p->ds_hit0;
    hit1 = &p->ds_hit1;
  } else if(delta == hit0->distance) {
    /* dir1 = dir0 (both track the same hit) */
    f3_set(p->ds_dir1, p->ds_dir0);
    p->ds_hit1 = *hit0;
  } else if(!S3D_HIT_NONE(hit1) && delta == hit1->distance) {
    /* dir1 tracks the backward hit */
    f3_set(p->ds_dir1, p->ds_dir1); /* already correct */
    p->ds_hit1 = *hit1;
  } else {
    /* No intersection drove delta — dir1 doesn't correspond to a hit */
    p->ds_dir1[0] = 0; p->ds_dir1[1] = 0; p->ds_dir1[2] = 0;
    p->ds_hit1 = S3D_HIT_NULL;
  }
  p->ds_delta = delta;

  /* ------ Replicate sample_next_step_robust enclosure check ------ */
  {
    double pos_next[3];
    unsigned enc_id = ENCLOSURE_ID_NULL;

    if(S3D_HIT_NONE(&p->ds_hit0) || p->ds_hit0.distance > delta) {
      /* No hit in forward direction at delta — check next position */
      d3_set(pos_next, p->rwalk.vtx.P);
      move_pos_3d(pos_next, p->ds_dir0, delta);
      res = scene_get_enclosure_id_in_closed_boundaries(
        scn, pos_next, &enc_id);
      if(res == RES_BAD_OP) { enc_id = ENCLOSURE_ID_NULL; res = RES_OK; }
      if(res != RES_OK) goto error;
    } else {
      /* Hit at forward — get enclosure from hit primitive */
      unsigned enc_ids[2] = {ENCLOSURE_ID_NULL, ENCLOSURE_ID_NULL};
      scene_get_enclosure_ids(scn, p->ds_hit0.prim.prim_id, enc_ids);
      enc_id = f3_dot(p->ds_dir0, p->ds_hit0.normal) < 0
             ? enc_ids[0] : enc_ids[1];
    }

    if(enc_id != p->ds_enc_id) {
      /* Enclosure mismatch — retry with new direction */
      p->ds_robust_attempt++;
      if(p->ds_robust_attempt >= 100) {
        log_warn(scn->dev,
          "wavefront: conductive delta_sphere robust exceeded 100 attempts "
          "at (%g, %g, %g)\n", SPLIT3(p->rwalk.vtx.P));
        res = RES_BAD_OP;
        goto error;
      }
      /* Re-emit 2 rays with new random direction */
      setup_delta_sphere_rays(p, scn);
      p->phase = PATH_COUPLED_COND_DS_PENDING;
      goto exit;
    }
  }

  /* ------ Robust check passed — proceed with step ------ */
  res = solid_get_properties(p->ds_medium, &p->rwalk.vtx, &props);
  if(res != RES_OK) goto error;

  /* Handle volumic power */
  if(props.power != SDIS_VOLUMIC_POWER_NONE) {
    double power_term = 0;
    /* Inline handle_volumic_power logic for 3D */
    if(S3D_HIT_NONE(&p->ds_hit0) && S3D_HIT_NONE(&p->ds_hit1)) {
      double dim = (double)delta * scn->fp_to_meter;
      power_term = dim * dim / (2.0 * 3 * props.lambda);
      p->T.value += props.power * power_term;
    } else {
      const double delta_s_adjusted = delta_solid * RAY_RANGE_MAX_SCALE;
      const double delta_s_in_meter = delta_solid * scn->fp_to_meter;
      float N[3] = {0, 0, 0};
      double cos_U_N, h, h_in_meter;

      if(delta == p->ds_hit0.distance) {
        f3_normalize(N, p->ds_hit0.normal);
        cos_U_N = f3_dot(p->ds_dir0, N);
      } else {
        f3_normalize(N, p->ds_hit1.normal);
        cos_U_N = f3_dot(p->ds_dir1, N);
      }
      h = delta * fabs(cos_U_N);
      h_in_meter = h * scn->fp_to_meter;

      power_term = h_in_meter * h_in_meter / (2.0 * props.lambda);

      if(h == delta_s_adjusted) {
        power_term += -(delta_s_in_meter * delta_s_in_meter)
                     / (2.0 * 3 * props.lambda);
      } else if(h < delta_s_adjusted) {
        const double sin_a = h / delta_s_adjusted;
        /* 3D correction */
        const double tmp = (sin_a*sin_a*sin_a - sin_a) / (1 - sin_a);
        power_term += (delta_s_in_meter * delta_s_in_meter)
                    / (6.0 * props.lambda) * tmp;
      }
      p->T.value += props.power * power_term;
    }

    if(p->ctx.green_path && props.power != SDIS_VOLUMIC_POWER_NONE) {
      p->ds_green_power_term += power_term;
    }
  }

  /* Time rewind */
  delta_m = (double)delta * scn->fp_to_meter;
  mu = (2.0 * 3 * props.lambda) / (props.rho * props.cp * delta_m * delta_m);
  res = time_rewind(scn, mu, props.t0, p->rng, &p->rwalk, &p->ctx, &p->T);
  if(res != RES_OK) goto error;
  if(p->T.done) {
    p->phase = PATH_DONE;
    p->active = 0;
    p->done_reason = 4; /* time rewind */
    goto exit;
  }

  /* Update hit info */
  if(S3D_HIT_NONE(&p->ds_hit0) || p->ds_hit0.distance > delta) {
    p->rwalk.hit_3d = S3D_HIT_NULL;
    p->rwalk.hit_side = SDIS_SIDE_NULL__;
  } else {
    p->rwalk.hit_3d = p->ds_hit0;
    p->rwalk.hit_side = f3_dot(p->ds_hit0.normal, p->ds_dir0) < 0
                      ? SDIS_FRONT : SDIS_BACK;
  }

  /* Move position */
  move_pos_3d(p->rwalk.vtx.P, p->ds_dir0, delta);

  /* Register heat path vertex */
  res = register_heat_vertex(p->ctx.heat_path, &p->rwalk.vtx, p->T.value,
    SDIS_HEAT_VERTEX_CONDUCTION, (int)p->ctx.nbranchings);
  if(res != RES_OK) goto error;

  /* Check loop condition: keep walking while no hit */
  if(S3D_HIT_NONE(&p->rwalk.hit_3d)) {
    /* Still inside solid — enter next iteration */
    p->phase = PATH_COUPLED_CONDUCTIVE;
    p->needs_ray = 0;
  } else {
    /* Hit boundary — register green power term and transition */
    if(p->ctx.green_path && p->ds_props_ref.power != SDIS_VOLUMIC_POWER_NONE) {
      green_path_add_power_term(
        p->ctx.green_path, p->ds_medium,
        &p->rwalk.vtx, p->ds_green_power_term);
    }
    p->T.func = boundary_path_3d;
    p->rwalk.enc_id = ENCLOSURE_ID_NULL;
    p->ds_initialized = 0; /* reset for next conductive entry */
    p->phase = PATH_COUPLED_BOUNDARY;
    p->needs_ray = 0;
  }

exit:
  return res;
error:
  p->phase = PATH_DONE;
  p->active = 0;
  p->done_reason = -1;
  goto exit;
}

/* --- PATH_COUPLED_CONVECTIVE: run convective_path (may need startup ray) - */
static res_T
step_convective(struct path_state* p, struct sdis_scene* scn)
{
  res_T res = RES_OK;
  ASSERT(p && scn);

  /* convective_path internally may call trace_ray for startup.  For the M1
   * skeleton we delegate to the original synchronous function. */

  res = convective_path_3d(scn, &p->ctx, &p->rwalk, p->rng, &p->T);
  if(res != RES_OK && res != RES_BAD_OP) goto error;
  if(res == RES_BAD_OP) {
    p->phase = PATH_DONE;
    p->active = 0;
    goto exit;
  }

  if(p->T.done) {
    p->phase = PATH_DONE;
    p->active = 0;
  } else if(p->T.func == boundary_path_3d) {
    p->phase = PATH_COUPLED_BOUNDARY;
  } else if(p->T.func == conductive_path_3d) {
    p->phase = PATH_COUPLED_CONDUCTIVE;
  } else if(p->T.func == radiative_path_3d) {
    p->phase = PATH_COUPLED_RADIATIVE;
  } else {
    FATAL("wavefront: unexpected T.func after convective_path\n");
  }

exit:
  return res;
error:
  goto exit;
}

/* --- PATH_COUPLED_RADIATIVE: bounce into radiative from boundary --------- */
static res_T
step_coupled_radiative_begin(struct path_state* p, struct sdis_scene* scn)
{
  float N[3] = {0, 0, 0};
  float dir[3] = {0, 0, 0};
  ASSERT(p && scn);
  ASSERT(!S3D_HIT_NONE(&p->rwalk.hit_3d));

  /* Compute outward normal */
  f3_normalize(N, p->rwalk.hit_3d.normal);
  if(p->rwalk.hit_side == SDIS_BACK) {
    f3_minus(N, N);
  }

  /* Cosine-weighted hemisphere sampling around the normal */
  ssp_ran_hemisphere_cos_float(p->rng, N, dir, NULL);

  /* Set radiative direction and emit trace */
  f3_set(p->rad_direction, dir);
  p->rad_bounce_count = 0;
  p->rad_retry_count  = 0;

  setup_radiative_trace_ray(p, scn);
  p->phase = PATH_RAD_TRACE_PENDING;
  return RES_OK;
}

/*******************************************************************************
 * Wavefront loop core
 ******************************************************************************/

/* Advance one path by one step (without ray).  Returns 1 if path was advanced,
 * 0 if it needs a ray or is done. */
static res_T
advance_one_step_no_ray(struct path_state* p, struct sdis_scene* scn,
                        int* advanced)
{
  res_T res = RES_OK;
  *advanced = 0;

  if(!p->active) return RES_OK;

  p->needs_ray = 0;

  switch(p->phase) {
  case PATH_INIT:
    res = step_init(p, scn);
    *advanced = 1;
    break;

  case PATH_COUPLED_BOUNDARY:
    res = step_boundary(p, scn);
    *advanced = 1;
    break;

  case PATH_COUPLED_CONDUCTIVE:
    res = step_conductive(p, scn);
    *advanced = 1;
    break;

  case PATH_COUPLED_CONVECTIVE:
    res = step_convective(p, scn);
    *advanced = 1;
    break;

  case PATH_COUPLED_RADIATIVE:
    res = step_coupled_radiative_begin(p, scn);
    *advanced = 1;
    break;

  case PATH_RAD_TRACE_PENDING:
  case PATH_COUPLED_BOUNDARY_REINJECT:
  case PATH_COUPLED_COND_DS_PENDING:
    /* These phases need ray results — cannot advance without them */
    break;

  case PATH_DONE:
  case PATH_ERROR:
    break;

  /* --- B-4 fine-grained: ray-pending states (cannot advance w/o ray) --- */
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
    /* B-4 ray-pending: not yet activated, cannot advance without ray */
    break;

  /* --- B-4 fine-grained: compute-only states (future, no-op for now) --- */
  case PATH_RAD_PROCESS_HIT:
  case PATH_BND_DISPATCH:
  case PATH_BND_POST_ROBIN_CHECK:
  case PATH_BND_SS_REINJECT_DECIDE:
  case PATH_BND_SF_PROB_DISPATCH:
  case PATH_BND_SF_NULLCOLL_DECIDE:
  case PATH_BND_SFN_PROB_DISPATCH:
  case PATH_BND_SFN_RAD_DONE:
  case PATH_BND_SFN_COMPUTE_Ti:
  case PATH_BND_SFN_COMPUTE_Ti_RESUME:
  case PATH_BND_SFN_CHECK_PMIN_PMAX:
  case PATH_BND_EXT_CHECK:
  case PATH_BND_EXT_DIRECT_RESULT:
  case PATH_BND_EXT_DIFFUSE_RESULT:
  case PATH_BND_EXT_DIFFUSE_SHADOW_RESULT:
  case PATH_BND_EXT_FINALIZE:
  case PATH_CND_DS_CHECK_TEMP:
  case PATH_CND_DS_STEP_PROCESS:
  case PATH_CND_DS_STEP_ADVANCE:
  case PATH_CND_WOS_CHECK_TEMP:
  case PATH_CND_WOS_CLOSEST_RESULT:
  case PATH_CND_WOS_FALLBACK_RESULT:
  case PATH_CND_WOS_TIME_TRAVEL:
  case PATH_CND_CUSTOM:
  case PATH_CNV_INIT:
  case PATH_CNV_STARTUP_RESULT:
  case PATH_CNV_SAMPLE_LOOP:
  case PATH_ENC_QUERY_RESOLVE:
    /* B-4 compute-only: not yet activated, no-op fallback */
    break;

  case PATH_PHASE_COUNT:
  default:
    FATAL("wavefront: unknown path phase %d\n", (int)p->phase);
    break;
  }

  return res;
}

/* Advance one path after receiving a ray trace result */
static res_T
advance_one_step_with_ray(struct path_state* p, struct sdis_scene* scn,
                          const struct s3d_hit* hit0,
                          const struct s3d_hit* hit1)
{
  res_T res = RES_OK;

  if(!p->active) return RES_OK;

  p->needs_ray = 0;

  switch(p->phase) {
  case PATH_RAD_TRACE_PENDING:
    res = step_radiative_trace(p, scn, hit0);
    break;

  case PATH_COUPLED_COND_DS_PENDING:
    res = step_conductive_ds_process(p, scn, hit0, hit1);
    break;

  /* Future milestones for boundary reinject:
   * case PATH_COUPLED_BOUNDARY_REINJECT:
   *   res = step_boundary_reinject(p, scn, hit0, hit1);
   *   break;
   */

  /* --- B-4 fine-grained ray-pending states (activated in M1-M9) ---
   * These cases should never be reached until the corresponding milestone
   * sets the phase.  Listed explicitly so the compiler warns on missing
   * enum values. */
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
    FATAL("wavefront: advance_with_ray in B-4 phase %d not yet activated\n",
          (int)p->phase);
    break;

  default:
    FATAL("wavefront: advance_with_ray in unexpected phase %d\n",
          (int)p->phase);
    break;
  }

  return res;
}

/* Collect all pending ray requests into the batch arrays */
static res_T
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
      rr->filter_data  = NULL; /* delta_sphere / reinject: no filter */
      rr->user_id      = (uint32_t)i;

      wf->ray_to_path[ray_idx] = (uint32_t)i;
      wf->ray_slot[ray_idx]    = 1;
      p->ray_req.batch_idx2    = (uint32_t)ray_idx;
      ray_idx++;
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

  /* First, deliver ray results to every path that requested rays */
  for(r = 0; r < wf->ray_count; r++) {
    uint32_t pid = wf->ray_to_path[r];
    uint32_t slot = wf->ray_slot[r];
    struct path_state* p = &wf->paths[pid];

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
