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


/* Wavefront steps: picardN recursive stack (M8).  Split from sdis_wf_steps.c. */

#include "sdis_wf_steps.h"
#include "sdis_solve_wavefront.h"  /* struct wavefront_context (used by types) */
#include "sdis.h"

#include <rsys/float33.h>   /* f33_rotation, f33_mulf3, f33_basis (B-4 M1) */
#include <rsys/rsys_math.h> /* PI */
#include "sdis_brdf.h"
#include "sdis_c.h"
#include "sdis_camera.h"
#include "sdis_device_c.h"
#include "sdis_estimator_buffer_c.h"
#include "sdis_green.h"
#include "sdis_heat_path_boundary_c.h"
#include "sdis_heat_path_conductive_c.h"
#include "sdis_interface_c.h"
#include "sdis_log.h"
#include "sdis_medium_c.h"
#include "sdis_misc.h"
#include "sdis_radiative_env_c.h"
#include "sdis_realisation.h"
#include "sdis_scene_c.h"
#include "sdis_source_c.h"
#include "sdis_tile.h"

#include <float.h>
#include <stdlib.h>
#include <string.h>
#include <star/swf.h>  /* swf_tabulation_inverse, SWF_QUADRATIC (M9 WoS) */

/*******************************************************************************
 * B-4 M8: PicardN recursive stack state machine
 *
 * Replaces the synchronous solid_fluid_boundary_picardN_path_3d() call with
 * fine-grained states that support recursive COMPUTE_TEMPERATURE sampling
 * via an explicit stack (sfn_stack[MAX_PICARD_DEPTH=3]).
 *
 * State flow:
 *   SFN_PROB_DISPATCH [C] → conv / cond / rad
 *     → rad: SFN_RAD_TRACE [R] → (radiative sub-path loop) → SFN_RAD_DONE
 *     → SFN_RAD_DONE [C] → early accept or start Ti chain
 *     → SFN_COMPUTE_Ti [C] → T.done? use directly : push → BND_DISPATCH
 *     → SFN_COMPUTE_Ti_RESUME [C] → pop stack → SFN_CHECK_PMIN_PMAX
 *     → SFN_CHECK_PMIN_PMAX [C] → accept / reject / continue next Ti
 *
 * Note: The picardN algorithm does NOT handle external net flux (check_net_flux
 * enforces this), unlike picard1.  This simplifies the state machine.
 *
 * CPU reference: sdis_heat_path_boundary_Xd_solid_fluid_picardN.h
 ******************************************************************************/

/* MAX_PICARD_DEPTH is defined in sdis_wf_state.h */

/* Helper: perform picardN SWITCH_IN_RADIATIVE (accept radiative sub-path) */
static void
sfn_switch_in_radiative(struct path_state* p)
{
  p->rwalk = p->locals.bnd_sf.rwalk_s;
  p->T = p->locals.bnd_sf.T_s;
  if(p->ctx.heat_path) {
    (void)heat_path_restart(p->ctx.heat_path, &p->locals.bnd_sf.hvtx_s);
  }
  p->phase = PATH_BND_POST_ROBIN_CHECK;
  p->needs_ray = 0;
}

/* Helper: perform picardN NULL_COLLISION (reject, loop back) */
static void
sfn_null_collision(struct path_state* p)
{
  p->rwalk = p->locals.bnd_sf.rwalk_snapshot;
  p->T = p->locals.bnd_sf.T_snapshot;
  if(p->ctx.heat_path) {
    (void)heat_path_restart(p->ctx.heat_path, &p->locals.bnd_sf.hvtx_saved);
    if(p->locals.bnd_sf.ihvtx_radi_begin < p->locals.bnd_sf.ihvtx_radi_end) {
      heat_path_increment_sub_path_branch_id(
        p->ctx.heat_path,
        p->locals.bnd_sf.ihvtx_radi_begin,
        p->locals.bnd_sf.ihvtx_radi_end);
    }
  }
  if(p->ctx.green_path) {
    green_path_reset_limit(p->ctx.green_path);
  }
  /* h_hat remains set → prob_dispatch will skip init */
  p->phase = PATH_BND_SFN_PROB_DISPATCH;
  p->needs_ray = 0;
}

/* Helper: compute h_radi bounds for CHECK_PMIN_PMAX at stage i.
 * i = number of T_values already computed (1..6).
 * Returns h_radi_min/max (without *epsilon/h_hat factor). */
static void
sfn_compute_h_radi_bounds(const struct path_state* p,
                          const struct path_sfn_data* sfn,
                          int i,
                          double* out_min,
                          double* out_max)
{
  const double* T = sfn->stack[sfn->depth].T_values;
  double Tmin = p->ctx.Tmin;
  double Tmin2 = p->ctx.Tmin2;
  double Tmin3 = p->ctx.Tmin3;
  double That = p->ctx.That;
  double That2 = p->ctx.That2;
  double That3 = p->ctx.That3;

  /* Matches the exact formulas from picardN.h lines 355-385 */
  switch(i) {
  case 1: /* T0 known */
    *out_min = BOLTZMANN_CONSTANT * (Tmin3 + 3*Tmin2*T[0]);
    *out_max = BOLTZMANN_CONSTANT * (That3 + 3*That2*T[0]);
    break;
  case 2: /* T0,T1 known */
    *out_min = BOLTZMANN_CONSTANT * (Tmin3 + Tmin2*T[0] + 2*Tmin*T[0]*T[1]);
    *out_max = BOLTZMANN_CONSTANT * (That3 + That2*T[0] + 2*That*T[0]*T[1]);
    break;
  case 3: /* T0,T1,T2 known */
    *out_min = BOLTZMANN_CONSTANT * (Tmin3 + Tmin2*T[0] + Tmin*T[0]*T[1] + T[0]*T[1]*T[2]);
    *out_max = BOLTZMANN_CONSTANT * (That3 + That2*T[0] + That*T[0]*T[1] + T[0]*T[1]*T[2]);
    break;
  case 4: /* T0,T1,T2,T3 known */
    *out_min = BOLTZMANN_CONSTANT * (Tmin2*T[3] + Tmin*T[0]*T[3] + T[0]*T[1]*T[3] + T[0]*T[1]*T[2]);
    *out_max = BOLTZMANN_CONSTANT * (That2*T[3] + That*T[0]*T[3] + T[0]*T[1]*T[3] + T[0]*T[1]*T[2]);
    break;
  case 5: /* T0..T4 known */
    *out_min = BOLTZMANN_CONSTANT * (Tmin*T[3]*T[4] + T[0]*T[3]*T[4] + T[0]*T[1]*T[3] + T[0]*T[1]*T[2]);
    *out_max = BOLTZMANN_CONSTANT * (That*T[3]*T[4] + T[0]*T[3]*T[4] + T[0]*T[1]*T[3] + T[0]*T[1]*T[2]);
    break;
  case 6: /* T0..T5 known → exact */
    *out_min = BOLTZMANN_CONSTANT * (T[3]*T[4]*T[5] + T[0]*T[3]*T[4] + T[0]*T[1]*T[3] + T[0]*T[1]*T[2]);
    *out_max = *out_min; /* exact, no bounds */
    break;
  default:
    FATAL("sfn_compute_h_radi_bounds: invalid i=%d\n", ARG1(i));
    *out_min = *out_max = 0;
    break;
  }
}

/* --- PATH_BND_SFN_PROB_DISPATCH: picardN probability dispatch ------------ */
LOCAL_SYM res_T
step_bnd_sfn_prob_dispatch(struct path_state* p, struct sdis_scene* scn,
                          struct path_sfn_data* sfn)
{
  struct reinjection_step reinject_step = REINJECTION_STEP_NULL;
  struct solid_reinjection_args solid_reinject_args = SOLID_REINJECTION_ARGS_NULL;
  double r;
  res_T res = RES_OK;

  ASSERT(p && scn);
  (void)sfn; /* sfn_stack accessed via sfn in rad_done/compute_Ti/check */

  /* === First entry: compute probabilities (h_hat == 0 detector) === */
  if(p->locals.bnd_sf.h_hat == 0) {
    double delta, delta_m;
    struct sdis_interface* interf;
    struct sdis_interface_fragment frag = SDIS_INTERFACE_FRAGMENT_NULL;

    delta = p->locals.bnd_sf.chosen_dst / sqrt(3.0);
    delta_m = delta * scn->fp_to_meter;
    p->locals.bnd_sf.delta_m = delta_m;

    interf = scene_get_interface(scn, p->rwalk.hit_3d.prim.prim_id);

    setup_interface_fragment_3d(&frag, &p->rwalk.vtx,
                                &p->rwalk.hit_3d, p->rwalk.hit_side);

    /* PicardN does not handle external net flux — verify */
    {
      double phi = interface_side_get_flux(interf, &frag);
      if(phi != SDIS_FLUX_NONE && phi != 0) {
        log_err(scn->dev,
          "wavefront M8: invalid flux '%g' W/m^2 for picardN "
          "(picard_order=%lu, must be 0)\n",
          phi, (unsigned long)get_picard_order(&p->ctx));
        res = RES_BAD_OP_IRRECOVERABLE;
        goto error;
      }
    }

    p->locals.bnd_sf.h_conv =
        interface_get_convection_coef(interf, &frag);
    p->locals.bnd_sf.h_cond = p->locals.bnd_sf.lambda / delta_m;

    if(p->locals.bnd_sf.epsilon <= 0) {
      p->locals.bnd_sf.h_radi_hat = 0;
    } else {
      res = scene_check_temperature_range(scn);
      if(res != RES_OK) { res = RES_BAD_OP_IRRECOVERABLE; goto error; }
      p->locals.bnd_sf.h_radi_hat =
          4.0 * BOLTZMANN_CONSTANT * p->ctx.That3
              * p->locals.bnd_sf.epsilon;
    }

    p->locals.bnd_sf.h_hat = p->locals.bnd_sf.h_conv
                           + p->locals.bnd_sf.h_cond
                           + p->locals.bnd_sf.h_radi_hat;
    p->locals.bnd_sf.p_conv = p->locals.bnd_sf.h_conv / p->locals.bnd_sf.h_hat;
    p->locals.bnd_sf.p_cond = p->locals.bnd_sf.h_cond / p->locals.bnd_sf.h_hat;

    /* Save heat vertex for null-collision restart */
    if(p->ctx.heat_path) {
      p->locals.bnd_sf.hvtx_saved =
          *heat_path_get_last_vertex(p->ctx.heat_path);
    }

    /* PicardN: no external flux sub-state (check_net_flux enforced above).
     * Fall through directly to null-collision dispatch. */
  }

  /* === Null-collision dispatch === */
  r = ssp_rng_canonical(p->rng);
  p->locals.bnd_sf.r = r;

  /* --- Convective branch --- */
  if(r < p->locals.bnd_sf.p_conv) {
    p->T.func = convective_path_3d;
    p->rwalk.enc_id = p->locals.bnd_sf.enc_ids[p->locals.bnd_sf.fluid_side];
    p->rwalk.hit_side = p->locals.bnd_sf.fluid_side;
    p->phase = PATH_BND_POST_ROBIN_CHECK;
    p->needs_ray = 0;
    goto exit;
  }

  /* --- Conductive branch --- */
  if(r < p->locals.bnd_sf.p_conv + p->locals.bnd_sf.p_cond) {
    f3_set(reinject_step.direction, p->locals.bnd_sf.chosen_dir);
    reinject_step.distance = p->locals.bnd_sf.chosen_dst;
    reinject_step.hit_3d = p->locals.bnd_sf.chosen_hit;

    solid_reinject_args.reinjection = &reinject_step;
    solid_reinject_args.rwalk_ctx = &p->ctx;
    solid_reinject_args.rwalk = &p->rwalk;
    solid_reinject_args.rng = p->rng;
    solid_reinject_args.T = &p->T;
    solid_reinject_args.fp_to_meter = scn->fp_to_meter;
    res = solid_reinjection_3d(scn, p->locals.bnd_sf.solid_enc_id,
                               &solid_reinject_args);
    if(res != RES_OK) goto error;

    if(p->T.done) {
      p->phase = PATH_DONE;
      p->active = 0;
      p->done_reason = 4;
      goto exit;
    }

    if(p->T.func == conductive_path_3d) {
      p->ds_initialized = 0;
      p->phase = PATH_COUPLED_CONDUCTIVE;
    } else if(p->T.func == boundary_path_3d) {
      p->phase = PATH_COUPLED_BOUNDARY;
    } else {
      FATAL("wavefront M8: unexpected T.func after solid_reinjection\n");
    }
    p->needs_ray = 0;
    goto exit;
  }

  /* --- Radiative branch: start null-collision radiative sub-path --- */
  {
    float N[3] = {0, 0, 0};
    float dir[3] = {0, 0, 0};

    if(p->ctx.heat_path) {
      p->locals.bnd_sf.ihvtx_radi_begin =
          heat_path_get_vertices_count(p->ctx.heat_path) - 1;
    }

    /* Save snapshot for restore on reject */
    p->locals.bnd_sf.rwalk_snapshot = p->rwalk;
    p->locals.bnd_sf.T_snapshot = p->T;

    /* Set up rwalk for radiative sub-path: starts from fluid side */
    p->rwalk.enc_id =
        p->locals.bnd_sf.enc_ids[p->locals.bnd_sf.fluid_side];
    p->rwalk.hit_side = p->locals.bnd_sf.fluid_side;

    /* Compute outward normal (pointing into fluid) */
    f3_normalize(N, p->rwalk.hit_3d.normal);
    if(p->locals.bnd_sf.fluid_side == SDIS_BACK) {
      f3_minus(N, N);
    }

    /* Cosine-weighted hemisphere sampling */
    ssp_ran_hemisphere_cos_float(p->rng, N, dir, NULL);

    f3_set(p->locals.bnd_sf.rad_sub_direction, dir);
    p->locals.bnd_sf.rad_sub_bounce_count = 0;
    p->locals.bnd_sf.rad_sub_retry_count = 0;

    /* Setup ray request for radiative trace */
    {
      float pos[3];
      f3_set_d3(pos, p->rwalk.vtx.P);
      p->ray_req.origin[0] = pos[0];
      p->ray_req.origin[1] = pos[1];
      p->ray_req.origin[2] = pos[2];
      p->ray_req.direction[0] = dir[0];
      p->ray_req.direction[1] = dir[1];
      p->ray_req.direction[2] = dir[2];
      p->ray_req.range[0] = 0.0f;
      p->ray_req.range[1] = FLT_MAX;
      p->ray_req.ray_count = 1;

      p->filter_data_storage = HIT_FILTER_DATA_NULL;
      p->filter_data_storage.hit_3d = p->rwalk.hit_3d;
      p->filter_data_storage.epsilon = 1.e-6;
      p->filter_data_storage.scn = scn;
      p->filter_data_storage.enc_id = p->rwalk.enc_id;

      p->ray_bucket = RAY_BUCKET_RADIATIVE;
      p->ray_count_ext = 1;
      p->needs_ray = 1;
      p->phase = PATH_BND_SFN_RAD_TRACE;
    }
  }

exit:
  return res;
error:
  p->phase = PATH_DONE;
  p->active = 0;
  p->done_reason = -1;
  goto exit;
}

/* --- PATH_BND_SFN_RAD_TRACE: process radiative sub-path hit ------------- */
/* Functionally identical to step_bnd_sf_nullcoll_rad_trace, but transitions
 * to PATH_BND_SFN_RAD_DONE on completion instead of SF_NULLCOLL_DECIDE. */
LOCAL_SYM res_T
step_bnd_sfn_rad_trace(struct path_state* p, struct sdis_scene* scn,
                       const struct s3d_hit* trace_hit)
{
  res_T res = RES_OK;

  ASSERT(p && scn && trace_hit);

  /* --- Miss → radiative environment temperature → go to rad_done --- */
  if(S3D_HIT_NONE(trace_hit)) {
    struct sdis_radiative_ray ray = SDIS_RADIATIVE_RAY_NULL;
    double dir_d[3];
    double trad;

    d3_set_f3(dir_d, p->locals.bnd_sf.rad_sub_direction);
    d3_normalize(dir_d, dir_d);
    d3_set(ray.dir, dir_d);
    ray.time = p->rwalk.vtx.time;

    trad = radiative_env_get_temperature(scn->radenv, &ray);
    if(SDIS_TEMPERATURE_IS_UNKNOWN(trad)) {
      log_err(scn->dev,
        "wavefront M8: radiative sub-path reached unknown env from "
        "(%g, %g, %g) along (%g, %g, %g)\n",
        SPLIT3(p->rwalk.vtx.P), SPLIT3(dir_d));
      res = RES_BAD_OP;
      goto error;
    }

    p->T.value += trad;
    p->T.done = 1;
    d3_set_f3(p->rwalk.dir, p->locals.bnd_sf.rad_sub_direction);

    /* Clear stale hit — matches CPU set_limit_radiative_temperature which
     * ASSERTs SXD_HIT_NONE and sets hit_side = SDIS_SIDE_NULL__. */
    p->rwalk.hit_3d = S3D_HIT_NULL;
    p->rwalk.hit_side = SDIS_SIDE_NULL__;

    p->phase = PATH_BND_SFN_RAD_DONE;
    p->needs_ray = 0;
    goto exit;
  }

  /* --- Hit handling (identical to M5 rad_trace) --- */
  {
    struct sdis_interface_fragment frag = SDIS_INTERFACE_FRAGMENT_NULL;
    struct sdis_interface* interf = NULL;
    struct sdis_medium* chk_mdm = NULL;
    struct brdf brdf = BRDF_NULL;
    struct brdf_sample bounce = BRDF_SAMPLE_NULL;
    struct brdf_setup_args brdf_args = BRDF_SETUP_ARGS_NULL;
    double dir_d[3], pos_d[3], N[3], wi[3];

    p->rwalk.hit_3d = *trace_hit;

    d3_set_f3(dir_d, p->locals.bnd_sf.rad_sub_direction);
    d3_normalize(dir_d, dir_d);
    d3_set(pos_d, p->rwalk.vtx.P);
    {
      double vec[3];
      d3_add(pos_d, pos_d, d3_muld(vec, dir_d, trace_hit->distance));
    }
    d3_set(p->rwalk.vtx.P, pos_d);

    d3_set_f3(N, trace_hit->normal);
    d3_normalize(N, N);

    p->rwalk.hit_side = d3_dot(dir_d, N) < 0 ? SDIS_FRONT : SDIS_BACK;

    interf = scene_get_interface(scn, trace_hit->prim.prim_id);

    {
      struct sdis_rwalk_vertex vtx = SDIS_RWALK_VERTEX_NULL;
      d3_set(vtx.P, pos_d);
      vtx.time = p->rwalk.vtx.time;
      setup_interface_fragment_3d(&frag, &vtx, &p->rwalk.hit_3d,
                                  p->rwalk.hit_side);
    }

    res = wf_check_interface(interf, &frag,
                             p->locals.bnd_sf.rad_sub_retry_count >= 9);
    if(res != RES_OK) {
      if(p->locals.bnd_sf.rad_sub_retry_count < 9) {
        struct sdis_medium* solid_mdm;
        double delta_adj;

        p->locals.bnd_sf.rad_sub_retry_count++;

        if(sdis_medium_get_type(interf->medium_front) == SDIS_SOLID)
          solid_mdm = interf->medium_front;
        else
          solid_mdm = interf->medium_back;

        delta_adj = solid_get_delta(solid_mdm, &p->rwalk.vtx);
        move_away_primitive_boundaries_3d(&p->rwalk.hit_3d, delta_adj,
                                          p->rwalk.vtx.P);

        {
          float pos_f[3];
          f3_set_d3(pos_f, p->rwalk.vtx.P);
          p->ray_req.origin[0] = pos_f[0];
          p->ray_req.origin[1] = pos_f[1];
          p->ray_req.origin[2] = pos_f[2];
          p->ray_req.direction[0] = p->locals.bnd_sf.rad_sub_direction[0];
          p->ray_req.direction[1] = p->locals.bnd_sf.rad_sub_direction[1];
          p->ray_req.direction[2] = p->locals.bnd_sf.rad_sub_direction[2];
          p->ray_req.range[0] = 0.0f;
          p->ray_req.range[1] = FLT_MAX;
          p->ray_req.ray_count = 1;

          p->filter_data_storage = HIT_FILTER_DATA_NULL;
          p->filter_data_storage.hit_3d = p->rwalk.hit_3d;
          p->filter_data_storage.epsilon = 1.e-6;
          p->filter_data_storage.scn = scn;
          p->filter_data_storage.enc_id = p->rwalk.enc_id;

          p->ray_bucket = RAY_BUCKET_RADIATIVE;
          p->ray_count_ext = 1;
          p->needs_ray = 1;
          p->phase = PATH_BND_SFN_RAD_TRACE;
        }
        goto exit;
      }
      goto error;
    }
    p->locals.bnd_sf.rad_sub_retry_count = 0;

    chk_mdm = p->rwalk.hit_side == SDIS_FRONT
            ? interf->medium_front : interf->medium_back;
    if(sdis_medium_get_type(chk_mdm) == SDIS_SOLID) {
      log_err(scn->dev,
        "wavefront M8: radiative sub-path in solid at (%g, %g, %g)\n",
        SPLIT3(p->rwalk.vtx.P));
      res = RES_BAD_OP_IRRECOVERABLE;
      goto error;
    }

    brdf_args.interf = interf;
    brdf_args.frag = &frag;
    brdf_args.source_id = SDIS_INTERN_SOURCE_ID;
    res = brdf_setup(scn->dev, &brdf_args, &brdf);
    if(res != RES_OK) goto error;

    /* Absorption test */
    if(ssp_rng_canonical(p->rng) < brdf.emissivity) {
      p->rwalk.enc_id = ENCLOSURE_ID_NULL;
      p->T.func = boundary_path_3d;
      d3_set_f3(p->rwalk.dir, p->locals.bnd_sf.rad_sub_direction);

      p->phase = PATH_BND_SFN_RAD_DONE;
      p->needs_ray = 0;
      goto exit;
    }

    /* BRDF reflection */
    p->locals.bnd_sf.rad_sub_bounce_count++;
    d3_minus(wi, dir_d);
    switch(p->rwalk.hit_side) {
    case SDIS_FRONT: break;
    case SDIS_BACK: d3_minus(N, N); break;
    default: FATAL("Unreachable\n"); break;
    }
    brdf_sample(&brdf, p->rng, wi, N, &bounce);
    f3_set_d3(p->locals.bnd_sf.rad_sub_direction, bounce.dir);

    /* Emit next radiative trace ray */
    {
      float pos_f[3];
      f3_set_d3(pos_f, p->rwalk.vtx.P);
      p->ray_req.origin[0] = pos_f[0];
      p->ray_req.origin[1] = pos_f[1];
      p->ray_req.origin[2] = pos_f[2];
      p->ray_req.direction[0] = p->locals.bnd_sf.rad_sub_direction[0];
      p->ray_req.direction[1] = p->locals.bnd_sf.rad_sub_direction[1];
      p->ray_req.direction[2] = p->locals.bnd_sf.rad_sub_direction[2];
      p->ray_req.range[0] = 0.0f;
      p->ray_req.range[1] = FLT_MAX;
      p->ray_req.ray_count = 1;

      p->filter_data_storage = HIT_FILTER_DATA_NULL;
      p->filter_data_storage.hit_3d = p->rwalk.hit_3d;
      p->filter_data_storage.epsilon = 1.e-6;
      p->filter_data_storage.scn = scn;
      p->filter_data_storage.enc_id = p->rwalk.enc_id;

      p->ray_bucket = RAY_BUCKET_RADIATIVE;
      p->ray_count_ext = 1;
      p->needs_ray = 1;
      p->phase = PATH_BND_SFN_RAD_TRACE;
    }
  }

exit:
  return res;
error:
  p->phase = PATH_DONE;
  p->active = 0;
  p->done_reason = -1;
  goto exit;
}

/* --- PATH_BND_SFN_RAD_DONE: radiative sub-path completed ----------------- */
LOCAL_SYM res_T
step_bnd_sfn_rad_done(struct path_state* p, struct sdis_scene* scn,
                      struct path_sfn_data* sfn)
{
  double h_radi_min, p_radi_min;
  res_T res = RES_OK;

  ASSERT(p && scn);

  /* Save radiative sub-path endpoint state */
  p->locals.bnd_sf.rwalk_s = p->rwalk;
  p->locals.bnd_sf.T_s = p->T;

  if(p->ctx.heat_path) {
    p->locals.bnd_sf.hvtx_s =
        *heat_path_get_last_vertex(p->ctx.heat_path);
    p->locals.bnd_sf.ihvtx_radi_end =
        heat_path_get_vertices_count(p->ctx.heat_path);
  }

  /* Restore main rwalk/T from snapshot */
  p->rwalk = p->locals.bnd_sf.rwalk_snapshot;
  p->T = p->locals.bnd_sf.T_snapshot;

  /* Initial h_radi_min check (before any COMPUTE_TEMPERATURE) */
  h_radi_min = 4.0 * BOLTZMANN_CONSTANT * p->ctx.Tmin3
             * p->locals.bnd_sf.epsilon;
  p_radi_min = h_radi_min / p->locals.bnd_sf.h_hat;

  if(p->locals.bnd_sf.r < p->locals.bnd_sf.p_conv
                        + p->locals.bnd_sf.p_cond
                        + p_radi_min) {
    /* Early accept: use radiative sub-path */
    sfn_switch_in_radiative(p);
    goto exit;
  }

  /* Initialize sfn_stack frame for COMPUTE_TEMPERATURE chain.
   * For the outermost M8 sfn_stack_depth == 0.  For nested M8
   * (picard_order >= 3) it may be > 0 because the outer M8's
   * COMPUTE_Ti already pushed a frame. */
  {
    int d = sfn->depth;
    ASSERT(d >= 0 && d < MAX_PICARD_DEPTH);
    sfn->stack[d].return_state = PATH_BND_POST_ROBIN_CHECK;
    sfn->stack[d].partial_temperature = 0;
    sfn->stack[d].rwalk_saved = p->rwalk;
    sfn->stack[d].T_saved = p->T;
    sfn->stack[d].T_count = 0;
    sfn->stack[d].r = p->locals.bnd_sf.r;
    sfn->stack[d].p_conv = p->locals.bnd_sf.p_conv;
    sfn->stack[d].p_cond = p->locals.bnd_sf.p_cond;
    sfn->stack[d].h_hat = p->locals.bnd_sf.h_hat;
    memset(sfn->stack[d].T_values, 0, sizeof(sfn->stack[d].T_values));
  }

  /* Start COMPUTE_TEMPERATURE for T0 */
  p->phase = PATH_BND_SFN_COMPUTE_Ti;
  p->needs_ray = 0;

exit:
  return res;
}

/* --- PATH_BND_SFN_COMPUTE_Ti: compute i-th temperature sample ----------- */
LOCAL_SYM res_T
step_bnd_sfn_compute_Ti(struct path_state* p, struct sdis_scene* scn,
                        struct path_sfn_data* sfn)
{
  int i;
  struct rwalk* sample_rwalk;
  struct temperature* sample_T;
  res_T res = RES_OK;

  ASSERT(p && scn);
  ASSERT(sfn->depth >= 0 && sfn->depth < MAX_PICARD_DEPTH);

  i = sfn->stack[sfn->depth].T_count;
  ASSERT(i >= 0 && i < 6);

  /* T0-T2 sample from radiative endpoint; T3-T5 from boundary position */
  if(i < 3) {
    sample_rwalk = &p->locals.bnd_sf.rwalk_s;
    sample_T = &p->locals.bnd_sf.T_s;
  } else {
    sample_rwalk = &sfn->stack[sfn->depth].rwalk_saved;
    sample_T = &sfn->stack[sfn->depth].T_saved;
  }

  /* If T is already known (done), use it directly without recursion */
  if(sample_T->done) {
    sfn->stack[sfn->depth].T_values[i] = sample_T->value;
    sfn->stack[sfn->depth].T_count = i + 1;
    p->phase = PATH_BND_SFN_CHECK_PMIN_PMAX;
    p->needs_ray = 0;
    goto exit;
  }

  /* Need recursive sampling → push a new stack frame */
  if(sfn->depth + 1 >= MAX_PICARD_DEPTH) {
    /* Stack overflow → fallback to synchronous path */
    log_warn(scn->dev,
      "wavefront M8: sfn_stack overflow (depth=%d, MAX=%d) at (%g, %g, %g). "
      "Falling back to synchronous picardN.\n",
      sfn->depth + 1, MAX_PICARD_DEPTH,
      SPLIT3(p->rwalk.vtx.P));

    /* Restore original state and use synchronous fallback */
    p->rwalk = sfn->stack[0].rwalk_saved;
    p->T = sfn->stack[0].T_saved;
    sfn->depth = 0;

    /* Restore snapshot */
    p->rwalk = p->locals.bnd_sf.rwalk_snapshot;
    p->T = p->locals.bnd_sf.T_snapshot;
    p->locals.bnd_sf.h_hat = 0; /* force re-init */

    {
      struct sdis_interface_fragment frag = SDIS_INTERFACE_FRAGMENT_NULL;
      setup_interface_fragment_3d(&frag, &p->rwalk.vtx,
                                  &p->rwalk.hit_3d, p->rwalk.hit_side);
      res = solid_fluid_boundary_picardN_path_3d(
        scn, &p->ctx, &frag, &p->rwalk, p->rng, &p->T);
    }
    if(res != RES_OK && res != RES_BAD_OP) goto error;
    if(res == RES_BAD_OP) {
      p->phase = PATH_DONE;
      p->active = 0;
      goto exit;
    }
    if(p->T.done) {
      p->phase = PATH_DONE;
      p->active = 0;
      p->done_reason = 3;
    } else {
      p->phase = PATH_BND_POST_ROBIN_CHECK;
    }
    goto exit;
  }

  /* Push stack frame: save current context + start sub-path */
  /* Save nbranchings so we can restore on pop */
  p->locals.bnd_sf.coupled_nbranchings_saved = p->coupled_nbranchings;

  /* Set up rwalk/T for the sub-path */
  p->rwalk = *sample_rwalk;
  p->T = TEMPERATURE_NULL;
  p->T.func = boundary_path_3d;

  /* The sub-path is at branching level nbranchings+1 */
  p->ctx.nbranchings++;
  p->coupled_nbranchings = (int)p->ctx.nbranchings;

  /* Register sub-path heat vertex */
  if(p->ctx.heat_path) {
    struct sdis_heat_vertex heat_vtx = SDIS_HEAT_VERTEX_NULL;
    heat_vtx.P[0] = p->rwalk.vtx.P[0];
    heat_vtx.P[1] = p->rwalk.vtx.P[1];
    heat_vtx.P[2] = p->rwalk.vtx.P[2];
    heat_vtx.time = p->rwalk.vtx.time;
    heat_vtx.weight = 0;
    heat_vtx.type = SDIS_HEAT_VERTEX_RADIATIVE;
    heat_vtx.branch_id = (int)p->ctx.nbranchings;
    res = heat_path_restart(p->ctx.heat_path, &heat_vtx);
    if(res != RES_OK) goto error;
  }

  sfn->stack[sfn->depth].bnd_sf_backup = p->locals.bnd_sf;
  sfn->depth++;

  /* Mark that the sub-path will return to SFN_COMPUTE_Ti_RESUME */
  /* (this is used when the sub-path reaches PATH_DONE) */

  /* The sub-path enters sample_coupled_path which calls boundary_path,
   * conductive_path, etc.  In wavefront mode, this means re-entering
   * step_boundary via PATH_COUPLED_BOUNDARY. */
  p->phase = PATH_COUPLED_BOUNDARY;
  p->needs_ray = 0;

exit:
  return res;
error:
  p->phase = PATH_DONE;
  p->active = 0;
  p->done_reason = -1;
  goto exit;
}

/* --- PATH_BND_SFN_COMPUTE_Ti_RESUME: sub-path returned, pop stack -------- */
LOCAL_SYM res_T
step_bnd_sfn_compute_Ti_resume(struct path_state* p, struct sdis_scene* scn,
                               struct path_sfn_data* sfn)
{
  int i;
  res_T res = RES_OK;

  ASSERT(p && scn);
  ASSERT(sfn->depth > 0);

  /* The sub-path result is in p->T.value */
  ASSERT(p->T.done);

  /* Check temperature range */
  if(p->T.value < scn->tmin || scn->tmax < p->T.value) {
    log_err(scn->dev,
      "wavefront M8: invalid sub-path temperature %gK "
      "(range [%g, %g])\n",
      p->T.value, scn->tmin, scn->tmax);
    res = RES_BAD_OP_IRRECOVERABLE;
    goto error;
  }

  /* Pop stack frame */
  sfn->depth--;

  /* Restore bnd_sf locals (inner picard1 may have overwritten the union) */
  p->locals.bnd_sf = sfn->stack[sfn->depth].bnd_sf_backup;

  /* Record the computed temperature */
  i = sfn->stack[sfn->depth].T_count;
  sfn->stack[sfn->depth].T_values[i] = p->T.value;
  sfn->stack[sfn->depth].T_count = i + 1;

  /* Restore rwalk/T from the parent frame */
  p->rwalk = sfn->stack[sfn->depth].rwalk_saved;
  p->T = sfn->stack[sfn->depth].T_saved;

  /* Restore nbranchings */
  p->coupled_nbranchings = p->locals.bnd_sf.coupled_nbranchings_saved;
  p->ctx.nbranchings = (size_t)p->coupled_nbranchings;

  p->phase = PATH_BND_SFN_CHECK_PMIN_PMAX;
  p->needs_ray = 0;

exit:
  return res;
error:
  p->phase = PATH_DONE;
  p->active = 0;
  p->done_reason = -1;
  goto exit;
}

/* --- PATH_BND_SFN_CHECK_PMIN_PMAX: early accept/reject or continue ------- */
LOCAL_SYM res_T
step_bnd_sfn_check_pmin_pmax(struct path_state* p, struct sdis_scene* scn,
                             struct path_sfn_data* sfn)
{
  int i;
  double h_radi_min, h_radi_max;
  double p_radi_min, p_radi_max;
  double epsilon, h_hat, r, p_conv, p_cond;
  res_T res = RES_OK;

  ASSERT(p && scn);

  i = sfn->stack[sfn->depth].T_count;
  ASSERT(i >= 1 && i <= 6);

  epsilon = p->locals.bnd_sf.epsilon;
  h_hat = sfn->stack[sfn->depth].h_hat;
  r = sfn->stack[sfn->depth].r;
  p_conv = sfn->stack[sfn->depth].p_conv;
  p_cond = sfn->stack[sfn->depth].p_cond;

  sfn_compute_h_radi_bounds(p, sfn, i, &h_radi_min, &h_radi_max);

  p_radi_min = h_radi_min * epsilon / h_hat;
  p_radi_max = h_radi_max * epsilon / h_hat;

  /* --- Early accept --- */
  if(r < p_conv + p_cond + p_radi_min) {
    sfn_switch_in_radiative(p);
    goto exit;
  }

  /* --- Early reject (only when i < 6, i.e. bounds are not exact) --- */
  if(i < 6 && r > p_conv + p_cond + p_radi_max) {
    sfn_null_collision(p);
    goto exit;
  }

  /* --- Continue: need more Ti samples --- */
  if(i < 6) {
    p->phase = PATH_BND_SFN_COMPUTE_Ti;
    p->needs_ray = 0;
    goto exit;
  }

  /* i == 6: all T0-T5 computed, h_radi is now exact */
  /* Final decision: h_radi_min == h_radi_max at i=6 */
  if(r < p_conv + p_cond + p_radi_min) {
    sfn_switch_in_radiative(p);
  } else {
    sfn_null_collision(p);
  }

exit:
  return res;
}

