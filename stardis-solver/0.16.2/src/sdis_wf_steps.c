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

/* Wavefront step function library — implementation.
 *
 * All path-state transition functions (step_*, advance_one_step_*) and
 * ray setup helpers (setup_*) are defined here with LOCAL_SYM visibility.
 *
 * Extracted from sdis_solve_wavefront.c to break the #include .c hack
 * that previously coupled wavefront.c, persistent_wavefront.c, and test
 * files into a single translation unit.
 *
 * The physical algorithms are identical to the original — only the linkage
 * has changed (static → LOCAL_SYM).
 */

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

/*******************************************************************************
 * check_interface — copied from sdis_heat_path_radiative_Xd.h because it is
 * static in that header and not accessible from this translation unit.
 * This function is DIM-independent.
 ******************************************************************************/
LOCAL_SYM res_T
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
 * Ray-request setup helpers (matches trace_ray / sample_next_step API)
 ******************************************************************************/
LOCAL_SYM void
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

  /* B-4 M2: bucket classification */
  p->ray_bucket = RAY_BUCKET_RADIATIVE;
  p->ray_count_ext = 1;
  p->needs_ray = 1;
}

LOCAL_SYM void
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

  /* B-4 M2: bucket classification */
  p->ray_bucket = RAY_BUCKET_STEP_PAIR;
  p->ray_count_ext = 2;
  p->needs_ray = 1;
}

LOCAL_SYM void
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

  /* B-4 M2: bucket classification */
  p->ray_bucket = RAY_BUCKET_STARTUP;
  p->ray_count_ext = 1;
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
LOCAL_SYM res_T
step_init(struct path_state* p, struct sdis_scene* scn)
{
  ASSERT(p && scn);

  /* Register starting vertex (skip heat_path for now) */

  /* Emit the first radiative trace ray */
  setup_radiative_trace_ray(p, scn);
  p->phase = PATH_RAD_TRACE_PENDING;
  return RES_OK;
}

/* --- PATH_RAD_TRACE_PENDING -> process hit / BRDF decision ---------------- */
LOCAL_SYM res_T
step_radiative_trace(
  struct path_state* p,
  struct sdis_scene* scn,
  const struct s3d_hit* trace_hit)
{
  res_T res = RES_OK;
  ASSERT(p && scn && trace_hit);

  /* --- Miss -> radiative environment temperature -> done --- */
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
      /* Absorbed -> enter boundary path */
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
LOCAL_SYM res_T
step_boundary(struct path_state* p, struct sdis_scene* scn)
{
  res_T res = RES_OK;
  ASSERT(p && scn);

  /* Replicate sample_coupled_path's nbranchings management.
   *
   * In the original code, sample_coupled_path increments nbranchings ONCE
   * at entry, then the while(!T->done) loop calls T->func (boundary_path,
   * conductive_path, etc.) repeatedly WITHOUT changing nbranchings.
   * Deeper recursive calls (picardN -> sample_coupled_path) handle their
   * own increment/decrement on ctx->nbranchings via the call stack.
   *
   * In wavefront mode, each call to step_boundary / step_conductive /
   * step_convective corresponds to one iteration of that while loop.
   * We only increment on the FIRST entry (sentinel -1 -> 0), and simply
   * restore the saved value on re-entries within the same level. */
  if(p->coupled_nbranchings < 0) {
    /* First entry into coupled path: simulate SIZE_MAX + 1 = 0 */
    p->ctx.nbranchings = 0;
    p->coupled_nbranchings = 0;
  } else {
    /* Re-entry within same sample_coupled_path level -- restore, no increment */
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

  /* B4-M6: redirect ALL boundary dispatch to the fine-grained state machine.
   * step_bnd_dispatch handles solid/solid (M3 batch), picard1/N (sync
   * fallback until M5/M8), Dirichlet, and Robin post-check. */
  p->phase = PATH_BND_DISPATCH;
  res = step_bnd_dispatch(p, scn);
  /* Persist nbranchings */
  p->coupled_nbranchings = (int)p->ctx.nbranchings;

exit:
  return res;
error:
  goto exit;
}

/* --- PATH_COUPLED_CONDUCTIVE: wavefront delta_sphere entry/loop --------- */
LOCAL_SYM res_T
step_conductive(struct path_state* p, struct sdis_scene* scn)
{
  res_T res = RES_OK;
  ASSERT(p && scn);

  /* Custom paths and WoS: still delegate to the synchronous implementation.
   * Only delta_sphere is wavefront-ized. */
  if(p->ctx.diff_algo != SDIS_DIFFUSION_DELTA_SPHERE) {
    /* WoS or custom -- synchronous fallback */
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

  /* ----- Delta-sphere wavefront path (M4 fine-grained) ----- */

  /* Initialization: first entry dispatches ENC sub-state */
  if(!p->ds_initialized) {
    /* Emit 6-direction ENC query from current position.
     * After resolve, PATH_CND_DS_CHECK_TEMP finalises init. */
    d3_set(p->enc_query.query_pos, p->rwalk.vtx.P);
    p->enc_query.return_state = PATH_CND_DS_CHECK_TEMP;
    step_enc_query_emit(p);
    /* phase = PATH_ENC_QUERY_EMIT, needs_ray = 1 */
    goto exit;
  }

  /* Already initialised — go directly to temperature check */
  p->phase = PATH_CND_DS_CHECK_TEMP;

exit:
  return res;
error:
  /* Mark path as failed */
  p->phase = PATH_DONE;
  p->active = 0;
  p->done_reason = -1;
  goto exit;
}

/* --- PATH_COUPLED_COND_DS_PENDING / PATH_CND_DS_STEP_TRACE:
 *     process 2-ray results (delta sphere).
 *
 *     M4: this function now ONLY does hit processing + enc verification
 *     decision.  Volumic power / time rewind / position update / loop
 *     check are deferred to step_cnd_ds_step_advance().
 * -------------------------------------------------------------------- */
LOCAL_SYM res_T
step_conductive_ds_process(
  struct path_state* p,
  struct sdis_scene* scn,
  const struct s3d_hit* hit0,
  const struct s3d_hit* hit1)
{
  res_T res = RES_OK;
  float delta;
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

  /* Fix delta ~ hit0.distance edge case */
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
    /* No intersection drove delta -- dir1 doesn't correspond to a hit */
    p->ds_dir1[0] = 0; p->ds_dir1[1] = 0; p->ds_dir1[2] = 0;
    p->ds_hit1 = S3D_HIT_NULL;
  }
  p->ds_delta = delta;

  /* ------ Enclosure verification decision (M4: deferred to cascade) ---- */
  if(S3D_HIT_NONE(&p->ds_hit0) || p->ds_hit0.distance > delta) {
    /* No hit in forward direction at delta — need batched ENC query at
     * the projected next position.  Store pos_next for the ENC verify
     * step; cascade will call step_cnd_ds_step_enc_verify(). */
    double pos_next[3];
    d3_set(pos_next, p->rwalk.vtx.P);
    move_pos_3d(pos_next, p->ds_dir0, delta);
    d3_set(p->enc_query.query_pos, pos_next);
    p->phase = PATH_CND_DS_STEP_ENC_VERIFY;
    p->needs_ray = 0;
  } else {
    /* Hit at forward — get enclosure from hit primitive (pure compute) */
    unsigned enc_ids[2] = {ENCLOSURE_ID_NULL, ENCLOSURE_ID_NULL};
    unsigned enc_id;
    scene_get_enclosure_ids(scn, p->ds_hit0.prim.prim_id, enc_ids);
    enc_id = f3_dot(p->ds_dir0, p->ds_hit0.normal) < 0
           ? enc_ids[0] : enc_ids[1];

    if(enc_id != p->ds_enc_id) {
      /* Enclosure mismatch — retry with new direction (deferred) */
      p->ds_robust_attempt++;
      if(p->ds_robust_attempt >= 100) {
        log_warn(scn->dev,
          "wavefront: conductive delta_sphere robust exceeded 100 attempts "
          "at (%g, %g, %g)\n", SPLIT3(p->rwalk.vtx.P));
        res = RES_BAD_OP;
        goto error;
      }
      /* Cascade will enter step_cnd_ds_check_temp which re-emits rays */
      p->phase = PATH_CND_DS_CHECK_TEMP;
      p->needs_ray = 0;
    } else {
      /* Match — store result for step_advance check and proceed */
      p->enc_query.resolved_enc_id = enc_id;
      p->phase = PATH_CND_DS_STEP_ADVANCE;
      p->needs_ray = 0;
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

/*******************************************************************************
 * B-4 M4: Delta-sphere conductive fine-grained step functions
 ******************************************************************************/

/* --- PATH_CND_DS_CHECK_TEMP: finalize init (once) + loop top --------------- */
LOCAL_SYM res_T
step_cnd_ds_check_temp(struct path_state* p, struct sdis_scene* scn)
{
  res_T res = RES_OK;
  struct solid_props props = SOLID_PROPS_NULL;

  ASSERT(p && scn);

  /* Finalize initialisation from ENC resolve (runs only once per entry) */
  if(!p->ds_initialized) {
    unsigned enc_id = p->enc_query.resolved_enc_id;
    struct sdis_medium* mdm = NULL;

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

  /* --- Each loop iteration: fetch properties, check temperature --- */
  res = solid_get_properties(p->ds_medium, &p->rwalk.vtx, &props);
  if(res != RES_OK) goto error;
  res = check_solid_constant_properties(
    scn->dev, p->ctx.green_path != NULL, 0, &p->ds_props_ref, &props);
  if(res != RES_OK) goto error;

  /* Temperature known? */
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

  /* Store delta_solid parameter and reset retry counter */
  p->ds_delta_solid_param = (float)props.delta;
  p->ds_robust_attempt = 0;

  /* Emit 2 rays (forward + backward) */
  setup_delta_sphere_rays(p, scn);
  p->phase = PATH_CND_DS_STEP_TRACE;

exit:
  return res;
error:
  p->phase = PATH_DONE;
  p->active = 0;
  p->done_reason = -1;
  goto exit;
}

/* --- PATH_CND_DS_STEP_ENC_VERIFY: set up ENC sub-state at pos_next -------- */
LOCAL_SYM void
step_cnd_ds_step_enc_verify(struct path_state* p)
{
  ASSERT(p);
  /* enc_query.query_pos already set by step_conductive_ds_process */
  p->enc_query.return_state = PATH_CND_DS_STEP_ADVANCE;
  step_enc_query_emit(p);
  /* phase = PATH_ENC_QUERY_EMIT, needs_ray = 1 */
}

/* --- PATH_CND_DS_STEP_ADVANCE: enc check + volumic + time + pos + loop ---- */
LOCAL_SYM res_T
step_cnd_ds_step_advance(struct path_state* p, struct sdis_scene* scn)
{
  res_T res = RES_OK;
  struct solid_props props = SOLID_PROPS_NULL;
  const float delta = p->ds_delta;
  const float delta_solid = p->ds_delta_solid_param;
  double delta_m, mu;

  ASSERT(p && scn);

  /* ------ Enclosure verification (handles both direct-hit and ENC sub) --- */
  /* enc_query.resolved_enc_id is set by:
   *   - step_conductive_ds_process (direct from hit primitive), or
   *   - step_enc_query_resolve (from batched ENC sub-state).          */
  if(p->enc_query.resolved_enc_id != p->ds_enc_id) {
    /* Enclosure mismatch — retry with new direction */
    p->ds_robust_attempt++;
    if(p->ds_robust_attempt >= 100) {
      log_warn(scn->dev,
        "wavefront: conductive delta_sphere robust exceeded 100 attempts "
        "at (%g, %g, %g)\n", SPLIT3(p->rwalk.vtx.P));
      res = RES_BAD_OP;
      goto error;
    }
    /* Cascade will call step_cnd_ds_check_temp which re-emits rays */
    p->phase = PATH_CND_DS_CHECK_TEMP;
    p->needs_ray = 0;
    goto exit;
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
    /* Still inside solid — continue loop via check_temp */
    p->phase = PATH_CND_DS_CHECK_TEMP;
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

/*******************************************************************************
 * B-4 M1: Enclosure query sub-state machine
 ******************************************************************************/

/* Emit 6 rotated axis-aligned rays for enclosure identification.
 * Mirrors sdis_scene_Xd.h:scene_get_enclosure_id_in_closed_boundaries_3d.
 * The rotation by PI/4 around each axis avoids alignment with mesh edges. */
LOCAL_SYM void
step_enc_query_emit(struct path_state* p)
{
  float dirs[6][3] = {
    { 1, 0, 0}, {-1, 0, 0},
    { 0, 1, 0}, { 0,-1, 0},
    { 0, 0, 1}, { 0, 0,-1}
  };
  float frame[9];
  float pos[3];
  int i;

  ASSERT(p);

  /* Build rotation frame identical to the original function */
  f33_rotation(frame, (float)PI/4, (float)PI/4, (float)PI/4);

  /* Rotate all 6 directions */
  for(i = 0; i < 6; i++) {
    f33_mulf3(dirs[i], frame, dirs[i]);
    p->enc_query.directions[i][0] = dirs[i][0];
    p->enc_query.directions[i][1] = dirs[i][1];
    p->enc_query.directions[i][2] = dirs[i][2];
  }

  /* Set ray origin from query position */
  pos[0] = (float)p->enc_query.query_pos[0];
  pos[1] = (float)p->enc_query.query_pos[1];
  pos[2] = (float)p->enc_query.query_pos[2];
  p->ray_req.origin[0] = pos[0];
  p->ray_req.origin[1] = pos[1];
  p->ray_req.origin[2] = pos[2];

  /* Range: same as original [FLT_MIN, FLT_MAX] */
  p->ray_req.range[0] = FLT_MIN;
  p->ray_req.range[1] = FLT_MAX;

  /* Signal 6-ray extended request */
  p->ray_req.ray_count = 1; /* base count for collect compatibility */
  p->ray_count_ext = 6;
  p->ray_bucket = RAY_BUCKET_ENCLOSURE;
  p->needs_ray = 1;
  p->phase = PATH_ENC_QUERY_EMIT;
}

/* Resolve 6 directional hit results into an enclosure id.
 * Mirrors the original FOR_EACH(idir, 0, 6) loop + fallback logic. */
LOCAL_SYM res_T
step_enc_query_resolve(struct path_state* p, struct sdis_scene* scn)
{
  int i;
  unsigned enc_id = ENCLOSURE_ID_NULL;
  res_T res = RES_OK;

  ASSERT(p && scn);

  for(i = 0; i < 6; i++) {
    const struct s3d_hit* hit = &p->enc_query.dir_hits[i];
    float N[3];
    float cos_N_dir;

    if(S3D_HIT_NONE(hit)) continue;

    f3_normalize(N, hit->normal);
    cos_N_dir = f3_dot(N, p->enc_query.directions[i]);

    /* Same thresholds as original: distance > 1e-6, |cos| > 0.01 */
    if(hit->distance > 1.e-6f && fabsf(cos_N_dir) > 1.e-2f) {
      unsigned enc_ids[2];
      scene_get_enclosure_ids(scn, hit->prim.prim_id, enc_ids);
      enc_id = cos_N_dir < 0 ? enc_ids[0] : enc_ids[1];
      break; /* First valid hit determines enclosure */
    }
  }

  if(i >= 6) {
    /* All 6 directions failed -- fallback to brute-force traversal.
     * This matches the original fallback to scene_get_enclosure_id(). */
    res = scene_get_enclosure_id(scn, p->enc_query.query_pos, &enc_id);
    /* Swallow RES_BAD_OP: matches scene_get_enclosure_id_in_closed_boundaries
     * which returns enc_id=NULL on BAD_OP and lets caller decide to retry. */
    if(res == RES_BAD_OP) {
      enc_id = ENCLOSURE_ID_NULL;
      res = RES_OK;
    }
    if(res != RES_OK) {
      p->enc_query.resolved_enc_id = ENCLOSURE_ID_NULL;
      return res;
    }
  }

  p->enc_query.resolved_enc_id = enc_id;
  p->phase = p->enc_query.return_state;
  return RES_OK;
}

/*******************************************************************************
 * B-4 M3: Solid/Solid reinjection batch state machine
 ******************************************************************************/

/* Empirical scale factor from find_reinjection_ray */
#define SS_REINJECT_DST_MIN_SCALE 0.125f

/* Inlined version of sample_reinjection_dir_3d */
LOCAL_SYM void
wf_sample_reinjection_dir_3d(const struct rwalk* rwalk,
                              struct ssp_rng* rng,
                              float dir[3])
{
  float frame[9];
  ASSERT(rwalk && rng && dir);
  ssp_ran_circle_uniform_float(rng, dir, NULL);
  dir[2] = (float)(1.0/sqrt(2));
  f33_basis(frame, rwalk->hit_3d.normal);
  f33_mulf3(dir, frame, dir);
  f3_normalize(dir, dir);
}

/* Helper: setup the 4-ray request for solid/solid reinjection. */
LOCAL_SYM void
setup_ss_reinject_rays(struct path_state* p)
{
  float pos[3];
  float range[2];

  f3_set_d3(pos, p->rwalk.vtx.P);

  /* Filter data for self-intersection avoidance */
  p->filter_data_storage = HIT_FILTER_DATA_NULL;
  p->filter_data_storage.hit_3d = p->rwalk.hit_3d;
  p->filter_data_storage.epsilon =
      p->locals.bnd_ss.delta_boundary_frt * 0.01;

  /* Range: [0, FLT_MAX] (same as find_reinjection_ray) */
  range[0] = 0.0f;
  range[1] = FLT_MAX;

  /* Ray 0: front dir0 */
  p->ray_req.origin[0] = pos[0];
  p->ray_req.origin[1] = pos[1];
  p->ray_req.origin[2] = pos[2];
  p->ray_req.direction[0] = p->locals.bnd_ss.dir_frt[0][0];
  p->ray_req.direction[1] = p->locals.bnd_ss.dir_frt[0][1];
  p->ray_req.direction[2] = p->locals.bnd_ss.dir_frt[0][2];
  p->ray_req.range[0] = range[0];
  p->ray_req.range[1] = range[1];

  /* Ray 1: front dir1 */
  p->ray_req.direction2[0] = p->locals.bnd_ss.dir_frt[1][0];
  p->ray_req.direction2[1] = p->locals.bnd_ss.dir_frt[1][1];
  p->ray_req.direction2[2] = p->locals.bnd_ss.dir_frt[1][2];
  p->ray_req.range2[0] = range[0];
  p->ray_req.range2[1] = range[1];
  p->ray_req.ray_count = 2;

  /* B-4 M3: 4 rays total (frt*2 + bck*2), but ray_req only holds 2.
   * We use ray_count_ext=4 and emit the back side rays in collect. */
  p->ray_count_ext = 4;
  p->ray_bucket = RAY_BUCKET_STEP_PAIR;
  p->needs_ray = 1;
  p->phase = PATH_BND_SS_REINJECT_SAMPLE;
}

/* --- PATH_BND_SS_REINJECT_SAMPLE entry: prepare and emit 4 rays ---------- */
LOCAL_SYM res_T
step_bnd_ss_reinject_sample(struct path_state* p, struct sdis_scene* scn)
{
  struct sdis_interface* interf = NULL;
  struct sdis_medium* solid_frt = NULL;
  struct sdis_medium* solid_bck = NULL;
  unsigned enc_ids[2] = {ENCLOSURE_ID_NULL, ENCLOSURE_ID_NULL};
  const struct enclosure* enc_frt = NULL;
  const struct enclosure* enc_bck = NULL;
  float dir_frt_samp[3], dir_frt_refl[3];
  res_T res = RES_OK;

  ASSERT(p && scn);
  ASSERT(!S3D_HIT_NONE(&p->rwalk.hit_3d));

  /* Get enclosure ids and interface from the hit primitive */
  scene_get_enclosure_ids(scn, p->rwalk.hit_3d.prim.prim_id, enc_ids);
  interf = scene_get_interface(scn, p->rwalk.hit_3d.prim.prim_id);

  solid_frt = interface_get_medium(interf, SDIS_FRONT);
  solid_bck = interface_get_medium(interf, SDIS_BACK);
  ASSERT(solid_frt->type == SDIS_SOLID);
  ASSERT(solid_bck->type == SDIS_SOLID);

  /* Store enclosure ids */
  p->locals.bnd_ss.enc_ids[SDIS_FRONT] = enc_ids[SDIS_FRONT];
  p->locals.bnd_ss.enc_ids[SDIS_BACK]  = enc_ids[SDIS_BACK];

  /* Thermal contact resistance */
  {
    struct sdis_interface_fragment frag = SDIS_INTERFACE_FRAGMENT_NULL;
    struct sdis_rwalk_vertex vtx = SDIS_RWALK_VERTEX_NULL;
    d3_set(vtx.P, p->rwalk.vtx.P);
    vtx.time = p->rwalk.vtx.time;
    setup_interface_fragment_3d(&frag, &vtx, &p->rwalk.hit_3d,
                                p->rwalk.hit_side);
    p->locals.bnd_ss.tcr =
        interface_get_thermal_contact_resistance(interf, &frag);
  }

  /* Get thermal conductivity */
  p->locals.bnd_ss.lambda_frt =
      solid_get_thermal_conductivity(solid_frt, &p->rwalk.vtx);
  p->locals.bnd_ss.lambda_bck =
      solid_get_thermal_conductivity(solid_bck, &p->rwalk.vtx);

  /* Compute reinjection distances */
  p->locals.bnd_ss.delta_boundary_frt =
      solid_get_delta(solid_frt, &p->rwalk.vtx) * sqrt(3.0);
  p->locals.bnd_ss.delta_boundary_bck =
      solid_get_delta(solid_bck, &p->rwalk.vtx) * sqrt(3.0);

  /* Check MEDIUM_ID_MULTI enclosures */
  enc_frt = scene_get_enclosure(scn, enc_ids[SDIS_FRONT]);
  enc_bck = scene_get_enclosure(scn, enc_ids[SDIS_BACK]);
  p->locals.bnd_ss.multi_frt = (enc_frt->medium_id == MEDIUM_ID_MULTI) ? 1 : 0;
  p->locals.bnd_ss.multi_bck = (enc_bck->medium_id == MEDIUM_ID_MULTI) ? 1 : 0;

  /* If BOTH sides are multi-enclosure, short-circuit: build dummy reinjection
   * steps and go straight to decide. Mirrors the original code's early exit. */
  if(p->locals.bnd_ss.multi_frt && p->locals.bnd_ss.multi_bck) {
    /* Set up dummy reinjection data for both sides */
    f3_normalize(p->locals.bnd_ss.reinject_dir_frt, p->rwalk.hit_3d.normal);
    p->locals.bnd_ss.reinject_dst_frt =
        (float)p->locals.bnd_ss.delta_boundary_frt;
    p->locals.bnd_ss.reinject_hit_frt = S3D_HIT_NULL;
    f3_minus(p->locals.bnd_ss.reinject_dir_bck,
             p->locals.bnd_ss.reinject_dir_frt);
    p->locals.bnd_ss.reinject_dst_bck =
        (float)p->locals.bnd_ss.delta_boundary_bck;
    p->locals.bnd_ss.reinject_hit_bck = S3D_HIT_NULL;
    p->locals.bnd_ss.retry_count = 0;
    p->phase = PATH_BND_SS_REINJECT_DECIDE;
    p->needs_ray = 0;
    return RES_OK;
  }

  /* Save random walk position for retry logic */
  d3_set(p->locals.bnd_ss.rwalk_pos_backup, p->rwalk.vtx.P);
  p->locals.bnd_ss.retry_count = 0;
  p->locals.bnd_ss.position_was_moved = 0;

  /* Sample reinjection direction (RNG call matches original order) */
  wf_sample_reinjection_dir_3d(&p->rwalk, p->rng, dir_frt_samp);
  reflect_3d(dir_frt_refl, dir_frt_samp, p->rwalk.hit_3d.normal);

  /* Store 4 directions: frt_samp, frt_refl, bck_samp(-frt_samp), bck_refl(-frt_refl) */
  f3_set(p->locals.bnd_ss.dir_frt[0], dir_frt_samp);
  f3_set(p->locals.bnd_ss.dir_frt[1], dir_frt_refl);
  f3_minus(p->locals.bnd_ss.dir_bck[0], dir_frt_samp);
  f3_minus(p->locals.bnd_ss.dir_bck[1], dir_frt_refl);

  /* Initialize enc_id results */
  p->locals.bnd_ss.enc0_frt = ENCLOSURE_ID_NULL;
  p->locals.bnd_ss.enc1_frt = ENCLOSURE_ID_NULL;
  p->locals.bnd_ss.enc0_bck = ENCLOSURE_ID_NULL;
  p->locals.bnd_ss.enc1_bck = ENCLOSURE_ID_NULL;
  p->locals.bnd_ss.need_enc_frt = 0;
  p->locals.bnd_ss.need_enc_bck = 0;

  /* Emit 4 trace rays */
  setup_ss_reinject_rays(p);
  return RES_OK;
}

/* Helper: resolve reinjection ray direction/distance from 2 candidates. */
LOCAL_SYM void
resolve_reinjection_from_hits(
  const struct s3d_hit* hit0,    /* hit along dir0 */
  const struct s3d_hit* hit1,    /* hit along dir1 */
  const float dir0[3],
  const float dir1[3],
  unsigned enc0_id,              /* enclosure at dir0 endpoint */
  unsigned enc1_id,              /* enclosure at dir1 endpoint */
  unsigned solid_enc_id,         /* target solid enclosure */
  double distance,               /* max reinjection distance */
  float out_dir[3],
  float* out_dst,
  struct s3d_hit* out_hit)
{
  const float REINJECT_DST_MIN_SCALE_local = 0.125f;
  double dst_adjusted = distance * RAY_RANGE_MAX_SCALE;
  float reinject_threshold = (float)distance * REINJECT_DST_MIN_SCALE_local;
  double dst0 = -1, dst1 = -1;
  struct s3d_hit h0_local, h1_local;

  h0_local = *hit0;
  h1_local = *hit1;

  /* Check dir0 validity (enclosure must match solid) */
  if(enc0_id == solid_enc_id) {
    if(!S3D_HIT_NONE(hit0) && hit0->distance <= dst_adjusted) {
      dst0 = hit0->distance;
    } else {
      dst0 = distance;
      h0_local = S3D_HIT_NULL;
    }
  }

  /* Check dir1 validity */
  if(enc1_id == solid_enc_id) {
    if(!S3D_HIT_NONE(hit1) && hit1->distance <= dst_adjusted) {
      dst1 = hit1->distance;
    } else {
      dst1 = distance;
      h1_local = S3D_HIT_NULL;
    }
  }

  /* Both invalid */
  if(dst0 == -1 && dst1 == -1) {
    /* Signal failure: set dst to 0 */
    *out_dst = 0;
    *out_hit = S3D_HIT_NULL;
    f3_set(out_dir, dir0);
    return;
  }

  /* Direction selection logic (mirrors find_reinjection_ray) */
  if(dst0 == -1) {
    f3_set(out_dir, dir1);
    *out_dst = (float)dst1;
    *out_hit = h1_local;
  } else if(dst1 == -1) {
    f3_set(out_dir, dir0);
    *out_dst = (float)dst0;
    *out_hit = h0_local;
  } else if(dst0 < reinject_threshold && dst1 < reinject_threshold) {
    if(dst0 > dst1) {
      f3_set(out_dir, dir0); *out_dst = (float)dst0; *out_hit = h0_local;
    } else {
      f3_set(out_dir, dir1); *out_dst = (float)dst1; *out_hit = h1_local;
    }
  } else if(dst0 < reinject_threshold) {
    f3_set(out_dir, dir1); *out_dst = (float)dst1; *out_hit = h1_local;
  } else if(dst1 < reinject_threshold) {
    f3_set(out_dir, dir0); *out_dst = (float)dst0; *out_hit = h0_local;
  } else {
    /* Both valid: choose dir0, adjust distance */
    f3_set(out_dir, dir0);
    if(dst0 <= dst1) {
      *out_dst = (float)dst0;
      *out_hit = h0_local;
    } else {
      *out_dst = (float)dst1;
      *out_hit = S3D_HIT_NULL;
    }
    /* Snap to boundary if close */
    if(!S3D_HIT_NONE(hit0) && dst0 != *out_dst
    && fabs(dst0 - *out_dst) < dst0 * 0.1) {
      *out_dst = (float)dst0;
      *out_hit = h0_local;
    }
  }
}

/* --- PATH_BND_SS_REINJECT_SAMPLE: process 4-ray results ------------------- */
LOCAL_SYM res_T
step_bnd_ss_reinject_process(
  struct path_state* p,
  struct sdis_scene* scn,
  const struct s3d_hit* hit_frt0,
  const struct s3d_hit* hit_frt1,
  const struct s3d_hit* hit_bck0,
  const struct s3d_hit* hit_bck1)
{
  res_T res = RES_OK;
  unsigned enc_ids_tmp[2];
  enum sdis_side side;
  int frt_valid, bck_valid;

  ASSERT(p && scn);

  /* Store hits */
  p->locals.bnd_ss.ray_frt[0] = *hit_frt0;
  p->locals.bnd_ss.ray_frt[1] = *hit_frt1;
  p->locals.bnd_ss.ray_bck[0] = *hit_bck0;
  p->locals.bnd_ss.ray_bck[1] = *hit_bck1;

  /* --- Determine enclosure at each reinjection endpoint --- */
  /* Front dir0 */
  if(!p->locals.bnd_ss.multi_frt) {
    if(!S3D_HIT_NONE(hit_frt0)) {
      scene_get_enclosure_ids(scn, hit_frt0->prim.prim_id, enc_ids_tmp);
      side = f3_dot(p->locals.bnd_ss.dir_frt[0], hit_frt0->normal) < 0
           ? SDIS_FRONT : SDIS_BACK;
      p->locals.bnd_ss.enc0_frt = enc_ids_tmp[side];
    } else {
      p->locals.bnd_ss.need_enc_frt = 1; /* will need ENC query */
      p->locals.bnd_ss.enc0_frt = ENCLOSURE_ID_NULL;
    }

    /* Front dir1 */
    if(!S3D_HIT_NONE(hit_frt1)) {
      scene_get_enclosure_ids(scn, hit_frt1->prim.prim_id, enc_ids_tmp);
      side = f3_dot(p->locals.bnd_ss.dir_frt[1], hit_frt1->normal) < 0
           ? SDIS_FRONT : SDIS_BACK;
      p->locals.bnd_ss.enc1_frt = enc_ids_tmp[side];
    } else {
      p->locals.bnd_ss.need_enc_frt = 1;
      p->locals.bnd_ss.enc1_frt = ENCLOSURE_ID_NULL;
    }
  }

  /* Back dir0 */
  if(!p->locals.bnd_ss.multi_bck) {
    if(!S3D_HIT_NONE(hit_bck0)) {
      scene_get_enclosure_ids(scn, hit_bck0->prim.prim_id, enc_ids_tmp);
      side = f3_dot(p->locals.bnd_ss.dir_bck[0], hit_bck0->normal) < 0
           ? SDIS_FRONT : SDIS_BACK;
      p->locals.bnd_ss.enc0_bck = enc_ids_tmp[side];
    } else {
      p->locals.bnd_ss.need_enc_bck = 1;
      p->locals.bnd_ss.enc0_bck = ENCLOSURE_ID_NULL;
    }

    /* Back dir1 */
    if(!S3D_HIT_NONE(hit_bck1)) {
      scene_get_enclosure_ids(scn, hit_bck1->prim.prim_id, enc_ids_tmp);
      side = f3_dot(p->locals.bnd_ss.dir_bck[1], hit_bck1->normal) < 0
           ? SDIS_FRONT : SDIS_BACK;
      p->locals.bnd_ss.enc1_bck = enc_ids_tmp[side];
    } else {
      p->locals.bnd_ss.need_enc_bck = 1;
      p->locals.bnd_ss.enc1_bck = ENCLOSURE_ID_NULL;
    }
  }

  /* Try to resolve front reinjection */
  frt_valid = 1;
  if(!p->locals.bnd_ss.multi_frt) {
    resolve_reinjection_from_hits(
      &p->locals.bnd_ss.ray_frt[0], &p->locals.bnd_ss.ray_frt[1],
      p->locals.bnd_ss.dir_frt[0], p->locals.bnd_ss.dir_frt[1],
      p->locals.bnd_ss.enc0_frt, p->locals.bnd_ss.enc1_frt,
      p->locals.bnd_ss.enc_ids[SDIS_FRONT],
      p->locals.bnd_ss.delta_boundary_frt,
      p->locals.bnd_ss.reinject_dir_frt,
      &p->locals.bnd_ss.reinject_dst_frt,
      &p->locals.bnd_ss.reinject_hit_frt);
    frt_valid = (p->locals.bnd_ss.reinject_dst_frt > 0);
  } else {
    /* Multi-enclosure: use normalised normal as direction */
    f3_normalize(p->locals.bnd_ss.reinject_dir_frt, p->rwalk.hit_3d.normal);
    p->locals.bnd_ss.reinject_dst_frt =
        (float)p->locals.bnd_ss.delta_boundary_frt;
    p->locals.bnd_ss.reinject_hit_frt = S3D_HIT_NULL;
  }

  /* Try to resolve back reinjection */
  bck_valid = 1;
  if(!p->locals.bnd_ss.multi_bck) {
    resolve_reinjection_from_hits(
      &p->locals.bnd_ss.ray_bck[0], &p->locals.bnd_ss.ray_bck[1],
      p->locals.bnd_ss.dir_bck[0], p->locals.bnd_ss.dir_bck[1],
      p->locals.bnd_ss.enc0_bck, p->locals.bnd_ss.enc1_bck,
      p->locals.bnd_ss.enc_ids[SDIS_BACK],
      p->locals.bnd_ss.delta_boundary_bck,
      p->locals.bnd_ss.reinject_dir_bck,
      &p->locals.bnd_ss.reinject_dst_bck,
      &p->locals.bnd_ss.reinject_hit_bck);
    bck_valid = (p->locals.bnd_ss.reinject_dst_bck > 0);
  } else {
    float tmp[3];
    f3_normalize(tmp, p->rwalk.hit_3d.normal);
    f3_minus(p->locals.bnd_ss.reinject_dir_bck, tmp);
    p->locals.bnd_ss.reinject_dst_bck =
        (float)p->locals.bnd_ss.delta_boundary_bck;
    p->locals.bnd_ss.reinject_hit_bck = S3D_HIT_NULL;
  }

  /* Both sides failed: retry with new direction (matches original retry loop) */
  if(!frt_valid && !bck_valid) {
    p->locals.bnd_ss.retry_count++;
    if(p->locals.bnd_ss.retry_count >= 10) {
      log_warn(scn->dev,
        "wavefront M3: solid/solid reinjection failed after 10 attempts "
        "at (%g, %g, %g)\n", SPLIT3(p->rwalk.vtx.P));
      res = RES_BAD_OP_IRRECOVERABLE;
      goto error;
    }
    /* Restore position and retry */
    d3_set(p->rwalk.vtx.P, p->locals.bnd_ss.rwalk_pos_backup);

    /* Re-sample direction (consumes RNG in same order as original retry) */
    {
      float dir_frt_samp_r[3], dir_frt_refl_r[3];
      wf_sample_reinjection_dir_3d(&p->rwalk, p->rng, dir_frt_samp_r);
      reflect_3d(dir_frt_refl_r, dir_frt_samp_r, p->rwalk.hit_3d.normal);
      f3_set(p->locals.bnd_ss.dir_frt[0], dir_frt_samp_r);
      f3_set(p->locals.bnd_ss.dir_frt[1], dir_frt_refl_r);
      f3_minus(p->locals.bnd_ss.dir_bck[0], dir_frt_samp_r);
      f3_minus(p->locals.bnd_ss.dir_bck[1], dir_frt_refl_r);
    }
    p->locals.bnd_ss.enc0_frt = ENCLOSURE_ID_NULL;
    p->locals.bnd_ss.enc1_frt = ENCLOSURE_ID_NULL;
    p->locals.bnd_ss.enc0_bck = ENCLOSURE_ID_NULL;
    p->locals.bnd_ss.enc1_bck = ENCLOSURE_ID_NULL;
    p->locals.bnd_ss.need_enc_frt = 0;
    p->locals.bnd_ss.need_enc_bck = 0;

    setup_ss_reinject_rays(p);
    return RES_OK; /* re-emit rays, stay in REINJECT_SAMPLE */
  }

  /* At least one side is valid (or multi): check if we need ENC verification
   * at the endpoint for miss reinjections. */
  if(!p->locals.bnd_ss.multi_frt && frt_valid
  && S3D_HIT_NONE(&p->locals.bnd_ss.reinject_hit_frt)) {
    /* Need to verify enclosure at front reinjection point */
    double pos[3];
    d3_set(pos, p->rwalk.vtx.P);
    move_pos_3d(pos, p->locals.bnd_ss.reinject_dir_frt,
                p->locals.bnd_ss.reinject_dst_frt);
    d3_set(p->enc_query.query_pos, pos);
    p->enc_query.return_state = PATH_BND_SS_REINJECT_ENC;
    p->locals.bnd_ss.enc_side = 0; /* front */
    step_enc_query_emit(p);
    return RES_OK;
  }

  if(!p->locals.bnd_ss.multi_bck && bck_valid
  && S3D_HIT_NONE(&p->locals.bnd_ss.reinject_hit_bck)) {
    /* Need to verify enclosure at back reinjection point */
    double pos[3];
    d3_set(pos, p->rwalk.vtx.P);
    move_pos_3d(pos, p->locals.bnd_ss.reinject_dir_bck,
                p->locals.bnd_ss.reinject_dst_bck);
    d3_set(p->enc_query.query_pos, pos);
    p->enc_query.return_state = PATH_BND_SS_REINJECT_ENC;
    p->locals.bnd_ss.enc_side = 1; /* back */
    step_enc_query_emit(p);
    return RES_OK;
  }

  /* All enclosures resolved -- go to decide */
  p->phase = PATH_BND_SS_REINJECT_DECIDE;
  p->needs_ray = 0;
  return RES_OK;

error:
  p->phase = PATH_DONE;
  p->active = 0;
  p->done_reason = -1;
  return res;
}

/* --- PATH_BND_SS_REINJECT_ENC: ENC query result for reinjection verify ---- */
LOCAL_SYM res_T
step_bnd_ss_reinject_enc_result(struct path_state* p, struct sdis_scene* scn)
{
  unsigned enc_id;
  unsigned target_enc_id;
  res_T res = RES_OK;

  ASSERT(p && scn);

  enc_id = p->enc_query.resolved_enc_id;

  if(p->locals.bnd_ss.enc_side == 0) {
    /* Front side verification */
    target_enc_id = p->locals.bnd_ss.enc_ids[SDIS_FRONT];
    if(enc_id != target_enc_id) {
      /* Front reinjection invalid */
      p->locals.bnd_ss.reinject_dst_frt = 0;
    }

    /* Now check if back side also needs ENC verification */
    if(!p->locals.bnd_ss.multi_bck
    && p->locals.bnd_ss.reinject_dst_bck > 0
    && S3D_HIT_NONE(&p->locals.bnd_ss.reinject_hit_bck)) {
      double pos[3];
      d3_set(pos, p->rwalk.vtx.P);
      move_pos_3d(pos, p->locals.bnd_ss.reinject_dir_bck,
                  p->locals.bnd_ss.reinject_dst_bck);
      d3_set(p->enc_query.query_pos, pos);
      p->enc_query.return_state = PATH_BND_SS_REINJECT_ENC;
      p->locals.bnd_ss.enc_side = 1; /* back */
      step_enc_query_emit(p);
      return RES_OK;
    }
  } else {
    /* Back side verification */
    target_enc_id = p->locals.bnd_ss.enc_ids[SDIS_BACK];
    if(enc_id != target_enc_id) {
      /* Back reinjection invalid */
      p->locals.bnd_ss.reinject_dst_bck = 0;
    }
  }

  /* Both ENC verifications done. Check if we still have valid reinjections. */
  {
    int frt_valid = p->locals.bnd_ss.multi_frt
                 || (p->locals.bnd_ss.reinject_dst_frt > 0);
    int bck_valid = p->locals.bnd_ss.multi_bck
                 || (p->locals.bnd_ss.reinject_dst_bck > 0);

    if(!frt_valid && !bck_valid) {
      /* Both invalid after ENC check -- retry */
      p->locals.bnd_ss.retry_count++;
      if(p->locals.bnd_ss.retry_count >= 10) {
        log_warn(scn->dev,
          "wavefront M3: solid/solid reinjection failed (ENC) at "
          "(%g, %g, %g)\n", SPLIT3(p->rwalk.vtx.P));
        p->phase = PATH_DONE;
        p->active = 0;
        p->done_reason = -1;
        return RES_BAD_OP_IRRECOVERABLE;
      }
      d3_set(p->rwalk.vtx.P, p->locals.bnd_ss.rwalk_pos_backup);

      {
        float dir_frt_samp[3], dir_frt_refl[3];
        wf_sample_reinjection_dir_3d(&p->rwalk, p->rng, dir_frt_samp);
        reflect_3d(dir_frt_refl, dir_frt_samp, p->rwalk.hit_3d.normal);
        f3_set(p->locals.bnd_ss.dir_frt[0], dir_frt_samp);
        f3_set(p->locals.bnd_ss.dir_frt[1], dir_frt_refl);
        f3_minus(p->locals.bnd_ss.dir_bck[0], dir_frt_samp);
        f3_minus(p->locals.bnd_ss.dir_bck[1], dir_frt_refl);
      }
      p->locals.bnd_ss.enc0_frt = ENCLOSURE_ID_NULL;
      p->locals.bnd_ss.enc1_frt = ENCLOSURE_ID_NULL;
      p->locals.bnd_ss.enc0_bck = ENCLOSURE_ID_NULL;
      p->locals.bnd_ss.enc1_bck = ENCLOSURE_ID_NULL;
      p->locals.bnd_ss.need_enc_frt = 0;
      p->locals.bnd_ss.need_enc_bck = 0;

      setup_ss_reinject_rays(p);
      return RES_OK;
    }
  }

  p->phase = PATH_BND_SS_REINJECT_DECIDE;
  p->needs_ray = 0;
  return RES_OK;
}

/* --- PATH_BND_SS_REINJECT_DECIDE: probability choice + solid_reinjection -- */
LOCAL_SYM res_T
step_bnd_ss_reinject_decide(struct path_state* p, struct sdis_scene* scn)
{
  double r, proba;
  float* chosen_dir;
  float  chosen_dst;
  struct s3d_hit chosen_hit;
  unsigned solid_enc_id;
  struct reinjection_step reinject_step = REINJECTION_STEP_NULL;
  struct solid_reinjection_args solid_reinject_args = SOLID_REINJECTION_ARGS_NULL;
  res_T res = RES_OK;

  ASSERT(p && scn);

  /* Compute probability (mirrors solid_solid_boundary_path_3d) */
  if(p->locals.bnd_ss.tcr == 0) {
    /* No thermal contact resistance */
    const double tmp_frt =
        p->locals.bnd_ss.lambda_frt / p->locals.bnd_ss.reinject_dst_frt;
    const double tmp_bck =
        p->locals.bnd_ss.lambda_bck / p->locals.bnd_ss.reinject_dst_bck;
    proba = tmp_frt / (tmp_frt + tmp_bck);
  } else {
    const double delta_frt =
        p->locals.bnd_ss.reinject_dst_frt / sqrt(3.0);
    const double delta_bck =
        p->locals.bnd_ss.reinject_dst_bck / sqrt(3.0);
    const double tmp_frt = p->locals.bnd_ss.lambda_frt / delta_frt;
    const double tmp_bck = p->locals.bnd_ss.lambda_bck / delta_bck;
    const double tmp_tcr =
        p->locals.bnd_ss.tcr * tmp_frt * tmp_bck;
    switch(p->rwalk.hit_side) {
    case SDIS_BACK:
      proba = tmp_frt / (tmp_frt + tmp_bck + tmp_tcr);
      break;
    case SDIS_FRONT:
      proba = (tmp_frt + tmp_tcr) / (tmp_frt + tmp_bck + tmp_tcr);
      break;
    default:
      FATAL("wavefront M3: unreachable hit_side\n");
      proba = 0.5; /* suppress warning */
      break;
    }
  }

  /* Random choice: front or back */
  r = ssp_rng_canonical(p->rng);
  if(r < proba) {
    chosen_dir = p->locals.bnd_ss.reinject_dir_frt;
    chosen_dst = p->locals.bnd_ss.reinject_dst_frt;
    chosen_hit = p->locals.bnd_ss.reinject_hit_frt;
    solid_enc_id = p->locals.bnd_ss.enc_ids[SDIS_FRONT];
  } else {
    chosen_dir = p->locals.bnd_ss.reinject_dir_bck;
    chosen_dst = p->locals.bnd_ss.reinject_dst_bck;
    chosen_hit = p->locals.bnd_ss.reinject_hit_bck;
    solid_enc_id = p->locals.bnd_ss.enc_ids[SDIS_BACK];
  }

  /* Build reinjection_step for solid_reinjection call */
  f3_set(reinject_step.direction, chosen_dir);
  reinject_step.distance = chosen_dst;
  reinject_step.hit_3d = chosen_hit;

  /* Call solid_reinjection synchronously (pure compute: time_rewind + move) */
  solid_reinject_args.reinjection = &reinject_step;
  solid_reinject_args.rng = p->rng;
  solid_reinject_args.rwalk = &p->rwalk;
  solid_reinject_args.rwalk_ctx = &p->ctx;
  solid_reinject_args.T = &p->T;
  solid_reinject_args.fp_to_meter = scn->fp_to_meter;
  res = solid_reinjection_3d(scn, solid_enc_id, &solid_reinject_args);
  if(res != RES_OK) goto error;

  /* After solid_reinjection, check where we go next */
  if(p->T.done) {
    p->phase = PATH_DONE;
    p->active = 0;
    p->done_reason = 4; /* time rewind / temperature found */
    goto exit;
  }

  /* solid_reinjection sets T.func to indicate next step */
  if(p->T.func == conductive_path_3d) {
    p->ds_initialized = 0; /* reset for fresh conductive entry */
    p->phase = PATH_COUPLED_CONDUCTIVE;
  } else if(p->T.func == boundary_path_3d) {
    p->phase = PATH_COUPLED_BOUNDARY;
  } else {
    FATAL("wavefront M3: unexpected T.func after solid_reinjection\n");
  }
  p->needs_ray = 0;

exit:
  return RES_OK;
error:
  p->phase = PATH_DONE;
  p->active = 0;
  p->done_reason = -1;
  return res;
}

/*******************************************************************************
 * B-4 M5: Solid/fluid picard1 batch state machine
 *
 * Replaces the synchronous solid_fluid_boundary_picard1_path_3d() call with
 * fine-grained states:
 *   SF_REINJECT_SAMPLE  [R] → 2 reinjection rays (solid side)
 *   SF_REINJECT_ENC     [R] → ENC sub-query if miss
 *   SF_PROB_DISPATCH    [C] → probability dispatch (conv/cond/rad)
 *   SF_NULLCOLL_RAD_TRACE [R] → radiative sub-path trace
 *   SF_NULLCOLL_DECIDE  [C] → accept/reject radiative path
 ******************************************************************************/

/* Helper: set up 2-ray request for SF reinjection (dir0 + reflect(dir0)) */
LOCAL_SYM void
setup_sf_reinject_rays(struct path_state* p)
{
  float pos[3];

  f3_set_d3(pos, p->rwalk.vtx.P);

  /* Filter: self-intersection avoidance */
  p->filter_data_storage = HIT_FILTER_DATA_NULL;
  p->filter_data_storage.hit_3d = p->rwalk.hit_3d;
  p->filter_data_storage.epsilon =
      p->locals.bnd_sf.delta_boundary * 0.01;

  /* Ray 0: dir0 */
  p->ray_req.origin[0] = pos[0];
  p->ray_req.origin[1] = pos[1];
  p->ray_req.origin[2] = pos[2];
  p->ray_req.direction[0] = p->locals.bnd_sf.reinject_dir[0][0];
  p->ray_req.direction[1] = p->locals.bnd_sf.reinject_dir[0][1];
  p->ray_req.direction[2] = p->locals.bnd_sf.reinject_dir[0][2];
  p->ray_req.range[0] = 0.0f;
  p->ray_req.range[1] = FLT_MAX;

  /* Ray 1: dir1 (reflected) */
  p->ray_req.direction2[0] = p->locals.bnd_sf.reinject_dir[1][0];
  p->ray_req.direction2[1] = p->locals.bnd_sf.reinject_dir[1][1];
  p->ray_req.direction2[2] = p->locals.bnd_sf.reinject_dir[1][2];
  p->ray_req.range2[0] = 0.0f;
  p->ray_req.range2[1] = FLT_MAX;
  p->ray_req.ray_count = 2;

  p->ray_count_ext = 2;
  p->ray_bucket = RAY_BUCKET_STEP_PAIR;
  p->needs_ray = 1;
  p->phase = PATH_BND_SF_REINJECT_SAMPLE;
}

/* --- PATH_BND_SF_REINJECT_SAMPLE entry: prepare interface + emit 2 rays -- */
LOCAL_SYM res_T
step_bnd_sf_reinject_sample(struct path_state* p, struct sdis_scene* scn)
{
  struct sdis_interface* interf = NULL;
  struct sdis_medium* solid = NULL;
  struct sdis_medium* fluid = NULL;
  unsigned enc_ids[2] = {ENCLOSURE_ID_NULL, ENCLOSURE_ID_NULL};
  const struct enclosure* solid_enc = NULL;
  float dir_samp[3], dir_refl[3];
  res_T res = RES_OK;

  ASSERT(p && scn);
  ASSERT(!S3D_HIT_NONE(&p->rwalk.hit_3d));

  /* Get interface and determine solid/fluid sides */
  interf = scene_get_interface(scn, p->rwalk.hit_3d.prim.prim_id);
  solid = interface_get_medium(interf, SDIS_FRONT);
  fluid = interface_get_medium(interf, SDIS_BACK);
  p->locals.bnd_sf.solid_side = SDIS_FRONT;
  p->locals.bnd_sf.fluid_side = SDIS_BACK;
  if(solid->type != SDIS_SOLID) {
    SWAP(struct sdis_medium*, solid, fluid);
    p->locals.bnd_sf.solid_side = SDIS_BACK;
    p->locals.bnd_sf.fluid_side = SDIS_FRONT;
    ASSERT(fluid->type == SDIS_FLUID);
  }

  /* Get enclosure ids */
  scene_get_enclosure_ids(scn, p->rwalk.hit_3d.prim.prim_id, enc_ids);
  p->locals.bnd_sf.enc_ids[SDIS_FRONT] = enc_ids[SDIS_FRONT];
  p->locals.bnd_sf.enc_ids[SDIS_BACK] = enc_ids[SDIS_BACK];
  p->locals.bnd_sf.solid_enc_id = enc_ids[p->locals.bnd_sf.solid_side];

  /* Get solid thermal conductivity and delta */
  p->locals.bnd_sf.lambda =
      solid_get_thermal_conductivity(solid, &p->rwalk.vtx);
  {
    double delta = solid_get_delta(solid, &p->rwalk.vtx);
    p->locals.bnd_sf.delta_boundary = sqrt(3.0) * delta; /* DIM=3 */
  }

  /* Get emissivity from fluid side */
  {
    struct sdis_interface_fragment frag_fluid = SDIS_INTERFACE_FRAGMENT_NULL;
    setup_interface_fragment_3d(&frag_fluid, &p->rwalk.vtx,
                                &p->rwalk.hit_3d, p->locals.bnd_sf.fluid_side);
    p->locals.bnd_sf.epsilon = interface_side_get_emissivity(
        interf, SDIS_INTERN_SOURCE_ID, &frag_fluid);

    if(p->locals.bnd_sf.epsilon <= 0) {
      p->locals.bnd_sf.Tref = 0;
    } else {
      p->locals.bnd_sf.Tref =
          interface_side_get_reference_temperature(interf, &frag_fluid);
      res = check_Tref_3d(scn, frag_fluid.P, p->locals.bnd_sf.Tref,
                          "step_bnd_sf_reinject_sample");
      if(res != RES_OK) goto error;
    }
  }

  /* Check MEDIUM_ID_MULTI enclosure (no geometry → use normal direction) */
  solid_enc = scene_get_enclosure(scn, p->locals.bnd_sf.solid_enc_id);
  if(solid_enc->medium_id == MEDIUM_ID_MULTI) {
    /* Multi-enclosure: skip ray trace, use normal as reinjection direction */
    float N[3];
    f3_normalize(N, p->rwalk.hit_3d.normal);
    /* Ensure direction points into solid side */
    if(p->locals.bnd_sf.solid_side == SDIS_BACK) {
      f3_minus(N, N);
    }
    p->locals.bnd_sf.chosen_dir[0] = N[0];
    p->locals.bnd_sf.chosen_dir[1] = N[1];
    p->locals.bnd_sf.chosen_dir[2] = N[2];
    p->locals.bnd_sf.chosen_dst = (float)p->locals.bnd_sf.delta_boundary;
    p->locals.bnd_sf.chosen_hit = S3D_HIT_NULL;
    p->locals.bnd_sf.retry_count = 0;

    /* Skip ray trace, go directly to prob_dispatch via solid_reinjection */
    p->phase = PATH_BND_SF_PROB_DISPATCH;
    p->needs_ray = 0;
    return RES_OK;
  }

  /* Save position backup for retry logic */
  d3_set(p->locals.bnd_sf.rwalk_pos_backup, p->rwalk.vtx.P);
  p->locals.bnd_sf.retry_count = 0;
  p->locals.bnd_sf.need_enc = 0;

  /* Sample reinjection direction (matches original RNG order) */
  wf_sample_reinjection_dir_3d(&p->rwalk, p->rng, dir_samp);
  reflect_3d(dir_refl, dir_samp, p->rwalk.hit_3d.normal);

  /* Ensure directions point into solid side */
  if(p->locals.bnd_sf.solid_side == SDIS_BACK) {
    f3_minus(dir_samp, dir_samp);
    f3_minus(dir_refl, dir_refl);
  }

  /* Store 2 directions */
  f3_set(p->locals.bnd_sf.reinject_dir[0], dir_samp);
  f3_set(p->locals.bnd_sf.reinject_dir[1], dir_refl);

  /* Initialize enclosure id results */
  p->locals.bnd_sf.enc0_id = ENCLOSURE_ID_NULL;
  p->locals.bnd_sf.enc1_id = ENCLOSURE_ID_NULL;

  /* Emit 2 reinjection rays */
  setup_sf_reinject_rays(p);
  return RES_OK;

error:
  p->phase = PATH_DONE;
  p->active = 0;
  p->done_reason = -1;
  return res;
}

/* --- PATH_BND_SF_REINJECT_SAMPLE: process 2-ray reinjection results ------ */
LOCAL_SYM res_T
step_bnd_sf_reinject_process(
  struct path_state* p,
  struct sdis_scene* scn,
  const struct s3d_hit* hit0,
  const struct s3d_hit* hit1)
{
  unsigned enc_ids_tmp[2];
  enum sdis_side side;
  int valid;
  res_T res = RES_OK;

  ASSERT(p && scn);

  /* Store hits */
  p->locals.bnd_sf.reinject_hit[0] = *hit0;
  p->locals.bnd_sf.reinject_hit[1] = *hit1;

  /* Determine enclosure at each reinjection endpoint */
  if(!S3D_HIT_NONE(hit0)) {
    scene_get_enclosure_ids(scn, hit0->prim.prim_id, enc_ids_tmp);
    side = f3_dot(p->locals.bnd_sf.reinject_dir[0], hit0->normal) < 0
         ? SDIS_FRONT : SDIS_BACK;
    p->locals.bnd_sf.enc0_id = enc_ids_tmp[side];
  } else {
    p->locals.bnd_sf.need_enc = 1;
    p->locals.bnd_sf.enc0_id = ENCLOSURE_ID_NULL;
  }

  if(!S3D_HIT_NONE(hit1)) {
    scene_get_enclosure_ids(scn, hit1->prim.prim_id, enc_ids_tmp);
    side = f3_dot(p->locals.bnd_sf.reinject_dir[1], hit1->normal) < 0
         ? SDIS_FRONT : SDIS_BACK;
    p->locals.bnd_sf.enc1_id = enc_ids_tmp[side];
  } else {
    p->locals.bnd_sf.need_enc = 1;
    p->locals.bnd_sf.enc1_id = ENCLOSURE_ID_NULL;
  }

  /* Resolve reinjection from 2 hits (reuse M3 utility) */
  resolve_reinjection_from_hits(
    hit0, hit1,
    p->locals.bnd_sf.reinject_dir[0], p->locals.bnd_sf.reinject_dir[1],
    p->locals.bnd_sf.enc0_id, p->locals.bnd_sf.enc1_id,
    p->locals.bnd_sf.solid_enc_id,
    p->locals.bnd_sf.delta_boundary,
    p->locals.bnd_sf.chosen_dir,
    &p->locals.bnd_sf.chosen_dst,
    &p->locals.bnd_sf.chosen_hit);

  valid = (p->locals.bnd_sf.chosen_dst > 0);

  if(!valid) {
    /* Both directions invalid: retry with new direction */
    p->locals.bnd_sf.retry_count++;
    if(p->locals.bnd_sf.retry_count >= 10) {
      log_warn(scn->dev,
        "wavefront M5: SF reinjection failed after 10 attempts at "
        "(%g, %g, %g)\n", SPLIT3(p->rwalk.vtx.P));
      res = RES_BAD_OP_IRRECOVERABLE;
      goto error;
    }
    /* Restore position and re-sample */
    d3_set(p->rwalk.vtx.P, p->locals.bnd_sf.rwalk_pos_backup);

    {
      float dir_samp[3], dir_refl[3];
      wf_sample_reinjection_dir_3d(&p->rwalk, p->rng, dir_samp);
      reflect_3d(dir_refl, dir_samp, p->rwalk.hit_3d.normal);
      if(p->locals.bnd_sf.solid_side == SDIS_BACK) {
        f3_minus(dir_samp, dir_samp);
        f3_minus(dir_refl, dir_refl);
      }
      f3_set(p->locals.bnd_sf.reinject_dir[0], dir_samp);
      f3_set(p->locals.bnd_sf.reinject_dir[1], dir_refl);
    }
    p->locals.bnd_sf.enc0_id = ENCLOSURE_ID_NULL;
    p->locals.bnd_sf.enc1_id = ENCLOSURE_ID_NULL;
    p->locals.bnd_sf.need_enc = 0;

    setup_sf_reinject_rays(p);
    return RES_OK; /* re-emit rays */
  }

  /* Check if ENC verification is needed (miss reinjection) */
  if(S3D_HIT_NONE(&p->locals.bnd_sf.chosen_hit)) {
    double pos[3];
    d3_set(pos, p->rwalk.vtx.P);
    move_pos_3d(pos, p->locals.bnd_sf.chosen_dir,
                p->locals.bnd_sf.chosen_dst);
    d3_set(p->enc_query.query_pos, pos);
    p->enc_query.return_state = PATH_BND_SF_REINJECT_ENC;
    step_enc_query_emit(p);
    return RES_OK;
  }

  /* Reinjection resolved — proceed to probability dispatch */
  p->phase = PATH_BND_SF_PROB_DISPATCH;
  p->needs_ray = 0;
  return RES_OK;

error:
  p->phase = PATH_DONE;
  p->active = 0;
  p->done_reason = -1;
  return res;
}

/* --- PATH_BND_SF_REINJECT_ENC: ENC verification result ------------------- */
LOCAL_SYM res_T
step_bnd_sf_reinject_enc_result(struct path_state* p, struct sdis_scene* scn)
{
  unsigned enc_id;
  res_T res = RES_OK;

  ASSERT(p && scn);
  enc_id = p->enc_query.resolved_enc_id;

  if(enc_id != p->locals.bnd_sf.solid_enc_id) {
    /* Reinjection endpoint not in solid enclosure — retry */
    p->locals.bnd_sf.retry_count++;
    if(p->locals.bnd_sf.retry_count >= 10) {
      log_warn(scn->dev,
        "wavefront M5: SF reinjection ENC failed at (%g, %g, %g)\n",
        SPLIT3(p->rwalk.vtx.P));
      p->phase = PATH_DONE;
      p->active = 0;
      p->done_reason = -1;
      return RES_BAD_OP_IRRECOVERABLE;
    }
    d3_set(p->rwalk.vtx.P, p->locals.bnd_sf.rwalk_pos_backup);

    {
      float dir_samp[3], dir_refl[3];
      wf_sample_reinjection_dir_3d(&p->rwalk, p->rng, dir_samp);
      reflect_3d(dir_refl, dir_samp, p->rwalk.hit_3d.normal);
      if(p->locals.bnd_sf.solid_side == SDIS_BACK) {
        f3_minus(dir_samp, dir_samp);
        f3_minus(dir_refl, dir_refl);
      }
      f3_set(p->locals.bnd_sf.reinject_dir[0], dir_samp);
      f3_set(p->locals.bnd_sf.reinject_dir[1], dir_refl);
    }
    p->locals.bnd_sf.enc0_id = ENCLOSURE_ID_NULL;
    p->locals.bnd_sf.enc1_id = ENCLOSURE_ID_NULL;
    p->locals.bnd_sf.need_enc = 0;

    setup_sf_reinject_rays(p);
    return RES_OK;
  }

  /* ENC verified — proceed to probability dispatch */
  p->phase = PATH_BND_SF_PROB_DISPATCH;
  p->needs_ray = 0;
  return RES_OK;
}

/* --- PATH_BND_SF_PROB_DISPATCH: solid_reinjection + prob dispatch --------- */
LOCAL_SYM res_T
step_bnd_sf_prob_dispatch(struct path_state* p, struct sdis_scene* scn)
{
  struct reinjection_step reinject_step = REINJECTION_STEP_NULL;
  struct solid_reinjection_args solid_reinject_args = SOLID_REINJECTION_ARGS_NULL;
  struct handle_net_flux_args net_flux_args = HANDLE_NET_FLUX_ARGS_NULL;
  struct sdis_interface* interf = NULL;
  struct sdis_interface_fragment frag = SDIS_INTERFACE_FRAGMENT_NULL;
  double r;
  res_T res = RES_OK;

  ASSERT(p && scn);

  /* === First entry: compute probabilities === */
  /* We detect first entry by checking if h_hat is still 0 (not yet computed) */
  if(p->locals.bnd_sf.h_hat == 0) {
    double delta, delta_m;

    /* delta from reinjection distance (NOT solid_reinjection yet) */
    delta = p->locals.bnd_sf.chosen_dst / sqrt(3.0); /* DIM=3 */
    delta_m = delta * scn->fp_to_meter;
    p->locals.bnd_sf.delta_m = delta_m;

    /* Compute transfer coefficients */
    interf = scene_get_interface(scn, p->rwalk.hit_3d.prim.prim_id);

    /* Setup original fragment (from boundary hit) */
    setup_interface_fragment_3d(&frag, &p->rwalk.vtx,
                                &p->rwalk.hit_3d, p->rwalk.hit_side);

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

    /* handle_net_flux (pure compute) */
    net_flux_args.interf = interf;
    net_flux_args.frag = &frag;
    net_flux_args.green_path = p->ctx.green_path;
    net_flux_args.picard_order = get_picard_order(&p->ctx);
    net_flux_args.h_cond = p->locals.bnd_sf.h_cond;
    net_flux_args.h_conv = p->locals.bnd_sf.h_conv;
    net_flux_args.h_radi = p->locals.bnd_sf.h_radi_hat;
    res = handle_net_flux_3d(scn, &net_flux_args, &p->T);
    if(res != RES_OK) goto error;

    /* Save heat vertex for null-collision restart (before ext flux) */
    if(p->ctx.heat_path) {
      p->locals.bnd_sf.hvtx_saved =
          *heat_path_get_last_vertex(p->ctx.heat_path);
    }

    /* M7: route to external net flux sub-state machine instead of sync call.
     * When EXT completes, it returns to PATH_BND_SF_PROB_DISPATCH where
     * h_hat != 0 → goes directly to null-collision dispatch. */
    p->ext_flux.return_state = PATH_BND_SF_PROB_DISPATCH;
    p->phase = PATH_BND_EXT_CHECK;
    p->needs_ray = 0;
    return step_bnd_ext_check(p, scn);
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
    /* Perform solid_reinjection again (time_rewind + move to new position) */
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

    /* Route based on T.func set by solid_reinjection */
    if(p->T.func == conductive_path_3d) {
      p->ds_initialized = 0; /* reset for fresh conductive entry */
      p->phase = PATH_COUPLED_CONDUCTIVE;
    } else if(p->T.func == boundary_path_3d) {
      p->phase = PATH_COUPLED_BOUNDARY;
    } else {
      FATAL("wavefront M5: unexpected T.func after solid_reinjection\n");
    }
    p->needs_ray = 0;
    goto exit;
  }

  /* --- Radiative branch: start null-collision sub-path --- */
  {
    float N[3] = {0, 0, 0};
    float dir[3] = {0, 0, 0};

    if(p->ctx.heat_path) {
      p->locals.bnd_sf.ihvtx_radi_begin =
          heat_path_get_vertices_count(p->ctx.heat_path) - 1;
    }

    /* Save snapshot of rwalk/T for restore on accept or reject */
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

    /* Set up radiative direction and emit trace */
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
      p->phase = PATH_BND_SF_NULLCOLL_RAD_TRACE;
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

/* --- PATH_BND_SF_NULLCOLL_RAD_TRACE: process radiative sub-path hit ------ */
LOCAL_SYM res_T
step_bnd_sf_nullcoll_rad_trace(
  struct path_state* p,
  struct sdis_scene* scn,
  const struct s3d_hit* trace_hit)
{
  res_T res = RES_OK;

  ASSERT(p && scn && trace_hit);

  /* --- Miss → radiative environment temperature → go to decide --- */
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
        "wavefront M5: radiative sub-path reached unknown env from "
        "(%g, %g, %g) along (%g, %g, %g)\n",
        SPLIT3(p->rwalk.vtx.P), SPLIT3(dir_d));
      res = RES_BAD_OP;
      goto error;
    }

    /* Set T for the sub-path (will be used in decide step) */
    p->T.value += trad;
    p->T.done = 1;

    /* Store radiative direction for Tref retrieval */
    d3_set_f3(p->rwalk.dir, p->locals.bnd_sf.rad_sub_direction);

    p->phase = PATH_BND_SF_NULLCOLL_DECIDE;
    p->needs_ray = 0;
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

    /* Compute new position */
    d3_set_f3(dir_d, p->locals.bnd_sf.rad_sub_direction);
    d3_normalize(dir_d, dir_d);
    d3_set(pos_d, p->rwalk.vtx.P);
    {
      double vec[3];
      d3_add(pos_d, pos_d, d3_muld(vec, dir_d, trace_hit->distance));
    }
    d3_set(p->rwalk.vtx.P, pos_d);

    /* Normal */
    d3_set_f3(N, trace_hit->normal);
    d3_normalize(N, N);

    /* Hit side */
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

    /* Check interface validity */
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

        /* Re-trace from adjusted position */
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
          p->phase = PATH_BND_SF_NULLCOLL_RAD_TRACE;
        }
        goto exit;
      }
      goto error;
    }
    p->locals.bnd_sf.rad_sub_retry_count = 0;

    /* Verify path is in fluid */
    chk_mdm = p->rwalk.hit_side == SDIS_FRONT
            ? interf->medium_front : interf->medium_back;
    if(sdis_medium_get_type(chk_mdm) == SDIS_SOLID) {
      log_err(scn->dev,
        "wavefront M5: radiative sub-path in solid at (%g, %g, %g)\n",
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

    /* Absorption test: absorbed → go to decide with boundary Tref */
    if(ssp_rng_canonical(p->rng) < brdf.emissivity) {
      /* Absorbed at this boundary → path ended */
      p->rwalk.enc_id = ENCLOSURE_ID_NULL;
      p->T.func = boundary_path_3d;

      /* Store direction for Tref retrieval */
      d3_set_f3(p->rwalk.dir, p->locals.bnd_sf.rad_sub_direction);

      p->phase = PATH_BND_SF_NULLCOLL_DECIDE;
      p->needs_ray = 0;
      goto exit;
    }

    /* BRDF reflection: sample new direction */
    p->locals.bnd_sf.rad_sub_bounce_count++;
    d3_minus(wi, dir_d);
    switch(p->rwalk.hit_side) {
    case SDIS_FRONT: break;
    case SDIS_BACK: d3_minus(N, N); break;
    default: FATAL("Unreachable\n"); break;
    }
    brdf_sample(&brdf, p->rng, wi, N, &bounce);
    f3_set_d3(p->locals.bnd_sf.rad_sub_direction, bounce.dir);

    /* Set up next radiative trace */
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
      p->phase = PATH_BND_SF_NULLCOLL_RAD_TRACE;
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

/* --- PATH_BND_SF_NULLCOLL_DECIDE: accept/reject radiative sub-path ------- */
LOCAL_SYM res_T
step_bnd_sf_nullcoll_decide(struct path_state* p, struct sdis_scene* scn)
{
  double Tref_s = 0;
  double h_radi, p_radi;
  double r;
  res_T res = RES_OK;

  ASSERT(p && scn);

  /* Retrieve Tref_s at the end of the candidate radiative sub-path.
   * This mirrors rwalk_get_Tref from the CPU picard1 code. */
  if(p->T.done) {
    /* Miss → radiative environment reference temperature */
    struct sdis_radiative_ray ray = SDIS_RADIATIVE_RAY_NULL;
    d3_set(ray.dir, p->rwalk.dir);
    d3_normalize(ray.dir, ray.dir);
    ray.time = p->rwalk.vtx.time;
    Tref_s = radiative_env_get_reference_temperature(scn->radenv, &ray);
  } else {
    /* Absorbed at boundary → get reference temperature from interface */
    struct sdis_interface_fragment frag = SDIS_INTERFACE_FRAGMENT_NULL;
    struct sdis_interface* interf = NULL;

    ASSERT(!S3D_HIT_NONE(&p->rwalk.hit_3d));
    interf = scene_get_interface(scn, p->rwalk.hit_3d.prim.prim_id);

    setup_interface_fragment_3d(&frag, &p->rwalk.vtx,
                                &p->rwalk.hit_3d, p->rwalk.hit_side);
    Tref_s = interface_side_get_reference_temperature(interf, &frag);
  }

  res = check_Tref_3d(scn, p->rwalk.vtx.P, Tref_s,
                      "step_bnd_sf_nullcoll_decide");
  if(res != RES_OK) goto error;

  /* Compute actual h_radi using both Tref and Tref_s */
  {
    double T0 = p->locals.bnd_sf.Tref;
    double T1 = Tref_s;
    h_radi = BOLTZMANN_CONSTANT * p->locals.bnd_sf.epsilon
           * (T0*T0*T0 + T0*T0*T1 + T0*T1*T1 + T1*T1*T1);
  }
  p_radi = h_radi / p->locals.bnd_sf.h_hat;

  r = p->locals.bnd_sf.r;

  if(r < p->locals.bnd_sf.p_conv + p->locals.bnd_sf.p_cond + p_radi) {
    /* === ACCEPT: use the radiative sub-path result === */
    /* rwalk/T already contain the sub-path result (no need to restore) */
    p->phase = PATH_BND_POST_ROBIN_CHECK;
    p->needs_ray = 0;
    goto exit;
  }

  /* === REJECT: null-collision, discard sub-path and loop back === */
  {
    /* Restore rwalk/T from snapshot */
    p->rwalk = p->locals.bnd_sf.rwalk_snapshot;
    p->T = p->locals.bnd_sf.T_snapshot;

    if(p->ctx.green_path) {
      green_path_reset_limit(p->ctx.green_path);
    }

    if(p->ctx.heat_path) {
      size_t ihvtx_radi_end =
          heat_path_get_vertices_count(p->ctx.heat_path);
      heat_path_increment_sub_path_branch_id(
        p->ctx.heat_path,
        p->locals.bnd_sf.ihvtx_radi_begin,
        ihvtx_radi_end);
      res = heat_path_restart(p->ctx.heat_path,
                              &p->locals.bnd_sf.hvtx_saved);
      if(res != RES_OK) goto error;
    }

    /* Loop back to prob_dispatch for next null-collision iteration.
     * Note: h_hat is already set, so prob_dispatch will skip init. */
    p->phase = PATH_BND_SF_PROB_DISPATCH;
    p->needs_ray = 0;
    goto exit;
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
LOCAL_SYM res_T
step_convective(struct path_state* p, struct sdis_scene* scn)
{
  res_T res = RES_OK;
  ASSERT(p && scn);

  /* M6: redirect to fine-grained convective state machine */
  p->phase = PATH_CNV_INIT;
  return step_cnv_init(p, scn);
}

/*******************************************************************************
 * B-4 M7: External net flux batch state machine
 *
 * Replaces the synchronous handle_external_net_flux_3d() call with a sub-state
 * machine that emits shadow rays and diffuse bounce rays through the wavefront
 * batch trace pipeline.
 *
 * State flow:
 *   EXT_CHECK → (shadow)   EXT_DIRECT_TRACE → EXT_DIRECT_RESULT
 *             → (diffuse)  EXT_DIFFUSE_TRACE → EXT_DIFFUSE_RESULT
 *                          → (reflect) EXT_DIFFUSE_SHADOW_TRACE
 *                                      → EXT_DIFFUSE_SHADOW_RESULT → loop
 *                          → (miss/absorb) EXT_FINALIZE
 *             → (no flux)  return_state (bypass)
 *
 * See CPU reference: sdis_heat_path_boundary_Xd_handle_external_net_flux.h
 ******************************************************************************/

/* --- PATH_BND_EXT_CHECK: initialise ext flux, decide shadow ray ---------- */
LOCAL_SYM res_T
step_bnd_ext_check(struct path_state* p, struct sdis_scene* scn)
{
  struct sdis_interface* interf = NULL;
  struct sdis_interface_fragment frag = SDIS_INTERFACE_FRAGMENT_NULL;
  unsigned enc_ids[2] = {ENCLOSURE_ID_NULL, ENCLOSURE_ID_NULL};
  int handle_flux = 0;
  unsigned src_id = 0;
  double cos_theta = 0;
  res_T res = RES_OK;

  ASSERT(p && scn);

  /* Zero accumulated flux */
  p->ext_flux.flux_direct = 0;
  p->ext_flux.flux_diffuse_reflected = 0;
  p->ext_flux.flux_scattered = 0;
  p->ext_flux.scattered_dir[0] = 0;
  p->ext_flux.scattered_dir[1] = 0;
  p->ext_flux.scattered_dir[2] = 0;
  p->ext_flux.nbounces = 0;

  /* Get interface from the boundary hit */
  interf = scene_get_interface(scn, p->rwalk.hit_3d.prim.prim_id);

  /* Setup fluid-side fragment */
  setup_interface_fragment_3d(&frag, &p->rwalk.vtx,
                              &p->rwalk.hit_3d, p->rwalk.hit_side);
  if(sdis_medium_get_type(interf->medium_front) == SDIS_FLUID) {
    frag.side = SDIS_FRONT;
  } else {
    ASSERT(sdis_medium_get_type(interf->medium_back) == SDIS_FLUID);
    frag.side = SDIS_BACK;
  }

  /* Retrieve enclosure ids */
  scene_get_enclosure_ids(scn, p->rwalk.hit_3d.prim.prim_id, enc_ids);
  p->ext_flux.enc_id_fluid = enc_ids[frag.side];

  /* Check if external flux handling is needed */
  handle_flux = interface_side_is_external_flux_handled(interf, &frag);
  handle_flux = handle_flux && (scn->source != NULL);
  if(!handle_flux) {
    /* No external flux → skip directly to caller */
    p->phase = p->ext_flux.return_state;
    p->needs_ray = 0;
    goto exit;
  }

  /* Check emissivity */
  src_id = sdis_source_get_id(scn->source);
  p->ext_flux.emissivity =
      interface_side_get_emissivity(interf, src_id, &frag);
  res = interface_side_check_emissivity(scn->dev, p->ext_flux.emissivity,
                                        frag.P, frag.time);
  if(res != RES_OK) goto error;

  if(p->ext_flux.emissivity == 0) {
    p->phase = p->ext_flux.return_state;
    p->needs_ray = 0;
    goto exit;
  }

  /* Get source properties */
  res = source_get_props(scn->source, frag.time, &p->ext_flux.src_props);
  if(res != RES_OK) goto error;

  /* Sample a direction toward the source */
  res = source_sample(scn->source, &p->ext_flux.src_props, p->rng,
                      frag.P, &p->ext_flux.src_sample);
  if(res != RES_OK) goto error;

  /* Save fragment info for later */
  p->ext_flux.frag_time = frag.time;
  d3_set(p->ext_flux.frag_P, frag.P);

  /* Compute outward normal (toward fluid) */
  d3_set(p->ext_flux.N, frag.Ng);
  if(frag.side == SDIS_BACK) {
    d3_minus(p->ext_flux.N, p->ext_flux.N);
  }

  /* Compute sum_h for denominator */
  p->ext_flux.sum_h = p->locals.bnd_sf.h_cond
                    + p->locals.bnd_sf.h_conv
                    + p->locals.bnd_sf.h_radi_hat;

  /* Save green_path handle */
  p->ext_flux.green_path = p->ctx.green_path;

  /* cos_theta for direct contribution check */
  cos_theta = d3_dot(p->ext_flux.N, p->ext_flux.src_sample.dir);
  p->ext_flux.cos_theta = cos_theta;

  if(cos_theta > 0) {
    /* Source is above the surface → emit shadow ray to check occlusion */
    float pos_f[3];
    f3_set_d3(pos_f, p->ext_flux.frag_P);

    p->ext_flux.pos[0] = pos_f[0];
    p->ext_flux.pos[1] = pos_f[1];
    p->ext_flux.pos[2] = pos_f[2];

    p->ray_req.origin[0] = pos_f[0];
    p->ray_req.origin[1] = pos_f[1];
    p->ray_req.origin[2] = pos_f[2];
    p->ray_req.direction[0] = (float)p->ext_flux.src_sample.dir[0];
    p->ray_req.direction[1] = (float)p->ext_flux.src_sample.dir[1];
    p->ray_req.direction[2] = (float)p->ext_flux.src_sample.dir[2];
    p->ray_req.range[0] = 0.0f;
    p->ray_req.range[1] = (float)p->ext_flux.src_sample.dst;
    p->ray_req.ray_count = 1;

    /* Filter: skip self-intersection with current hit */
    p->filter_data_storage = HIT_FILTER_DATA_NULL;
    p->filter_data_storage.hit_3d = p->rwalk.hit_3d;
    p->filter_data_storage.epsilon = 1.e-6;
    p->filter_data_storage.scn = scn;
    p->filter_data_storage.enc_id = p->ext_flux.enc_id_fluid;

    p->ray_bucket = RAY_BUCKET_SHADOW;
    p->ray_count_ext = 1;
    p->needs_ray = 1;
    p->phase = PATH_BND_EXT_DIRECT_TRACE;
  } else {
    /* Source behind surface → skip direct contribution, go to diffuse */
    p->ext_flux.flux_direct = 0;

    /* Set up initial diffuse bounce ray */
    {
      float pos_f[3], dir_f[3];
      double dir_d[3];

      f3_set_d3(pos_f, p->ext_flux.frag_P);

      /* Cosine-weighted hemisphere sample for diffuse */
      ssp_ran_hemisphere_cos(p->rng, p->ext_flux.N, dir_d, NULL);
      dir_f[0] = (float)dir_d[0];
      dir_f[1] = (float)dir_d[1];
      dir_f[2] = (float)dir_d[2];

      p->ext_flux.pos[0] = pos_f[0];
      p->ext_flux.pos[1] = pos_f[1];
      p->ext_flux.pos[2] = pos_f[2];
      p->ext_flux.dir[0] = dir_f[0];
      p->ext_flux.dir[1] = dir_f[1];
      p->ext_flux.dir[2] = dir_f[2];
      p->ext_flux.hit = p->rwalk.hit_3d;

      p->ray_req.origin[0] = pos_f[0];
      p->ray_req.origin[1] = pos_f[1];
      p->ray_req.origin[2] = pos_f[2];
      p->ray_req.direction[0] = dir_f[0];
      p->ray_req.direction[1] = dir_f[1];
      p->ray_req.direction[2] = dir_f[2];
      p->ray_req.range[0] = 0.0f;
      p->ray_req.range[1] = FLT_MAX;
      p->ray_req.ray_count = 1;

      p->filter_data_storage = HIT_FILTER_DATA_NULL;
      p->filter_data_storage.hit_3d = p->rwalk.hit_3d;
      p->filter_data_storage.epsilon = 1.e-6;
      p->filter_data_storage.scn = scn;
      p->filter_data_storage.enc_id = p->ext_flux.enc_id_fluid;

      p->ray_bucket = RAY_BUCKET_RADIATIVE;
      p->ray_count_ext = 1;
      p->needs_ray = 1;
      p->phase = PATH_BND_EXT_DIFFUSE_TRACE;
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

/* --- PATH_BND_EXT_DIRECT_RESULT: process shadow ray result --------------- */
LOCAL_SYM res_T
step_bnd_ext_direct_result(struct path_state* p, struct sdis_scene* scn,
                           const struct s3d_hit* hit)
{
  res_T res = RES_OK;
  double dir_d[3];
  float  pos_f[3], dir_f[3];

  ASSERT(p && scn && hit);

  /* Shadow ray: if we hit something, source is occluded → direct = 0 */
  if(!S3D_HIT_NONE(hit)) {
    p->ext_flux.flux_direct = 0;
  } else {
    /* Source visible → compute direct contribution */
    double Ld = p->ext_flux.src_sample.radiance_term; /* [W/m^2/sr] */
    p->ext_flux.flux_direct =
        p->ext_flux.cos_theta * Ld / p->ext_flux.src_sample.pdf; /* [W/m^2] */
  }

  /* Now set up the first diffuse bounce ray (same as CPU's
   * compute_incident_diffuse_flux entry) */
  f3_set_d3(pos_f, p->ext_flux.frag_P);

  /* Cosine-weighted hemisphere sample for diffuse bounce */
  ssp_ran_hemisphere_cos(p->rng, p->ext_flux.N, dir_d, NULL);
  dir_f[0] = (float)dir_d[0];
  dir_f[1] = (float)dir_d[1];
  dir_f[2] = (float)dir_d[2];

  p->ext_flux.pos[0] = pos_f[0];
  p->ext_flux.pos[1] = pos_f[1];
  p->ext_flux.pos[2] = pos_f[2];
  p->ext_flux.dir[0] = dir_f[0];
  p->ext_flux.dir[1] = dir_f[1];
  p->ext_flux.dir[2] = dir_f[2];
  p->ext_flux.hit = p->rwalk.hit_3d;

  p->ray_req.origin[0] = pos_f[0];
  p->ray_req.origin[1] = pos_f[1];
  p->ray_req.origin[2] = pos_f[2];
  p->ray_req.direction[0] = dir_f[0];
  p->ray_req.direction[1] = dir_f[1];
  p->ray_req.direction[2] = dir_f[2];
  p->ray_req.range[0] = 0.0f;
  p->ray_req.range[1] = FLT_MAX;
  p->ray_req.ray_count = 1;

  p->filter_data_storage = HIT_FILTER_DATA_NULL;
  p->filter_data_storage.hit_3d = p->rwalk.hit_3d;
  p->filter_data_storage.epsilon = 1.e-6;
  p->filter_data_storage.scn = scn;
  p->filter_data_storage.enc_id = p->ext_flux.enc_id_fluid;

  p->ray_bucket = RAY_BUCKET_RADIATIVE;
  p->ray_count_ext = 1;
  p->needs_ray = 1;
  p->phase = PATH_BND_EXT_DIFFUSE_TRACE;

  return res;
}

/* --- PATH_BND_EXT_DIFFUSE_RESULT: process diffuse bounce ray result ------ */
LOCAL_SYM res_T
step_bnd_ext_diffuse_result(struct path_state* p, struct sdis_scene* scn,
                            const struct s3d_hit* hit)
{
  res_T res = RES_OK;

  ASSERT(p && scn && hit);

  /* --- Miss: ray escaped to environment → scattered flux = PI --- */
  if(S3D_HIT_NONE(hit)) {
    p->ext_flux.flux_scattered = PI;
    p->ext_flux.scattered_dir[0] = p->ext_flux.dir[0];
    p->ext_flux.scattered_dir[1] = p->ext_flux.dir[1];
    p->ext_flux.scattered_dir[2] = p->ext_flux.dir[2];

    p->phase = PATH_BND_EXT_FINALIZE;
    p->needs_ray = 0;
    goto exit;
  }

  /* --- Hit: BRDF decision at bounce position --- */
  {
    struct sdis_interface_fragment frag = SDIS_INTERFACE_FRAGMENT_NULL;
    struct sdis_interface* interf = NULL;
    struct brdf brdf = BRDF_NULL;
    struct brdf_sample bounce = BRDF_SAMPLE_NULL;
    struct brdf_setup_args brdf_args = BRDF_SETUP_ARGS_NULL;
    double dir_d[3], pos_d[3], N[3], wi[3];

    /* Update stored hit */
    p->ext_flux.hit = *hit;

    /* Compute new position from ray origin + t * direction */
    d3_set_f3(dir_d, p->ext_flux.dir);
    d3_normalize(dir_d, dir_d);
    d3_set_f3(pos_d, p->ext_flux.pos);
    {
      double vec[3];
      d3_add(pos_d, pos_d, d3_muld(vec, dir_d, hit->distance));
    }

    /* Update position in ext_flux for next bounce */
    p->ext_flux.pos[0] = (float)pos_d[0];
    p->ext_flux.pos[1] = (float)pos_d[1];
    p->ext_flux.pos[2] = (float)pos_d[2];

    /* Normal at hit */
    d3_set_f3(N, hit->normal);
    d3_normalize(N, N);

    /* Determine hit side */
    {
      enum sdis_side side = d3_dot(dir_d, N) < 0 ? SDIS_FRONT : SDIS_BACK;

      /* Get interface at hit */
      interf = scene_get_interface(scn, hit->prim.prim_id);

      /* Set up fragment */
      {
        struct sdis_rwalk_vertex vtx = SDIS_RWALK_VERTEX_NULL;
        d3_set(vtx.P, pos_d);
        vtx.time = p->ext_flux.frag_time;
        setup_interface_fragment_3d(&frag, &vtx, hit, side);
      }

      /* BRDF at intersection */
      brdf_args.interf = interf;
      brdf_args.frag = &frag;
      brdf_args.source_id = sdis_source_get_id(scn->source);
      res = brdf_setup(scn->dev, &brdf_args, &brdf);
      if(res != RES_OK) goto error;

      /* Absorption test */
      if(ssp_rng_canonical(p->rng) < brdf.emissivity) {
        /* Absorbed → diffuse bounce terminates with no further contribution */
        p->phase = PATH_BND_EXT_FINALIZE;
        p->needs_ray = 0;
        goto exit;
      }

      /* Reflection: sample new direction */
      d3_minus(wi, dir_d);
      switch(side) {
      case SDIS_FRONT: break;
      case SDIS_BACK: d3_minus(N, N); break;
      default: FATAL("Unreachable\n"); break;
      }
      brdf_sample(&brdf, p->rng, wi, N, &bounce);

      /* Calculate the direct contribution at this bounce position.
       * This depends on whether the bounce is specular or diffuse. */
      if(bounce.cpnt == BRDF_SPECULAR) {
        /* Specular bounce: trace to source along specular direction */
        struct source_sample samp = SOURCE_SAMPLE_NULL;
        res = source_trace_to(scn->source, &p->ext_flux.src_props,
                              pos_d, bounce.dir, &samp);
        if(res != RES_OK) goto error;

        if(!SOURCE_SAMPLE_NONE(&samp)) {
          /* Emit shadow ray along specular direction to check occlusion */
          p->ext_flux.dir[0] = (float)bounce.dir[0];
          p->ext_flux.dir[1] = (float)bounce.dir[1];
          p->ext_flux.dir[2] = (float)bounce.dir[2];

          /* Store the source sample radiance_term for shadow result.
           * Mark cos_theta <= 0 so that diffuse_shadow_result uses L = Ld
           * (specular contribution) rather than L = Ld*cos/(PI*pdf). */
          p->ext_flux.src_sample = samp;
          p->ext_flux.cos_theta = -1;

          p->ray_req.origin[0] = p->ext_flux.pos[0];
          p->ray_req.origin[1] = p->ext_flux.pos[1];
          p->ray_req.origin[2] = p->ext_flux.pos[2];
          p->ray_req.direction[0] = (float)samp.dir[0];
          p->ray_req.direction[1] = (float)samp.dir[1];
          p->ray_req.direction[2] = (float)samp.dir[2];
          p->ray_req.range[0] = 0.0f;
          p->ray_req.range[1] = (float)samp.dst;
          p->ray_req.ray_count = 1;

          p->filter_data_storage = HIT_FILTER_DATA_NULL;
          p->filter_data_storage.hit_3d = *hit;
          p->filter_data_storage.epsilon = 1.e-6;
          p->filter_data_storage.scn = scn;
          p->filter_data_storage.enc_id = p->ext_flux.enc_id_fluid;

          p->ray_bucket = RAY_BUCKET_SHADOW;
          p->ray_count_ext = 1;
          p->needs_ray = 1;
          p->phase = PATH_BND_EXT_DIFFUSE_SHADOW_TRACE;

          /* Save the specular bounce direction as the next diffuse dir */
          /* (will be used after shadow result to continue bouncing) */
          p->ext_flux.dir[0] = (float)bounce.dir[0];
          p->ext_flux.dir[1] = (float)bounce.dir[1];
          p->ext_flux.dir[2] = (float)bounce.dir[2];
          goto exit;
        }
        /* Source not reachable from specular bounce → no shadow ray needed,
         * just continue bouncing in the specular direction */
        p->ext_flux.dir[0] = (float)bounce.dir[0];
        p->ext_flux.dir[1] = (float)bounce.dir[1];
        p->ext_flux.dir[2] = (float)bounce.dir[2];

      } else {
        /* Diffuse bounce: sample a direction toward the source */
        struct source_sample samp = SOURCE_SAMPLE_NULL;
        double cos_theta_bounce;

        ASSERT(bounce.cpnt == BRDF_DIFFUSE);

        res = source_sample(scn->source, &p->ext_flux.src_props, p->rng,
                            pos_d, &samp);
        if(res != RES_OK) goto error;

        cos_theta_bounce = d3_dot(samp.dir, N);
        if(cos_theta_bounce > 0) {
          /* Source above this surface → emit shadow ray */
          p->ext_flux.src_sample = samp;
          p->ext_flux.cos_theta = cos_theta_bounce;

          p->ray_req.origin[0] = p->ext_flux.pos[0];
          p->ray_req.origin[1] = p->ext_flux.pos[1];
          p->ray_req.origin[2] = p->ext_flux.pos[2];
          p->ray_req.direction[0] = (float)samp.dir[0];
          p->ray_req.direction[1] = (float)samp.dir[1];
          p->ray_req.direction[2] = (float)samp.dir[2];
          p->ray_req.range[0] = 0.0f;
          p->ray_req.range[1] = (float)samp.dst;
          p->ray_req.ray_count = 1;

          p->filter_data_storage = HIT_FILTER_DATA_NULL;
          p->filter_data_storage.hit_3d = *hit;
          p->filter_data_storage.epsilon = 1.e-6;
          p->filter_data_storage.scn = scn;
          p->filter_data_storage.enc_id = p->ext_flux.enc_id_fluid;

          p->ray_bucket = RAY_BUCKET_SHADOW;
          p->ray_count_ext = 1;
          p->needs_ray = 1;
          p->phase = PATH_BND_EXT_DIFFUSE_SHADOW_TRACE;

          /* Save bounce direction for next diffuse bounce */
          p->ext_flux.dir[0] = (float)bounce.dir[0];
          p->ext_flux.dir[1] = (float)bounce.dir[1];
          p->ext_flux.dir[2] = (float)bounce.dir[2];
          goto exit;
        }
        /* Source behind this surface → L = 0, no shadow ray needed */
        /* Just continue bouncing */
        p->ext_flux.dir[0] = (float)bounce.dir[0];
        p->ext_flux.dir[1] = (float)bounce.dir[1];
        p->ext_flux.dir[2] = (float)bounce.dir[2];
      }

      /* No shadow ray needed at this bounce → emit next diffuse bounce ray */
      p->ext_flux.nbounces++;

      p->ray_req.origin[0] = p->ext_flux.pos[0];
      p->ray_req.origin[1] = p->ext_flux.pos[1];
      p->ray_req.origin[2] = p->ext_flux.pos[2];
      p->ray_req.direction[0] = p->ext_flux.dir[0];
      p->ray_req.direction[1] = p->ext_flux.dir[1];
      p->ray_req.direction[2] = p->ext_flux.dir[2];
      p->ray_req.range[0] = 0.0f;
      p->ray_req.range[1] = FLT_MAX;
      p->ray_req.ray_count = 1;

      p->filter_data_storage = HIT_FILTER_DATA_NULL;
      p->filter_data_storage.hit_3d = *hit;
      p->filter_data_storage.epsilon = 1.e-6;
      p->filter_data_storage.scn = scn;
      p->filter_data_storage.enc_id = p->ext_flux.enc_id_fluid;

      p->ray_bucket = RAY_BUCKET_RADIATIVE;
      p->ray_count_ext = 1;
      p->needs_ray = 1;
      p->phase = PATH_BND_EXT_DIFFUSE_TRACE;
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

/* --- PATH_BND_EXT_DIFFUSE_SHADOW_RESULT: shadow ray at bounce position --- */
LOCAL_SYM res_T
step_bnd_ext_diffuse_shadow_result(struct path_state* p,
                                   struct sdis_scene* scn,
                                   const struct s3d_hit* hit)
{
  res_T res = RES_OK;

  ASSERT(p && scn && hit);

  /* Process shadow ray result at bounce position */
  if(S3D_HIT_NONE(hit)) {
    /* Not occluded → accumulate direct contribution at this bounce.
     * For specular bounces, L = Ld (radiance_term).
     * For diffuse bounces, L = Ld * cos_theta / (PI * pdf). */
    double Ld = p->ext_flux.src_sample.radiance_term;
    double L;

    if(p->ext_flux.cos_theta > 0
    && p->ext_flux.src_sample.pdf > 0) {
      /* Diffuse bounce: L = Ld * cos / (PI * pdf) */
      L = Ld * p->ext_flux.cos_theta
        / (PI * p->ext_flux.src_sample.pdf);
    } else {
      /* Specular bounce: L = Ld */
      L = Ld;
    }
    p->ext_flux.flux_diffuse_reflected += L; /* [W/m^2/sr] */
  }
  /* If occluded, no contribution */

  /* Continue with next diffuse bounce ray */
  p->ext_flux.nbounces++;

  p->ray_req.origin[0] = p->ext_flux.pos[0];
  p->ray_req.origin[1] = p->ext_flux.pos[1];
  p->ray_req.origin[2] = p->ext_flux.pos[2];
  p->ray_req.direction[0] = p->ext_flux.dir[0];
  p->ray_req.direction[1] = p->ext_flux.dir[1];
  p->ray_req.direction[2] = p->ext_flux.dir[2];
  p->ray_req.range[0] = 0.0f;
  p->ray_req.range[1] = FLT_MAX;
  p->ray_req.ray_count = 1;

  p->filter_data_storage = HIT_FILTER_DATA_NULL;
  p->filter_data_storage.hit_3d = p->ext_flux.hit;
  p->filter_data_storage.epsilon = 1.e-6;
  p->filter_data_storage.scn = scn;
  p->filter_data_storage.enc_id = p->ext_flux.enc_id_fluid;

  p->ray_bucket = RAY_BUCKET_RADIATIVE;
  p->ray_count_ext = 1;
  p->needs_ray = 1;
  p->phase = PATH_BND_EXT_DIFFUSE_TRACE;

  return res;
}

/* --- PATH_BND_EXT_FINALIZE: sum flux, apply to T, return to caller ------- */
LOCAL_SYM res_T
step_bnd_ext_finalize(struct path_state* p, struct sdis_scene* scn)
{
  struct sdis_green_external_flux_terms green =
      SDIS_GREEN_EXTERNAL_FLUX_TERMS_NULL;
  double incident_flux;
  double net_flux, net_flux_sc;
  res_T res = RES_OK;

  ASSERT(p && scn);

  /* CPU reference: handle_external_net_flux lines 340-385 */

  /* flux_diffuse_reflected is accumulated as [W/m^2/sr], multiply by PI to
   * get [W/m^2] (matches CPU: diffuse_flux->reflected *= PI) */
  p->ext_flux.flux_diffuse_reflected *= PI; /* [W/m^2] */

  /* incident_flux = direct + diffuse_reflected (excludes scattered) */
  incident_flux = p->ext_flux.flux_direct
                + p->ext_flux.flux_diffuse_reflected;

  /* net_flux (relative to source power) */
  net_flux = incident_flux * p->ext_flux.emissivity;

  /* net_flux_sc (relative to source diffuse radiance) */
  net_flux_sc = p->ext_flux.flux_scattered * p->ext_flux.emissivity;

  /* Green function terms */
  green.term_wrt_power = net_flux / p->ext_flux.sum_h;
  green.term_wrt_diffuse_radiance = net_flux_sc / p->ext_flux.sum_h;
  green.time = p->ext_flux.frag_time;
  green.dir[0] = p->ext_flux.scattered_dir[0];
  green.dir[1] = p->ext_flux.scattered_dir[1];
  green.dir[2] = p->ext_flux.scattered_dir[2];

  /* Apply to temperature */
  p->T.value += green.term_wrt_power * p->ext_flux.src_props.power;
  if(green.term_wrt_diffuse_radiance) {
    p->T.value +=
        green.term_wrt_diffuse_radiance
      * source_get_diffuse_radiance(scn->source, green.time, green.dir);
  }

  /* Register in green path if available */
  if(p->ext_flux.green_path) {
    res = green_path_add_external_flux_terms(p->ext_flux.green_path, &green);
    if(res != RES_OK) goto error;
  }

  /* Return to caller */
  p->phase = p->ext_flux.return_state;
  p->needs_ray = 0;

exit:
  return res;
error:
  p->phase = PATH_DONE;
  p->active = 0;
  p->done_reason = -1;
  goto exit;
}

/*******************************************************************************
 * B-4 M6: Convective path fine-grained state machine
 ******************************************************************************/

/* --- PATH_CNV_INIT: check known fluid temp + decide startup ray or loop -- */
LOCAL_SYM res_T
step_cnv_init(struct path_state* p, struct sdis_scene* scn)
{
  const struct enclosure* enc = NULL;
  struct sdis_medium* mdm = NULL;
  double temperature;
  struct fluid_props props_ref = FLUID_PROPS_NULL;
  res_T res = RES_OK;

  ASSERT(p && scn);

  /* Get the enclosure and its medium */
  enc = scene_get_enclosure(scn, p->rwalk.enc_id);
  if(enc->medium_id == MEDIUM_ID_MULTI) {
    log_err(scn->dev,
      "wavefront M6: enclosure with multiple media at (%g, %g, %g)\n",
      SPLIT3(p->rwalk.vtx.P));
    res = RES_BAD_ARG;
    goto error;
  }
  res = scene_get_enclosure_medium(scn, enc, &mdm);
  if(res != RES_OK) goto error;
  if(sdis_medium_get_type(mdm) != SDIS_FLUID) {
    log_err(scn->dev,
      "wavefront M6: convective path in non-fluid medium at (%g, %g, %g)\n",
      SPLIT3(p->rwalk.vtx.P));
    res = RES_BAD_OP;
    goto error;
  }

  /* Check known fluid temperature */
  temperature = fluid_get_temperature(mdm, &p->rwalk.vtx);
  if(SDIS_TEMPERATURE_IS_KNOWN(temperature)) {
    p->T.value += temperature;
    p->T.done = 1;
    if(p->ctx.green_path) {
      res = green_path_set_limit_vertex(
        p->ctx.green_path, mdm, &p->rwalk.vtx, p->rwalk.elapsed_time);
      if(res != RES_OK) goto error;
    }
    if(p->ctx.heat_path) {
      heat_path_get_last_vertex(p->ctx.heat_path)->weight = p->T.value;
    }
    p->phase = PATH_DONE;
    p->active = 0;
    p->done_reason = 2; /* temperature known */
    goto exit;
  }

  /* Check if path starts from fluid interior (HIT_NONE) */
  if(S3D_HIT_NONE(&p->rwalk.hit_3d)) {
    /* Need startup ray along +Z to find initial hit */
    setup_convective_startup_ray(p);
    p->phase = PATH_CNV_STARTUP_TRACE;
    goto exit;
  }

  /* Already have a hit — get fluid properties for the loop */
  res = fluid_get_properties(mdm, &p->rwalk.vtx, &props_ref);
  if(res != RES_OK) goto error;

  /* Store loop parameters in locals.cnv */
  p->locals.cnv.enc_id = p->rwalk.enc_id;
  p->locals.cnv.hc_upper_bound = enc->hc_upper_bound;
  p->locals.cnv.rho_cp = props_ref.rho * props_ref.cp;
  p->locals.cnv.S_over_V = enc->S_over_V;

  /* Handle edge case: hc_upper_bound == 0 → initial condition */
  if(enc->hc_upper_bound == 0) {
    p->rwalk.vtx.time = props_ref.t0;
    temperature = fluid_get_temperature(mdm, &p->rwalk.vtx);
    if(SDIS_TEMPERATURE_IS_KNOWN(temperature)) {
      p->T.value += temperature;
      p->T.done = 1;
      if(p->ctx.green_path) {
        res = green_path_set_limit_vertex(
          p->ctx.green_path, mdm, &p->rwalk.vtx, p->rwalk.elapsed_time);
        if(res != RES_OK) goto error;
      }
      if(p->ctx.heat_path) {
        heat_path_get_last_vertex(p->ctx.heat_path)->weight = p->T.value;
      }
      p->phase = PATH_DONE;
      p->active = 0;
      p->done_reason = 2;
    } else {
      log_err(scn->dev,
        "wavefront M6: undefined initial condition at (%g, %g, %g)\n",
        SPLIT3(p->rwalk.vtx.P));
      res = RES_BAD_OP;
      goto error;
    }
    goto exit;
  }

  /* Ready for sampling loop */
  p->phase = PATH_CNV_SAMPLE_LOOP;

exit:
  return res;
error:
  p->phase = PATH_DONE;
  p->active = 0;
  p->done_reason = -1;
  goto exit;
}

/* --- PATH_CNV_STARTUP_RESULT: process startup ray result ----------------- */
LOCAL_SYM res_T
step_cnv_startup_result(struct path_state* p, struct sdis_scene* scn,
                        const struct s3d_hit* hit)
{
  const struct enclosure* enc = NULL;
  struct sdis_medium* mdm = NULL;
  struct fluid_props props_ref = FLUID_PROPS_NULL;
  res_T res = RES_OK;

  ASSERT(p && scn && hit);

  if(S3D_HIT_NONE(hit)) {
    log_err(scn->dev,
      "wavefront M6: convective startup ray missed — position (%g, %g, %g) "
      "lies in surrounding fluid whose temperature must be known.\n",
      SPLIT3(p->rwalk.vtx.P));
    res = RES_BAD_OP;
    goto error;
  }

  /* Set hit and determine hit_side */
  p->rwalk.hit_3d = *hit;
  {
    float startup_dir[3] = {0.0f, 0.0f, 1.0f};
    p->rwalk.hit_side = f3_dot(hit->normal, startup_dir) < 0
      ? SDIS_FRONT : SDIS_BACK;
  }

  /* Get enclosure and fluid properties for the loop */
  enc = scene_get_enclosure(scn, p->rwalk.enc_id);
  res = scene_get_enclosure_medium(scn, enc, &mdm);
  if(res != RES_OK) goto error;

  res = fluid_get_properties(mdm, &p->rwalk.vtx, &props_ref);
  if(res != RES_OK) goto error;

  p->locals.cnv.enc_id = p->rwalk.enc_id;
  p->locals.cnv.hc_upper_bound = enc->hc_upper_bound;
  p->locals.cnv.rho_cp = props_ref.rho * props_ref.cp;
  p->locals.cnv.S_over_V = enc->S_over_V;

  /* Handle edge case: hc_upper_bound == 0 */
  if(enc->hc_upper_bound == 0) {
    double temperature;
    p->rwalk.vtx.time = props_ref.t0;
    temperature = fluid_get_temperature(mdm, &p->rwalk.vtx);
    if(SDIS_TEMPERATURE_IS_KNOWN(temperature)) {
      p->T.value += temperature;
      p->T.done = 1;
      p->phase = PATH_DONE;
      p->active = 0;
      p->done_reason = 2;
    } else {
      log_err(scn->dev,
        "wavefront M6: undefined initial condition at (%g, %g, %g)\n",
        SPLIT3(p->rwalk.vtx.P));
      res = RES_BAD_OP;
      goto error;
    }
    goto exit;
  }

  p->phase = PATH_CNV_SAMPLE_LOOP;

exit:
  return res;
error:
  p->phase = PATH_DONE;
  p->active = 0;
  p->done_reason = -1;
  goto exit;
}

/* --- PATH_CNV_SAMPLE_LOOP: null-collision sampling (pure compute) -------- */
LOCAL_SYM res_T
step_cnv_sample_loop(struct path_state* p, struct sdis_scene* scn)
{
  const struct enclosure* enc = NULL;
  struct sdis_medium* mdm = NULL;
  struct sdis_interface* interf = NULL;
  struct sdis_interface_fragment frag = SDIS_INTERFACE_FRAGMENT_NULL;
  struct s3d_primitive prim;
  struct s3d_attrib attr_P, attr_N;
  unsigned enc_ids[2] = {ENCLOSURE_ID_NULL, ENCLOSURE_ID_NULL};
  struct fluid_props props = FLUID_PROPS_NULL;
  double mu, hc, r;
  float st[2];
  res_T res = RES_OK;

  ASSERT(p && scn);

  enc = scene_get_enclosure(scn, p->locals.cnv.enc_id);
  res = scene_get_enclosure_medium(scn, enc, &mdm);
  if(res != RES_OK) goto error;

  /* Fetch fluid properties at current position */
  res = fluid_get_properties(mdm, &p->rwalk.vtx, &props);
  if(res != RES_OK) goto error;

  /* Time rewind */
  mu = p->locals.cnv.hc_upper_bound
     / (props.rho * props.cp) * p->locals.cnv.S_over_V;
  res = time_rewind(scn, mu, props.t0, p->rng, &p->rwalk, &p->ctx, &p->T);
  if(res != RES_OK) goto error;
  if(p->T.done) {
    p->phase = PATH_DONE;
    p->active = 0;
    p->done_reason = 4; /* time rewind */
    goto exit;
  }

  /* Uniformly sample the enclosure surface */
  s3d_scene_view_sample(
    enc->s3d_view,
    ssp_rng_canonical_float(p->rng),
    ssp_rng_canonical_float(p->rng),
    ssp_rng_canonical_float(p->rng),
    &prim, p->rwalk.hit_3d.uv);
  f2_set(st, p->rwalk.hit_3d.uv);

  /* Map from enclosure local to scene global prim_id */
  p->rwalk.hit_3d.prim.prim_id =
      enclosure_local2global_prim_id(enc, prim.prim_id);

  /* Get position and normal at sampled point */
  s3d_primitive_get_attrib(&p->rwalk.hit_3d.prim, S3D_POSITION, st, &attr_P);
  s3d_primitive_get_attrib(&p->rwalk.hit_3d.prim, S3D_GEOMETRY_NORMAL,
                           st, &attr_N);
  d3_set_f3(p->rwalk.vtx.P, attr_P.value);
  f3_set(p->rwalk.hit_3d.normal, attr_N.value);

  /* Determine interface side */
  scene_get_enclosure_ids(scn, p->rwalk.hit_3d.prim.prim_id, enc_ids);
  if(p->locals.cnv.enc_id == enc_ids[SDIS_BACK]) {
    p->rwalk.hit_side = SDIS_BACK;
  } else if(p->locals.cnv.enc_id == enc_ids[SDIS_FRONT]) {
    p->rwalk.hit_side = SDIS_FRONT;
  } else {
    FATAL("wavefront M6: unexpected fluid interface\n");
  }

  /* Get interface */
  interf = scene_get_interface(scn, p->rwalk.hit_3d.prim.prim_id);

  /* Register heat path vertex */
  res = register_heat_vertex(p->ctx.heat_path, &p->rwalk.vtx, p->T.value,
    SDIS_HEAT_VERTEX_CONVECTION, (int)p->ctx.nbranchings);
  if(res != RES_OK) goto error;

  /* Setup fragment at sampled position */
  setup_interface_fragment_3d(&frag, &p->rwalk.vtx,
                              &p->rwalk.hit_3d, p->rwalk.hit_side);

  /* Get convection coefficient */
  hc = interface_get_convection_coef(interf, &frag);
  if(hc > enc->hc_upper_bound) {
    log_err(scn->dev,
      "wavefront M6: hc (%g) exceeds hc_upper_bound (%g) at (%g, %g, %g)\n",
      hc, enc->hc_upper_bound, SPLIT3(p->rwalk.vtx.P));
    res = RES_BAD_OP;
    goto error;
  }

  /* Accept/reject null-collision */
  r = ssp_rng_canonical_float(p->rng);
  if(r < hc / enc->hc_upper_bound) {
    /* True convection → boundary dispatch */
    p->rwalk.hit_3d.distance = 0;
    p->rwalk.enc_id = ENCLOSURE_ID_NULL;
    p->T.func = boundary_path_3d;
    p->phase = PATH_BND_DISPATCH;
    p->needs_ray = 0;
  } else {
    /* Null-collision → loop back to self */
    p->phase = PATH_CNV_SAMPLE_LOOP;
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

/*******************************************************************************
 * B-4 M6: Boundary dispatch + Robin post-check
 ******************************************************************************/

/* --- PATH_BND_DISPATCH: Dirichlet check + 3-way dispatch ----------------- */
LOCAL_SYM res_T
step_bnd_dispatch(struct path_state* p, struct sdis_scene* scn)
{
  struct sdis_interface* interf = NULL;
  struct sdis_interface_fragment frag = SDIS_INTERFACE_FRAGMENT_NULL;
  struct sdis_medium* mdm_front = NULL;
  struct sdis_medium* mdm_back = NULL;
  double tmp;
  res_T res = RES_OK;

  ASSERT(p && scn);
  ASSERT(!S3D_HIT_NONE(&p->rwalk.hit_3d));

  /* Replicate boundary_path_3d entry logic:
   *   1. Setup fragment + normalize normal
   *   2. Check Dirichlet temperature
   *   3. Get front/back media → 3-way dispatch
   *   4. Robin post-check */

  setup_interface_fragment_3d(&frag, &p->rwalk.vtx,
                              &p->rwalk.hit_3d, p->rwalk.hit_side);
  f3_normalize(p->rwalk.hit_3d.normal, p->rwalk.hit_3d.normal);

  interf = scene_get_interface(scn, p->rwalk.hit_3d.prim.prim_id);

  /* Check Dirichlet temperature */
  tmp = interface_side_get_temperature(interf, &frag);
  if(SDIS_TEMPERATURE_IS_KNOWN(tmp)) {
    p->T.value += tmp;
    p->T.done = 1;
    if(p->ctx.green_path) {
      res = green_path_set_limit_interface_fragment(
        p->ctx.green_path, interf, &frag, p->rwalk.elapsed_time);
      if(res != RES_OK) goto error;
    }
    if(p->ctx.heat_path) {
      heat_path_get_last_vertex(p->ctx.heat_path)->weight = p->T.value;
    }
    p->phase = PATH_DONE;
    p->active = 0;
    p->done_reason = 3; /* boundary done */
    goto exit;
  }

  mdm_front = interface_get_medium(interf, SDIS_FRONT);
  mdm_back = interface_get_medium(interf, SDIS_BACK);

  /* 3-way dispatch */
  if(mdm_front->type == mdm_back->type) {
    /* solid/solid → M3 batched reinjection */
    res = step_bnd_ss_reinject_sample(p, scn);
  } else if(p->ctx.nbranchings == p->ctx.max_branchings) {
    /* solid/fluid picard1 → M5 batched state machine */
    res = step_bnd_sf_reinject_sample(p, scn);
  } else {
    /* solid/fluid picardN → synchronous fallback (until M8) */
    res = solid_fluid_boundary_picardN_path_3d(
      scn, &p->ctx, &frag, &p->rwalk, p->rng, &p->T);
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
  }

exit:
  return res;
error:
  p->phase = PATH_DONE;
  p->active = 0;
  p->done_reason = -1;
  goto exit;
}

/* --- PATH_BND_POST_ROBIN_CHECK: Robin boundary condition post-check ------ */
LOCAL_SYM res_T
step_bnd_post_robin_check(struct path_state* p, struct sdis_scene* scn)
{
  res_T res = RES_OK;

  ASSERT(p && scn);

  /* Robin post-check: if next path is convective or conductive, check
   * whether the medium temperature is already known from the boundary.
   * Mirrors boundary_path_3d lines 101-105. */
  if(p->T.func == convective_path_3d || p->T.func == conductive_path_3d) {
    res = query_medium_temperature_from_boundary_3d(
      scn, &p->ctx, &p->rwalk, &p->T);
    if(res != RES_OK) goto error;
    if(p->T.done) {
      p->phase = PATH_DONE;
      p->active = 0;
      p->done_reason = 2; /* temperature known */
      goto exit;
    }
  }

  /* Temperature not known — route to next path type */
  if(p->T.func == convective_path_3d) {
    p->phase = PATH_COUPLED_CONVECTIVE;
  } else if(p->T.func == conductive_path_3d) {
    p->phase = PATH_COUPLED_CONDUCTIVE;
  } else if(p->T.func == radiative_path_3d) {
    p->phase = PATH_COUPLED_RADIATIVE;
  } else if(p->T.func == boundary_path_3d) {
    p->phase = PATH_BND_DISPATCH;
  } else {
    FATAL("wavefront M6: unexpected T.func in post-robin check\n");
  }
  p->needs_ray = 0;

exit:
  return res;
error:
  p->phase = PATH_DONE;
  p->active = 0;
  p->done_reason = -1;
  goto exit;
}

/* --- PATH_COUPLED_RADIATIVE: bounce into radiative from boundary --------- */
LOCAL_SYM res_T
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
 * Dispatch — advance one path by one step
 ******************************************************************************/

/* Advance one path by one step (without ray).  Returns 1 if path was advanced,
 * 0 if it needs a ray or is done. */
LOCAL_SYM res_T
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
    /* These phases need ray results -- cannot advance without them */
    break;

  case PATH_DONE:
  case PATH_ERROR:
    break;

  /* --- B-4 fine-grained: ray-pending states (cannot advance w/o ray) --- */
  case PATH_BND_SS_REINJECT_SAMPLE:
  case PATH_BND_SF_REINJECT_SAMPLE:
  case PATH_BND_SF_NULLCOLL_RAD_TRACE:
  case PATH_BND_SFN_RAD_TRACE:
  case PATH_BND_EXT_DIRECT_TRACE:
  case PATH_BND_EXT_DIFFUSE_TRACE:
  case PATH_BND_EXT_DIFFUSE_SHADOW_TRACE:
  case PATH_CND_INIT_ENC:
  case PATH_CND_DS_STEP_TRACE:
  case PATH_CND_WOS_CLOSEST:
  case PATH_CND_WOS_FALLBACK_TRACE:
  case PATH_CNV_STARTUP_TRACE:
  case PATH_ENC_QUERY_EMIT:
    /* B-4: ray-pending, cannot advance without ray */
    break;

  /* --- B-4 M1: Enclosure query resolve (compute-only, activated) --- */
  case PATH_ENC_QUERY_RESOLVE:
    res = step_enc_query_resolve(p, scn);
    *advanced = 1;
    break;

  /* --- B-4 M3: solid/solid reinjection (compute-only, activated) --- */
  case PATH_BND_SS_REINJECT_DECIDE:
    res = step_bnd_ss_reinject_decide(p, scn);
    *advanced = 1;
    break;
  case PATH_BND_SS_REINJECT_ENC:
    res = step_bnd_ss_reinject_enc_result(p, scn);
    *advanced = 1;
    break;

  /* --- B-4 M4: delta-sphere conductive (compute-only, activated) --- */
  case PATH_CND_DS_CHECK_TEMP:
    res = step_cnd_ds_check_temp(p, scn);
    *advanced = 1;
    break;
  case PATH_CND_DS_STEP_ENC_VERIFY:
    step_cnd_ds_step_enc_verify(p);
    *advanced = 1;
    break;
  case PATH_CND_DS_STEP_ADVANCE:
    res = step_cnd_ds_step_advance(p, scn);
    *advanced = 1;
    break;

  /* --- B-4 M6: Convective path fine-grained (compute-only, activated) --- */
  case PATH_CNV_INIT:
    res = step_cnv_init(p, scn);
    *advanced = 1;
    break;
  case PATH_CNV_SAMPLE_LOOP:
    res = step_cnv_sample_loop(p, scn);
    *advanced = 1;
    break;

  /* --- B-4 M6: Boundary dispatch + Robin post-check (activated) --- */
  case PATH_BND_DISPATCH:
    res = step_bnd_dispatch(p, scn);
    *advanced = 1;
    break;
  case PATH_BND_POST_ROBIN_CHECK:
    res = step_bnd_post_robin_check(p, scn);
    *advanced = 1;
    break;

  /* --- B-4 M5: solid/fluid picard1 (compute-only, activated) --- */
  case PATH_BND_SF_REINJECT_ENC:
    res = step_bnd_sf_reinject_enc_result(p, scn);
    *advanced = 1;
    break;
  case PATH_BND_SF_PROB_DISPATCH:
    res = step_bnd_sf_prob_dispatch(p, scn);
    *advanced = 1;
    break;
  case PATH_BND_SF_NULLCOLL_DECIDE:
    res = step_bnd_sf_nullcoll_decide(p, scn);
    *advanced = 1;
    break;

  /* --- B-4 M7: External net flux (compute-only, activated) --- */
  case PATH_BND_EXT_CHECK:
    res = step_bnd_ext_check(p, scn);
    *advanced = 1;
    break;
  case PATH_BND_EXT_FINALIZE:
    res = step_bnd_ext_finalize(p, scn);
    *advanced = 1;
    break;

  /* --- B-4 fine-grained: compute-only states (future, no-op for now) --- */
  case PATH_RAD_PROCESS_HIT:
  case PATH_BND_SFN_PROB_DISPATCH:
  case PATH_BND_SFN_RAD_DONE:
  case PATH_BND_SFN_COMPUTE_Ti:
  case PATH_BND_SFN_COMPUTE_Ti_RESUME:
  case PATH_BND_SFN_CHECK_PMIN_PMAX:
  case PATH_CND_DS_STEP_PROCESS:
  case PATH_CND_WOS_CHECK_TEMP:
  case PATH_CND_WOS_CLOSEST_RESULT:
  case PATH_CND_WOS_FALLBACK_RESULT:
  case PATH_CND_WOS_TIME_TRAVEL:
  case PATH_CND_CUSTOM:
  case PATH_CNV_STARTUP_RESULT:
    /* B-4 compute-only: not yet activated, no-op fallback */
    break;

  /* M7 result phases: handled inline by advance_one_step_with_ray
   * (TRACE → step_*_result). Never assigned as p->phase directly. */
  case PATH_BND_EXT_DIRECT_RESULT:
  case PATH_BND_EXT_DIFFUSE_RESULT:
  case PATH_BND_EXT_DIFFUSE_SHADOW_RESULT:
    break;

  case PATH_PHASE_COUNT:
  default:
    VFATAL("wavefront: unknown path phase %d\n", ARG1((int)p->phase));
    break;
  }

  return res;
}

/* Advance one path after receiving a ray trace result */
LOCAL_SYM res_T
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

  /* --- B-4 M3: solid/solid reinjection -- receive 4-ray results ---
   * 4 hits already pre-delivered to bnd_ss.ray_frt/ray_bck by distribute. */
  case PATH_BND_SS_REINJECT_SAMPLE: {
    const struct s3d_hit* h_frt0 = &p->locals.bnd_ss.ray_frt[0];
    const struct s3d_hit* h_frt1 = &p->locals.bnd_ss.ray_frt[1];
    const struct s3d_hit* h_bck0 = &p->locals.bnd_ss.ray_bck[0];
    const struct s3d_hit* h_bck1 = &p->locals.bnd_ss.ray_bck[1];
    res = step_bnd_ss_reinject_process(p, scn, h_frt0, h_frt1, h_bck0, h_bck1);
    break;
  }

  /* PATH_BND_SS_REINJECT_ENC is compute-only (handled in advance_no_ray) */

  /* --- B-4 M4: delta-sphere 2-ray results --- */
  case PATH_CND_DS_STEP_TRACE:
    res = step_conductive_ds_process(p, scn, hit0, hit1);
    break;

  /* --- B-4 M5: solid/fluid picard1 reinjection (2-ray result) --- */
  case PATH_BND_SF_REINJECT_SAMPLE:
    res = step_bnd_sf_reinject_process(p, scn, hit0, hit1);
    break;

  /* --- B-4 M5: radiative sub-path trace (1-ray result) --- */
  case PATH_BND_SF_NULLCOLL_RAD_TRACE:
    res = step_bnd_sf_nullcoll_rad_trace(p, scn, hit0);
    break;

  /* --- B-4 M7: External net flux ray results (activated) --- */
  case PATH_BND_EXT_DIRECT_TRACE:
    res = step_bnd_ext_direct_result(p, scn, hit0);
    break;
  case PATH_BND_EXT_DIFFUSE_TRACE:
    res = step_bnd_ext_diffuse_result(p, scn, hit0);
    break;
  case PATH_BND_EXT_DIFFUSE_SHADOW_TRACE:
    res = step_bnd_ext_diffuse_shadow_result(p, scn, hit0);
    break;

  /* --- B-4 future: ray-pending states, not yet activated --- */
  case PATH_BND_SFN_RAD_TRACE:
  case PATH_CND_INIT_ENC:
  case PATH_CND_DS_STEP_ENC_VERIFY:
  case PATH_CND_WOS_CLOSEST:
  case PATH_CND_WOS_FALLBACK_TRACE:
  /* --- B-4 M1: Enclosure query -- receive 6 ray results --- */
  case PATH_ENC_QUERY_EMIT:
    /* 6 hits already pre-delivered to enc_query.dir_hits[] by distribute.
     * Transition to resolve phase (will be cascaded immediately). */
    p->phase = PATH_ENC_QUERY_RESOLVE;
    break;

  /* --- B-4 M6: Convective startup ray result --- */
  case PATH_CNV_STARTUP_TRACE:
    res = step_cnv_startup_result(p, scn, hit0);
    break;

  default:
    VFATAL("wavefront: advance_with_ray in unexpected phase %d\n",
           ARG1((int)p->phase));
    break;
  }

  return res;
}
