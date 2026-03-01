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


/* Wavefront steps: core dispatch + utility + ray setup.  Split from sdis_wf_steps.c. */

#include "sdis_wf_steps.h"
#include "sdis_solve_wavefront.h"  /* struct wavefront_context (used by types) */
#include "sdis_solve_persistent_wavefront.h"  /* P1: full wavefront_pool definition */
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
step_conductive(struct path_state* p, struct sdis_scene* scn,
                struct path_enc_data* enc)
{
  res_T res = RES_OK;
  ASSERT(p && scn);

  /* Custom paths: still delegate to the synchronous implementation.
   * Delta_sphere (M4) and WoS (M9) are wavefront-ized. */
  if(p->ctx.diff_algo == SDIS_DIFFUSION_WOS) {
    /* ----- Walk on Spheres wavefront path (M9 fine-grained) ----- */
    if(!p->locals.cnd_wos.wos_initialized) {
      /* First entry: emit 6-ray enc_query to discover enc_id.
       * After result, PATH_CND_WOS_CHECK_TEMP initialises WoS. */
      step_enc_query_emit(p, enc, p->rwalk.vtx.P, PATH_CND_WOS_CHECK_TEMP);
    } else {
      /* Re-entry after coupled boundary: go straight to check_temp */
      p->phase = PATH_CND_WOS_CHECK_TEMP;
    }
    goto exit;
  }

  if(p->ctx.diff_algo != SDIS_DIFFUSION_DELTA_SPHERE) {
    /* Custom -- synchronous fallback */
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
    /* Emit 6-ray enc_query from current position.
     * After result, PATH_CND_DS_CHECK_TEMP finalises init. */
    step_enc_query_emit(p, enc, p->rwalk.vtx.P, PATH_CND_DS_CHECK_TEMP);
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
  const struct s3d_hit* hit1,
  struct path_enc_data* enc)
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
    d3_set(enc->query_pos, pos_next);
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
      enc->resolved_enc_id = enc_id;
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
                        int* advanced,
                        struct wavefront_pool* pool, size_t slot_idx)
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
    res = step_conductive(p, scn, &pool->enc_arr[slot_idx]);
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
  case PATH_CND_WOS_FALLBACK_TRACE:
  case PATH_CNV_STARTUP_TRACE:
    /* B-4: ray-pending, cannot advance without ray */
    break;

  /* B-4 M9: cp-pending — cannot advance without closest_point result */
  case PATH_CND_WOS_CLOSEST:
  case PATH_CND_WOS_DIFFUSION_CHECK:
    break;

  /* B-4 M10: enc_locate-pending — cannot advance without enc_locate result */
  case PATH_ENC_LOCATE_PENDING:
    break;

  /* --- B-4 M10: Enclosure locate result (compute-only, activated) --- */
  case PATH_ENC_LOCATE_RESULT:
    res = step_enc_locate_result(p, scn, &pool->enc_arr[slot_idx]);
    *advanced = 1;
    break;

  /* --- B-4 M3: solid/solid reinjection (compute-only, activated) --- */
  case PATH_BND_SS_REINJECT_DECIDE:
    res = step_bnd_ss_reinject_decide(p, scn);
    *advanced = 1;
    break;
  case PATH_BND_SS_REINJECT_ENC:
    res = step_bnd_ss_reinject_enc_result(p, scn, &pool->enc_arr[slot_idx]);
    *advanced = 1;
    break;

  /* --- B-4 M4: delta-sphere conductive (compute-only, activated) --- */
  case PATH_CND_DS_CHECK_TEMP:
    res = step_cnd_ds_check_temp(p, scn, &pool->enc_arr[slot_idx]);
    *advanced = 1;
    break;
  case PATH_CND_DS_STEP_ENC_VERIFY:
    step_cnd_ds_step_enc_verify(p, &pool->enc_arr[slot_idx]);
    *advanced = 1;
    break;
  case PATH_CND_DS_STEP_ADVANCE:
    res = step_cnd_ds_step_advance(p, scn, &pool->enc_arr[slot_idx]);
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
    res = step_bnd_sf_reinject_enc_result(p, scn, &pool->enc_arr[slot_idx]);
    *advanced = 1;
    break;
  case PATH_BND_SF_PROB_DISPATCH:
    res = step_bnd_sf_prob_dispatch(p, scn, &pool->ext_arr[slot_idx]);
    *advanced = 1;
    break;
  case PATH_BND_SF_NULLCOLL_DECIDE:
    res = step_bnd_sf_nullcoll_decide(p, scn);
    *advanced = 1;
    break;

  /* --- B-4 M7: External net flux (compute-only, activated) --- */
  case PATH_BND_EXT_CHECK:
    res = step_bnd_ext_check(p, scn, &pool->ext_arr[slot_idx]);
    *advanced = 1;
    break;
  case PATH_BND_EXT_FINALIZE:
    res = step_bnd_ext_finalize(p, scn, &pool->ext_arr[slot_idx]);
    *advanced = 1;
    break;

  /* --- B-4 M8: picardN compute-only states (activated) --- */
  case PATH_BND_SFN_PROB_DISPATCH:
    res = step_bnd_sfn_prob_dispatch(p, scn, &pool->sfn_arr[slot_idx]);
    *advanced = 1;
    break;
  case PATH_BND_SFN_RAD_DONE:
    res = step_bnd_sfn_rad_done(p, scn, &pool->sfn_arr[slot_idx]);
    *advanced = 1;
    break;
  case PATH_BND_SFN_COMPUTE_Ti:
    res = step_bnd_sfn_compute_Ti(p, scn, &pool->sfn_arr[slot_idx]);
    *advanced = 1;
    break;
  case PATH_BND_SFN_COMPUTE_Ti_RESUME:
    res = step_bnd_sfn_compute_Ti_resume(p, scn, &pool->sfn_arr[slot_idx]);
    *advanced = 1;
    break;
  case PATH_BND_SFN_CHECK_PMIN_PMAX:
    res = step_bnd_sfn_check_pmin_pmax(p, scn, &pool->sfn_arr[slot_idx]);
    *advanced = 1;
    break;

  /* --- B-4 M9: Walk on Spheres conductive (compute-only, activated) --- */
  case PATH_CND_WOS_CHECK_TEMP:
    res = step_cnd_wos_check_temp(p, scn);
    *advanced = 1;
    break;
  case PATH_CND_WOS_CLOSEST_RESULT:
    res = step_cnd_wos_closest_result(p, scn);
    *advanced = 1;
    break;
  case PATH_CND_WOS_DIFFUSION_CHECK_RESULT:
    res = step_cnd_wos_diffusion_check_result(p, scn);
    *advanced = 1;
    break;
  case PATH_CND_WOS_TIME_TRAVEL:
    res = step_cnd_wos_time_travel(p, scn);
    *advanced = 1;
    break;

  /* --- B-4 fine-grained: compute-only states (future, not yet activated) --- */
  case PATH_RAD_PROCESS_HIT:
  case PATH_CND_DS_STEP_PROCESS:
  case PATH_CND_WOS_FALLBACK_RESULT: /* unreachable: handled inline by with_ray */
  case PATH_CND_CUSTOM:
  case PATH_CNV_STARTUP_RESULT:
    VFATAL("wavefront: advance_no_ray hit unactivated future state %d -- "
           "this state has no step function yet\n", ARG1((int)p->phase));
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
                          const struct s3d_hit* hit1,
                          struct wavefront_pool* pool, size_t slot_idx)
{
  res_T res = RES_OK;

  if(!p->active) return RES_OK;

  p->needs_ray = 0;

  switch(p->phase) {
  case PATH_RAD_TRACE_PENDING:
    res = step_radiative_trace(p, scn, hit0);
    break;

  case PATH_COUPLED_COND_DS_PENDING:
    res = step_conductive_ds_process(p, scn, hit0, hit1,
                                     &pool->enc_arr[slot_idx]);
    break;

  /* --- B-4 M3: solid/solid reinjection -- receive 4-ray results ---
   * 4 hits already pre-delivered to bnd_ss.ray_frt/ray_bck by distribute. */
  case PATH_BND_SS_REINJECT_SAMPLE: {
    const struct s3d_hit* h_frt0 = &p->locals.bnd_ss.ray_frt[0];
    const struct s3d_hit* h_frt1 = &p->locals.bnd_ss.ray_frt[1];
    const struct s3d_hit* h_bck0 = &p->locals.bnd_ss.ray_bck[0];
    const struct s3d_hit* h_bck1 = &p->locals.bnd_ss.ray_bck[1];
    res = step_bnd_ss_reinject_process(p, scn, h_frt0, h_frt1, h_bck0, h_bck1,
                                        &pool->enc_arr[slot_idx]);
    break;
  }

  /* PATH_BND_SS_REINJECT_ENC is compute-only (handled in advance_no_ray) */

  /* --- B-4 M4: delta-sphere 2-ray results --- */
  case PATH_CND_DS_STEP_TRACE:
    res = step_conductive_ds_process(p, scn, hit0, hit1,
                                     &pool->enc_arr[slot_idx]);
    break;

  /* --- B-4 M5: solid/fluid picard1 reinjection (2-ray result) --- */
  case PATH_BND_SF_REINJECT_SAMPLE:
    res = step_bnd_sf_reinject_process(p, scn, hit0, hit1,
                                       &pool->enc_arr[slot_idx]);
    break;

  /* --- B-4 M5: radiative sub-path trace (1-ray result) --- */
  case PATH_BND_SF_NULLCOLL_RAD_TRACE:
    res = step_bnd_sf_nullcoll_rad_trace(p, scn, hit0);
    break;

  /* --- B-4 M7: External net flux ray results (activated) --- */
  case PATH_BND_EXT_DIRECT_TRACE:
    res = step_bnd_ext_direct_result(p, scn, hit0,
                                     &pool->ext_arr[slot_idx]);
    break;
  case PATH_BND_EXT_DIFFUSE_TRACE:
    res = step_bnd_ext_diffuse_result(p, scn, hit0,
                                      &pool->ext_arr[slot_idx]);
    break;
  case PATH_BND_EXT_DIFFUSE_SHADOW_TRACE:
    res = step_bnd_ext_diffuse_shadow_result(p, scn, hit0,
                                              &pool->ext_arr[slot_idx]);
    break;

  /* --- B-4 M8: picardN radiative sub-path ray result (activated) --- */
  case PATH_BND_SFN_RAD_TRACE:
    res = step_bnd_sfn_rad_trace(p, scn, hit0);
    break;

  /* --- B-4 M9: WoS conductive fallback -- 1-ray result --- */
  case PATH_CND_WOS_FALLBACK_TRACE:
    res = step_cnd_wos_fallback_result(p, scn, hit0);
    break;

  /* --- B-4 future: ray-pending states, not yet activated --- */
  case PATH_CND_INIT_ENC:
    VFATAL("wavefront: advance_with_ray hit unactivated future state %d -- "
           "this state has no step function yet\n", ARG1((int)p->phase));
    break;
  /* NOTE: PATH_CND_DS_STEP_ENC_VERIFY is compute-only (already handled by
   * advance_one_step_no_ray).  It must NOT appear here. */

  /* --- B-4 M6: Convective startup ray result --- */
  case PATH_CNV_STARTUP_TRACE:
    res = step_cnv_startup_result(p, scn, hit0);
    break;

  /* --- B-4 M1-v2: 6-ray enclosure query results ---
   * 6 hits already pre-delivered to enc_query.dir_hits[] by distribute. */
  case PATH_ENC_QUERY_EMIT:
    res = step_enc_query_resolve(p, scn, &pool->enc_arr[slot_idx]);
    break;

  /* --- B-4 M1-v2: Fallback 1-ray result ---
   * 1 hit pre-delivered to enc_query.fb_hit by distribute. */
  case PATH_ENC_QUERY_FB_EMIT:
    res = step_enc_query_fb_resolve(p, scn, &pool->enc_arr[slot_idx]);
    break;

  default:
    VFATAL("wavefront: advance_with_ray in unexpected phase %d\n",
           ARG1((int)p->phase));
    break;
  }

  return res;
}

