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


/* Wavefront steps: convective + boundary dispatch + Robin (M6).  Split from sdis_wf_steps.c. */

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
 * B-4 M6: Convective path fine-grained state machine
 ******************************************************************************/

/* --- PATH_CNV_INIT: check known fluid temp + decide startup ray or loop -- */
LOCAL_SYM res_T
step_cnv_init(struct path_state* p, struct path_hot* hot, struct sdis_scene* scn)
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
    hot->phase = (uint8_t)PATH_DONE;
    hot->active = 0;
    p->done_reason = 2; /* temperature known */
    goto exit;
  }

  /* Check if path starts from fluid interior (HIT_NONE) */
  if(S3D_HIT_NONE(&p->rwalk.hit_3d)) {
    /* Need startup ray along +Z to find initial hit */
    setup_convective_startup_ray(p, hot);
    hot->phase = (uint8_t)PATH_CNV_STARTUP_TRACE;
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
      hot->phase = (uint8_t)PATH_DONE;
      hot->active = 0;
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
  hot->phase = (uint8_t)PATH_CNV_SAMPLE_LOOP;

exit:
  return res;
error:
  hot->phase = (uint8_t)PATH_DONE;
  hot->active = 0;
  p->done_reason = -1;
  goto exit;
}

/* --- PATH_CNV_STARTUP_RESULT: process startup ray result ----------------- */
LOCAL_SYM res_T
step_cnv_startup_result(struct path_state* p, struct path_hot* hot,
                        struct sdis_scene* scn,
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
      hot->phase = (uint8_t)PATH_DONE;
      hot->active = 0;
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

  hot->phase = (uint8_t)PATH_CNV_SAMPLE_LOOP;

exit:
  return res;
error:
  hot->phase = (uint8_t)PATH_DONE;
  hot->active = 0;
  p->done_reason = -1;
  goto exit;
}

/* --- PATH_CNV_SAMPLE_LOOP: null-collision sampling (pure compute) -------- */
LOCAL_SYM res_T
step_cnv_sample_loop(struct path_state* p, struct path_hot* hot,
                     struct sdis_scene* scn)
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
    hot->phase = (uint8_t)PATH_DONE;
    hot->active = 0;
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
    hot->phase = (uint8_t)PATH_BND_DISPATCH;
    hot->needs_ray = 0;
  } else {
    /* Null-collision → loop back to self */
    hot->phase = (uint8_t)PATH_CNV_SAMPLE_LOOP;
    hot->needs_ray = 0;
  }

exit:
  return res;
error:
  hot->phase = (uint8_t)PATH_DONE;
  hot->active = 0;
  p->done_reason = -1;
  goto exit;
}

/*******************************************************************************
 * B-4 M6: Boundary dispatch + Robin post-check
 ******************************************************************************/

/* --- PATH_BND_DISPATCH: Dirichlet check + 3-way dispatch ----------------- */
LOCAL_SYM res_T
step_bnd_dispatch(struct path_state* p, struct path_hot* hot,
                  struct sdis_scene* scn)
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
    hot->phase = (uint8_t)PATH_DONE;
    hot->active = 0;
    p->done_reason = 3; /* boundary done */
    goto exit;
  }

  mdm_front = interface_get_medium(interf, SDIS_FRONT);
  mdm_back = interface_get_medium(interf, SDIS_BACK);

  /* 3-way dispatch */
  if(mdm_front->type == mdm_back->type) {
    /* solid/solid → M3 batched reinjection */
    res = step_bnd_ss_reinject_sample(p, hot, scn);
  } else if(p->ctx.nbranchings == p->ctx.max_branchings) {
    /* solid/fluid picard1 → M5 batched state machine.
     * Reset h_hat so that step_bnd_sf_prob_dispatch re-computes transfer
     * coefficients and calls handle_net_flux for EVERY boundary visit
     * (not just the first one). Without this, h_hat retains its non-zero
     * value from a previous visit and the init block (which includes the
     * handle_net_flux call) is skipped entirely. */
    p->locals.bnd_sf.is_picardn = 0;
    p->locals.bnd_sf.h_hat = 0;
    res = step_bnd_sf_reinject_sample(p, hot, scn);
  } else {
    /* solid/fluid picardN → M8 batched state machine.
     * Reuse M5 SF reinjection setup but mark as picardN so that
     * reinject_enc_result routes to SFN_PROB_DISPATCH.
     * NOTE: sfn_stack_depth is NOT reset here; for picard_order >= 3 this
     * function may be entered from a COMPUTE_Ti sub-path of an outer M8,
     * so the stack must nest properly. */
    p->locals.bnd_sf.is_picardn = 1;
    p->locals.bnd_sf.h_hat = 0;        /* force init in sfn_prob_dispatch  */
    res = step_bnd_sf_reinject_sample(p, hot, scn);
  }

exit:
  return res;
error:
  hot->phase = (uint8_t)PATH_DONE;
  hot->active = 0;
  p->done_reason = -1;
  goto exit;
}

/* --- PATH_BND_POST_ROBIN_CHECK: Robin boundary condition post-check ------ */
LOCAL_SYM res_T
step_bnd_post_robin_check(struct path_state* p, struct path_hot* hot,
                          struct sdis_scene* scn)
{
  res_T res = RES_OK;

  ASSERT(p && scn);

  /* Unconditional T.done check — mirrors CPU boundary_path_Xd.h line 88:
   *   "if(T->done) goto exit;"
   * This catches radiative miss-accept paths (T.done=1, T.func still
   * boundary_path_3d) that must NOT re-enter PATH_BND_DISPATCH. */
  if(p->T.done) {
    hot->phase = (uint8_t)PATH_DONE;
    hot->active = 0;
    p->done_reason = 2; /* temperature known */
    goto exit;
  }

  /* Robin post-check: if next path is convective or conductive, check
   * whether the medium temperature is already known from the boundary.
   * Mirrors boundary_path_3d lines 101-105. */
  if(p->T.func == convective_path_3d || p->T.func == conductive_path_3d) {
    res = query_medium_temperature_from_boundary_3d(
      scn, &p->ctx, &p->rwalk, &p->T);
    if(res != RES_OK) goto error;
    if(p->T.done) {
      hot->phase = (uint8_t)PATH_DONE;
      hot->active = 0;
      p->done_reason = 2; /* temperature known */
      goto exit;
    }
  }

  /* Temperature not known — route to next path type */
  if(p->T.func == convective_path_3d) {
    hot->phase = (uint8_t)PATH_COUPLED_CONVECTIVE;
  } else if(p->T.func == conductive_path_3d) {
    hot->phase = (uint8_t)PATH_COUPLED_CONDUCTIVE;
  } else if(p->T.func == radiative_path_3d) {
    hot->phase = (uint8_t)PATH_COUPLED_RADIATIVE;
  } else if(p->T.func == boundary_path_3d) {
    hot->phase = (uint8_t)PATH_BND_DISPATCH;
  } else {
    FATAL("wavefront M6: unexpected T.func in post-robin check\n");
  }
  hot->needs_ray = 0;

exit:
  return res;
error:
  hot->phase = (uint8_t)PATH_DONE;
  hot->active = 0;
  p->done_reason = -1;
  goto exit;
}

