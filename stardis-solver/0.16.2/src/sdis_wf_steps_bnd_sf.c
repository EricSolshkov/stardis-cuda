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


/* Wavefront steps: solid/fluid picard1 boundary (M5).  Split from sdis_wf_steps.c. */

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
    p->phase = p->locals.bnd_sf.is_picardn
             ? PATH_BND_SFN_PROB_DISPATCH
             : PATH_BND_SF_PROB_DISPATCH;
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
  const struct s3d_hit* hit1,
  struct path_enc_data* enc)
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
    {
      /* Diagnostic: dump raw hit data for each ray to understand enc mismatch */
      unsigned enc0_pair[2] = {0,0}, enc1_pair[2] = {0,0};
      float dot0 = 0, dot1 = 0;
      if(!S3D_HIT_NONE(hit0)) {
        scene_get_enclosure_ids(scn, hit0->prim.prim_id, enc0_pair);
        dot0 = f3_dot(p->locals.bnd_sf.reinject_dir[0], hit0->normal);
      }
      if(!S3D_HIT_NONE(hit1)) {
        scene_get_enclosure_ids(scn, hit1->prim.prim_id, enc1_pair);
        dot1 = f3_dot(p->locals.bnd_sf.reinject_dir[1], hit1->normal);
      }
      // log_warn(scn->dev,
      //   "M5_SF_RETRY retry=%d path=%u oprim=%u pos=%g,%g,%g h0prim=%u h0dist=%g h0N=%g,%g,%g h0dot=%g h0enc=%u,%u h0side=%s h0enc_res=%u h1prim=%u h1dist=%g h1N=%g,%g,%g h1dot=%g h1enc=%u,%u h1side=%s h1enc_res=%u solid_enc=%u delta=%g dst=%g\n",
      //   p->locals.bnd_sf.retry_count, p->path_id,
      //   p->rwalk.hit_3d.prim.prim_id, SPLIT3(p->rwalk.vtx.P),
      //   S3D_HIT_NONE(hit0) ? 0xFFFFFFFF : hit0->prim.prim_id,
      //   S3D_HIT_NONE(hit0) ? 0.0 : (double)hit0->distance,
      //   S3D_HIT_NONE(hit0) ? 0.0 : (double)hit0->normal[0],
      //   S3D_HIT_NONE(hit0) ? 0.0 : (double)hit0->normal[1],
      //   S3D_HIT_NONE(hit0) ? 0.0 : (double)hit0->normal[2],
      //   (double)dot0, enc0_pair[0], enc0_pair[1],
      //   dot0 < 0 ? "FRT" : "BCK",
      //   p->locals.bnd_sf.enc0_id,
      //   S3D_HIT_NONE(hit1) ? 0xFFFFFFFF : hit1->prim.prim_id,
      //   S3D_HIT_NONE(hit1) ? 0.0 : (double)hit1->distance,
      //   S3D_HIT_NONE(hit1) ? 0.0 : (double)hit1->normal[0],
      //   S3D_HIT_NONE(hit1) ? 0.0 : (double)hit1->normal[1],
      //   S3D_HIT_NONE(hit1) ? 0.0 : (double)hit1->normal[2],
      //   (double)dot1, enc1_pair[0], enc1_pair[1],
      //   dot1 < 0 ? "FRT" : "BCK",
      //   p->locals.bnd_sf.enc1_id,
      //   p->locals.bnd_sf.solid_enc_id,
      //   p->locals.bnd_sf.delta_boundary,
      //   (double)p->locals.bnd_sf.chosen_dst);
    }
    if(p->locals.bnd_sf.retry_count >= 10) {
      log_warn(scn->dev,
        "M5_SF_FAIL path=%u px=%u,%u spp=%u steps=%zu prim=%u pos=%g,%g,%g N=%g,%g,%g solid_side=%d solid_enc=%u enc0=%u enc1=%u delta=%g dst=%g h0miss=%d h1miss=%d dir0=%g,%g,%g dir1=%g,%g,%g\n",
        p->path_id, p->pixel_x, p->pixel_y,
        p->realisation_idx, p->steps_taken,
        p->rwalk.hit_3d.prim.prim_id, SPLIT3(p->rwalk.vtx.P),
        p->rwalk.hit_3d.normal[0], p->rwalk.hit_3d.normal[1],
        p->rwalk.hit_3d.normal[2], (int)p->locals.bnd_sf.solid_side,
        p->locals.bnd_sf.solid_enc_id,
        p->locals.bnd_sf.enc0_id, p->locals.bnd_sf.enc1_id,
        p->locals.bnd_sf.delta_boundary,
        (double)p->locals.bnd_sf.chosen_dst,
        S3D_HIT_NONE(hit0) ? 1 : 0,
        S3D_HIT_NONE(hit1) ? 1 : 0,
        (double)p->locals.bnd_sf.reinject_dir[0][0],
        (double)p->locals.bnd_sf.reinject_dir[0][1],
        (double)p->locals.bnd_sf.reinject_dir[0][2],
        (double)p->locals.bnd_sf.reinject_dir[1][0],
        (double)p->locals.bnd_sf.reinject_dir[1][1],
        (double)p->locals.bnd_sf.reinject_dir[1][2]);
      res = RES_BAD_OP_IRRECOVERABLE;
      goto error;
    }
    /* Move position away from primitive boundaries to avoid edge/corner
     * numerical issues (mirrors original find_reinjection_ray logic). */
    move_away_primitive_boundaries_3d(&p->rwalk.hit_3d,
      p->locals.bnd_sf.delta_boundary, p->locals.bnd_sf.rwalk_pos_backup);

    /* Restore (possibly moved) position and re-sample */
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
    step_enc_query_emit(p, enc, pos, PATH_BND_SF_REINJECT_ENC);
    return RES_OK;
  }

  /* Reinjection resolved — proceed to probability dispatch */
  p->phase = p->locals.bnd_sf.is_picardn
           ? PATH_BND_SFN_PROB_DISPATCH
           : PATH_BND_SF_PROB_DISPATCH;
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
step_bnd_sf_reinject_enc_result(struct path_state* p, struct sdis_scene* scn,
                               struct path_enc_data* enc)
{
  unsigned enc_id;
  res_T res = RES_OK;

  ASSERT(p && scn);
  enc_id = enc->resolved_enc_id;

  if(enc_id != p->locals.bnd_sf.solid_enc_id) {
    /* Reinjection endpoint not in solid enclosure — retry */
    p->locals.bnd_sf.retry_count++;
     /*log_warn(scn->dev,
       "M5_SF_ENC_RETRY retry=%d path=%u prim=%u pos=%g,%g,%g solid_enc=%u enc_resolved=%u dir=%g,%g,%g dst=%g\n",
       p->locals.bnd_sf.retry_count, p->path_id,
       p->rwalk.hit_3d.prim.prim_id,
       SPLIT3(p->rwalk.vtx.P),
       p->locals.bnd_sf.solid_enc_id, enc_id,
       (double)p->locals.bnd_sf.chosen_dir[0],
       (double)p->locals.bnd_sf.chosen_dir[1],
       (double)p->locals.bnd_sf.chosen_dir[2],
       (double)p->locals.bnd_sf.chosen_dst);*/
    if(p->locals.bnd_sf.retry_count >= 10) {
      /*log_warn(scn->dev,
        "M5_SF_ENC_FAIL path=%u px=%u,%u spp=%u steps=%zu prim=%u pos=%g,%g,%g solid_enc=%u enc_resolved=%u dir=%g,%g,%g dst=%g\n",
        p->path_id, p->pixel_x, p->pixel_y,
        p->realisation_idx, p->steps_taken,
        p->rwalk.hit_3d.prim.prim_id, SPLIT3(p->rwalk.vtx.P),
        p->locals.bnd_sf.solid_enc_id, enc_id,
        (double)p->locals.bnd_sf.chosen_dir[0],
        (double)p->locals.bnd_sf.chosen_dir[1],
        (double)p->locals.bnd_sf.chosen_dir[2],
        (double)p->locals.bnd_sf.chosen_dst);*/
      p->phase = PATH_DONE;
      p->active = 0;
      p->done_reason = -1;
      return RES_BAD_OP_IRRECOVERABLE;
    }
    /* Move position away from primitive boundaries before retry */
    move_away_primitive_boundaries_3d(&p->rwalk.hit_3d,
      p->locals.bnd_sf.delta_boundary, p->locals.bnd_sf.rwalk_pos_backup);

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
  p->phase = p->locals.bnd_sf.is_picardn
           ? PATH_BND_SFN_PROB_DISPATCH
           : PATH_BND_SF_PROB_DISPATCH;
  p->needs_ray = 0;
  return RES_OK;
}

/* --- PATH_BND_SF_PROB_DISPATCH: solid_reinjection + prob dispatch --------- */
LOCAL_SYM res_T
step_bnd_sf_prob_dispatch(struct path_state* p, struct sdis_scene* scn,
                         struct path_ext_data* ext)
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
    ext->return_state = PATH_BND_SF_PROB_DISPATCH;
    p->phase = PATH_BND_EXT_CHECK;
    p->needs_ray = 0;
    return step_bnd_ext_check(p, scn, ext);
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
      p->locals.cnd_wos.wos_initialized = 0; /* union was bnd_sf — clear */
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

    /* Clear stale hit — matches CPU set_limit_radiative_temperature which
     * ASSERTs SXD_HIT_NONE and sets hit_side = SDIS_SIDE_NULL__. */
    p->rwalk.hit_3d = S3D_HIT_NULL;
    p->rwalk.hit_side = SDIS_SIDE_NULL__;

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

