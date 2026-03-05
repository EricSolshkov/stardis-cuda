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


/* Wavefront steps: solid/solid boundary reinjection (M3).  Split from sdis_wf_steps.c. */

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
setup_ss_reinject_rays(struct path_state* p, struct path_hot* hot)
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
  hot->ray_count_ext = (uint8_t)4;
  hot->ray_bucket = (uint8_t)RAY_BUCKET_STEP_PAIR;
  hot->needs_ray = 1;
  hot->phase = (uint8_t)PATH_BND_SS_REINJECT_SAMPLE;
}

/* --- PATH_BND_SS_REINJECT_SAMPLE entry: prepare and emit 4 rays ---------- */
LOCAL_SYM res_T
step_bnd_ss_reinject_sample(struct path_state* p, struct path_hot* hot, struct sdis_scene* scn)
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
    hot->phase = (uint8_t)PATH_BND_SS_REINJECT_DECIDE;
    hot->needs_ray = 0;
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
  setup_ss_reinject_rays(p, hot);
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
  struct path_hot* hot,
  struct sdis_scene* scn,
  const struct s3d_hit* hit_frt0,
  const struct s3d_hit* hit_frt1,
  const struct s3d_hit* hit_bck0,
  const struct s3d_hit* hit_bck1,
  struct path_enc_data* enc)
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
    log_warn(scn->dev,
      "M3_SS_RETRY retry=%d path=%u prim=%u pos=%g,%g,%g dst_frt=%g dst_bck=%g enc0f=%u enc1f=%u enc0b=%u enc1b=%u target_enc=%u,%u\n",
      p->locals.bnd_ss.retry_count, p->path_id,
      p->rwalk.hit_3d.prim.prim_id,
      SPLIT3(p->rwalk.vtx.P),
      (double)p->locals.bnd_ss.reinject_dst_frt,
      (double)p->locals.bnd_ss.reinject_dst_bck,
      p->locals.bnd_ss.enc0_frt, p->locals.bnd_ss.enc1_frt,
      p->locals.bnd_ss.enc0_bck, p->locals.bnd_ss.enc1_bck,
      p->locals.bnd_ss.enc_ids[SDIS_FRONT],
      p->locals.bnd_ss.enc_ids[SDIS_BACK]);
    if(p->locals.bnd_ss.retry_count >= 10) {
      log_warn(scn->dev,
        "M3_SS_FAIL path=%u px=%u,%u spp=%u steps=%zu prim=%u pos=%g,%g,%g N=%g,%g,%g side=%d enc_frt=%u enc_bck=%u multi=%d,%d dst_frt=%g dst_bck=%g delta_frt=%g delta_bck=%g enc0f=%u enc1f=%u enc0b=%u enc1b=%u\n",
        p->path_id, p->pixel_x, p->pixel_y,
        p->realisation_idx, p->steps_taken,
        p->rwalk.hit_3d.prim.prim_id, SPLIT3(p->rwalk.vtx.P),
        p->rwalk.hit_3d.normal[0], p->rwalk.hit_3d.normal[1],
        p->rwalk.hit_3d.normal[2], (int)p->rwalk.hit_side,
        p->locals.bnd_ss.enc_ids[SDIS_FRONT],
        p->locals.bnd_ss.enc_ids[SDIS_BACK],
        p->locals.bnd_ss.multi_frt, p->locals.bnd_ss.multi_bck,
        (double)p->locals.bnd_ss.reinject_dst_frt,
        (double)p->locals.bnd_ss.reinject_dst_bck,
        p->locals.bnd_ss.delta_boundary_frt,
        p->locals.bnd_ss.delta_boundary_bck,
        p->locals.bnd_ss.enc0_frt, p->locals.bnd_ss.enc1_frt,
        p->locals.bnd_ss.enc0_bck, p->locals.bnd_ss.enc1_bck);
      res = RES_BAD_OP_IRRECOVERABLE;
      goto error;
    }
    /* Move position away from primitive boundaries before retry
     * (mirrors original find_reinjection_ray inner loop logic). */
    move_away_primitive_boundaries_3d(&p->rwalk.hit_3d,
      MMAX(p->locals.bnd_ss.delta_boundary_frt,
           p->locals.bnd_ss.delta_boundary_bck),
      p->locals.bnd_ss.rwalk_pos_backup);
    p->locals.bnd_ss.position_was_moved = 1;

    /* Restore (possibly moved) position and retry */
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

    setup_ss_reinject_rays(p, hot);
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
    p->locals.bnd_ss.enc_side = 0; /* front */
    step_enc_query_emit(p, hot, enc, pos, PATH_BND_SS_REINJECT_ENC);
    return RES_OK;
  }

  if(!p->locals.bnd_ss.multi_bck && bck_valid
  && S3D_HIT_NONE(&p->locals.bnd_ss.reinject_hit_bck)) {
    /* Need to verify enclosure at back reinjection point */
    double pos[3];
    d3_set(pos, p->rwalk.vtx.P);
    move_pos_3d(pos, p->locals.bnd_ss.reinject_dir_bck,
                p->locals.bnd_ss.reinject_dst_bck);
    p->locals.bnd_ss.enc_side = 1; /* back */
    step_enc_query_emit(p, hot, enc, pos, PATH_BND_SS_REINJECT_ENC);
    return RES_OK;
  }

  /* All enclosures resolved -- go to decide */
  hot->phase = (uint8_t)PATH_BND_SS_REINJECT_DECIDE;
  hot->needs_ray = 0;
  return RES_OK;

error:
  hot->phase = (uint8_t)PATH_DONE;
  hot->active = 0;
  p->done_reason = -1;
  return res;
}

/* --- PATH_BND_SS_REINJECT_ENC: ENC query result for reinjection verify ---- */
LOCAL_SYM res_T
step_bnd_ss_reinject_enc_result(struct path_state* p, struct path_hot* hot,
                               struct sdis_scene* scn,
                               struct path_enc_data* enc)
{
  unsigned enc_id;
  unsigned target_enc_id;
  res_T res = RES_OK;

  ASSERT(p && scn);

  enc_id = enc->resolved_enc_id;

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
      p->locals.bnd_ss.enc_side = 1; /* back */
      step_enc_query_emit(p, hot, enc, pos, PATH_BND_SS_REINJECT_ENC);
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
        hot->phase = (uint8_t)PATH_DONE;
        hot->active = 0;
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

      setup_ss_reinject_rays(p, hot);
      return RES_OK;
    }
  }

  hot->phase = (uint8_t)PATH_BND_SS_REINJECT_DECIDE;
  hot->needs_ray = 0;
  return RES_OK;
}

/* --- PATH_BND_SS_REINJECT_DECIDE: probability choice + solid_reinjection -- */
LOCAL_SYM res_T
step_bnd_ss_reinject_decide(struct path_state* p, struct path_hot* hot, struct sdis_scene* scn)
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

  /* Compute probability (mirrors solid_solid_boundary_path_3d).
   *
   * Guard against NaN: when one side is invalid (reinject_dst == 0 or
   * lambda == 0), the ratio lambda/dst produces 0/0 = NaN which poisons
   * the probability and causes the wrong side to be chosen.
   * Short-circuit to the valid side in such cases. */
  {
    const int frt_ok = p->locals.bnd_ss.reinject_dst_frt > 0
                    && p->locals.bnd_ss.lambda_frt > 0;
    const int bck_ok = p->locals.bnd_ss.reinject_dst_bck > 0
                    && p->locals.bnd_ss.lambda_bck > 0;

    if(frt_ok && bck_ok) {
      /* Both sides physically valid — normal probability */
      if(p->locals.bnd_ss.tcr == 0) {
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
          proba = 0.5;
          break;
        }
      }
    } else if(frt_ok) {
      proba = 1.0; /* only front is valid */
    } else if(bck_ok) {
      proba = 0.0; /* only back is valid */
    } else {
      /* Neither side has lambda > 0 AND dst > 0.
       * Force the side that at least has dst > 0
       * (probability step will pass through solid_reinjection
       * which will fail gracefully if lambda is truly 0). */
      proba = p->locals.bnd_ss.reinject_dst_frt > 0 ? 1.0 : 0.0;
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
    hot->phase = (uint8_t)PATH_DONE;
    hot->active = 0;
    p->done_reason = 4; /* time rewind / temperature found */
    goto exit;
  }

  /* solid_reinjection sets T.func to indicate next step */
  if(p->T.func == conductive_path_3d) {
    p->ds_initialized = 0; /* reset for fresh conductive entry */
    p->locals.cnd_wos.wos_initialized = 0; /* union was bnd_ss — clear */
    hot->phase = (uint8_t)PATH_COUPLED_CONDUCTIVE;
  } else if(p->T.func == boundary_path_3d) {
    hot->phase = (uint8_t)PATH_COUPLED_BOUNDARY;
  } else {
    FATAL("wavefront M3: unexpected T.func after solid_reinjection\n");
  }
  hot->needs_ray = 0;

exit:
  return RES_OK;
error:
  hot->phase = (uint8_t)PATH_DONE;
  hot->active = 0;
  p->done_reason = -1;
  return res;
}

