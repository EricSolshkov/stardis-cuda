/* Copyright (C) 2016-2025 |MÃ©so|Star> (contact@meso-star.com)
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


/* Wavefront steps: external net flux (M7).  Split from sdis_wf_steps.c. */

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
 * B-4 M7: External net flux batch state machine
 *
 * Replaces the synchronous handle_external_net_flux_3d() call with a sub-state
 * machine that emits shadow rays and diffuse bounce rays through the wavefront
 * batch trace pipeline.
 *
 * State flow:
 *   EXT_CHECK â†?(shadow)   EXT_DIRECT_TRACE â†?EXT_DIRECT_RESULT
 *             â†?(diffuse)  EXT_DIFFUSE_TRACE â†?EXT_DIFFUSE_RESULT
 *                          â†?(reflect) EXT_DIFFUSE_SHADOW_TRACE
 *                                      â†?EXT_DIFFUSE_SHADOW_RESULT â†?loop
 *                          â†?(miss/absorb) EXT_FINALIZE
 *             â†?(no flux)  return_state (bypass)
 *
 * See CPU reference: sdis_heat_path_boundary_Xd_handle_external_net_flux.h
 ******************************************************************************/

/* --- PATH_BND_EXT_CHECK: initialise ext flux, decide shadow ray ---------- */
LOCAL_SYM res_T
step_bnd_ext_check(struct path_state* p, struct path_hot* hot,
                   struct sdis_scene* scn,
                   struct path_ext_data* ext)
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
  ext->flux_direct = 0;
  ext->flux_diffuse_reflected = 0;
  ext->flux_scattered = 0;
  ext->scattered_dir[0] = 0;
  ext->scattered_dir[1] = 0;
  ext->scattered_dir[2] = 0;
  ext->nbounces = 0;

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
  ext->enc_id_fluid = enc_ids[frag.side];

  /* Check if external flux handling is needed */
  handle_flux = interface_side_is_external_flux_handled(interf, &frag);
  handle_flux = handle_flux && (scn->source != NULL);
  if(!handle_flux) {
    /* No external flux â†?skip directly to caller */
    hot->phase = (uint8_t)ext->return_state;
    hot->needs_ray = 0;
    goto exit;
  }

  /* Check emissivity */
  src_id = sdis_source_get_id(scn->source);
  ext->emissivity =
      interface_side_get_emissivity(interf, src_id, &frag);
  res = interface_side_check_emissivity(scn->dev, ext->emissivity,
                                        frag.P, frag.time);
  if(res != RES_OK) goto error;

  if(ext->emissivity == 0) {
    hot->phase = (uint8_t)ext->return_state;
    hot->needs_ray = 0;
    goto exit;
  }

  /* Get source properties */
  res = source_get_props(scn->source, frag.time, &ext->src_props);
  if(res != RES_OK) goto error;

  /* Sample a direction toward the source */
  res = source_sample(scn->source, &ext->src_props, p->rng,
                      frag.P, &ext->src_sample);
  if(res != RES_OK) goto error;

  /* Save fragment info for later */
  ext->frag_time = frag.time;
  d3_set(ext->frag_P, frag.P);

  /* Compute outward normal (toward fluid) */
  d3_set(ext->N, frag.Ng);
  if(frag.side == SDIS_BACK) {
    d3_minus(ext->N, ext->N);
  }

  /* Compute sum_h for denominator */
  ext->sum_h = p->locals.bnd_sf.h_cond
                    + p->locals.bnd_sf.h_conv
                    + p->locals.bnd_sf.h_radi_hat;

  /* Save green_path handle */
  ext->green_path = p->ctx.green_path;

  /* cos_theta for direct contribution check */
  cos_theta = d3_dot(ext->N, ext->src_sample.dir);
  ext->cos_theta = cos_theta;

  if(cos_theta > 0) {
    /* Source is above the surface â†?emit shadow ray to check occlusion */
    float pos_f[3];
    f3_set_d3(pos_f, ext->frag_P);

    ext->pos[0] = pos_f[0];
    ext->pos[1] = pos_f[1];
    ext->pos[2] = pos_f[2];

    p->ray_req.origin[0] = pos_f[0];
    p->ray_req.origin[1] = pos_f[1];
    p->ray_req.origin[2] = pos_f[2];
    p->ray_req.direction[0] = (float)ext->src_sample.dir[0];
    p->ray_req.direction[1] = (float)ext->src_sample.dir[1];
    p->ray_req.direction[2] = (float)ext->src_sample.dir[2];
    p->ray_req.range[0] = 0.0f;
    p->ray_req.range[1] = (float)ext->src_sample.dst;
    p->ray_req.ray_count = 1;

    /* Filter: skip self-intersection with current hit */
    p->filter_data_storage = HIT_FILTER_DATA_NULL;
    p->filter_data_storage.hit_3d = p->rwalk.hit_3d;
    p->filter_data_storage.epsilon = 1.e-6;
    p->filter_data_storage.scn = scn;
    p->filter_data_storage.enc_id = ext->enc_id_fluid;

    hot->ray_bucket = (uint8_t)RAY_BUCKET_SHADOW;
    hot->ray_count_ext = (uint8_t)1;
    hot->needs_ray = 1;
    hot->phase = (uint8_t)PATH_BND_EXT_DIRECT_TRACE;
  } else {
    /* Source behind surface â†?skip direct contribution, go to diffuse */
    ext->flux_direct = 0;

    /* Set up initial diffuse bounce ray */
    {
      float pos_f[3], dir_f[3];
      double dir_d[3];

      f3_set_d3(pos_f, ext->frag_P);

      /* Cosine-weighted hemisphere sample for diffuse */
      ssp_ran_hemisphere_cos(p->rng, ext->N, dir_d, NULL);
      dir_f[0] = (float)dir_d[0];
      dir_f[1] = (float)dir_d[1];
      dir_f[2] = (float)dir_d[2];

      ext->pos[0] = pos_f[0];
      ext->pos[1] = pos_f[1];
      ext->pos[2] = pos_f[2];
      ext->dir[0] = dir_f[0];
      ext->dir[1] = dir_f[1];
      ext->dir[2] = dir_f[2];
      ext->hit = p->rwalk.hit_3d;

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
      p->filter_data_storage.enc_id = ext->enc_id_fluid;

      hot->ray_bucket = (uint8_t)RAY_BUCKET_RADIATIVE;
      hot->ray_count_ext = (uint8_t)1;
      hot->needs_ray = 1;
      hot->phase = (uint8_t)PATH_BND_EXT_DIFFUSE_TRACE;
    }
  }

exit:
  return res;
error:
  hot->phase = (uint8_t)PATH_DONE;
  hot->active = 0;
  p->done_reason = -1;
  goto exit;
}

/* --- PATH_BND_EXT_DIRECT_RESULT: process shadow ray result --------------- */
LOCAL_SYM res_T
step_bnd_ext_direct_result(struct path_state* p, struct path_hot* hot,
                           struct sdis_scene* scn,
                           const struct s3d_hit* hit,
                           struct path_ext_data* ext)
{
  res_T res = RES_OK;
  double dir_d[3];
  float  pos_f[3], dir_f[3];

  ASSERT(p && scn && hit);

  /* Shadow ray: if we hit something, source is occluded â†?direct = 0 */
  if(!S3D_HIT_NONE(hit)) {
    ext->flux_direct = 0;
  } else {
    /* Source visible â†?compute direct contribution */
    double Ld = ext->src_sample.radiance_term; /* [W/m^2/sr] */
    ext->flux_direct =
        ext->cos_theta * Ld / ext->src_sample.pdf; /* [W/m^2] */
  }

  /* Now set up the first diffuse bounce ray (same as CPU's
   * compute_incident_diffuse_flux entry) */
  f3_set_d3(pos_f, ext->frag_P);

  /* Cosine-weighted hemisphere sample for diffuse bounce */
  ssp_ran_hemisphere_cos(p->rng, ext->N, dir_d, NULL);
  dir_f[0] = (float)dir_d[0];
  dir_f[1] = (float)dir_d[1];
  dir_f[2] = (float)dir_d[2];

  ext->pos[0] = pos_f[0];
  ext->pos[1] = pos_f[1];
  ext->pos[2] = pos_f[2];
  ext->dir[0] = dir_f[0];
  ext->dir[1] = dir_f[1];
  ext->dir[2] = dir_f[2];
  ext->hit = p->rwalk.hit_3d;

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
  p->filter_data_storage.enc_id = ext->enc_id_fluid;

  hot->ray_bucket = (uint8_t)RAY_BUCKET_RADIATIVE;
  hot->ray_count_ext = (uint8_t)1;
  hot->needs_ray = 1;
  hot->phase = (uint8_t)PATH_BND_EXT_DIFFUSE_TRACE;

  return res;
}

/* --- PATH_BND_EXT_DIFFUSE_RESULT: process diffuse bounce ray result ------ */
LOCAL_SYM res_T
step_bnd_ext_diffuse_result(struct path_state* p, struct path_hot* hot,
                            struct sdis_scene* scn,
                            const struct s3d_hit* hit,
                            struct path_ext_data* ext)
{
  res_T res = RES_OK;

  ASSERT(p && scn && hit);

  /* --- Miss: ray escaped to environment â†?scattered flux = PI --- */
  if(S3D_HIT_NONE(hit)) {
    ext->flux_scattered = PI;
    ext->scattered_dir[0] = ext->dir[0];
    ext->scattered_dir[1] = ext->dir[1];
    ext->scattered_dir[2] = ext->dir[2];

    hot->phase = (uint8_t)PATH_BND_EXT_FINALIZE;
    hot->needs_ray = 0;
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
    ext->hit = *hit;

    /* Compute new position from ray origin + t * direction */
    d3_set_f3(dir_d, ext->dir);
    d3_normalize(dir_d, dir_d);
    d3_set_f3(pos_d, ext->pos);
    {
      double vec[3];
      d3_add(pos_d, pos_d, d3_muld(vec, dir_d, hit->distance));
    }

    /* Update position in ext_flux for next bounce */
    ext->pos[0] = (float)pos_d[0];
    ext->pos[1] = (float)pos_d[1];
    ext->pos[2] = (float)pos_d[2];

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
        vtx.time = ext->frag_time;
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
        /* Absorbed â†?diffuse bounce terminates with no further contribution */
        hot->phase = (uint8_t)PATH_BND_EXT_FINALIZE;
        hot->needs_ray = 0;
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
        res = source_trace_to(scn->source, &ext->src_props,
                              pos_d, bounce.dir, &samp);
        if(res != RES_OK) goto error;

        if(!SOURCE_SAMPLE_NONE(&samp)) {
          /* Emit shadow ray along specular direction to check occlusion */
          ext->dir[0] = (float)bounce.dir[0];
          ext->dir[1] = (float)bounce.dir[1];
          ext->dir[2] = (float)bounce.dir[2];

          /* Store the source sample radiance_term for shadow result.
           * Mark cos_theta <= 0 so that diffuse_shadow_result uses L = Ld
           * (specular contribution) rather than L = Ld*cos/(PI*pdf). */
          ext->src_sample = samp;
          ext->cos_theta = -1;

          p->ray_req.origin[0] = ext->pos[0];
          p->ray_req.origin[1] = ext->pos[1];
          p->ray_req.origin[2] = ext->pos[2];
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
          p->filter_data_storage.enc_id = ext->enc_id_fluid;

          hot->ray_bucket = (uint8_t)RAY_BUCKET_SHADOW;
          hot->ray_count_ext = (uint8_t)1;
          hot->needs_ray = 1;
          hot->phase = (uint8_t)PATH_BND_EXT_DIFFUSE_SHADOW_TRACE;

          /* Save the specular bounce direction as the next diffuse dir */
          /* (will be used after shadow result to continue bouncing) */
          ext->dir[0] = (float)bounce.dir[0];
          ext->dir[1] = (float)bounce.dir[1];
          ext->dir[2] = (float)bounce.dir[2];
          goto exit;
        }
        /* Source not reachable from specular bounce â†?no shadow ray needed,
         * just continue bouncing in the specular direction */
        ext->dir[0] = (float)bounce.dir[0];
        ext->dir[1] = (float)bounce.dir[1];
        ext->dir[2] = (float)bounce.dir[2];

      } else {
        /* Diffuse bounce: sample a direction toward the source */
        struct source_sample samp = SOURCE_SAMPLE_NULL;
        double cos_theta_bounce;

        ASSERT(bounce.cpnt == BRDF_DIFFUSE);

        res = source_sample(scn->source, &ext->src_props, p->rng,
                            pos_d, &samp);
        if(res != RES_OK) goto error;

        cos_theta_bounce = d3_dot(samp.dir, N);
        if(cos_theta_bounce > 0) {
          /* Source above this surface â†?emit shadow ray */
          ext->src_sample = samp;
          ext->cos_theta = cos_theta_bounce;

          p->ray_req.origin[0] = ext->pos[0];
          p->ray_req.origin[1] = ext->pos[1];
          p->ray_req.origin[2] = ext->pos[2];
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
          p->filter_data_storage.enc_id = ext->enc_id_fluid;

          hot->ray_bucket = (uint8_t)RAY_BUCKET_SHADOW;
          hot->ray_count_ext = (uint8_t)1;
          hot->needs_ray = 1;
          hot->phase = (uint8_t)PATH_BND_EXT_DIFFUSE_SHADOW_TRACE;

          /* Save bounce direction for next diffuse bounce */
          ext->dir[0] = (float)bounce.dir[0];
          ext->dir[1] = (float)bounce.dir[1];
          ext->dir[2] = (float)bounce.dir[2];
          goto exit;
        }
        /* Source behind this surface â†?L = 0, no shadow ray needed */
        /* Just continue bouncing */
        ext->dir[0] = (float)bounce.dir[0];
        ext->dir[1] = (float)bounce.dir[1];
        ext->dir[2] = (float)bounce.dir[2];
      }

      /* No shadow ray needed at this bounce â†?emit next diffuse bounce ray */
      ext->nbounces++;

      p->ray_req.origin[0] = ext->pos[0];
      p->ray_req.origin[1] = ext->pos[1];
      p->ray_req.origin[2] = ext->pos[2];
      p->ray_req.direction[0] = ext->dir[0];
      p->ray_req.direction[1] = ext->dir[1];
      p->ray_req.direction[2] = ext->dir[2];
      p->ray_req.range[0] = 0.0f;
      p->ray_req.range[1] = FLT_MAX;
      p->ray_req.ray_count = 1;

      p->filter_data_storage = HIT_FILTER_DATA_NULL;
      p->filter_data_storage.hit_3d = *hit;
      p->filter_data_storage.epsilon = 1.e-6;
      p->filter_data_storage.scn = scn;
      p->filter_data_storage.enc_id = ext->enc_id_fluid;

      hot->ray_bucket = (uint8_t)RAY_BUCKET_RADIATIVE;
      hot->ray_count_ext = (uint8_t)1;
      hot->needs_ray = 1;
      hot->phase = (uint8_t)PATH_BND_EXT_DIFFUSE_TRACE;
    }
  }

exit:
  return res;
error:
  hot->phase = (uint8_t)PATH_DONE;
  hot->active = 0;
  p->done_reason = -1;
  goto exit;
}

/* --- PATH_BND_EXT_DIFFUSE_SHADOW_RESULT: shadow ray at bounce position --- */
LOCAL_SYM res_T
step_bnd_ext_diffuse_shadow_result(struct path_state* p,
                                   struct path_hot* hot,
                                   struct sdis_scene* scn,
                                   const struct s3d_hit* hit,
                                   struct path_ext_data* ext)
{
  res_T res = RES_OK;

  ASSERT(p && scn && hit);

  /* Process shadow ray result at bounce position */
  if(S3D_HIT_NONE(hit)) {
    /* Not occluded â†?accumulate direct contribution at this bounce.
     * For specular bounces, L = Ld (radiance_term).
     * For diffuse bounces, L = Ld * cos_theta / (PI * pdf). */
    double Ld = ext->src_sample.radiance_term;
    double L;

    if(ext->cos_theta > 0
    && ext->src_sample.pdf > 0) {
      /* Diffuse bounce: L = Ld * cos / (PI * pdf) */
      L = Ld * ext->cos_theta
        / (PI * ext->src_sample.pdf);
    } else {
      /* Specular bounce: L = Ld */
      L = Ld;
    }
    ext->flux_diffuse_reflected += L; /* [W/m^2/sr] */
  }
  /* If occluded, no contribution */

  /* Continue with next diffuse bounce ray */
  ext->nbounces++;

  p->ray_req.origin[0] = ext->pos[0];
  p->ray_req.origin[1] = ext->pos[1];
  p->ray_req.origin[2] = ext->pos[2];
  p->ray_req.direction[0] = ext->dir[0];
  p->ray_req.direction[1] = ext->dir[1];
  p->ray_req.direction[2] = ext->dir[2];
  p->ray_req.range[0] = 0.0f;
  p->ray_req.range[1] = FLT_MAX;
  p->ray_req.ray_count = 1;

  p->filter_data_storage = HIT_FILTER_DATA_NULL;
  p->filter_data_storage.hit_3d = ext->hit;
  p->filter_data_storage.epsilon = 1.e-6;
  p->filter_data_storage.scn = scn;
  p->filter_data_storage.enc_id = ext->enc_id_fluid;

  hot->ray_bucket = (uint8_t)RAY_BUCKET_RADIATIVE;
  hot->ray_count_ext = (uint8_t)1;
  hot->needs_ray = 1;
  hot->phase = (uint8_t)PATH_BND_EXT_DIFFUSE_TRACE;

  return res;
}

/* --- PATH_BND_EXT_FINALIZE: sum flux, apply to T, return to caller ------- */
LOCAL_SYM res_T
step_bnd_ext_finalize(struct path_state* p, struct path_hot* hot,
                      struct sdis_scene* scn,
                      struct path_ext_data* ext)
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
  ext->flux_diffuse_reflected *= PI; /* [W/m^2] */

  /* incident_flux = direct + diffuse_reflected (excludes scattered) */
  incident_flux = ext->flux_direct
                + ext->flux_diffuse_reflected;

  /* net_flux (relative to source power) */
  net_flux = incident_flux * ext->emissivity;

  /* net_flux_sc (relative to source diffuse radiance) */
  net_flux_sc = ext->flux_scattered * ext->emissivity;

  /* Green function terms */
  green.term_wrt_power = net_flux / ext->sum_h;
  green.term_wrt_diffuse_radiance = net_flux_sc / ext->sum_h;
  green.time = ext->frag_time;
  green.dir[0] = ext->scattered_dir[0];
  green.dir[1] = ext->scattered_dir[1];
  green.dir[2] = ext->scattered_dir[2];

  /* Apply to temperature */
  p->T.value += green.term_wrt_power * ext->src_props.power;
  if(green.term_wrt_diffuse_radiance) {
    p->T.value +=
        green.term_wrt_diffuse_radiance
      * source_get_diffuse_radiance(scn->source, green.time, green.dir);
  }

  /* Register in green path if available */
  if(ext->green_path) {
    res = green_path_add_external_flux_terms(ext->green_path, &green);
    if(res != RES_OK) goto error;
  }

  /* Return to caller */
  hot->phase = (uint8_t)ext->return_state;
  hot->needs_ray = 0;

exit:
  return res;
error:
  hot->phase = (uint8_t)PATH_DONE;
  hot->active = 0;
  p->done_reason = -1;
  goto exit;
}

