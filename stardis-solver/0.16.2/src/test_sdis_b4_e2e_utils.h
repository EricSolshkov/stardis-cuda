/* Copyright (C) 2016-2025 |Meso|Star> (contact@meso-star.com)
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

/* Phase B-4: End-to-end (delayed) test shared utilities.
 *
 * Provides:
 *   - Shader parameter structs and callbacks (solid, fluid, interface, radenv,
 *     source)
 *   - Medium / interface / radenv / source factory functions
 *   - Camera factory
 *   - solve_camera runner (depth-first vs wavefront)
 *   - Per-pixel statistical-compatibility image comparison
 *   - Analytical-temperature comparison helper
 *
 * All end-to-end B-4 tests (#include this file) link sdis_obj (OBJECT lib)
 * so DLL-export issues do not arise.
 */

#ifndef TEST_SDIS_B4_E2E_UTILS_H
#define TEST_SDIS_B4_E2E_UTILS_H

#include "sdis.h"
#include "test_sdis_utils.h"

#include <rsys/clock_time.h>
#include <rsys_math.h>
#include <star/ssp.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <float.h>

/* ========================================================================== */
/* Configuration defaults                                                     */
/* ========================================================================== */
#define E2E_IMG_WIDTH   32
#define E2E_IMG_HEIGHT  32
#define E2E_SPP         64
#define E2E_TOL_SIGMA   4.0
#define E2E_PASS_RATE   0.95

/* ========================================================================== */
/* Shader parameter structures                                                */
/* ========================================================================== */
struct e2e_fluid_params {
  double temperature;
  double cp;
  double rho;
};

struct e2e_solid_params {
  double temperature; /* NaN = unknown */
  double lambda;
  double cp;
  double rho;
  double delta;
  double power;       /* DBL_MAX = none */
};

struct e2e_interf_params {
  double hc;
  double epsilon;
  double specular;
  double reference_temperature;
};

struct e2e_source_params {
  double position[3];
  double power;
  double radius;
};

/* ========================================================================== */
/* Shader callbacks -- fluid                                                  */
/* ========================================================================== */
static double
e2e_fluid_get_temperature
  (const struct sdis_rwalk_vertex* vtx, struct sdis_data* data)
{
  const struct e2e_fluid_params* f = sdis_data_get(data);
  (void)vtx;
  return f->temperature;
}
static double
e2e_fluid_get_cp
  (const struct sdis_rwalk_vertex* vtx, struct sdis_data* data)
{
  const struct e2e_fluid_params* f = sdis_data_get(data);
  (void)vtx;
  return f->cp;
}
static double
e2e_fluid_get_rho
  (const struct sdis_rwalk_vertex* vtx, struct sdis_data* data)
{
  const struct e2e_fluid_params* f = sdis_data_get(data);
  (void)vtx;
  return f->rho;
}

/* ========================================================================== */
/* Shader callbacks -- solid                                                  */
/* ========================================================================== */
static double
e2e_solid_get_temperature
  (const struct sdis_rwalk_vertex* vtx, struct sdis_data* data)
{
  const struct e2e_solid_params* s = sdis_data_get(data);
  (void)vtx;
  return s->temperature;
}
static double
e2e_solid_get_lambda
  (const struct sdis_rwalk_vertex* vtx, struct sdis_data* data)
{
  const struct e2e_solid_params* s = sdis_data_get(data);
  (void)vtx;
  return s->lambda;
}
static double
e2e_solid_get_cp
  (const struct sdis_rwalk_vertex* vtx, struct sdis_data* data)
{
  const struct e2e_solid_params* s = sdis_data_get(data);
  (void)vtx;
  return s->cp;
}
static double
e2e_solid_get_rho
  (const struct sdis_rwalk_vertex* vtx, struct sdis_data* data)
{
  const struct e2e_solid_params* s = sdis_data_get(data);
  (void)vtx;
  return s->rho;
}
static double
e2e_solid_get_delta
  (const struct sdis_rwalk_vertex* vtx, struct sdis_data* data)
{
  const struct e2e_solid_params* s = sdis_data_get(data);
  (void)vtx;
  return s->delta;
}
static double
e2e_solid_get_power
  (const struct sdis_rwalk_vertex* vtx, struct sdis_data* data)
{
  const struct e2e_solid_params* s = sdis_data_get(data);
  (void)vtx;
  return s->power;
}

/* ========================================================================== */
/* Shader callbacks -- interface                                              */
/* ========================================================================== */
static double
e2e_interf_get_hc
  (const struct sdis_interface_fragment* frag, struct sdis_data* data)
{
  const struct e2e_interf_params* p = sdis_data_get(data);
  (void)frag;
  return p->hc;
}
static double
e2e_interf_get_epsilon
  (const struct sdis_interface_fragment* frag,
   const unsigned source_id,
   struct sdis_data* data)
{
  const struct e2e_interf_params* p = sdis_data_get(data);
  (void)frag; (void)source_id;
  return p->epsilon;
}
static double
e2e_interf_get_specular
  (const struct sdis_interface_fragment* frag,
   const unsigned source_id,
   struct sdis_data* data)
{
  const struct e2e_interf_params* p = sdis_data_get(data);
  (void)frag; (void)source_id;
  return p->specular;
}
static double
e2e_interf_get_temperature
  (const struct sdis_interface_fragment* frag, struct sdis_data* data)
{
  const struct e2e_interf_params* p = sdis_data_get(data);
  (void)frag;
  return p->reference_temperature;
}

/* ========================================================================== */
/* Shader callbacks -- radiative environment                                  */
/* ========================================================================== */
static double e2e_radenv_temp = 300.0;
static double e2e_radenv_ref  = 300.0;

static double
e2e_radenv_get_temperature
  (const struct sdis_radiative_ray* ray, struct sdis_data* data)
{
  (void)ray; (void)data;
  return e2e_radenv_temp;
}
static double
e2e_radenv_get_reference
  (const struct sdis_radiative_ray* ray, struct sdis_data* data)
{
  (void)ray; (void)data;
  return e2e_radenv_ref;
}

/* ========================================================================== */
/* Shader callbacks -- spherical source                                       */
/* ========================================================================== */
static double e2e_src_pos[3] = {0, 0, 2};
static double e2e_src_power  = 100.0;

static void
e2e_source_get_position
  (const double time, double pos[3], struct sdis_data* data)
{
  (void)time; (void)data;
  pos[0] = e2e_src_pos[0];
  pos[1] = e2e_src_pos[1];
  pos[2] = e2e_src_pos[2];
}
static double
e2e_source_get_power
  (const double time, struct sdis_data* data)
{
  (void)time; (void)data;
  return e2e_src_power;
}

/* ========================================================================== */
/* Factory helpers                                                            */
/* ========================================================================== */
static struct sdis_medium*
e2e_create_fluid(struct sdis_device* dev, double temp, double cp, double rho)
{
  struct sdis_fluid_shader shader = DUMMY_FLUID_SHADER;
  struct sdis_data* data = NULL;
  struct sdis_medium* fluid = NULL;
  struct e2e_fluid_params params;

  params.temperature = temp;
  params.cp = cp;
  params.rho = rho;

  OK(sdis_data_create(dev, sizeof(params), ALIGNOF(params), NULL, &data));
  memcpy(sdis_data_get(data), &params, sizeof(params));

  shader.temperature = e2e_fluid_get_temperature;
  shader.calorific_capacity = e2e_fluid_get_cp;
  shader.volumic_mass = e2e_fluid_get_rho;

  OK(sdis_fluid_create(dev, &shader, data, &fluid));
  OK(sdis_data_ref_put(data));
  return fluid;
}

static struct sdis_medium*
e2e_create_solid
  (struct sdis_device* dev,
   double temperature, double lambda, double cp,
   double rho, double delta, double power)
{
  struct sdis_solid_shader shader = DUMMY_SOLID_SHADER;
  struct sdis_data* data = NULL;
  struct sdis_medium* solid = NULL;
  struct e2e_solid_params params;

  params.temperature = temperature;
  params.lambda = lambda;
  params.cp = cp;
  params.rho = rho;
  params.delta = delta;
  params.power = power;

  OK(sdis_data_create(dev, sizeof(params), ALIGNOF(params), NULL, &data));
  memcpy(sdis_data_get(data), &params, sizeof(params));

  shader.thermal_conductivity = e2e_solid_get_lambda;
  shader.calorific_capacity = e2e_solid_get_cp;
  shader.volumic_mass = e2e_solid_get_rho;
  shader.delta = e2e_solid_get_delta;
  shader.temperature = e2e_solid_get_temperature;
  shader.volumic_power = e2e_solid_get_power;

  OK(sdis_solid_create(dev, &shader, data, &solid));
  OK(sdis_data_ref_put(data));
  return solid;
}

static struct sdis_interface*
e2e_create_interface
  (struct sdis_device* dev,
   struct sdis_medium* front,
   struct sdis_medium* back,
   double hc, double epsilon, double specular, double ref_temp)
{
  struct sdis_interface_shader shader = SDIS_INTERFACE_SHADER_NULL;
  struct sdis_data* data = NULL;
  struct sdis_interface* interf = NULL;
  struct e2e_interf_params params;

  params.hc = hc;
  params.epsilon = epsilon;
  params.specular = specular;
  params.reference_temperature = ref_temp;

  OK(sdis_data_create(dev, sizeof(params), ALIGNOF(params), NULL, &data));
  memcpy(sdis_data_get(data), &params, sizeof(params));

  shader.convection_coef = e2e_interf_get_hc;
  if(sdis_medium_get_type(front) == SDIS_FLUID) {
    shader.front.emissivity = e2e_interf_get_epsilon;
    shader.front.specular_fraction = e2e_interf_get_specular;
    shader.front.reference_temperature = e2e_interf_get_temperature;
  }
  if(sdis_medium_get_type(back) == SDIS_FLUID) {
    shader.back.emissivity = e2e_interf_get_epsilon;
    shader.back.specular_fraction = e2e_interf_get_specular;
    shader.back.reference_temperature = e2e_interf_get_temperature;
  }

  OK(sdis_interface_create(dev, front, back, &shader, data, &interf));
  OK(sdis_data_ref_put(data));
  return interf;
}

static struct sdis_radiative_env*
e2e_create_radenv(struct sdis_device* dev, double temp, double ref)
{
  struct sdis_radiative_env_shader shader = SDIS_RADIATIVE_ENV_SHADER_NULL;
  struct sdis_radiative_env* radenv = NULL;

  e2e_radenv_temp = temp;
  e2e_radenv_ref = ref;
  shader.temperature = e2e_radenv_get_temperature;
  shader.reference_temperature = e2e_radenv_get_reference;
  OK(sdis_radiative_env_create(dev, &shader, NULL, &radenv));
  return radenv;
}

static struct sdis_source*
e2e_create_point_source
  (struct sdis_device* dev,
   const double pos[3], double power, double radius)
{
  struct sdis_spherical_source_shader shader
    = SDIS_SPHERICAL_SOURCE_SHADER_NULL;
  struct sdis_source* src = NULL;

  e2e_src_pos[0] = pos[0];
  e2e_src_pos[1] = pos[1];
  e2e_src_pos[2] = pos[2];
  e2e_src_power = power;

  shader.position = e2e_source_get_position;
  shader.power = e2e_source_get_power;
  shader.radius = radius;

  OK(sdis_spherical_source_create(dev, &shader, NULL, &src));
  return src;
}

static struct sdis_camera*
e2e_create_camera(struct sdis_device* dev, size_t w, size_t h)
{
  const double pos[3] = {2, 2, 2};
  const double tgt[3] = {0.5, 0.5, 0.5};
  const double up[3]  = {0, 0, 1};
  struct sdis_camera* cam = NULL;

  OK(sdis_camera_create(dev, &cam));
  OK(sdis_camera_set_proj_ratio(cam, (double)w / (double)h));
  OK(sdis_camera_set_fov(cam, MDEG2RAD(60)));
  OK(sdis_camera_look_at(cam, pos, tgt, up));
  return cam;
}

/* ========================================================================== */
/* solve_camera runner -- wavefront vs depth-first                            */
/* ========================================================================== */
static res_T
e2e_run_solve_camera
  (struct sdis_scene* scn,
   struct sdis_camera* cam,
   int use_wavefront,
   size_t width, size_t height, size_t spp,
   size_t picard_order,
   enum sdis_diffusion_algorithm diff_algo,
   struct sdis_estimator_buffer** out_img,
   struct time* out_elapsed)
{
  struct sdis_solve_camera_args args = SDIS_SOLVE_CAMERA_ARGS_DEFAULT;
  struct time t0, t1;
  res_T res;

  args.cam = cam;
  args.time_range[0] = INF;
  args.time_range[1] = INF;
  args.image_definition[0] = width;
  args.image_definition[1] = height;
  args.spp = spp;
  args.picard_order = picard_order;
  args.diff_algo = diff_algo;

#ifdef _WIN32
  _putenv_s("STARDIS_WAVEFRONT",  use_wavefront ? "1" : "0");
#else
  setenv("STARDIS_WAVEFRONT",  use_wavefront ? "1" : "0", 1);
#endif

  time_current(&t0);
  res = sdis_solve_camera(scn, &args, out_img);
  time_current(&t1);

  if(out_elapsed) time_sub(out_elapsed, &t1, &t0);
  return res;
}

/* ========================================================================== */
/* Per-pixel statistical-compatibility comparison                             */
/* ========================================================================== */
static void
e2e_compare_images
  (const struct sdis_estimator_buffer* img_ref,
   const struct sdis_estimator_buffer* img_wf,
   double tol_sigma,
   double pass_rate_threshold,
   int* out_pass,
   size_t* out_compared,
   size_t* out_compatible)
{
  size_t def[2] = {0, 0};
  size_t x, y;
  size_t n_compared = 0, n_compat = 0;

  OK(sdis_estimator_buffer_get_definition(img_ref, def));

  for(y = 0; y < def[1]; y++) {
    for(x = 0; x < def[0]; x++) {
      const struct sdis_estimator* est_ref = NULL;
      const struct sdis_estimator* est_wf = NULL;
      struct sdis_mc mc_ref, mc_wf;
      size_t count_ref = 0, count_wf = 0;

      OK(sdis_estimator_buffer_at(img_ref, x, y, &est_ref));
      OK(sdis_estimator_buffer_at(img_wf,  x, y, &est_wf));

      /* Skip pixels with no valid realisations */
      if(sdis_estimator_get_realisation_count(est_ref, &count_ref) != RES_OK
         || count_ref == 0) continue;
      if(sdis_estimator_get_realisation_count(est_wf, &count_wf) != RES_OK
         || count_wf == 0) continue;

      if(sdis_estimator_get_temperature(est_ref, &mc_ref) != RES_OK) continue;
      if(sdis_estimator_get_temperature(est_wf,  &mc_wf)  != RES_OK) continue;

      n_compared++;
      {
        double se_combined = sqrt(mc_ref.SE * mc_ref.SE
                                  + mc_wf.SE * mc_wf.SE);
        double diff = fabs(mc_ref.E - mc_wf.E);
        if(se_combined < 1e-12 || diff <= tol_sigma * se_combined) {
          n_compat++;
        } else if(n_compared - n_compat <= 5) {
          fprintf(stderr,
            "  pixel (%lu,%lu): ref=%.6f wf=%.6f diff=%.2e se=%.2e "
            "(%.1f sigma)\n",
            (unsigned long)x, (unsigned long)y,
            mc_ref.E, mc_wf.E, diff, se_combined,
            se_combined > 0 ? diff / se_combined : 0);
        }
      }
    }
  }

  fprintf(stdout, "  Pixels compared: %lu, compatible: %lu (%.1f%%)\n",
    (unsigned long)n_compared, (unsigned long)n_compat,
    n_compared > 0 ? 100.0 * (double)n_compat / (double)n_compared : 0.0);

  if(out_compared) *out_compared = n_compared;
  if(out_compatible) *out_compatible = n_compat;

  *out_pass = (n_compared > 0
    && (double)n_compat / (double)n_compared >= pass_rate_threshold);
}

/* ========================================================================== */
/* Analytical temperature comparison                                          */
/* ========================================================================== */
/* Compare each pixel's mean temperature against an analytically-expected
 * temperature (function of pixel position).  Returns the fraction of pixels
 * whose mean temperature falls within `max_rel_err` of the expected value. */
typedef double (*e2e_analytic_temp_fn)(double x, double y);

static double
e2e_compare_analytic
  (const struct sdis_estimator_buffer* img,
   e2e_analytic_temp_fn expected_fn,
   double max_rel_err)
{
  size_t def[2] = {0, 0};
  size_t x, y;
  size_t n_compared = 0, n_ok = 0;

  OK(sdis_estimator_buffer_get_definition(img, def));

  for(y = 0; y < def[1]; y++) {
    for(x = 0; x < def[0]; x++) {
      const struct sdis_estimator* est = NULL;
      struct sdis_mc mc;
      double T_expected, rel;
      size_t cnt = 0;

      OK(sdis_estimator_buffer_at(img, x, y, &est));
      if(sdis_estimator_get_realisation_count(est, &cnt) != RES_OK
         || cnt == 0) continue;
      if(sdis_estimator_get_temperature(est, &mc) != RES_OK) continue;

      T_expected = expected_fn(
        ((double)x + 0.5) / (double)def[0],
        ((double)y + 0.5) / (double)def[1]);

      if(fabs(T_expected) < 1e-12) continue; /* skip zero-expected */
      n_compared++;

      rel = fabs(mc.E - T_expected) / fabs(T_expected);
      if(rel <= max_rel_err) n_ok++;
    }
  }

  return n_compared > 0 ? (double)n_ok / (double)n_compared : 0.0;
}

/* ========================================================================== */
/* Simple box scene builder (common unit-cube geometry, variable physics)     */
/* ========================================================================== */
struct e2e_box_scene {
  struct sdis_device*        dev;
  struct sdis_medium*        front_medium;
  struct sdis_medium*        back_medium;
  struct sdis_interface*     interf;
  struct sdis_interface*     interfaces[12];
  struct sdis_radiative_env* radenv;
  struct sdis_source*        source;
  struct sdis_scene*         scn;
  struct sdis_camera*        cam;
};

static void
e2e_box_scene_create
  (struct e2e_box_scene* bs,
   struct sdis_medium* front,
   struct sdis_medium* back,
   struct sdis_interface* interf,
   struct sdis_radiative_env* radenv,
   struct sdis_source* source,
   double t_range_lo, double t_range_hi,
   size_t img_w, size_t img_h)
{
  struct sdis_scene_create_args scn_args = SDIS_SCENE_CREATE_ARGS_DEFAULT;
  int i;

  memset(bs, 0, sizeof(*bs));
  bs->front_medium = front;
  bs->back_medium  = back;
  bs->interf       = interf;
  bs->radenv       = radenv;
  bs->source       = source;

  for(i = 0; i < 12; i++) bs->interfaces[i] = interf;

  /* Device */
  OK(sdis_device_create(&SDIS_DEVICE_CREATE_ARGS_DEFAULT, &bs->dev));

  /* Scene */
  scn_args.get_indices   = box_get_indices;
  scn_args.get_position  = box_get_position;
  scn_args.get_interface = box_get_interface;
  scn_args.nprimitives   = box_ntriangles;
  scn_args.nvertices     = box_nvertices;
  scn_args.t_range[0]    = t_range_lo;
  scn_args.t_range[1]    = t_range_hi;
  scn_args.radenv        = radenv;
  scn_args.source        = source;
  scn_args.context       = bs->interfaces;
  OK(sdis_scene_create(bs->dev, &scn_args, &bs->scn));

  /* Camera */
  bs->cam = e2e_create_camera(bs->dev, img_w, img_h);
}

static void
e2e_box_scene_destroy(struct e2e_box_scene* bs)
{
  if(bs->cam)    { OK(sdis_camera_ref_put(bs->cam));             bs->cam = NULL; }
  if(bs->scn)    { OK(sdis_scene_ref_put(bs->scn));              bs->scn = NULL; }
  if(bs->source) { OK(sdis_source_ref_put(bs->source));          bs->source = NULL; }
  if(bs->radenv) { OK(sdis_radiative_env_ref_put(bs->radenv));   bs->radenv = NULL; }
  if(bs->interf) { OK(sdis_interface_ref_put(bs->interf));       bs->interf = NULL; }
  if(bs->back_medium)  { OK(sdis_medium_ref_put(bs->back_medium));  bs->back_medium = NULL; }
  if(bs->front_medium) { OK(sdis_medium_ref_put(bs->front_medium)); bs->front_medium = NULL; }
  if(bs->dev)    { free_default_device(bs->dev);                 bs->dev = NULL; }
}

/* ========================================================================== */
/* Convenience: run wavefront vs depth-first comparison on a box scene.       */
/* Returns 1 on pass, 0 on fail.                                              */
/* ========================================================================== */
static int
e2e_wavefront_vs_depthfirst
  (struct e2e_box_scene* bs,
   size_t width, size_t height, size_t spp,
   size_t picard_order,
   enum sdis_diffusion_algorithm diff_algo,
   double tol_sigma,
   double pass_rate)
{
  struct sdis_estimator_buffer* img_ref = NULL;
  struct sdis_estimator_buffer* img_wf  = NULL;
  struct time t_ref, t_wf;
  int pass = 0;
  size_t n_cmp = 0, n_ok = 0;

  fprintf(stdout, "    Depth-first run ...\n");
  OK(e2e_run_solve_camera(bs->scn, bs->cam, 0,
    width, height, spp, picard_order, diff_algo, &img_ref, &t_ref));

  fprintf(stdout, "    Wavefront run ...\n");
  OK(e2e_run_solve_camera(bs->scn, bs->cam, 1,
    width, height, spp, picard_order, diff_algo, &img_wf, &t_wf));

  e2e_compare_images(img_ref, img_wf, tol_sigma, pass_rate,
                     &pass, &n_cmp, &n_ok);

  {
    int64_t usec_ref = time_val(&t_ref, TIME_USEC);
    int64_t usec_wf  = time_val(&t_wf,  TIME_USEC);
    if(usec_wf > 0) {
      fprintf(stdout, "    Speedup: %.2fx\n",
        (double)usec_ref / (double)usec_wf);
    }
  }

  OK(sdis_estimator_buffer_ref_put(img_ref));
  OK(sdis_estimator_buffer_ref_put(img_wf));
  return pass;
}

#endif /* TEST_SDIS_B4_E2E_UTILS_H */
