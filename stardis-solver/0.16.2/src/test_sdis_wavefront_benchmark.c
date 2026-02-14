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

/* Phase B-2 wavefront benchmark test.
 *
 * Compares the wavefront path-stepping solver (solve_tile_wavefront) against
 * the original depth-first solver (solve_tile) on a unit-cube scene with
 * fluid exterior / solid interior coupling.
 *
 * Reports:
 *   - Wall-clock time for each mode
 *   - Per-pixel mean temperature comparison (statistical compatibility)
 *   - Speedup ratio
 */

#include "sdis.h"
#include "test_sdis_utils.h"

#include <rsys/clock_time.h>
#include <rsys_math.h>
#include <star/ssp.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* -------------------------------------------------------------------------- */
/* Configuration                                                              */
/* -------------------------------------------------------------------------- */
#define BENCH_IMG_WIDTH   32
#define BENCH_IMG_HEIGHT  32
#define BENCH_SPP         64
#define BENCH_TOL_SIGMA   4.0  /* max deviations for statistical equivalence */

/* -------------------------------------------------------------------------- */
/* Medium parameter structures                                                */
/* -------------------------------------------------------------------------- */
struct bench_fluid {
  double temperature;
  double cp;
  double rho;
};

struct bench_solid {
  double temperature; /* SDIS_TEMPERATURE_UNKNOWN means unknown */
  double lambda;
  double cp;
  double rho;
  double delta;
  double power;
};

struct bench_interf {
  double hc;
  double epsilon;
  double specular;
  double reference_temperature;
};

/* -------------------------------------------------------------------------- */
/* Shader callbacks — fluid                                                   */
/* -------------------------------------------------------------------------- */
static double
bench_fluid_get_temperature
  (const struct sdis_rwalk_vertex* vtx, struct sdis_data* data)
{
  const struct bench_fluid* f = sdis_data_get(data);
  (void)vtx;
  return f->temperature;
}
static double
bench_fluid_get_cp
  (const struct sdis_rwalk_vertex* vtx, struct sdis_data* data)
{
  const struct bench_fluid* f = sdis_data_get(data);
  (void)vtx;
  return f->cp;
}
static double
bench_fluid_get_rho
  (const struct sdis_rwalk_vertex* vtx, struct sdis_data* data)
{
  const struct bench_fluid* f = sdis_data_get(data);
  (void)vtx;
  return f->rho;
}

/* -------------------------------------------------------------------------- */
/* Shader callbacks — solid                                                   */
/* -------------------------------------------------------------------------- */
static double
bench_solid_get_temperature
  (const struct sdis_rwalk_vertex* vtx, struct sdis_data* data)
{
  const struct bench_solid* s = sdis_data_get(data);
  (void)vtx;
  return s->temperature;
}
static double
bench_solid_get_lambda
  (const struct sdis_rwalk_vertex* vtx, struct sdis_data* data)
{
  const struct bench_solid* s = sdis_data_get(data);
  (void)vtx;
  return s->lambda;
}
static double
bench_solid_get_cp
  (const struct sdis_rwalk_vertex* vtx, struct sdis_data* data)
{
  const struct bench_solid* s = sdis_data_get(data);
  (void)vtx;
  return s->cp;
}
static double
bench_solid_get_rho
  (const struct sdis_rwalk_vertex* vtx, struct sdis_data* data)
{
  const struct bench_solid* s = sdis_data_get(data);
  (void)vtx;
  return s->rho;
}
static double
bench_solid_get_delta
  (const struct sdis_rwalk_vertex* vtx, struct sdis_data* data)
{
  const struct bench_solid* s = sdis_data_get(data);
  (void)vtx;
  return s->delta;
}
static double
bench_solid_get_power
  (const struct sdis_rwalk_vertex* vtx, struct sdis_data* data)
{
  const struct bench_solid* s = sdis_data_get(data);
  (void)vtx;
  return s->power;
}

/* -------------------------------------------------------------------------- */
/* Shader callbacks — interface                                               */
/* -------------------------------------------------------------------------- */
static double
bench_interf_get_hc
  (const struct sdis_interface_fragment* frag, struct sdis_data* data)
{
  const struct bench_interf* p = sdis_data_get(data);
  (void)frag;
  return p->hc;
}
static double
bench_interf_get_epsilon
  (const struct sdis_interface_fragment* frag,
   const unsigned source_id,
   struct sdis_data* data)
{
  const struct bench_interf* p = sdis_data_get(data);
  (void)frag; (void)source_id;
  return p->epsilon;
}
static double
bench_interf_get_specular
  (const struct sdis_interface_fragment* frag,
   const unsigned source_id,
   struct sdis_data* data)
{
  const struct bench_interf* p = sdis_data_get(data);
  (void)frag; (void)source_id;
  return p->specular;
}
static double
bench_interf_get_reference_temperature
  (const struct sdis_interface_fragment* frag, struct sdis_data* data)
{
  const struct bench_interf* p = sdis_data_get(data);
  (void)frag;
  return p->reference_temperature;
}

/* -------------------------------------------------------------------------- */
/* Shader callbacks — radiative environment                                   */
/* -------------------------------------------------------------------------- */
static double bench_radenv_temperature = 300.0;
static double bench_radenv_reference   = 300.0;

static double
bench_radenv_get_temperature
  (const struct sdis_radiative_ray* ray, struct sdis_data* data)
{
  (void)ray; (void)data;
  return bench_radenv_temperature;
}
static double
bench_radenv_get_reference
  (const struct sdis_radiative_ray* ray, struct sdis_data* data)
{
  (void)ray; (void)data;
  return bench_radenv_reference;
}

/* -------------------------------------------------------------------------- */
/* Scene creation                                                             */
/* -------------------------------------------------------------------------- */
static struct sdis_medium*
bench_create_fluid(struct sdis_device* dev, double temp)
{
  struct sdis_fluid_shader shader = DUMMY_FLUID_SHADER;
  struct sdis_data* data = NULL;
  struct sdis_medium* fluid = NULL;
  struct bench_fluid params;

  params.temperature = temp;
  params.cp = 1000.0;
  params.rho = 1.2;

  OK(sdis_data_create(dev, sizeof(params), ALIGNOF(params), NULL, &data));
  memcpy(sdis_data_get(data), &params, sizeof(params));

  shader.temperature = bench_fluid_get_temperature;
  shader.calorific_capacity = bench_fluid_get_cp;
  shader.volumic_mass = bench_fluid_get_rho;

  OK(sdis_fluid_create(dev, &shader, data, &fluid));
  OK(sdis_data_ref_put(data));
  return fluid;
}

static struct sdis_medium*
bench_create_solid(struct sdis_device* dev)
{
  struct sdis_solid_shader shader = DUMMY_SOLID_SHADER;
  struct sdis_data* data = NULL;
  struct sdis_medium* solid = NULL;
  struct bench_solid params;

  params.temperature = SDIS_TEMPERATURE_NONE;
  params.lambda = 0.1;
  params.cp = 800.0;
  params.rho = 2700.0;
  params.delta = 0.05; /* 1/20 of unit box */
  params.power = SDIS_VOLUMIC_POWER_NONE;

  OK(sdis_data_create(dev, sizeof(params), ALIGNOF(params), NULL, &data));
  memcpy(sdis_data_get(data), &params, sizeof(params));

  shader.thermal_conductivity = bench_solid_get_lambda;
  shader.calorific_capacity = bench_solid_get_cp;
  shader.volumic_mass = bench_solid_get_rho;
  shader.delta = bench_solid_get_delta;
  shader.temperature = bench_solid_get_temperature;
  shader.volumic_power = bench_solid_get_power;

  OK(sdis_solid_create(dev, &shader, data, &solid));
  OK(sdis_data_ref_put(data));
  return solid;
}

static struct sdis_interface*
bench_create_interface
  (struct sdis_device* dev,
   struct sdis_medium* front,
   struct sdis_medium* back,
   double hc, double epsilon, double specular)
{
  struct sdis_interface_shader shader = SDIS_INTERFACE_SHADER_NULL;
  struct sdis_data* data = NULL;
  struct sdis_interface* interf = NULL;
  struct bench_interf params;

  params.hc = hc;
  params.epsilon = epsilon;
  params.specular = specular;
  params.reference_temperature = 300.0;

  OK(sdis_data_create(dev, sizeof(params), ALIGNOF(params), NULL, &data));
  memcpy(sdis_data_get(data), &params, sizeof(params));

  shader.convection_coef = bench_interf_get_hc;
  if(sdis_medium_get_type(front) == SDIS_FLUID) {
    shader.front.emissivity = bench_interf_get_epsilon;
    shader.front.specular_fraction = bench_interf_get_specular;
    shader.front.reference_temperature = bench_interf_get_reference_temperature;
  }
  if(sdis_medium_get_type(back) == SDIS_FLUID) {
    shader.back.emissivity = bench_interf_get_epsilon;
    shader.back.specular_fraction = bench_interf_get_specular;
    shader.back.reference_temperature = bench_interf_get_reference_temperature;
  }

  OK(sdis_interface_create(dev, front, back, &shader, data, &interf));
  OK(sdis_data_ref_put(data));
  return interf;
}

static struct sdis_radiative_env*
bench_create_radenv(struct sdis_device* dev, double temp, double ref)
{
  struct sdis_radiative_env_shader shader = SDIS_RADIATIVE_ENV_SHADER_NULL;
  struct sdis_radiative_env* radenv = NULL;

  bench_radenv_temperature = temp;
  bench_radenv_reference = ref;
  shader.temperature = bench_radenv_get_temperature;
  shader.reference_temperature = bench_radenv_get_reference;
  OK(sdis_radiative_env_create(dev, &shader, NULL, &radenv));
  return radenv;
}

static struct sdis_camera*
bench_create_camera(struct sdis_device* dev)
{
  const double pos[3] = {2, 2, 2};
  const double tgt[3] = {0.5, 0.5, 0.5};
  const double up[3]  = {0, 0, 1};
  struct sdis_camera* cam = NULL;

  OK(sdis_camera_create(dev, &cam));
  OK(sdis_camera_set_proj_ratio(cam,
    (double)BENCH_IMG_WIDTH / (double)BENCH_IMG_HEIGHT));
  OK(sdis_camera_set_fov(cam, MDEG2RAD(60)));
  OK(sdis_camera_look_at(cam, pos, tgt, up));
  return cam;
}

/* -------------------------------------------------------------------------- */
/* Benchmark run                                                              */
/* -------------------------------------------------------------------------- */
static res_T
bench_run
  (struct sdis_scene* scn,
   struct sdis_camera* cam,
   const int use_wavefront,
   struct sdis_estimator_buffer** out_img,
   struct time* out_elapsed)
{
  struct sdis_solve_camera_args args = SDIS_SOLVE_CAMERA_ARGS_DEFAULT;
  struct time t0, t1;
  res_T res;

  args.cam = cam;
  args.time_range[0] = INF;
  args.time_range[1] = INF;
  args.image_definition[0] = BENCH_IMG_WIDTH;
  args.image_definition[1] = BENCH_IMG_HEIGHT;
  args.spp = BENCH_SPP;

  /* Set environment variable to control wavefront mode */
  if(use_wavefront) {
#ifdef _WIN32
    _putenv_s("STARDIS_WAVEFRONT", "1");
#else
    setenv("STARDIS_WAVEFRONT", "1", 1);
#endif
  } else {
#ifdef _WIN32
    _putenv_s("STARDIS_WAVEFRONT", "0");
#else
    setenv("STARDIS_WAVEFRONT", "0", 1);
#endif
  }

  time_current(&t0);
  res = sdis_solve_camera(scn, &args, out_img);
  time_current(&t1);

  if(out_elapsed) time_sub(out_elapsed, &t1, &t0);
  return res;
}

/* -------------------------------------------------------------------------- */
/* Compare two images                                                         */
/* -------------------------------------------------------------------------- */
static void
bench_compare_images
  (const struct sdis_estimator_buffer* img_ref,
   const struct sdis_estimator_buffer* img_wf,
   int* out_pass)
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

      if(sdis_estimator_get_realisation_count(est_ref, &count_ref) != RES_OK
         || count_ref == 0) continue;
      if(sdis_estimator_get_realisation_count(est_wf, &count_wf) != RES_OK
         || count_wf == 0) continue;
      if(sdis_estimator_get_temperature(est_ref, &mc_ref) != RES_OK) continue;
      if(sdis_estimator_get_temperature(est_wf,  &mc_wf)  != RES_OK) continue;

      n_compared++;

      /* Statistical compatibility: |mean_ref - mean_wf| < BENCH_TOL_SIGMA * combined SE */
      {
        double se_combined = sqrt(mc_ref.SE * mc_ref.SE
                                  + mc_wf.SE * mc_wf.SE);
        double diff = fabs(mc_ref.E - mc_wf.E);

        if(se_combined < 1e-12 || diff <= BENCH_TOL_SIGMA * se_combined) {
          n_compat++;
        } else {
          if(n_compared - n_compat <= 5) { /* Print first few failures */
            fprintf(stderr,
              "  pixel (%lu,%lu): ref=%.6f wf=%.6f diff=%.2e se=%.2e "
              "(%.1f sigma)\n",
              (unsigned long)x, (unsigned long)y,
              mc_ref.E, mc_wf.E, diff, se_combined,
              se_combined > 0 ? diff/se_combined : 0);
          }
        }
      }
    }
  }

  fprintf(stdout, "  Pixels compared: %lu, compatible: %lu (%.1f%%)\n",
    (unsigned long)n_compared, (unsigned long)n_compat,
    n_compared > 0 ? 100.0*(double)n_compat/(double)n_compared : 0.0);

  /* Pass if >= 95% of pixels are statistically compatible */
  *out_pass = (n_compared > 0 && n_compat * 100 >= n_compared * 95);
}

/* -------------------------------------------------------------------------- */
/* main                                                                       */
/* -------------------------------------------------------------------------- */
int
main(int argc, char** argv)
{
  struct sdis_device* dev = NULL;
  struct sdis_medium* fluid = NULL;
  struct sdis_medium* solid = NULL;
  struct sdis_interface* interf = NULL;
  struct sdis_interface* interfaces[12]; /* one per box triangle */
  struct sdis_radiative_env* radenv = NULL;
  struct sdis_scene_create_args scn_args = SDIS_SCENE_CREATE_ARGS_DEFAULT;
  struct sdis_scene* scn = NULL;
  struct sdis_camera* cam = NULL;
  struct sdis_estimator_buffer* img_ref = NULL;
  struct sdis_estimator_buffer* img_wf  = NULL;
  struct time t_ref, t_wf;
  int is_master = 0;
  int pass = 0;
  size_t i;

  (void)argc; (void)argv;

  fprintf(stdout, "=== Wavefront Benchmark Test (Phase B-2/M5) ===\n");
  fprintf(stdout, "Image: %dx%d, SPP: %d\n",
    BENCH_IMG_WIDTH, BENCH_IMG_HEIGHT, BENCH_SPP);

  /* ---- Device ---- */
  create_default_device(&argc, &argv, &is_master, &dev);

  /* ---- Media ---- */
  fluid = bench_create_fluid(dev, 350.0);
  solid = bench_create_solid(dev);

  /* ---- Interface: fluid (front) / solid (back), hc=0.1, eps=1.0 ---- */
  interf = bench_create_interface(dev, fluid, solid, 0.1, 1.0, 0.0);

  /* All 12 triangles of the box share the same interface */
  for(i = 0; i < 12; i++) interfaces[i] = interf;

  /* ---- Radiative environment ---- */
  radenv = bench_create_radenv(dev, 300.0, 300.0);

  /* ---- Scene ---- */
  scn_args.get_indices = box_get_indices;
  scn_args.get_position = box_get_position;
  scn_args.get_interface = box_get_interface;
  scn_args.nprimitives = box_ntriangles;
  scn_args.nvertices = box_nvertices;
  scn_args.t_range[0] = 300;
  scn_args.t_range[1] = 400;
  scn_args.radenv = radenv;
  scn_args.context = interfaces;
  OK(sdis_scene_create(dev, &scn_args, &scn));

  /* ---- Camera ---- */
  cam = bench_create_camera(dev);

  if(is_master) {
    /* ---- Run 1: Original (depth-first) ---- */
    fprintf(stdout, "\n--- Run 1: Depth-first (original) ---\n");
    OK(bench_run(scn, cam, 0, &img_ref, &t_ref));
    {
      char ts[128];
      time_dump(&t_ref, TIME_ALL, NULL, ts, sizeof(ts));
      fprintf(stdout, "  Time: %s\n", ts);
    }

    /* ---- Run 2: Wavefront ---- */
    fprintf(stdout, "\n--- Run 2: Wavefront ---\n");
    OK(bench_run(scn, cam, 1, &img_wf, &t_wf));
    {
      char ts[128];
      time_dump(&t_wf, TIME_ALL, NULL, ts, sizeof(ts));
      fprintf(stdout, "  Time: %s\n", ts);
    }

    /* ---- Compare ---- */
    fprintf(stdout, "\n--- Comparison ---\n");
    bench_compare_images(img_ref, img_wf, &pass);

    /* ---- Speedup ---- */
    {
      int64_t usec_ref = time_val(&t_ref, TIME_USEC);
      int64_t usec_wf  = time_val(&t_wf,  TIME_USEC);
      if(usec_wf > 0) {
        fprintf(stdout, "  Speedup: %.2fx\n",
          (double)usec_ref / (double)usec_wf);
      }
    }

    fprintf(stdout, "\n%s\n", pass ? "PASS" : "FAIL (statistical mismatch)");

    /* ---- Cleanup images ---- */
    OK(sdis_estimator_buffer_ref_put(img_ref));
    OK(sdis_estimator_buffer_ref_put(img_wf));
  }

  /* ---- Cleanup ---- */
  OK(sdis_camera_ref_put(cam));
  OK(sdis_scene_ref_put(scn));
  OK(sdis_radiative_env_ref_put(radenv));
  OK(sdis_interface_ref_put(interf));
  OK(sdis_medium_ref_put(solid));
  OK(sdis_medium_ref_put(fluid));
  free_default_device(dev);

  CHK(mem_allocated_size() == 0);
  return pass ? 0 : 1;
}
