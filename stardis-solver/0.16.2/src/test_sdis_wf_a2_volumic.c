/* WF-A2: Volumic power steady-state conduction (wavefront probe).
 *
 * Scene: unit cube solid, lambda=0.1, delta=1/60, P=10 W/m^3
 *   -X face: imposed temperature T0 = 320 K
 *   +X face: imposed temperature T0 = 320 K
 *   Other 4 faces: adiabatic
 *
 * Analytic: T(x) = P / (2*LAMBDA) * (1/4 - (x-0.5)^2) + T0
 *         = 50 * (0.25 - (x-0.5)^2) + 320
 *
 * 11 probes along X (y=0.5, z=0.5), dual validation A+B.
 *
 * Reference CPU test: test_sdis_volumic_power.c
 */

#include "sdis.h"
#include "test_sdis_utils.h"
#include "test_sdis_wf_p0_utils.h"
#include "test_sdis_csv_utils.h"

#include <rsys/mem_allocator.h>
#include <stdio.h>
#include <math.h>

/* ========================================================================== */
/* Physical constants                                                         */
/* ========================================================================== */
#define A2_T0      320.0
#define A2_LAMBDA  0.1
#define A2_CP      2.0
#define A2_RHO     25.0
#define A2_DELTA   (1.0 / 60.0)
#define A2_P0      10.0

/* ========================================================================== */
/* Media shaders                                                              */
/* ========================================================================== */
struct a2_solid {
  double lambda;
  double rho;
  double cp;
  double delta;
  double vpower;
  double initial_temperature;
};

static double
a2_solid_get_calorific_capacity
  (const struct sdis_rwalk_vertex* vtx, struct sdis_data* data)
{
  CHK(vtx != NULL);
  return ((struct a2_solid*)sdis_data_cget(data))->cp;
}

static double
a2_solid_get_thermal_conductivity
  (const struct sdis_rwalk_vertex* vtx, struct sdis_data* data)
{
  CHK(vtx != NULL);
  return ((struct a2_solid*)sdis_data_cget(data))->lambda;
}

static double
a2_solid_get_volumic_mass
  (const struct sdis_rwalk_vertex* vtx, struct sdis_data* data)
{
  CHK(vtx != NULL);
  return ((struct a2_solid*)sdis_data_cget(data))->rho;
}

static double
a2_solid_get_delta
  (const struct sdis_rwalk_vertex* vtx, struct sdis_data* data)
{
  CHK(vtx != NULL);
  return ((struct a2_solid*)sdis_data_cget(data))->delta;
}

static double
a2_solid_get_temperature
  (const struct sdis_rwalk_vertex* vtx, struct sdis_data* data)
{
  CHK(vtx != NULL);
  (void)data;
  if(vtx->time > 0)
    return SDIS_TEMPERATURE_NONE;
  else
    return ((struct a2_solid*)sdis_data_cget(data))->initial_temperature;
}

static double
a2_solid_get_volumic_power
  (const struct sdis_rwalk_vertex* vtx, struct sdis_data* data)
{
  CHK(vtx != NULL);
  return ((struct a2_solid*)sdis_data_cget(data))->vpower;
}

/* ========================================================================== */
/* Interface shaders                                                          */
/* ========================================================================== */
struct a2_interf {
  double temperature;
};

static double
a2_interface_get_temperature
  (const struct sdis_interface_fragment* frag, struct sdis_data* data)
{
  const struct a2_interf* p = sdis_data_cget(data);
  CHK(frag && data);
  return p->temperature;
}

static double
a2_interface_get_convection_coef
  (const struct sdis_interface_fragment* frag, struct sdis_data* data)
{
  CHK(frag && data);
  return 0.0;
}

/* ========================================================================== */
/* Analytic solution                                                          */
/* ========================================================================== */
static double
a2_analytic(double x)
{
  double dx = x - 0.5;
  return A2_P0 / (2.0 * A2_LAMBDA) * (0.25 - dx * dx) + A2_T0;
}

/* ========================================================================== */
/* Test body                                                                  */
/* ========================================================================== */
int
main(int argc, char** argv)
{
  struct sdis_device* dev = NULL;
  struct sdis_data* data = NULL;
  struct sdis_medium* fluid = NULL;
  struct sdis_medium* solid = NULL;
  struct sdis_interface* interf_adiabatic = NULL;
  struct sdis_interface* interf_t0 = NULL;
  struct sdis_scene* scn = NULL;
  struct sdis_scene_create_args scn_args = SDIS_SCENE_CREATE_ARGS_DEFAULT;
  struct sdis_fluid_shader fluid_shader = DUMMY_FLUID_SHADER;
  struct sdis_solid_shader solid_shader = DUMMY_SOLID_SHADER;
  struct sdis_interface_shader interf_shader = SDIS_INTERFACE_SHADER_NULL;
  struct sdis_interface* box_interfaces[12];
  struct a2_interf* interf_props = NULL;
  struct a2_solid* solid_props = NULL;
  int pass = 0;
  (void)argc; (void)argv;

  printf("=== WF-A2: Volumic power steady-state conduction (wavefront probe) ===\n");

  OK(sdis_device_create(&SDIS_DEVICE_CREATE_ARGS_DEFAULT, &dev));

  /* ---- Fluid (dummy) ---- */
  OK(sdis_fluid_create(dev, &fluid_shader, NULL, &fluid));

  /* ---- Solid (with volumic power) ---- */
  solid_shader.calorific_capacity = a2_solid_get_calorific_capacity;
  solid_shader.thermal_conductivity = a2_solid_get_thermal_conductivity;
  solid_shader.volumic_mass = a2_solid_get_volumic_mass;
  solid_shader.delta = a2_solid_get_delta;
  solid_shader.temperature = a2_solid_get_temperature;
  solid_shader.volumic_power = a2_solid_get_volumic_power;

  OK(sdis_data_create(dev, sizeof(struct a2_solid), 16, NULL, &data));
  solid_props = sdis_data_get(data);
  solid_props->lambda = A2_LAMBDA;
  solid_props->cp = A2_CP;
  solid_props->rho = A2_RHO;
  solid_props->delta = A2_DELTA;
  solid_props->vpower = A2_P0;
  solid_props->initial_temperature = A2_T0;
  OK(sdis_solid_create(dev, &solid_shader, data, &solid));
  OK(sdis_data_ref_put(data));

  /* ---- Interface shader ---- */
  interf_shader.convection_coef = a2_interface_get_convection_coef;
  interf_shader.front.temperature = a2_interface_get_temperature;

  /* Adiabatic: T=NONE */
  OK(sdis_data_create(dev, sizeof(struct a2_interf), 16, NULL, &data));
  interf_props = sdis_data_get(data);
  interf_props->temperature = SDIS_TEMPERATURE_NONE;
  OK(sdis_interface_create(dev, solid, fluid, &interf_shader, data,
    &interf_adiabatic));
  OK(sdis_data_ref_put(data));

  /* T0: T=320 */
  OK(sdis_data_create(dev, sizeof(struct a2_interf), 16, NULL, &data));
  interf_props = sdis_data_get(data);
  interf_props->temperature = A2_T0;
  OK(sdis_interface_create(dev, solid, fluid, &interf_shader, data,
    &interf_t0));
  OK(sdis_data_ref_put(data));

  /* Release media */
  OK(sdis_medium_ref_put(solid));
  OK(sdis_medium_ref_put(fluid));

  /* ---- Triangle-to-interface mapping ---- */
  /* -Z(0-1), -X(2-3), +Z(4-5), +X(6-7), +Y(8-9), -Y(10-11) */
  box_interfaces[0] = box_interfaces[1] = interf_adiabatic;  /* Front -Z */
  box_interfaces[2] = box_interfaces[3] = interf_t0;          /* Left -X: T0 */
  box_interfaces[4] = box_interfaces[5] = interf_adiabatic;  /* Back +Z */
  box_interfaces[6] = box_interfaces[7] = interf_t0;          /* Right +X: T0 */
  box_interfaces[8] = box_interfaces[9] = interf_adiabatic;  /* Top +Y */
  box_interfaces[10] = box_interfaces[11] = interf_adiabatic; /* Bottom -Y */

  /* ---- Scene ---- */
  scn_args.get_indices = box_get_indices;
  scn_args.get_interface = box_get_interface;
  scn_args.get_position = box_get_position;
  scn_args.nprimitives = box_ntriangles;
  scn_args.nvertices = box_nvertices;
  scn_args.context = box_interfaces;
  OK(sdis_scene_create(dev, &scn_args, &scn));

  /* Release interfaces */
  OK(sdis_interface_ref_put(interf_adiabatic));
  OK(sdis_interface_ref_put(interf_t0));

  /* ---- Run probe sweep with CSV output ---- */
  {
    struct sdis_solve_probe_args args_arr[P0_MAX_PROBES_BATCH];
    struct sdis_estimator* ests[P0_MAX_PROBES_BATCH];
    int n_pass_primary = 0;
    size_t i;
    FILE* csv = csv_open("A2");

    fprintf(stdout, "  Running %d probes (batch), %d realisations each ...\n",
      P0_NPROBES, P0_NREALISATIONS);

    for(i = 0; i < P0_NPROBES; i++) {
      double x = 0.05 + (double)i * 0.9 / (double)(P0_NPROBES - 1);
      args_arr[i] = SDIS_SOLVE_PROBE_ARGS_DEFAULT;
      args_arr[i].nrealisations = P0_NREALISATIONS;
      args_arr[i].position[0] = x;
      args_arr[i].position[1] = 0.5;
      args_arr[i].position[2] = 0.5;
      args_arr[i].picard_order = 1;
      args_arr[i].diff_algo = SDIS_DIFFUSION_DELTA_SPHERE;
      ests[i] = NULL;
    }

    OK(sdis_solve_persistent_wavefront_probe_batch(
      scn, P0_NPROBES, args_arr, ests));

    for(i = 0; i < P0_NPROBES; i++) {
      double x = args_arr[i].position[0];
      double T_ref = a2_analytic(x);
      struct sdis_mc mc;

      n_pass_primary += p0_compare_analytic(ests[i], T_ref, P0_TOL_SIGMA);
      OK(sdis_estimator_get_temperature(ests[i], &mc));

      csv_row(csv, "A2", "default", "gpu_wf", "DS",
              x, 0.5, 0.5, INF, 1, P0_NREALISATIONS,
              mc.E, mc.SE, T_ref);

      p0_print_probe_result(x, ests[i], NULL, T_ref);
      OK(sdis_estimator_ref_put(ests[i]));
    }

    csv_close(csv);

    fprintf(stdout, "  Primary: %d/%d probes pass (%.1f%%)\n",
      n_pass_primary, P0_NPROBES,
      100.0 * (double)n_pass_primary / (double)P0_NPROBES);
    pass = (double)n_pass_primary / (double)P0_NPROBES >= P0_PASS_RATE;
  }

  CHK(pass);
  printf("WF-A2: PASS\n");

  /* ---- Cleanup ---- */
  OK(sdis_scene_ref_put(scn));
  OK(sdis_device_ref_put(dev));

  CHK(mem_allocated_size() == 0);
  return 0;
}
