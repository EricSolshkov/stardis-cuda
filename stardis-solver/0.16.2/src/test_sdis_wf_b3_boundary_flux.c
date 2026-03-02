/* WF-B3: Boundary flux â€?interior temperature profile (wavefront probe).
 *
 * Scene (3D only): unit cube solid [0,1]^3 with surrounding fluid.
 *   Solid: lambda=0.1, rho=25, cp=2, delta=1/40
 *
 *   -X face: convection H=0.5, T_fluid=300, emission eps=1, Tref=Tb=0
 *   +X face: convection H=0.5, T_fluid=300, emission eps=1, Tref=300
 *   Other faces: adiabatic (H=0, eps=0)
 *   Radiative environment: T=300, Tref=300
 *
 * Steady-state analytic:
 *   Hrad = 4*sigma*Tref^3*eps
 *   T(+X) = (H*Tf + Hrad*Trad + lambda*Tb) / (H + Hrad + lambda)
 *   T(x)  = Tb + (T(+X) - Tb) * x   (linear profile from x=0 to x=1)
 *
 * Because sdis_solve_probe_boundary_flux has no wavefront variant, we
 * verify the interior temperature profile instead. 5 probes at x = 0.1,
 * 0.3, 0.5, 0.7, 0.9 and y=z=0.5, time=INF (steady state).
 *
 * Reference CPU test: test_sdis_solve_boundary_flux.c
 */

#include "sdis.h"
#include "test_sdis_utils.h"
#include "test_sdis_wf_p0_utils.h"
#include "test_sdis_csv_utils.h"

#include <rsys/mem_allocator.h>
#include <stdio.h>
#include <math.h>
#include <string.h>

/* ========================================================================== */
/* Physical constants (identical to CPU test)                                  */
/* ========================================================================== */
#define B3_LAMBDA  0.1
#define B3_RHO     25.0
#define B3_CP      2.0
#define B3_DELTA   (1.0 / 40.0)
#define B3_H       0.5
#define B3_Tf      300.0
#define B3_Tb      0.0
#define B3_Trad    300.0
#define B3_Tref    300.0
#define B3_EPSILON 1.0
#define B3_NREALS  10000
#define B3_NPROBES_CT 5
#define B3_TOL_SIGMA 3.0
#define B3_PASS_RATE 0.95

#ifndef BOLTZMANN_CONSTANT
#define BOLTZMANN_CONSTANT 5.670374419e-8
#endif

#define B3_Hrad (4.0 * BOLTZMANN_CONSTANT * B3_Tref * B3_Tref * B3_Tref * B3_EPSILON)

/* Analytic solution */
#define B3_T_RIGHT ((B3_H*B3_Tf + B3_Hrad*B3_Trad + B3_LAMBDA*B3_Tb) / (B3_H + B3_Hrad + B3_LAMBDA))
#define B3_T(x) (B3_Tb + (B3_T_RIGHT - B3_Tb) * (x))

/* ========================================================================== */
/* Media                                                                      */
/* ========================================================================== */
static double
b3_solid_get_cp(const struct sdis_rwalk_vertex* vtx, struct sdis_data* data)
{
  (void)data; CHK(vtx); return B3_CP;
}

static double
b3_solid_get_lambda(const struct sdis_rwalk_vertex* vtx, struct sdis_data* data)
{
  (void)data; CHK(vtx); return B3_LAMBDA;
}

static double
b3_solid_get_rho(const struct sdis_rwalk_vertex* vtx, struct sdis_data* data)
{
  (void)data; CHK(vtx); return B3_RHO;
}

static double
b3_solid_get_delta(const struct sdis_rwalk_vertex* vtx, struct sdis_data* data)
{
  (void)data; CHK(vtx); return B3_DELTA;
}

static double
b3_solid_get_temperature(const struct sdis_rwalk_vertex* vtx,
                         struct sdis_data* data)
{
  (void)data; CHK(vtx);
  return SDIS_TEMPERATURE_NONE;
}

static double
b3_fluid_get_temperature(const struct sdis_rwalk_vertex* vtx,
                         struct sdis_data* data)
{
  CHK(vtx && data);
  return *(const double*)sdis_data_cget(data);
}

/* ========================================================================== */
/* Interfaces                                                                 */
/* ========================================================================== */
struct b3_interf {
  double temperature;
  double emissivity;
  double hc;
  double reference_temperature;
};

static double
b3_interf_get_temperature(const struct sdis_interface_fragment* frag,
                          struct sdis_data* data)
{
  CHK(frag && data);
  return ((const struct b3_interf*)sdis_data_cget(data))->temperature;
}

static double
b3_interf_get_emissivity(const struct sdis_interface_fragment* frag,
                         const unsigned source_id, struct sdis_data* data)
{
  (void)source_id; CHK(frag && data);
  return ((const struct b3_interf*)sdis_data_cget(data))->emissivity;
}

static double
b3_interf_get_h(const struct sdis_interface_fragment* frag,
                struct sdis_data* data)
{
  CHK(frag && data);
  return ((const struct b3_interf*)sdis_data_cget(data))->hc;
}

static double
b3_interf_get_Tref(const struct sdis_interface_fragment* frag,
                   struct sdis_data* data)
{
  CHK(frag && data);
  return ((const struct b3_interf*)sdis_data_cget(data))->reference_temperature;
}

/* ========================================================================== */
/* Radiative environment                                                      */
/* ========================================================================== */
static double
b3_radenv_get_temperature(const struct sdis_radiative_ray* ray,
                          struct sdis_data* data)
{
  (void)ray; (void)data; return B3_Trad;
}

static double
b3_radenv_get_Tref(const struct sdis_radiative_ray* ray,
                   struct sdis_data* data)
{
  (void)ray; (void)data; return B3_Trad;
}

/* ========================================================================== */
/* Test body                                                                  */
/* ========================================================================== */
int
main(int argc, char** argv)
{
  struct sdis_device* dev = NULL;
  struct sdis_radiative_env* radenv = NULL;
  struct sdis_radiative_env_shader radenv_shader = SDIS_RADIATIVE_ENV_SHADER_NULL;
  struct sdis_scene* scn = NULL;
  struct sdis_medium* solid = NULL;
  struct sdis_medium* fluid1 = NULL;
  struct sdis_medium* fluid2 = NULL;
  struct sdis_solid_shader solid_shader = DUMMY_SOLID_SHADER;
  struct sdis_fluid_shader fluid_shader = DUMMY_FLUID_SHADER;
  struct sdis_interface_shader interf_shader = SDIS_INTERFACE_SHADER_NULL;
  struct sdis_interface* interf_adiabatic = NULL;
  struct sdis_interface* interf_Tb = NULL;
  struct sdis_interface* interf_H = NULL;
  struct sdis_interface* box_interfaces[12];
  struct sdis_scene_create_args scn_args = SDIS_SCENE_CREATE_ARGS_DEFAULT;
  struct sdis_data* data = NULL;
  struct b3_interf* ip = NULL;
  double* fluid_temp = NULL;
  int n_pass = 0;
  int n_probes;

  static const double probe_x[] = {0.1, 0.3, 0.5, 0.7, 0.9};
  int i;
  FILE* csv = NULL;
  (void)argc; (void)argv;

  n_probes = (int)(sizeof(probe_x)/sizeof(probe_x[0]));

  printf("=== WF-B3: Boundary flux - interior temperature profile (wavefront) ===\n");
  printf("  Hrad = %.6f\n", B3_Hrad);
  printf("  T(+X) = %.6f\n", B3_T_RIGHT);
  printf("  T(x=0.5) = %.6f\n", B3_T(0.5));

  csv = csv_open("B3");

  OK(sdis_device_create(&SDIS_DEVICE_CREATE_ARGS_DEFAULT, &dev));

  /* ---- Radiative environment ---- */
  radenv_shader.temperature = b3_radenv_get_temperature;
  radenv_shader.reference_temperature = b3_radenv_get_Tref;
  OK(sdis_radiative_env_create(dev, &radenv_shader, NULL, &radenv));

  /* ---- Solid medium ---- */
  solid_shader.calorific_capacity = b3_solid_get_cp;
  solid_shader.thermal_conductivity = b3_solid_get_lambda;
  solid_shader.volumic_mass = b3_solid_get_rho;
  solid_shader.delta = b3_solid_get_delta;
  solid_shader.temperature = b3_solid_get_temperature;
  OK(sdis_solid_create(dev, &solid_shader, NULL, &solid));

  /* ---- Fluid media (two separate, same T=300, as in CPU test) ---- */
  fluid_shader.temperature = b3_fluid_get_temperature;

  OK(sdis_data_create(dev, sizeof(double), ALIGNOF(double), NULL, &data));
  fluid_temp = sdis_data_get(data);
  *fluid_temp = B3_Tf;
  OK(sdis_fluid_create(dev, &fluid_shader, data, &fluid1));
  OK(sdis_fluid_create(dev, &fluid_shader, data, &fluid2));
  OK(sdis_data_ref_put(data));

  /* ---- Interface shader ---- */
  interf_shader.convection_coef = b3_interf_get_h;
  interf_shader.front.temperature = b3_interf_get_temperature;
  interf_shader.front.specular_fraction = NULL;
  interf_shader.back = SDIS_INTERFACE_SIDE_SHADER_NULL;

  /* ---- Adiabatic interface ---- */
  OK(sdis_data_create(dev, sizeof(struct b3_interf), 16, NULL, &data));
  ip = sdis_data_get(data);
  ip->hc = 0;
  ip->temperature = SDIS_TEMPERATURE_NONE;
  ip->emissivity = 0;
  ip->reference_temperature = 0;
  OK(sdis_interface_create(dev, solid, fluid1, &interf_shader, data,
    &interf_adiabatic));
  OK(sdis_data_ref_put(data));

  /* ---- Tb interface (-X face) ---- */
  OK(sdis_data_create(dev, sizeof(struct b3_interf), 16, NULL, &data));
  ip = sdis_data_get(data);
  ip->hc = B3_H;
  ip->temperature = B3_Tb;
  ip->emissivity = B3_EPSILON;
  ip->reference_temperature = B3_Tb;
  interf_shader.back.emissivity = b3_interf_get_emissivity;
  interf_shader.back.reference_temperature = b3_interf_get_Tref;
  OK(sdis_interface_create(dev, solid, fluid1, &interf_shader, data,
    &interf_Tb));
  interf_shader.back.emissivity = NULL;
  interf_shader.back.reference_temperature = NULL;
  OK(sdis_data_ref_put(data));

  /* ---- H interface (+X face) ---- */
  OK(sdis_data_create(dev, sizeof(struct b3_interf), 16, NULL, &data));
  ip = sdis_data_get(data);
  ip->hc = B3_H;
  ip->temperature = SDIS_TEMPERATURE_NONE;
  ip->emissivity = B3_EPSILON;
  ip->reference_temperature = B3_Tref;
  interf_shader.back.emissivity = b3_interf_get_emissivity;
  interf_shader.back.reference_temperature = b3_interf_get_Tref;
  OK(sdis_interface_create(dev, solid, fluid2, &interf_shader, data,
    &interf_H));
  interf_shader.back.emissivity = NULL;
  interf_shader.back.reference_temperature = NULL;
  OK(sdis_data_ref_put(data));

  /* ---- Per-primitive interface mapping (box from test_sdis_utils.h) ---- */
  box_interfaces[0]  = box_interfaces[1]  = interf_adiabatic; /* Front  -Z */
  box_interfaces[2]  = box_interfaces[3]  = interf_Tb;        /* Left   -X */
  box_interfaces[4]  = box_interfaces[5]  = interf_adiabatic; /* Back   +Z */
  box_interfaces[6]  = box_interfaces[7]  = interf_H;         /* Right  +X */
  box_interfaces[8]  = box_interfaces[9]  = interf_adiabatic; /* Top    +Y */
  box_interfaces[10] = box_interfaces[11] = interf_adiabatic; /* Bottom -Y */

  /* ---- Create scene (3D box from test_sdis_utils.h) ---- */
  scn_args.get_indices = box_get_indices;
  scn_args.get_interface = box_get_interface;
  scn_args.get_position = box_get_position;
  scn_args.nprimitives = box_ntriangles;
  scn_args.nvertices = box_nvertices;
  scn_args.t_range[0] = 0;
  scn_args.t_range[1] = 300;
  scn_args.radenv = radenv;
  scn_args.context = box_interfaces;
  OK(sdis_scene_create(dev, &scn_args, &scn));

  /* ---- Release media and interfaces ---- */
  OK(sdis_medium_ref_put(solid));
  OK(sdis_medium_ref_put(fluid1));
  OK(sdis_medium_ref_put(fluid2));
  OK(sdis_interface_ref_put(interf_adiabatic));
  OK(sdis_interface_ref_put(interf_Tb));
  OK(sdis_interface_ref_put(interf_H));

  /* ================================================================== */
  /* Solve: steady-state interior probes along x-axis — batch           */
  /* ================================================================== */
  printf("  Running %d probes (batch), %d realisations each ...\n",
    n_probes, B3_NREALS);

  {
    struct sdis_solve_probe_args args_arr[B3_NPROBES_CT];
    struct sdis_estimator*       ests[B3_NPROBES_CT];

    for(i = 0; i < n_probes; i++) {
      args_arr[i] = SDIS_SOLVE_PROBE_ARGS_DEFAULT;
      args_arr[i].nrealisations = B3_NREALS;
      args_arr[i].position[0]   = probe_x[i];
      args_arr[i].position[1]   = 0.5;
      args_arr[i].position[2]   = 0.5;
      args_arr[i].time_range[0] = INF;
      args_arr[i].time_range[1] = INF;
      ests[i] = NULL;
    }

    OK(sdis_solve_persistent_wavefront_probe_batch(
      scn, (size_t)n_probes, args_arr, ests));

    for(i = 0; i < n_probes; i++) {
      struct sdis_mc mc;
      double ref;
      int pass;

      OK(sdis_estimator_get_temperature(ests[i], &mc));

      ref = B3_T(probe_x[i]);
      pass = p0_compare_analytic(ests[i], ref, B3_TOL_SIGMA);
      n_pass += pass;

      csv_row(csv, "B3", "default", "gpu_wf", "DS",
              probe_x[i], 0.5, 0.5, INF, 1, B3_NREALS, mc.E, mc.SE, ref);

      printf("  x=%.1f  wf=%.6f (SE=%.2e)  ref=%.6f  %s (%.1f sigma)\n",
        probe_x[i], mc.E, mc.SE, ref,
        pass ? "PASS" : "FAIL",
        mc.SE > 0 ? fabs(mc.E - ref) / mc.SE : 0.0);

      OK(sdis_estimator_ref_put(ests[i]));
    }
  }

  csv_close(csv);

  /* ---- Summary ---- */
  printf("\n  Primary:    %d/%d probes pass (%.1f%%)\n",
    n_pass, n_probes, 100.0 * (double)n_pass / (double)n_probes);
  CHK((double)n_pass / (double)n_probes >= B3_PASS_RATE);

  printf("WF-B3: PASS\n");

  /* ---- Cleanup ---- */
  OK(sdis_radiative_env_ref_put(radenv));
  OK(sdis_scene_ref_put(scn));
  OK(sdis_device_ref_put(dev));

  CHK(mem_allocated_size() == 0);
  return 0;
}
