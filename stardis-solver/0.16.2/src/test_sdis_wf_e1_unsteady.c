/* WF-E1: 3D transient conduction in a unit cube (wavefront probe).
 *
 * Scene: solid cube (0,0,0)-(1,1,1), FP_TO_METER=0.1
 *   Material: Cp=2000 J/K/kg, lambda=0.5 W/m/K, rho=2500 kg/m^3, delta=1/60
 *   Initial temperature: 280 K (t<=0)
 *   Six faces fixed at different temperatures:
 *     -X=T0=310, +X=T1=320, -Y=T2=330, +Y=T3=310, -Z=T4=320, +Z=T5=300
 *
 * Probe at (0.3, 0.4, 0.6) [fp units], 9 time points.
 * Analytic reference: Green function series solution (hardcoded values).
 *
 * Verification: wavefront result vs Green function analytic values.
 *   Each probe: |T_wf - T_ref| <= 4 * SE  (relaxed for transient variance)
 *   Pass criterion: >= 90% of probes pass.
 *
 * Reference CPU test: test_sdis_unsteady.c
 */

#include "sdis.h"
#include "test_sdis_utils.h"
#include "test_sdis_wf_p0_utils.h"

#include <rsys/mem_allocator.h>
#include <stdio.h>
#include <math.h>

/* ========================================================================== */
/* Physical constants                                                         */
/* ========================================================================== */
#define E1_T_INIT   280.0   /* [K] initial temperature */
#define E1_T0       310.0   /* [K] -X face */
#define E1_T1       320.0   /* [K] +X face */
#define E1_T2       330.0   /* [K] -Y face */
#define E1_T3       310.0   /* [K] +Y face */
#define E1_T4       320.0   /* [K] -Z face (= +Z box tri) */
#define E1_T5       300.0   /* [K] +Z face (= -Z box tri) */

#define E1_CP       2000.0  /* [J/K/kg] */
#define E1_LAMBDA   0.5     /* [W/m/K] */
#define E1_RHO      2500.0  /* [kg/m^3] */
#define E1_DELTA    (1.0 / 60.0)

#define E1_FP_TO_METER  0.1

#define E1_NREALS   10000
#define E1_TOL_SIGMA 4.0   /* relaxed for transient (higher variance) */
#define E1_PASS_RATE 0.90  /* 90% pass rate */

/* ========================================================================== */
/* Reference data: Green function analytic solution                           */
/* ========================================================================== */
struct e1_reference {
  double pos[3]; /* [fp units] */
  double time;   /* [s] */
  double temp;   /* [K] */
};

static const struct e1_reference e1_refs[] = {
  {{0.3, 0.4, 0.6}, 1000.0000, 281.33455593977152},
  {{0.3, 0.4, 0.6}, 2000.0000, 286.90151817350699},
  {{0.3, 0.4, 0.6}, 3000.0000, 292.84330866161531},
  {{0.3, 0.4, 0.6}, 4000.0000, 297.81444160746452},
  {{0.3, 0.4, 0.6}, 5000.0000, 301.70787295764546},
  {{0.3, 0.4, 0.6}, 10000.000, 310.78920179442139},
  {{0.3, 0.4, 0.6}, 20000.000, 313.37629443163121},
  {{0.3, 0.4, 0.6}, 30000.000, 313.51064004438581},
  {{0.3, 0.4, 0.6}, 1000000.0, 313.51797642855502}
};
static const size_t e1_nrefs = sizeof(e1_refs) / sizeof(e1_refs[0]);

/* ========================================================================== */
/* Solid medium shaders                                                       */
/* ========================================================================== */
static double
e1_solid_get_calorific_capacity
  (const struct sdis_rwalk_vertex* vtx, struct sdis_data* data)
{
  (void)data; CHK(vtx != NULL);
  return E1_CP;
}

static double
e1_solid_get_thermal_conductivity
  (const struct sdis_rwalk_vertex* vtx, struct sdis_data* data)
{
  (void)data; CHK(vtx != NULL);
  return E1_LAMBDA;
}

static double
e1_solid_get_volumic_mass
  (const struct sdis_rwalk_vertex* vtx, struct sdis_data* data)
{
  (void)data; CHK(vtx != NULL);
  return E1_RHO;
}

static double
e1_solid_get_delta
  (const struct sdis_rwalk_vertex* vtx, struct sdis_data* data)
{
  (void)data; CHK(vtx != NULL);
  return E1_DELTA;
}

static double
e1_solid_get_temperature
  (const struct sdis_rwalk_vertex* vtx, struct sdis_data* data)
{
  (void)data; CHK(vtx != NULL);
  if(vtx->time <= 0) return E1_T_INIT;
  return SDIS_TEMPERATURE_NONE;
}

/* ========================================================================== */
/* Dummy exterior fluid                                                       */
/* ========================================================================== */
/* (required by Stardis: an interface must separate 2 media) */

/* ========================================================================== */
/* Interface shader: face temperature based on normal                         */
/* ========================================================================== */
static double
e1_interface_get_temperature
  (const struct sdis_interface_fragment* frag, struct sdis_data* data)
{
  (void)data; CHK(frag != NULL);
  /* Normal-based face identification (same as CPU test) */
       if(frag->Ng[0] ==  1) return E1_T0;
  else if(frag->Ng[0] == -1) return E1_T1;
  else if(frag->Ng[1] ==  1) return E1_T2;
  else if(frag->Ng[1] == -1) return E1_T3;
  else if(frag->Ng[2] ==  1) return E1_T4;
  else if(frag->Ng[2] == -1) return E1_T5;
  else { CHK(0 && "Unreachable"); return 0; }
}

/* ========================================================================== */
/* Test body                                                                  */
/* ========================================================================== */
int
main(int argc, char** argv)
{
  struct sdis_device* dev = NULL;
  struct sdis_medium* solid = NULL;
  struct sdis_medium* dummy = NULL;
  struct sdis_interface* interf = NULL;
  struct sdis_scene* scn = NULL;
  struct sdis_scene_create_args scn_args = SDIS_SCENE_CREATE_ARGS_DEFAULT;
  struct sdis_solid_shader solid_shader = SDIS_SOLID_SHADER_NULL;
  struct sdis_fluid_shader fluid_shader = SDIS_FLUID_SHADER_NULL;
  struct sdis_interface_shader interf_shader = SDIS_INTERFACE_SHADER_NULL;
  struct sdis_interface* box_interfaces[12];
  int n_pass = 0;
  size_t i;
  (void)argc; (void)argv;

  printf("=== WF-E1: 3D transient conduction (wavefront probe) ===\n");

  OK(sdis_device_create(&SDIS_DEVICE_CREATE_ARGS_DEFAULT, &dev));

  /* ---- Solid medium ---- */
  solid_shader.calorific_capacity = e1_solid_get_calorific_capacity;
  solid_shader.thermal_conductivity = e1_solid_get_thermal_conductivity;
  solid_shader.volumic_mass = e1_solid_get_volumic_mass;
  solid_shader.delta = e1_solid_get_delta;
  solid_shader.temperature = e1_solid_get_temperature;
  OK(sdis_solid_create(dev, &solid_shader, NULL, &solid));

  /* ---- Dummy exterior fluid ---- */
  fluid_shader.calorific_capacity = dummy_medium_getter;
  fluid_shader.volumic_mass = dummy_medium_getter;
  fluid_shader.temperature = dummy_medium_getter;
  OK(sdis_fluid_create(dev, &fluid_shader, NULL, &dummy));

  /* ---- Interface: front.temperature = back.temperature = face T ---- */
  interf_shader.front.temperature = e1_interface_get_temperature;
  interf_shader.back.temperature = e1_interface_get_temperature;
  OK(sdis_interface_create(dev, solid, dummy, &interf_shader, NULL, &interf));

  /* Release media (scene keeps refs) */
  OK(sdis_medium_ref_put(solid));
  OK(sdis_medium_ref_put(dummy));

  /* ---- Triangle-to-interface mapping (all 12 faces same interface) ---- */
  for(i = 0; i < 12; i++)
    box_interfaces[i] = interf;

  /* ---- Scene ---- */
  scn_args.get_indices = box_get_indices;
  scn_args.get_interface = box_get_interface;
  scn_args.get_position = box_get_position;
  scn_args.nprimitives = box_ntriangles;
  scn_args.nvertices = box_nvertices;
  scn_args.context = box_interfaces;
  scn_args.fp_to_meter = E1_FP_TO_METER;
  OK(sdis_scene_create(dev, &scn_args, &scn));

  /* Release interface */
  OK(sdis_interface_ref_put(interf));

  /* ---- Run probes at 9 time points ---- */
  fprintf(stdout, "  Running %lu time-point probes, %d realisations each ...\n",
    (unsigned long)e1_nrefs, E1_NREALS);

  for(i = 0; i < e1_nrefs; i++) {
    const struct e1_reference* ref = &e1_refs[i];
    struct sdis_solve_probe_args args = SDIS_SOLVE_PROBE_ARGS_DEFAULT;
    struct sdis_estimator* est_wf = NULL;
    struct sdis_mc mc_wf;
    int pass;

    args.nrealisations = E1_NREALS;
    args.position[0] = ref->pos[0];
    args.position[1] = ref->pos[1];
    args.position[2] = ref->pos[2];
    args.time_range[0] = ref->time;
    args.time_range[1] = ref->time;
    args.diff_algo = SDIS_DIFFUSION_DELTA_SPHERE;

    OK(sdis_solve_persistent_wavefront_probe(scn, &args, &est_wf));
    OK(sdis_estimator_get_temperature(est_wf, &mc_wf));

    /* Primary: wavefront vs Green function analytic (4 sigma, relaxed) */
    pass = p0_compare_analytic(est_wf, ref->temp, E1_TOL_SIGMA);
    n_pass += pass;

    fprintf(stdout,
      "  t=%10.1f s  wf=%.6f (SE=%.2e)  ref=%.6f  %s (%.1f sigma)\n",
      ref->time, mc_wf.E, mc_wf.SE, ref->temp,
      pass ? "PASS" : "FAIL",
      mc_wf.SE > 0 ? fabs(mc_wf.E - ref->temp) / mc_wf.SE : 0.0);

    OK(sdis_estimator_ref_put(est_wf));
  }

  fprintf(stdout, "  Primary:    %d/%lu probes pass (%.1f%%)\n",
    n_pass, (unsigned long)e1_nrefs,
    100.0 * (double)n_pass / (double)e1_nrefs);

  /* Pass criterion: >= 90% */
  CHK((double)n_pass / (double)e1_nrefs >= E1_PASS_RATE);

  printf("WF-E1: PASS\n");

  /* ---- Cleanup ---- */
  OK(sdis_scene_ref_put(scn));
  OK(sdis_device_ref_put(dev));

  CHK(mem_allocated_size() == 0);
  return 0;
}
