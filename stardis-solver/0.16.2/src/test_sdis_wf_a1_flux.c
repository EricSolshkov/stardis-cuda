/* WF-A1: Fixed T + fixed flux steady-state conduction (wavefront probe).
 *
 * Scene: unit cube solid, lambda=0.1, delta=0.05
 *   -X face: imposed flux PHI = 10 W/m^2
 *   +X face: imposed temperature T0 = 320 K
 *   Other 4 faces: adiabatic
 *
 * Analytic: T(x) = T0 + (1-x) * PHI / LAMBDA
 *           T(x) = 320 + 100*(1-x)
 *
 * 11 probes along X (y=0.5, z=0.5), dual validation A+B.
 *
 * Reference CPU test: test_sdis_flux.c
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
#define A1_PHI     10.0
#define A1_T0      320.0
#define A1_LAMBDA  0.1
#define A1_CP      2.0
#define A1_RHO     25.0
#define A1_DELTA   (1.0 / 20.0)

/* ========================================================================== */
/* Media shaders                                                              */
/* ========================================================================== */
static double
a1_solid_get_calorific_capacity
  (const struct sdis_rwalk_vertex* vtx, struct sdis_data* data)
{
  (void)data;
  CHK(vtx != NULL);
  return A1_CP;
}

static double
a1_solid_get_thermal_conductivity
  (const struct sdis_rwalk_vertex* vtx, struct sdis_data* data)
{
  (void)data;
  CHK(vtx != NULL);
  return A1_LAMBDA;
}

static double
a1_solid_get_volumic_mass
  (const struct sdis_rwalk_vertex* vtx, struct sdis_data* data)
{
  (void)data;
  CHK(vtx != NULL);
  return A1_RHO;
}

static double
a1_solid_get_delta
  (const struct sdis_rwalk_vertex* vtx, struct sdis_data* data)
{
  (void)data;
  CHK(vtx != NULL);
  return A1_DELTA;
}

static double
a1_solid_get_temperature
  (const struct sdis_rwalk_vertex* vtx, struct sdis_data* data)
{
  (void)data;
  CHK(vtx != NULL);
  if(vtx->time > 0)
    return SDIS_TEMPERATURE_NONE;
  else
    return A1_T0;
}

/* ========================================================================== */
/* Interface shaders                                                          */
/* ========================================================================== */
struct a1_interf {
  double temperature;
  double phi;
};

static double
a1_interface_get_temperature
  (const struct sdis_interface_fragment* frag, struct sdis_data* data)
{
  const struct a1_interf* p = sdis_data_cget(data);
  CHK(frag && data);
  return p->temperature;
}

static double
a1_interface_get_flux
  (const struct sdis_interface_fragment* frag, struct sdis_data* data)
{
  const struct a1_interf* p = sdis_data_cget(data);
  CHK(frag && data);
  return p->phi;
}

/* ========================================================================== */
/* Analytic solution                                                          */
/* ========================================================================== */
static double
a1_analytic(double x)
{
  return A1_T0 + (1.0 - x) * A1_PHI / A1_LAMBDA;
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
  struct sdis_interface* interf_phi = NULL;
  struct sdis_scene* scn = NULL;
  struct sdis_scene_create_args scn_args = SDIS_SCENE_CREATE_ARGS_DEFAULT;
  struct sdis_fluid_shader fluid_shader = DUMMY_FLUID_SHADER;
  struct sdis_solid_shader solid_shader = DUMMY_SOLID_SHADER;
  struct sdis_interface_shader interf_shader = SDIS_INTERFACE_SHADER_NULL;
  struct sdis_interface* box_interfaces[12];
  struct a1_interf* interf_props = NULL;
  int pass = 0;
  (void)argc; (void)argv;

  printf("=== WF-A1: Flux BC steady-state conduction (wavefront probe) ===\n");

  OK(sdis_device_create(&SDIS_DEVICE_CREATE_ARGS_DEFAULT, &dev));

  /* ---- Media ---- */
  OK(sdis_fluid_create(dev, &fluid_shader, NULL, &fluid));

  solid_shader.calorific_capacity = a1_solid_get_calorific_capacity;
  solid_shader.thermal_conductivity = a1_solid_get_thermal_conductivity;
  solid_shader.volumic_mass = a1_solid_get_volumic_mass;
  solid_shader.delta = a1_solid_get_delta;
  solid_shader.temperature = a1_solid_get_temperature;
  OK(sdis_solid_create(dev, &solid_shader, NULL, &solid));

  /* ---- Interface shader (temperature + flux) ---- */
  interf_shader.front.temperature = a1_interface_get_temperature;
  interf_shader.front.flux = a1_interface_get_flux;

  /* Adiabatic: T=NONE, phi=0 */
  OK(sdis_data_create(dev, sizeof(struct a1_interf), 16, NULL, &data));
  interf_props = sdis_data_get(data);
  interf_props->temperature = SDIS_TEMPERATURE_NONE;
  interf_props->phi = 0.0;
  OK(sdis_interface_create(dev, solid, fluid, &interf_shader, data,
    &interf_adiabatic));
  OK(sdis_data_ref_put(data));

  /* T0: T=320, phi=0 (unused) */
  OK(sdis_data_create(dev, sizeof(struct a1_interf), 16, NULL, &data));
  interf_props = sdis_data_get(data);
  interf_props->temperature = A1_T0;
  interf_props->phi = 0.0;
  OK(sdis_interface_create(dev, solid, fluid, &interf_shader, data,
    &interf_t0));
  OK(sdis_data_ref_put(data));

  /* PHI: T=NONE, phi=10 */
  OK(sdis_data_create(dev, sizeof(struct a1_interf), 16, NULL, &data));
  interf_props = sdis_data_get(data);
  interf_props->temperature = SDIS_TEMPERATURE_NONE;
  interf_props->phi = A1_PHI;
  OK(sdis_interface_create(dev, solid, fluid, &interf_shader, data,
    &interf_phi));
  OK(sdis_data_ref_put(data));

  /* Release media (scene keeps refs) */
  OK(sdis_medium_ref_put(solid));
  OK(sdis_medium_ref_put(fluid));

  /* ---- Triangle-to-interface mapping (12 triangles, 6 faces) ---- */
  /* test_sdis_utils.h box: -Z(0-1), -X(2-3), +Z(4-5), +X(6-7),
   *                        +Y(8-9), -Y(10-11) */
  box_interfaces[0] = box_interfaces[1] = interf_adiabatic;  /* Front -Z */
  box_interfaces[2] = box_interfaces[3] = interf_phi;         /* Left -X: flux */
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
  OK(sdis_interface_ref_put(interf_phi));

  /* ---- Run dual-validation probe sweep ---- */
  pass = p0_run_probe_sweep(
    scn, a1_analytic,
    P0_NPROBES, P0_NREALISATIONS,
    1,  /* picard_order */
    SDIS_DIFFUSION_DELTA_SPHERE,
    0.5, 0.5);  /* y, z fixed */

  CHK(pass);
  printf("WF-A1: PASS\n");

  /* ---- Cleanup ---- */
  OK(sdis_scene_ref_put(scn));
  OK(sdis_device_ref_put(dev));

  CHK(mem_allocated_size() == 0);
  return 0;
}
