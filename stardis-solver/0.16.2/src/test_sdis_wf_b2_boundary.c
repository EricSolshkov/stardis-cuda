/* WF-B2: Dirichlet + Robin (convection) BC steady-state conduction
 *        (wavefront probe).
 *
 * Scene: unit cube solid, lambda=0.1, delta=0.05
 *   -X face: Dirichlet T_b = 300 K
 *   +X face: Robin BC, H=0.5, fluid temperature T_f = 310 K
 *   Other 4 faces: adiabatic
 *
 * Analytic:
 *   -lambda * dT/dx = H * (T(1) - T_f)
 *   => c1 = H*(T_f - T_b) / (H + lambda) = 0.5*10 / 0.6 = 8.333...
 *   => T(x) = 300 + 8.333*x
 *
 * 11 probes along X (y=0.5, z=0.5), dual validation A+B.
 *
 * Reference CPU test: test_sdis_solve_boundary.c (boundary probes),
 *   here we test internal volume temperature.
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
#define B2_TB      300.0   /* Dirichlet temperature (-X) */
#define B2_TF      310.0   /* Fluid temperature (+X) */
#define B2_H       0.5     /* Convection coefficient */
#define B2_LAMBDA  0.1
#define B2_CP      2.0
#define B2_RHO     25.0
#define B2_DELTA   (1.0 / 20.0)

/* ========================================================================== */
/* Solid shader                                                               */
/* ========================================================================== */
static double
b2_solid_get_calorific_capacity
  (const struct sdis_rwalk_vertex* vtx, struct sdis_data* data)
{
  (void)data;
  CHK(vtx != NULL);
  return B2_CP;
}

static double
b2_solid_get_thermal_conductivity
  (const struct sdis_rwalk_vertex* vtx, struct sdis_data* data)
{
  (void)data;
  CHK(vtx != NULL);
  return B2_LAMBDA;
}

static double
b2_solid_get_volumic_mass
  (const struct sdis_rwalk_vertex* vtx, struct sdis_data* data)
{
  (void)data;
  CHK(vtx != NULL);
  return B2_RHO;
}

static double
b2_solid_get_delta
  (const struct sdis_rwalk_vertex* vtx, struct sdis_data* data)
{
  (void)data;
  CHK(vtx != NULL);
  return B2_DELTA;
}

static double
b2_solid_get_temperature
  (const struct sdis_rwalk_vertex* vtx, struct sdis_data* data)
{
  (void)data;
  CHK(vtx != NULL);
  if(vtx->time > 0)
    return SDIS_TEMPERATURE_NONE;
  else
    return B2_TB;
}

/* ========================================================================== */
/* Fluid shader (convection side, constant temperature Tf)                    */
/* ========================================================================== */
static double
b2_fluid_get_temperature
  (const struct sdis_rwalk_vertex* vtx, struct sdis_data* data)
{
  (void)data;
  CHK(vtx != NULL);
  return B2_TF;
}

static double
b2_fluid_get_calorific_capacity
  (const struct sdis_rwalk_vertex* vtx, struct sdis_data* data)
{
  (void)data;
  CHK(vtx != NULL);
  return B2_CP;
}

static double
b2_fluid_get_volumic_mass
  (const struct sdis_rwalk_vertex* vtx, struct sdis_data* data)
{
  (void)data;
  CHK(vtx != NULL);
  return B2_RHO;
}

/* ========================================================================== */
/* Interface shaders                                                          */
/* ========================================================================== */
struct b2_interf {
  double temperature;
  double hc;
};

static double
b2_interf_get_temperature
  (const struct sdis_interface_fragment* frag, struct sdis_data* data)
{
  const struct b2_interf* p = sdis_data_cget(data);
  CHK(frag && data);
  return p->temperature;
}

static double
b2_interf_get_convection_coef
  (const struct sdis_interface_fragment* frag, struct sdis_data* data)
{
  const struct b2_interf* p = sdis_data_cget(data);
  CHK(frag && data);
  return p->hc;
}

/* ========================================================================== */
/* Analytic solution                                                          */
/* ========================================================================== */
static double
b2_analytic(double x)
{
  double c1 = B2_H * (B2_TF - B2_TB) / (B2_H + B2_LAMBDA);
  return B2_TB + c1 * x;
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
  struct sdis_interface* interf_dirichlet = NULL;
  struct sdis_interface* interf_robin = NULL;
  struct sdis_scene* scn = NULL;
  struct sdis_scene_create_args scn_args = SDIS_SCENE_CREATE_ARGS_DEFAULT;
  struct sdis_fluid_shader fluid_shader = DUMMY_FLUID_SHADER;
  struct sdis_solid_shader solid_shader = DUMMY_SOLID_SHADER;
  struct sdis_interface_shader interf_shader = SDIS_INTERFACE_SHADER_NULL;
  struct sdis_interface* box_interfaces[12];
  struct b2_interf* interf_props = NULL;
  int pass = 0;
  (void)argc; (void)argv;

  printf("=== WF-B2: Dirichlet + Robin BC steady-state (wavefront probe) ===\n");

  OK(sdis_device_create(&SDIS_DEVICE_CREATE_ARGS_DEFAULT, &dev));

  /* ---- Solid medium ---- */
  solid_shader.calorific_capacity = b2_solid_get_calorific_capacity;
  solid_shader.thermal_conductivity = b2_solid_get_thermal_conductivity;
  solid_shader.volumic_mass = b2_solid_get_volumic_mass;
  solid_shader.delta = b2_solid_get_delta;
  solid_shader.temperature = b2_solid_get_temperature;
  OK(sdis_solid_create(dev, &solid_shader, NULL, &solid));

  /* ---- Fluid medium (for convection side) ---- */
  fluid_shader.temperature = b2_fluid_get_temperature;
  fluid_shader.calorific_capacity = b2_fluid_get_calorific_capacity;
  fluid_shader.volumic_mass = b2_fluid_get_volumic_mass;
  OK(sdis_fluid_create(dev, &fluid_shader, NULL, &fluid));

  /* ---- Interface shader ---- */
  interf_shader.convection_coef = b2_interf_get_convection_coef;
  interf_shader.front.temperature = b2_interf_get_temperature;

  /* Adiabatic: T=NONE, hc=0 */
  OK(sdis_data_create(dev, sizeof(struct b2_interf), 16, NULL, &data));
  interf_props = sdis_data_get(data);
  interf_props->temperature = SDIS_TEMPERATURE_NONE;
  interf_props->hc = 0.0;
  OK(sdis_interface_create(dev, solid, fluid, &interf_shader, data,
    &interf_adiabatic));
  OK(sdis_data_ref_put(data));

  /* Dirichlet (-X): T=Tb=300, hc=0 */
  OK(sdis_data_create(dev, sizeof(struct b2_interf), 16, NULL, &data));
  interf_props = sdis_data_get(data);
  interf_props->temperature = B2_TB;
  interf_props->hc = 0.0;
  OK(sdis_interface_create(dev, solid, fluid, &interf_shader, data,
    &interf_dirichlet));
  OK(sdis_data_ref_put(data));

  /* Robin (+X): T=NONE, hc=H=0.5, front=solid, back=fluid */
  {
    struct sdis_interface_shader robin_shader = SDIS_INTERFACE_SHADER_NULL;
    robin_shader.convection_coef = b2_interf_get_convection_coef;
    robin_shader.convection_coef_upper_bound = B2_H;
    robin_shader.front.temperature = b2_interf_get_temperature;

    OK(sdis_data_create(dev, sizeof(struct b2_interf), 16, NULL, &data));
    interf_props = sdis_data_get(data);
    interf_props->temperature = SDIS_TEMPERATURE_NONE;
    interf_props->hc = B2_H;
    OK(sdis_interface_create(dev, solid, fluid, &robin_shader, data,
      &interf_robin));
    OK(sdis_data_ref_put(data));
  }

  /* Release media */
  OK(sdis_medium_ref_put(solid));
  OK(sdis_medium_ref_put(fluid));

  /* ---- Triangle-to-interface mapping ---- */
  /* -Z(0-1), -X(2-3), +Z(4-5), +X(6-7), +Y(8-9), -Y(10-11) */
  box_interfaces[0] = box_interfaces[1] = interf_adiabatic;    /* Front -Z */
  box_interfaces[2] = box_interfaces[3] = interf_dirichlet;     /* Left -X */
  box_interfaces[4] = box_interfaces[5] = interf_adiabatic;    /* Back +Z */
  box_interfaces[6] = box_interfaces[7] = interf_robin;         /* Right +X */
  box_interfaces[8] = box_interfaces[9] = interf_adiabatic;    /* Top +Y */
  box_interfaces[10] = box_interfaces[11] = interf_adiabatic;  /* Bottom -Y */

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
  OK(sdis_interface_ref_put(interf_dirichlet));
  OK(sdis_interface_ref_put(interf_robin));

  /* ---- Run dual-validation probe sweep ---- */
  pass = p0_run_probe_sweep(
    scn, b2_analytic,
    P0_NPROBES, P0_NREALISATIONS,
    1,  /* picard_order */
    SDIS_DIFFUSION_DELTA_SPHERE,
    0.5, 0.5);

  CHK(pass);
  printf("WF-B2: PASS\n");

  /* ---- Cleanup ---- */
  OK(sdis_scene_ref_put(scn));
  OK(sdis_device_ref_put(dev));

  CHK(mem_allocated_size() == 0);
  return 0;
}
