/* WF-A6: Volumic power + convection BC steady-state (wavefront probe).
 *
 * Scene: elongated slab (-LENGTH,-0.5,-LENGTH) to (LENGTH,0.5,LENGTH)
 *   Solid: lambda=100, cp=500000, rho=1000, delta=0.2, P=10000 W/m^3
 *   Bottom (-Y): convection H=50, T_fluid=100 K
 *   Top (+Y):    convection H=50, T_fluid=100 K
 *   Other 4 faces: adiabatic (H=0)
 *
 * Analytic (1D along Y with body source + convection BC):
 *   T(y) = -P/(2*LAMBDA)*y^2 + Tf + P/(2*H) + P/(8*LAMBDA)
 *   At y=0: T_ref = 100 + 10000/100 + 10000/800 = 212.5 K
 *
 * Single probe at origin (0, 0, 0), centre of the slab.
 *
 * Reference CPU test: test_sdis_volumic_power4.c
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
#define A6_TF      100.0
#define A6_POWER   10000.0
#define A6_H       50.0
#define A6_LAMBDA  100.0
#define A6_CP      500000.0
#define A6_RHO     1000.0
#define A6_DELTA   0.2
#define A6_LENGTH  10000.0

#define A6_NREALS  100000

/* ========================================================================== */
/* Geometry: elongated box along X and Z, thin along Y                        */
/* ========================================================================== */
static const double a6_vertices[8 * 3] = {
  -A6_LENGTH, -0.5, -A6_LENGTH,   /*  0 */
   A6_LENGTH, -0.5, -A6_LENGTH,   /*  1 */
  -A6_LENGTH,  0.5, -A6_LENGTH,   /*  2 */
   A6_LENGTH,  0.5, -A6_LENGTH,   /*  3 */
  -A6_LENGTH, -0.5,  A6_LENGTH,   /*  4 */
   A6_LENGTH, -0.5,  A6_LENGTH,   /*  5 */
  -A6_LENGTH,  0.5,  A6_LENGTH,   /*  6 */
   A6_LENGTH,  0.5,  A6_LENGTH    /*  7 */
};
static const size_t a6_nvertices = 8;

/* Same triangulation order as unit box (box_indices from test_sdis_utils.h):
 *   -Z(0-1), -X(2-3), +Z(4-5), +X(6-7), +Y(8-9), -Y(10-11) */
static const size_t a6_indices[12 * 3] = {
  0, 2, 1,  1, 2, 3,     /* -Z: adiabatic */
  0, 4, 2,  2, 4, 6,     /* -X: adiabatic */
  4, 5, 6,  6, 5, 7,     /* +Z: adiabatic */
  3, 7, 5,  5, 1, 3,     /* +X: adiabatic */
  2, 6, 7,  7, 3, 2,     /* +Y: convection (fluid1) */
  0, 1, 5,  5, 4, 0      /* -Y: convection (fluid2) */
};
static const size_t a6_ntriangles = 12;

struct a6_geometry {
  struct sdis_interface** interfaces;
};

static void
a6_get_indices(const size_t itri, size_t ids[3], void* ctx)
{
  (void)ctx;
  CHK(itri < a6_ntriangles);
  ids[0] = a6_indices[itri * 3 + 0];
  ids[1] = a6_indices[itri * 3 + 1];
  ids[2] = a6_indices[itri * 3 + 2];
}

static void
a6_get_position(const size_t ivert, double pos[3], void* ctx)
{
  (void)ctx;
  CHK(ivert < a6_nvertices);
  pos[0] = a6_vertices[ivert * 3 + 0];
  pos[1] = a6_vertices[ivert * 3 + 1];
  pos[2] = a6_vertices[ivert * 3 + 2];
}

static void
a6_get_interface(const size_t itri, struct sdis_interface** bound, void* ctx)
{
  struct a6_geometry* g = ctx;
  CHK(itri < a6_ntriangles && bound);
  *bound = g->interfaces[itri];
}

/* ========================================================================== */
/* Solid shader                                                               */
/* ========================================================================== */
static double
a6_solid_get_calorific_capacity
  (const struct sdis_rwalk_vertex* vtx, struct sdis_data* data)
{
  (void)data;
  CHK(vtx != NULL);
  return A6_CP;
}

static double
a6_solid_get_thermal_conductivity
  (const struct sdis_rwalk_vertex* vtx, struct sdis_data* data)
{
  (void)data;
  CHK(vtx != NULL);
  return A6_LAMBDA;
}

static double
a6_solid_get_volumic_mass
  (const struct sdis_rwalk_vertex* vtx, struct sdis_data* data)
{
  (void)data;
  CHK(vtx != NULL);
  return A6_RHO;
}

static double
a6_solid_get_delta
  (const struct sdis_rwalk_vertex* vtx, struct sdis_data* data)
{
  (void)data;
  CHK(vtx != NULL);
  return A6_DELTA;
}

static double
a6_solid_get_temperature
  (const struct sdis_rwalk_vertex* vtx, struct sdis_data* data)
{
  (void)data;
  CHK(vtx != NULL);
  if(vtx->time > 0)
    return SDIS_TEMPERATURE_NONE;
  else
    return A6_TF;
}

static double
a6_solid_get_volumic_power
  (const struct sdis_rwalk_vertex* vtx, struct sdis_data* data)
{
  (void)data;
  CHK(vtx != NULL);
  return A6_POWER;
}

/* ========================================================================== */
/* Fluid shader (constant temperature Tf)                                     */
/* ========================================================================== */
static double
a6_fluid_get_temperature
  (const struct sdis_rwalk_vertex* vtx, struct sdis_data* data)
{
  (void)data;
  CHK(vtx != NULL);
  return A6_TF;
}

static double
a6_fluid_get_calorific_capacity
  (const struct sdis_rwalk_vertex* vtx, struct sdis_data* data)
{
  (void)data;
  CHK(vtx != NULL);
  return A6_CP;
}

static double
a6_fluid_get_volumic_mass
  (const struct sdis_rwalk_vertex* vtx, struct sdis_data* data)
{
  (void)data;
  CHK(vtx != NULL);
  return A6_RHO;
}

/* ========================================================================== */
/* Interface shaders                                                          */
/* ========================================================================== */
struct a6_interf {
  double temperature;
  double hc;
};

static double
a6_interf_get_temperature
  (const struct sdis_interface_fragment* frag, struct sdis_data* data)
{
  CHK(frag && data);
  return ((const struct a6_interf*)sdis_data_cget(data))->temperature;
}

static double
a6_interf_get_convection_coef
  (const struct sdis_interface_fragment* frag, struct sdis_data* data)
{
  CHK(frag && data);
  return ((const struct a6_interf*)sdis_data_cget(data))->hc;
}

/* ========================================================================== */
/* Test body                                                                  */
/* ========================================================================== */
int
main(int argc, char** argv)
{
  struct sdis_device* dev = NULL;
  struct sdis_data* data = NULL;
  struct sdis_medium* fluid1 = NULL;
  struct sdis_medium* fluid2 = NULL;
  struct sdis_medium* solid = NULL;
  struct sdis_interface* interf_adiabatic = NULL;
  struct sdis_interface* interf_conv_top = NULL;
  struct sdis_interface* interf_conv_bot = NULL;
  struct sdis_scene* scn = NULL;
  struct sdis_scene_create_args scn_args = SDIS_SCENE_CREATE_ARGS_DEFAULT;
  struct sdis_fluid_shader fluid1_shader = DUMMY_FLUID_SHADER;
  struct sdis_fluid_shader fluid2_shader = DUMMY_FLUID_SHADER;
  struct sdis_solid_shader solid_shader = DUMMY_SOLID_SHADER;
  struct sdis_interface_shader interf_shader = SDIS_INTERFACE_SHADER_NULL;
  struct sdis_interface* box_interfaces[12];
  struct a6_interf* interf_props = NULL;
  struct a6_geometry geom;
  struct sdis_estimator *est_wf = NULL, *est_df = NULL;
  struct sdis_solve_probe_args args = SDIS_SOLVE_PROBE_ARGS_DEFAULT;
  double T_ref;
  int pass;
  (void)argc; (void)argv;

  printf("=== WF-A6: Volumic power + convection BC (wavefront probe) ===\n");

  /* Analytic reference at y=0:
   *   T = -P/(2*LAMBDA)*0 + Tf + P/(2*H) + P/(8*LAMBDA)
   *   T = 100 + 100 + 12.5 = 212.5 */
  T_ref = A6_TF + A6_POWER / (2.0 * A6_H) + A6_POWER / (8.0 * A6_LAMBDA);
  printf("  Analytic T(y=0) = %.4f K\n", T_ref);

  OK(sdis_device_create(&SDIS_DEVICE_CREATE_ARGS_DEFAULT, &dev));

  /* ---- Fluids (constant temperature Tf, for convection boundaries) ---- */
  fluid1_shader.temperature = a6_fluid_get_temperature;
  fluid1_shader.calorific_capacity = a6_fluid_get_calorific_capacity;
  fluid1_shader.volumic_mass = a6_fluid_get_volumic_mass;
  OK(sdis_fluid_create(dev, &fluid1_shader, NULL, &fluid1));

  fluid2_shader.temperature = a6_fluid_get_temperature;
  fluid2_shader.calorific_capacity = a6_fluid_get_calorific_capacity;
  fluid2_shader.volumic_mass = a6_fluid_get_volumic_mass;
  OK(sdis_fluid_create(dev, &fluid2_shader, NULL, &fluid2));

  /* ---- Solid (with volumic power) ---- */
  solid_shader.calorific_capacity = a6_solid_get_calorific_capacity;
  solid_shader.thermal_conductivity = a6_solid_get_thermal_conductivity;
  solid_shader.volumic_mass = a6_solid_get_volumic_mass;
  solid_shader.delta = a6_solid_get_delta;
  solid_shader.temperature = a6_solid_get_temperature;
  solid_shader.volumic_power = a6_solid_get_volumic_power;
  OK(sdis_solid_create(dev, &solid_shader, NULL, &solid));

  /* ---- Adiabatic interface (solid/fluid1, H=0) ---- */
  interf_shader.front.temperature = a6_interf_get_temperature;
  interf_shader.convection_coef = a6_interf_get_convection_coef;

  OK(sdis_data_create(dev, sizeof(struct a6_interf),
    ALIGNOF(struct a6_interf), NULL, &data));
  interf_props = sdis_data_get(data);
  interf_props->temperature = SDIS_TEMPERATURE_NONE;
  interf_props->hc = 0.0;
  OK(sdis_interface_create(dev, solid, fluid1, &interf_shader, data,
    &interf_adiabatic));
  OK(sdis_data_ref_put(data));

  /* ---- Convection interface Top (solid/fluid1, H=50) ---- */
  {
    struct sdis_interface_shader conv_shader = SDIS_INTERFACE_SHADER_NULL;
    conv_shader.front.temperature = a6_interf_get_temperature;
    conv_shader.convection_coef = a6_interf_get_convection_coef;
    conv_shader.convection_coef_upper_bound = A6_H;

    OK(sdis_data_create(dev, sizeof(struct a6_interf),
      ALIGNOF(struct a6_interf), NULL, &data));
    interf_props = sdis_data_get(data);
    interf_props->temperature = SDIS_TEMPERATURE_NONE;
    interf_props->hc = A6_H;
    OK(sdis_interface_create(dev, solid, fluid1, &conv_shader, data,
      &interf_conv_top));
    OK(sdis_data_ref_put(data));
  }

  /* ---- Convection interface Bottom (solid/fluid2, H=50) ---- */
  {
    struct sdis_interface_shader conv_shader = SDIS_INTERFACE_SHADER_NULL;
    conv_shader.front.temperature = a6_interf_get_temperature;
    conv_shader.convection_coef = a6_interf_get_convection_coef;
    conv_shader.convection_coef_upper_bound = A6_H;

    OK(sdis_data_create(dev, sizeof(struct a6_interf),
      ALIGNOF(struct a6_interf), NULL, &data));
    interf_props = sdis_data_get(data);
    interf_props->temperature = SDIS_TEMPERATURE_NONE;
    interf_props->hc = A6_H;
    OK(sdis_interface_create(dev, solid, fluid2, &conv_shader, data,
      &interf_conv_bot));
    OK(sdis_data_ref_put(data));
  }

  /* Release media */
  OK(sdis_medium_ref_put(solid));
  OK(sdis_medium_ref_put(fluid1));
  OK(sdis_medium_ref_put(fluid2));

  /* ---- Triangle-to-interface mapping ---- */
  box_interfaces[0] = box_interfaces[1] = interf_adiabatic;     /* -Z */
  box_interfaces[2] = box_interfaces[3] = interf_adiabatic;     /* -X */
  box_interfaces[4] = box_interfaces[5] = interf_adiabatic;     /* +Z */
  box_interfaces[6] = box_interfaces[7] = interf_adiabatic;     /* +X */
  box_interfaces[8] = box_interfaces[9] = interf_conv_top;       /* +Y */
  box_interfaces[10] = box_interfaces[11] = interf_conv_bot;     /* -Y */

  /* ---- Scene ---- */
  geom.interfaces = box_interfaces;

  scn_args.get_indices = a6_get_indices;
  scn_args.get_interface = a6_get_interface;
  scn_args.get_position = a6_get_position;
  scn_args.nprimitives = a6_ntriangles;
  scn_args.nvertices = a6_nvertices;
  scn_args.context = &geom;
  OK(sdis_scene_create(dev, &scn_args, &scn));

  /* Release interfaces */
  OK(sdis_interface_ref_put(interf_adiabatic));
  OK(sdis_interface_ref_put(interf_conv_top));
  OK(sdis_interface_ref_put(interf_conv_bot));

  /* ---- Probe at origin ---- */
  args.nrealisations = A6_NREALS;
  args.position[0] = 0.0;
  args.position[1] = 0.0;
  args.position[2] = 0.0;
  args.picard_order = 1;
  args.diff_algo = SDIS_DIFFUSION_DELTA_SPHERE;

  printf("  Running %lu realisations at (0,0,0) ...\n",
    (unsigned long)A6_NREALS);

  OK(sdis_solve_wavefront_probe(scn, &args, &est_wf));

  pass = p0_compare_analytic(est_wf, T_ref, P0_TOL_SIGMA);

  if(P0_ENABLE_DIAG) {
    OK(sdis_solve_probe(scn, &args, &est_df));
  }
  p0_print_probe_result(0.0, est_wf, est_df, T_ref);

  OK(sdis_estimator_ref_put(est_wf));
  if(est_df) OK(sdis_estimator_ref_put(est_df));

  CHK(pass);
  printf("WF-A6: PASS\n");

  /* ---- Cleanup ---- */
  OK(sdis_scene_ref_put(scn));
  OK(sdis_device_ref_put(dev));

  CHK(mem_allocated_size() == 0);
  return 0;
}
