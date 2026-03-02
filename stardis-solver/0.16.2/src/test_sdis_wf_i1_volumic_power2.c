/* WF-I1: Nested volumic power (wavefront probe, 3D).
 *
 * Scene (identical to CPU test_sdis_volumic_power2.c):
 *   Outer cuboid: [-0.5,0.5] x [-1,1] x [-0.5,0.5]   (solid1)
 *   Inner cube:   [-0.1,0.1] x [0.4,0.6] x [-0.5,0.5] (solid2)
 *
 *   solid1: lambda=1, Cp=500000, rho=1000, delta=0.01, no volumic power
 *   solid2: lambda=10, Cp=500000, rho=1000, delta=0.01, Pw=10000
 *
 *   fluid1: T=373.15K (100 Celsius)
 *   fluid2: T=273.15K (0 Celsius)
 *
 *   Interfaces (per-rectangle, 18 rectangles = 36 triangles):
 *     rect 0: solid1/fluid1 adiabatic  (left)
 *     rect 1: solid1/fluid1 convective h=5 (top)
 *     rect 2: solid1/fluid1 adiabatic  (right)
 *     rect 3: solid1/fluid2 convective h=10 (bottom)
 *     rect 4-5: solid1/fluid1 adiabatic (front/back outer)
 *     rect 6-11: solid1/fluid1 adiabatic (front/back inner ring)
 *     rect 12-15: solid1/solid2 (inner cube 4 side faces)
 *     rect 16-17: solid2/fluid1 adiabatic (inner cube back/front)
 *
 * Reference: EDF Syrthès industrial solver (3D values).
 *   Temperatures in Celsius. Converted to Kelvin by +273.15K.
 *   Tolerance: max(3*sigma, 5K) per probe — generous for external reference.
 *
 * 8 probes at (0, y, 0) for y in {0.85, 0.65, 0.45, 0.25, 0.05, -0.15,
 * -0.35, -0.55}. These positions are all in solid1 (outside the inner cube).
 *
 * Verification: wavefront result vs Syrthès reference.
 *   Pass criterion: >= 90% of probes pass within tolerance.
 *
 * Reference CPU test: test_sdis_volumic_power2.c
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
#define I1_N       10000
#define I1_PW      10000.0
#define I1_DELTA   0.01
#define I1_TOL_K   5.0    /* fixed tolerance in Kelvin for external ref */
#define I1_TOL_SIGMA 3.0
#define I1_PASS_RATE 0.90

/* ========================================================================== */
/* Geometry (identical to CPU test)                                            */
/* ========================================================================== */
static const double i1_vertices[16 * 3] = {
  -0.5, -1.0, -0.5,
  -0.5,  1.0, -0.5,
   0.5,  1.0, -0.5,
   0.5, -1.0, -0.5,
  -0.5, -1.0,  0.5,
  -0.5,  1.0,  0.5,
   0.5,  1.0,  0.5,
   0.5, -1.0,  0.5,
  -0.1,  0.4, -0.5,
  -0.1,  0.6, -0.5,
   0.1,  0.6, -0.5,
   0.1,  0.4, -0.5,
  -0.1,  0.4,  0.5,
  -0.1,  0.6,  0.5,
   0.1,  0.6,  0.5,
   0.1,  0.4,  0.5
};
static const size_t i1_nvertices = 16;

static const size_t i1_indices[36 * 3] = {
  0, 4, 5,  5, 1, 0,     /* rect 0: Cuboid left    (tri  0, 1) */
  1, 5, 6,  6, 2, 1,     /* rect 1: Cuboid top     (tri  2, 3) */
  6, 7, 3,  3, 2, 6,     /* rect 2: Cuboid right   (tri  4, 5) */
  0, 3, 7,  7, 4, 0,     /* rect 3: Cuboid bottom  (tri  6, 7) */
  /* Cuboid back (4 regions around inner cube opening) */
  0, 1, 9,  9, 8, 0,     /* rect 4 (tri  8, 9) */
  1, 2, 10, 10, 9, 1,    /* rect 5 (tri 10,11) */
  2, 3, 11, 11, 10, 2,   /* rect 6 (tri 12,13) */
  3, 0, 8,  8, 11, 3,    /* rect 7 (tri 14,15) */
  /* Cuboid front (4 regions around inner cube opening) */
  5, 4, 12, 12, 13, 5,   /* rect 8  (tri 16,17) */
  5, 13, 14, 14, 6, 5,   /* rect 9  (tri 18,19) */
  6, 14, 15, 15, 7, 6,   /* rect 10 (tri 20,21) */
  7, 15, 12, 12, 4, 7,   /* rect 11 (tri 22,23) */
  8, 12, 13, 13, 9, 8,   /* rect 12: Cube left   (tri 24,25) */
  9, 13, 14, 14, 10, 9,  /* rect 13: Cube top    (tri 26,27) */
  14, 15, 11, 11, 10, 14,/* rect 14: Cube right  (tri 28,29) */
  8, 11, 15, 15, 12, 8,  /* rect 15: Cube bottom (tri 30,31) */
  8, 9, 10,  10, 11, 8,  /* rect 16: Cube back   (tri 32,33) */
  12, 15, 14, 14, 13, 12 /* rect 17: Cube front  (tri 34,35) */
};
static const size_t i1_ntriangles = 36;

/* ========================================================================== */
/* Geometry callbacks                                                         */
/* ========================================================================== */
struct i1_geometry {
  struct sdis_interface** interfaces;
};

static void
i1_get_indices(const size_t itri, size_t ids[3], void* ctx)
{
  (void)ctx;
  CHK(itri < i1_ntriangles);
  ids[0] = i1_indices[itri * 3 + 0];
  ids[1] = i1_indices[itri * 3 + 1];
  ids[2] = i1_indices[itri * 3 + 2];
}

static void
i1_get_position(const size_t ivert, double pos[3], void* ctx)
{
  (void)ctx;
  CHK(ivert < i1_nvertices);
  pos[0] = i1_vertices[ivert * 3 + 0];
  pos[1] = i1_vertices[ivert * 3 + 1];
  pos[2] = i1_vertices[ivert * 3 + 2];
}

static void
i1_get_interface(const size_t itri, struct sdis_interface** bound, void* ctx)
{
  struct i1_geometry* g = ctx;
  CHK(g && bound && itri < i1_ntriangles);
  /* Map triangle to rectangle: itri/2 */
  *bound = g->interfaces[itri / 2];
}

/* ========================================================================== */
/* Solid medium                                                               */
/* ========================================================================== */
struct i1_solid {
  double cp;
  double lambda;
  double rho;
  double delta;
  double P;
  double T;
};

static double
i1_solid_cp(const struct sdis_rwalk_vertex* vtx, struct sdis_data* data)
{
  CHK(vtx && data);
  return ((const struct i1_solid*)sdis_data_cget(data))->cp;
}

static double
i1_solid_lambda(const struct sdis_rwalk_vertex* vtx, struct sdis_data* data)
{
  CHK(vtx && data);
  return ((const struct i1_solid*)sdis_data_cget(data))->lambda;
}

static double
i1_solid_rho(const struct sdis_rwalk_vertex* vtx, struct sdis_data* data)
{
  CHK(vtx && data);
  return ((const struct i1_solid*)sdis_data_cget(data))->rho;
}

static double
i1_solid_delta(const struct sdis_rwalk_vertex* vtx, struct sdis_data* data)
{
  CHK(vtx && data);
  return ((const struct i1_solid*)sdis_data_cget(data))->delta;
}

static double
i1_solid_temperature(const struct sdis_rwalk_vertex* vtx, struct sdis_data* data)
{
  CHK(vtx && data);
  return ((const struct i1_solid*)sdis_data_cget(data))->T;
}

static double
i1_solid_volumic_power(const struct sdis_rwalk_vertex* vtx,
                       struct sdis_data* data)
{
  CHK(vtx && data);
  return ((const struct i1_solid*)sdis_data_cget(data))->P;
}

/* ========================================================================== */
/* Fluid medium                                                               */
/* ========================================================================== */
struct i1_fluid {
  double temperature;
};

static double
i1_fluid_temperature(const struct sdis_rwalk_vertex* vtx,
                     struct sdis_data* data)
{
  CHK(vtx && data);
  return ((const struct i1_fluid*)sdis_data_cget(data))->temperature;
}

/* ========================================================================== */
/* Interface                                                                  */
/* ========================================================================== */
struct i1_interf {
  double h;
  double temperature;
};

static double
i1_interf_h(const struct sdis_interface_fragment* frag, struct sdis_data* data)
{
  CHK(frag && data);
  return ((const struct i1_interf*)sdis_data_cget(data))->h;
}

static double
i1_interf_temperature(const struct sdis_interface_fragment* frag,
                      struct sdis_data* data)
{
  CHK(frag && data);
  return ((const struct i1_interf*)sdis_data_cget(data))->temperature;
}

/* ========================================================================== */
/* Reference data: EDF Syrthes 3D values (in Celsius, from CPU test)          */
/* ========================================================================== */
struct i1_probe_ref {
  double pos[3];
  double T_celsius;  /* Syrthes 3D reference in Celsius */
};

/* Check 1: lambda1=1, lambda2=10, Pw=10000 */
static const struct i1_probe_ref i1_refs1[] = {
  {{0, 0.85, 0}, 189.13},
  {{0, 0.65, 0}, 247.09},
  {{0, 0.45, 0}, 308.42},
  {{0, 0.25, 0}, 233.55},
  {{0, 0.05, 0}, 192.30},
  {{0,-0.15, 0}, 156.98},
  {{0,-0.35, 0}, 123.43},
  {{0,-0.55, 0},  90.040}
};
static const size_t i1_nrefs1 = sizeof(i1_refs1) / sizeof(i1_refs1[0]);

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
  struct sdis_medium* solid1 = NULL;
  struct sdis_medium* solid2 = NULL;
  struct sdis_scene* scn = NULL;
  struct sdis_scene_create_args scn_args = SDIS_SCENE_CREATE_ARGS_DEFAULT;
  struct sdis_fluid_shader fluid_shader = SDIS_FLUID_SHADER_NULL;
  struct sdis_solid_shader solid_shader = SDIS_SOLID_SHADER_NULL;
  struct sdis_interface_shader interf_shader = SDIS_INTERFACE_SHADER_NULL;
  struct sdis_interface* interf_solid1_adiabatic = NULL;
  struct sdis_interface* interf_solid2_adiabatic = NULL;
  struct sdis_interface* interf_solid1_solid2 = NULL;
  struct sdis_interface* interf_solid1_fluid1 = NULL;
  struct sdis_interface* interf_solid1_fluid2 = NULL;
  struct sdis_interface* interfaces[18]; /* 18 rectangles */
  struct i1_geometry geom;
  struct i1_solid* sp = NULL;
  struct i1_fluid* fp = NULL;
  struct i1_interf* ip = NULL;
  int n_pass = 0;
  size_t i;
  FILE* csv = NULL;
  (void)argc; (void)argv;

  printf("=== WF-I1: Nested volumic power — Syrthes reference (wavefront) ===\n");

  OK(sdis_device_create(&SDIS_DEVICE_CREATE_ARGS_DEFAULT, &dev));

  /* ---- Fluid shaders ---- */
  fluid_shader.temperature = i1_fluid_temperature;
  fluid_shader.calorific_capacity = dummy_medium_getter;
  fluid_shader.volumic_mass = dummy_medium_getter;

  /* ---- Create fluid1 (T=373.15K = 100C) ---- */
  OK(sdis_data_create(dev, sizeof(struct i1_fluid),
    ALIGNOF(struct i1_fluid), NULL, &data));
  fp = sdis_data_get(data);
  fp->temperature = 373.15;
  OK(sdis_fluid_create(dev, &fluid_shader, data, &fluid1));
  OK(sdis_data_ref_put(data));

  /* ---- Create fluid2 (T=273.15K = 0C) ---- */
  OK(sdis_data_create(dev, sizeof(struct i1_fluid),
    ALIGNOF(struct i1_fluid), NULL, &data));
  fp = sdis_data_get(data);
  fp->temperature = 273.15;
  OK(sdis_fluid_create(dev, &fluid_shader, data, &fluid2));
  OK(sdis_data_ref_put(data));

  /* ---- Solid shaders ---- */
  solid_shader.calorific_capacity = i1_solid_cp;
  solid_shader.thermal_conductivity = i1_solid_lambda;
  solid_shader.volumic_mass = i1_solid_rho;
  solid_shader.delta = i1_solid_delta;
  solid_shader.temperature = i1_solid_temperature;
  solid_shader.volumic_power = i1_solid_volumic_power;

  /* ---- Create solid1 (lambda=1, no volumic power) ---- */
  OK(sdis_data_create(dev, sizeof(struct i1_solid),
    ALIGNOF(struct i1_solid), NULL, &data));
  sp = sdis_data_get(data);
  sp->cp = 500000.0;
  sp->rho = 1000.0;
  sp->lambda = 1.0;
  sp->delta = I1_DELTA;
  sp->P = SDIS_VOLUMIC_POWER_NONE;
  sp->T = SDIS_TEMPERATURE_NONE;
  OK(sdis_solid_create(dev, &solid_shader, data, &solid1));
  OK(sdis_data_ref_put(data));

  /* ---- Create solid2 (lambda=10, Pw=10000) ---- */
  OK(sdis_data_create(dev, sizeof(struct i1_solid),
    ALIGNOF(struct i1_solid), NULL, &data));
  sp = sdis_data_get(data);
  sp->cp = 500000.0;
  sp->rho = 1000.0;
  sp->lambda = 10.0;
  sp->delta = I1_DELTA;
  sp->P = I1_PW;
  sp->T = SDIS_TEMPERATURE_NONE;
  OK(sdis_solid_create(dev, &solid_shader, data, &solid2));
  OK(sdis_data_ref_put(data));

  /* ---- Create solid1/solid2 interface (no shader) ---- */
  OK(sdis_interface_create(dev, solid2, solid1, &SDIS_INTERFACE_SHADER_NULL,
    NULL, &interf_solid1_solid2));

  /* ---- Interface shader for convective boundaries ---- */
  interf_shader.convection_coef = i1_interf_h;

  /* ---- Create adiabatic interfaces ---- */
  OK(sdis_data_create(dev, sizeof(struct i1_interf),
    ALIGNOF(struct i1_interf), NULL, &data));
  ip = sdis_data_get(data);
  ip->h = 0;
  ip->temperature = SDIS_TEMPERATURE_NONE;
  OK(sdis_interface_create(dev, solid1, fluid1, &interf_shader, data,
    &interf_solid1_adiabatic));
  OK(sdis_interface_create(dev, solid2, fluid1, &interf_shader, data,
    &interf_solid2_adiabatic));
  OK(sdis_data_ref_put(data));

  /* ---- Interface shader for temperature boundaries ---- */
  interf_shader.front.temperature = i1_interf_temperature;

  /* ---- Create solid1/fluid1 (h=5) ---- */
  OK(sdis_data_create(dev, sizeof(struct i1_interf),
    ALIGNOF(struct i1_interf), NULL, &data));
  ip = sdis_data_get(data);
  ip->h = 5;
  ip->temperature = SDIS_TEMPERATURE_NONE;
  OK(sdis_interface_create(dev, solid1, fluid1, &interf_shader, data,
    &interf_solid1_fluid1));
  OK(sdis_data_ref_put(data));

  /* ---- Create solid1/fluid2 (h=10) ---- */
  OK(sdis_data_create(dev, sizeof(struct i1_interf),
    ALIGNOF(struct i1_interf), NULL, &data));
  ip = sdis_data_get(data);
  ip->h = 10;
  ip->temperature = SDIS_TEMPERATURE_NONE;
  OK(sdis_interface_create(dev, solid1, fluid2, &interf_shader, data,
    &interf_solid1_fluid2));
  OK(sdis_data_ref_put(data));

  /* ---- Per-rectangle interface mapping (identical to CPU test) ---- */
  interfaces[0]  = interf_solid1_adiabatic;  /* left */
  interfaces[1]  = interf_solid1_fluid1;     /* top (h=5, fluid1) */
  interfaces[2]  = interf_solid1_adiabatic;  /* right */
  interfaces[3]  = interf_solid1_fluid2;     /* bottom (h=10, fluid2) */
  interfaces[4]  = interf_solid1_adiabatic;  /* back outer ring */
  interfaces[5]  = interf_solid1_adiabatic;
  interfaces[6]  = interf_solid1_adiabatic;
  interfaces[7]  = interf_solid1_adiabatic;
  interfaces[8]  = interf_solid1_adiabatic;  /* front outer ring */
  interfaces[9]  = interf_solid1_adiabatic;
  interfaces[10] = interf_solid1_adiabatic;
  interfaces[11] = interf_solid1_adiabatic;
  interfaces[12] = interf_solid1_solid2;     /* inner cube left */
  interfaces[13] = interf_solid1_solid2;     /* inner cube top */
  interfaces[14] = interf_solid1_solid2;     /* inner cube right */
  interfaces[15] = interf_solid1_solid2;     /* inner cube bottom */
  interfaces[16] = interf_solid2_adiabatic;  /* inner cube back */
  interfaces[17] = interf_solid2_adiabatic;  /* inner cube front */

  /* ---- Create scene ---- */
  geom.interfaces = interfaces;
  scn_args.get_indices = i1_get_indices;
  scn_args.get_interface = i1_get_interface;
  scn_args.get_position = i1_get_position;
  scn_args.nprimitives = i1_ntriangles;
  scn_args.nvertices = i1_nvertices;
  scn_args.context = &geom;
  OK(sdis_scene_create(dev, &scn_args, &scn));

  /* ================================================================== */
  /* Check 1: lambda1=1, lambda2=10, Pw=10000                          */
  /* Reference: EDF Syrthes 3D values (in Celsius)                     */
  /* ================================================================== */
  printf("  Check 1: lambda1=1, lambda2=10, Pw=%g\n", I1_PW);
  csv = csv_open("I1");
  printf("  Running %lu probes, %d realisations each ...\n",
    (unsigned long)i1_nrefs1, I1_N);

  for(i = 0; i < i1_nrefs1; i++) {
    struct sdis_solve_probe_args args = SDIS_SOLVE_PROBE_ARGS_DEFAULT;
    struct sdis_estimator* est_wf = NULL;
    struct sdis_mc mc;
    double ref_K;
    double tol;
    int pass;

    args.nrealisations = I1_N;
    args.position[0] = i1_refs1[i].pos[0];
    args.position[1] = i1_refs1[i].pos[1];
    args.position[2] = i1_refs1[i].pos[2];
    args.time_range[0] = INF;
    args.time_range[1] = INF;

    OK(sdis_solve_wavefront_probe(scn, &args, &est_wf));
    OK(sdis_estimator_get_temperature(est_wf, &mc));

    /* Reference in Kelvin */
    ref_K = i1_refs1[i].T_celsius + 273.15;

    /* Tolerance: max(3*SE, I1_TOL_K)
     * The Syrthes reference itself has numerical uncertainty, so we use a
     * generous fixed tolerance of 5K combined with statistical tolerance. */
    tol = I1_TOL_SIGMA * mc.SE;
    if(tol < I1_TOL_K) tol = I1_TOL_K;
    pass = fabs(mc.E - ref_K) <= tol;
    n_pass += pass;

    /* CSV: primary DS row + complementary WoS variant */
    {
      char csv_sub[32];
      sprintf(csv_sub, "y=%.2f", i1_refs1[i].pos[1]);
      csv_row(csv, "I1", csv_sub, "gpu_wf", "DS",
              i1_refs1[i].pos[0], i1_refs1[i].pos[1], i1_refs1[i].pos[2],
              INF, 1, I1_N, mc.E, mc.SE, ref_K);

    }

    printf("  (0, %+.2f, 0)  wf=%.2fK (%.2fC)  SE=%.2e  "
      "ref=%.2fC  diff=%.2fK  tol=%.2fK  %s\n",
      i1_refs1[i].pos[1],
      mc.E, mc.E - 273.15, mc.SE,
      i1_refs1[i].T_celsius,
      fabs(mc.E - ref_K), tol,
      pass ? "PASS" : "FAIL");

    OK(sdis_estimator_ref_put(est_wf));
  }

  csv_close(csv);

  /* ---- Summary ---- */
  printf("\n  Primary:    %d/%lu probes pass (%.1f%%)\n",
    n_pass, (unsigned long)i1_nrefs1,
    100.0 * (double)n_pass / (double)i1_nrefs1);
  CHK((double)n_pass / (double)i1_nrefs1 >= I1_PASS_RATE);

  printf("WF-I1: PASS\n");

  /* ---- Cleanup ---- */
  OK(sdis_interface_ref_put(interf_solid1_adiabatic));
  OK(sdis_interface_ref_put(interf_solid2_adiabatic));
  OK(sdis_interface_ref_put(interf_solid1_fluid1));
  OK(sdis_interface_ref_put(interf_solid1_fluid2));
  OK(sdis_interface_ref_put(interf_solid1_solid2));
  OK(sdis_medium_ref_put(fluid1));
  OK(sdis_medium_ref_put(fluid2));
  OK(sdis_medium_ref_put(solid1));
  OK(sdis_medium_ref_put(solid2));
  OK(sdis_scene_ref_put(scn));
  OK(sdis_device_ref_put(dev));

  CHK(mem_allocated_size() == 0);
  return 0;
}
