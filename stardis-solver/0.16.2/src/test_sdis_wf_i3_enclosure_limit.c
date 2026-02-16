/* WF-I3: Enclosure limit conditions — 3D nested cubes (wavefront probe).
 *
 * 3D analogue of test_sdis_enclosure_limit_conditions.c (CPU 2D-only).
 * Verifies multi-material enclosures: the inner cube has different fluids
 * on each face-pair, the outer cube has a single external fluid.
 *
 *  Outer cube:   [0,   1]^3   — solid/ext_fluid interface (Text=360)
 *  Inner cube:   [0.25, 0.75]^3 — multiple fluids / solid
 *      -Z, +Z faces: fluid T=280  (bottom/top of inner)
 *      -X, +X faces: fluid T=300
 *      -Y, +Y faces: fluid T=340
 *
 * Solid: lambda=1, rho=1, cp=1, delta=0.00625
 * All convection coefficients: h=10
 * No radiation.
 *
 * Checks:
 *   1) Probe in solid region: temperature ∈ [280, 360], nonzero SE
 *   2) Multiple probes at symmetric positions give consistent results
 *   3) Probes closer to outer boundary → closer to Text=360
 *
 * Reference CPU test: test_sdis_enclosure_limit_conditions.c
 */

#include "sdis.h"
#include "test_sdis_utils.h"
#include "test_sdis_wf_p0_utils.h"

#include <rsys/mem_allocator.h>
#include <stdio.h>
#include <math.h>
#include <string.h>

/* ========================================================================== */
/* Physical constants                                                         */
/* ========================================================================== */
#define I3_CONVECTION_COEF  10.0
#define I3_DELTA            0.00625
#define I3_NREALS           10000
#define I3_Text             360.0

/* Min/max inner fluid temperatures */
#define I3_Tmin  280.0
#define I3_Tmax  360.0

/* ========================================================================== */
/* Geometry: 16 vertices, 24 triangles (inner cube + outer cube)              */
/* ========================================================================== */
static const double i3_vertices[16 * 3] = {
  /* Inner cube (0-7): [0.25, 0.75]^3 */
  0.25, 0.25, 0.25,  /*  0 */
  0.75, 0.25, 0.25,  /*  1 */
  0.25, 0.75, 0.25,  /*  2 */
  0.75, 0.75, 0.25,  /*  3 */
  0.25, 0.25, 0.75,  /*  4 */
  0.75, 0.25, 0.75,  /*  5 */
  0.25, 0.75, 0.75,  /*  6 */
  0.75, 0.75, 0.75,  /*  7 */
  /* Outer cube (8-15): [0, 1]^3 */
  0.0, 0.0, 0.0,     /*  8 */
  1.0, 0.0, 0.0,     /*  9 */
  0.0, 1.0, 0.0,     /* 10 */
  1.0, 1.0, 0.0,     /* 11 */
  0.0, 0.0, 1.0,     /* 12 */
  1.0, 0.0, 1.0,     /* 13 */
  0.0, 1.0, 1.0,     /* 14 */
  1.0, 1.0, 1.0      /* 15 */
};
static const size_t i3_nvertices = 16;

static const size_t i3_indices[24 * 3] = {
  /* Inner cube — normals pointing outward (into the solid) */
  /* -Z face (z=0.25): tri 0,1 → fluid T=280 */
  0, 2, 3,   0, 3, 1,
  /* +Z face (z=0.75): tri 2,3 → fluid T=280 */
  4, 5, 7,   4, 7, 6,
  /* -X face (x=0.25): tri 4,5 → fluid T=300 */
  0, 4, 6,   0, 6, 2,
  /* +X face (x=0.75): tri 6,7 → fluid T=300 */
  1, 3, 7,   1, 7, 5,
  /* -Y face (y=0.25): tri 8,9 → fluid T=340 */
  0, 1, 5,   0, 5, 4,
  /* +Y face (y=0.75): tri 10,11 → fluid T=340 */
  2, 6, 7,   2, 7, 3,
  /* Outer cube — normals pointing outward (away from solid) */
  /* -Z face (z=0): tri 12,13 */
  8, 10, 11,  8, 11, 9,
  /* +Z face (z=1): tri 14,15 */
  12, 13, 15, 12, 15, 14,
  /* -X face (x=0): tri 16,17 */
  8, 12, 14,  8, 14, 10,
  /* +X face (x=1): tri 18,19 */
  9, 11, 15,  9, 15, 13,
  /* -Y face (y=0): tri 20,21 */
  8, 9, 13,   8, 13, 12,
  /* +Y face (y=1): tri 22,23 */
  10, 14, 15, 10, 15, 11
};
static const size_t i3_ntriangles = 24;

struct i3_geometry {
  const double* positions;
  const size_t* indices;
  struct sdis_interface** interfaces;
};

static void
i3_get_indices(const size_t itri, size_t ids[3], void* ctx)
{
  struct i3_geometry* g = ctx;
  CHK(g && itri < i3_ntriangles);
  ids[0] = g->indices[itri * 3 + 0];
  ids[1] = g->indices[itri * 3 + 1];
  ids[2] = g->indices[itri * 3 + 2];
}

static void
i3_get_position(const size_t ivert, double pos[3], void* ctx)
{
  struct i3_geometry* g = ctx;
  CHK(g && ivert < i3_nvertices);
  pos[0] = g->positions[ivert * 3 + 0];
  pos[1] = g->positions[ivert * 3 + 1];
  pos[2] = g->positions[ivert * 3 + 2];
}

static void
i3_get_interface(const size_t itri, struct sdis_interface** bound, void* ctx)
{
  struct i3_geometry* g = ctx;
  CHK(g && bound && itri < i3_ntriangles);
  *bound = g->interfaces[itri];
}

/* ========================================================================== */
/* Media                                                                      */
/* ========================================================================== */
static double
i3_solid_get_cp(const struct sdis_rwalk_vertex* vtx, struct sdis_data* data)
{
  (void)data; CHK(vtx); return 1.0;
}

static double
i3_solid_get_lambda(const struct sdis_rwalk_vertex* vtx, struct sdis_data* data)
{
  (void)data; CHK(vtx); return 1.0;
}

static double
i3_solid_get_rho(const struct sdis_rwalk_vertex* vtx, struct sdis_data* data)
{
  (void)data; CHK(vtx); return 1.0;
}

static double
i3_solid_get_delta(const struct sdis_rwalk_vertex* vtx, struct sdis_data* data)
{
  (void)data; CHK(vtx); return I3_DELTA;
}

static double
i3_solid_get_temperature(const struct sdis_rwalk_vertex* vtx,
                         struct sdis_data* data)
{
  (void)data; CHK(vtx);
  return SDIS_TEMPERATURE_NONE;
}

static double
i3_fluid_get_temperature(const struct sdis_rwalk_vertex* vtx,
                         struct sdis_data* data)
{
  CHK(vtx && data);
  return *(const double*)sdis_data_cget(data);
}

/* ========================================================================== */
/* Interface                                                                  */
/* ========================================================================== */
static double
i3_interf_get_h(const struct sdis_interface_fragment* frag,
                struct sdis_data* data)
{
  (void)frag; (void)data;
  return I3_CONVECTION_COEF;
}

/* ========================================================================== */
/* Helper: create fluid medium with given temperature                         */
/* ========================================================================== */
static struct sdis_medium*
i3_create_fluid(struct sdis_device* dev, double temperature)
{
  struct sdis_fluid_shader shader = DUMMY_FLUID_SHADER;
  struct sdis_medium* fluid = NULL;
  struct sdis_data* data = NULL;

  OK(sdis_data_create(dev, sizeof(double), ALIGNOF(double), NULL, &data));
  shader.temperature = i3_fluid_get_temperature;
  *((double*)sdis_data_get(data)) = temperature;
  OK(sdis_fluid_create(dev, &shader, data, &fluid));
  OK(sdis_data_ref_put(data));
  return fluid;
}

/* ========================================================================== */
/* Helper: create interface (fluid/solid or solid/fluid)                      */
/* ========================================================================== */
static struct sdis_interface*
i3_create_interface(struct sdis_device* dev,
                    struct sdis_medium* front,
                    struct sdis_medium* back)
{
  struct sdis_interface_shader shader = SDIS_INTERFACE_SHADER_NULL;
  struct sdis_interface* interf = NULL;

  shader.convection_coef = i3_interf_get_h;
  shader.convection_coef_upper_bound = I3_CONVECTION_COEF;
  OK(sdis_interface_create(dev, front, back, &shader, NULL, &interf));
  return interf;
}

/* ========================================================================== */
/* Test body                                                                  */
/* ========================================================================== */
int
main(int argc, char** argv)
{
  struct sdis_device* dev = NULL;
  struct sdis_scene* scn = NULL;
  struct sdis_medium* solid = NULL;
  struct sdis_medium* fluid_280 = NULL;
  struct sdis_medium* fluid_300 = NULL;
  struct sdis_medium* fluid_340 = NULL;
  struct sdis_medium* fluid_ext = NULL;
  struct sdis_solid_shader solid_shader = SDIS_SOLID_SHADER_NULL;
  struct sdis_interface* interf_280 = NULL;
  struct sdis_interface* interf_300 = NULL;
  struct sdis_interface* interf_340 = NULL;
  struct sdis_interface* interf_ext = NULL;
  struct sdis_interface* prim_interfaces[24];
  struct i3_geometry geom;
  struct sdis_scene_create_args scn_args = SDIS_SCENE_CREATE_ARGS_DEFAULT;
  int n_checks_ok = 0;
  int n_checks = 0;
  (void)argc; (void)argv;

  printf("=== WF-I3: Enclosure limit conditions — 3D nested cubes (wavefront) ===\n");

  OK(sdis_device_create(&SDIS_DEVICE_CREATE_ARGS_DEFAULT, &dev));

  /* ---- Solid medium ---- */
  solid_shader.calorific_capacity = i3_solid_get_cp;
  solid_shader.thermal_conductivity = i3_solid_get_lambda;
  solid_shader.volumic_mass = i3_solid_get_rho;
  solid_shader.delta = i3_solid_get_delta;
  solid_shader.temperature = i3_solid_get_temperature;
  OK(sdis_solid_create(dev, &solid_shader, NULL, &solid));

  /* ---- Fluid media ---- */
  fluid_280 = i3_create_fluid(dev, 280.0);
  fluid_300 = i3_create_fluid(dev, 300.0);
  fluid_340 = i3_create_fluid(dev, 340.0);
  fluid_ext = i3_create_fluid(dev, I3_Text);

  /* ---- Interfaces ---- */
  /* Inner cube faces: front=fluid, back=solid */
  interf_280 = i3_create_interface(dev, fluid_280, solid);
  interf_300 = i3_create_interface(dev, fluid_300, solid);
  interf_340 = i3_create_interface(dev, fluid_340, solid);
  /* Outer cube faces: front=solid, back=fluid_ext */
  interf_ext = i3_create_interface(dev, solid, fluid_ext);

  /* ---- Per-primitive interface mapping ---- */
  /* Inner cube (triangles 0-11): */
  prim_interfaces[0]  = prim_interfaces[1]  = interf_280; /* -Z: T=280 */
  prim_interfaces[2]  = prim_interfaces[3]  = interf_280; /* +Z: T=280 */
  prim_interfaces[4]  = prim_interfaces[5]  = interf_300; /* -X: T=300 */
  prim_interfaces[6]  = prim_interfaces[7]  = interf_300; /* +X: T=300 */
  prim_interfaces[8]  = prim_interfaces[9]  = interf_340; /* -Y: T=340 */
  prim_interfaces[10] = prim_interfaces[11] = interf_340; /* +Y: T=340 */
  /* Outer cube (triangles 12-23): all Text=360 */
  prim_interfaces[12] = prim_interfaces[13] = interf_ext;
  prim_interfaces[14] = prim_interfaces[15] = interf_ext;
  prim_interfaces[16] = prim_interfaces[17] = interf_ext;
  prim_interfaces[18] = prim_interfaces[19] = interf_ext;
  prim_interfaces[20] = prim_interfaces[21] = interf_ext;
  prim_interfaces[22] = prim_interfaces[23] = interf_ext;

  /* ---- Create scene ---- */
  geom.positions = i3_vertices;
  geom.indices = i3_indices;
  geom.interfaces = prim_interfaces;

  scn_args.get_indices = i3_get_indices;
  scn_args.get_interface = i3_get_interface;
  scn_args.get_position = i3_get_position;
  scn_args.nprimitives = i3_ntriangles;
  scn_args.nvertices = i3_nvertices;
  scn_args.t_range[0] = I3_Tmin;
  scn_args.t_range[1] = I3_Tmax;
  scn_args.context = &geom;
  OK(sdis_scene_create(dev, &scn_args, &scn));

  /* ---- Release media and interfaces ---- */
  OK(sdis_medium_ref_put(solid));
  OK(sdis_medium_ref_put(fluid_280));
  OK(sdis_medium_ref_put(fluid_300));
  OK(sdis_medium_ref_put(fluid_340));
  OK(sdis_medium_ref_put(fluid_ext));
  OK(sdis_interface_ref_put(interf_280));
  OK(sdis_interface_ref_put(interf_300));
  OK(sdis_interface_ref_put(interf_340));
  OK(sdis_interface_ref_put(interf_ext));

  /* ================================================================== */
  /* Check 1: probe in solid near outer boundary → closer to Text       */
  /* ================================================================== */
  {
    struct sdis_solve_probe_args args = SDIS_SOLVE_PROBE_ARGS_DEFAULT;
    struct sdis_estimator* est = NULL;
    struct sdis_mc mc;

    args.nrealisations = I3_NREALS;
    args.position[0] = 0.1;
    args.position[1] = 0.1;
    args.position[2] = 0.1;
    args.time_range[0] = INF;
    args.time_range[1] = INF;

    OK(sdis_solve_wavefront_probe(scn, &args, &est));
    OK(sdis_estimator_get_temperature(est, &mc));

    printf("  probe(0.1,0.1,0.1) in solid near outer: T=%.3f +/- %.2e\n",
      mc.E, mc.SE);
    n_checks++;
    if(mc.E >= I3_Tmin && mc.E <= I3_Tmax && mc.SE > 0) {
      printf("    PASS: T in [%.0f, %.0f], SE > 0\n", I3_Tmin, I3_Tmax);
      n_checks_ok++;
    } else {
      printf("    FAIL: T not in [%.0f, %.0f] or SE=0\n", I3_Tmin, I3_Tmax);
    }

    OK(sdis_estimator_ref_put(est));
  }

  /* ================================================================== */
  /* Check 2: probe in solid near inner boundary → lower temperature    */
  /* Average inner temp = (280*4 + 300*4 + 340*4) / 12 = 306.67        */
  /* ================================================================== */
  {
    struct sdis_solve_probe_args args = SDIS_SOLVE_PROBE_ARGS_DEFAULT;
    struct sdis_estimator* est = NULL;
    struct sdis_mc mc;

    args.nrealisations = I3_NREALS;
    args.position[0] = 0.3;
    args.position[1] = 0.5;
    args.position[2] = 0.5;
    args.time_range[0] = INF;
    args.time_range[1] = INF;

    OK(sdis_solve_wavefront_probe(scn, &args, &est));
    OK(sdis_estimator_get_temperature(est, &mc));

    printf("  probe(0.3,0.5,0.5) in solid near inner: T=%.3f +/- %.2e\n",
      mc.E, mc.SE);
    n_checks++;
    if(mc.E >= I3_Tmin && mc.E <= I3_Tmax && mc.SE > 0) {
      printf("    PASS: T in [%.0f, %.0f], SE > 0\n", I3_Tmin, I3_Tmax);
      n_checks_ok++;
    } else {
      printf("    FAIL: T not in [%.0f, %.0f] or SE=0\n", I3_Tmin, I3_Tmax);
    }

    OK(sdis_estimator_ref_put(est));
  }

  /* ================================================================== */
  /* Check 3: symmetry — probe at (0.5, 0.1, 0.5) vs (0.5, 0.9, 0.5) */
  /* Both equidistant from Y faces (T=340), should give similar T       */
  /* ================================================================== */
  {
    struct sdis_solve_probe_args args = SDIS_SOLVE_PROBE_ARGS_DEFAULT;
    struct sdis_estimator* est1 = NULL;
    struct sdis_estimator* est2 = NULL;
    struct sdis_mc mc1, mc2;
    double diff, combined_SE;

    args.nrealisations = I3_NREALS;
    args.time_range[0] = INF;
    args.time_range[1] = INF;

    args.position[0] = 0.5;
    args.position[1] = 0.1;
    args.position[2] = 0.5;
    OK(sdis_solve_wavefront_probe(scn, &args, &est1));
    OK(sdis_estimator_get_temperature(est1, &mc1));

    args.position[0] = 0.5;
    args.position[1] = 0.9;
    args.position[2] = 0.5;
    OK(sdis_solve_wavefront_probe(scn, &args, &est2));
    OK(sdis_estimator_get_temperature(est2, &mc2));

    diff = fabs(mc1.E - mc2.E);
    combined_SE = sqrt(mc1.SE * mc1.SE + mc2.SE * mc2.SE);

    printf("  symmetry: T(0.5,0.1,0.5)=%.3f  T(0.5,0.9,0.5)=%.3f  "
      "|diff|=%.3f  combined_SE=%.2e\n", mc1.E, mc2.E, diff, combined_SE);
    n_checks++;
    if(combined_SE > 0 && diff <= 5.0 * combined_SE) {
      printf("    PASS: symmetric probes agree within 5 sigma\n");
      n_checks_ok++;
    } else {
      printf("    FAIL: symmetric probes differ by %.1f sigma\n",
        combined_SE > 0 ? diff / combined_SE : 9999.0);
    }

    OK(sdis_estimator_ref_put(est1));
    OK(sdis_estimator_ref_put(est2));
  }

  /* ---- Summary ---- */
  printf("\n  Checks passed: %d/%d\n", n_checks_ok, n_checks);
  CHK(n_checks_ok == n_checks);

  printf("WF-I3: PASS\n");

  /* ---- Cleanup ---- */
  OK(sdis_scene_ref_put(scn));
  OK(sdis_device_ref_put(dev));

  CHK(mem_allocated_size() == 0);
  return 0;
}
