/* WF-A3: Dual-material + contact thermal resistance (wavefront probe).
 *
 * Scene: two-slab solid [0,X0] with lambda1=0.1, [X0,L] with lambda2=0.2
 *   Contact resistance R at x=X0=3, L=4
 *   -X face (x=0): Dirichlet T0=0
 *   +X face (x=L): Dirichlet TL=100
 *   Other faces: adiabatic
 *
 * Analytic (1D steady-state with contact resistance):
 *   q  = (TL - T0) / (X0/LAMBDA1 + R + (L-X0)/LAMBDA2)
 *   solid1: T(x)  = T0 + q/LAMBDA1 * x           (x <= X0)
 *   solid2: T(x)  = TL - q/LAMBDA2 * (L - x)     (x >= X0)
 *
 * Fixed R grid: {0.01, 0.1, 1, 10, 100} to cover 4 decades.
 * For each R, 11 probes along X (y=L/2, z=L/2).
 *
 * Reference CPU test: test_sdis_contact_resistance.c
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
#define A3_T0       0.0
#define A3_TL       100.0
#define A3_LAMBDA1  0.1
#define A3_LAMBDA2  0.2
#define A3_L        4.0
#define A3_X0       3.0
#define A3_CP       2.0
#define A3_RHO      25.0
#define A3_DELTA1   (A3_X0 / 25.0)
#define A3_DELTA2   ((A3_L - A3_X0) / 25.0)

#define A3_NREALS   10000

/* ========================================================================== */
/* Geometry: 12 vertices, 22 triangles (2 slabs sharing face at x=X0)         */
/* Directly from test_sdis_contact_resistance.h                               */
/* ========================================================================== */
static const double a3_vertices[12 * 3] = {
  0,       0,       0,        /*  0 */
  A3_X0,   0,       0,        /*  1 */
  A3_L,    0,       0,        /*  2 */
  0,       A3_L,    0,        /*  3 */
  A3_X0,   A3_L,    0,        /*  4 */
  A3_L,    A3_L,    0,        /*  5 */
  0,       0,       A3_L,     /*  6 */
  A3_X0,   0,       A3_L,     /*  7 */
  A3_L,    0,       A3_L,     /*  8 */
  0,       A3_L,    A3_L,     /*  9 */
  A3_X0,   A3_L,    A3_L,     /* 10 */
  A3_L,    A3_L,    A3_L      /* 11 */
};
static const size_t a3_nvertices = 12;

static const size_t a3_indices[22 * 3] = {
  /* -Z face (4 triangles) */
  0, 3, 1,  1, 3, 4,     /* tri 0,1: solid1 part */
  1, 4, 2,  2, 4, 5,     /* tri 2,3: solid2 part */
  /* -X face (2 triangles) */
  0, 6, 3,  3, 6, 9,     /* tri 4,5: solid1 */
  /* +Z face (4 triangles) */
  6, 7, 9,  9, 7, 10,    /* tri 6,7: solid1 part */
  7, 8, 10, 10, 8, 11,   /* tri 8,9: solid2 part */
  /* +X face (2 triangles) */
  5, 11, 8,  8, 2, 5,    /* tri 10,11: solid2 */
  /* +Y face (4 triangles) */
  3, 9, 10, 10, 4, 3,    /* tri 12,13: solid1 part */
  4, 10, 11, 11, 5, 4,   /* tri 14,15: solid2 part */
  /* -Y face (4 triangles) */
  0, 1, 7,  7, 6, 0,     /* tri 16,17: solid1 part */
  1, 2, 8,  8, 7, 1,     /* tri 18,19: solid2 part */
  /* Internal contact face at x=X0 (2 triangles) */
  4, 10, 7,  7, 1, 4     /* tri 20,21: solid1↔solid2 contact */
};
static const size_t a3_ntriangles = 22;

struct a3_geometry {
  const double* positions;
  const size_t* indices;
  struct sdis_interface** interfaces;
};

static void
a3_get_indices(const size_t itri, size_t ids[3], void* ctx)
{
  struct a3_geometry* g = ctx;
  CHK(itri < a3_ntriangles);
  ids[0] = g->indices[itri * 3 + 0];
  ids[1] = g->indices[itri * 3 + 1];
  ids[2] = g->indices[itri * 3 + 2];
}

static void
a3_get_position(const size_t ivert, double pos[3], void* ctx)
{
  struct a3_geometry* g = ctx;
  CHK(ivert < a3_nvertices);
  pos[0] = g->positions[ivert * 3 + 0];
  pos[1] = g->positions[ivert * 3 + 1];
  pos[2] = g->positions[ivert * 3 + 2];
}

static void
a3_get_interface(const size_t itri, struct sdis_interface** bound, void* ctx)
{
  struct a3_geometry* g = ctx;
  CHK(itri < a3_ntriangles && bound);
  *bound = g->interfaces[itri];
}

/* ========================================================================== */
/* Media shaders                                                              */
/* ========================================================================== */
struct a3_solid {
  double lambda;
  double delta;
};

static double
a3_solid_get_calorific_capacity
  (const struct sdis_rwalk_vertex* vtx, struct sdis_data* data)
{
  (void)data;
  CHK(vtx != NULL);
  return A3_CP;
}

static double
a3_solid_get_thermal_conductivity
  (const struct sdis_rwalk_vertex* vtx, struct sdis_data* data)
{
  CHK(vtx != NULL);
  return ((struct a3_solid*)sdis_data_cget(data))->lambda;
}

static double
a3_solid_get_volumic_mass
  (const struct sdis_rwalk_vertex* vtx, struct sdis_data* data)
{
  (void)data;
  CHK(vtx != NULL);
  return A3_RHO;
}

static double
a3_solid_get_delta
  (const struct sdis_rwalk_vertex* vtx, struct sdis_data* data)
{
  CHK(vtx != NULL);
  return ((struct a3_solid*)sdis_data_cget(data))->delta;
}

static double
a3_solid_get_temperature
  (const struct sdis_rwalk_vertex* vtx, struct sdis_data* data)
{
  (void)data;
  CHK(vtx != NULL);
  if(vtx->time > 0)
    return SDIS_TEMPERATURE_NONE;
  else
    return (A3_T0 + A3_TL) / 2.0;
}

/* ========================================================================== */
/* Interface shaders                                                          */
/* ========================================================================== */
struct a3_interf {
  double temperature;
  double resistance;
};

static double
a3_interface_get_temperature
  (const struct sdis_interface_fragment* frag, struct sdis_data* data)
{
  const struct a3_interf* p = sdis_data_cget(data);
  CHK(frag && data);
  return p->temperature;
}

static double
a3_interface_get_convection_coef
  (const struct sdis_interface_fragment* frag, struct sdis_data* data)
{
  CHK(frag && data);
  (void)frag; (void)data;
  return 0.0;
}

static double
a3_interface_get_contact_resistance
  (const struct sdis_interface_fragment* frag, struct sdis_data* data)
{
  const struct a3_interf* p = sdis_data_cget(data);
  CHK(frag && data);
  return p->resistance;
}

/* ========================================================================== */
/* Analytic solution for given R                                              */
/* ========================================================================== */
struct a3_analytic_ctx {
  double R;
  double q;
  double ref_L;  /* T at x=X0 from solid1 side */
  double ref_R;  /* T at x=X0 from solid2 side */
};

static void
a3_compute_analytic(double R, struct a3_analytic_ctx* ctx)
{
  /* Heat flux through the composite slab:
   *   q = (TL - T0) / (X0/LAMBDA1 + R + (L-X0)/LAMBDA2) */
  ctx->R = R;
  ctx->q = (A3_TL - A3_T0) / (A3_X0 / A3_LAMBDA1 + R + (A3_L - A3_X0) / A3_LAMBDA2);

  /* Temperature at x=X0 from left (solid1) side */
  ctx->ref_L = A3_T0 + ctx->q / A3_LAMBDA1 * A3_X0;

  /* Temperature at x=X0 from right (solid2) side */
  ctx->ref_R = A3_TL - ctx->q / A3_LAMBDA2 * (A3_L - A3_X0);
}

static double
a3_analytic_temperature(const struct a3_analytic_ctx* ctx, double x)
{
  if(x <= A3_X0) {
    /* Solid1: T(x) = T0 + q/LAMBDA1 * x */
    return A3_T0 + ctx->q / A3_LAMBDA1 * x;
  } else {
    /* Solid2: T(x) = TL - q/LAMBDA2 * (L - x) */
    return A3_TL - ctx->q / A3_LAMBDA2 * (A3_L - x);
  }
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
  struct sdis_medium* solid1 = NULL;
  struct sdis_medium* solid2 = NULL;
  struct sdis_interface* interf_adiab1 = NULL;
  struct sdis_interface* interf_adiab2 = NULL;
  struct sdis_interface* interf_T0 = NULL;
  struct sdis_interface* interf_TL = NULL;
  struct sdis_interface* interf_R = NULL;
  struct sdis_scene* scn = NULL;
  struct sdis_scene_create_args scn_args = SDIS_SCENE_CREATE_ARGS_DEFAULT;
  struct sdis_fluid_shader fluid_shader = DUMMY_FLUID_SHADER;
  struct sdis_solid_shader solid1_shader = DUMMY_SOLID_SHADER;
  struct sdis_solid_shader solid2_shader = DUMMY_SOLID_SHADER;
  struct sdis_interface_shader interf_shader = SDIS_INTERFACE_SHADER_NULL;
  struct sdis_interface* prim_interfaces[22];
  struct a3_interf* interf_R_props = NULL;
  struct a3_interf* interf_props = NULL;
  struct a3_geometry geom;
  int n_pass_total = 0;
  int n_probes_total = 0;
  size_t ri;
  (void)argc; (void)argv;

  /* 5 fixed R values spanning 4 decades (same range as CPU test's
   * r = 10^uniform(-2,2) ∈ [0.01, 100]) */
  static const double R_values[] = { 0.01, 0.1, 1.0, 10.0, 100.0 };
  static const size_t nR = sizeof(R_values) / sizeof(R_values[0]);

  /* Probe positions — matching CPU pattern: probes placed inside each solid
   * with 5% margin from boundaries.
   *   solid1: x ∈ [0.05*X0, 0.95*X0] = [0.15, 2.85]
   *   solid2: x ∈ [X0+0.05*(L-X0), X0+0.95*(L-X0)] = [3.05, 3.95]
   * 4 probes per solid, 8 per R, alternating solid1/solid2 as CPU does. */
#define A3_NPROBES_PER_SOLID 4
  static const double a3_x_solid1[A3_NPROBES_PER_SOLID] = {
    0.15 + 0.5 * (2.85 - 0.15) / A3_NPROBES_PER_SOLID,  /* 0.4875 */
    0.15 + 1.5 * (2.85 - 0.15) / A3_NPROBES_PER_SOLID,  /* 1.1625 */
    0.15 + 2.5 * (2.85 - 0.15) / A3_NPROBES_PER_SOLID,  /* 1.8375 */
    0.15 + 3.5 * (2.85 - 0.15) / A3_NPROBES_PER_SOLID   /* 2.5125 */
  };
  static const double a3_x_solid2[A3_NPROBES_PER_SOLID] = {
    3.05 + 0.5 * (3.95 - 3.05) / A3_NPROBES_PER_SOLID,  /* 3.1625 */
    3.05 + 1.5 * (3.95 - 3.05) / A3_NPROBES_PER_SOLID,  /* 3.3875 */
    3.05 + 2.5 * (3.95 - 3.05) / A3_NPROBES_PER_SOLID,  /* 3.6125 */
    3.05 + 3.5 * (3.95 - 3.05) / A3_NPROBES_PER_SOLID   /* 3.8375 */
  };

  printf("=== WF-A3: Contact resistance steady-state (wavefront probe) ===\n");

  OK(sdis_device_create(&SDIS_DEVICE_CREATE_ARGS_DEFAULT, &dev));

  /* ---- Fluid (dummy) ---- */
  OK(sdis_fluid_create(dev, &fluid_shader, NULL, &fluid));

  /* ---- Solid 1 (lambda1=0.1) ---- */
  solid1_shader.calorific_capacity = a3_solid_get_calorific_capacity;
  solid1_shader.thermal_conductivity = a3_solid_get_thermal_conductivity;
  solid1_shader.volumic_mass = a3_solid_get_volumic_mass;
  solid1_shader.delta = a3_solid_get_delta;
  solid1_shader.temperature = a3_solid_get_temperature;
  OK(sdis_data_create(dev, sizeof(struct a3_solid),
    ALIGNOF(struct a3_solid), NULL, &data));
  ((struct a3_solid*)sdis_data_get(data))->lambda = A3_LAMBDA1;
  ((struct a3_solid*)sdis_data_get(data))->delta = A3_DELTA1;
  OK(sdis_solid_create(dev, &solid1_shader, data, &solid1));
  OK(sdis_data_ref_put(data));

  /* ---- Solid 2 (lambda2=0.2) ---- */
  solid2_shader.calorific_capacity = a3_solid_get_calorific_capacity;
  solid2_shader.thermal_conductivity = a3_solid_get_thermal_conductivity;
  solid2_shader.volumic_mass = a3_solid_get_volumic_mass;
  solid2_shader.delta = a3_solid_get_delta;
  solid2_shader.temperature = a3_solid_get_temperature;
  OK(sdis_data_create(dev, sizeof(struct a3_solid),
    ALIGNOF(struct a3_solid), NULL, &data));
  ((struct a3_solid*)sdis_data_get(data))->lambda = A3_LAMBDA2;
  ((struct a3_solid*)sdis_data_get(data))->delta = A3_DELTA2;
  OK(sdis_solid_create(dev, &solid2_shader, data, &solid2));
  OK(sdis_data_ref_put(data));

  /* ---- Interface shader for boundary faces ---- */
  interf_shader.front.temperature = a3_interface_get_temperature;
  interf_shader.back.temperature = a3_interface_get_temperature;
  interf_shader.convection_coef = a3_interface_get_convection_coef;

  /* Adiabatic 1 (solid1/fluid) */
  OK(sdis_data_create(dev, sizeof(struct a3_interf),
    ALIGNOF(struct a3_interf), NULL, &data));
  interf_props = sdis_data_get(data);
  interf_props->temperature = SDIS_TEMPERATURE_NONE;
  interf_props->resistance = 0.0;
  OK(sdis_interface_create(dev, solid1, fluid, &interf_shader, data,
    &interf_adiab1));
  OK(sdis_data_ref_put(data));

  /* Adiabatic 2 (solid2/fluid) */
  OK(sdis_data_create(dev, sizeof(struct a3_interf),
    ALIGNOF(struct a3_interf), NULL, &data));
  interf_props = sdis_data_get(data);
  interf_props->temperature = SDIS_TEMPERATURE_NONE;
  interf_props->resistance = 0.0;
  OK(sdis_interface_create(dev, solid2, fluid, &interf_shader, data,
    &interf_adiab2));
  OK(sdis_data_ref_put(data));

  /* T0 (solid1/fluid, left): T=0 */
  OK(sdis_data_create(dev, sizeof(struct a3_interf),
    ALIGNOF(struct a3_interf), NULL, &data));
  interf_props = sdis_data_get(data);
  interf_props->temperature = A3_T0;
  interf_props->resistance = 0.0;
  OK(sdis_interface_create(dev, solid1, fluid, &interf_shader, data,
    &interf_T0));
  OK(sdis_data_ref_put(data));

  /* TL (solid2/fluid, right): T=100 */
  OK(sdis_data_create(dev, sizeof(struct a3_interf),
    ALIGNOF(struct a3_interf), NULL, &data));
  interf_props = sdis_data_get(data);
  interf_props->temperature = A3_TL;
  interf_props->resistance = 0.0;
  OK(sdis_interface_create(dev, solid2, fluid, &interf_shader, data,
    &interf_TL));
  OK(sdis_data_ref_put(data));

  /* ---- Contact interface (solid1/solid2) — mutable R via sdis_data ---- */
  /* Like CPU test: one interface, change R through mutable pointer. */
  {
    struct sdis_interface_shader r_shader = SDIS_INTERFACE_SHADER_NULL;
    r_shader.front.temperature = a3_interface_get_temperature;
    r_shader.back.temperature = a3_interface_get_temperature;
    r_shader.thermal_contact_resistance = a3_interface_get_contact_resistance;

    OK(sdis_data_create(dev, sizeof(struct a3_interf),
      ALIGNOF(struct a3_interf), NULL, &data));
    interf_props = sdis_data_get(data);
    interf_props->temperature = SDIS_TEMPERATURE_NONE;
    interf_props->resistance = 0.0;
    OK(sdis_interface_create(dev, solid1, solid2, &r_shader, data,
      &interf_R));
    OK(sdis_data_ref_put(data));
  }

  /* Mutable pointer — updated before each R group */
  interf_R_props = sdis_data_get(sdis_interface_get_data(interf_R));

  /* ---- Triangle-to-interface mapping (22 triangles) ---- */
  /* -Z (tri 0-3): solid1 part 0,1 + solid2 part 2,3 */
  prim_interfaces[0] = interf_adiab1;
  prim_interfaces[1] = interf_adiab1;
  prim_interfaces[2] = interf_adiab2;
  prim_interfaces[3] = interf_adiab2;
  /* -X (tri 4-5): solid1 left face = T0 */
  prim_interfaces[4] = interf_T0;
  prim_interfaces[5] = interf_T0;
  /* +Z (tri 6-9): solid1 part 6,7 + solid2 part 8,9 */
  prim_interfaces[6] = interf_adiab1;
  prim_interfaces[7] = interf_adiab1;
  prim_interfaces[8] = interf_adiab2;
  prim_interfaces[9] = interf_adiab2;
  /* +X (tri 10-11): solid2 right face = TL */
  prim_interfaces[10] = interf_TL;
  prim_interfaces[11] = interf_TL;
  /* +Y (tri 12-15): solid1 part 12,13 + solid2 part 14,15 */
  prim_interfaces[12] = interf_adiab1;
  prim_interfaces[13] = interf_adiab1;
  prim_interfaces[14] = interf_adiab2;
  prim_interfaces[15] = interf_adiab2;
  /* -Y (tri 16-19): solid1 part 16,17 + solid2 part 18,19 */
  prim_interfaces[16] = interf_adiab1;
  prim_interfaces[17] = interf_adiab1;
  prim_interfaces[18] = interf_adiab2;
  prim_interfaces[19] = interf_adiab2;
  /* Internal contact (tri 20-21) */
  prim_interfaces[20] = interf_R;
  prim_interfaces[21] = interf_R;

  /* ---- Scene (created once, R changed via mutable pointer) ---- */
  geom.positions = a3_vertices;
  geom.indices = a3_indices;
  geom.interfaces = prim_interfaces;

  scn_args.get_indices = a3_get_indices;
  scn_args.get_interface = a3_get_interface;
  scn_args.get_position = a3_get_position;
  scn_args.nprimitives = a3_ntriangles;
  scn_args.nvertices = a3_nvertices;
  scn_args.context = &geom;
  OK(sdis_scene_create(dev, &scn_args, &scn));

  /* ---- Loop over R values ---- */
  for(ri = 0; ri < nR; ri++) {
    double R = R_values[ri];
    struct a3_analytic_ctx ctx;
    int n_pass_primary = 0;
    int n_probes = 0;
    size_t i;

    /* Update resistance via mutable pointer (like CPU test) */
    interf_R_props->resistance = R;

    printf("\n  R = %.2e ...\n", R);
    a3_compute_analytic(R, &ctx);
    printf("  q=%.4f, ref_L=%.4f, ref_R=%.4f\n", ctx.q, ctx.ref_L, ctx.ref_R);
    fprintf(stdout, "  Running %d probes (4 solid1 + 4 solid2), "
      "%d realisations each ...\n", 2 * A3_NPROBES_PER_SOLID, A3_NREALS);

    /* ---- Sweep: alternating solid1/solid2 (like CPU isimul%2) ---- */
    for(i = 0; i < 2 * (size_t)A3_NPROBES_PER_SOLID; i++) {
      double x, T_ref;
      struct sdis_solve_probe_args args = SDIS_SOLVE_PROBE_ARGS_DEFAULT;
      struct sdis_estimator *est_wf = NULL, *est_df = NULL;

      if(i % 2) {
        /* Odd: solid1, x ∈ [0.05*X0, 0.95*X0] */
        x = a3_x_solid1[i / 2];
        T_ref = a3_analytic_temperature(&ctx, x);
      } else {
        /* Even: solid2, x ∈ [X0+0.05*(L-X0), X0+0.95*(L-X0)] */
        x = a3_x_solid2[i / 2];
        T_ref = a3_analytic_temperature(&ctx, x);
      }

      args.nrealisations = A3_NREALS;
      args.position[0] = x;
      args.position[1] = A3_L / 2.0;
      args.position[2] = A3_L / 2.0;
      args.picard_order = 1;
      args.diff_algo = SDIS_DIFFUSION_DELTA_SPHERE;

      OK(sdis_solve_wavefront_probe(scn, &args, &est_wf));
      n_pass_primary += p0_compare_analytic(est_wf, T_ref, P0_TOL_SIGMA);
      n_probes++;

      if(P0_ENABLE_DIAG) {
        OK(sdis_solve_probe(scn, &args, &est_df));
      }
      p0_print_probe_result(x, est_wf, est_df, T_ref);

      OK(sdis_estimator_ref_put(est_wf));
      if(est_df) OK(sdis_estimator_ref_put(est_df));
    }

    fprintf(stdout, "  R=%.2e  Primary: %d/%d pass\n",
      R, n_pass_primary, n_probes);

    n_pass_total += n_pass_primary;
    n_probes_total += n_probes;
  }

  fprintf(stdout, "\n  Overall: %d/%d pass\n", n_pass_total, n_probes_total);
  CHK(n_probes_total > 0);
  CHK((double)n_pass_total / (double)n_probes_total >= P0_PASS_RATE);
  printf("WF-A3: PASS\n");

  /* ---- Cleanup ---- */
  OK(sdis_scene_ref_put(scn));
  OK(sdis_interface_ref_put(interf_R));
  OK(sdis_interface_ref_put(interf_adiab1));
  OK(sdis_interface_ref_put(interf_adiab2));
  OK(sdis_interface_ref_put(interf_T0));
  OK(sdis_interface_ref_put(interf_TL));
  OK(sdis_medium_ref_put(solid1));
  OK(sdis_medium_ref_put(solid2));
  OK(sdis_medium_ref_put(fluid));
  OK(sdis_device_ref_put(dev));

  CHK(mem_allocated_size() == 0);
  return 0;
}
