/* WF-B5: Trilinear temperature field with supershape (wavefront probe).
 *
 * Scene: non-convex supershape boundary (s3dut, 256x128)
 *   f0: A=1.5, B=1, M=11, N0=1, N1=1, N2=2
 *   f1: A=1,   B=2, M=3.6, N0=1, N1=2, N2=0.7
 *   radius = 1
 *
 * The supershape is immersed in a trilinear temperature field:
 *   T(x,y,z) = 333*x' + 432*y' + 579*z'
 *   where q' = (q - lower)/(upper - lower),  lower=-3, upper=+3
 *
 * The boundary temperature is set to the analytic trilinear profile.
 * Interior probes should recover the same profile by MC estimation.
 *
 * Material: lambda=25, Cp=500, rho=7500, delta=adaptive via medium_spread
 *
 * 10 probes at deterministic positions near the origin (all well inside the
 * supershape, at radius < 0.2 from origin).
 *
 * Reference: analytic trilinear profile (exact closed-form).
 * Tolerance: >= 95% of probes within 3 sigma of analytic.
 *
 * Reference CPU test: test_sdis_solve_probe_list.c
 */

#include "sdis.h"
#include "test_sdis_utils.h"
#include "test_sdis_wf_p0_utils.h"
#include "test_sdis_csv_utils.h"

#include <star/s3dut.h>
#include <rsys_math.h>
#include <stdio.h>
#include <math.h>

/* ========================================================================== */
/* Constants (identical to CPU test)                                           */
/* ========================================================================== */
#define B5_LAMBDA   25.0
#define B5_CP       500.0
#define B5_RHO      7500.0
#define B5_NREALS   10000
#define B5_TOL_SIGMA  3.0
#define B5_PASS_RATE  0.95

/* Trilinear profile range (identical to CPU test) */
#define B5_LOWER  (-3.0)
#define B5_UPPER  (+3.0)

/* ========================================================================== */
/* Trilinear profile (identical to CPU test)                                  */
/* ========================================================================== */
static double
trilinear_profile(const double pos[3])
{
  const double a = 333.0;
  const double b = 432.0;
  const double c = 579.0;
  double x, y, z;
  CHK(pos);
  x = (pos[0] - B5_LOWER) / (B5_UPPER - B5_LOWER);
  y = (pos[1] - B5_LOWER) / (B5_UPPER - B5_LOWER);
  z = (pos[2] - B5_LOWER) / (B5_UPPER - B5_LOWER);
  return a*x + b*y + c*z;
}

/* ========================================================================== */
/* Deterministic probe positions (well inside supersh, radius < 0.15)         */
/* ========================================================================== */
#define B5_NPROBES 10

static const double b5_positions[B5_NPROBES][3] = {
  { 0.00,  0.00,  0.00},
  { 0.10,  0.05, -0.05},
  {-0.10,  0.10,  0.00},
  { 0.05, -0.10,  0.08},
  {-0.08,  0.06,  0.04},
  { 0.00, -0.12,  0.00},
  { 0.07,  0.07,  0.07},
  {-0.05, -0.05, -0.05},
  { 0.12,  0.00,  0.00},
  { 0.00,  0.00, -0.10}
};

/* ========================================================================== */
/* Geometry (supershape via s3dut, identical to CPU test)                      */
/* ========================================================================== */
struct b5_context {
  struct s3dut_mesh_data msh;
  struct sdis_interface* interf;
};

static void
b5_get_indices(const size_t itri, size_t ids[3], void* context)
{
  const struct b5_context* ctx = context;
  CHK(ids && ctx && itri < ctx->msh.nprimitives);
  /* Flip winding ids[1]<->ids[2] so normals point inward (into the solid) */
  ids[0] = ctx->msh.indices[itri*3+0];
  ids[2] = ctx->msh.indices[itri*3+1];
  ids[1] = ctx->msh.indices[itri*3+2];
}

static void
b5_get_position(const size_t ivert, double pos[3], void* context)
{
  const struct b5_context* ctx = context;
  CHK(pos && ctx && ivert < ctx->msh.nvertices);
  pos[0] = ctx->msh.positions[ivert*3+0];
  pos[1] = ctx->msh.positions[ivert*3+1];
  pos[2] = ctx->msh.positions[ivert*3+2];
}

static void
b5_get_interface(const size_t itri, struct sdis_interface** bound, void* context)
{
  const struct b5_context* ctx = context;
  CHK(bound && ctx && itri < ctx->msh.nprimitives);
  *bound = ctx->interf;
}

/* ========================================================================== */
/* Interface shader: boundary temperature = trilinear profile                 */
/* ========================================================================== */
static double
b5_interface_get_temperature(const struct sdis_interface_fragment* frag,
                             struct sdis_data* data)
{
  (void)data;
  CHK(frag);
  return trilinear_profile(frag->P);
}

/* ========================================================================== */
/* Solid shader (same material as CPU test)                                   */
/* ========================================================================== */
struct b5_solid_data {
  double delta;
};

static double
b5_solid_cp(const struct sdis_rwalk_vertex* v, struct sdis_data* d)
{
  (void)v; (void)d; return B5_CP;
}

static double
b5_solid_lambda(const struct sdis_rwalk_vertex* v, struct sdis_data* d)
{
  (void)v; (void)d; return B5_LAMBDA;
}

static double
b5_solid_rho(const struct sdis_rwalk_vertex* v, struct sdis_data* d)
{
  (void)v; (void)d; return B5_RHO;
}

static double
b5_solid_delta(const struct sdis_rwalk_vertex* v, struct sdis_data* d)
{
  CHK(v && d);
  return ((const struct b5_solid_data*)sdis_data_cget(d))->delta;
}

static double
b5_solid_temp(const struct sdis_rwalk_vertex* v, struct sdis_data* d)
{
  (void)v; (void)d;
  return SDIS_TEMPERATURE_NONE;
}

/* ========================================================================== */
/* Fluid shader (dummy environment surrounding the supershape)                */
/* ========================================================================== */
static double
b5_fluid_temp(const struct sdis_rwalk_vertex* vtx, struct sdis_data* data)
{
  (void)vtx; (void)data;
  return 350.0;
}

/* ========================================================================== */
/* Test body                                                                  */
/* ========================================================================== */
int
main(int argc, char** argv)
{
  struct sdis_device* dev = NULL;
  struct sdis_medium* fluid = NULL;
  struct sdis_medium* solid = NULL;
  struct sdis_interface* interf = NULL;
  struct sdis_scene* scn = NULL;
  struct sdis_data* solid_dat = NULL;
  struct b5_solid_data* sp = NULL;
  struct s3dut_super_formula f0 = S3DUT_SUPER_FORMULA_NULL;
  struct s3dut_super_formula f1 = S3DUT_SUPER_FORMULA_NULL;
  struct s3dut_mesh* msh = NULL;
  struct sdis_scene_create_args scn_args = SDIS_SCENE_CREATE_ARGS_DEFAULT;
  struct sdis_solid_shader solid_shader = SDIS_SOLID_SHADER_NULL;
  struct sdis_fluid_shader fluid_shader = DUMMY_FLUID_SHADER;
  struct sdis_interface_shader interf_shader = SDIS_INTERFACE_SHADER_NULL;
  struct b5_context ctx;
  double spread;
  int n_pass = 0;
  int i;
  FILE* csv = NULL;
  (void)argc; (void)argv;

  printf("=== WF-B5: Trilinear field with supershape (wavefront probe) ===\n");

  OK(sdis_device_create(&SDIS_DEVICE_CREATE_ARGS_DEFAULT, &dev));

  /* ---- Fluid (dummy environment) ---- */
  fluid_shader.temperature = b5_fluid_temp;
  OK(sdis_fluid_create(dev, &fluid_shader, NULL, &fluid));

  /* ---- Solid with sdis_data for adaptive delta ---- */
  OK(sdis_data_create(dev, sizeof(struct b5_solid_data),
    ALIGNOF(struct b5_solid_data), NULL, &solid_dat));
  sp = sdis_data_get(solid_dat);
  sp->delta = 0.01; /* will be updated after scene creation */

  solid_shader.calorific_capacity = b5_solid_cp;
  solid_shader.thermal_conductivity = b5_solid_lambda;
  solid_shader.volumic_mass = b5_solid_rho;
  solid_shader.delta = b5_solid_delta;
  solid_shader.temperature = b5_solid_temp;
  OK(sdis_solid_create(dev, &solid_shader, solid_dat, &solid));
  OK(sdis_data_ref_put(solid_dat));

  /* ---- Interface: boundary temperature = trilinear profile ---- */
  interf_shader.front.temperature = b5_interface_get_temperature;
  interf_shader.back.temperature = b5_interface_get_temperature;
  OK(sdis_interface_create(dev, solid, fluid, &interf_shader, NULL, &interf));

  /* ---- Supershape geometry (identical to CPU test parameters) ---- */
  f0.A = 1.5; f0.B = 1; f0.M = 11.0; f0.N0 = 1; f0.N1 = 1; f0.N2 = 2.0;
  f1.A = 1.0; f1.B = 2; f1.M =  3.6; f1.N0 = 1; f1.N1 = 2; f1.N2 = 0.7;
  OK(s3dut_create_super_shape(NULL, &f0, &f1, 1, 256, 128, &msh));
  OK(s3dut_mesh_get_data(msh, &ctx.msh));
  ctx.interf = interf;

  scn_args.get_indices = b5_get_indices;
  scn_args.get_position = b5_get_position;
  scn_args.get_interface = b5_get_interface;
  scn_args.nprimitives = ctx.msh.nprimitives;
  scn_args.nvertices = ctx.msh.nvertices;
  scn_args.context = &ctx;
  OK(sdis_scene_create(dev, &scn_args, &scn));

  OK(s3dut_mesh_ref_put(msh));

  /* ---- Adaptive delta (same strategy as CPU: 4V/S / 30) ---- */
  sp = sdis_data_get(sdis_medium_get_data(solid));
  OK(sdis_scene_get_medium_spread(scn, solid, &spread));
  sp->delta = 0.4 / spread;
  printf("  Adaptive delta = %.6f  (spread = %.2f)\n", sp->delta, spread);
  csv = csv_open("B5");

  /* ================================================================== */
  /* Solve: 10 probes at deterministic positions                        */
  /* ================================================================== */
  printf("  Running %d probes, %d realisations each ...\n",
    B5_NPROBES, B5_NREALS);

  for(i = 0; i < B5_NPROBES; i++) {
    struct sdis_solve_probe_args args = SDIS_SOLVE_PROBE_ARGS_DEFAULT;
    struct sdis_estimator* est_wf = NULL;
    struct sdis_mc mc;
    double ref;
    int pass;

    args.nrealisations = B5_NREALS;
    args.position[0] = b5_positions[i][0];
    args.position[1] = b5_positions[i][1];
    args.position[2] = b5_positions[i][2];
    args.time_range[0] = INF;
    args.time_range[1] = INF;

    OK(sdis_solve_wavefront_probe(scn, &args, &est_wf));
    OK(sdis_estimator_get_temperature(est_wf, &mc));

    ref = trilinear_profile(b5_positions[i]);
    pass = p0_compare_analytic(est_wf, ref, B5_TOL_SIGMA);
    n_pass += pass;

    /* CSV: primary DS row + complementary WoS variant */
    csv_row(csv, "B5", "default", "gpu_wf", "DS",
            b5_positions[i][0], b5_positions[i][1], b5_positions[i][2],
            INF, 1, B5_NREALS, mc.E, mc.SE, ref);

    printf("  probe(%+.2f,%+.2f,%+.2f)  wf=%.4f (SE=%.2e)  ref=%.4f  "
      "%s (%.1f sigma)\n",
      b5_positions[i][0], b5_positions[i][1], b5_positions[i][2],
      mc.E, mc.SE, ref,
      pass ? "PASS" : "FAIL",
      mc.SE > 0 ? fabs(mc.E - ref) / mc.SE : 0.0);

    OK(sdis_estimator_ref_put(est_wf));
  }

  csv_close(csv);

  /* ---- Summary ---- */
  printf("\n  Primary:    %d/%d probes pass (%.1f%%)\n",
    n_pass, B5_NPROBES,
    100.0 * (double)n_pass / (double)B5_NPROBES);
  CHK((double)n_pass / (double)B5_NPROBES >= B5_PASS_RATE);

  printf("WF-B5: PASS\n");

  /* ---- Cleanup ---- */
  OK(sdis_scene_ref_put(scn));
  OK(sdis_interface_ref_put(interf));
  OK(sdis_medium_ref_put(fluid));
  OK(sdis_medium_ref_put(solid));
  OK(sdis_device_ref_put(dev));

  CHK(mem_allocated_size() == 0);
  return 0;
}
