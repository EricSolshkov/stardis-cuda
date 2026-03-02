/* WF-G1: Robustness test on non-convex supershape (wavefront probe).
 *
 * Scene: supershape boundary (s3dut, 128x64)
 *   f0: A=1, B=1, M=20, N0=1, N1=1, N2=5
 *   f1: A=1, B=1, M=7,  N0=1, N1=2, N2=5
 *   radius=1 (default)
 *   Material: lambda=10, Cp=1, rho=1, delta=0.4/spread (adaptive)
 *
 * Sub-test 1 (trilinear profile):
 *   T(P) = 333*x' + 432*y' + 579*z'  where q' = (q+10)/20
 *   Probe at (0,0,0) => T_ref = trilinear_temperature({0,0,0})
 *
 * Sub-test 2 (volumetric power):
 *   Pw = 10000, interface T = 0 via volumetric formula
 *   T(P) = beta*(P.x^2 - upper.x^2) + ... , beta = -Pw/(6*lambda)
 *   Probe at (0,0,0) => T_ref = volumetric_temperature({0,0,0}, upper)
 *
 * Both: steady-state (time_range = {INF, INF}), delta_sphere algorithm.
 * Verification: failure_count <= 0.05% of N, eq_eps(T.E, Tref, 3*T.SE)
 *
 * Reference CPU test: test_sdis_solid_random_walk_robustness.c
 */

#include "sdis.h"
#include "test_sdis_utils.h"
#include "test_sdis_wf_p0_utils.h"

#include <star/s3dut.h>
#include <rsys_math.h>

/* ---- Constants ---- */
#define G1_LAMBDA  10.0
#define G1_PW      10000.0
#define G1_NREALS  10000
#define G1_TOL_SIGMA 3.0

/*******************************************************************************
 * Helper functions
 ******************************************************************************/
static double
trilinear_temperature(const double pos[3])
{
  const double a = 333.0;
  const double b = 432.0;
  const double c = 579.0;
  double x = (pos[0] + 10.0) / 20.0;
  double y = (pos[1] + 10.0) / 20.0;
  double z = (pos[2] + 10.0) / 20.0;
  CHK(pos[0] >= -10.0 && pos[0] <= 10.0);
  CHK(pos[1] >= -10.0 && pos[1] <= 10.0);
  CHK(pos[2] >= -10.0 && pos[2] <= 10.0);
  return a*x + b*y + c*z;
}

static double
volumetric_temperature(const double pos[3], const double upper[3])
{
  const double beta = -1.0 / 3.0 * G1_PW / (2.0 * G1_LAMBDA);
  double temp =
      beta * (pos[0]*pos[0] - upper[0]*upper[0])
    + beta * (pos[1]*pos[1] - upper[1]*upper[1])
    + beta * (pos[2]*pos[2] - upper[2]*upper[2]);
  CHK(temp > 0);
  return temp;
}

/*******************************************************************************
 * Geometry (supershape via s3dut)
 ******************************************************************************/
enum profile {
  PROFILE_UNKNOWN,
  PROFILE_TRILINEAR,
  PROFILE_VOLUMETRIC_POWER
};

struct interf_data {
  enum profile profile;
  double upper[3];
  double h;
};

struct solid_data {
  double delta;
  double cp;
  double lambda;
  double rho;
  double temperature;
  double power;
};

struct g1_context {
  struct s3dut_mesh_data msh;
  struct sdis_interface* interf;
};

static void
g1_get_indices(const size_t itri, size_t ids[3], void* context)
{
  const struct g1_context* ctx = context;
  CHK(ids && ctx && itri < ctx->msh.nprimitives);
  /* Flip winding ids[1]<->ids[2] so normals point inward (into the solid) */
  ids[0] = ctx->msh.indices[itri*3+0];
  ids[2] = ctx->msh.indices[itri*3+1];
  ids[1] = ctx->msh.indices[itri*3+2];
}

static void
g1_get_position(const size_t ivert, double pos[3], void* context)
{
  const struct g1_context* ctx = context;
  CHK(pos && ctx && ivert < ctx->msh.nvertices);
  pos[0] = ctx->msh.positions[ivert*3+0];
  pos[1] = ctx->msh.positions[ivert*3+1];
  pos[2] = ctx->msh.positions[ivert*3+2];
}

static void
g1_get_interface(const size_t itri, struct sdis_interface** bound, void* context)
{
  const struct g1_context* ctx = context;
  CHK(bound && ctx && itri < ctx->msh.nprimitives);
  *bound = ctx->interf;
}

/*******************************************************************************
 * Interface shader
 ******************************************************************************/
static double
g1_interface_get_temperature
  (const struct sdis_interface_fragment* frag, struct sdis_data* data)
{
  const struct interf_data* id;
  CHK(data != NULL && frag != NULL);
  id = sdis_data_cget(data);
  switch(id->profile) {
    case PROFILE_UNKNOWN:
      return SDIS_TEMPERATURE_NONE;
    case PROFILE_TRILINEAR:
      return trilinear_temperature(frag->P);
    case PROFILE_VOLUMETRIC_POWER:
      return volumetric_temperature(frag->P, id->upper);
    default: break;
  }
  FATAL("Unreachable code.\n");
  return 0;
}

static double
g1_interface_get_convection_coef
  (const struct sdis_interface_fragment* frag, struct sdis_data* data)
{
  CHK(frag != NULL);
  return ((const struct interf_data*)sdis_data_cget(data))->h;
}

/*******************************************************************************
 * Solid shader (via sdis_data)
 ******************************************************************************/
static double g1_solid_cp(const struct sdis_rwalk_vertex* v, struct sdis_data* d)
{ CHK(v && d); return ((const struct solid_data*)sdis_data_cget(d))->cp; }
static double g1_solid_lambda(const struct sdis_rwalk_vertex* v, struct sdis_data* d)
{ CHK(v && d); return ((const struct solid_data*)sdis_data_cget(d))->lambda; }
static double g1_solid_rho(const struct sdis_rwalk_vertex* v, struct sdis_data* d)
{ CHK(v && d); return ((const struct solid_data*)sdis_data_cget(d))->rho; }
static double g1_solid_delta(const struct sdis_rwalk_vertex* v, struct sdis_data* d)
{ CHK(v && d); return ((const struct solid_data*)sdis_data_cget(d))->delta; }
static double g1_solid_temp(const struct sdis_rwalk_vertex* v, struct sdis_data* d)
{ CHK(v && d); return ((const struct solid_data*)sdis_data_cget(d))->temperature; }
static double g1_solid_power(const struct sdis_rwalk_vertex* v, struct sdis_data* d)
{ CHK(v && d); return ((const struct solid_data*)sdis_data_cget(d))->power; }

/*******************************************************************************
 * Fluid shader
 ******************************************************************************/
static double
g1_fluid_temp(const struct sdis_rwalk_vertex* vtx, struct sdis_data* data)
{
  CHK(vtx != NULL);
  (void)data;
  return 350.0; /* same as CPU test Tfluid */
}

/*******************************************************************************
 * Main
 ******************************************************************************/
int
main(int argc, char** argv)
{
  struct sdis_device* dev = NULL;
  struct sdis_medium* fluid = NULL;
  struct sdis_medium* solid = NULL;
  struct sdis_interface* interf = NULL;
  struct sdis_scene* scn = NULL;
  struct sdis_data* solid_dat = NULL;
  struct sdis_data* interf_dat = NULL;
  struct solid_data* sp = NULL;
  struct interf_data* ip = NULL;
  struct s3dut_super_formula f0 = S3DUT_SUPER_FORMULA_NULL;
  struct s3dut_super_formula f1 = S3DUT_SUPER_FORMULA_NULL;
  struct s3dut_mesh* msh = NULL;
  struct sdis_scene_create_args scn_args = SDIS_SCENE_CREATE_ARGS_DEFAULT;
  struct sdis_solid_shader solid_shader = SDIS_SOLID_SHADER_NULL;
  struct sdis_fluid_shader fluid_shader = DUMMY_FLUID_SHADER;
  struct sdis_interface_shader interf_shader = SDIS_INTERFACE_SHADER_NULL;
  struct g1_context ctx;
  double lower[3], upper[3], spread;
  int all_pass = 1;
  (void)argc; (void)argv;

  OK(sdis_device_create(&SDIS_DEVICE_CREATE_ARGS_DEFAULT, &dev));

  /* ---- Fluid ---- */
  fluid_shader.temperature = g1_fluid_temp;
  OK(sdis_fluid_create(dev, &fluid_shader, NULL, &fluid));

  /* ---- Solid with sdis_data ---- */
  OK(sdis_data_create(dev, sizeof(struct solid_data),
    ALIGNOF(struct solid_data), NULL, &solid_dat));
  sp = sdis_data_get(solid_dat);
  sp->delta = 0.01;
  sp->lambda = G1_LAMBDA;
  sp->cp = 1.0;
  sp->rho = 1.0;
  sp->temperature = SDIS_TEMPERATURE_NONE;
  sp->power = SDIS_VOLUMIC_POWER_NONE;

  solid_shader.calorific_capacity = g1_solid_cp;
  solid_shader.thermal_conductivity = g1_solid_lambda;
  solid_shader.volumic_mass = g1_solid_rho;
  solid_shader.delta = g1_solid_delta;
  solid_shader.temperature = g1_solid_temp;
  solid_shader.volumic_power = g1_solid_power;
  OK(sdis_solid_create(dev, &solid_shader, solid_dat, &solid));
  OK(sdis_data_ref_put(solid_dat));

  /* ---- Interface with sdis_data ---- */
  OK(sdis_data_create(dev, sizeof(struct interf_data),
    ALIGNOF(struct interf_data), NULL, &interf_dat));
  ip = sdis_data_get(interf_dat);
  ip->h = 1.0;
  ip->profile = PROFILE_UNKNOWN;

  interf_shader.convection_coef = g1_interface_get_convection_coef;
  interf_shader.front.temperature = g1_interface_get_temperature;
  OK(sdis_interface_create(dev, solid, fluid, &interf_shader,
    interf_dat, &interf));
  OK(sdis_data_ref_put(interf_dat));

  /* ---- Supershape geometry ---- */
  f0.A = 1; f0.B = 1; f0.M = 20; f0.N0 = 1; f0.N1 = 1; f0.N2 = 5;
  f1.A = 1; f1.B = 1; f1.M = 7;  f1.N0 = 1; f1.N1 = 2; f1.N2 = 5;
  OK(s3dut_create_super_shape(NULL, &f0, &f1, 1, 128, 64, &msh));
  OK(s3dut_mesh_get_data(msh, &ctx.msh));
  ctx.interf = interf;

  scn_args.get_indices = g1_get_indices;
  scn_args.get_position = g1_get_position;
  scn_args.get_interface = g1_get_interface;
  scn_args.nprimitives = ctx.msh.nprimitives;
  scn_args.nvertices = ctx.msh.nvertices;
  scn_args.context = &ctx;
  OK(sdis_scene_create(dev, &scn_args, &scn));

  OK(s3dut_mesh_ref_put(msh));

  /* ---- Retrieve geometry metrics for adaptive delta ---- */
  sp = sdis_data_get(sdis_medium_get_data(solid));
  ip = sdis_data_get(sdis_interface_get_data(interf));
  OK(sdis_scene_get_medium_spread(scn, solid, &spread));
  OK(sdis_scene_get_aabb(scn, lower, upper));
  sp->delta = 0.4 / spread;
  ip->upper[0] = upper[0];
  ip->upper[1] = upper[1];
  ip->upper[2] = upper[2];

  /* ==== Sub-test 1: Trilinear temperature profile ==== */
  {
    struct sdis_solve_probe_args args = SDIS_SOLVE_PROBE_ARGS_DEFAULT;
    struct sdis_estimator* est = NULL;
    struct sdis_mc T;
    size_t nfails;
    double Tref;
    const double pos[3] = {0.0, 0.0, 0.0};
    int ok;

    fprintf(stdout, "  Sub-test 1: trilinear profile (delta-sphere)\n");
    ip->profile = PROFILE_TRILINEAR;
    sp->delta = 0.4 / spread;
    sp->power = SDIS_VOLUMIC_POWER_NONE;

    args.nrealisations = G1_NREALS;
    args.position[0] = pos[0];
    args.position[1] = pos[1];
    args.position[2] = pos[2];
    args.time_range[0] = INF;
    args.time_range[1] = INF;
    args.diff_algo = SDIS_DIFFUSION_DELTA_SPHERE;

    OK(sdis_solve_persistent_wavefront_probe(scn, &args, &est));
    OK(sdis_estimator_get_temperature(est, &T));
    OK(sdis_estimator_get_failure_count(est, &nfails));

    Tref = trilinear_temperature(pos);
    ok = (nfails <= (size_t)(G1_NREALS * 0.0005))
      && eq_eps(T.E, Tref, T.SE * G1_TOL_SIGMA);

    fprintf(stdout,
      "    T = %g ~ %g +/- %g [%g, %g]; #failures = %lu / %d  %s\n",
      Tref, T.E, T.SE, T.E - 3*T.SE, T.E + 3*T.SE,
      (unsigned long)nfails, G1_NREALS, ok ? "PASS" : "FAIL");

    if(!ok) all_pass = 0;
    OK(sdis_estimator_ref_put(est));
  }

  /* ==== Sub-test 2: Volumetric power profile ==== */
  {
    struct sdis_solve_probe_args args = SDIS_SOLVE_PROBE_ARGS_DEFAULT;
    struct sdis_estimator* est = NULL;
    struct sdis_mc T;
    size_t nfails;
    double Tref;
    const double pos[3] = {0.0, 0.0, 0.0};
    int ok;

    fprintf(stdout, "  Sub-test 2: volumetric power (delta-sphere)\n");
    ip->profile = PROFILE_VOLUMETRIC_POWER;
    sp->delta = 0.4 / spread;
    sp->power = G1_PW;

    args.nrealisations = G1_NREALS;
    args.position[0] = pos[0];
    args.position[1] = pos[1];
    args.position[2] = pos[2];
    args.time_range[0] = INF;
    args.time_range[1] = INF;
    args.diff_algo = SDIS_DIFFUSION_DELTA_SPHERE;

    OK(sdis_solve_persistent_wavefront_probe(scn, &args, &est));
    OK(sdis_estimator_get_temperature(est, &T));
    OK(sdis_estimator_get_failure_count(est, &nfails));

    Tref = volumetric_temperature(pos, upper);
    ok = (nfails <= (size_t)(G1_NREALS * 0.0005))
      && eq_eps(T.E, Tref, T.SE * G1_TOL_SIGMA);

    fprintf(stdout,
      "    T = %g ~ %g +/- %g [%g, %g]; #failures = %lu / %d  %s\n",
      Tref, T.E, T.SE, T.E - 3*T.SE, T.E + 3*T.SE,
      (unsigned long)nfails, G1_NREALS, ok ? "PASS" : "FAIL");

    if(!ok) all_pass = 0;
    OK(sdis_estimator_ref_put(est));
  }

  /* ---- Cleanup ---- */
  OK(sdis_scene_ref_put(scn));
  OK(sdis_interface_ref_put(interf));
  OK(sdis_medium_ref_put(fluid));
  OK(sdis_medium_ref_put(solid));
  OK(sdis_device_ref_put(dev));

  CHK(mem_allocated_size() == 0);

  if(all_pass) {
    printf("WF-G1: PASS\n");
  } else {
    printf("WF-G1: FAIL\n");
  }
  CHK(all_pass);
  return 0;
}
