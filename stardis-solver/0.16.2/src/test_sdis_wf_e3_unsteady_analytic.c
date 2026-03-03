/* WF-E3: Unsteady analytic profile in a supershape (wavefront probe).
 *
 * Scene: non-convex supershape solid, lambda=0.1, rho=25, cp=2, delta=1/20
 *   Boundary temperature: T(x,y,z,t) = (B1*(x^3*z - 3*x*y^2*z)
 *     + B2*sin(kx*x)*sin(ky*y)*sin(kz*z)*exp(-alpha*(kx^2+ky^2+kz^2)*t)) / lambda
 *   where kx=ky=kz=pi/4, alpha=lambda/(rho*cp), B1=10, B2=1000
 *   Interior temperature: unknown (SDIS_TEMPERATURE_NONE)
 *   Initial time: t0 = -INF (no initial condition; fully boundary-driven)
 *
 * Supershape:
 *   f0: A=1.5, B=1, M=11, N0=1, N1=1, N2=2
 *   f1: A=1,   B=2, M=3.6, N0=1, N1=2, N2=0.7
 *   radius=1, nslices=256
 *
 * Probe at (0.2, 0.3, 0.4), t=5s.
 * Analytic reference: temperature(pos, time) computed from the formula above.
 *
 * Verification: wavefront result vs analytic temperature.
 *   |T_wf - T_ref| <= 3 * SE
 *   100000 realisations (same as CPU test).
 *
 * Reference CPU test: test_sdis_unsteady_analytic_profile.c
 */

#include "sdis.h"
#include "test_sdis_utils.h"
#include "test_sdis_wf_p0_utils.h"
#include "test_sdis_csv_utils.h"

#include <star/s3dut.h>
#include <rsys/mem_allocator.h>
#include <rsys_math.h>

#include <stdio.h>
#include <math.h>

/* ========================================================================== */
/* Physical constants (identical to CPU test)                                  */
/* ========================================================================== */
#define E3_LAMBDA  0.1       /* [W/(m.K)] */
#define E3_RHO     25.0      /* [kg/m^3] */
#define E3_CP      2.0       /* [J/K/kg] */
#define E3_DELTA   (1.0/20.0)

#define E3_NREALS  100000    /* same as CPU */
#define E3_TOL_SIGMA 3.0
#define E3_PASS_RATE 0.95

/* ========================================================================== */
/* Analytic temperature (identical to CPU test)                                */
/* ========================================================================== */
static double
e3_temperature(const double pos[3], const double time)
{
  const double kx = PI / 4.0;
  const double ky = PI / 4.0;
  const double kz = PI / 4.0;
  const double alpha = E3_LAMBDA / (E3_RHO * E3_CP);

  const double B1 = 10.0;
  const double B2 = 1000.0;

  double x = pos[0], y = pos[1], z = pos[2], t = time;
  double a, b;

  a = B1 * (x*x*x*z - 3.0*x*y*y*z);
  b = B2 * sin(kx*x) * sin(ky*y) * sin(kz*z)
    * exp(-alpha * (kx*kx + ky*ky + kz*kz) * t);

  return (a + b) / E3_LAMBDA;
}

/* ========================================================================== */
/* Supershape geometry (identical to CPU test)                                 */
/* ========================================================================== */
static struct s3dut_mesh*
e3_create_super_shape(void)
{
  struct s3dut_mesh* mesh = NULL;
  struct s3dut_super_formula f0 = S3DUT_SUPER_FORMULA_NULL;
  struct s3dut_super_formula f1 = S3DUT_SUPER_FORMULA_NULL;
  const double radius = 1;
  const unsigned nslices = 256;

  f0.A = 1.5; f0.B = 1; f0.M = 11.0; f0.N0 = 1; f0.N1 = 1; f0.N2 = 2.0;
  f1.A = 1.0; f1.B = 2; f1.M =  3.6; f1.N0 = 1; f1.N1 = 2; f1.N2 = 0.7;
  OK(s3dut_create_super_shape(NULL, &f0, &f1, radius, nslices, nslices/2, &mesh));
  return mesh;
}

/* ========================================================================== */
/* Scene context                                                              */
/* ========================================================================== */
struct e3_scene_ctx {
  struct s3dut_mesh_data mesh_data;
  struct sdis_interface* interf;
};

static void
e3_get_indices(const size_t itri, size_t ids[3], void* ctx)
{
  struct e3_scene_ctx* c = ctx;
  CHK(ids && c && itri < c->mesh_data.nprimitives);
  /* Flip winding so normal points into the supershape (same as CPU test) */
  ids[0] = c->mesh_data.indices[itri*3+0];
  ids[1] = c->mesh_data.indices[itri*3+2];
  ids[2] = c->mesh_data.indices[itri*3+1];
}

static void
e3_get_interface(const size_t itri, struct sdis_interface** interf, void* ctx)
{
  struct e3_scene_ctx* c = ctx;
  CHK(interf && c && itri < c->mesh_data.nprimitives);
  *interf = c->interf;
}

static void
e3_get_position(const size_t ivert, double pos[3], void* ctx)
{
  struct e3_scene_ctx* c = ctx;
  CHK(pos && c && ivert < c->mesh_data.nvertices);
  pos[0] = c->mesh_data.positions[ivert*3+0];
  pos[1] = c->mesh_data.positions[ivert*3+1];
  pos[2] = c->mesh_data.positions[ivert*3+2];
}

/* ========================================================================== */
/* Solid medium shaders                                                       */
/* ========================================================================== */
static double e3_solid_cp(const struct sdis_rwalk_vertex* v, struct sdis_data* d)
  { (void)v; (void)d; return E3_CP; }
static double e3_solid_lambda(const struct sdis_rwalk_vertex* v, struct sdis_data* d)
  { (void)v; (void)d; return E3_LAMBDA; }
static double e3_solid_rho(const struct sdis_rwalk_vertex* v, struct sdis_data* d)
  { (void)v; (void)d; return E3_RHO; }
static double e3_solid_delta(const struct sdis_rwalk_vertex* v, struct sdis_data* d)
  { (void)v; (void)d; return E3_DELTA; }
static double e3_solid_temp(const struct sdis_rwalk_vertex* v, struct sdis_data* d)
  { (void)v; (void)d; return SDIS_TEMPERATURE_NONE; }

/* ========================================================================== */
/* Interface shader: boundary temperature = analytic formula at (P, time)     */
/* ========================================================================== */
static double
e3_interf_get_temperature
  (const struct sdis_interface_fragment* frag, struct sdis_data* data)
{
  (void)data;
  return e3_temperature(frag->P, frag->time);
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
  struct s3dut_mesh* super_shape = NULL;
  struct sdis_scene_create_args scn_args = SDIS_SCENE_CREATE_ARGS_DEFAULT;
  struct sdis_solid_shader solid_shader = SDIS_SOLID_SHADER_NULL;
  struct sdis_fluid_shader fluid_shader = SDIS_FLUID_SHADER_NULL;
  struct sdis_interface_shader interf_shader = SDIS_INTERFACE_SHADER_NULL;
  struct e3_scene_ctx ctx;
  struct sdis_solve_probe_args args = SDIS_SOLVE_PROBE_ARGS_DEFAULT;
  struct sdis_estimator* est_wf = NULL;
  struct sdis_mc mc_wf;
  double ref;
  int pass;

  const double pos[3] = {0.2, 0.3, 0.4};
  const double time = 5.0;

  (void)argc; (void)argv;

  printf("=== WF-E3: Unsteady analytic profile in supershape (wavefront probe) ===\n");

  OK(sdis_device_create(&SDIS_DEVICE_CREATE_ARGS_DEFAULT, &dev));

  /* ---- Supershape geometry ---- */
  super_shape = e3_create_super_shape();
  OK(s3dut_mesh_get_data(super_shape, &ctx.mesh_data));

  /* ---- Solid medium (unknown T, t0=-INF) ---- */
  solid_shader.calorific_capacity = e3_solid_cp;
  solid_shader.thermal_conductivity = e3_solid_lambda;
  solid_shader.volumic_mass = e3_solid_rho;
  solid_shader.delta = e3_solid_delta;
  solid_shader.temperature = e3_solid_temp;
  solid_shader.t0 = -INF;
  OK(sdis_solid_create(dev, &solid_shader, NULL, &solid));

  /* ---- Dummy exterior fluid ---- */
  fluid_shader.calorific_capacity = dummy_medium_getter;
  fluid_shader.volumic_mass = dummy_medium_getter;
  fluid_shader.temperature = dummy_medium_getter;
  OK(sdis_fluid_create(dev, &fluid_shader, NULL, &dummy));

  /* ---- Interface: T = analytic on both sides ---- */
  interf_shader.front.temperature = e3_interf_get_temperature;
  interf_shader.back.temperature = e3_interf_get_temperature;
  OK(sdis_interface_create(dev, solid, dummy, &interf_shader, NULL, &interf));

  /* Release media */
  OK(sdis_medium_ref_put(solid));
  OK(sdis_medium_ref_put(dummy));

  /* ---- Scene ---- */
  ctx.interf = interf;
  scn_args.get_indices = e3_get_indices;
  scn_args.get_interface = e3_get_interface;
  scn_args.get_position = e3_get_position;
  scn_args.nprimitives = ctx.mesh_data.nprimitives;
  scn_args.nvertices = ctx.mesh_data.nvertices;
  scn_args.context = &ctx;
  OK(sdis_scene_create(dev, &scn_args, &scn));

  OK(sdis_interface_ref_put(interf));

  /* ---- Run wavefront probe ---- */
  ref = e3_temperature(pos, time);
  fprintf(stdout, "  Probe at (%.1f, %.1f, %.1f), t=%.1f s\n",
    pos[0], pos[1], pos[2], time);
  fprintf(stdout, "  Analytic reference: T = %.6f K\n", ref);
  fprintf(stdout, "  Running %d realisations ...\n", E3_NREALS);

  args.nrealisations = E3_NREALS;
  args.position[0] = pos[0];
  args.position[1] = pos[1];
  args.position[2] = pos[2];
  args.time_range[0] = time;
  args.time_range[1] = time;
  args.diff_algo = SDIS_DIFFUSION_DELTA_SPHERE;

  OK(sdis_solve_persistent_wavefront_probe(scn, &args, &est_wf));
  OK(sdis_estimator_get_temperature(est_wf, &mc_wf));

  /* Primary: wavefront vs analytic */
  pass = p0_compare_analytic(est_wf, ref, E3_TOL_SIGMA);

  fprintf(stdout,
    "  wf=%.6f (SE=%.2e)  ref=%.6f  %s (%.1f sigma)\n",
    mc_wf.E, mc_wf.SE, ref,
    pass ? "PASS" : "FAIL",
    mc_wf.SE > 0 ? fabs(mc_wf.E - ref) / mc_wf.SE : 0.0);

  CHK(pass);
  printf("WF-E3: PASS\n");

  /* ---- CSV output: time sweep + x sweep ---- */
  {
    FILE* csv = csv_open("E3");
    static const double e3_times[] = {
      0.5, 1.0, 2.0, 3.0, 5.0, 8.0, 12.0, 20.0,
      50.0, 100.0, 200.0, 500.0, 1000.0
    };
    static const double e3_xs[] = {
      -0.45, -0.4, -0.3, -0.2, -0.1, 0.0, 0.1, 0.2, 0.3, 0.4, 0.45
    };
    size_t e3_ntimes = sizeof(e3_times) / sizeof(double);
    size_t e3_nxs    = sizeof(e3_xs) / sizeof(double);
    size_t ti, xi;

    /* Record the already-computed t=5 result */
    csv_row(csv, "E3", "t=5.0", "gpu_wf", "DS",
            pos[0], pos[1], pos[2], time,
            1, E3_NREALS, mc_wf.E, mc_wf.SE, ref);

    /* Time sweep at fixed position (0.2, 0.3, 0.4) */
    fprintf(stdout, "\n  CSV time sweep at (%.1f, %.1f, %.1f) ...\n",
      pos[0], pos[1], pos[2]);
    for(ti = 0; ti < e3_ntimes; ti++) {
      struct sdis_solve_probe_args targs = SDIS_SOLVE_PROBE_ARGS_DEFAULT;
      struct sdis_estimator* est_t = NULL;
      struct sdis_mc mc_t;
      double ref_t;
      char sub[32];

      if(fabs(e3_times[ti] - time) < 0.01) continue;

      targs.nrealisations = E3_NREALS;
      targs.position[0] = pos[0];
      targs.position[1] = pos[1];
      targs.position[2] = pos[2];
      targs.time_range[0] = e3_times[ti];
      targs.time_range[1] = e3_times[ti];
      targs.diff_algo = SDIS_DIFFUSION_DELTA_SPHERE;

      OK(sdis_solve_persistent_wavefront_probe(scn, &targs, &est_t));
      OK(sdis_estimator_get_temperature(est_t, &mc_t));

      ref_t = e3_temperature(pos, e3_times[ti]);

      sprintf(sub, "t=%.1f", e3_times[ti]);
      csv_row(csv, "E3", sub, "gpu_wf", "DS",
              pos[0], pos[1], pos[2], e3_times[ti],
              1, E3_NREALS, mc_t.E, mc_t.SE, ref_t);

      fprintf(stdout, "  t=%5.1f  wf=%.6f (SE=%.2e)  ref=%.6f\n",
              e3_times[ti], mc_t.E, mc_t.SE, ref_t);

      OK(sdis_estimator_ref_put(est_t));
    }

    /* X sweep at t=5, y=0.3, z=0.4 */
    fprintf(stdout, "\n  CSV x sweep at t=%.1f, y=%.1f, z=%.1f ...\n",
      time, pos[1], pos[2]);
    for(xi = 0; xi < e3_nxs; xi++) {
      double xp[3];
      struct sdis_solve_probe_args xargs = SDIS_SOLVE_PROBE_ARGS_DEFAULT;
      struct sdis_estimator* est_x = NULL;
      struct sdis_mc mc_x;
      double ref_x;
      char sub[32];

      xp[0] = e3_xs[xi]; xp[1] = pos[1]; xp[2] = pos[2];

      if(fabs(xp[0] - pos[0]) < 0.01) continue;

      xargs.nrealisations = E3_NREALS;
      xargs.position[0] = xp[0];
      xargs.position[1] = xp[1];
      xargs.position[2] = xp[2];
      xargs.time_range[0] = time;
      xargs.time_range[1] = time;
      xargs.diff_algo = SDIS_DIFFUSION_DELTA_SPHERE;

      OK(sdis_solve_persistent_wavefront_probe(scn, &xargs, &est_x));
      OK(sdis_estimator_get_temperature(est_x, &mc_x));

      ref_x = e3_temperature(xp, time);

      sprintf(sub, "x=%.1f", xp[0]);
      csv_row(csv, "E3", sub, "gpu_wf", "DS",
              xp[0], xp[1], xp[2], time,
              1, E3_NREALS, mc_x.E, mc_x.SE, ref_x);

      fprintf(stdout, "  x=%5.1f  wf=%.6f (SE=%.2e)  ref=%.6f\n",
              xp[0], mc_x.E, mc_x.SE, ref_x);

      OK(sdis_estimator_ref_put(est_x));
    }

    csv_close(csv);
  }

  /* ---- Cleanup ---- */
  OK(sdis_estimator_ref_put(est_wf));
  OK(s3dut_mesh_ref_put(super_shape));
  OK(sdis_scene_ref_put(scn));
  OK(sdis_device_ref_put(dev));

  CHK(mem_allocated_size() == 0);
  return 0;
}
