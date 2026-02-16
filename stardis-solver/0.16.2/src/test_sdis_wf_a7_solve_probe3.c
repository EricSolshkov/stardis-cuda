/* WF-A7: Embedded sphere same-material test (wavefront probe).
 *
 * Scene: unit cube (0,0,0)-(1,1,1) with a centered sphere (r=0.25, 64x32).
 *   The sphere surface is a solid-solid interface (transparent, same material).
 *   Front face (-Z, tri 0-1): T = 300 K
 *   Back face  (+Z, tri 4-5): T = 350 K
 *   Other box faces:           adiabatic (no temperature)
 *
 *   Material: Cp=2, lambda=50, rho=25, delta=1/20
 *
 * Probe at (0.5, 0.5, 0.5) => T_ref = 325 K (linear interpolation).
 * Steady-state: time_range = {INF, INF}
 *
 * Verification: wavefront result vs analytic linear profile.
 *   |T_wf - T_ref| <= 3 * SE
 *   failure_count < N/1000
 *
 * Reference CPU test: test_sdis_solve_probe3.c
 */

#include "sdis.h"
#include "test_sdis_utils.h"
#include "test_sdis_wf_p0_utils.h"

#include <rsys/stretchy_array.h>
#include <rsys_math.h>
#include <star/s3dut.h>

/* ---- Constants ---- */
#define A7_NREALS      10000
#define A7_TOL_SIGMA   3.0
#define A7_T_FRONT     300.0
#define A7_T_BACK      350.0

/*******************************************************************************
 * Geometry (box + centered sphere via stretchy_array)
 ******************************************************************************/
struct a7_context {
  double* positions;
  size_t* indices;
  struct sdis_interface* solid_fluid_Tnone;
  struct sdis_interface* solid_fluid_T300;
  struct sdis_interface* solid_fluid_T350;
  struct sdis_interface* solid_solid;
};
static const struct a7_context A7_CONTEXT_NULL = {NULL, NULL, NULL, NULL, NULL, NULL};

static void
a7_get_indices(const size_t itri, size_t ids[3], void* context)
{
  struct a7_context* ctx = context;
  ids[0] = ctx->indices[itri*3+0];
  ids[1] = ctx->indices[itri*3+1];
  ids[2] = ctx->indices[itri*3+2];
}

static void
a7_get_position(const size_t ivert, double pos[3], void* context)
{
  struct a7_context* ctx = context;
  pos[0] = ctx->positions[ivert*3+0];
  pos[1] = ctx->positions[ivert*3+1];
  pos[2] = ctx->positions[ivert*3+2];
}

static void
a7_get_interface(const size_t itri, struct sdis_interface** bound, void* context)
{
  struct a7_context* ctx = context;
  CHK(bound != NULL && context != NULL);

  if(itri == 0 || itri == 1) {           /* Box front face (-Z) */
    *bound = ctx->solid_fluid_T300;
  } else if(itri == 4 || itri == 5) {    /* Box back face (+Z) */
    *bound = ctx->solid_fluid_T350;
  } else if(itri < box_ntriangles) {     /* Box remaining faces */
    *bound = ctx->solid_fluid_Tnone;
  } else {                               /* Sphere (solid-solid) */
    *bound = ctx->solid_solid;
  }
}

/*******************************************************************************
 * Medium shaders (inline, no sdis_data needed since properties are constant)
 ******************************************************************************/
static double
a7_temp_unknown(const struct sdis_rwalk_vertex* vtx, struct sdis_data* data)
{
  (void)data; CHK(vtx != NULL);
  return SDIS_TEMPERATURE_NONE;
}
static double
a7_solid_cp(const struct sdis_rwalk_vertex* vtx, struct sdis_data* data)
{
  (void)data; CHK(vtx != NULL);
  return 2.0;
}
static double
a7_solid_lambda(const struct sdis_rwalk_vertex* vtx, struct sdis_data* data)
{
  (void)data; CHK(vtx != NULL);
  return 50.0;
}
static double
a7_solid_rho(const struct sdis_rwalk_vertex* vtx, struct sdis_data* data)
{
  (void)data; CHK(vtx != NULL);
  return 25.0;
}
static double
a7_solid_delta(const struct sdis_rwalk_vertex* vtx, struct sdis_data* data)
{
  (void)data; CHK(vtx != NULL);
  return 1.0 / 20.0;
}

/*******************************************************************************
 * Interface shaders
 ******************************************************************************/
struct a7_interf {
  double temperature;
};

static double
a7_null_interface_value
  (const struct sdis_interface_fragment* frag, struct sdis_data* data)
{
  CHK(frag != NULL);
  (void)data;
  return 0;
}

static double
a7_interface_get_temperature
  (const struct sdis_interface_fragment* frag, struct sdis_data* data)
{
  CHK(data != NULL && frag != NULL);
  return ((const struct a7_interf*)sdis_data_cget(data))->temperature;
}

/*******************************************************************************
 * Main
 ******************************************************************************/
int
main(int argc, char** argv)
{
  struct sdis_device* dev = NULL;
  struct sdis_data* data = NULL;
  struct sdis_medium* solid = NULL;
  struct sdis_medium* fluid = NULL;
  struct sdis_interface* Tnone = NULL;
  struct sdis_interface* T300 = NULL;
  struct sdis_interface* T350 = NULL;
  struct sdis_interface* solid_solid = NULL;
  struct sdis_scene* scn = NULL;
  struct sdis_estimator* est_wf = NULL;
  struct sdis_scene_create_args scn_args = SDIS_SCENE_CREATE_ARGS_DEFAULT;
  struct sdis_fluid_shader fluid_shader = DUMMY_FLUID_SHADER;
  struct sdis_solid_shader solid_shader = DUMMY_SOLID_SHADER;
  struct sdis_interface_shader interf_shader = DUMMY_INTERFACE_SHADER;
  struct sdis_solve_probe_args solve_args = SDIS_SOLVE_PROBE_ARGS_DEFAULT;
  struct sdis_mc T_mc;
  struct s3dut_mesh* msh = NULL;
  struct s3dut_mesh_data msh_data;
  struct a7_context ctx = A7_CONTEXT_NULL;
  struct a7_interf* interf_param = NULL;
  double ref;
  size_t nreals, nfails;
  size_t i;
  int pass;
  (void)argc; (void)argv;

  OK(sdis_device_create(&SDIS_DEVICE_CREATE_ARGS_DEFAULT, &dev));

  /* ---- Fluid ---- */
  fluid_shader.temperature = a7_temp_unknown;
  OK(sdis_fluid_create(dev, &fluid_shader, NULL, &fluid));

  /* ---- Solid ---- */
  solid_shader.calorific_capacity = a7_solid_cp;
  solid_shader.thermal_conductivity = a7_solid_lambda;
  solid_shader.volumic_mass = a7_solid_rho;
  solid_shader.delta = a7_solid_delta;
  solid_shader.temperature = a7_temp_unknown;
  OK(sdis_solid_create(dev, &solid_shader, NULL, &solid));

  /* ---- Interface: adiabatic (no temperature, no convection) ---- */
  interf_shader.convection_coef = a7_null_interface_value;
  interf_shader.thermal_contact_resistance = NULL; /* solid/fluid: no TCR */
  interf_shader.front = SDIS_INTERFACE_SIDE_SHADER_NULL;
  interf_shader.back = SDIS_INTERFACE_SIDE_SHADER_NULL;
  OK(sdis_interface_create(dev, solid, fluid, &interf_shader, NULL, &Tnone));

  /* ---- Interface: T=300 (front face) ---- */
  OK(sdis_data_create(dev, sizeof(struct a7_interf),
    ALIGNOF(struct a7_interf), NULL, &data));
  interf_param = sdis_data_get(data);
  interf_param->temperature = A7_T_FRONT;
  interf_shader.front.temperature = a7_interface_get_temperature;
  OK(sdis_interface_create(dev, solid, fluid, &interf_shader, data, &T300));
  OK(sdis_data_ref_put(data));

  /* ---- Interface: T=350 (back face) ---- */
  OK(sdis_data_create(dev, sizeof(struct a7_interf),
    ALIGNOF(struct a7_interf), NULL, &data));
  interf_param = sdis_data_get(data);
  interf_param->temperature = A7_T_BACK;
  OK(sdis_interface_create(dev, solid, fluid, &interf_shader, data, &T350));
  OK(sdis_data_ref_put(data));

  /* ---- Interface: solid-solid (transparent, same material) ---- */
  interf_shader = SDIS_INTERFACE_SHADER_NULL;
  OK(sdis_interface_create(dev, solid, solid, &interf_shader, NULL,
    &solid_solid));

  /* Release media (scene holds refs) */
  OK(sdis_medium_ref_put(solid));
  OK(sdis_medium_ref_put(fluid));

  /* ---- Register box geometry ---- */
  FOR_EACH(i, 0, box_nvertices) {
    sa_push(ctx.positions, box_vertices[i*3+0]);
    sa_push(ctx.positions, box_vertices[i*3+1]);
    sa_push(ctx.positions, box_vertices[i*3+2]);
  }
  FOR_EACH(i, 0, box_ntriangles) {
    sa_push(ctx.indices, box_indices[i*3+0]);
    sa_push(ctx.indices, box_indices[i*3+1]);
    sa_push(ctx.indices, box_indices[i*3+2]);
  }

  /* ---- Sphere at center (0.5, 0.5, 0.5), radius=0.25 ---- */
  OK(s3dut_create_sphere(NULL, 0.25, 64, 32, &msh));
  OK(s3dut_mesh_get_data(msh, &msh_data));
  FOR_EACH(i, 0, msh_data.nvertices) {
    sa_push(ctx.positions, msh_data.positions[i*3+0] + 0.5);
    sa_push(ctx.positions, msh_data.positions[i*3+1] + 0.5);
    sa_push(ctx.positions, msh_data.positions[i*3+2] + 0.5);
  }
  FOR_EACH(i, 0, msh_data.nprimitives) {
    sa_push(ctx.indices, msh_data.indices[i*3+0] + box_nvertices);
    sa_push(ctx.indices, msh_data.indices[i*3+1] + box_nvertices);
    sa_push(ctx.indices, msh_data.indices[i*3+2] + box_nvertices);
  }
  OK(s3dut_mesh_ref_put(msh));

  /* ---- Create scene ---- */
  ctx.solid_fluid_Tnone = Tnone;
  ctx.solid_fluid_T300 = T300;
  ctx.solid_fluid_T350 = T350;
  ctx.solid_solid = solid_solid;
  scn_args.get_indices = a7_get_indices;
  scn_args.get_interface = a7_get_interface;
  scn_args.get_position = a7_get_position;
  scn_args.nprimitives = sa_size(ctx.indices) / 3;
  scn_args.nvertices = sa_size(ctx.positions) / 3;
  scn_args.context = &ctx;
  OK(sdis_scene_create(dev, &scn_args, &scn));

  /* Release scene data */
  OK(sdis_interface_ref_put(Tnone));
  OK(sdis_interface_ref_put(T300));
  OK(sdis_interface_ref_put(T350));
  OK(sdis_interface_ref_put(solid_solid));
  sa_release(ctx.positions);
  sa_release(ctx.indices);

  /* ---- Solve: probe at center (0.5, 0.5, 0.5) ---- */
  solve_args.nrealisations = A7_NREALS;
  solve_args.position[0] = 0.5;
  solve_args.position[1] = 0.5;
  solve_args.position[2] = 0.5;
  solve_args.time_range[0] = INF;
  solve_args.time_range[1] = INF;

  OK(sdis_solve_wavefront_probe(scn, &solve_args, &est_wf));
  OK(sdis_estimator_get_realisation_count(est_wf, &nreals));
  OK(sdis_estimator_get_failure_count(est_wf, &nfails));
  OK(sdis_estimator_get_temperature(est_wf, &T_mc));

  /* ---- Verify ---- */
  ref = A7_T_BACK * solve_args.position[2]
      + (1.0 - solve_args.position[2]) * A7_T_FRONT;

  fprintf(stdout,
    "  Temperature at (%g, %g, %g) = %g ~ %g +/- %g\n",
    SPLIT3(solve_args.position), ref, T_mc.E, T_mc.SE);
  fprintf(stdout,
    "  #failures = %lu / %lu\n",
    (unsigned long)nfails, (unsigned long)A7_NREALS);

  pass = (nfails + nreals == (size_t)A7_NREALS)
      && (nfails < (size_t)(A7_NREALS / 1000))
      && p0_compare_analytic(est_wf, ref, A7_TOL_SIGMA);

  fprintf(stdout, "  WF-A7: %s\n", pass ? "PASS" : "FAIL");

  /* ---- Cleanup ---- */
  OK(sdis_estimator_ref_put(est_wf));
  OK(sdis_scene_ref_put(scn));
  OK(sdis_device_ref_put(dev));

  CHK(mem_allocated_size() == 0);
  CHK(pass);
  return 0;
}
