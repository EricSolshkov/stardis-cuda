/* WF-F1: External flux (solar source), Lambertian and Specular ground
 *         (wavefront probe).
 *
 * Scene: thin wall + ground, illuminated by a distant spherical source (Sun).
 *   Wall: solid, lambda=1.15, rho=1700, cp=800, delta=0.005
 *     x=[-0.1, 0], y=[500, 1500], z=[-500, 500]
 *     eps=1 for external source, eps=0 for internal radiation
 *     H_conv = 10 W/(m^2*K)
 *   Ground: solid, lambda=1.15
 *     x=[0, 1e12], y=[-1, 0 or +1], z=[-1e6, 1e6]  (3D)
 *     eps=0, specular={0 or 1}
 *
 * Source: Sun at (cos30*1.5e11, sin30*1.5e11, 0)
 *   P = 3.845e26 W, R = 6.5991756e8 m
 *
 * Radenv: T=0K, Tref=300K
 * Fluid temperature: 300K
 *
 * Probe at wall center: (-0.05, 1000, 0)
 *
 * Analytic reference:
 *   Lambertian ground (specular=0): T = 375.88 K
 *   Specular ground (specular=1):   T = 417.77 K
 *
 * Reference CPU test: test_sdis_external_flux.c
 */

#include "sdis.h"
#include "test_sdis_utils.h"
#include "test_sdis_wf_p0_utils.h"
#include "test_sdis_csv_utils.h"

#include <rsys/mem_allocator.h>
#include <stdio.h>
#include <math.h>

/* ========================================================================== */
/* Physical constants                                                         */
/* ========================================================================== */
#define F1_T_FLUID        300.0
#define F1_T_REF          300.0

#define F1_LAMBDA          1.15
#define F1_RHO          1700.0
#define F1_CP            800.0
#define F1_DELTA           0.005   /* 0.1/20 */

#define F1_SRC_POWER       3.845e26
#define F1_SRC_RADIUS      6.5991756e8
#define F1_SRC_DISTANCE    1.5e11
#define F1_SRC_ELEVATION   30.0     /* degrees */

#define F1_WALL_HC         10.0

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#define DEG2RAD(d) ((d) * M_PI / 180.0)

#define F1_NREALS_LAMBERT  10000
#define F1_NREALS_SPECULAR 100000

/* ========================================================================== */
/* Geometry: 16 vertices, 24 triangles (ground 12 + wall 12)                  */
/* ========================================================================== */
static const double f1_vertices[16 * 3] = {
  /* Ground (8 verts) */
     0.0, -1.0, -1e6,   /*  0 */
   1e12, -1.0, -1e6,    /*  1 */
     0.0,  1.0, -1e6,   /*  2 */
   1e12,  1.0, -1e6,    /*  3 */
     0.0, -1.0,  1e6,   /*  4 */
   1e12, -1.0,  1e6,    /*  5 */
     0.0,  1.0,  1e6,   /*  6 */
   1e12,  1.0,  1e6,    /*  7 */
  /* Wall (8 verts) */
  -0.1, 500.0, -500.0,  /*  8 */
   0.0, 500.0, -500.0,  /*  9 */
  -0.1, 1500.0, -500.0, /* 10 */
   0.0, 1500.0, -500.0, /* 11 */
  -0.1, 500.0,  500.0,  /* 12 */
   0.0, 500.0,  500.0,  /* 13 */
  -0.1, 1500.0,  500.0, /* 14 */
   0.0, 1500.0,  500.0  /* 15 */
};
static const size_t f1_nvertices = 16;

static const size_t f1_indices[24 * 3] = {
  /* Ground (tri 0-11): box faces */
  0, 2, 1,  1, 2, 3,     /*  0, 1: -Z */
  0, 4, 2,  2, 4, 6,     /*  2, 3: -X */
  4, 5, 6,  6, 5, 7,     /*  4, 5: +Z */
  3, 7, 1,  1, 7, 5,     /*  6, 7: +X */
  2, 6, 3,  3, 6, 7,     /*  8, 9: +Y (top = exposed surface) */
  0, 1, 4,  4, 1, 5,     /* 10,11: -Y */
  /* Wall (tri 12-23): box faces */
   8, 10,  9,   9, 10, 11,  /* 12,13: -Z */
   8, 12, 10,  10, 12, 14,  /* 14,15: -X (back of wall) */
  12, 13, 14,  14, 13, 15,  /* 16,17: +Z */
  11, 15,  9,   9, 15, 13,  /* 18,19: +X (front, facing sun/ground) */
  10, 14, 11,  11, 14, 15,  /* 20,21: +Y (top) */
   8,  9, 12,  12,  9, 13   /* 22,23: -Y (bottom) */
};
static const size_t f1_ntriangles = 24;

struct f1_geometry {
  struct sdis_interface** interfaces;
};

static void
f1_get_indices(const size_t itri, size_t ids[3], void* ctx)
{
  (void)ctx;
  CHK(itri < f1_ntriangles);
  ids[0] = f1_indices[itri * 3 + 0];
  ids[1] = f1_indices[itri * 3 + 1];
  ids[2] = f1_indices[itri * 3 + 2];
}

static void
f1_get_position(const size_t ivert, double pos[3], void* ctx)
{
  (void)ctx;
  CHK(ivert < f1_nvertices);
  pos[0] = f1_vertices[ivert * 3 + 0];
  pos[1] = f1_vertices[ivert * 3 + 1];
  pos[2] = f1_vertices[ivert * 3 + 2];
}

static void
f1_get_interface(const size_t itri, struct sdis_interface** bound, void* ctx)
{
  struct f1_geometry* g = ctx;
  CHK(itri < f1_ntriangles && bound);
  *bound = g->interfaces[itri];
}

/* ========================================================================== */
/* Solid shader (wall material)                                               */
/* ========================================================================== */
static double
f1_solid_get_calorific_capacity
  (const struct sdis_rwalk_vertex* vtx, struct sdis_data* data)
{
  (void)data;
  CHK(vtx != NULL);
  return F1_CP;
}

static double
f1_solid_get_thermal_conductivity
  (const struct sdis_rwalk_vertex* vtx, struct sdis_data* data)
{
  (void)data;
  CHK(vtx != NULL);
  return F1_LAMBDA;
}

static double
f1_solid_get_volumic_mass
  (const struct sdis_rwalk_vertex* vtx, struct sdis_data* data)
{
  (void)data;
  CHK(vtx != NULL);
  return F1_RHO;
}

static double
f1_solid_get_delta
  (const struct sdis_rwalk_vertex* vtx, struct sdis_data* data)
{
  (void)data;
  CHK(vtx != NULL);
  return F1_DELTA;
}

static double
f1_solid_get_temperature
  (const struct sdis_rwalk_vertex* vtx, struct sdis_data* data)
{
  (void)data;
  CHK(vtx != NULL);
  return SDIS_TEMPERATURE_NONE;
}

/* ========================================================================== */
/* Fluid shader (constant temperature = T_FLUID)                              */
/* ========================================================================== */
static double
f1_fluid_get_temperature
  (const struct sdis_rwalk_vertex* vtx, struct sdis_data* data)
{
  (void)data;
  CHK(vtx != NULL);
  return F1_T_FLUID;
}

static double
f1_fluid_get_volumic_mass
  (const struct sdis_rwalk_vertex* vtx, struct sdis_data* data)
{
  (void)data;
  CHK(vtx != NULL);
  return F1_RHO;
}

static double
f1_fluid_get_calorific_capacity
  (const struct sdis_rwalk_vertex* vtx, struct sdis_data* data)
{
  (void)data;
  CHK(vtx != NULL);
  return F1_CP;
}

/* ========================================================================== */
/* Interface shaders                                                          */
/* ========================================================================== */
struct f1_interf {
  double emissivity;
  double specular_fraction;
  double hc;
};

/* Wall emissivity: eps=1 for external sources, eps=0 for internal */
static double
f1_wall_get_emissivity
  (const struct sdis_interface_fragment* frag,
   const unsigned source_id,
   struct sdis_data* data)
{
  (void)frag;
  CHK(data != NULL);
  return source_id == SDIS_INTERN_SOURCE_ID
    ? 0.0
    : ((const struct f1_interf*)sdis_data_cget(data))->emissivity;
}

/* Ground emissivity: always 0 */
static double
f1_ground_get_emissivity
  (const struct sdis_interface_fragment* frag,
   const unsigned source_id,
   struct sdis_data* data)
{
  (void)frag; (void)source_id; (void)data;
  return 0.0;
}

static double
f1_interf_get_specular_fraction
  (const struct sdis_interface_fragment* frag,
   const unsigned source_id,
   struct sdis_data* data)
{
  (void)frag;
  CHK(data != NULL);
  return source_id == SDIS_INTERN_SOURCE_ID
    ? 0.0
    : ((const struct f1_interf*)sdis_data_cget(data))->specular_fraction;
}

static double
f1_interf_get_convection_coef
  (const struct sdis_interface_fragment* frag, struct sdis_data* data)
{
  CHK(frag && data);
  return ((const struct f1_interf*)sdis_data_cget(data))->hc;
}

static double
f1_interf_get_temperature
  (const struct sdis_interface_fragment* frag, struct sdis_data* data)
{
  (void)frag; (void)data;
  return SDIS_TEMPERATURE_NONE;
}

static double
f1_interf_get_ref_temperature
  (const struct sdis_interface_fragment* frag, struct sdis_data* data)
{
  (void)frag; (void)data;
  return F1_T_REF;
}

/* ========================================================================== */
/* Radiative environment (T = 0K ambient, Tref = 300K)                        */
/* ========================================================================== */
static double
f1_radenv_get_temperature
  (const struct sdis_radiative_ray* ray, struct sdis_data* data)
{
  (void)ray; (void)data;
  return 0.0;
}

static double
f1_radenv_get_reference
  (const struct sdis_radiative_ray* ray, struct sdis_data* data)
{
  (void)ray; (void)data;
  return F1_T_REF;
}

/* ========================================================================== */
/* Spherical source (Sun)                                                     */
/* ========================================================================== */
static void
f1_source_get_position
  (const double time, double pos[3], struct sdis_data* data)
{
  (void)time; (void)data;
  pos[0] = cos(DEG2RAD(F1_SRC_ELEVATION)) * F1_SRC_DISTANCE;
  pos[1] = sin(DEG2RAD(F1_SRC_ELEVATION)) * F1_SRC_DISTANCE;
  pos[2] = 0.0;
}

static double
f1_source_get_power
  (const double time, struct sdis_data* data)
{
  (void)time; (void)data;
  return F1_SRC_POWER;
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
  struct sdis_medium* wall_solid = NULL;
  struct sdis_medium* ground_solid = NULL;
  struct sdis_interface* interf_wall = NULL;
  struct sdis_interface* interf_ground = NULL;
  struct sdis_radiative_env* radenv = NULL;
  struct sdis_source* source = NULL;
  struct sdis_scene* scn = NULL;
  struct sdis_scene_create_args scn_args = SDIS_SCENE_CREATE_ARGS_DEFAULT;
  struct sdis_solid_shader solid_shader = DUMMY_SOLID_SHADER;
  struct sdis_fluid_shader fluid_shader = DUMMY_FLUID_SHADER;
  struct sdis_radiative_env_shader radenv_shader = SDIS_RADIATIVE_ENV_SHADER_NULL;
  struct sdis_spherical_source_shader source_shader
    = SDIS_SPHERICAL_SOURCE_SHADER_NULL;
  struct sdis_interface* prim_interfaces[24];
  struct f1_geometry geom;
  struct f1_interf* ground_interf_data = NULL;
  int n_pass = 0, n_total = 0;
  size_t i;
  FILE* csv = NULL;
  (void)argc; (void)argv;

  printf("=== WF-F1: External flux / solar source (wavefront probe) ===\n");
  csv = csv_open("F1");

  OK(sdis_device_create(&SDIS_DEVICE_CREATE_ARGS_DEFAULT, &dev));

  /* ---- Fluid (constant temperature = T_FLUID) ---- */
  fluid_shader.temperature = f1_fluid_get_temperature;
  fluid_shader.calorific_capacity = f1_fluid_get_calorific_capacity;
  fluid_shader.volumic_mass = f1_fluid_get_volumic_mass;
  OK(sdis_fluid_create(dev, &fluid_shader, NULL, &fluid));

  /* ---- Wall solid ---- */
  solid_shader.calorific_capacity = f1_solid_get_calorific_capacity;
  solid_shader.thermal_conductivity = f1_solid_get_thermal_conductivity;
  solid_shader.volumic_mass = f1_solid_get_volumic_mass;
  solid_shader.delta = f1_solid_get_delta;
  solid_shader.temperature = f1_solid_get_temperature;
  OK(sdis_solid_create(dev, &solid_shader, NULL, &wall_solid));

  /* ---- Ground solid (reuse same shader) ---- */
  OK(sdis_solid_create(dev, &solid_shader, NULL, &ground_solid));

  /* ---- Wall interface (solid/fluid): eps_ext=1, eps_int=0, hc=10 ---- */
  {
    struct sdis_interface_shader sh = SDIS_INTERFACE_SHADER_NULL;
    sh.front.temperature = f1_interf_get_temperature;
    sh.back.temperature = f1_interf_get_temperature;
    sh.back.emissivity = f1_wall_get_emissivity;
    sh.back.specular_fraction = f1_interf_get_specular_fraction;
    sh.back.reference_temperature = f1_interf_get_ref_temperature;
    sh.convection_coef = f1_interf_get_convection_coef;
    sh.convection_coef_upper_bound = F1_WALL_HC;

    OK(sdis_data_create(dev, sizeof(struct f1_interf),
      ALIGNOF(struct f1_interf), NULL, &data));
    {
      struct f1_interf* wp = sdis_data_get(data);
      wp->emissivity = 1.0;
      wp->specular_fraction = 0.0;
      wp->hc = F1_WALL_HC;
    }
    OK(sdis_interface_create(dev, wall_solid, fluid, &sh, data,
      &interf_wall));
    OK(sdis_data_ref_put(data));
  }

  /* ---- Ground interface (solid/fluid): eps=0, specular=mutable ---- */
  {
    struct sdis_interface_shader sh = SDIS_INTERFACE_SHADER_NULL;
    sh.front.temperature = f1_interf_get_temperature;
    sh.back.temperature = f1_interf_get_temperature;
    sh.back.emissivity = f1_ground_get_emissivity;
    sh.back.specular_fraction = f1_interf_get_specular_fraction;
    sh.back.reference_temperature = f1_interf_get_ref_temperature;
    sh.convection_coef = f1_interf_get_convection_coef;
    sh.convection_coef_upper_bound = 0.0;

    OK(sdis_data_create(dev, sizeof(struct f1_interf),
      ALIGNOF(struct f1_interf), NULL, &data));
    {
      struct f1_interf* gp = sdis_data_get(data);
      gp->emissivity = 0.0;
      gp->specular_fraction = 0.0;  /* initial: Lambertian */
      gp->hc = 0.0;
    }
    OK(sdis_interface_create(dev, ground_solid, fluid, &sh, data,
      &interf_ground));
    /* Keep data ref to get mutable pointer */
    ground_interf_data = sdis_data_get(data);
    OK(sdis_data_ref_put(data));
  }

  /* Get mutable pointer for ground specular fraction */
  ground_interf_data = sdis_data_get(
    sdis_interface_get_data(interf_ground));

  /* Release media */
  OK(sdis_medium_ref_put(fluid));
  OK(sdis_medium_ref_put(wall_solid));
  OK(sdis_medium_ref_put(ground_solid));

  /* ---- Radiative environment (T=0K, Tref=300K) ---- */
  radenv_shader.temperature = f1_radenv_get_temperature;
  radenv_shader.reference_temperature = f1_radenv_get_reference;
  OK(sdis_radiative_env_create(dev, &radenv_shader, NULL, &radenv));

  /* ---- Spherical source (Sun) ---- */
  source_shader.position = f1_source_get_position;
  source_shader.power = f1_source_get_power;
  source_shader.radius = F1_SRC_RADIUS;
  OK(sdis_spherical_source_create(dev, &source_shader, NULL, &source));

  /* ---- Triangle-to-interface mapping ---- */
  /* Ground: tri 0-11 */
  for(i = 0; i < 12; i++)
    prim_interfaces[i] = interf_ground;
  /* Wall: tri 12-23 */
  for(i = 12; i < 24; i++)
    prim_interfaces[i] = interf_wall;

  /* ---- Scene creation ---- */
  geom.interfaces = prim_interfaces;

  scn_args.get_indices = f1_get_indices;
  scn_args.get_interface = f1_get_interface;
  scn_args.get_position = f1_get_position;
  scn_args.nprimitives = f1_ntriangles;
  scn_args.nvertices = f1_nvertices;
  scn_args.t_range[0] = 0; /* [K] */
  scn_args.t_range[1] = 0; /* [K] */
  scn_args.source = source;
  scn_args.radenv = radenv;
  scn_args.context = &geom;
  OK(sdis_scene_create(dev, &scn_args, &scn));

  /* Release refs */
  OK(sdis_interface_ref_put(interf_wall));
  OK(sdis_interface_ref_put(interf_ground));
  OK(sdis_radiative_env_ref_put(radenv));
  OK(sdis_source_ref_put(source));

  /* ---- Sub-test 1: Lambertian ground (specular=0) ---- */
  {
    struct sdis_solve_probe_args args = SDIS_SOLVE_PROBE_ARGS_DEFAULT;
    struct sdis_estimator *est_wf = NULL, *est_df = NULL;
    double T_ref = 375.88;
    int pass;

    ground_interf_data->specular_fraction = 0.0;

    printf("\n  [1] Lambertian ground (specular=0), ref=%.2f K\n", T_ref);

    args.nrealisations = F1_NREALS_LAMBERT;
    args.position[0] = -0.05;
    args.position[1] = 1000.0;
    args.position[2] = 0.0;
    args.picard_order = 1;
    args.diff_algo = SDIS_DIFFUSION_DELTA_SPHERE;

    OK(sdis_solve_persistent_wavefront_probe(scn, &args, &est_wf));

    pass = p0_compare_analytic(est_wf, T_ref, P0_TOL_SIGMA);
    n_pass += pass;
    n_total++;

    /* CSV: primary DS row + complementary WoS variant */
    {
      struct sdis_mc mc_csv;
      OK(sdis_estimator_get_temperature(est_wf, &mc_csv));
      csv_row(csv, "F1", "lambertian", "gpu_wf", "DS",
              args.position[0], args.position[1], args.position[2],
              INF, 1, F1_NREALS_LAMBERT, mc_csv.E, mc_csv.SE, T_ref);

    }

    if(P0_ENABLE_DIAG) {
      OK(sdis_solve_probe(scn, &args, &est_df));
    }

    p0_print_probe_result(-0.05, est_wf, est_df, T_ref);

    OK(sdis_estimator_ref_put(est_wf));
    if(est_df) OK(sdis_estimator_ref_put(est_df));
  }

  /* ---- Sub-test 2: Specular ground (specular=1) ---- */
  {
    struct sdis_solve_probe_args args = SDIS_SOLVE_PROBE_ARGS_DEFAULT;
    struct sdis_estimator *est_wf = NULL, *est_df = NULL;
    double T_ref = 417.77;
    int pass;

    ground_interf_data->specular_fraction = 1.0;

    printf("\n  [2] Specular ground (specular=1), ref=%.2f K\n", T_ref);

    args.nrealisations = F1_NREALS_SPECULAR;
    args.position[0] = -0.05;
    args.position[1] = 1000.0;
    args.position[2] = 0.0;
    args.picard_order = 1;
    args.diff_algo = SDIS_DIFFUSION_DELTA_SPHERE;

    OK(sdis_solve_persistent_wavefront_probe(scn, &args, &est_wf));

    pass = p0_compare_analytic(est_wf, T_ref, P0_TOL_SIGMA);
    n_pass += pass;
    n_total++;

    /* CSV: primary DS row + complementary WoS variant */
    {
      struct sdis_mc mc_csv;
      OK(sdis_estimator_get_temperature(est_wf, &mc_csv));
      csv_row(csv, "F1", "specular", "gpu_wf", "DS",
              args.position[0], args.position[1], args.position[2],
              INF, 1, F1_NREALS_SPECULAR, mc_csv.E, mc_csv.SE, T_ref);

    }

    if(P0_ENABLE_DIAG) {
      OK(sdis_solve_probe(scn, &args, &est_df));
    }

    p0_print_probe_result(-0.05, est_wf, est_df, T_ref);

    OK(sdis_estimator_ref_put(est_wf));
    if(est_df) OK(sdis_estimator_ref_put(est_df));
  }

  csv_close(csv);
  fprintf(stdout, "\n  Primary: %d/%d sub-tests pass (%.1f%%)\n",
    n_pass, n_total,
    100.0 * (double)n_pass / (double)n_total);

  CHK((double)n_pass / (double)n_total >= P0_PASS_RATE);

  printf("WF-F1: PASS\n");

  /* ---- Cleanup ---- */
  OK(sdis_scene_ref_put(scn));
  OK(sdis_device_ref_put(dev));

  CHK(mem_allocated_size() == 0);
  return 0;
}
