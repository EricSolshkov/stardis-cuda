/* WF-C3: Picard multi-order, conducto-radiative (wavefront probe).
 *
 * Scene: thin solid plate [0,0.1] + right fluid enclosure [0.1,1.1]
 *   Solid: lambda=1.15, cp=800, rho=1000, delta=0.0025
 *   Left face (-X): radiative (eps=1) facing radenv (T=280K)
 *   Right face (+X): radiative (eps=1) facing fluid (T=350K boundary)
 *   Top/Bottom: adiabatic
 *
 * 6 sub-configs varying picard_order, Tref, volumic_power:
 *   (1) Picard1, Tref=300 constant       -> T=314.9999..
 *   (2) Picard1, T4 reference (self-ref) -> T=320.3712..
 *   (3) Picard2, Tref=300 constant       -> T=320.3712..
 *   (4) Picard3, large DeltaT (200~500)  -> T=416.4023
 *   (5) Picard1, power=1000, Tref=300    -> T=324.2526..
 *   (6) Picard1, power=1000, T4 ref      -> T=327.9598..
 *
 * Probe at (0.05, 0, 0) (solid centre).
 *
 * Reference CPU test: test_sdis_picard.c
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
/* Physical constants                                                         */
/* ========================================================================== */
#define C3_LAMBDA    1.15
#define C3_RHO       1000.0
#define C3_CP        800.0
#define C3_DELTA     0.0025
#define C3_NREALS    10000

/* ========================================================================== */
/* Geometry: 12 vertices, 22 triangles (solid + right fluid)                  */
/* ========================================================================== */
static const double c3_vertices[12 * 3] = {
  0.0, -1.0, -1.0,   /*  0 */
  0.1, -1.0, -1.0,   /*  1 */
  0.0,  1.0, -1.0,   /*  2 */
  0.1,  1.0, -1.0,   /*  3 */
  0.0, -1.0,  1.0,   /*  4 */
  0.1, -1.0,  1.0,   /*  5 */
  0.0,  1.0,  1.0,   /*  6 */
  0.1,  1.0,  1.0,   /*  7 */
  1.1, -1.0, -1.0,   /*  8 */
  1.1,  1.0, -1.0,   /*  9 */
  1.1, -1.0,  1.0,   /* 10 */
  1.1,  1.0,  1.0    /* 11 */
};
static const size_t c3_nvertices = 12;

static const size_t c3_indices[22 * 3] = {
  0, 2, 1,  1, 2, 3,     /* Solid -Z  (tri 0,1)   */
  0, 4, 2,  2, 4, 6,     /* Solid -X  (tri 2,3)   */
  4, 5, 6,  6, 5, 7,     /* Solid +Z  (tri 4,5)   */
  3, 7, 1,  1, 7, 5,     /* Solid +X  (tri 6,7)   */
  2, 6, 3,  3, 6, 7,     /* Solid +Y  (tri 8,9)   */
  0, 1, 4,  4, 1, 5,     /* Solid -Y  (tri 10,11) */
  1,  3, 8,  8,  3,  9,  /* Fluid -Z  (tri 12,13) */
  5, 10, 7,  7, 10, 11,  /* Fluid +Z  (tri 14,15) */
  9, 11, 8,  8, 11, 10,  /* Fluid +X  (tri 16,17) */
  3,  7, 9,  9,  7, 11,  /* Fluid +Y  (tri 18,19) */
  1,  8, 5,  5,  8, 10   /* Fluid -Y  (tri 20,21) */
};
static const size_t c3_ntriangles = 22;

struct c3_geometry {
  const double* positions;
  const size_t* indices;
  struct sdis_interface** interfaces;
};

static void
c3_get_indices(const size_t itri, size_t ids[3], void* ctx)
{
  struct c3_geometry* g = ctx;
  CHK(itri < c3_ntriangles);
  ids[0] = g->indices[itri * 3 + 0];
  ids[1] = g->indices[itri * 3 + 1];
  ids[2] = g->indices[itri * 3 + 2];
}

static void
c3_get_position(const size_t ivert, double pos[3], void* ctx)
{
  struct c3_geometry* g = ctx;
  CHK(ivert < c3_nvertices);
  pos[0] = g->positions[ivert * 3 + 0];
  pos[1] = g->positions[ivert * 3 + 1];
  pos[2] = g->positions[ivert * 3 + 2];
}

static void
c3_get_interface(const size_t itri, struct sdis_interface** bound, void* ctx)
{
  struct c3_geometry* g = ctx;
  CHK(itri < c3_ntriangles && bound);
  *bound = g->interfaces[itri];
}

/* ========================================================================== */
/* Solid shader                                                               */
/* ========================================================================== */
struct c3_solid {
  double lambda;
  double rho;
  double cp;
  double volumic_power;
};

static double
c3_solid_get_calorific_capacity
  (const struct sdis_rwalk_vertex* vtx, struct sdis_data* data)
{
  CHK(vtx != NULL);
  return ((const struct c3_solid*)sdis_data_cget(data))->cp;
}

static double
c3_solid_get_thermal_conductivity
  (const struct sdis_rwalk_vertex* vtx, struct sdis_data* data)
{
  CHK(vtx != NULL);
  return ((const struct c3_solid*)sdis_data_cget(data))->lambda;
}

static double
c3_solid_get_volumic_mass
  (const struct sdis_rwalk_vertex* vtx, struct sdis_data* data)
{
  CHK(vtx != NULL);
  return ((const struct c3_solid*)sdis_data_cget(data))->rho;
}

static double
c3_solid_get_delta
  (const struct sdis_rwalk_vertex* vtx, struct sdis_data* data)
{
  (void)data;
  CHK(vtx != NULL);
  return C3_DELTA;
}

static double
c3_solid_get_temperature
  (const struct sdis_rwalk_vertex* vtx, struct sdis_data* data)
{
  (void)data;
  CHK(vtx != NULL);
  return SDIS_TEMPERATURE_NONE;
}

static double
c3_solid_get_volumic_power
  (const struct sdis_rwalk_vertex* vtx, struct sdis_data* data)
{
  CHK(vtx != NULL);
  return ((const struct c3_solid*)sdis_data_cget(data))->volumic_power;
}

/* ========================================================================== */
/* Interface shaders                                                          */
/* ========================================================================== */
struct c3_interf {
  double temperature;
  double h;
  double emissivity;
  double specular_fraction;
  double Tref;
};

static double
c3_interf_get_temperature
  (const struct sdis_interface_fragment* frag, struct sdis_data* data)
{
  CHK(frag && data);
  return ((const struct c3_interf*)sdis_data_cget(data))->temperature;
}

static double
c3_interf_get_convection_coef
  (const struct sdis_interface_fragment* frag, struct sdis_data* data)
{
  CHK(frag && data);
  return ((const struct c3_interf*)sdis_data_cget(data))->h;
}

static double
c3_interf_get_emissivity
  (const struct sdis_interface_fragment* frag,
   const unsigned source_id,
   struct sdis_data* data)
{
  (void)source_id;
  CHK(frag && data);
  return ((const struct c3_interf*)sdis_data_cget(data))->emissivity;
}

static double
c3_interf_get_specular_fraction
  (const struct sdis_interface_fragment* frag,
   const unsigned source_id,
   struct sdis_data* data)
{
  (void)source_id;
  CHK(frag && data);
  return ((const struct c3_interf*)sdis_data_cget(data))->specular_fraction;
}

static double
c3_interf_get_Tref
  (const struct sdis_interface_fragment* frag, struct sdis_data* data)
{
  CHK(frag && data);
  return ((const struct c3_interf*)sdis_data_cget(data))->Tref;
}

/* ========================================================================== */
/* Radiative environment shader                                                */
/* ========================================================================== */
struct c3_radenv {
  double temperature;
  double reference;
};

static double
c3_radenv_get_temperature
  (const struct sdis_radiative_ray* ray, struct sdis_data* data)
{
  (void)ray;
  CHK(data != NULL);
  return ((const struct c3_radenv*)sdis_data_cget(data))->temperature;
}

static double
c3_radenv_get_reference
  (const struct sdis_radiative_ray* ray, struct sdis_data* data)
{
  (void)ray;
  CHK(data != NULL);
  return ((const struct c3_radenv*)sdis_data_cget(data))->reference;
}

/* ========================================================================== */
/* Interface factory (mirrors CPU test)                                       */
/* ========================================================================== */
static void
c3_create_interface
  (struct sdis_device* dev,
   struct sdis_medium* front,
   struct sdis_medium* back,
   const struct c3_interf* ifp,
   struct sdis_interface** out_interf)
{
  struct sdis_interface_shader shader = SDIS_INTERFACE_SHADER_NULL;
  struct sdis_data* data = NULL;

  shader.front.temperature = c3_interf_get_temperature;
  shader.back.temperature = c3_interf_get_temperature;

  if(sdis_medium_get_type(front) != sdis_medium_get_type(back)) {
    shader.convection_coef = c3_interf_get_convection_coef;
  }
  if(sdis_medium_get_type(front) == SDIS_FLUID) {
    shader.front.emissivity = c3_interf_get_emissivity;
    shader.front.specular_fraction = c3_interf_get_specular_fraction;
    shader.front.reference_temperature = c3_interf_get_Tref;
  }
  if(sdis_medium_get_type(back) == SDIS_FLUID) {
    shader.back.emissivity = c3_interf_get_emissivity;
    shader.back.specular_fraction = c3_interf_get_specular_fraction;
    shader.back.reference_temperature = c3_interf_get_Tref;
  }
  shader.convection_coef_upper_bound =
    ifp->h > 0 ? ifp->h : 0;

  OK(sdis_data_create(dev, sizeof(struct c3_interf),
    ALIGNOF(struct c3_interf), NULL, &data));
  memcpy(sdis_data_get(data), ifp, sizeof(*ifp));

  OK(sdis_interface_create(dev, front, back, &shader, data, out_interf));
  OK(sdis_data_ref_put(data));
}

/* ========================================================================== */
/* Sub-test configuration                                                     */
/* ========================================================================== */
enum c3_iface_type {
  C3_ADIABATIC = 0,
  C3_SOLID_FLUID_mX = 1,
  C3_SOLID_FLUID_pX = 2,
  C3_BOUNDARY_pX = 3,
  C3_IFACE_COUNT = 4
};

struct c3_config {
  const char* label;
  size_t picard_order;
  enum sdis_diffusion_algorithm diff_algo;
  double t_range[2];
  double volumic_power;
  double radenv_temp;
  double radenv_ref;
  double Tref_mX;
  double Tref_pX;
  double Tref_boundary;
  double boundary_temp;
  double ref_T;      /* expected probe temperature */
};

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
  struct sdis_medium* dummy = NULL; /* dummy solid (lambda=0) */
  struct sdis_radiative_env* radenv = NULL;
  struct sdis_interface* interfaces[C3_IFACE_COUNT];
  struct c3_interf* pinterf_props[C3_IFACE_COUNT];
  struct c3_radenv* pradenv_props = NULL;
  struct c3_solid* psolid_props = NULL;
  struct sdis_interface* prim_interfaces[22];
  struct sdis_scene_create_args scn_args = SDIS_SCENE_CREATE_ARGS_DEFAULT;
  struct sdis_scene* scn = NULL;
  struct sdis_fluid_shader fluid_shader = DUMMY_FLUID_SHADER;
  struct sdis_solid_shader solid_shader = DUMMY_SOLID_SHADER;
  struct sdis_solid_shader dummy_shader = DUMMY_SOLID_SHADER;
  struct sdis_radiative_env_shader radenv_shader = SDIS_RADIATIVE_ENV_SHADER_NULL;
  struct c3_geometry geom;
  int n_pass = 0, n_total = 0;
  size_t ti;
  FILE* csv = NULL;
  (void)argc; (void)argv;

  /* 6 sub-test configurations */
  static const struct c3_config configs[] = {
    /* [0] Picard1, const Tref=300 */
    { "Picard1 const-Tref", 1, SDIS_DIFFUSION_WOS,
      {280.0, 350.0}, SDIS_VOLUMIC_POWER_NONE,
      280.0, 300.0,
      300.0, 300.0, 300.0, 350.0,
      314.99999999999989 },
    /* [1] Picard1, T4 self-reference */
    { "Picard1 T4-ref", 1, SDIS_DIFFUSION_DELTA_SPHERE,
      {280.0, 350.0}, SDIS_VOLUMIC_POWER_NONE,
      280.0, 280.0,
      312.12650299072266, 328.61602649893723, 350.0, 350.0,
      320.37126474482994 },
    /* [2] Picard2, const Tref=300 */
    { "Picard2 const-Tref", 2, SDIS_DIFFUSION_WOS,
      {280.0, 350.0}, SDIS_VOLUMIC_POWER_NONE,
      280.0, 300.0,
      300.0, 300.0, 300.0, 350.0,
      320.37126474482994 },
    /* [3] Picard3, large DeltaT */
    { "Picard3 large-dT", 3, SDIS_DIFFUSION_WOS,
      {200.0, 500.0}, SDIS_VOLUMIC_POWER_NONE,
      200.0, 200.0,
      350.0, 450.0, 500.0, 500.0,
      416.4023 },
    /* [4] Picard1+power, const Tref=300 */
    { "Picard1+P const-Tref", 1, SDIS_DIFFUSION_DELTA_SPHERE,
      {280.0, 350.0}, 1000.0,
      280.0, 300.0,
      300.0, 300.0, 300.0, 350.0,
      324.25266420769509 },
    /* [5] Picard1+power, T4 ref */
    { "Picard1+P T4-ref", 1, SDIS_DIFFUSION_WOS,
      {280.0, 350.0}, 1000.0,
      280.0, 280.0,
      318.75148773193359, 334.99422024159708, 350.0, 350.0,
      327.95981050850446 }
  };
  static const size_t n_configs = sizeof(configs) / sizeof(configs[0]);

  printf("=== WF-C3: Picard multi-order conducto-radiative (wavefront probe) ===\n");
  csv = csv_open("C3");

  OK(sdis_device_create(&SDIS_DEVICE_CREATE_ARGS_DEFAULT, &dev));

  /* ---- Fluid medium (T = NONE) ---- */
  fluid_shader.temperature = c3_solid_get_temperature;
  OK(sdis_fluid_create(dev, &fluid_shader, NULL, &fluid));

  /* ---- Main solid (lambda=1.15, mutable power) ---- */
  OK(sdis_data_create(dev, sizeof(struct c3_solid),
    ALIGNOF(struct c3_solid), NULL, &data));
  psolid_props = sdis_data_get(data);
  psolid_props->lambda = C3_LAMBDA;
  psolid_props->rho = C3_RHO;
  psolid_props->cp = C3_CP;
  psolid_props->volumic_power = SDIS_VOLUMIC_POWER_NONE;
  solid_shader.calorific_capacity = c3_solid_get_calorific_capacity;
  solid_shader.thermal_conductivity = c3_solid_get_thermal_conductivity;
  solid_shader.volumic_mass = c3_solid_get_volumic_mass;
  solid_shader.delta = c3_solid_get_delta;
  solid_shader.temperature = c3_solid_get_temperature;
  solid_shader.volumic_power = c3_solid_get_volumic_power;
  OK(sdis_solid_create(dev, &solid_shader, data, &solid));
  OK(sdis_data_ref_put(data));

  /* ---- Dummy solid (lambda=0, for adiabatic / boundary interfaces) ---- */
  OK(sdis_data_create(dev, sizeof(struct c3_solid),
    ALIGNOF(struct c3_solid), NULL, &data));
  {
    struct c3_solid* pd = sdis_data_get(data);
    pd->lambda = 0.0;
    pd->rho = C3_RHO;
    pd->cp = C3_CP;
    pd->volumic_power = SDIS_VOLUMIC_POWER_NONE;
  }
  dummy_shader.calorific_capacity = c3_solid_get_calorific_capacity;
  dummy_shader.thermal_conductivity = c3_solid_get_thermal_conductivity;
  dummy_shader.volumic_mass = c3_solid_get_volumic_mass;
  dummy_shader.delta = c3_solid_get_delta;
  dummy_shader.temperature = c3_solid_get_temperature;
  dummy_shader.volumic_power = c3_solid_get_volumic_power;
  OK(sdis_solid_create(dev, &dummy_shader, data, &dummy));
  OK(sdis_data_ref_put(data));

  /* ---- Radiative environment ---- */
  OK(sdis_data_create(dev, sizeof(struct c3_radenv),
    ALIGNOF(struct c3_radenv), NULL, &data));
  pradenv_props = sdis_data_get(data);
  pradenv_props->temperature = 280.0;
  pradenv_props->reference = 300.0;
  radenv_shader.temperature = c3_radenv_get_temperature;
  radenv_shader.reference_temperature = c3_radenv_get_reference;
  OK(sdis_radiative_env_create(dev, &radenv_shader, data, &radenv));
  OK(sdis_data_ref_put(data));

  /* ---- Interfaces ---- */
  {
    struct c3_interf ifp;

    /* [0] ADIABATIC: solid / dummy */
    ifp.temperature = SDIS_TEMPERATURE_NONE;
    ifp.h = -1.0;
    ifp.emissivity = -1.0;
    ifp.specular_fraction = -1.0;
    ifp.Tref = SDIS_TEMPERATURE_NONE;
    c3_create_interface(dev, solid, dummy, &ifp, &interfaces[C3_ADIABATIC]);

    /* [1] SOLID_FLUID_mX: solid / fluid (left face, eps=1, specular=0) */
    ifp.temperature = SDIS_TEMPERATURE_NONE;
    ifp.h = 0.0;
    ifp.emissivity = 1.0;
    ifp.specular_fraction = 0.0;
    ifp.Tref = 280.0;
    c3_create_interface(dev, solid, fluid, &ifp,
      &interfaces[C3_SOLID_FLUID_mX]);

    /* [2] SOLID_FLUID_pX: solid / fluid (right face, eps=1, specular=0) */
    ifp.temperature = SDIS_TEMPERATURE_NONE;
    ifp.h = 0.0;
    ifp.emissivity = 1.0;
    ifp.specular_fraction = 0.0;
    ifp.Tref = 350.0;
    c3_create_interface(dev, solid, fluid, &ifp,
      &interfaces[C3_SOLID_FLUID_pX]);

    /* [3] BOUNDARY_pX: fluid / dummy (boundary, T=350K) */
    ifp.temperature = 350.0;
    ifp.h = -1.0;
    ifp.emissivity = 1.0;
    ifp.specular_fraction = 0.0;
    ifp.Tref = 350.0;
    c3_create_interface(dev, fluid, dummy, &ifp,
      &interfaces[C3_BOUNDARY_pX]);
  }

  /* Get mutable pointers for dynamic reconfiguration */
  {
    size_t ci;
    for(ci = 0; ci < C3_IFACE_COUNT; ci++) {
      pinterf_props[ci] = sdis_data_get(
        sdis_interface_get_data(interfaces[ci]));
    }
    pradenv_props = sdis_data_get(
      sdis_radiative_env_get_data(radenv));
  }

  /* Release media refs (scene will hold them) */
  OK(sdis_medium_ref_put(solid));
  OK(sdis_medium_ref_put(fluid));
  OK(sdis_medium_ref_put(dummy));

  /* ---- Triangle-to-interface mapping ---- */
  /* Solid (tri 0-11) */
  prim_interfaces[0]  = prim_interfaces[1]  = interfaces[C3_ADIABATIC];
  prim_interfaces[2]  = prim_interfaces[3]  = interfaces[C3_SOLID_FLUID_mX];
  prim_interfaces[4]  = prim_interfaces[5]  = interfaces[C3_ADIABATIC];
  prim_interfaces[6]  = prim_interfaces[7]  = interfaces[C3_SOLID_FLUID_pX];
  prim_interfaces[8]  = prim_interfaces[9]  = interfaces[C3_ADIABATIC];
  prim_interfaces[10] = prim_interfaces[11] = interfaces[C3_ADIABATIC];
  /* Fluid (tri 12-21): all BOUNDARY_pX */
  prim_interfaces[12] = prim_interfaces[13] = interfaces[C3_BOUNDARY_pX];
  prim_interfaces[14] = prim_interfaces[15] = interfaces[C3_BOUNDARY_pX];
  prim_interfaces[16] = prim_interfaces[17] = interfaces[C3_BOUNDARY_pX];
  prim_interfaces[18] = prim_interfaces[19] = interfaces[C3_BOUNDARY_pX];
  prim_interfaces[20] = prim_interfaces[21] = interfaces[C3_BOUNDARY_pX];

  /* ---- Scene creation ---- */
  geom.positions = c3_vertices;
  geom.indices = c3_indices;
  geom.interfaces = prim_interfaces;

  scn_args.get_indices = c3_get_indices;
  scn_args.get_interface = c3_get_interface;
  scn_args.get_position = c3_get_position;
  scn_args.nprimitives = c3_ntriangles;
  scn_args.nvertices = c3_nvertices;
  scn_args.t_range[0] = 280.0;
  scn_args.t_range[1] = 350.0;
  scn_args.radenv = radenv;
  scn_args.context = &geom;
  OK(sdis_scene_create(dev, &scn_args, &scn));

  /* Release interfaces */
  {
    size_t ci;
    for(ci = 0; ci < C3_IFACE_COUNT; ci++)
      OK(sdis_interface_ref_put(interfaces[ci]));
  }
  OK(sdis_radiative_env_ref_put(radenv));

  /* ---- Run 6 sub-tests ---- */
  for(ti = 0; ti < n_configs; ti++) {
    const struct c3_config* cfg = &configs[ti];
    struct sdis_solve_probe_args args = SDIS_SOLVE_PROBE_ARGS_DEFAULT;
    struct sdis_estimator *est_wf = NULL, *est_df = NULL;
    int pass;

    printf("\n  [%lu] %s (picard=%lu)\n",
      (unsigned long)ti, cfg->label, (unsigned long)cfg->picard_order);

    /* Reconfigure radenv */
    pradenv_props->temperature = cfg->radenv_temp;
    pradenv_props->reference = cfg->radenv_ref;

    /* Reconfigure interface Tref / boundary temperature */
    pinterf_props[C3_SOLID_FLUID_mX]->Tref = cfg->Tref_mX;
    pinterf_props[C3_SOLID_FLUID_pX]->Tref = cfg->Tref_pX;
    pinterf_props[C3_BOUNDARY_pX]->Tref = cfg->Tref_boundary;
    pinterf_props[C3_BOUNDARY_pX]->temperature = cfg->boundary_temp;

    /* Reconfigure solid volumic power */
    psolid_props->volumic_power = cfg->volumic_power;

    /* Reconfigure temperature range if needed */
    {
      double t_range[2];
      t_range[0] = cfg->t_range[0];
      t_range[1] = cfg->t_range[1];
      OK(sdis_scene_set_temperature_range(scn, t_range));
    }

    /* Probe at solid centre */
    args.nrealisations = C3_NREALS;
    args.position[0] = 0.05;
    args.position[1] = 0.0;
    args.position[2] = 0.0;
    args.picard_order = cfg->picard_order;
    args.diff_algo = cfg->diff_algo;

    OK(sdis_solve_wavefront_probe(scn, &args, &est_wf));

    pass = p0_compare_analytic(est_wf, cfg->ref_T, P0_TOL_SIGMA);
    n_pass += pass;
    n_total++;

    /* CSV: primary row + complementary variant */
    {
      struct sdis_mc mc_csv;
      OK(sdis_estimator_get_temperature(est_wf, &mc_csv));
      csv_row(csv, "C3", cfg->label, "gpu_wf",
              cfg->diff_algo == SDIS_DIFFUSION_WOS ? "WoS" : "DS",
              0.05, 0.0, 0.0, INF, (int)cfg->picard_order,
              C3_NREALS, mc_csv.E, mc_csv.SE, cfg->ref_T);

    }

    if(P0_ENABLE_DIAG) {
      OK(sdis_solve_probe(scn, &args, &est_df));
    }

    p0_print_probe_result(0.05, est_wf, est_df, cfg->ref_T);

    OK(sdis_estimator_ref_put(est_wf));
    if(est_df) OK(sdis_estimator_ref_put(est_df));
  }

  csv_close(csv);
  fprintf(stdout, "\n  Primary: %d/%d configs pass (%.1f%%)\n",
    n_pass, n_total,
    100.0 * (double)n_pass / (double)n_total);

  CHK((double)n_pass / (double)n_total >= P0_PASS_RATE);

  printf("WF-C3: PASS\n");

  /* ---- Cleanup ---- */
  OK(sdis_scene_ref_put(scn));
  OK(sdis_device_ref_put(dev));

  CHK(mem_allocated_size() == 0);
  return 0;
}
