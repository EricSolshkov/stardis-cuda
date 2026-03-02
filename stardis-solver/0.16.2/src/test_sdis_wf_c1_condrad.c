/* WF-C1: Conducto-radiative coupling, Picard order 1 (wavefront probe).
 *
 * Scene: solid cube [-1,1]^3 with two fluid enclosures on Â±X sides.
 *   Solid: lambda=0.1, cp=1, rho=1, delta=0.1
 *   Left fluid enclosure: T0=300 K (Dirichlet + radiative)
 *   Right fluid enclosure: T1=310 K (Dirichlet + radiative)
 *   Emissivity=1 on solid/fluid faces, specular=1
 *   Surrounding solid: lambda=0 (no conduction escape)
 *
 * Analytic (picard1, linearised radiation):
 *   hr = 4 * sigma * Tref^3 * epsilon
 *   tmp = lambda / (2*lambda + thickness*hr) * (T1 - T0)
 *   Ts0 = T0 + tmp,  Ts1 = T1 - tmp
 *   T(x) = Ts0*(1-u) + Ts1*u,  u = (x+1)/thickness
 *
 * 9 probes in solid region x âˆ?[-0.8, 0.8] (y=0, z=0).
 * Dual validation A+B.
 *
 * Reference CPU test: test_sdis_conducto_radiative.c
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
#define C1_LAMBDA     0.1
#define C1_EMISSIVITY 1.0
#define C1_TREF       300.0
#define C1_T0         300.0
#define C1_T1         310.0
#define C1_THICKNESS  2.0
#define C1_NREALS     20000  /* higher count for radiation paths */
#define C1_NPROBES    9

/* ========================================================================== */
/* Geometry: 16 vertices, 32 triangles                                        */
/* ========================================================================== */
static const double c1_vertices[16 * 3] = {
  -1.0, -1.0, -1.0,   /*  0: inner cube */
   1.0, -1.0, -1.0,   /*  1 */
  -1.0,  1.0, -1.0,   /*  2 */
   1.0,  1.0, -1.0,   /*  3 */
  -1.0, -1.0,  1.0,   /*  4 */
   1.0, -1.0,  1.0,   /*  5 */
  -1.0,  1.0,  1.0,   /*  6 */
   1.0,  1.0,  1.0,   /*  7 */
  -1.5, -1.0, -1.0,   /*  8: outer shell */
   1.5, -1.0, -1.0,   /*  9 */
  -1.5,  1.0, -1.0,   /* 10 */
   1.5,  1.0, -1.0,   /* 11 */
  -1.5, -1.0,  1.0,   /* 12 */
   1.5, -1.0,  1.0,   /* 13 */
  -1.5,  1.0,  1.0,   /* 14 */
   1.5,  1.0,  1.0    /* 15 */
};
static const size_t c1_nvertices = 16;

static const size_t c1_indices[32 * 3] = {
  /* Solid cube (tri 0-11) */
  0, 2, 1,  1, 2, 3,     /* 0,1:  back  (-Z) */
  0, 4, 2,  2, 4, 6,     /* 2,3:  left  (-X) */
  4, 5, 6,  6, 5, 7,     /* 4,5:  front (+Z) */
  3, 7, 1,  1, 7, 5,     /* 6,7:  right (+X) */
  2, 6, 3,  3, 6, 7,     /* 8,9:  top   (+Y) */
  0, 1, 4,  4, 1, 5,     /* 10,11: bottom (-Y) */

  /* Left fluid enclosure (tri 12-21) */
   8, 10, 0,   0, 10, 2,  /* 12,13: back  */
   8, 12, 10, 10, 12, 14, /* 14,15: left (shell -X) */
  12,  4, 14, 14,  4,  6, /* 16,17: front */
  10, 14,  2,  2, 14,  6, /* 18,19: top   */
   8,  0, 12, 12,  0,  4, /* 20,21: bottom */

  /* Right fluid enclosure (tri 22-31) */
   1,  3,  9,  9,  3, 11, /* 22,23: back  */
   5, 13,  7,  7, 13, 15, /* 24,25: front */
  11, 15,  9,  9, 15, 13, /* 26,27: right (shell +X) */
   3,  7, 11, 11,  7, 15, /* 28,29: top   */
   1,  9,  5,  5,  9, 13  /* 30,31: bottom */
};
static const size_t c1_ntriangles = 32;

struct c1_geometry {
  const double* positions;
  const size_t* indices;
  struct sdis_interface** interfaces;
};

static void
c1_get_indices(const size_t itri, size_t ids[3], void* ctx)
{
  struct c1_geometry* g = ctx;
  CHK(itri < c1_ntriangles);
  ids[0] = g->indices[itri * 3 + 0];
  ids[1] = g->indices[itri * 3 + 1];
  ids[2] = g->indices[itri * 3 + 2];
}

static void
c1_get_position(const size_t ivert, double pos[3], void* ctx)
{
  struct c1_geometry* g = ctx;
  CHK(ivert < c1_nvertices);
  pos[0] = g->positions[ivert * 3 + 0];
  pos[1] = g->positions[ivert * 3 + 1];
  pos[2] = g->positions[ivert * 3 + 2];
}

static void
c1_get_interface(const size_t itri, struct sdis_interface** bound, void* ctx)
{
  struct c1_geometry* g = ctx;
  CHK(itri < c1_ntriangles && bound);
  *bound = g->interfaces[itri];
}

/* ========================================================================== */
/* Media shaders                                                              */
/* ========================================================================== */
struct c1_solid {
  double lambda;
  double initial_temperature;
};

static double
c1_temperature_unknown
  (const struct sdis_rwalk_vertex* vtx, struct sdis_data* data)
{
  (void)data;
  CHK(vtx != NULL);
  return SDIS_TEMPERATURE_NONE;
}

static double
c1_solid_get_calorific_capacity
  (const struct sdis_rwalk_vertex* vtx, struct sdis_data* data)
{
  (void)data;
  CHK(vtx != NULL);
  return 1.0;
}

static double
c1_solid_get_thermal_conductivity
  (const struct sdis_rwalk_vertex* vtx, struct sdis_data* data)
{
  CHK(vtx != NULL);
  return ((struct c1_solid*)sdis_data_cget(data))->lambda;
}

static double
c1_solid_get_volumic_mass
  (const struct sdis_rwalk_vertex* vtx, struct sdis_data* data)
{
  (void)data;
  CHK(vtx != NULL);
  return 1.0;
}

static double
c1_solid_get_delta
  (const struct sdis_rwalk_vertex* vtx, struct sdis_data* data)
{
  (void)data;
  CHK(vtx != NULL);
  return 0.1;
}

static double
c1_solid_get_temperature
  (const struct sdis_rwalk_vertex* vtx, struct sdis_data* data)
{
  CHK(vtx != NULL);
  if(vtx->time > 0)
    return SDIS_TEMPERATURE_NONE;
  else
    return ((struct c1_solid*)sdis_data_cget(data))->initial_temperature;
}

/* ========================================================================== */
/* Interface shaders                                                          */
/* ========================================================================== */
struct c1_interfac {
  double temperature;
  double convection_coef;
  double emissivity;
  double specular_fraction;
  double Tref;
};

static double
c1_interf_get_temperature
  (const struct sdis_interface_fragment* frag, struct sdis_data* data)
{
  CHK(frag && data);
  return ((const struct c1_interfac*)sdis_data_cget(data))->temperature;
}

static double
c1_interf_get_convection_coef
  (const struct sdis_interface_fragment* frag, struct sdis_data* data)
{
  CHK(frag && data);
  return ((const struct c1_interfac*)sdis_data_cget(data))->convection_coef;
}

static double
c1_interf_get_emissivity
  (const struct sdis_interface_fragment* frag,
   const unsigned source_id,
   struct sdis_data* data)
{
  (void)source_id;
  CHK(frag && data);
  return ((const struct c1_interfac*)sdis_data_cget(data))->emissivity;
}

static double
c1_interf_get_specular_fraction
  (const struct sdis_interface_fragment* frag,
   const unsigned source_id,
   struct sdis_data* data)
{
  (void)source_id;
  CHK(frag && data);
  return ((const struct c1_interfac*)sdis_data_cget(data))->specular_fraction;
}

static double
c1_interf_get_Tref
  (const struct sdis_interface_fragment* frag, struct sdis_data* data)
{
  CHK(frag && data);
  return ((const struct c1_interfac*)sdis_data_cget(data))->Tref;
}

/* ========================================================================== */
/* Interface factory (mirrors CPU test create_interface)                       */
/* ========================================================================== */
static void
c1_create_interface
  (struct sdis_device* dev,
   struct sdis_medium* front,
   struct sdis_medium* back,
   const struct c1_interfac* interf,
   struct sdis_interface** out_interf)
{
  struct sdis_interface_shader shader = SDIS_INTERFACE_SHADER_NULL;
  struct sdis_data* data = NULL;

  shader.front.temperature = c1_interf_get_temperature;
  shader.back.temperature = c1_interf_get_temperature;
  if(sdis_medium_get_type(front) != sdis_medium_get_type(back)) {
    shader.convection_coef = c1_interf_get_convection_coef;
  }
  if(sdis_medium_get_type(front) == SDIS_FLUID) {
    shader.front.emissivity = c1_interf_get_emissivity;
    shader.front.specular_fraction = c1_interf_get_specular_fraction;
    shader.front.reference_temperature = c1_interf_get_Tref;
  }
  if(sdis_medium_get_type(back) == SDIS_FLUID) {
    shader.back.emissivity = c1_interf_get_emissivity;
    shader.back.specular_fraction = c1_interf_get_specular_fraction;
    shader.back.reference_temperature = c1_interf_get_Tref;
  }
  shader.convection_coef_upper_bound =
    interf->convection_coef > 0 ? interf->convection_coef : 0;

  OK(sdis_data_create(dev, sizeof(struct c1_interfac),
    ALIGNOF(struct c1_interfac), NULL, &data));
  *((struct c1_interfac*)sdis_data_get(data)) = *interf;

  OK(sdis_interface_create(dev, front, back, &shader, data, out_interf));
  OK(sdis_data_ref_put(data));
}

/* ========================================================================== */
/* Analytic solution (picard1, linearised radiation in solid region)           */
/* ========================================================================== */
static double c1_hr;        /* set in main() */
static double c1_Ts0, c1_Ts1;

static double
c1_analytic(double u)
{
  /* u is normalised: u = (x + 1) / thickness
   * Called from p0_run_probe_sweep with x âˆ?[0, 1] representing
   * the probe parameter; we map it to physical x âˆ?[-0.8, 0.8] */
  double x = -0.8 + 1.6 * u;
  double t = (x + 1.0) / C1_THICKNESS;
  return c1_Ts0 * (1.0 - t) + c1_Ts1 * t;
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
  struct sdis_medium* solid = NULL;
  struct sdis_medium* solid2 = NULL;
  struct sdis_interface* interfaces[5];
  struct sdis_interface* prim_interfaces[32];
  struct sdis_scene_create_args scn_args = SDIS_SCENE_CREATE_ARGS_DEFAULT;
  struct sdis_fluid_shader fluid_shader = DUMMY_FLUID_SHADER;
  struct sdis_solid_shader solid_shader = DUMMY_SOLID_SHADER;
  struct sdis_scene* scn = NULL;
  struct c1_geometry geom;
  struct c1_interfac ifp;
  double tmp;
  size_t i;
  int n_pass_primary = 0, n_pass_diag = 0;
  (void)argc; (void)argv;

  printf("=== WF-C1: Conducto-radiative Picard1 (wavefront probe) ===\n");

  OK(sdis_device_create(&SDIS_DEVICE_CREATE_ARGS_DEFAULT, &dev));

  /* ---- Fluid medium (unknown temperature) ---- */
  fluid_shader.temperature = c1_temperature_unknown;
  OK(sdis_fluid_create(dev, &fluid_shader, NULL, &fluid));

  /* ---- Main solid (lambda=0.1, initial T = (T0+T1)/2) ---- */
  OK(sdis_data_create(dev, sizeof(struct c1_solid),
    ALIGNOF(struct c1_solid), NULL, &data));
  ((struct c1_solid*)sdis_data_get(data))->lambda = C1_LAMBDA;
  ((struct c1_solid*)sdis_data_get(data))->initial_temperature =
    (C1_T0 + C1_T1) / 2.0;
  solid_shader.calorific_capacity = c1_solid_get_calorific_capacity;
  solid_shader.thermal_conductivity = c1_solid_get_thermal_conductivity;
  solid_shader.volumic_mass = c1_solid_get_volumic_mass;
  solid_shader.delta = c1_solid_get_delta;
  solid_shader.temperature = c1_solid_get_temperature;
  OK(sdis_solid_create(dev, &solid_shader, data, &solid));
  OK(sdis_data_ref_put(data));

  /* ---- Surrounding solid (lambda=0, unknown temperature) ---- */
  OK(sdis_data_create(dev, sizeof(struct c1_solid),
    ALIGNOF(struct c1_solid), NULL, &data));
  ((struct c1_solid*)sdis_data_get(data))->lambda = 0.0;
  ((struct c1_solid*)sdis_data_get(data))->initial_temperature = 0.0;
  {
    struct sdis_solid_shader s2_shader = DUMMY_SOLID_SHADER;
    s2_shader.calorific_capacity = c1_solid_get_calorific_capacity;
    s2_shader.thermal_conductivity = c1_solid_get_thermal_conductivity;
    s2_shader.volumic_mass = c1_solid_get_volumic_mass;
    s2_shader.delta = c1_solid_get_delta;
    s2_shader.temperature = c1_temperature_unknown;
    OK(sdis_solid_create(dev, &s2_shader, data, &solid2));
  }
  OK(sdis_data_ref_put(data));

  /* ---- Interfaces (5 types, mirrors CPU test exactly) ---- */

  /* [0] solid/solid2: conduction-only (no radiation) */
  ifp.temperature = SDIS_TEMPERATURE_NONE;
  ifp.convection_coef = -1.0;
  ifp.emissivity = -1.0;
  ifp.specular_fraction = -1.0;
  ifp.Tref = C1_TREF;
  c1_create_interface(dev, solid, solid2, &ifp, &interfaces[0]);

  /* [1] solid/fluid: radiative emission (eps=1, specular=1) */
  ifp.temperature = SDIS_TEMPERATURE_NONE;
  ifp.convection_coef = 0.0;
  ifp.emissivity = C1_EMISSIVITY;
  ifp.specular_fraction = 1.0;
  ifp.Tref = C1_TREF;
  c1_create_interface(dev, solid, fluid, &ifp, &interfaces[1]);

  /* [2] fluid/solid2: perfect reflection (eps=0, specular=1) */
  ifp.temperature = SDIS_TEMPERATURE_NONE;
  ifp.convection_coef = 0.0;
  ifp.emissivity = 0.0;
  ifp.specular_fraction = 1.0;
  ifp.Tref = C1_TREF;
  c1_create_interface(dev, fluid, solid2, &ifp, &interfaces[2]);

  /* [3] fluid/solid2: T0=300 boundary (eps=1) */
  ifp.temperature = C1_T0;
  ifp.convection_coef = 0.0;
  ifp.emissivity = 1.0;
  ifp.specular_fraction = 1.0;
  ifp.Tref = C1_T0;
  c1_create_interface(dev, fluid, solid2, &ifp, &interfaces[3]);

  /* [4] fluid/solid2: T1=310 boundary (eps=1) */
  ifp.temperature = C1_T1;
  ifp.convection_coef = 0.0;
  ifp.emissivity = 1.0;
  ifp.specular_fraction = 1.0;
  ifp.Tref = C1_T1;
  c1_create_interface(dev, fluid, solid2, &ifp, &interfaces[4]);

  /* ---- Per-triangle interface mapping ---- */
  /* Solid cube (tri 0-11) */
  prim_interfaces[0] = prim_interfaces[1] = interfaces[0];  /* back -Z */
  prim_interfaces[2] = prim_interfaces[3] = interfaces[1];  /* left -X: rad */
  prim_interfaces[4] = prim_interfaces[5] = interfaces[0];  /* front +Z */
  prim_interfaces[6] = prim_interfaces[7] = interfaces[1];  /* right +X: rad */
  prim_interfaces[8] = prim_interfaces[9] = interfaces[0];  /* top +Y */
  prim_interfaces[10] = prim_interfaces[11] = interfaces[0]; /* bottom -Y */

  /* Left fluid enclosure (tri 12-21) */
  prim_interfaces[12] = prim_interfaces[13] = interfaces[2]; /* back: reflect */
  prim_interfaces[14] = prim_interfaces[15] = interfaces[3]; /* left shell: T0 */
  prim_interfaces[16] = prim_interfaces[17] = interfaces[2]; /* front: reflect */
  prim_interfaces[18] = prim_interfaces[19] = interfaces[2]; /* top: reflect */
  prim_interfaces[20] = prim_interfaces[21] = interfaces[2]; /* bottom: reflect */

  /* Right fluid enclosure (tri 22-31) */
  prim_interfaces[22] = prim_interfaces[23] = interfaces[2]; /* back: reflect */
  prim_interfaces[24] = prim_interfaces[25] = interfaces[2]; /* front: reflect */
  prim_interfaces[26] = prim_interfaces[27] = interfaces[4]; /* right shell: T1 */
  prim_interfaces[28] = prim_interfaces[29] = interfaces[2]; /* top: reflect */
  prim_interfaces[30] = prim_interfaces[31] = interfaces[2]; /* bottom: reflect */

  /* ---- Scene creation ---- */
  geom.positions = c1_vertices;
  geom.indices = c1_indices;
  geom.interfaces = prim_interfaces;

  scn_args.get_indices = c1_get_indices;
  scn_args.get_interface = c1_get_interface;
  scn_args.get_position = c1_get_position;
  scn_args.nprimitives = c1_ntriangles;
  scn_args.nvertices = c1_nvertices;
  scn_args.t_range[0] = C1_T0 < C1_T1 ? C1_T0 : C1_T1;
  scn_args.t_range[1] = C1_T0 > C1_T1 ? C1_T0 : C1_T1;
  scn_args.context = &geom;
  OK(sdis_scene_create(dev, &scn_args, &scn));

  /* Release media after scene takes refs */
  OK(sdis_medium_ref_put(solid));
  OK(sdis_medium_ref_put(solid2));
  OK(sdis_medium_ref_put(fluid));

  /* Release interfaces */
  for(i = 0; i < 5; i++)
    OK(sdis_interface_ref_put(interfaces[i]));

  /* ---- Compute analytic reference ---- */
  c1_hr = 4.0 * BOLTZMANN_CONSTANT
        * C1_TREF * C1_TREF * C1_TREF * C1_EMISSIVITY;
  tmp = C1_LAMBDA / (2.0 * C1_LAMBDA + C1_THICKNESS * c1_hr)
      * (C1_T1 - C1_T0);
  c1_Ts0 = C1_T0 + tmp;
  c1_Ts1 = C1_T1 - tmp;

  printf("  hr = %.6e, Ts0 = %.4f, Ts1 = %.4f\n", c1_hr, c1_Ts0, c1_Ts1);

  /* ---- Run probe sweep in solid region ---- */
  /* We probe x âˆ?[-0.8, 0.8], passing through c1_analytic()
   * which maps the normalised parameter u âˆ?[0,1] to physical x.
   * The sweep utility sets y_fixed=0, z_fixed=0 but we need probes
   * inside the solid cube which spans [-1,1]^3 so y=0, z=0 is fine. */
  {
    /* Manual sweep since c1_analytic maps u differently — batch */
    struct sdis_solve_probe_args args_arr[C1_NPROBES];
    struct sdis_estimator*       ests[C1_NPROBES];
    double xs[C1_NPROBES];
    double T_refs[C1_NPROBES];

    fprintf(stdout, "  Running %d probes (batch), %d realisations each ...\n",
      C1_NPROBES, C1_NREALS);

    for(i = 0; i < C1_NPROBES; i++) {
      double u = (double)i / (double)(C1_NPROBES - 1);
      double x = -0.8 + 1.6 * u;
      double t_norm = (x + 1.0) / C1_THICKNESS;

      xs[i]     = x;
      T_refs[i] = c1_Ts0 * (1.0 - t_norm) + c1_Ts1 * t_norm;

      args_arr[i] = SDIS_SOLVE_PROBE_ARGS_DEFAULT;
      args_arr[i].nrealisations = C1_NREALS;
      args_arr[i].position[0]   = x;
      args_arr[i].position[1]   = 0.0;
      args_arr[i].position[2]   = 0.0;
      args_arr[i].picard_order  = 1;
      args_arr[i].diff_algo     = SDIS_DIFFUSION_DELTA_SPHERE;
      ests[i] = NULL;
    }

    OK(sdis_solve_persistent_wavefront_probe_batch(
      scn, C1_NPROBES, args_arr, ests));

    for(i = 0; i < C1_NPROBES; i++) {
      struct sdis_estimator *est_df = NULL;

      n_pass_primary += p0_compare_analytic(ests[i], T_refs[i], P0_TOL_SIGMA);

      if(P0_ENABLE_DIAG) {
        OK(sdis_solve_probe(scn, &args_arr[i], &est_df));
        n_pass_diag += p0_diag_compare(ests[i], est_df, P0_DIAG_SIGMA);
      }

      p0_print_probe_result(xs[i], ests[i], est_df, T_refs[i]);

      OK(sdis_estimator_ref_put(ests[i]));
      if(est_df)
        OK(sdis_estimator_ref_put(est_df));
    }
  }

  fprintf(stdout, "  Primary:    %d/%d probes pass (%.1f%%)\n",
    n_pass_primary, C1_NPROBES,
    100.0 * (double)n_pass_primary / (double)C1_NPROBES);

  if(P0_ENABLE_DIAG)
    fprintf(stdout, "  Diagnostic: %d/%d probes consistent (%.1f%%)\n",
      n_pass_diag, C1_NPROBES,
      100.0 * (double)n_pass_diag / (double)C1_NPROBES);

  /* Only Primary decides PASS/FAIL */
  CHK((double)n_pass_primary / (double)C1_NPROBES >= P0_PASS_RATE);

  printf("WF-C1: PASS\n");

  /* ---- Cleanup ---- */
  OK(sdis_scene_ref_put(scn));
  OK(sdis_device_ref_put(dev));

  CHK(mem_allocated_size() == 0);
  return 0;
}
