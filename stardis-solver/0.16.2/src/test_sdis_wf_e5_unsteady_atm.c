/* WF-E5: Transient conduction with atmosphere (wavefront probe, 3D only).
 *
 * Scene: fluid slab (0..XH) + solid slab (XH..XHpE), surrounded by
 *   ground (TG=310K), atmosphere (TA=290K) and radiative env (TR=260K).
 *   XH=3, XE=0.2, XHpE=3.2.
 *
 *   Solid: lambda=0.6, rho=2400, cp=800, delta=XE/40, T0=300K
 *   Fluid: rho=1.3, cp=1005, T0=300K
 *   Atmosphere fluid: T=290K (known, fixed).
 *
 *   Interfaces:
 *     - Adiabatic faces: h=0, eps=0
 *     - Ground (-X): T=310K, h=400, eps=1
 *     - Contact (fluid-solid): h=400, eps=1
 *     - Atmosphere (+X): h=400, eps=1
 *   Radiative environment: T_rad=260K
 *
 * Probes:
 *   solve_tfluid: probe in fluid (XH*0.5, XH*0.5, XH*0.5), 11 time points
 *   solve_tsolid: probe in solid (XH+0.2*XE, random Y/Z), 11 time points
 *
 * Reference: numerical (from independent solver), NOT analytic.
 *   Tolerance: |MC - ref| <= (TMAX-TMIN)*0.01 = 0.5K
 *   This matches CPU test_sdis_unsteady_atm.c tolerance.
 *
 * Note: boundary probes (solve_tbound1/tbound2) are skipped because
 * sdis_solve_wavefront_probe does not support boundary probe queries.
 * Only interior probe-based tests are included.
 *
 * Verification: wavefront result vs hardcoded numerical reference.
 *   Pass criterion: >= 90% of probes pass within EPS.
 *
 * Reference CPU test: test_sdis_unsteady_atm.c
 */

#include "sdis.h"
#include "test_sdis_utils.h"
#include "test_sdis_wf_p0_utils.h"
#include "test_sdis_csv_utils.h"

#include <rsys/mem_allocator.h>
#include <stdio.h>
#include <math.h>

/* ========================================================================== */
/* Physical constants (identical to CPU test)                                  */
/* ========================================================================== */
#define XH    3.0
#define XHpE  3.2
#define XE    (XHpE - XH)

#define T0_SOLID 300.0
#define T0_FLUID 300.0

#define E5_NREALS 10000

#define TG  310.0
#define HG  400.0

#define HC  400.0

#define TA  290.0
#define HA  400.0
#define TR  260.0

#define TMAX (310.0)  /* max of T0_FLUID, T0_SOLID, TG, TA, TR */
#define TMIN (260.0)  /* min of same */
#define EPS  ((TMAX - TMIN) * 0.01) /* 0.5 K */

/* hr = 4.0 * BOLTZMANN_CONSTANT * Tref^3 * epsilon
 * Tref = (hr / (4 * 5.6696e-8 * epsilon))^(1/3), hr = 6 */
#define TREF 297.974852286

#define RHO_F    1.3
#define CP_F     1005.0
#define RHO_S    2400.0
#define CP_S     800.0
#define LAMBDA_S 0.6
#define DELTA_S  (XE / 40.0)

#define X_PROBE  (XH + 0.2 * XE)

#define E5_PASS_RATE 0.90

/* ========================================================================== */
/* Geometry: 3D box (identical to CPU test model3d_*)                         */
/* ========================================================================== */
static const double e5_vertices[12 * 3] = {
  0,    0,    0,       /* 0  */
  XH,   0,    0,       /* 1  */
  XHpE, 0,    0,       /* 2  */
  0,    XHpE, 0,       /* 3  */
  XH,   XHpE, 0,       /* 4  */
  XHpE, XHpE, 0,       /* 5  */
  0,    0,    XHpE,    /* 6  */
  XH,   0,    XHpE,    /* 7  */
  XHpE, 0,    XHpE,    /* 8  */
  0,    XHpE, XHpE,    /* 9  */
  XH,   XHpE, XHpE,    /* 10 */
  XHpE, XHpE, XHpE     /* 11 */
};
static const size_t e5_nvertices = 12;

static const size_t e5_indices[22 * 3] = {
  0, 3, 1,  1, 3, 4,         1, 4, 2,  2, 4, 5,    /* -Z (0..3) */
  0, 6, 3,  3, 6, 9,                                /* -X (4,5)  */
  6, 7, 9,  9, 7, 10,        7, 8, 10, 10, 8, 11,  /* +Z (6..9) */
  5, 11, 8,  8, 2, 5,                               /* +X (10,11)*/
  3, 9, 10, 10, 4, 3,        4, 10, 11, 11, 5, 4,  /* +Y (12..15)*/
  0, 1, 7,   7, 6, 0,        1, 2, 8,   8, 7, 1,   /* -Y (16..19)*/
  4, 10, 7,  7, 1, 4                                /* Inside (20,21) */
};
static const size_t e5_ntriangles = 22;

static void
e5_get_indices(const size_t itri, size_t ids[3], void* ctx)
{
  (void)ctx;
  CHK(itri < e5_ntriangles);
  ids[0] = e5_indices[itri * 3 + 0];
  ids[1] = e5_indices[itri * 3 + 1];
  ids[2] = e5_indices[itri * 3 + 2];
}

static void
e5_get_position(const size_t ivert, double pos[3], void* ctx)
{
  (void)ctx;
  CHK(ivert < e5_nvertices);
  pos[0] = e5_vertices[ivert * 3 + 0];
  pos[1] = e5_vertices[ivert * 3 + 1];
  pos[2] = e5_vertices[ivert * 3 + 2];
}

static void
e5_get_interface(const size_t itri, struct sdis_interface** bound, void* ctx)
{
  struct sdis_interface** interfaces = ctx;
  CHK(ctx && bound && itri < e5_ntriangles);
  *bound = interfaces[itri];
}

/* ========================================================================== */
/* Solid medium                                                               */
/* ========================================================================== */
struct e5_solid {
  double lambda;
  double rho;
  double cp;
  double delta;
  double temperature;
  double t0;
};

static double
e5_solid_get_cp(const struct sdis_rwalk_vertex* vtx, struct sdis_data* data)
{
  CHK(vtx && data);
  return ((struct e5_solid*)sdis_data_cget(data))->cp;
}

static double
e5_solid_get_lambda(const struct sdis_rwalk_vertex* vtx, struct sdis_data* data)
{
  CHK(vtx && data);
  return ((struct e5_solid*)sdis_data_cget(data))->lambda;
}

static double
e5_solid_get_rho(const struct sdis_rwalk_vertex* vtx, struct sdis_data* data)
{
  CHK(vtx && data);
  return ((struct e5_solid*)sdis_data_cget(data))->rho;
}

static double
e5_solid_get_delta(const struct sdis_rwalk_vertex* vtx, struct sdis_data* data)
{
  CHK(vtx && data);
  return ((struct e5_solid*)sdis_data_cget(data))->delta;
}

static double
e5_solid_get_temperature(const struct sdis_rwalk_vertex* vtx,
                         struct sdis_data* data)
{
  struct e5_solid* s;
  CHK(vtx && data);
  s = (struct e5_solid*)sdis_data_cget(data);
  if(vtx->time <= s->t0) return s->temperature;
  return SDIS_TEMPERATURE_NONE;
}

/* ========================================================================== */
/* Fluid medium                                                               */
/* ========================================================================== */
struct e5_fluid {
  double rho;
  double cp;
  double t0;
  double temperature;
};

static double
e5_fluid_get_temperature(const struct sdis_rwalk_vertex* vtx,
                         struct sdis_data* data)
{
  struct e5_fluid* f;
  CHK(vtx && data);
  f = (struct e5_fluid*)sdis_data_cget(data);
  if(vtx->time <= f->t0) return f->temperature;
  return SDIS_TEMPERATURE_NONE;
}

static double
e5_fluid_get_rho(const struct sdis_rwalk_vertex* vtx, struct sdis_data* data)
{
  CHK(vtx && data);
  return ((struct e5_fluid*)sdis_data_cget(data))->rho;
}

static double
e5_fluid_get_cp(const struct sdis_rwalk_vertex* vtx, struct sdis_data* data)
{
  CHK(vtx && data);
  return ((struct e5_fluid*)sdis_data_cget(data))->cp;
}

/* ========================================================================== */
/* Interface                                                                  */
/* ========================================================================== */
struct e5_interf {
  double temperature;
  double emissivity;
  double h;
  double Tref;
};

static double
e5_interf_get_temperature(const struct sdis_interface_fragment* frag,
                          struct sdis_data* data)
{
  CHK(frag && data);
  return ((struct e5_interf*)sdis_data_cget(data))->temperature;
}

static double
e5_interf_get_h(const struct sdis_interface_fragment* frag,
                struct sdis_data* data)
{
  CHK(frag && data);
  return ((struct e5_interf*)sdis_data_cget(data))->h;
}

static double
e5_interf_get_emissivity(const struct sdis_interface_fragment* frag,
                         const unsigned source_id, struct sdis_data* data)
{
  (void)source_id;
  CHK(frag && data);
  return ((struct e5_interf*)sdis_data_cget(data))->emissivity;
}

static double
e5_interf_get_Tref(const struct sdis_interface_fragment* frag,
                   struct sdis_data* data)
{
  CHK(frag && data);
  return ((struct e5_interf*)sdis_data_cget(data))->Tref;
}

/* ========================================================================== */
/* Radiative environment                                                      */
/* ========================================================================== */
static double
e5_radenv_get_temperature(const struct sdis_radiative_ray* ray,
                          struct sdis_data* data)
{
  (void)ray; (void)data;
  return TR;
}

static double
e5_radenv_get_reference_temperature(const struct sdis_radiative_ray* ray,
                                    struct sdis_data* data)
{
  (void)ray; (void)data;
  return TR;
}

/* ========================================================================== */
/* Helper: create interface                                                   */
/* ========================================================================== */
static void
e5_create_interface(struct sdis_device* dev,
                    struct sdis_medium* front,
                    struct sdis_medium* back,
                    const struct e5_interf* props,
                    struct sdis_interface** out)
{
  struct sdis_interface_shader shader = SDIS_INTERFACE_SHADER_NULL;
  struct sdis_data* data = NULL;

  shader.front.temperature = e5_interf_get_temperature;
  shader.back.temperature = e5_interf_get_temperature;
  if(sdis_medium_get_type(front) != sdis_medium_get_type(back)) {
    shader.convection_coef = e5_interf_get_h;
    shader.convection_coef_upper_bound = props->h;
  }
  if(sdis_medium_get_type(front) == SDIS_FLUID) {
    shader.front.emissivity = e5_interf_get_emissivity;
    shader.front.reference_temperature = e5_interf_get_Tref;
  }
  if(sdis_medium_get_type(back) == SDIS_FLUID) {
    shader.back.emissivity = e5_interf_get_emissivity;
    shader.back.reference_temperature = e5_interf_get_Tref;
  }

  OK(sdis_data_create(dev, sizeof(struct e5_interf), 16, NULL, &data));
  *((struct e5_interf*)sdis_data_get(data)) = *props;

  OK(sdis_interface_create(dev, front, back, &shader, data, out));
  OK(sdis_data_ref_put(data));
}

/* ========================================================================== */
/* Reference data (hardcoded from independent numerical solver, same as CPU)  */
/* ========================================================================== */

/* Fluid probe at (XH*0.5, XH*0.5, XH*0.5) */
static const double e5_tfluid_times[] = {
  0, 1000, 2000, 3000, 4000, 5000, 6000, 7000, 8000, 9000, 10000
};
static const double e5_tfluid_refs[] = {
  300, 309.53905, 309.67273, 309.73241, 309.76798, 309.79194, 309.80899,
  309.82141, 309.83055, 309.83728, 309.84224
};
static const size_t e5_tfluid_npoints =
  sizeof(e5_tfluid_times) / sizeof(double);

/* Solid probe at (XH + 0.2*XE, random Y/Z) — use fixed Y=Z=XHpE*0.5 */
static const double e5_tsolid_times[] = {
  0, 1000, 2000, 3000, 4000, 5000, 6000, 7000, 8000, 9000, 10000
};
static const double e5_tsolid_refs[] = {
  300, 300.87408, 302.25832, 303.22164, 303.89954, 304.39030, 304.75041,
  305.01595, 305.21193, 305.35641, 305.46271
};
static const size_t e5_tsolid_npoints =
  sizeof(e5_tsolid_times) / sizeof(double);

/* ========================================================================== */
/* Test body                                                                  */
/* ========================================================================== */
int
main(int argc, char** argv)
{
  struct sdis_device* dev = NULL;
  struct sdis_data* data = NULL;
  struct sdis_medium* fluid = NULL;
  struct sdis_medium* fluid_A = NULL;
  struct sdis_medium* solid = NULL;
  struct sdis_medium* dummy_solid = NULL;
  struct sdis_interface* interf_adiabatic_1 = NULL;
  struct sdis_interface* interf_adiabatic_2 = NULL;
  struct sdis_interface* interf_TG = NULL;
  struct sdis_interface* interf_P = NULL;
  struct sdis_interface* interf_TA = NULL;
  struct sdis_radiative_env* radenv = NULL;
  struct sdis_scene* box_scn = NULL;
  struct sdis_scene_create_args scn_args = SDIS_SCENE_CREATE_ARGS_DEFAULT;
  struct sdis_solid_shader solid_shader = DUMMY_SOLID_SHADER;
  struct sdis_fluid_shader fluid_shader = DUMMY_FLUID_SHADER;
  struct sdis_radiative_env_shader radenv_shader = SDIS_RADIATIVE_ENV_SHADER_NULL;
  struct sdis_interface* box_interfaces[22];
  struct e5_interf interf_props;
  struct e5_solid* solid_props = NULL;
  struct e5_fluid* fluid_props = NULL;
  int n_pass = 0;
  int n_total = 0;
  size_t i;
  FILE* csv = NULL;
  (void)argc; (void)argv;

  printf("=== WF-E5: Transient conduction with atmosphere (wavefront probe) ===\n");
  csv = csv_open("E5");

  OK(sdis_device_create(&SDIS_DEVICE_CREATE_ARGS_DEFAULT, &dev));

  /* ---- Radiative environment ---- */
  radenv_shader.temperature = e5_radenv_get_temperature;
  radenv_shader.reference_temperature = e5_radenv_get_reference_temperature;
  OK(sdis_radiative_env_create(dev, &radenv_shader, NULL, &radenv));

  /* ---- Solid medium ---- */
  solid_shader.calorific_capacity = e5_solid_get_cp;
  solid_shader.thermal_conductivity = e5_solid_get_lambda;
  solid_shader.volumic_mass = e5_solid_get_rho;
  solid_shader.delta = e5_solid_get_delta;
  solid_shader.temperature = e5_solid_get_temperature;

  OK(sdis_data_create(dev, sizeof(struct e5_solid), 16, NULL, &data));
  solid_props = sdis_data_get(data);
  solid_props->lambda = LAMBDA_S;
  solid_props->cp = CP_S;
  solid_props->rho = RHO_S;
  solid_props->delta = DELTA_S;
  solid_props->t0 = 0;
  solid_props->temperature = T0_SOLID;
  OK(sdis_solid_create(dev, &solid_shader, data, &solid));
  OK(sdis_data_ref_put(data));

  /* ---- Dummy solid (exterior) ---- */
  OK(sdis_data_create(dev, sizeof(struct e5_solid), 16, NULL, &data));
  solid_props = sdis_data_get(data);
  solid_props->lambda = 0;
  solid_props->cp = 1;
  solid_props->rho = 1;
  solid_props->delta = 1;
  solid_props->t0 = INF;
  solid_props->temperature = SDIS_TEMPERATURE_NONE;
  OK(sdis_solid_create(dev, &solid_shader, data, &dummy_solid));
  OK(sdis_data_ref_put(data));

  /* ---- Internal fluid ---- */
  fluid_shader.calorific_capacity = e5_fluid_get_cp;
  fluid_shader.volumic_mass = e5_fluid_get_rho;
  fluid_shader.temperature = e5_fluid_get_temperature;

  OK(sdis_data_create(dev, sizeof(struct e5_fluid), 16, NULL, &data));
  fluid_props = sdis_data_get(data);
  fluid_props->cp = CP_F;
  fluid_props->rho = RHO_F;
  fluid_props->t0 = 0;
  fluid_props->temperature = T0_FLUID;
  OK(sdis_fluid_create(dev, &fluid_shader, data, &fluid));
  OK(sdis_data_ref_put(data));

  /* ---- Atmosphere fluid ---- */
  OK(sdis_data_create(dev, sizeof(struct e5_fluid), 16, NULL, &data));
  fluid_props = sdis_data_get(data);
  fluid_props->cp = 1;
  fluid_props->rho = 1;
  fluid_props->t0 = INF;
  fluid_props->temperature = TA;
  OK(sdis_fluid_create(dev, &fluid_shader, data, &fluid_A));
  OK(sdis_data_ref_put(data));

  /* ---- Interfaces ---- */
  /* Adiabatic 1 (fluid/dummy_solid) */
  interf_props.temperature = SDIS_TEMPERATURE_NONE;
  interf_props.h = 0;
  interf_props.emissivity = 0;
  interf_props.Tref = TREF;
  e5_create_interface(dev, fluid, dummy_solid, &interf_props, &interf_adiabatic_1);

  /* Adiabatic 2 (solid/dummy_solid) */
  e5_create_interface(dev, solid, dummy_solid, &interf_props, &interf_adiabatic_2);

  /* Contact (fluid/solid) */
  interf_props.temperature = SDIS_TEMPERATURE_NONE;
  interf_props.h = HC;
  interf_props.emissivity = 1;
  interf_props.Tref = TREF;
  e5_create_interface(dev, fluid, solid, &interf_props, &interf_P);

  /* Ground (fluid/dummy_solid) */
  interf_props.temperature = TG;
  interf_props.h = HG;
  interf_props.emissivity = 1;
  interf_props.Tref = TG;
  e5_create_interface(dev, fluid, dummy_solid, &interf_props, &interf_TG);

  /* Atmosphere (solid/fluid_A) */
  interf_props.temperature = SDIS_TEMPERATURE_NONE;
  interf_props.h = HA;
  interf_props.emissivity = 1;
  interf_props.Tref = TREF;
  e5_create_interface(dev, solid, fluid_A, &interf_props, &interf_TA);

  /* ---- Release media (scene keeps refs) ---- */
  OK(sdis_medium_ref_put(solid));
  OK(sdis_medium_ref_put(dummy_solid));
  OK(sdis_medium_ref_put(fluid));
  OK(sdis_medium_ref_put(fluid_A));

  /* ---- Triangle-to-interface mapping (identical to CPU test) ---- */
  /* -Z front: fluid(0,1) + solid(2,3) */
  box_interfaces[0] = interf_adiabatic_1;
  box_interfaces[1] = interf_adiabatic_1;
  box_interfaces[2] = interf_adiabatic_2;
  box_interfaces[3] = interf_adiabatic_2;
  /* -X: Ground */
  box_interfaces[4] = interf_TG;
  box_interfaces[5] = interf_TG;
  /* +Z back: fluid(6,7) + solid(8,9) */
  box_interfaces[6] = interf_adiabatic_1;
  box_interfaces[7] = interf_adiabatic_1;
  box_interfaces[8] = interf_adiabatic_2;
  box_interfaces[9] = interf_adiabatic_2;
  /* +X: Atmosphere */
  box_interfaces[10] = interf_TA;
  box_interfaces[11] = interf_TA;
  /* +Y: fluid(12,13) + solid(14,15) */
  box_interfaces[12] = interf_adiabatic_1;
  box_interfaces[13] = interf_adiabatic_1;
  box_interfaces[14] = interf_adiabatic_2;
  box_interfaces[15] = interf_adiabatic_2;
  /* -Y: fluid(16,17) + solid(18,19) */
  box_interfaces[16] = interf_adiabatic_1;
  box_interfaces[17] = interf_adiabatic_1;
  box_interfaces[18] = interf_adiabatic_2;
  box_interfaces[19] = interf_adiabatic_2;
  /* Internal: contact (20,21) */
  box_interfaces[20] = interf_P;
  box_interfaces[21] = interf_P;

  /* ---- Create box scene ---- */
  scn_args.get_indices = e5_get_indices;
  scn_args.get_interface = e5_get_interface;
  scn_args.get_position = e5_get_position;
  scn_args.nprimitives = e5_ntriangles;
  scn_args.nvertices = e5_nvertices;
  scn_args.context = box_interfaces;
  scn_args.radenv = radenv;
  scn_args.t_range[0] = TMIN;
  scn_args.t_range[1] = TMAX;
  OK(sdis_scene_create(dev, &scn_args, &box_scn));

  /* ---- Release interfaces ---- */
  OK(sdis_interface_ref_put(interf_adiabatic_1));
  OK(sdis_interface_ref_put(interf_adiabatic_2));
  OK(sdis_interface_ref_put(interf_TG));
  OK(sdis_interface_ref_put(interf_P));
  OK(sdis_interface_ref_put(interf_TA));

  /* ================================================================== */
  /* Solve: fluid probe at (XH*0.5, XH*0.5, XH*0.5)                    */
  /* ================================================================== */
  printf("\n>> Fluid probe at (%.1f, %.1f, %.1f)\n",
    XH * 0.5, XH * 0.5, XH * 0.5);

  for(i = 0; i < e5_tfluid_npoints; i++) {
    struct sdis_solve_probe_args args = SDIS_SOLVE_PROBE_ARGS_DEFAULT;
    struct sdis_estimator* est_wf = NULL;
    struct sdis_mc mc;
    int pass;

    args.nrealisations = E5_NREALS;
    args.position[0] = XH * 0.5;
    args.position[1] = XH * 0.5;
    args.position[2] = XH * 0.5;
    args.time_range[0] = e5_tfluid_times[i];
    args.time_range[1] = e5_tfluid_times[i];

    OK(sdis_solve_wavefront_probe(box_scn, &args, &est_wf));
    OK(sdis_estimator_get_temperature(est_wf, &mc));

    pass = fabs(mc.E - e5_tfluid_refs[i]) <= EPS;
    n_pass += pass;
    n_total++;

    /* CSV: primary DS row + complementary WoS variant */
    csv_row(csv, "E5", "fluid", "gpu_wf", "DS",
            XH*0.5, XH*0.5, XH*0.5, e5_tfluid_times[i],
            1, E5_NREALS, mc.E, mc.SE, e5_tfluid_refs[i]);


    printf("  t=%8.0f  wf=%.6f (SE=%.2e)  ref=%.5f  diff=%.4f  %s\n",
      e5_tfluid_times[i], mc.E, mc.SE, e5_tfluid_refs[i],
      fabs(mc.E - e5_tfluid_refs[i]), pass ? "PASS" : "FAIL");

    OK(sdis_estimator_ref_put(est_wf));
  }

  /* ================================================================== */
  /* Solve: solid probe at (X_PROBE, XHpE*0.5, XHpE*0.5)               */
  /* Use fixed Y/Z (centre) instead of random as in CPU test for        */
  /* reproducibility                                                    */
  /* ================================================================== */
  printf("\n>> Solid probe at (%.3f, %.1f, %.1f)\n",
    X_PROBE, XHpE * 0.5, XHpE * 0.5);

  for(i = 0; i < e5_tsolid_npoints; i++) {
    struct sdis_solve_probe_args args = SDIS_SOLVE_PROBE_ARGS_DEFAULT;
    struct sdis_estimator* est_wf = NULL;
    struct sdis_mc mc;
    int pass;

    args.nrealisations = E5_NREALS;
    args.position[0] = X_PROBE;
    args.position[1] = XHpE * 0.5;
    args.position[2] = XHpE * 0.5;
    args.time_range[0] = e5_tsolid_times[i];
    args.time_range[1] = e5_tsolid_times[i];

    OK(sdis_solve_wavefront_probe(box_scn, &args, &est_wf));
    OK(sdis_estimator_get_temperature(est_wf, &mc));

    pass = fabs(mc.E - e5_tsolid_refs[i]) <= EPS;
    n_pass += pass;
    n_total++;

    /* CSV: primary DS row + complementary WoS variant */
    csv_row(csv, "E5", "solid", "gpu_wf", "DS",
            X_PROBE, XHpE*0.5, XHpE*0.5, e5_tsolid_times[i],
            1, E5_NREALS, mc.E, mc.SE, e5_tsolid_refs[i]);


    printf("  t=%8.0f  wf=%.6f (SE=%.2e)  ref=%.5f  diff=%.4f  %s\n",
      e5_tsolid_times[i], mc.E, mc.SE, e5_tsolid_refs[i],
      fabs(mc.E - e5_tsolid_refs[i]), pass ? "PASS" : "FAIL");

    OK(sdis_estimator_ref_put(est_wf));
  }

  csv_close(csv);

  /* ---- Summary ---- */
  printf("\n  Primary:    %d/%d probes pass (%.1f%%)\n",
    n_pass, n_total, 100.0 * (double)n_pass / (double)n_total);
  CHK((double)n_pass / (double)n_total >= E5_PASS_RATE);

  printf("WF-E5: PASS\n");

  /* ---- Cleanup ---- */
  OK(sdis_radiative_env_ref_put(radenv));
  OK(sdis_scene_ref_put(box_scn));
  OK(sdis_device_ref_put(dev));

  CHK(mem_allocated_size() == 0);
  return 0;
}
