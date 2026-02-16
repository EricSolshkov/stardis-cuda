/* WF-C4: Transient flux + convection + radiation Picard (wavefront probe).
 *
 * Scene: solid slab [0, 0.1] x [-1,1]^2 + right fluid [0.1, 1.1]
 *   Solid: lambda=1.15, rho=1700, cp=800, delta=0.005
 *   Initial temperature: linear profile T(x) = u*(T2-T1)+T1,  u=x/0.1
 *     T1=306.334 (left), T2=294.941 (right)
 *
 *   Left face (-X): net flux phi=10000 W/m², h=2, eps=1, fluid T=330K
 *   Right face (+X): h=8, eps=1, fluid T=290K
 *   Top/Bottom/Front/Back: adiabatic
 *   Radiative environment: T=320K, T_ref=300K
 *
 * 15 probes: 3 x-positions (0.01, 0.05, 0.09) x 5 times (1000..10000)
 * picard_order = 1
 *
 * Reference: analytically computed (hardcoded), tolerance = 3σ.
 *
 * Reference CPU test: test_sdis_flux2.c
 */

#include "sdis.h"
#include "test_sdis_utils.h"
#include "test_sdis_wf_p0_utils.h"

#include <rsys/mem_allocator.h>
#include <stdio.h>
#include <math.h>
#include <string.h>

/* ========================================================================== */
/* Physical constants (identical to CPU test)                                  */
/* ========================================================================== */
#define C4_LAMBDA  1.15
#define C4_RHO     1700.0
#define C4_CP      800.0
#define C4_DELTA   0.005
#define C4_NREALS  10000
#define C4_PICARD  1

#define C4_TOL_SIGMA  3.0
#define C4_PASS_RATE   0.95

/* ========================================================================== */
/* Geometry (identical to CPU test_sdis_flux2.c)                              */
/* ========================================================================== */
static const double c4_vertices[12 * 3] = {
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
static const size_t c4_nvertices = 12;

static const size_t c4_indices[22 * 3] = {
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
static const size_t c4_ntriangles = 22;

struct c4_geometry {
  const double* positions;
  const size_t* indices;
  struct sdis_interface** interfaces;
};

static void
c4_get_indices(const size_t itri, size_t ids[3], void* ctx)
{
  struct c4_geometry* g = ctx;
  CHK(g && itri < c4_ntriangles);
  ids[0] = g->indices[itri * 3 + 0];
  ids[1] = g->indices[itri * 3 + 1];
  ids[2] = g->indices[itri * 3 + 2];
}

static void
c4_get_position(const size_t ivert, double pos[3], void* ctx)
{
  struct c4_geometry* g = ctx;
  CHK(g && ivert < c4_nvertices);
  pos[0] = g->positions[ivert * 3 + 0];
  pos[1] = g->positions[ivert * 3 + 1];
  pos[2] = g->positions[ivert * 3 + 2];
}

static void
c4_get_interface(const size_t itri, struct sdis_interface** bound, void* ctx)
{
  struct c4_geometry* g = ctx;
  CHK(g && bound && itri < c4_ntriangles);
  *bound = g->interfaces[itri];
}

/* ========================================================================== */
/* Solid medium                                                               */
/* ========================================================================== */
struct c4_solid {
  double lambda;
  double rho;
  double cp;
};

static double
c4_solid_get_cp(const struct sdis_rwalk_vertex* vtx, struct sdis_data* data)
{
  CHK(vtx && data);
  return ((struct c4_solid*)sdis_data_cget(data))->cp;
}

static double
c4_solid_get_lambda(const struct sdis_rwalk_vertex* vtx, struct sdis_data* data)
{
  CHK(vtx && data);
  return ((struct c4_solid*)sdis_data_cget(data))->lambda;
}

static double
c4_solid_get_rho(const struct sdis_rwalk_vertex* vtx, struct sdis_data* data)
{
  CHK(vtx && data);
  return ((struct c4_solid*)sdis_data_cget(data))->rho;
}

static double
c4_solid_get_delta(const struct sdis_rwalk_vertex* vtx, struct sdis_data* data)
{
  (void)data; CHK(vtx);
  return C4_DELTA;
}

static double
c4_solid_get_temperature(const struct sdis_rwalk_vertex* vtx,
                         struct sdis_data* data)
{
  (void)data; CHK(vtx);
  if(vtx->time > 0) {
    return SDIS_TEMPERATURE_NONE;
  } else {
    /* Initial temperature: linear profile between T1 and T2 */
    const double T1 = 306.334;
    const double T2 = 294.941;
    double u = vtx->P[0] / 0.1;
    if(u < 0.0) u = 0.0;
    if(u > 1.0) u = 1.0;
    return u * (T2 - T1) + T1;
  }
}

/* ========================================================================== */
/* Fluid medium                                                               */
/* ========================================================================== */
struct c4_fluid {
  double temperature;
};

static double
c4_fluid_get_temperature(const struct sdis_rwalk_vertex* vtx,
                         struct sdis_data* data)
{
  CHK(vtx && data);
  return ((struct c4_fluid*)sdis_data_cget(data))->temperature;
}

/* ========================================================================== */
/* Interfaces                                                                 */
/* ========================================================================== */
enum c4_interface_type {
  C4_ADIABATIC,
  C4_FIXED_TEMPERATURE,
  C4_SOLID_FLUID_WITH_FLUX,
  C4_SOLID_FLUID,
  C4_INTERFACES_COUNT__
};

struct c4_interf {
  double h;
  double emissivity;
  double phi;
  double temperature;
  double Tref;
};

static double
c4_interf_get_temperature(const struct sdis_interface_fragment* frag,
                          struct sdis_data* data)
{
  CHK(frag && data);
  return ((struct c4_interf*)sdis_data_cget(data))->temperature;
}

static double
c4_interf_get_Tref(const struct sdis_interface_fragment* frag,
                   struct sdis_data* data)
{
  CHK(frag && data);
  return ((struct c4_interf*)sdis_data_cget(data))->Tref;
}

static double
c4_interf_get_h(const struct sdis_interface_fragment* frag,
                struct sdis_data* data)
{
  CHK(frag && data);
  return ((struct c4_interf*)sdis_data_cget(data))->h;
}

static double
c4_interf_get_emissivity(const struct sdis_interface_fragment* frag,
                         const unsigned source_id, struct sdis_data* data)
{
  (void)source_id;
  CHK(frag && data);
  return ((struct c4_interf*)sdis_data_cget(data))->emissivity;
}

static double
c4_interf_get_specular(const struct sdis_interface_fragment* frag,
                       const unsigned source_id, struct sdis_data* data)
{
  (void)source_id; (void)frag; (void)data;
  return 0;
}

static double
c4_interf_get_flux(const struct sdis_interface_fragment* frag,
                   struct sdis_data* data)
{
  CHK(frag && data);
  return ((struct c4_interf*)sdis_data_cget(data))->phi;
}

/* ========================================================================== */
/* Radiative environment                                                      */
/* ========================================================================== */
static double
c4_radenv_get_temperature(const struct sdis_radiative_ray* ray,
                          struct sdis_data* data)
{
  (void)ray; (void)data;
  return 320.0;
}

static double
c4_radenv_get_Tref(const struct sdis_radiative_ray* ray,
                   struct sdis_data* data)
{
  (void)ray; (void)data;
  return 300.0;
}

/* ========================================================================== */
/* Helper: create interface                                                   */
/* ========================================================================== */
static void
c4_create_interface(struct sdis_device* dev,
                    struct sdis_medium* front,
                    struct sdis_medium* back,
                    const struct c4_interf* props,
                    int has_flux,
                    struct sdis_interface** out)
{
  struct sdis_interface_shader shader = SDIS_INTERFACE_SHADER_NULL;
  struct sdis_data* data = NULL;

  shader.front.temperature = c4_interf_get_temperature;
  shader.back.temperature = c4_interf_get_temperature;
  if(has_flux) {
    shader.front.flux = c4_interf_get_flux;
    shader.back.flux = c4_interf_get_flux;
  }
  if(sdis_medium_get_type(front) != sdis_medium_get_type(back)) {
    shader.convection_coef = c4_interf_get_h;
  }
  if(sdis_medium_get_type(front) == SDIS_FLUID) {
    shader.front.emissivity = c4_interf_get_emissivity;
    shader.front.specular_fraction = c4_interf_get_specular;
    shader.front.reference_temperature = c4_interf_get_Tref;
  }
  if(sdis_medium_get_type(back) == SDIS_FLUID) {
    shader.back.emissivity = c4_interf_get_emissivity;
    shader.back.specular_fraction = c4_interf_get_specular;
    shader.back.reference_temperature = c4_interf_get_Tref;
  }
  shader.convection_coef_upper_bound = (props->h > 0) ? props->h : 0;

  OK(sdis_data_create(dev, sizeof(struct c4_interf),
    ALIGNOF(struct c4_interf), NULL, &data));
  memcpy(sdis_data_get(data), props, sizeof(struct c4_interf));

  OK(sdis_interface_create(dev, front, back, &shader, data, out));
  OK(sdis_data_ref_put(data));
}

/* ========================================================================== */
/* Reference data (identical to CPU test — analytically computed)              */
/* ========================================================================== */
struct c4_probe {
  double x;
  double time;
  double ref;
};

static const struct c4_probe c4_probes[] = {
  {0.01, 1000.0,  481.72005748728628},
  {0.05, 1000.0,  335.19469995601020},
  {0.09, 1000.0,  299.94436943411478},
  {0.01, 2000.0,  563.21759568607558},
  {0.05, 2000.0,  392.79827670626440},
  {0.09, 2000.0,  324.89742556243448},
  {0.01, 3000.0,  620.25242712533577},
  {0.05, 3000.0,  444.73414407361213},
  {0.09, 3000.0,  359.44045704073852},
  {0.01, 4000.0,  665.65935222224493},
  {0.05, 4000.0,  490.32470982110840},
  {0.09, 4000.0,  393.89924931902408},
  {0.01, 10000.0, 830.44439052891505},
  {0.05, 10000.0, 664.82771620162805},
  {0.09, 10000.0, 533.92442748613928}
};
static const size_t c4_nprobes = sizeof(c4_probes) / sizeof(c4_probes[0]);

/* ========================================================================== */
/* Test body                                                                  */
/* ========================================================================== */
int
main(int argc, char** argv)
{
  struct sdis_device* dev = NULL;
  struct sdis_radiative_env* radenv = NULL;
  struct sdis_radiative_env_shader radenv_shader = SDIS_RADIATIVE_ENV_SHADER_NULL;
  struct sdis_scene* scn = NULL;
  struct sdis_medium* solid = NULL;
  struct sdis_medium* dummy = NULL;
  struct sdis_medium* fluid1 = NULL;
  struct sdis_medium* fluid2 = NULL;
  struct sdis_interface* interfaces[C4_INTERFACES_COUNT__];
  struct sdis_interface* prim_interfaces[22];
  struct c4_geometry geom;
  struct sdis_scene_create_args scn_args = SDIS_SCENE_CREATE_ARGS_DEFAULT;
  struct sdis_solid_shader solid_shader = SDIS_SOLID_SHADER_NULL;
  struct sdis_fluid_shader fluid_shader = DUMMY_FLUID_SHADER;
  struct sdis_data* data = NULL;
  struct c4_solid* sp = NULL;
  struct c4_fluid* fp = NULL;
  struct c4_interf ip;
  int n_pass = 0;
  size_t i;
  (void)argc; (void)argv;

  printf("=== WF-C4: Transient flux + convection + radiation Picard (wavefront) ===\n");

  OK(sdis_device_create(&SDIS_DEVICE_CREATE_ARGS_DEFAULT, &dev));

  /* ---- Radiative environment ---- */
  radenv_shader.temperature = c4_radenv_get_temperature;
  radenv_shader.reference_temperature = c4_radenv_get_Tref;
  OK(sdis_radiative_env_create(dev, &radenv_shader, NULL, &radenv));

  /* ---- Solid medium ---- */
  solid_shader.calorific_capacity = c4_solid_get_cp;
  solid_shader.thermal_conductivity = c4_solid_get_lambda;
  solid_shader.volumic_mass = c4_solid_get_rho;
  solid_shader.delta = c4_solid_get_delta;
  solid_shader.temperature = c4_solid_get_temperature;

  OK(sdis_data_create(dev, sizeof(struct c4_solid),
    ALIGNOF(struct c4_solid), NULL, &data));
  sp = sdis_data_get(data);
  sp->lambda = C4_LAMBDA;
  sp->rho = C4_RHO;
  sp->cp = C4_CP;
  OK(sdis_solid_create(dev, &solid_shader, data, &solid));
  OK(sdis_data_ref_put(data));

  /* ---- Dummy solid (for adiabatic boundaries) ---- */
  OK(sdis_data_create(dev, sizeof(struct c4_solid),
    ALIGNOF(struct c4_solid), NULL, &data));
  sp = sdis_data_get(data);
  sp->lambda = 0;
  sp->rho = C4_RHO;
  sp->cp = C4_CP;
  OK(sdis_solid_create(dev, &solid_shader, data, &dummy));
  OK(sdis_data_ref_put(data));

  /* ---- Fluid media ---- */
  fluid_shader.temperature = c4_fluid_get_temperature;

  OK(sdis_data_create(dev, sizeof(struct c4_fluid),
    ALIGNOF(struct c4_fluid), NULL, &data));
  fp = sdis_data_get(data);
  fp->temperature = 330.0;
  OK(sdis_fluid_create(dev, &fluid_shader, data, &fluid1));
  OK(sdis_data_ref_put(data));

  OK(sdis_data_create(dev, sizeof(struct c4_fluid),
    ALIGNOF(struct c4_fluid), NULL, &data));
  fp = sdis_data_get(data);
  fp->temperature = 290.0;
  OK(sdis_fluid_create(dev, &fluid_shader, data, &fluid2));
  OK(sdis_data_ref_put(data));

  /* ---- Interfaces ---- */
  /* Adiabatic (solid / dummy) */
  ip.h = 0;
  ip.emissivity = 0;
  ip.phi = SDIS_FLUX_NONE;
  ip.temperature = SDIS_TEMPERATURE_NONE;
  ip.Tref = SDIS_TEMPERATURE_NONE;
  c4_create_interface(dev, solid, dummy, &ip, 1, &interfaces[C4_ADIABATIC]);

  /* Fixed temperature (fluid2 / dummy) — right enclosure boundary */
  ip.h = 1;
  ip.emissivity = 1;
  ip.phi = SDIS_FLUX_NONE;
  ip.temperature = 280.0;
  ip.Tref = 300.0;
  c4_create_interface(dev, fluid2, dummy, &ip, 1, &interfaces[C4_FIXED_TEMPERATURE]);

  /* Solid-fluid with flux (solid / fluid1) — left face */
  ip.h = 2;
  ip.emissivity = 1;
  ip.phi = 10000.0;
  ip.temperature = SDIS_TEMPERATURE_NONE;
  ip.Tref = 300.0;
  c4_create_interface(dev, solid, fluid1, &ip, 1,
    &interfaces[C4_SOLID_FLUID_WITH_FLUX]);

  /* Solid-fluid (solid / fluid2) — right face of solid */
  ip.h = 8;
  ip.emissivity = 1;
  ip.phi = SDIS_FLUX_NONE;
  ip.temperature = SDIS_TEMPERATURE_NONE;
  ip.Tref = 300.0;
  c4_create_interface(dev, solid, fluid2, &ip, 1, &interfaces[C4_SOLID_FLUID]);

  /* ---- Per-primitive interface mapping (identical to CPU) ---- */
  prim_interfaces[0]  = prim_interfaces[1]  = interfaces[C4_ADIABATIC];
  prim_interfaces[2]  = prim_interfaces[3]  = interfaces[C4_SOLID_FLUID_WITH_FLUX];
  prim_interfaces[4]  = prim_interfaces[5]  = interfaces[C4_ADIABATIC];
  prim_interfaces[6]  = prim_interfaces[7]  = interfaces[C4_SOLID_FLUID];
  prim_interfaces[8]  = prim_interfaces[9]  = interfaces[C4_ADIABATIC];
  prim_interfaces[10] = prim_interfaces[11] = interfaces[C4_ADIABATIC];
  /* Right fluid enclosure */
  prim_interfaces[12] = prim_interfaces[13] = interfaces[C4_FIXED_TEMPERATURE];
  prim_interfaces[14] = prim_interfaces[15] = interfaces[C4_FIXED_TEMPERATURE];
  prim_interfaces[16] = prim_interfaces[17] = interfaces[C4_FIXED_TEMPERATURE];
  prim_interfaces[18] = prim_interfaces[19] = interfaces[C4_FIXED_TEMPERATURE];
  prim_interfaces[20] = prim_interfaces[21] = interfaces[C4_FIXED_TEMPERATURE];

  /* ---- Create scene ---- */
  geom.positions = c4_vertices;
  geom.indices = c4_indices;
  geom.interfaces = prim_interfaces;

  scn_args.get_indices = c4_get_indices;
  scn_args.get_interface = c4_get_interface;
  scn_args.get_position = c4_get_position;
  scn_args.nprimitives = c4_ntriangles;
  scn_args.nvertices = c4_nvertices;
  scn_args.t_range[0] = 300;
  scn_args.t_range[1] = 300;
  scn_args.context = &geom;
  scn_args.radenv = radenv;
  OK(sdis_scene_create(dev, &scn_args, &scn));

  /* ---- Release media and interfaces ---- */
  OK(sdis_medium_ref_put(solid));
  OK(sdis_medium_ref_put(dummy));
  OK(sdis_medium_ref_put(fluid1));
  OK(sdis_medium_ref_put(fluid2));
  OK(sdis_interface_ref_put(interfaces[C4_ADIABATIC]));
  OK(sdis_interface_ref_put(interfaces[C4_FIXED_TEMPERATURE]));
  OK(sdis_interface_ref_put(interfaces[C4_SOLID_FLUID_WITH_FLUX]));
  OK(sdis_interface_ref_put(interfaces[C4_SOLID_FLUID]));

  /* ================================================================== */
  /* Solve: 15 probes at (x, 0, 0), picard_order=1                     */
  /* ================================================================== */
  printf("  Running %lu probes, %d realisations each, picard_order=%d ...\n",
    (unsigned long)c4_nprobes, C4_NREALS, C4_PICARD);

  for(i = 0; i < c4_nprobes; i++) {
    struct sdis_solve_probe_args args = SDIS_SOLVE_PROBE_ARGS_DEFAULT;
    struct sdis_estimator* est_wf = NULL;
    struct sdis_mc mc;
    int pass;

    args.nrealisations = C4_NREALS;
    args.picard_order = C4_PICARD;
    args.position[0] = c4_probes[i].x;
    args.position[1] = 0;
    args.position[2] = 0;
    args.time_range[0] = c4_probes[i].time;
    args.time_range[1] = c4_probes[i].time;

    OK(sdis_solve_wavefront_probe(scn, &args, &est_wf));
    OK(sdis_estimator_get_temperature(est_wf, &mc));

    pass = p0_compare_analytic(est_wf, c4_probes[i].ref, C4_TOL_SIGMA);
    n_pass += pass;

    printf("  x=%.2f t=%.0f  wf=%.6f (SE=%.2e)  ref=%.6f  %s (%.1f sigma)\n",
      c4_probes[i].x, c4_probes[i].time,
      mc.E, mc.SE, c4_probes[i].ref,
      pass ? "PASS" : "FAIL",
      mc.SE > 0 ? fabs(mc.E - c4_probes[i].ref) / mc.SE : 0.0);

    OK(sdis_estimator_ref_put(est_wf));
  }

  /* ---- Summary ---- */
  printf("\n  Primary:    %d/%lu probes pass (%.1f%%)\n",
    n_pass, (unsigned long)c4_nprobes,
    100.0 * (double)n_pass / (double)c4_nprobes);
  CHK((double)n_pass / (double)c4_nprobes >= C4_PASS_RATE);

  printf("WF-C4: PASS\n");

  /* ---- Cleanup ---- */
  OK(sdis_radiative_env_ref_put(radenv));
  OK(sdis_scene_ref_put(scn));
  OK(sdis_device_ref_put(dev));

  CHK(mem_allocated_size() == 0);
  return 0;
}
