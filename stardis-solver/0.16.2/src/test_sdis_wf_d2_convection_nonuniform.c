/* WF-D2: Non-uniform convection coefficients, steady-state (wavefront probe).
 *
 * Scene: unit cube fluid [0,1]^3, rho=25, cp=2
 *   6 faces with DIFFERENT H and T values:
 *     -X: H0=100,  T0=300      +X: H1=30,   T1=310
 *     -Y: H2=3020, T2=320      +Y: H3=7300, T3=330
 *     +Z: H4=3400, T4=340      -Z: H5=50,   T5=350
 *
 * Analytic (lumped parameter, steady state, S_i = V = 1):
 *   nu  = sum(Hi) / (rho*cp)
 *       = (100+30+3020+7300+3400+50) / 50 = 278
 *   Tinf = sum(Hi*Ti) / sum(Hi)
 *        = (100*300+30*310+3020*320+7300*330+3400*340+50*350) / 13900
 *        = 4573900/13900 ≈ 329.1367 K
 *
 * Reference CPU test: test_sdis_convection_non_uniform.c
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
#define D2_T0     300.0    /* -X face */
#define D2_T1     310.0    /* +X face */
#define D2_T2     320.0    /* -Y face */
#define D2_T3     330.0    /* +Y face */
#define D2_T4     340.0    /* +Z face */
#define D2_T5     350.0    /* -Z face */

#define D2_HC0    100.0    /* -X face */
#define D2_HC1     30.0    /* +X face */
#define D2_HC2   3020.0    /* -Y face */
#define D2_HC3   7300.0    /* +Y face */
#define D2_HC4   3400.0    /* +Z face */
#define D2_HC5     50.0    /* -Z face */

#define D2_RHO     25.0
#define D2_CP       2.0

/* Analytic steady-state temperature (lumped parameter) */
#define D2_SUM_H   (D2_HC0 + D2_HC1 + D2_HC2 + D2_HC3 + D2_HC4 + D2_HC5)
#define D2_SUM_HT  (D2_HC0*D2_T0 + D2_HC1*D2_T1 + D2_HC2*D2_T2  \
                   + D2_HC3*D2_T3 + D2_HC4*D2_T4 + D2_HC5*D2_T5)
#define D2_TINF    (D2_SUM_HT / D2_SUM_H)

#define D2_NREALS  100000
#define D2_NPROBES 5

/* ========================================================================== */
/* Fluid shader (steady-state: T = NONE => MC solves for it)                  */
/* ========================================================================== */
static double
d2_fluid_get_temperature
  (const struct sdis_rwalk_vertex* vtx, struct sdis_data* data)
{
  (void)data;
  CHK(vtx != NULL);
  return SDIS_TEMPERATURE_NONE;
}

static double
d2_fluid_get_volumic_mass
  (const struct sdis_rwalk_vertex* vtx, struct sdis_data* data)
{
  (void)data;
  CHK(vtx != NULL);
  return D2_RHO;
}

static double
d2_fluid_get_calorific_capacity
  (const struct sdis_rwalk_vertex* vtx, struct sdis_data* data)
{
  (void)data;
  CHK(vtx != NULL);
  return D2_CP;
}

/* ========================================================================== */
/* Interface shaders (varying H per face)                                     */
/* ========================================================================== */
struct d2_interf {
  double temperature;
  double hc;
};

static double
d2_interf_get_temperature
  (const struct sdis_interface_fragment* frag, struct sdis_data* data)
{
  const struct d2_interf* p = sdis_data_cget(data);
  CHK(frag && data);
  return p->temperature;
}

static double
d2_interf_get_convection_coef
  (const struct sdis_interface_fragment* frag, struct sdis_data* data)
{
  const struct d2_interf* p = sdis_data_cget(data);
  CHK(frag && data);
  return p->hc;
}

static double
d2_interf_get_emissivity
  (const struct sdis_interface_fragment* frag,
   const unsigned source_id,
   struct sdis_data* data)
{
  (void)source_id;
  CHK(frag && data);
  return 0.0;
}

static double
d2_interf_get_specular_fraction
  (const struct sdis_interface_fragment* frag,
   const unsigned source_id,
   struct sdis_data* data)
{
  (void)source_id;
  CHK(frag && data);
  return 0.0;
}

/* ========================================================================== */
/* Interface factory                                                          */
/* ========================================================================== */
static struct sdis_interface*
d2_create_interface
  (struct sdis_device* dev,
   struct sdis_medium* fluid_medium,
   struct sdis_medium* solid_wall,
   double temperature,
   double hc)
{
  struct sdis_interface_shader shader = SDIS_INTERFACE_SHADER_NULL;
  struct sdis_data* data = NULL;
  struct sdis_interface* interf = NULL;
  struct d2_interf* props = NULL;

  shader.convection_coef = d2_interf_get_convection_coef;
  shader.convection_coef_upper_bound = hc;
  shader.front.temperature = d2_interf_get_temperature;
  shader.front.emissivity = d2_interf_get_emissivity;
  shader.front.specular_fraction = d2_interf_get_specular_fraction;

  OK(sdis_data_create(dev, sizeof(struct d2_interf),
    ALIGNOF(struct d2_interf), NULL, &data));
  props = sdis_data_get(data);
  props->temperature = temperature;
  props->hc = hc;
  OK(sdis_interface_create(dev, fluid_medium, solid_wall, &shader, data,
    &interf));
  OK(sdis_data_ref_put(data));
  return interf;
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
  struct sdis_interface* iface[6];
  struct sdis_scene* scn = NULL;
  struct sdis_scene_create_args scn_args = SDIS_SCENE_CREATE_ARGS_DEFAULT;
  struct sdis_fluid_shader fluid_shader = DUMMY_FLUID_SHADER;
  struct sdis_solid_shader solid_shader = DUMMY_SOLID_SHADER;
  struct sdis_interface* box_interfaces[12];
  int n_pass_primary = 0, n_pass_diag = 0;
  size_t i;
  (void)argc; (void)argv;

  /* Probe positions (all inside the unit cube) */
  static const double probe_pos[D2_NPROBES][3] = {
    {0.25, 0.25, 0.25},
    {0.75, 0.25, 0.25},
    {0.50, 0.50, 0.50},
    {0.25, 0.75, 0.75},
    {0.75, 0.75, 0.75}
  };

  printf("=== WF-D2: Non-uniform convection, steady-state (wavefront probe) ===\n");
  printf("  Analytic T_inf = %.4f K\n", D2_TINF);

  OK(sdis_device_create(&SDIS_DEVICE_CREATE_ARGS_DEFAULT, &dev));

  /* ---- Fluid medium (steady-state: T = NONE) ---- */
  fluid_shader.temperature = d2_fluid_get_temperature;
  fluid_shader.calorific_capacity = d2_fluid_get_calorific_capacity;
  fluid_shader.volumic_mass = d2_fluid_get_volumic_mass;
  OK(sdis_fluid_create(dev, &fluid_shader, NULL, &fluid));

  /* ---- Solid wall (dummy, for interface backing) ---- */
  OK(sdis_solid_create(dev, &solid_shader, NULL, &solid));

  /* ---- Create 6 interfaces with different (T, H) pairs ---- */
  iface[0] = d2_create_interface(dev, fluid, solid, D2_T0, D2_HC0);
  iface[1] = d2_create_interface(dev, fluid, solid, D2_T1, D2_HC1);
  iface[2] = d2_create_interface(dev, fluid, solid, D2_T2, D2_HC2);
  iface[3] = d2_create_interface(dev, fluid, solid, D2_T3, D2_HC3);
  iface[4] = d2_create_interface(dev, fluid, solid, D2_T4, D2_HC4);
  iface[5] = d2_create_interface(dev, fluid, solid, D2_T5, D2_HC5);

  /* Release media */
  OK(sdis_medium_ref_put(solid));
  OK(sdis_medium_ref_put(fluid));

  /* ---- Triangle-to-interface mapping ---- */
  /* test_sdis_utils.h box faces (same mapping as CPU test):
   *   -Z(0-1), -X(2-3), +Z(4-5), +X(6-7), +Y(8-9), -Y(10-11)
   * CPU test mapping:
   *   Front(-Z)=T5, Left(-X)=T0, Back(+Z)=T4, Right(+X)=T1,
   *   Top(+Y)=T3, Bottom(-Y)=T2 */
  box_interfaces[0]  = box_interfaces[1]  = iface[5]; /* Front -Z: T5=350, H5=50 */
  box_interfaces[2]  = box_interfaces[3]  = iface[0]; /* Left  -X: T0=300, H0=100 */
  box_interfaces[4]  = box_interfaces[5]  = iface[4]; /* Back  +Z: T4=340, H4=3400 */
  box_interfaces[6]  = box_interfaces[7]  = iface[1]; /* Right +X: T1=310, H1=30 */
  box_interfaces[8]  = box_interfaces[9]  = iface[3]; /* Top   +Y: T3=330, H3=7300 */
  box_interfaces[10] = box_interfaces[11] = iface[2]; /* Bot   -Y: T2=320, H2=3020 */

  /* ---- Scene ---- */
  scn_args.get_indices = box_get_indices;
  scn_args.get_interface = box_get_interface;
  scn_args.get_position = box_get_position;
  scn_args.nprimitives = box_ntriangles;
  scn_args.nvertices = box_nvertices;
  scn_args.context = box_interfaces;
  OK(sdis_scene_create(dev, &scn_args, &scn));

  /* Release interfaces */
  for(i = 0; i < 6; i++)
    OK(sdis_interface_ref_put(iface[i]));

  /* ---- Run manual probe sweep ---- */
  fprintf(stdout, "  Running %d probes, %d realisations each ...\n",
    D2_NPROBES, D2_NREALS);

  for(i = 0; i < D2_NPROBES; i++) {
    double T_ref = D2_TINF;
    struct sdis_solve_probe_args args = SDIS_SOLVE_PROBE_ARGS_DEFAULT;
    struct sdis_estimator *est_wf = NULL, *est_df = NULL;

    args.nrealisations = D2_NREALS;
    args.position[0] = probe_pos[i][0];
    args.position[1] = probe_pos[i][1];
    args.position[2] = probe_pos[i][2];
    args.picard_order = 1;
    args.diff_algo = SDIS_DIFFUSION_DELTA_SPHERE;
    /* Steady-state: default time_range = {INF, INF} */

    OK(sdis_solve_wavefront_probe(scn, &args, &est_wf));

    n_pass_primary += p0_compare_analytic(est_wf, T_ref, P0_TOL_SIGMA);

    if(P0_ENABLE_DIAG) {
      OK(sdis_solve_probe(scn, &args, &est_df));
      n_pass_diag += p0_diag_compare(est_wf, est_df, P0_DIAG_SIGMA);
    }

    p0_print_probe_result(probe_pos[i][0], est_wf, est_df, T_ref);

    OK(sdis_estimator_ref_put(est_wf));
    if(est_df)
      OK(sdis_estimator_ref_put(est_df));
  }

  fprintf(stdout, "  Primary:    %d/%d probes pass (%.1f%%)\n",
    n_pass_primary, D2_NPROBES,
    100.0 * (double)n_pass_primary / (double)D2_NPROBES);

  if(P0_ENABLE_DIAG)
    fprintf(stdout, "  Diagnostic: %d/%d probes consistent (%.1f%%)\n",
      n_pass_diag, D2_NPROBES,
      100.0 * (double)n_pass_diag / (double)D2_NPROBES);

  CHK((double)n_pass_primary / (double)D2_NPROBES >= P0_PASS_RATE);
  CHK((double)n_pass_diag >= 0);

  printf("WF-D2: PASS\n");

  /* ---- Cleanup ---- */
  OK(sdis_scene_ref_put(scn));
  OK(sdis_device_ref_put(dev));

  CHK(mem_allocated_size() == 0);
  return 0;
}
