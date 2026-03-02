/* WF-D1: Uniform convection, 6-face cooling, steady-state (wavefront probe).
 *
 * Scene: unit cube fluid, rho=25, cp=2
 *   6 faces with different temperatures, all H=10 W/(m^2*K)
 *   T0=300 (-X), T1=310 (+X), T2=320 (-Y), T3=330 (+Y),
 *   T4=340 (+Z), T5=350 (-Z)
 *
 * Analytic (steady-state, lumped parameter):
 *   Tinf = (T0+T1+T2+T3+T4+T5)/6 = 325 K
 *   (position-independent for lumped model)
 *
 * 5 probes at various interior positions, dual validation A+B.
 *
 * Reference CPU test: test_sdis_convection.c
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
#define D1_T0   300.0   /* -X face */
#define D1_T1   310.0   /* +X face */
#define D1_T2   320.0   /* -Y face */
#define D1_T3   330.0   /* +Y face */
#define D1_T4   340.0   /* +Z face */
#define D1_T5   350.0   /* -Z face */

#define D1_H     10.0   /* convection coef on all faces */
#define D1_RHO   25.0
#define D1_CP     2.0

#define D1_TINF  ((D1_T0 + D1_T1 + D1_T2 + D1_T3 + D1_T4 + D1_T5) / 6.0)

#define D1_NREALS  100000  /* convection needs higher count */
#define D1_NPROBES 5

/* ========================================================================== */
/* Fluid shader (steady-state: T = NONE => MC solves for it)                  */
/* ========================================================================== */
static double
d1_fluid_get_temperature
  (const struct sdis_rwalk_vertex* vtx, struct sdis_data* data)
{
  (void)data;
  CHK(vtx != NULL);
  return SDIS_TEMPERATURE_NONE;
}

static double
d1_fluid_get_volumic_mass
  (const struct sdis_rwalk_vertex* vtx, struct sdis_data* data)
{
  (void)data;
  CHK(vtx != NULL);
  return D1_RHO;
}

static double
d1_fluid_get_calorific_capacity
  (const struct sdis_rwalk_vertex* vtx, struct sdis_data* data)
{
  (void)data;
  CHK(vtx != NULL);
  return D1_CP;
}

/* ========================================================================== */
/* Interface shaders                                                          */
/* ========================================================================== */
struct d1_interf {
  double temperature;
};

static double
d1_interf_get_temperature
  (const struct sdis_interface_fragment* frag, struct sdis_data* data)
{
  const struct d1_interf* p = sdis_data_cget(data);
  CHK(frag && data);
  return p->temperature;
}

static double
d1_interf_get_convection_coef
  (const struct sdis_interface_fragment* frag, struct sdis_data* data)
{
  CHK(frag && data);
  return D1_H;
}

static double
d1_interf_get_emissivity
  (const struct sdis_interface_fragment* frag,
   const unsigned source_id,
   struct sdis_data* data)
{
  (void)source_id;
  CHK(frag && data);
  return 0.0;
}

static double
d1_interf_get_specular_fraction
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
d1_create_interface
  (struct sdis_device* dev,
   struct sdis_medium* fluid_medium,
   struct sdis_medium* solid_wall,
   const struct sdis_interface_shader* shader,
   double temperature)
{
  struct sdis_data* data = NULL;
  struct sdis_interface* interf = NULL;
  struct d1_interf* props = NULL;

  OK(sdis_data_create(dev, sizeof(struct d1_interf),
    ALIGNOF(struct d1_interf), NULL, &data));
  props = sdis_data_get(data);
  props->temperature = temperature;
  OK(sdis_interface_create(dev, fluid_medium, solid_wall, shader, data,
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
  struct sdis_interface* iface_t0 = NULL;
  struct sdis_interface* iface_t1 = NULL;
  struct sdis_interface* iface_t2 = NULL;
  struct sdis_interface* iface_t3 = NULL;
  struct sdis_interface* iface_t4 = NULL;
  struct sdis_interface* iface_t5 = NULL;
  struct sdis_scene* scn = NULL;
  struct sdis_scene_create_args scn_args = SDIS_SCENE_CREATE_ARGS_DEFAULT;
  struct sdis_fluid_shader fluid_shader = DUMMY_FLUID_SHADER;
  struct sdis_solid_shader solid_shader = DUMMY_SOLID_SHADER;
  struct sdis_interface_shader interf_shader = DUMMY_INTERFACE_SHADER;
  struct sdis_interface* box_interfaces[12];
  int n_pass_primary = 0, n_pass_diag = 0;
  size_t i;
  (void)argc; (void)argv;

  /* Probe positions (all inside the unit cube) */
  static const double probe_pos[D1_NPROBES][3] = {
    {0.25, 0.25, 0.25},
    {0.75, 0.25, 0.25},
    {0.50, 0.50, 0.50},
    {0.25, 0.75, 0.75},
    {0.75, 0.75, 0.75}
  };

  printf("=== WF-D1: Convection 6-face cooling, steady-state (wavefront probe) ===\n");

  OK(sdis_device_create(&SDIS_DEVICE_CREATE_ARGS_DEFAULT, &dev));

  /* ---- Fluid medium (steady-state: T = NONE) ---- */
  fluid_shader.temperature = d1_fluid_get_temperature;
  fluid_shader.calorific_capacity = d1_fluid_get_calorific_capacity;
  fluid_shader.volumic_mass = d1_fluid_get_volumic_mass;
  OK(sdis_fluid_create(dev, &fluid_shader, NULL, &fluid));

  /* ---- Solid wall (dummy, for interface backing) ---- */
  OK(sdis_solid_create(dev, &solid_shader, NULL, &solid));

  /* ---- Interface shader (convection) ---- */
  interf_shader.convection_coef = d1_interf_get_convection_coef;
  interf_shader.convection_coef_upper_bound = D1_H;
  interf_shader.front.temperature = d1_interf_get_temperature;
  interf_shader.front.emissivity = d1_interf_get_emissivity;
  interf_shader.front.specular_fraction = d1_interf_get_specular_fraction;

  /* ---- Create 6 interfaces with different temperatures ---- */
  iface_t0 = d1_create_interface(dev, fluid, solid, &interf_shader, D1_T0);
  iface_t1 = d1_create_interface(dev, fluid, solid, &interf_shader, D1_T1);
  iface_t2 = d1_create_interface(dev, fluid, solid, &interf_shader, D1_T2);
  iface_t3 = d1_create_interface(dev, fluid, solid, &interf_shader, D1_T3);
  iface_t4 = d1_create_interface(dev, fluid, solid, &interf_shader, D1_T4);
  iface_t5 = d1_create_interface(dev, fluid, solid, &interf_shader, D1_T5);

  /* Release media */
  OK(sdis_medium_ref_put(solid));
  OK(sdis_medium_ref_put(fluid));

  /* ---- Triangle-to-interface mapping ---- */
  /* test_sdis_utils.h box faces:
   *   -Z(0-1), -X(2-3), +Z(4-5), +X(6-7), +Y(8-9), -Y(10-11)
   * CPU test_sdis_convection.c mapping:
   *   Front(-Z)=T5, Left(-X)=T0, Back(+Z)=T4, Right(+X)=T1,
   *   Top(+Y)=T3, Bottom(-Y)=T2 */
  box_interfaces[0] = box_interfaces[1] = iface_t5;   /* Front -Z: T5=350 */
  box_interfaces[2] = box_interfaces[3] = iface_t0;   /* Left -X:  T0=300 */
  box_interfaces[4] = box_interfaces[5] = iface_t4;   /* Back +Z:  T4=340 */
  box_interfaces[6] = box_interfaces[7] = iface_t1;   /* Right +X: T1=310 */
  box_interfaces[8] = box_interfaces[9] = iface_t3;   /* Top +Y:   T3=330 */
  box_interfaces[10] = box_interfaces[11] = iface_t2;  /* Bottom -Y: T2=320 */

  /* ---- Scene ---- */
  scn_args.get_indices = box_get_indices;
  scn_args.get_interface = box_get_interface;
  scn_args.get_position = box_get_position;
  scn_args.nprimitives = box_ntriangles;
  scn_args.nvertices = box_nvertices;
  scn_args.context = box_interfaces;
  OK(sdis_scene_create(dev, &scn_args, &scn));

  /* Release interfaces */
  OK(sdis_interface_ref_put(iface_t0));
  OK(sdis_interface_ref_put(iface_t1));
  OK(sdis_interface_ref_put(iface_t2));
  OK(sdis_interface_ref_put(iface_t3));
  OK(sdis_interface_ref_put(iface_t4));
  OK(sdis_interface_ref_put(iface_t5));

  /* ---- Run manual probe sweep (not axis-aligned) — batch ---- */
  fprintf(stdout, "  Running %d probes (batch), %d realisations each ...\n",
    D1_NPROBES, D1_NREALS);
  fprintf(stdout, "  T_inf (analytic) = %.1f K\n", D1_TINF);

  {
    struct sdis_solve_probe_args args_arr[D1_NPROBES];
    struct sdis_estimator*       ests[D1_NPROBES];

    for(i = 0; i < D1_NPROBES; i++) {
      args_arr[i] = SDIS_SOLVE_PROBE_ARGS_DEFAULT;
      args_arr[i].nrealisations = D1_NREALS;
      args_arr[i].position[0]   = probe_pos[i][0];
      args_arr[i].position[1]   = probe_pos[i][1];
      args_arr[i].position[2]   = probe_pos[i][2];
      args_arr[i].picard_order  = 1;
      args_arr[i].diff_algo     = SDIS_DIFFUSION_DELTA_SPHERE;
      ests[i] = NULL;
    }

    OK(sdis_solve_persistent_wavefront_probe_batch(
      scn, D1_NPROBES, args_arr, ests));

    for(i = 0; i < D1_NPROBES; i++) {
      double T_ref = D1_TINF;
      struct sdis_estimator *est_df = NULL;

      n_pass_primary += p0_compare_analytic(ests[i], T_ref, P0_TOL_SIGMA);

      if(P0_ENABLE_DIAG) {
        OK(sdis_solve_probe(scn, &args_arr[i], &est_df));
        n_pass_diag += p0_diag_compare(ests[i], est_df, P0_DIAG_SIGMA);
      }

      p0_print_probe_result(probe_pos[i][0], ests[i], est_df, T_ref);

      OK(sdis_estimator_ref_put(ests[i]));
      if(est_df)
        OK(sdis_estimator_ref_put(est_df));
    }
  }

  fprintf(stdout, "  Primary:    %d/%d probes pass (%.1f%%)\n",
    n_pass_primary, D1_NPROBES,
    100.0 * (double)n_pass_primary / (double)D1_NPROBES);

  if(P0_ENABLE_DIAG)
    fprintf(stdout, "  Diagnostic: %d/%d probes consistent (%.1f%%)\n",
      n_pass_diag, D1_NPROBES,
      100.0 * (double)n_pass_diag / (double)D1_NPROBES);

  /* Only Primary decides PASS/FAIL */
  CHK((double)n_pass_primary / (double)D1_NPROBES >= P0_PASS_RATE);
  CHK((double)n_pass_diag >= 0);  /* suppress unused warning */

  printf("WF-D1: PASS\n");

  /* ---- Cleanup ---- */
  OK(sdis_scene_ref_put(scn));
  OK(sdis_device_ref_put(dev));

  CHK(mem_allocated_size() == 0);
  return 0;
}
