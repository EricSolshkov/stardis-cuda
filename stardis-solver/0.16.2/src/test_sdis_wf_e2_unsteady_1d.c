/* WF-E2: 1D transient conduction in a slab (wavefront probe, 3D geometry).
 *
 * Scene: solid cube (0,0,0)-(1,1,1), FP_TO_METER=0.1
 *   Material: Cp=2000 J/K/kg, lambda=0.5 W/m/K, rho=2500 kg/m^3, delta=1/80
 *   Initial temperature: 280 K (t<=0)
 *   -X face: T0=310K,  +X face: T1=320K,  other 4 faces: adiabatic
 *
 * This simulates a 1D slab problem: the temperature varies only along X.
 * Probe at (0.5, 0.5, 0.5) [fp units], 9 time points.
 *
 * Analytic reference: Green function series solution for 1D heat equation.
 *   (values identical to CPU test_sdis_unsteady_1d.c at position x=0.5)
 *   Steady-state limit: (T0+T1)/2 = 315 K
 *
 * Verification: wavefront result vs Green function analytic values.
 *   Each probe: |T_wf - T_ref| <= 3 * SE
 *   Pass criterion: >= 95% of probes pass.
 *
 * Reference CPU test: test_sdis_unsteady_1d.c (2D version)
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
#define E2_T_INIT   280.0   /* [K] initial temperature */
#define E2_T0       310.0   /* [K] -X face (= square left) */
#define E2_T1       320.0   /* [K] +X face (= square right) */

#define E2_CP       2000.0  /* [J/K/kg] */
#define E2_LAMBDA   0.5     /* [W/m/K] */
#define E2_RHO      2500.0  /* [kg/m^3] */
#define E2_DELTA    (1.0 / 80.0)

#define E2_FP_TO_METER  0.1

#define E2_NREALS   10000
#define E2_TOL_SIGMA 3.0
#define E2_PASS_RATE 0.95

/* ========================================================================== */
/* Reference data: Green function analytic (1D heat eq, x=0.5)                */
/* ========================================================================== */
struct e2_reference {
  double time;   /* [s] */
  double temp;   /* [K] */
};

static const struct e2_reference e2_refs[] = {
  { 1000.0000, 280.02848664122115},
  { 2000.0000, 280.86935314560424},
  { 3000.0000, 282.88587826961236},
  { 4000.0000, 285.39698306113996},
  { 5000.0000, 287.96909375994932},
  {10000.000,  298.39293888670881},
  {20000.000,  308.80965010883347},
  {30000.000,  312.69280796373141},
  {1000000.0,  315.00000000000000}
};
static const size_t e2_nrefs = sizeof(e2_refs) / sizeof(e2_refs[0]);

/* Probe at center of cube, x=0.5 */
static const double e2_probe_pos[3] = {0.5, 0.5, 0.5};

/* ========================================================================== */
/* Solid medium shaders                                                       */
/* ========================================================================== */
static double
e2_solid_get_calorific_capacity
  (const struct sdis_rwalk_vertex* vtx, struct sdis_data* data)
{
  (void)data; CHK(vtx != NULL);
  return E2_CP;
}

static double
e2_solid_get_thermal_conductivity
  (const struct sdis_rwalk_vertex* vtx, struct sdis_data* data)
{
  (void)data; CHK(vtx != NULL);
  return E2_LAMBDA;
}

static double
e2_solid_get_volumic_mass
  (const struct sdis_rwalk_vertex* vtx, struct sdis_data* data)
{
  (void)data; CHK(vtx != NULL);
  return E2_RHO;
}

static double
e2_solid_get_delta
  (const struct sdis_rwalk_vertex* vtx, struct sdis_data* data)
{
  (void)data; CHK(vtx != NULL);
  return E2_DELTA;
}

static double
e2_solid_get_temperature
  (const struct sdis_rwalk_vertex* vtx, struct sdis_data* data)
{
  (void)data; CHK(vtx != NULL);
  if(vtx->time <= 0) return E2_T_INIT;
  return SDIS_TEMPERATURE_NONE;
}

/* ========================================================================== */
/* Interface shader: -X face => T0, +X face => T1, others => adiabatic       */
/* ========================================================================== */
static double
e2_interface_get_temperature
  (const struct sdis_interface_fragment* frag, struct sdis_data* data)
{
  (void)data; CHK(frag != NULL);
  /* box_indices in test_sdis_utils.h:
   *   -Z(0-1), -X(2-3), +Z(4-5), +X(6-7), +Y(8-9), -Y(10-11)
   * Normal of -X face: Ng = (-1,0,0) => Ng[0]=-1? No, it depends on
   * winding. Let's use normal direction:
   *   -X face normal points toward -X => Ng[0] = -1
   *   +X face normal points toward +X => Ng[0] = +1
   * But the CPU test maps:
   *   left boundary (Ng[0]== 1 in CPU test, which means outward normal from
   *   the box pointing into the 'solid') => T0=310
   *   right boundary (Ng[0]==-1) => T1=320
   * Wait, in the 2D CPU test, the square_indices define segments:
   *   0: Bottom (1->0), 1: Left (0->2), 2: Top (2->3), 3: Right (3->1)
   *   Left face has outward normal pointing toward -X (Ng[0]=1 means
   *   outward pointing into the domain? No.)
   * The CPU test uses:
   *   if(frag->Ng[0] ==  1) return T0;
   *   if(frag->Ng[0] == -1) return T1;
   * So Ng[0]==+1 => T0=310 (left), Ng[0]==-1 => T1=320 (right).
   *
   * For 3D box, we need to match the same physics: T0=310 on one X-face,
   * T1=320 on the other X-face, adiabatic elsewhere.
   * In box_indices, -X face (tri 2-3) has Ng pointing toward -X.
   * +X face (tri 6-7) has Ng pointing toward +X.
   * But the outward normal of -X face from the box interior is (-1,0,0).
   * With box vertices and winding: triangle 2 = {0,4,2}, normal points
   * in the -X direction, so frag->Ng[0] < 0. Hmm, the actual Ng depends
   * on the geometric normal. For box triangles:
   *   -X face tri {0,4,2}: cross((4-0),(2-0)) = cross((0,0,1),(0,1,0))
   *     = (-1,0,0). So Ng[0]=-1 for -X face.
   *   +X face tri {3,7,5}: cross((7-3),(5-3)) = cross((0,0,1),(0,-1,0))
   *     = (1,0,0). So Ng[0]=+1 for +X face.
   * So in our 3D box: Ng[0]==+1 => +X face, Ng[0]==-1 => -X face.
   *
   * To match physics of CPU 2D test where x=0 side has T0=310:
   *   -X face (leftmost, x=0): T0=310 => Ng[0]==-1 maps to T0.
   *   Actually wait: in the 2D CPU test convention:
   *     Ng[0]==1 => T0=310 (this is "left" in the square geometry)
   *     Ng[0]==-1 => T1=320 (this is "right")
   *   In the 2D square, left segment has Ng pointing outward, which
   *   actually is Ng[0]=-1... Let me just keep it physically consistent.
   *
   * The analytic result at x=0.5 with T(x=0)=310, T(x=1)=320 is:
   *   steady-state = (310+320)/2 = 315K. This matches the reference.
   *
   * For 3D: -X face is x=0, +X face is x=1.
   *   We need x=0 face => 310K, x=1 face => 320K.
   *   -X face Ng: Ng[0]<0 (pointing outward from box at x=0).
   *   +X face Ng: Ng[0]>0 (pointing outward from box at x=1).
   *
   * But Stardis interface normals point from front medium to back medium.
   * For solid-fluid interface, front=solid, so Ng typically points inward.
   * The actual Ng sign matters for the shader. We just map both normals:
   */
  if(fabs(frag->Ng[0]) > 0.5) {
    /* X-facing face */
    if(frag->Ng[0] > 0) return E2_T1;  /* +X (x=1) => T1=320 */
    else                 return E2_T0;  /* -X (x=0) => T0=310 */
  }
  /* Other faces: adiabatic */
  return SDIS_TEMPERATURE_NONE;
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
  struct sdis_scene_create_args scn_args = SDIS_SCENE_CREATE_ARGS_DEFAULT;
  struct sdis_solid_shader solid_shader = SDIS_SOLID_SHADER_NULL;
  struct sdis_fluid_shader fluid_shader = SDIS_FLUID_SHADER_NULL;
  struct sdis_interface_shader interf_shader = SDIS_INTERFACE_SHADER_NULL;
  struct sdis_interface* box_interfaces[12];
  int n_pass = 0;
  size_t i;
  (void)argc; (void)argv;

  printf("=== WF-E2: 1D transient conduction (wavefront probe, 3D geometry) ===\n");

  OK(sdis_device_create(&SDIS_DEVICE_CREATE_ARGS_DEFAULT, &dev));

  /* ---- Solid medium ---- */
  solid_shader.calorific_capacity = e2_solid_get_calorific_capacity;
  solid_shader.thermal_conductivity = e2_solid_get_thermal_conductivity;
  solid_shader.volumic_mass = e2_solid_get_volumic_mass;
  solid_shader.delta = e2_solid_get_delta;
  solid_shader.temperature = e2_solid_get_temperature;
  OK(sdis_solid_create(dev, &solid_shader, NULL, &solid));

  /* ---- Dummy exterior fluid ---- */
  fluid_shader.calorific_capacity = dummy_medium_getter;
  fluid_shader.volumic_mass = dummy_medium_getter;
  fluid_shader.temperature = dummy_medium_getter;
  OK(sdis_fluid_create(dev, &fluid_shader, NULL, &dummy));

  /* ---- Interface: face temperature or adiabatic ---- */
  interf_shader.front.temperature = e2_interface_get_temperature;
  interf_shader.back.temperature  = e2_interface_get_temperature;
  OK(sdis_interface_create(dev, solid, dummy, &interf_shader, NULL, &interf));

  /* Release media */
  OK(sdis_medium_ref_put(solid));
  OK(sdis_medium_ref_put(dummy));

  /* ---- All 12 box triangles share the same interface ---- */
  for(i = 0; i < 12; i++)
    box_interfaces[i] = interf;

  /* ---- Scene ---- */
  scn_args.get_indices = box_get_indices;
  scn_args.get_interface = box_get_interface;
  scn_args.get_position = box_get_position;
  scn_args.nprimitives = box_ntriangles;
  scn_args.nvertices = box_nvertices;
  scn_args.context = box_interfaces;
  scn_args.fp_to_meter = E2_FP_TO_METER;
  OK(sdis_scene_create(dev, &scn_args, &scn));

  OK(sdis_interface_ref_put(interf));

  /* ---- Run probes at 9 time points ---- */
  fprintf(stdout, "  Running %lu time-point probes, %d realisations each ...\n",
    (unsigned long)e2_nrefs, E2_NREALS);

  for(i = 0; i < e2_nrefs; i++) {
    const struct e2_reference* ref = &e2_refs[i];
    struct sdis_solve_probe_args args = SDIS_SOLVE_PROBE_ARGS_DEFAULT;
    struct sdis_estimator* est_wf = NULL;
    struct sdis_mc mc_wf;
    int pass;

    args.nrealisations = E2_NREALS;
    args.position[0] = e2_probe_pos[0];
    args.position[1] = e2_probe_pos[1];
    args.position[2] = e2_probe_pos[2];
    args.time_range[0] = ref->time;
    args.time_range[1] = ref->time;
    args.diff_algo = SDIS_DIFFUSION_DELTA_SPHERE;

    OK(sdis_solve_wavefront_probe(scn, &args, &est_wf));
    OK(sdis_estimator_get_temperature(est_wf, &mc_wf));

    /* Primary: wavefront vs Green function analytic (3 sigma) */
    pass = p0_compare_analytic(est_wf, ref->temp, E2_TOL_SIGMA);
    n_pass += pass;

    fprintf(stdout,
      "  t=%10.1f s  wf=%.6f (SE=%.2e)  ref=%.6f  %s (%.1f sigma)\n",
      ref->time, mc_wf.E, mc_wf.SE, ref->temp,
      pass ? "PASS" : "FAIL",
      mc_wf.SE > 0 ? fabs(mc_wf.E - ref->temp) / mc_wf.SE : 0.0);

    OK(sdis_estimator_ref_put(est_wf));
  }

  fprintf(stdout, "  Primary:    %d/%lu probes pass (%.1f%%)\n",
    n_pass, (unsigned long)e2_nrefs,
    100.0 * (double)n_pass / (double)e2_nrefs);

  /* Pass criterion: >= 95% */
  CHK((double)n_pass / (double)e2_nrefs >= E2_PASS_RATE);

  printf("WF-E2: PASS\n");

  /* ---- Cleanup ---- */
  OK(sdis_scene_ref_put(scn));
  OK(sdis_device_ref_put(dev));

  CHK(mem_allocated_size() == 0);
  return 0;
}
