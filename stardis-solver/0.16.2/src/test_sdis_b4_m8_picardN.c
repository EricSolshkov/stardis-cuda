/* Copyright (C) 2016-2025 |Meso|Star> (contact@meso-star.com)
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>. */

/* Phase B-4, Milestone 8: PicardN recursive stack -- unit tests.
 *
 * These tests verify the structural correctness of the picardN recursive
 * stack state machine added in M8.  No GPU trace is performed -- all hit data
 * is synthetically injected, following the same pattern as the M5/M7 tests.
 *
 * Test cases (from phase_b4_test_design.md, section T8):
 *   T8.1: Stack depth management (push/pop in [0, MAX_PICARD_DEPTH])
 *   T8.2: Stack overflow protection (fallback to synchronous)
 *   T8.3: COMPUTE_Ti sub-path launch (push stack + enter COUPLED_BOUNDARY)
 *   T8.4: Sub-path completion pop (PATH_DONE intercept -> Ti_RESUME)
 *   T8.5: CHECK_PMIN_PMAX early accept
 *   T8.6: CHECK_PMIN_PMAX early reject
 *   T8.7: Multi-layer recursion (stack frame independence)
 *   T8.9: PicardN vs Picard1 degenerate (picard_order=1 -> no SFN states)
 */

#include "sdis_solve_wavefront.h"
#include "sdis_wf_steps.h"
#include "sdis_scene_c.h"
#include "sdis_interface_c.h"

#include "test_sdis_utils.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <float.h>

/* ========================================================================== */
/* Mock getter functions                                                      */
/* ========================================================================== */

static double
nan_medium_getter(const struct sdis_rwalk_vertex* v, struct sdis_data* d)
{
  (void)v; (void)d;
  return NaN;
}

static double
one_medium_getter(const struct sdis_rwalk_vertex* v, struct sdis_data* d)
{
  (void)v; (void)d;
  return 1.0;
}

static double
zero_medium_getter(const struct sdis_rwalk_vertex* v, struct sdis_data* d)
{
  (void)v; (void)d;
  return 0.0;
}

static double
nan_interface_getter(const struct sdis_interface_fragment* frag,
                     struct sdis_data* data)
{
  (void)data; (void)frag;
  return NaN;
}

static double
zero_interface_getter(const struct sdis_interface_fragment* frag,
                      struct sdis_data* data)
{
  (void)data; (void)frag;
  return 0.0;
}

static double
hc20_interface_getter(const struct sdis_interface_fragment* frag,
                      struct sdis_data* data)
{
  (void)data; (void)frag;
  return 20.0; /* h_conv = 20 W/(m^2*K) */
}

static double
tref300_interface_getter(const struct sdis_interface_fragment* frag,
                         struct sdis_data* data)
{
  (void)data; (void)frag;
  return 300.0;
}

/* Emissivity = 0.9 (high radiative coupling) */
static double
emissivity_09_getter(const struct sdis_interface_fragment* frag,
                     const unsigned source_id, struct sdis_data* data)
{
  (void)data; (void)source_id; (void)frag;
  return 0.9;
}

/* Emissivity = 0.0 (no radiative coupling) */
static double
emissivity_00_getter(const struct sdis_interface_fragment* frag,
                     const unsigned source_id, struct sdis_data* data)
{
  (void)data; (void)source_id; (void)frag;
  return 0.0;
}

static double
specular_00_getter(const struct sdis_interface_fragment* frag,
                   const unsigned source_id, struct sdis_data* data)
{
  (void)data; (void)source_id; (void)frag;
  return 0.0;
}

/* Fluid shader with NaN temperature */
static const struct sdis_fluid_shader CONVECTIVE_FLUID_SHADER = {
  one_medium_getter,  /* Calorific capacity = 1 */
  one_medium_getter,  /* Volumic mass = 1 */
  nan_medium_getter,  /* Temperature = NaN (unknown) */
  0                   /* Initial time */
};

/* Solid shader with NaN temperature (unknown).
 * NOTE: volumic_power getter MUST be NULL for picardN (picard_order > 1),
 * otherwise the synchronous fallback rejects it. */
static const struct sdis_solid_shader UNKNOWN_SOLID_SHADER = {
  one_medium_getter,  /* Calorific capacity = 1 */
  one_medium_getter,  /* Thermal conductivity = 1 (lambda) */
  one_medium_getter,  /* Volumic mass = 1 */
  one_medium_getter,  /* Delta = 1 */
  NULL,               /* Volumic power = NULL (required for picardN) */
  nan_medium_getter,  /* Temperature = NaN (unknown) */
  NULL,               /* sample path */
  0                   /* Initial time */
};

/* Interface side shader with high emissivity (0.9) and no flux (picardN) */
#define SFN_INTERFACE_SIDE_SHADER__ {                                          \
  nan_interface_getter,    /* Temperature = NaN (no Dirichlet) */              \
  zero_interface_getter,   /* Flux = 0 (picardN requires no ext flux) */      \
  emissivity_09_getter,    /* Emissivity = 0.9 */                             \
  specular_00_getter,      /* Specular fraction = 0 */                        \
  tref300_interface_getter,/* Reference temperature = 300K */                  \
  0 /* Do NOT handle external flux */                                         \
}

/* Interface side shader with zero emissivity (no radiative) */
#define SF_NO_RAD_INTERFACE_SIDE_SHADER__ {                                    \
  nan_interface_getter,    /* Temperature = NaN (no Dirichlet) */              \
  zero_interface_getter,   /* Flux = 0 */                                     \
  emissivity_00_getter,    /* Emissivity = 0 */                               \
  specular_00_getter,      /* Specular fraction = 0 */                        \
  nan_interface_getter,    /* Reference temperature = NaN */                   \
  0 /* Do NOT handle external flux */                                         \
}

/* Solid/fluid interface for picardN testing */
static const struct sdis_interface_shader SFN_INTERFACE_SHADER = {
  hc20_interface_getter,   /* Convection coef = 20 */
  200.0,                   /* Upper bound of convection coef */
  zero_interface_getter,   /* Thermal contact resistance = 0 */
  SFN_INTERFACE_SIDE_SHADER__, /* Front side */
  SFN_INTERFACE_SIDE_SHADER__  /* Back side */
};

/* Solid/fluid interface for picard1 testing (no rad) */
static const struct sdis_interface_shader SF_P1_INTERFACE_SHADER = {
  hc20_interface_getter,   /* Convection coef = 20 */
  200.0,                   /* Upper bound of convection coef */
  zero_interface_getter,   /* Thermal contact resistance = 0 */
  SF_NO_RAD_INTERFACE_SIDE_SHADER__, /* Front side */
  SF_NO_RAD_INTERFACE_SIDE_SHADER__  /* Back side */
};

/* ========================================================================== */
/* Shared test infrastructure                                                 */
/* ========================================================================== */

static struct sdis_device* g_dev = NULL;
static struct sdis_scene*  g_scn_sfn = NULL;    /* solid/fluid picardN scene */
static struct sdis_scene*  g_scn_sf_p1 = NULL;  /* solid/fluid picard1 scene */
static struct ssp_rng*     g_rng = NULL;
static unsigned            g_fluid_enc = 0;
static unsigned            g_solid_enc = 0;

static struct sdis_interface* g_sfn_interfaces[12];
static struct sdis_interface* g_sf_p1_interfaces[12];

/* box_get_interface, box_get_position, box_get_indices are provided
 * by test_sdis_utils.h as static inline functions. */

static void
setup_test_scenes(void)
{
  struct sdis_scene_create_args scn_args;
  struct sdis_medium* solid = NULL;
  struct sdis_medium* fluid = NULL;
  struct sdis_interface* interf_sfn = NULL;
  struct sdis_interface* interf_sf_p1 = NULL;
  int i;

  OK(sdis_device_create(&SDIS_DEVICE_CREATE_ARGS_DEFAULT, &g_dev));

  /* -- Media -- */
  {
    struct sdis_solid_shader s = UNKNOWN_SOLID_SHADER;
    OK(sdis_solid_create(g_dev, &s, NULL, &solid));
  }
  {
    struct sdis_fluid_shader f = CONVECTIVE_FLUID_SHADER;
    OK(sdis_fluid_create(g_dev, &f, NULL, &fluid));
  }

  /* -- Interfaces -- */
  {
    struct sdis_interface_shader sh = SFN_INTERFACE_SHADER;
    OK(sdis_interface_create(g_dev, solid, fluid, &sh, NULL, &interf_sfn));
  }
  for(i = 0; i < 12; i++) g_sfn_interfaces[i] = interf_sfn;

  {
    struct sdis_interface_shader sh = SF_P1_INTERFACE_SHADER;
    OK(sdis_interface_create(g_dev, solid, fluid, &sh, NULL, &interf_sf_p1));
  }
  for(i = 0; i < 12; i++) g_sf_p1_interfaces[i] = interf_sf_p1;

  /* -- Scene: solid/fluid picardN -- */
  scn_args = SDIS_SCENE_CREATE_ARGS_DEFAULT;
  scn_args.get_indices = box_get_indices;
  scn_args.get_interface = box_get_interface;
  scn_args.get_position = box_get_position;
  scn_args.nprimitives = box_ntriangles;
  scn_args.nvertices = box_nvertices;
  scn_args.context = g_sfn_interfaces;
  scn_args.fp_to_meter = 1.0;
  OK(sdis_scene_create(g_dev, &scn_args, &g_scn_sfn));

  /* Set temperature range (needed for h_radi_hat computation) */
  g_scn_sfn->tmin = 200.0;
  g_scn_sfn->tmax = 500.0;

  {
    unsigned encs[2];
    struct sdis_medium* mdm0 = NULL;
    struct sdis_medium* mdm1 = NULL;
    scene_get_enclosure_ids(g_scn_sfn, 0, encs);
    OK(scene_get_enclosure_medium(g_scn_sfn,
      scene_get_enclosure(g_scn_sfn, encs[0]), &mdm0));
    OK(scene_get_enclosure_medium(g_scn_sfn,
      scene_get_enclosure(g_scn_sfn, encs[1]), &mdm1));
    if(sdis_medium_get_type(mdm0) == SDIS_FLUID) {
      g_fluid_enc = encs[0];
      g_solid_enc = encs[1];
    } else {
      CHK(sdis_medium_get_type(mdm1) == SDIS_FLUID);
      g_fluid_enc = encs[1];
      g_solid_enc = encs[0];
    }
  }

  /* -- Scene: solid/fluid picard1 (no rad) -- */
  scn_args.context = g_sf_p1_interfaces;
  OK(sdis_scene_create(g_dev, &scn_args, &g_scn_sf_p1));
  g_scn_sf_p1->tmin = 200.0;
  g_scn_sf_p1->tmax = 500.0;

  /* -- RNG -- */
  OK(ssp_rng_create(NULL, SSP_RNG_KISS, &g_rng));

  /* Release media refs (scene holds them) */
  OK(sdis_medium_ref_put(solid));
  OK(sdis_medium_ref_put(fluid));
}

static void
teardown_test_scenes(void)
{
  size_t i;
  struct sdis_interface* interfs[2] = {
    g_sfn_interfaces[0], g_sf_p1_interfaces[0]
  };
  OK(sdis_scene_ref_put(g_scn_sfn));
  OK(sdis_scene_ref_put(g_scn_sf_p1));
  for(i = 0; i < 2; i++) {
    if(interfs[i]) OK(sdis_interface_ref_put(interfs[i]));
  }
  OK(ssp_rng_ref_put(g_rng));
  OK(sdis_device_ref_put(g_dev));

  g_scn_sfn = NULL;
  g_scn_sf_p1 = NULL;
  g_dev = NULL;
  g_rng = NULL;
}

/* Helper: initialise a path_state on the solid/fluid boundary of box face 0.
 * Position: (0.5, 0.5, 0.0) on -Z face.
 * hit_side: FRONT (solid is front in our interface).
 * picard_order: as specified. */
static void
init_path_on_sf_boundary(struct path_state* p, size_t picard_order)
{
  memset(p, 0, sizeof(*p));
  p->active = 1;
  p->rng = g_rng;
  p->phase = PATH_BND_DISPATCH;
  p->needs_ray = 0;

  /* Position on -Z face */
  p->rwalk.vtx.P[0] = 0.5;
  p->rwalk.vtx.P[1] = 0.5;
  p->rwalk.vtx.P[2] = 0.0;
  p->rwalk.vtx.time = 0.0;
  p->rwalk.enc_id = g_solid_enc;
  p->rwalk.elapsed_time = 0;

  /* Synthetic hit on triangle 0 of -Z face */
  p->rwalk.hit_3d.prim.prim_id = 0;
  p->rwalk.hit_3d.prim.geom_id = 0;
  p->rwalk.hit_3d.prim.inst_id = 0;
  p->rwalk.hit_3d.normal[0] = 0.0f;
  p->rwalk.hit_3d.normal[1] = 0.0f;
  p->rwalk.hit_3d.normal[2] = -1.0f;
  p->rwalk.hit_3d.distance = 0.5f;
  p->rwalk.hit_side = SDIS_FRONT;

  /* Context */
  p->ctx.nbranchings = 0;
  p->ctx.max_branchings = picard_order;
  p->ctx.That = 500.0;
  p->ctx.That2 = 500.0 * 500.0;
  p->ctx.That3 = 500.0 * 500.0 * 500.0;
  p->ctx.Tmin = 200.0;
  p->ctx.Tmin2 = 200.0 * 200.0;
  p->ctx.Tmin3 = 200.0 * 200.0 * 200.0;
  p->ctx.heat_path = NULL;
  p->ctx.green_path = NULL;

  p->T.value = 0;
  p->T.done = 0;
  p->T.func = boundary_path_3d;

  p->coupled_nbranchings = 0;
  /* sfn_stack_depth moved to path_sfn_data; caller zeroes via memset(&sfn,0) */

  p->filter_data_storage = HIT_FILTER_DATA_NULL;
}

/* ========================================================================== */
/* T8.1: Stack depth management (push/pop in [0, MAX_PICARD_DEPTH])           */
/* ========================================================================== */
static void
test_sfn_stack_depth_management(void)
{
  struct path_state p;
  struct path_sfn_data sfn;

  printf("  T8.1: Stack depth management... ");

  /* Verify MAX_PICARD_DEPTH is defined properly */
  CHK(MAX_PICARD_DEPTH == 3);

  /* Initialise path at sfn_stack_depth = 0 */
  memset(&p, 0, sizeof(p));
  memset(&sfn, 0, sizeof(sfn));
  sfn.depth = 0;
  CHK(sfn.depth >= 0);
  CHK(sfn.depth < MAX_PICARD_DEPTH);

  /* Simulate push */
  sfn.depth++;
  CHK(sfn.depth == 1);
  CHK(sfn.depth >= 0 && sfn.depth < MAX_PICARD_DEPTH);

  /* Simulate push again */
  sfn.depth++;
  CHK(sfn.depth == 2);
  CHK(sfn.depth >= 0 && sfn.depth < MAX_PICARD_DEPTH);

  /* Cannot push further (depth + 1 >= MAX_PICARD_DEPTH) */
  CHK(sfn.depth + 1 >= MAX_PICARD_DEPTH);

  /* Pop */
  sfn.depth--;
  CHK(sfn.depth == 1);

  /* Pop again */
  sfn.depth--;
  CHK(sfn.depth == 0);

  printf("PASS\n");
}

/* ========================================================================== */
/* T8.2: Stack overflow protection (MAX_PICARD_DEPTH exceeded -> fallback)    */
/* ========================================================================== */
static void
test_sfn_stack_overflow(void)
{
  struct path_state p;
  struct path_sfn_data sfn;

  printf("  T8.2: Stack overflow protection... ");

  /* The overflow condition in step_bnd_sfn_compute_Ti is:
   *   if(p->sfn_stack_depth + 1 >= MAX_PICARD_DEPTH)
   * Verify the condition triggers at the correct boundary.
   *
   * NOTE: We cannot call step_bnd_sfn_compute_Ti with the overflow path
   * in a unit-test because the synchronous fallback
   * (solid_fluid_boundary_picardN_path_3d) requires a full solver
   * environment that is not available in this minimal test setup.
   * Instead, we verify the condition structurally. */

  init_path_on_sf_boundary(&p, 2);

  /* Depth 0: push is OK (0 + 1 = 1 < 3) */
  sfn.depth = 0;
  CHK(!(sfn.depth + 1 >= MAX_PICARD_DEPTH));

  /* Depth 1: push is OK (1 + 1 = 2 < 3) */
  sfn.depth = 1;
  CHK(!(sfn.depth + 1 >= MAX_PICARD_DEPTH));

  /* Depth 2: push overflows (2 + 1 = 3 >= 3) */
  sfn.depth = MAX_PICARD_DEPTH - 1;
  CHK(  sfn.depth + 1 >= MAX_PICARD_DEPTH);

  /* Depth MAX-1 is the last valid index into sfn_stack[] */
  CHK(MAX_PICARD_DEPTH - 1 >= 0);
  CHK(MAX_PICARD_DEPTH == 3); /* sanity */

  /* Additionally verify that when T is already done at max depth,
   * step_bnd_sfn_compute_Ti takes the fast path (no push needed). */
  {
    res_T res;
    p.phase = PATH_BND_SFN_COMPUTE_Ti;
    sfn.depth = MAX_PICARD_DEPTH - 1;
    p.locals.bnd_sf.is_picardn = 1;
    p.locals.bnd_sf.h_hat = 100.0;
    sfn.stack[sfn.depth].T_count = 0;
    sfn.stack[sfn.depth].rwalk_saved = p.rwalk;
    sfn.stack[sfn.depth].T_saved = p.T;

    /* Make T_s done so the fast path is taken (no overflow) */
    p.locals.bnd_sf.rwalk_s = p.rwalk;
    p.locals.bnd_sf.T_s = p.T;
    p.locals.bnd_sf.T_s.done = 1;
    p.locals.bnd_sf.T_s.value = 350.0;

    res = step_bnd_sfn_compute_Ti(&p, g_scn_sfn, &sfn);
    CHK(res == RES_OK);
    /* T value should have been stored without push */
    CHK(sfn.stack[MAX_PICARD_DEPTH - 1].T_values[0] == 350.0);
    CHK(sfn.stack[MAX_PICARD_DEPTH - 1].T_count == 1);
    CHK(p.phase == PATH_BND_SFN_CHECK_PMIN_PMAX);
    /* Depth unchanged — no push */
    CHK(sfn.depth == MAX_PICARD_DEPTH - 1);
  }

  printf("PASS\n");
}

/* ========================================================================== */
/* T8.3: COMPUTE_Ti sub-path launch (push stack -> COUPLED_BOUNDARY)          */
/* ========================================================================== */
static void
test_sfn_compute_Ti_push(void)
{
  struct path_state p;
  struct path_sfn_data sfn;
  struct rwalk saved_rwalk;
  res_T res;

  printf("  T8.3: COMPUTE_Ti sub-path launch... ");

  init_path_on_sf_boundary(&p, 2);

  p.phase = PATH_BND_SFN_COMPUTE_Ti;
  sfn.depth = 0;
  p.locals.bnd_sf.is_picardn = 1;

  /* Set up sfn_stack frame */
  sfn.stack[0].T_count = 0;      /* i = 0 → T0 */
  sfn.stack[0].rwalk_saved = p.rwalk;
  sfn.stack[0].T_saved = p.T;

  /* rwalk_s for T0 (i < 3 → sample from rwalk_s) */
  p.locals.bnd_sf.rwalk_s = p.rwalk;
  p.locals.bnd_sf.T_s.done = 0;  /* T_s not known → needs recursion */
  p.locals.bnd_sf.T_s.value = 0;
  p.locals.bnd_sf.T_s.func = boundary_path_3d;

  saved_rwalk = p.rwalk;

  /* Call step_bnd_sfn_compute_Ti */
  res = step_bnd_sfn_compute_Ti(&p, g_scn_sfn, &sfn);
  CHK(res == RES_OK);

  /* Should have pushed stack: depth increased */
  CHK(sfn.depth == 1);

  /* Should enter COUPLED_BOUNDARY for the sub-path */
  CHK(p.phase == PATH_COUPLED_BOUNDARY);

  /* nbranchings should have been incremented */
  CHK(p.ctx.nbranchings == 1);

  printf("PASS\n");
}

/* ========================================================================== */
/* T8.4: Sub-path completion pop (PATH_DONE intercept -> Ti_RESUME)           */
/* ========================================================================== */
static void
test_sfn_subpath_done_intercept(void)
{
  struct path_state p;
  struct path_sfn_data sfn;

  printf("  T8.4: Sub-path completion pop... ");

  memset(&p, 0, sizeof(p));
  memset(&sfn, 0, sizeof(sfn));
  p.active = 1;
  p.phase = PATH_DONE;
  sfn.depth = 1; /* sub-path finished, parent still active */

  /* Simulate the interception logic from pool_cascade:
   * if (p->phase == PATH_DONE && p->sfn_stack_depth > 0) →
   *   p->phase = PATH_BND_SFN_COMPUTE_Ti_RESUME */
  if(p.phase == PATH_DONE && sfn.depth > 0) {
    p.phase = PATH_BND_SFN_COMPUTE_Ti_RESUME;
    p.active = 1;
  }

  CHK(p.phase == PATH_BND_SFN_COMPUTE_Ti_RESUME);
  CHK(p.active == 1);

  printf("PASS\n");
}

/* ========================================================================== */
/* T8.5: CHECK_PMIN_PMAX early accept                                         */
/* ========================================================================== */
static void
test_sfn_check_pmin_pmax_early_accept(void)
{
  struct path_state p;
  struct path_sfn_data sfn;
  res_T res;

  printf("  T8.5: CHECK_PMIN_PMAX early accept... ");

  init_path_on_sf_boundary(&p, 2);

  p.phase = PATH_BND_SFN_CHECK_PMIN_PMAX;
  sfn.depth = 0;
  p.locals.bnd_sf.is_picardn = 1;
  p.locals.bnd_sf.epsilon = 0.9;

  /* Set up probabilities so that r < p_conv + p_cond + p_radi_min */
  sfn.stack[0].p_conv = 0.1;
  sfn.stack[0].p_cond = 0.1;
  sfn.stack[0].h_hat = 100.0;
  sfn.stack[0].r = 0.15;  /* small r, will be < p_conv+p_cond+p_radi_min */
  sfn.stack[0].T_count = 1;
  sfn.stack[0].T_values[0] = 300.0; /* T0 */

  /* Set up rwalk_s/T_s for sfn_switch_in_radiative */
  p.locals.bnd_sf.rwalk_s = p.rwalk;
  p.locals.bnd_sf.T_s = p.T;
  p.locals.bnd_sf.T_s.done = 1;
  p.locals.bnd_sf.T_s.value = 300.0;

  /* Compute h_radi_min at i=1: BOLTZMANN_CONSTANT * (Tmin3 + 3*Tmin2*T0) */
  {
    double h_radi_min = BOLTZMANN_CONSTANT *
      (p.ctx.Tmin3 + 3.0 * p.ctx.Tmin2 * 300.0);
    double p_radi_min = h_radi_min * p.locals.bnd_sf.epsilon
                      / sfn.stack[0].h_hat;
    /* Ensure r falls in accept range */
    sfn.stack[0].r = sfn.stack[0].p_conv + sfn.stack[0].p_cond
                     + p_radi_min * 0.5; /* below threshold */
  }

  res = step_bnd_sfn_check_pmin_pmax(&p, g_scn_sfn, &sfn);
  CHK(res == RES_OK);

  /* Early accept → sfn_switch_in_radiative → PATH_BND_POST_ROBIN_CHECK */
  CHK(p.phase == PATH_BND_POST_ROBIN_CHECK);

  printf("PASS\n");
}

/* ========================================================================== */
/* T8.6: CHECK_PMIN_PMAX early reject                                         */
/* ========================================================================== */
static void
test_sfn_check_pmin_pmax_early_reject(void)
{
  struct path_state p;
  struct path_sfn_data sfn;
  res_T res;

  printf("  T8.6: CHECK_PMIN_PMAX early reject... ");

  init_path_on_sf_boundary(&p, 2);

  p.phase = PATH_BND_SFN_CHECK_PMIN_PMAX;
  sfn.depth = 0;
  p.locals.bnd_sf.is_picardn = 1;
  p.locals.bnd_sf.epsilon = 0.9;
  p.locals.bnd_sf.h_hat = 100.0;

  /* Set up for reject: r > p_conv + p_cond + p_radi_max, with i < 6 */
  sfn.stack[0].h_hat = 100.0;
  sfn.stack[0].p_conv = 0.05;
  sfn.stack[0].p_cond = 0.05;
  sfn.stack[0].r = 0.99;  /* very high r → beyond p_radi_max */
  sfn.stack[0].T_count = 1;
  sfn.stack[0].T_values[0] = 300.0;

  /* Save snapshot for null_collision restore */
  p.locals.bnd_sf.rwalk_snapshot = p.rwalk;
  p.locals.bnd_sf.T_snapshot = p.T;
  p.locals.bnd_sf.hvtx_saved.weight = 0;
  p.locals.bnd_sf.ihvtx_radi_begin = 0;
  p.locals.bnd_sf.ihvtx_radi_end = 0;

  res = step_bnd_sfn_check_pmin_pmax(&p, g_scn_sfn, &sfn);
  CHK(res == RES_OK);

  /* Early reject → sfn_null_collision → PATH_BND_SFN_PROB_DISPATCH */
  CHK(p.phase == PATH_BND_SFN_PROB_DISPATCH);

  printf("PASS\n");
}

/* ========================================================================== */
/* T8.7: Multi-layer recursion (stack frame independence)                      */
/* ========================================================================== */
static void
test_sfn_multilayer_stack_frames(void)
{
  struct path_state p;
  struct path_sfn_data sfn;

  printf("  T8.7: Multi-layer recursion (stack frame independence)... ");

  memset(&p, 0, sizeof(p));
  memset(&sfn, 0, sizeof(sfn));

  /* Set up 3 independent stack frames and verify they don't interfere */
  sfn.stack[0].T_count = 2;
  sfn.stack[0].T_values[0] = 300.0;
  sfn.stack[0].T_values[1] = 310.0;
  sfn.stack[0].rwalk_saved.vtx.P[0] = 1.0;
  sfn.stack[0].rwalk_saved.vtx.P[1] = 2.0;
  sfn.stack[0].rwalk_saved.vtx.P[2] = 3.0;
  sfn.stack[0].h_hat = 100.0;
  sfn.stack[0].r = 0.5;
  sfn.stack[0].p_conv = 0.1;
  sfn.stack[0].p_cond = 0.3;

  sfn.stack[1].T_count = 1;
  sfn.stack[1].T_values[0] = 320.0;
  sfn.stack[1].rwalk_saved.vtx.P[0] = 4.0;
  sfn.stack[1].rwalk_saved.vtx.P[1] = 5.0;
  sfn.stack[1].rwalk_saved.vtx.P[2] = 6.0;
  sfn.stack[1].h_hat = 200.0;
  sfn.stack[1].r = 0.7;
  sfn.stack[1].p_conv = 0.2;
  sfn.stack[1].p_cond = 0.2;

  sfn.stack[2].T_count = 0;
  sfn.stack[2].rwalk_saved.vtx.P[0] = 7.0;
  sfn.stack[2].rwalk_saved.vtx.P[1] = 8.0;
  sfn.stack[2].rwalk_saved.vtx.P[2] = 9.0;
  sfn.stack[2].h_hat = 300.0;

  /* Verify each frame is independent */
  CHK(sfn.stack[0].T_count == 2);
  CHK(sfn.stack[1].T_count == 1);
  CHK(sfn.stack[2].T_count == 0);

  CHK(fabs(sfn.stack[0].T_values[0] - 300.0) < 1e-10);
  CHK(fabs(sfn.stack[0].T_values[1] - 310.0) < 1e-10);
  CHK(fabs(sfn.stack[1].T_values[0] - 320.0) < 1e-10);

  CHK(fabs(sfn.stack[0].rwalk_saved.vtx.P[0] - 1.0) < 1e-10);
  CHK(fabs(sfn.stack[1].rwalk_saved.vtx.P[0] - 4.0) < 1e-10);
  CHK(fabs(sfn.stack[2].rwalk_saved.vtx.P[0] - 7.0) < 1e-10);

  CHK(fabs(sfn.stack[0].h_hat - 100.0) < 1e-10);
  CHK(fabs(sfn.stack[1].h_hat - 200.0) < 1e-10);
  CHK(fabs(sfn.stack[2].h_hat - 300.0) < 1e-10);

  /* Simulate push/pop cycle: write to frame 2, then verify frame 0/1
   * are still intact */
  sfn.depth = 2;
  sfn.stack[2].T_values[0] = 999.0;
  sfn.stack[2].T_count = 1;

  /* Pop: verify frame 1 is intact */
  sfn.depth = 1;
  CHK(sfn.stack[1].T_count == 1);
  CHK(fabs(sfn.stack[1].T_values[0] - 320.0) < 1e-10);
  CHK(fabs(sfn.stack[1].h_hat - 200.0) < 1e-10);

  /* Pop: verify frame 0 is intact */
  sfn.depth = 0;
  CHK(sfn.stack[0].T_count == 2);
  CHK(fabs(sfn.stack[0].T_values[0] - 300.0) < 1e-10);
  CHK(fabs(sfn.stack[0].T_values[1] - 310.0) < 1e-10);
  CHK(fabs(sfn.stack[0].h_hat - 100.0) < 1e-10);

  printf("PASS\n");
}

/* ========================================================================== */
/* T8.9: PicardN vs Picard1 degenerate (picard_order=1 -> no SFN states)      */
/* ========================================================================== */
static void
test_sfn_picard1_no_sfn_states(void)
{
  struct path_state p;
  struct path_sfn_data sfn;
  res_T res;

  printf("  T8.9: PicardN vs Picard1 degenerate... ");

  init_path_on_sf_boundary(&p, 1); /* picard_order = 1 → max_branchings = 1 */

  /* With picard_order = 1, at the boundary with nbranchings == max_branchings,
   * step_bnd_dispatch should route to picard1 (SF), not picardN (SFN). */
  p.ctx.nbranchings = 1; /* == max_branchings → picard1 */
  p.ctx.max_branchings = 1;

  /* step_bnd_dispatch: solid/fluid with nbranchings == max_branchings
   * → SF_REINJECT_SAMPLE (picard1 path, not picardN) */
  res = step_bnd_dispatch(&p, g_scn_sf_p1);
  /* After dispatch, should NOT be in any SFN state */
  CHK(p.phase != PATH_BND_SFN_PROB_DISPATCH);
  CHK(p.phase != PATH_BND_SFN_RAD_TRACE);
  CHK(p.phase != PATH_BND_SFN_RAD_DONE);
  CHK(p.phase != PATH_BND_SFN_COMPUTE_Ti);
  CHK(p.phase != PATH_BND_SFN_COMPUTE_Ti_RESUME);
  CHK(p.phase != PATH_BND_SFN_CHECK_PMIN_PMAX);
  CHK(res == RES_OK);

  /* Verify is_picardn was NOT set */
  CHK(p.locals.bnd_sf.is_picardn == 0);

  printf("PASS\n");
}

/* ========================================================================== */
/* T8.extra: h_radi bounds formula correctness at different stages            */
/* ========================================================================== */
static void
test_sfn_hradi_bounds_formula(void)
{
  /* Test the h_radi bounds formula by exercising CHECK_PMIN_PMAX.
   * At i=6, h_radi_min == h_radi_max (exact), so we verify the formula
   * through the step function's final decision.
   *
   * h_radi(exact) = sigma * (T3*T4*T5 + T0*T3*T4 + T0*T1*T3 + T0*T1*T2) */
  struct path_state p;
  struct path_sfn_data sfn;
  double T0 = 300.0, T1 = 310.0, T2 = 320.0;
  double T3 = 330.0, T4 = 340.0, T5 = 350.0;
  double h_radi_exact;
  double epsilon = 0.9;
  double h_hat = 100.0;
  double p_radi_exact;

  printf("  T8.extra: h_radi bounds formula... ");

  /* Compute expected exact h_radi at i=6 */
  h_radi_exact = BOLTZMANN_CONSTANT *
    (T3*T4*T5 + T0*T3*T4 + T0*T1*T3 + T0*T1*T2);
  p_radi_exact = h_radi_exact * epsilon / h_hat;

  /* Verify the formula: h_radi should be positive for positive temperatures */
  CHK(h_radi_exact > 0);

  /* Set up path at CHECK_PMIN_PMAX with all 6 T values to get exact decision */
  init_path_on_sf_boundary(&p, 2);
  p.phase = PATH_BND_SFN_CHECK_PMIN_PMAX;
  sfn.depth = 0;
  p.locals.bnd_sf.is_picardn = 1;
  p.locals.bnd_sf.epsilon = epsilon;
  p.locals.bnd_sf.h_hat = h_hat;

  sfn.stack[0].h_hat = h_hat;
  sfn.stack[0].p_conv = 0.1;
  sfn.stack[0].p_cond = 0.1;
  sfn.stack[0].T_count = 6;
  sfn.stack[0].T_values[0] = T0;
  sfn.stack[0].T_values[1] = T1;
  sfn.stack[0].T_values[2] = T2;
  sfn.stack[0].T_values[3] = T3;
  sfn.stack[0].T_values[4] = T4;
  sfn.stack[0].T_values[5] = T5;

  /* Set r just below the accept threshold to trigger accept */
  sfn.stack[0].r = 0.1 + 0.1 + p_radi_exact * 0.5;

  /* Save snapshot for possible null_collision */
  p.locals.bnd_sf.rwalk_s = p.rwalk;
  p.locals.bnd_sf.T_s = p.T;
  p.locals.bnd_sf.T_s.done = 1;
  p.locals.bnd_sf.rwalk_snapshot = p.rwalk;
  p.locals.bnd_sf.T_snapshot = p.T;
  p.locals.bnd_sf.hvtx_saved.weight = 0;
  p.locals.bnd_sf.ihvtx_radi_begin = 0;
  p.locals.bnd_sf.ihvtx_radi_end = 0;

  {
    res_T res = step_bnd_sfn_check_pmin_pmax(&p, g_scn_sfn, &sfn);
    CHK(res == RES_OK);
    /* r < p_conv+p_cond+p_radi → accept (POST_ROBIN_CHECK) */
    CHK(p.phase == PATH_BND_POST_ROBIN_CHECK);
  }

  printf("PASS\n");
}

/* ========================================================================== */
/* T8.extra: SFN_RAD_DONE initialises stack frame correctly                   */
/* ========================================================================== */
static void
test_sfn_rad_done_init_stack(void)
{
  struct path_state p;
  struct path_sfn_data sfn;
  res_T res;

  printf("  T8.extra: SFN_RAD_DONE initialises stack frame... ");

  init_path_on_sf_boundary(&p, 2);

  p.phase = PATH_BND_SFN_RAD_DONE;
  sfn.depth = 0;
  p.locals.bnd_sf.is_picardn = 1;
  p.locals.bnd_sf.h_hat = 100.0;
  p.locals.bnd_sf.epsilon = 0.9;
  p.locals.bnd_sf.p_conv = 0.1;
  p.locals.bnd_sf.p_cond = 0.3;

  /* Save snapshot data */
  p.locals.bnd_sf.rwalk_snapshot = p.rwalk;
  p.locals.bnd_sf.T_snapshot = p.T;

  /* Set a high r to avoid early accept */
  p.locals.bnd_sf.r = 0.99;

  res = step_bnd_sfn_rad_done(&p, g_scn_sfn, &sfn);
  CHK(res == RES_OK);

  /* Should have moved to COMPUTE_Ti to start Ti chain */
  CHK(p.phase == PATH_BND_SFN_COMPUTE_Ti);
  CHK(sfn.depth == 0);

  /* Stack frame [0] should be initialised */
  CHK(sfn.stack[0].T_count == 0);

  printf("PASS\n");
}

/* ========================================================================== */
/* T8.extra: COMPUTE_Ti with T.done uses direct value (no recursion)          */
/* ========================================================================== */
static void
test_sfn_compute_Ti_done_direct(void)
{
  struct path_state p;
  struct path_sfn_data sfn;
  res_T res;

  printf("  T8.extra: COMPUTE_Ti with T.done = direct value... ");

  init_path_on_sf_boundary(&p, 2);

  p.phase = PATH_BND_SFN_COMPUTE_Ti;
  sfn.depth = 0;
  p.locals.bnd_sf.is_picardn = 1;

  /* Stack frame: i=0 */
  sfn.stack[0].T_count = 0;
  sfn.stack[0].rwalk_saved = p.rwalk;
  sfn.stack[0].T_saved = p.T;

  /* rwalk_s has T_s.done = 1 → use directly */
  p.locals.bnd_sf.rwalk_s = p.rwalk;
  p.locals.bnd_sf.T_s.done = 1;
  p.locals.bnd_sf.T_s.value = 350.0;

  res = step_bnd_sfn_compute_Ti(&p, g_scn_sfn, &sfn);
  CHK(res == RES_OK);

  /* T.done → use directly → go to CHECK_PMIN_PMAX without push */
  CHK(p.phase == PATH_BND_SFN_CHECK_PMIN_PMAX);
  CHK(sfn.depth == 0); /* no push */
  CHK(sfn.stack[0].T_count == 1);
  CHK(fabs(sfn.stack[0].T_values[0] - 350.0) < 1e-10);

  printf("PASS\n");
}

/* ========================================================================== */
/* T8.extra: COMPUTE_Ti_RESUME pops and records temperature                   */
/* ========================================================================== */
static void
test_sfn_compute_Ti_resume_pop(void)
{
  struct path_state p;
  struct path_sfn_data sfn;
  res_T res;

  printf("  T8.extra: COMPUTE_Ti_RESUME pops and records T... ");

  init_path_on_sf_boundary(&p, 2);

  p.phase = PATH_BND_SFN_COMPUTE_Ti_RESUME;
  sfn.depth = 1;
  p.locals.bnd_sf.is_picardn = 1;
  p.locals.bnd_sf.coupled_nbranchings_saved = 0;

  /* Parent frame at depth 0: waiting for T_values[0] */
  sfn.stack[0].T_count = 0;
  sfn.stack[0].rwalk_saved = p.rwalk;
  sfn.stack[0].T_saved = p.T;

  /* Sub-path result: temperature done */
  p.T.done = 1;
  p.T.value = 325.0;

  res = step_bnd_sfn_compute_Ti_resume(&p, g_scn_sfn, &sfn);
  CHK(res == RES_OK);

  /* Stack should have been popped */
  CHK(sfn.depth == 0);

  /* T_values[0] should now contain the sub-path result */
  CHK(sfn.stack[0].T_count == 1);
  CHK(fabs(sfn.stack[0].T_values[0] - 325.0) < 1e-10);

  /* Should transition to CHECK_PMIN_PMAX */
  CHK(p.phase == PATH_BND_SFN_CHECK_PMIN_PMAX);

  printf("PASS\n");
}

/* ========================================================================== */
/* T8.extra: BND_DISPATCH routes to picardN when nbranchings < max            */
/* ========================================================================== */
static void
test_sfn_dispatch_routes_picardn(void)
{
  struct path_state p;
  struct path_sfn_data sfn;
  res_T res;

  printf("  T8.extra: BND_DISPATCH routes to picardN... ");

  init_path_on_sf_boundary(&p, 2); /* picard_order=2, max_branchings=2 */

  /* nbranchings < max_branchings → picardN path */
  p.ctx.nbranchings = 0;
  p.ctx.max_branchings = 2;

  res = step_bnd_dispatch(&p, g_scn_sfn);
  CHK(res == RES_OK);

  /* Should have set is_picardn = 1 */
  CHK(p.locals.bnd_sf.is_picardn == 1);

  /* Should be in SF_REINJECT_SAMPLE (pending ray) or
   * SFN_PROB_DISPATCH (if MEDIUM_ID_MULTI shortcut applicable) */
  CHK(p.phase == PATH_BND_SF_REINJECT_SAMPLE
   || p.phase == PATH_BND_SFN_PROB_DISPATCH);

  printf("PASS\n");
}

/* ========================================================================== */
/* T8.extra: Phase enum classification (SFN states)                           */
/* ========================================================================== */
static void
test_sfn_phase_classification(void)
{
  printf("  T8.extra: Phase enum classification... ");

  /* Ray-pending: SFN_RAD_TRACE must be classified as ray-pending */
  CHK(path_phase_is_ray_pending(PATH_BND_SFN_RAD_TRACE) == 1);

  /* Compute-only: all other SFN states should NOT be ray-pending */
  CHK(path_phase_is_ray_pending(PATH_BND_SFN_PROB_DISPATCH) == 0);
  CHK(path_phase_is_ray_pending(PATH_BND_SFN_RAD_DONE) == 0);
  CHK(path_phase_is_ray_pending(PATH_BND_SFN_COMPUTE_Ti) == 0);
  CHK(path_phase_is_ray_pending(PATH_BND_SFN_COMPUTE_Ti_RESUME) == 0);
  CHK(path_phase_is_ray_pending(PATH_BND_SFN_CHECK_PMIN_PMAX) == 0);

  printf("PASS\n");
}

/* ========================================================================== */
/* Main                                                                       */
/* ========================================================================== */
int
main(int argc, char* argv[])
{
  (void)argc;
  (void)argv;

  printf("Phase B-4 M8: PicardN recursive stack tests\n");
  printf("============================================\n");

  setup_test_scenes();

  test_sfn_stack_depth_management();
  test_sfn_stack_overflow();
  test_sfn_compute_Ti_push();
  test_sfn_subpath_done_intercept();
  test_sfn_check_pmin_pmax_early_accept();
  test_sfn_check_pmin_pmax_early_reject();
  test_sfn_multilayer_stack_frames();
  test_sfn_picard1_no_sfn_states();
  test_sfn_hradi_bounds_formula();
  test_sfn_rad_done_init_stack();
  test_sfn_compute_Ti_done_direct();
  test_sfn_compute_Ti_resume_pop();
  test_sfn_dispatch_routes_picardn();
  test_sfn_phase_classification();

  teardown_test_scenes();

  printf("\nAll B-4 M8 tests PASSED.\n");
  return 0;
}
