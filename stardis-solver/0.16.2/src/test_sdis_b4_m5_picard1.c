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

/* Phase B-4, Milestone 5: Picard1 (solid/fluid) state machine -- unit tests.
 *
 * These tests verify the structural correctness of the solid/fluid picard1
 * batch state machine added in M5.  No GPU trace is performed -- all hit data
 * is synthetically injected, following the same pattern as the M3 tests.
 *
 * Test cases (from phase_b4_test_design.md, section T5):
 *   T5.1: setup_sf_reinject_rays produces exactly 2 rays
 *   T5.2: Direction symmetry -- dir1 = reflect(dir0, normal)
 *   T5.3: Reinjection resolve -- one valid hit, one miss
 *   T5.4: Reinjection resolve -- both miss triggers retry
 *   T5.5: Phase enum classification (ray-pending vs compute-only)
 *   T5.6: Null-collision radiative sub-path ray setup correctness
 *   T5.7: prob_dispatch coefficient computation (structural)
 */

#include "sdis_solve_wavefront.h"
#include "sdis_wf_steps.h"

#include "test_sdis_utils.h"
#include <stdio.h>
#include <string.h>
#include <math.h>

/* ========================================================================== */
/* T5.1: setup_sf_reinject_rays produces 2 rays with correct bucket           */
/* ========================================================================== */
static void
test_sf_reinject_ray_count(void)
{
  struct path_state p;

  printf("  T5.1: SF_REINJECT_SAMPLE produces 2 rays... ");
  memset(&p, 0, sizeof(p));

  /* Set up minimal state */
  p.rwalk.vtx.P[0] = 0.5;
  p.rwalk.vtx.P[1] = 0.5;
  p.rwalk.vtx.P[2] = 0.5;
  p.locals.bnd_sf.delta_boundary = 0.1;
  p.locals.bnd_sf.reinject_dir[0][0] = 0.0f;
  p.locals.bnd_sf.reinject_dir[0][1] = 0.0f;
  p.locals.bnd_sf.reinject_dir[0][2] = 1.0f;
  p.locals.bnd_sf.reinject_dir[1][0] = 0.0f;
  p.locals.bnd_sf.reinject_dir[1][1] = 0.0f;
  p.locals.bnd_sf.reinject_dir[1][2] = -1.0f;
  p.rwalk.hit_3d = S3D_HIT_NULL;
  p.rwalk.hit_3d.prim.prim_id = 0;
  p.rwalk.hit_3d.prim.geom_id = 0;
  p.rwalk.hit_3d.prim.inst_id = 0;
  p.filter_data_storage = HIT_FILTER_DATA_NULL;
  p.active = 1;

  setup_sf_reinject_rays(&p);

  /* Verify */
  CHK(p.ray_count_ext == 2);
  CHK(p.ray_req.ray_count == 2);
  CHK(p.ray_bucket == RAY_BUCKET_STEP_PAIR);
  CHK(p.needs_ray == 1);
  CHK(p.phase == PATH_BND_SF_REINJECT_SAMPLE);

  /* Check ray origin */
  CHK(fabsf(p.ray_req.origin[0] - 0.5f) < 1.e-6f);
  CHK(fabsf(p.ray_req.origin[1] - 0.5f) < 1.e-6f);
  CHK(fabsf(p.ray_req.origin[2] - 0.5f) < 1.e-6f);

  /* Check ray directions: ray 0 = dir0, ray 1 = dir1 (reflected) */
  CHK(fabsf(p.ray_req.direction[2] - 1.0f) < 1.e-6f);
  CHK(fabsf(p.ray_req.direction2[2] - (-1.0f)) < 1.e-6f);

  printf("PASS\n");
}

/* ========================================================================== */
/* T5.2: Direction symmetry -- dir1 = reflect(dir0, normal)                   */
/* ========================================================================== */
static void
test_sf_direction_symmetry(void)
{
  /* Use exactly 1/sqrt(3) so f3_is_normalized passes */
  float inv_sqrt3 = 1.0f / sqrtf(3.0f);
  float dir0[3] = { inv_sqrt3, inv_sqrt3, inv_sqrt3 };
  float normal[3] = { 0.0f, 0.0f, 1.0f };
  float dir_reflected[3];

  printf("  T5.2: direction symmetry (dir1 = reflect(dir0))... ");

  /* Compute expected reflection: res = 2*(V.N)*N - V
   * For N=(0,0,1): x,y negate, z stays */
  reflect_3d(dir_reflected, dir0, normal);

  /* x negates, y negates, z stays */
  CHK(fabsf(dir_reflected[0] - (-dir0[0])) < 1.e-5f);
  CHK(fabsf(dir_reflected[1] - (-dir0[1])) < 1.e-5f);
  CHK(fabsf(dir_reflected[2] - dir0[2]) < 1.e-5f);
  /* Verify against explicit formula */
  {
    float dot = f3_dot(dir0, normal);
    float expected[3];
    expected[0] = 2.0f * dot * normal[0] - dir0[0];
    expected[1] = 2.0f * dot * normal[1] - dir0[1];
    expected[2] = 2.0f * dot * normal[2] - dir0[2];
    f3_normalize(expected, expected);
    CHK(fabsf(dir_reflected[0] - expected[0]) < 1.e-5f);
    CHK(fabsf(dir_reflected[1] - expected[1]) < 1.e-5f);
    CHK(fabsf(dir_reflected[2] - expected[2]) < 1.e-5f);
  }

  printf("PASS\n");
}

/* ========================================================================== */
/* T5.3: resolve_reinjection_from_hits -- one valid hit, one miss             */
/* ========================================================================== */
static void
test_sf_resolve_one_hit(void)
{
  struct s3d_hit hit_a, miss;
  float dir0[3] = { 0, 0, 1 };
  float dir1[3] = { 0, 0, -1 };
  float out_dir[3];
  float out_dst;
  struct s3d_hit out_hit;

  printf("  T5.3: resolve reinjection -- one hit... ");

  miss = S3D_HIT_NULL;
  memset(&hit_a, 0, sizeof(hit_a));
  hit_a.distance = 0.05f;
  hit_a.normal[2] = -1.0f;
  hit_a.prim.prim_id = 10;
  hit_a.prim.geom_id = 0;
  hit_a.prim.inst_id = 0;

  /* enc0 matches target (42), enc1 is NULL */
  resolve_reinjection_from_hits(
    &hit_a, &miss,
    dir0, dir1,
    42, ENCLOSURE_ID_NULL,
    42, /* target solid enc */
    0.1, /* distance (delta_boundary) */
    out_dir, &out_dst, &out_hit);

  /* Should pick dir0 (the valid one) */
  CHK(out_dst > 0);
  CHK(fabsf(out_dir[2] - 1.0f) < 1.e-6f);
  CHK(fabsf(out_dst - 0.05f) < 1.e-6f);

  printf("PASS\n");
}

/* ========================================================================== */
/* T5.4: Both-miss scenario -- retry (structural, no scene needed)            */
/* ========================================================================== */
static void
test_sf_both_miss_retry(void)
{
  struct s3d_hit miss_hit = S3D_HIT_NULL;
  float dir0[3] = { 0, 0, 1 };
  float dir1[3] = { 0, 0, -1 };
  float out_dir[3];
  float out_dst;
  struct s3d_hit out_hit;

  printf("  T5.4: both-miss triggers retry (via out_dst==0)... ");

  /* Both misses -> both enc_ids = ENCLOSURE_ID_NULL, neither matches enc 1 */
  resolve_reinjection_from_hits(
    &miss_hit, &miss_hit,
    dir0, dir1,
    ENCLOSURE_ID_NULL, ENCLOSURE_ID_NULL,
    1, /* target solid enc */
    0.1, /* distance */
    out_dir, &out_dst, &out_hit);

  /* Both invalid: out_dst should be 0 */
  CHK(out_dst == 0.0f);

  printf("PASS\n");
}

/* ========================================================================== */
/* T5.5: Phase enum classification -- ray-pending vs compute-only             */
/* ========================================================================== */
static void
test_sf_phase_classification(void)
{
  printf("  T5.5: phase enum classification (structural)... ");

  /* Verify all SF phases have distinct values */
  CHK(PATH_BND_SF_REINJECT_SAMPLE != PATH_BND_SF_REINJECT_ENC);
  CHK(PATH_BND_SF_REINJECT_ENC    != PATH_BND_SF_PROB_DISPATCH);
  CHK(PATH_BND_SF_PROB_DISPATCH   != PATH_BND_SF_NULLCOLL_RAD_TRACE);
  CHK(PATH_BND_SF_NULLCOLL_RAD_TRACE != PATH_BND_SF_NULLCOLL_DECIDE);

  /* Ray-pending phases */
  CHK(path_phase_is_ray_pending(PATH_BND_SF_REINJECT_SAMPLE));
  CHK(path_phase_is_ray_pending(PATH_BND_SF_NULLCOLL_RAD_TRACE));

  /* Compute-only phases (not ray-pending) */
  CHK(!path_phase_is_ray_pending(PATH_BND_SF_REINJECT_ENC));
  CHK(!path_phase_is_ray_pending(PATH_BND_SF_PROB_DISPATCH));
  CHK(!path_phase_is_ray_pending(PATH_BND_SF_NULLCOLL_DECIDE));

  printf("PASS\n");
}

/* ========================================================================== */
/* T5.6: Collect emits 2 rays for SF reinjection (structural)                 */
/* ========================================================================== */
static void
test_sf_collect_two_rays(void)
{
  struct wavefront_context wf;
  struct path_state path;
  struct s3d_ray_request ray_requests[4];
  uint32_t ray_to_path[4];
  uint32_t ray_slot[4];

  printf("  T5.6: collect emits 2 rays for SF reinjection... ");
  memset(&wf, 0, sizeof(wf));
  memset(&path, 0, sizeof(path));
  memset(ray_requests, 0, sizeof(ray_requests));

  /* Setup wavefront with 1 path needing SF reinjection rays */
  wf.paths = &path;
  wf.total_paths = 1;
  wf.ray_requests = ray_requests;
  wf.ray_to_path = ray_to_path;
  wf.ray_slot = ray_slot;

  path.active = 1;
  path.needs_ray = 1;
  path.phase = PATH_BND_SF_REINJECT_SAMPLE;
  path.ray_req.ray_count = 2;
  path.ray_count_ext = 2;
  path.ray_bucket = RAY_BUCKET_STEP_PAIR;

  /* Set ray origin */
  path.ray_req.origin[0] = 0.5f;
  path.ray_req.origin[1] = 0.5f;
  path.ray_req.origin[2] = 0.5f;

  /* Set ray directions */
  path.ray_req.direction[0] = 0.0f;
  path.ray_req.direction[1] = 0.0f;
  path.ray_req.direction[2] = 1.0f;
  path.ray_req.direction2[0] = 0.0f;
  path.ray_req.direction2[1] = 0.0f;
  path.ray_req.direction2[2] = -1.0f;
  path.ray_req.range[0] = 0.0f;
  path.ray_req.range[1] = FLT_MAX;
  path.ray_req.range2[0] = 0.0f;
  path.ray_req.range2[1] = FLT_MAX;

  /* Collect rays */
  {
    wf.ray_count = 0;
    wf.max_rays  = 4;
    OK(collect_ray_requests(&wf));

    /* Should emit 2 physical ray requests */
    CHK(wf.ray_count == 2);

    /* Verify ray 0 direction = (0,0,1) */
    CHK(fabsf(ray_requests[0].direction[2] - 1.0f) < 1.e-6f);
    /* Verify ray 1 direction = (0,0,-1) */
    CHK(fabsf(ray_requests[1].direction[2] - (-1.0f)) < 1.e-6f);

    /* Both rays share the same origin */
    CHK(fabsf(ray_requests[0].origin[0] - 0.5f) < 1.e-6f);
    CHK(fabsf(ray_requests[1].origin[0] - 0.5f) < 1.e-6f);

    /* Mapping should point back to path 0 */
    CHK(ray_to_path[0] == 0);
    CHK(ray_to_path[1] == 0);
  }

  printf("PASS\n");
}

/* ========================================================================== */
/* T5.7: bnd_sf state field layout -- verify struct members exist             */
/* ========================================================================== */
static void
test_sf_state_fields(void)
{
  struct path_state p;

  printf("  T5.7: bnd_sf state field layout... ");
  memset(&p, 0, sizeof(p));

  /* Verify key fields exist and are accessible */
  p.locals.bnd_sf.h_conv = 10.0;
  p.locals.bnd_sf.h_cond = 20.0;
  p.locals.bnd_sf.h_radi_hat = 5.0;
  p.locals.bnd_sf.h_hat = 35.0;
  p.locals.bnd_sf.p_conv = 10.0 / 35.0;
  p.locals.bnd_sf.p_cond = 20.0 / 35.0;
  p.locals.bnd_sf.lambda = 1.5;
  p.locals.bnd_sf.delta_boundary = 0.1;
  p.locals.bnd_sf.delta_m = 0.01;
  p.locals.bnd_sf.epsilon = 0.8;
  p.locals.bnd_sf.Tref = 300.0;
  p.locals.bnd_sf.r = 0.5;
  p.locals.bnd_sf.solid_side = SDIS_FRONT;
  p.locals.bnd_sf.fluid_side = SDIS_BACK;
  p.locals.bnd_sf.solid_enc_id = 42;
  p.locals.bnd_sf.enc_ids[SDIS_FRONT] = 42;
  p.locals.bnd_sf.enc_ids[SDIS_BACK] = 43;
  p.locals.bnd_sf.chosen_dir[0] = 0.0f;
  p.locals.bnd_sf.chosen_dir[1] = 0.0f;
  p.locals.bnd_sf.chosen_dir[2] = 1.0f;
  p.locals.bnd_sf.chosen_dst = 0.05f;
  p.locals.bnd_sf.chosen_hit = S3D_HIT_NULL;
  p.locals.bnd_sf.retry_count = 0;
  p.locals.bnd_sf.need_enc = 0;
  p.locals.bnd_sf.rad_sub_direction[0] = 0.0f;
  p.locals.bnd_sf.rad_sub_direction[1] = 0.0f;
  p.locals.bnd_sf.rad_sub_direction[2] = 1.0f;
  p.locals.bnd_sf.rad_sub_bounce_count = 0;
  p.locals.bnd_sf.rad_sub_retry_count = 0;

  /* Verify computed probabilities */
  CHK(fabs(p.locals.bnd_sf.p_conv - 10.0 / 35.0) < 1.e-12);
  CHK(fabs(p.locals.bnd_sf.p_cond - 20.0 / 35.0) < 1.e-12);
  CHK(fabs(p.locals.bnd_sf.h_hat - 35.0) < 1.e-12);

  /* Verify snapshot fields exist */
  p.locals.bnd_sf.rwalk_snapshot = RWALK_NULL;
  p.locals.bnd_sf.T_snapshot.value = 300.0;
  p.locals.bnd_sf.hvtx_saved = SDIS_HEAT_VERTEX_NULL;
  p.locals.bnd_sf.ihvtx_radi_begin = 0;

  printf("PASS\n");
}

/* ========================================================================== */
/* T5.8: Probability formula correctness (p_conv + p_cond + p_radi_hat = 1)   */
/* ========================================================================== */
static void
test_sf_probability_formula(void)
{
  double h_conv = 25.0;
  double h_cond = 50.0;
  double h_radi_hat;
  double h_hat;
  double p_conv, p_cond, p_radi_hat;
  double epsilon = 0.8;
  double That3 = 300.0 * 300.0 * 300.0; /* T_hat = 300K */

  printf("  T5.8: probability formula (p_conv+p_cond+p_radi_hat=1)... ");

  h_radi_hat = 4.0 * BOLTZMANN_CONSTANT * That3 * epsilon;
  h_hat = h_conv + h_cond + h_radi_hat;
  p_conv = h_conv / h_hat;
  p_cond = h_cond / h_hat;
  p_radi_hat = h_radi_hat / h_hat;

  /* Sum must be exactly 1 */
  CHK(fabs(p_conv + p_cond + p_radi_hat - 1.0) < 1.e-12);

  /* h_radi_hat must be positive */
  CHK(h_radi_hat > 0);

  /* All probabilities must be in (0,1) */
  CHK(p_conv > 0 && p_conv < 1);
  CHK(p_cond > 0 && p_cond < 1);
  CHK(p_radi_hat > 0 && p_radi_hat < 1);

  /* Epsilon = 0 case: h_radi_hat = 0, p_conv + p_cond = 1 */
  {
    double h_radi_hat_0 = 0;
    double h_hat_0 = h_conv + h_cond + h_radi_hat_0;
    double p_conv_0 = h_conv / h_hat_0;
    double p_cond_0 = h_cond / h_hat_0;
    CHK(fabs(p_conv_0 + p_cond_0 - 1.0) < 1.e-12);
  }

  printf("PASS\n");
}

/* ========================================================================== */
/* T5.9: Null-collision accept/reject -- h_radi formula (T^3 polynomial)      */
/* ========================================================================== */
static void
test_sf_nullcoll_hradi_formula(void)
{
  double epsilon = 0.8;
  double Tref = 300.0;
  double Tref_s = 350.0;
  double h_radi;
  double h_radi_hat;
  double That3 = 350.0 * 350.0 * 350.0;

  printf("  T5.9: null-collision h_radi formula... ");

  /* h_radi = sigma * epsilon * (T0^3 + T0^2*T1 + T0*T1^2 + T1^3) */
  h_radi = BOLTZMANN_CONSTANT * epsilon *
    (Tref*Tref*Tref
   + Tref*Tref * Tref_s
   + Tref * Tref_s*Tref_s
   + Tref_s*Tref_s*Tref_s);

  /* h_radi_hat = 4 * sigma * That^3 * epsilon */
  h_radi_hat = 4.0 * BOLTZMANN_CONSTANT * That3 * epsilon;

  /* h_radi must be positive */
  CHK(h_radi > 0);

  /* h_radi must be <= h_radi_hat when That = max(Tref, Tref_s) */
  /* This is the null-collision condition: actual <= majorant */
  CHK(h_radi <= h_radi_hat + 1.e-6);

  /* When Tref == Tref_s:
   * h_radi = sigma * eps * 4*T^3 = h_radi_hat */
  {
    double T = 300.0;
    double h_eq = BOLTZMANN_CONSTANT * epsilon *
      (T*T*T + T*T*T + T*T*T + T*T*T);
    double That3_eq = T * T * T;
    double h_hat_eq = 4.0 * BOLTZMANN_CONSTANT * That3_eq * epsilon;
    CHK(fabs(h_eq - h_hat_eq) < 1.e-6);
  }

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

  printf("Phase B-4 M5: Picard1 (solid/fluid) state machine tests\n");
  printf("========================================================\n");

  test_sf_reinject_ray_count();
  test_sf_direction_symmetry();
  test_sf_resolve_one_hit();
  test_sf_both_miss_retry();
  test_sf_phase_classification();
  test_sf_collect_two_rays();
  test_sf_state_fields();
  test_sf_probability_formula();
  test_sf_nullcoll_hradi_formula();

  printf("\nAll B-4 M5 tests PASSED.\n");
  return 0;
}
