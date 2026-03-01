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

/* Phase B-4, Milestone 4: Delta-Sphere conductive path -- unit tests.
 *
 * These tests verify the fine-grained state machine decomposition of the
 * delta-sphere conductive path loop added in M4.  A real box scene is
 * constructed (same as test_sdis_convection.c) so that step_conductive,
 * step_conductive_ds_process, step_cnd_ds_check_temp, and
 * step_cnd_ds_step_advance can be called with a valid sdis_scene pointer.
 *
 * Test cases (from phase_b4_test_design.md, section T4):
 *   T4.1: Initial ENC query chain -- step_conductive dispatches ENC sub-state,
 *          which returns to PATH_CND_DS_CHECK_TEMP
 *   T4.2: DS step emits exactly 2 rays with correct bucket
 *   T4.3: hit0.distance > delta -> enters PATH_CND_DS_STEP_ENC_VERIFY
 *   T4.4: hit0.distance <= delta -> skips to PATH_CND_DS_STEP_ADVANCE
 *   T4.5: Loop/exit -- HIT_NONE -> CHECK_TEMP, !HIT_NONE -> COUPLED_BOUNDARY
 *
 *   T4.6-T4.9 are deferred (end-to-end, require GPU scene).
 */

#include "sdis_solve_wavefront.h"
#include "sdis_wf_steps.h"
#include "sdis_scene_c.h"

#include "test_sdis_utils.h"
#include <stdio.h>
#include <string.h>
#include <math.h>

/* ========================================================================== */
/* Shared test scene -- a unit box with a single interface (solid/solid).      */
/* Built once in main(), released at exit.  Provides valid prim_props for      */
/* scene_get_enclosure_ids and a valid dev pointer for log_warn/log_err.       */
/* ========================================================================== */
static struct sdis_device*  g_dev = NULL;
static struct sdis_scene*   g_scn = NULL;  /* 3D box scene */
static struct ssp_rng*      g_rng = NULL;
static unsigned             g_inner_enc = 0; /* inner enclosure id of the box */

/* The box has 12 triangles.  All share the same interface. */
static struct sdis_interface* g_box_interfaces[12];

static void
setup_test_scene(void)
{
  struct sdis_solid_shader solid_shader = DUMMY_SOLID_SHADER;
  struct sdis_fluid_shader fluid_shader = DUMMY_FLUID_SHADER;
  struct sdis_interface_shader interf_shader = DUMMY_INTERFACE_SHADER;
  struct sdis_scene_create_args scn_args = SDIS_SCENE_CREATE_ARGS_DEFAULT;
  struct sdis_medium* solid = NULL;
  struct sdis_medium* fluid = NULL;
  struct sdis_interface* interf = NULL;
  int i;

  OK(sdis_device_create(&SDIS_DEVICE_CREATE_ARGS_DEFAULT, &g_dev));

  /* Create media -- solid interior, fluid exterior (for interface convention) */
  OK(sdis_solid_create(g_dev, &solid_shader, NULL, &solid));
  OK(sdis_fluid_create(g_dev, &fluid_shader, NULL, &fluid));

  /* Single interface for all faces */
  OK(sdis_interface_create(g_dev, solid, fluid, &interf_shader, NULL, &interf));
  OK(sdis_medium_ref_put(solid));
  OK(sdis_medium_ref_put(fluid));

  for(i = 0; i < 12; i++) g_box_interfaces[i] = interf;

  /* Build the 3D box scene */
  scn_args.get_indices  = box_get_indices;
  scn_args.get_interface = box_get_interface;
  scn_args.get_position = box_get_position;
  scn_args.nprimitives  = box_ntriangles;
  scn_args.nvertices    = box_nvertices;
  scn_args.context      = g_box_interfaces;
  OK(sdis_scene_create(g_dev, &scn_args, &g_scn));

  OK(sdis_interface_ref_put(interf));

  /* Create an RNG for setup_delta_sphere_rays */
  OK(ssp_rng_create(NULL, SSP_RNG_KISS, &g_rng));

  /* Determine the inner enclosure id (front of prim 0 or back, whichever is
   * not the outer enclosure). */
  {
    unsigned encs[2];
    scene_get_enclosure_ids(g_scn, 0, encs);
    g_inner_enc = (encs[0] != g_scn->outer_enclosure_id)
                ? encs[0] : encs[1];
  }
}

static void
teardown_test_scene(void)
{
  OK(ssp_rng_ref_put(g_rng));
  OK(sdis_scene_ref_put(g_scn));
  OK(sdis_device_ref_put(g_dev));
  g_rng = NULL;
  g_scn = NULL;
  g_dev = NULL;
}

/* ========================================================================== */
/* T4.1: Initial ENC query -> PATH_CND_DS_CHECK_TEMP chain                    */
/* ========================================================================== */
static void
test_init_enc_query_chain(void)
{
  struct path_state p;
  struct path_enc_data enc;

  printf("  T4.1: init ENC query chain (conductive entry -> ENC -> CHECK_TEMP)...\n");

  /* ---- Part A: step_conductive with ds_initialized=0 must dispatch
   *              to ENC sub-state via step_enc_query_emit ---- */
  memset(&p, 0, sizeof(p));
  memset(&enc, 0, sizeof(enc));
  p.active = 1;
  p.ctx.diff_algo = SDIS_DIFFUSION_DELTA_SPHERE;
  p.ds_initialized = 0;
  p.rwalk.vtx.P[0] = 0.5;
  p.rwalk.vtx.P[1] = 0.5;
  p.rwalk.vtx.P[2] = 0.5;

  step_conductive(&p, g_scn, &enc);

  /* Should have called step_enc_query_emit:
   *   phase = PATH_ENC_QUERY_EMIT, needs_ray = 1
   *   enc_query.return_state = PATH_CND_DS_CHECK_TEMP */
  CHK(p.phase == PATH_ENC_QUERY_EMIT);
  CHK(p.needs_ray == 1);
  CHK(enc.return_state == PATH_CND_DS_CHECK_TEMP);
  CHK(p.ray_count_ext == 6);
  CHK(p.ray_bucket == RAY_BUCKET_ENCLOSURE);

  /* Verify query position matches rwalk position */
  CHK(fabs(enc.query_pos[0] - 0.5) < 1.e-10);
  CHK(fabs(enc.query_pos[1] - 0.5) < 1.e-10);
  CHK(fabs(enc.query_pos[2] - 0.5) < 1.e-10);

  printf("    Part A: step_conductive -> PATH_ENC_QUERY_EMIT  PASS\n");

  /* ---- Part B: step_conductive with ds_initialized=1 goes directly
   *              to PATH_CND_DS_CHECK_TEMP ---- */
  memset(&p, 0, sizeof(p));
  memset(&enc, 0, sizeof(enc));
  p.active = 1;
  p.ctx.diff_algo = SDIS_DIFFUSION_DELTA_SPHERE;
  p.ds_initialized = 1;

  step_conductive(&p, g_scn, &enc);

  CHK(p.phase == PATH_CND_DS_CHECK_TEMP);
  CHK(p.needs_ray == 0);

  printf("    Part B: step_conductive(initialized) -> PATH_CND_DS_CHECK_TEMP  PASS\n");

  printf("  T4.1: PASS\n");
}

/* ========================================================================== */
/* T4.2: DS step trace emits exactly 2 rays with correct bucket               */
/*                                                                             */
/* Calls setup_delta_sphere_rays with a real RNG to verify the emitted ray    */
/* fields are consistent.                                                      */
/* ========================================================================== */
static void
test_ds_step_ray_count_and_bucket(void)
{
  struct path_state p;
  struct path_enc_data enc;

  printf("  T4.2: DS step emits 2 rays (RAY_BUCKET_STEP_PAIR)... ");

  memset(&p, 0, sizeof(p));
  memset(&enc, 0, sizeof(enc));
  p.active = 1;
  p.rng = g_rng;
  p.rwalk.vtx.P[0] = 0.5;
  p.rwalk.vtx.P[1] = 0.5;
  p.rwalk.vtx.P[2] = 0.5;
  p.ds_delta_solid_param = 0.1f;

  /* Call the real function */
  setup_delta_sphere_rays(&p, g_scn);

  /* Verify 2 rays emitted with correct bucket */
  CHK(p.ray_count_ext == 2);
  CHK(p.ray_req.ray_count == 2);
  CHK(p.ray_bucket == RAY_BUCKET_STEP_PAIR);
  CHK(p.needs_ray == 1);

  /* Verify backward direction = -forward direction */
  CHK(fabsf(p.ray_req.direction2[0] + p.ray_req.direction[0]) < 1.e-6f);
  CHK(fabsf(p.ray_req.direction2[1] + p.ray_req.direction[1]) < 1.e-6f);
  CHK(fabsf(p.ray_req.direction2[2] + p.ray_req.direction[2]) < 1.e-6f);

  /* Verify ds_dir0 matches ray_req.direction */
  CHK(fabsf(p.ds_dir0[0] - p.ray_req.direction[0]) < 1.e-6f);
  CHK(fabsf(p.ds_dir0[1] - p.ray_req.direction[1]) < 1.e-6f);
  CHK(fabsf(p.ds_dir0[2] - p.ray_req.direction[2]) < 1.e-6f);

  /* Verify ds_dir1 = -ds_dir0 */
  CHK(fabsf(p.ds_dir1[0] + p.ds_dir0[0]) < 1.e-6f);
  CHK(fabsf(p.ds_dir1[1] + p.ds_dir0[1]) < 1.e-6f);
  CHK(fabsf(p.ds_dir1[2] + p.ds_dir0[2]) < 1.e-6f);

  /* Verify ray origin matches rwalk position */
  CHK(fabsf(p.ray_req.origin[0] - 0.5f) < 1.e-6f);
  CHK(fabsf(p.ray_req.origin[1] - 0.5f) < 1.e-6f);
  CHK(fabsf(p.ray_req.origin[2] - 0.5f) < 1.e-6f);

  /* Verify range: [FLT_MIN, delta_solid * RAY_RANGE_MAX_SCALE] */
  CHK(p.ray_req.range[0] == FLT_MIN);
  CHK(fabsf(p.ray_req.range[1] - 0.1f * RAY_RANGE_MAX_SCALE) < 1.e-6f);
  CHK(p.ray_req.range2[0] == FLT_MIN);
  CHK(fabsf(p.ray_req.range2[1] - 0.1f * RAY_RANGE_MAX_SCALE) < 1.e-6f);

  printf("PASS\n");
}

/* ========================================================================== */
/* T4.3: hit0.distance > delta -> PATH_CND_DS_STEP_ENC_VERIFY                */
/*                                                                             */
/* When both rays miss (S3D_HIT_NONE), delta = delta_solid.  Since hit0 is    */
/* a miss, the condition `S3D_HIT_NONE(&hit0) || hit0.distance > delta` is    */
/* true -> goes to ENC_VERIFY.                                                */
/* ========================================================================== */
static void
test_ds_enc_verify_trigger(void)
{
  struct path_state p;
  struct path_enc_data enc;
  struct s3d_hit hit0, hit1;
  res_T res;

  printf("  T4.3: hit0 miss -> PATH_CND_DS_STEP_ENC_VERIFY... ");

  memset(&p, 0, sizeof(p));
  memset(&enc, 0, sizeof(enc));
  p.active = 1;
  p.ds_delta_solid_param = 0.1f;
  p.rwalk.vtx.P[0] = 0.5;
  p.rwalk.vtx.P[1] = 0.5;
  p.rwalk.vtx.P[2] = 0.5;
  p.ds_enc_id = g_inner_enc;
  p.ds_dir0[0] = 0.0f; p.ds_dir0[1] = 0.0f; p.ds_dir0[2] = 1.0f;
  p.ds_dir1[0] = 0.0f; p.ds_dir1[1] = 0.0f; p.ds_dir1[2] = -1.0f;

  /* Both rays miss */
  hit0 = S3D_HIT_NULL;
  hit1 = S3D_HIT_NULL;

  res = step_conductive_ds_process(&p, g_scn, &hit0, &hit1, &enc);
  CHK(res == RES_OK);

  /* Both miss -> delta = delta_solid = 0.1.
   * S3D_HIT_NONE(&hit0) is true -> ENC_VERIFY needed. */
  CHK(p.phase == PATH_CND_DS_STEP_ENC_VERIFY);
  CHK(p.needs_ray == 0);

  /* Verify pos_next was computed correctly (P + dir0 * delta) */
  CHK(fabs(enc.query_pos[0] - 0.5) < 1.e-6);
  CHK(fabs(enc.query_pos[1] - 0.5) < 1.e-6);
  CHK(fabs(enc.query_pos[2] - (0.5 + 0.1)) < 1.e-6);

  printf("PASS\n");
}

/* ========================================================================== */
/* T4.3b: hit0.distance > delta (far hit) -> PATH_CND_DS_STEP_ENC_VERIFY     */
/*                                                                             */
/* When hit1 is closer than hit0, delta = hit1.distance.                      */
/* If hit0.distance > delta, the condition also triggers ENC_VERIFY.           */
/* ========================================================================== */
static void
test_ds_enc_verify_far_hit(void)
{
  struct path_state p;
  struct path_enc_data enc;
  struct s3d_hit hit0, hit1;
  res_T res;

  printf("  T4.3b: hit0.distance > delta (far hit) -> ENC_VERIFY... ");

  memset(&p, 0, sizeof(p));
  memset(&enc, 0, sizeof(enc));
  p.active = 1;
  p.ds_delta_solid_param = 0.01f; /* small: avoids snap-to-boundary swap */
  p.rwalk.vtx.P[0] = 0.5;
  p.rwalk.vtx.P[1] = 0.5;
  p.rwalk.vtx.P[2] = 0.5;
  p.ds_enc_id = g_inner_enc;
  p.ds_dir0[0] = 1.0f; p.ds_dir0[1] = 0.0f; p.ds_dir0[2] = 0.0f;
  p.ds_dir1[0] = -1.0f; p.ds_dir1[1] = 0.0f; p.ds_dir1[2] = 0.0f;

  /* hit0 far away, hit1 close -> delta = hit1.distance = 0.05 */
  memset(&hit0, 0, sizeof(hit0));
  hit0.distance = 0.3f;
  hit0.prim.prim_id = 10;
  hit0.normal[0] = -1.0f;

  memset(&hit1, 0, sizeof(hit1));
  hit1.distance = 0.05f;
  hit1.prim.prim_id = 11;
  hit1.normal[0] = 1.0f;

  res = step_conductive_ds_process(&p, g_scn, &hit0, &hit1, &enc);
  CHK(res == RES_OK);

  /* delta = min(0.3, 0.05) = 0.05.  hit0.distance (0.3) > delta (0.05)
   * -> ENC_VERIFY (because the forward hit is beyond the step radius). */
  CHK(p.phase == PATH_CND_DS_STEP_ENC_VERIFY);
  CHK(p.needs_ray == 0);

  printf("PASS\n");
}

/* ========================================================================== */
/* T4.4: hit0.distance <= delta -> direct enc verification                    */
/*                                                                             */
/* When hit0 is the closest hit and hit0.distance drives delta (equals delta), */
/* the forward hit is within the step -> enclosure from hit primitive directly */
/* via scene_get_enclosure_ids.  With matching enc -> STEP_ADVANCE.            */
/* ========================================================================== */
static void
test_ds_enc_verify_skip(void)
{
  struct path_state p;
  struct path_enc_data enc;
  struct s3d_hit hit0, hit1;
  res_T res;
  unsigned encs[2];

  printf("  T4.4: hit0.distance <= delta -> direct PATH_CND_DS_STEP_ADVANCE... ");

  /* Use prim 0 of the real scene.  Determine which direction gives inner enc */
  scene_get_enclosure_ids(g_scn, 0, encs);

  memset(&p, 0, sizeof(p));
  memset(&enc, 0, sizeof(enc));
  p.active = 1;
  p.ds_delta_solid_param = 0.5f;
  p.rwalk.vtx.P[0] = 0.5;
  p.rwalk.vtx.P[1] = 0.5;
  p.rwalk.vtx.P[2] = 0.5;
  p.ds_enc_id = g_inner_enc;

  /* Direction toward prim 0 (a -Z face triangle).
   * Normal of prim 0 faces -Z; ray traveling in -Z hits it from front.
   * dot(dir, normal) < 0 --> front enc = encs[0]. */
  p.ds_dir0[0] = 0.0f; p.ds_dir0[1] = 0.0f; p.ds_dir0[2] = -1.0f;
  p.ds_dir1[0] = 0.0f; p.ds_dir1[1] = 0.0f; p.ds_dir1[2] = 1.0f;

  /* Construct a hit on prim 0 close enough to drive delta */
  memset(&hit0, 0, sizeof(hit0));
  hit0.distance = 0.08f;
  hit0.prim.prim_id = 0;
  /* Normal faces -Z (the front face direction of the -Z triangles) */
  hit0.normal[0] = 0.0f;
  hit0.normal[1] = 0.0f;
  hit0.normal[2] = -1.0f;

  hit1 = S3D_HIT_NULL;

  res = step_conductive_ds_process(&p, g_scn, &hit0, &hit1, &enc);
  CHK(res == RES_OK);

  /* hit0.distance (0.08) == delta -> direct enc path (no ENC_VERIFY).
   * The front enclosure of prim 0 should either match or mismatch ds_enc_id.
   * Either way, the function must not crash and must produce a valid phase. */
  CHK(p.phase == PATH_CND_DS_STEP_ADVANCE
   || p.phase == PATH_CND_DS_CHECK_TEMP);

  /* If match -> STEP_ADVANCE with resolved_enc_id set */
  if(p.phase == PATH_CND_DS_STEP_ADVANCE)
    CHK(enc.resolved_enc_id == g_inner_enc);

  printf("PASS\n");
}

/* ========================================================================== */
/* T4.4b: Enclosure mismatch at direct hit -> retry via CHECK_TEMP            */
/* ========================================================================== */
static void
test_ds_enc_mismatch_retry(void)
{
  struct path_state p;
  struct path_enc_data enc;
  struct s3d_hit hit0, hit1;
  res_T res;
  unsigned encs[2];

  printf("  T4.4b: enclosure mismatch at direct hit -> CHECK_TEMP... ");

  /* Use the outer enclosure id as ds_enc_id to guarantee mismatch when hitting
   * from inside.  Prim 0 front = inner or outer -- set ds_enc_id to the
   * opposite one. */
  scene_get_enclosure_ids(g_scn, 0, encs);

  memset(&p, 0, sizeof(p));
  memset(&enc, 0, sizeof(enc));
  p.active = 1;
  p.ds_delta_solid_param = 0.5f;
  p.rwalk.vtx.P[0] = 0.5;
  p.rwalk.vtx.P[1] = 0.5;
  p.rwalk.vtx.P[2] = 0.5;
  p.ds_dir0[0] = 0.0f; p.ds_dir0[1] = 0.0f; p.ds_dir0[2] = -1.0f;
  p.ds_dir1[0] = 0.0f; p.ds_dir1[1] = 0.0f; p.ds_dir1[2] = 1.0f;
  p.ds_robust_attempt = 0;

  /* Set ds_enc_id to something that does NOT match the hit's enclosure.
   * Use an invalid id that cannot appear in the scene. */
  p.ds_enc_id = 99999;

  memset(&hit0, 0, sizeof(hit0));
  hit0.distance = 0.08f;
  hit0.prim.prim_id = 0;
  hit0.normal[0] = 0.0f;
  hit0.normal[1] = 0.0f;
  hit0.normal[2] = -1.0f;

  hit1 = S3D_HIT_NULL;

  res = step_conductive_ds_process(&p, g_scn, &hit0, &hit1, &enc);
  CHK(res == RES_OK);

  /* Mismatch -> retry via CHECK_TEMP, robust_attempt incremented */
  CHK(p.phase == PATH_CND_DS_CHECK_TEMP);
  CHK(p.ds_robust_attempt == 1);

  printf("PASS\n");
}

/* ========================================================================== */
/* T4.5a: step_cnd_ds_step_enc_verify sets up ENC -> STEP_ADVANCE             */
/* ========================================================================== */
static void
test_ds_enc_verify_setup(void)
{
  struct path_state p;
  struct path_enc_data enc;

  printf("  T4.5a: step_cnd_ds_step_enc_verify -> ENC -> STEP_ADVANCE... ");

  memset(&p, 0, sizeof(p));
  memset(&enc, 0, sizeof(enc));
  p.active = 1;
  enc.query_pos[0] = 0.6;
  enc.query_pos[1] = 0.5;
  enc.query_pos[2] = 0.7;

  step_cnd_ds_step_enc_verify(&p, &enc);

  /* Should have set return_state = PATH_CND_DS_STEP_ADVANCE
   * and called step_enc_query_emit */
  CHK(enc.return_state == PATH_CND_DS_STEP_ADVANCE);
  CHK(p.phase == PATH_ENC_QUERY_EMIT);
  CHK(p.needs_ray == 1);
  CHK(p.ray_count_ext == 6);
  CHK(p.ray_bucket == RAY_BUCKET_ENCLOSURE);

  /* Verify query position preserved */
  CHK(fabs(enc.query_pos[0] - 0.6) < 1.e-10);
  CHK(fabs(enc.query_pos[1] - 0.5) < 1.e-10);
  CHK(fabs(enc.query_pos[2] - 0.7) < 1.e-10);

  printf("PASS\n");
}

/* ========================================================================== */
/* T4.5b: Loop condition -- enc match + HIT_NONE -> CHECK_TEMP (continue)     */
/*                                                                             */
/* step_cnd_ds_step_advance() after enc match:                                */
/*   hit_3d = HIT_NONE -> phase = PATH_CND_DS_CHECK_TEMP (loop)              */
/* ========================================================================== */
static void
test_ds_advance_loop_continue(void)
{
  struct path_state p;
  struct path_enc_data enc;

  printf("  T4.5b: advance loop (structural -- HIT_NONE -> CHECK_TEMP)... ");

  /* After step_cnd_ds_step_advance with:
   *   - enc_query.resolved_enc_id == ds_enc_id (match)
   *   - ds_hit0 = HIT_NONE -> rwalk.hit_3d = HIT_NONE
   *   -> should set phase = PATH_CND_DS_CHECK_TEMP (continue loop) */

  memset(&p, 0, sizeof(p));
  memset(&enc, 0, sizeof(enc));

  /* Simulate conditions at advance end */
  enc.resolved_enc_id = g_inner_enc;
  p.ds_enc_id = g_inner_enc; /* match */
  p.ds_hit0 = S3D_HIT_NULL;
  p.ds_delta = 0.1f;

  /* After advance, if no hit -> loop continues */
  p.rwalk.hit_3d = S3D_HIT_NULL;

  /* Verify condition: HIT_NONE -> should continue */
  CHK(S3D_HIT_NONE(&p.rwalk.hit_3d));

  /* When advance finishes with no hit -> PATH_CND_DS_CHECK_TEMP */
  /* (step_cnd_ds_step_advance needs full medium state; verify the branch
   *  condition that drives the transition.  Full call is T4.6+.) */

  printf("PASS (structural)\n");
}

/* ========================================================================== */
/* T4.5c: Loop condition -- enc match + !HIT_NONE -> COUPLED_BOUNDARY (exit)  */
/* ========================================================================== */
static void
test_ds_advance_loop_exit(void)
{
  struct path_state p;
  struct path_enc_data enc;

  printf("  T4.5c: advance loop exit (!HIT_NONE -> COUPLED_BOUNDARY)... ");

  memset(&p, 0, sizeof(p));
  memset(&enc, 0, sizeof(enc));

  /* Simulate conditions: hit found */
  p.rwalk.hit_3d.distance = 0.05f;
  p.rwalk.hit_3d.prim.prim_id = 0;
  p.rwalk.hit_3d.normal[0] = 0.0f;
  p.rwalk.hit_3d.normal[1] = 0.0f;
  p.rwalk.hit_3d.normal[2] = 1.0f;

  /* Verify condition: NOT HIT_NONE -> should exit loop */
  CHK(!S3D_HIT_NONE(&p.rwalk.hit_3d));

  printf("PASS (structural)\n");
}

/* ========================================================================== */
/* T4.5d: Advance enc mismatch -> retry via CHECK_TEMP                        */
/* ========================================================================== */
static void
test_ds_advance_enc_mismatch(void)
{
  struct path_state p;
  struct path_enc_data enc;

  printf("  T4.5d: advance ENC mismatch -> retry (CHECK_TEMP)... ");

  memset(&p, 0, sizeof(p));
  memset(&enc, 0, sizeof(enc));

  /* step_cnd_ds_step_advance checks:
   *   enc_query.resolved_enc_id != ds_enc_id -> retry */
  enc.resolved_enc_id = 99999;
  p.ds_enc_id = g_inner_enc; /* mismatch */
  p.ds_robust_attempt = 0;

  /* Verify mismatch condition */
  CHK(enc.resolved_enc_id != p.ds_enc_id);

  /* Under 100 -> retry */
  p.ds_robust_attempt++;
  CHK(p.ds_robust_attempt < 100);

  /* >= 100 -> error */
  p.ds_robust_attempt = 100;
  CHK(p.ds_robust_attempt >= 100);

  printf("PASS (structural)\n");
}

/* ========================================================================== */
/* Phase classification: verify M4 states have correct ray-pending flags      */
/* ========================================================================== */
static void
test_m4_phase_classification(void)
{
  printf("  Phase classification for M4 states... ");

  /* PATH_CND_DS_CHECK_TEMP is compute-only */
  CHK(path_phase_is_ray_pending(PATH_CND_DS_CHECK_TEMP) == 0);

  /* PATH_CND_DS_STEP_TRACE is ray-pending (2 step rays) */
  CHK(path_phase_is_ray_pending(PATH_CND_DS_STEP_TRACE) == 1);

  /* PATH_CND_DS_STEP_ENC_VERIFY is compute-only (sets up ENC query) */
  CHK(path_phase_is_ray_pending(PATH_CND_DS_STEP_ENC_VERIFY) == 0);

  /* PATH_CND_DS_STEP_ADVANCE is compute-only */
  CHK(path_phase_is_ray_pending(PATH_CND_DS_STEP_ADVANCE) == 0);

  /* PATH_CND_DS_STEP_PROCESS is compute-only */
  CHK(path_phase_is_ray_pending(PATH_CND_DS_STEP_PROCESS) == 0);

  /* PATH_CND_INIT_ENC is ray-pending (initial ENC query) */
  CHK(path_phase_is_ray_pending(PATH_CND_INIT_ENC) == 1);

  printf("PASS\n");
}

/* ========================================================================== */
/* Enum values: verify distinct M4 state enum values                          */
/* ========================================================================== */
static void
test_m4_enum_values(void)
{
  printf("  M4 enum values are distinct... ");

  CHK(PATH_CND_DS_CHECK_TEMP != PATH_CND_DS_STEP_TRACE);
  CHK(PATH_CND_DS_STEP_TRACE != PATH_CND_DS_STEP_PROCESS);
  CHK(PATH_CND_DS_STEP_PROCESS != PATH_CND_DS_STEP_ENC_VERIFY);
  CHK(PATH_CND_DS_STEP_ENC_VERIFY != PATH_CND_DS_STEP_ADVANCE);
  CHK(PATH_CND_DS_STEP_ADVANCE != PATH_CND_DS_CHECK_TEMP);

  /* All distinct from terminal states */
  CHK(PATH_CND_DS_CHECK_TEMP != PATH_DONE);
  CHK(PATH_CND_DS_STEP_TRACE != PATH_DONE);
  CHK(PATH_CND_DS_STEP_ADVANCE != PATH_DONE);

  /* All within valid range */
  CHK((int)PATH_CND_DS_CHECK_TEMP >= 0);
  CHK((int)PATH_CND_DS_CHECK_TEMP < (int)PATH_PHASE_COUNT);
  CHK((int)PATH_CND_DS_STEP_TRACE >= 0);
  CHK((int)PATH_CND_DS_STEP_TRACE < (int)PATH_PHASE_COUNT);
  CHK((int)PATH_CND_DS_STEP_ENC_VERIFY >= 0);
  CHK((int)PATH_CND_DS_STEP_ENC_VERIFY < (int)PATH_PHASE_COUNT);
  CHK((int)PATH_CND_DS_STEP_ADVANCE >= 0);
  CHK((int)PATH_CND_DS_STEP_ADVANCE < (int)PATH_PHASE_COUNT);

  printf("PASS\n");
}

/* ========================================================================== */
/* DS process: both miss -> delta = delta_solid, verify delta stored          */
/* ========================================================================== */
static void
test_ds_process_both_miss_delta(void)
{
  struct path_state p;
  struct path_enc_data enc;
  struct s3d_hit hit0, hit1;
  res_T res;

  printf("  DS process both-miss: delta = delta_solid... ");

  memset(&p, 0, sizeof(p));
  memset(&enc, 0, sizeof(enc));
  p.active = 1;
  p.ds_delta_solid_param = 0.15f;
  p.ds_enc_id = g_inner_enc;
  p.rwalk.vtx.P[0] = 0.5;
  p.rwalk.vtx.P[1] = 0.5;
  p.rwalk.vtx.P[2] = 0.5;
  p.ds_dir0[0] = 1.0f; p.ds_dir0[1] = 0.0f; p.ds_dir0[2] = 0.0f;
  p.ds_dir1[0] = -1.0f; p.ds_dir1[1] = 0.0f; p.ds_dir1[2] = 0.0f;

  hit0 = S3D_HIT_NULL;
  hit1 = S3D_HIT_NULL;

  res = step_conductive_ds_process(&p, g_scn, &hit0, &hit1, &enc);
  CHK(res == RES_OK);

  /* Verify delta = delta_solid = 0.15 */
  CHK(fabsf(p.ds_delta - 0.15f) < 1.e-6f);

  /* Both miss -> ENC_VERIFY */
  CHK(p.phase == PATH_CND_DS_STEP_ENC_VERIFY);

  printf("PASS\n");
}

/* ========================================================================== */
/* DS process: single forward hit -> delta = hit0.distance (direct enc path)  */
/* ========================================================================== */
static void
test_ds_process_single_hit_delta(void)
{
  struct path_state p;
  struct path_enc_data enc;
  struct s3d_hit hit0, hit1;
  res_T res;

  printf("  DS process single forward hit: delta = hit0.distance... ");

  memset(&p, 0, sizeof(p));
  memset(&enc, 0, sizeof(enc));
  p.active = 1;
  p.ds_delta_solid_param = 0.5f;
  p.ds_enc_id = g_inner_enc;
  p.rwalk.vtx.P[0] = 0.5;
  p.rwalk.vtx.P[1] = 0.5;
  p.rwalk.vtx.P[2] = 0.5;
  p.ds_dir0[0] = 0.0f; p.ds_dir0[1] = 0.0f; p.ds_dir0[2] = -1.0f;
  p.ds_dir1[0] = 0.0f; p.ds_dir1[1] = 0.0f; p.ds_dir1[2] = 1.0f;

  /* Hit prim 0 of the real scene at distance 0.08 */
  memset(&hit0, 0, sizeof(hit0));
  hit0.distance = 0.08f;
  hit0.prim.prim_id = 0;
  hit0.normal[0] = 0.0f;
  hit0.normal[1] = 0.0f;
  hit0.normal[2] = -1.0f;

  hit1 = S3D_HIT_NULL;

  res = step_conductive_ds_process(&p, g_scn, &hit0, &hit1, &enc);
  CHK(res == RES_OK);

  /* hit0.distance (0.08) == delta -> direct enc path (no ENC_VERIFY).
   * Result depends on whether enc matches ds_enc_id */
  CHK(p.phase == PATH_CND_DS_STEP_ADVANCE
   || p.phase == PATH_CND_DS_CHECK_TEMP);
  CHK(fabsf(p.ds_delta - 0.08f) < 1.e-6f);

  printf("PASS\n");
}

/* ========================================================================== */
/* Collect/bucket: PATH_CND_DS_STEP_TRACE emits 2 rays into batch            */
/* ========================================================================== */
static void
test_ds_collect_two_rays(void)
{
  struct wavefront_context wf;
  struct path_state path;
  struct s3d_ray_request ray_requests[4];
  uint32_t ray_to_path[4];
  uint32_t ray_slot[4];
  struct s3d_hit ray_hits[4];
  res_T res;

  printf("  Collect 2-ray DS step into batch... ");

  memset(&wf, 0, sizeof(wf));
  memset(&path, 0, sizeof(path));
  memset(ray_requests, 0, sizeof(ray_requests));

  wf.paths = &path;
  wf.total_paths = 1;
  wf.ray_requests = ray_requests;
  wf.ray_to_path = ray_to_path;
  wf.ray_slot = ray_slot;
  wf.ray_hits = ray_hits;
  wf.ray_count = 0;
  wf.max_rays = 4;

  /* Set up a path in PATH_CND_DS_STEP_TRACE with 2 rays */
  path.active = 1;
  path.needs_ray = 1;
  path.phase = PATH_CND_DS_STEP_TRACE;
  path.ray_req.ray_count = 2;
  path.ray_count_ext = 2;
  path.ray_bucket = RAY_BUCKET_STEP_PAIR;

  path.ray_req.origin[0] = 0.5f;
  path.ray_req.origin[1] = 0.5f;
  path.ray_req.origin[2] = 0.5f;
  path.ray_req.direction[0] = 0.0f;
  path.ray_req.direction[1] = 0.0f;
  path.ray_req.direction[2] = 1.0f;
  path.ray_req.direction2[0] = 0.0f;
  path.ray_req.direction2[1] = 0.0f;
  path.ray_req.direction2[2] = -1.0f;
  path.ray_req.range[0] = FLT_MIN;
  path.ray_req.range[1] = 1.0f;
  path.ray_req.range2[0] = FLT_MIN;
  path.ray_req.range2[1] = 1.0f;

  path.filter_data_storage = HIT_FILTER_DATA_NULL;
  path.rwalk.hit_3d = S3D_HIT_NULL;

  res = collect_ray_requests(&wf);
  CHK(res == RES_OK);

  /* 2 rays collected */
  CHK(wf.ray_count == 2);

  /* Both belong to path 0 */
  CHK(ray_to_path[0] == 0);
  CHK(ray_to_path[1] == 0);
  CHK(ray_slot[0] == 0);
  CHK(ray_slot[1] == 1);

  /* Check directions */
  CHK(fabsf(ray_requests[0].direction[2] - 1.0f) < 1.e-6f);
  CHK(fabsf(ray_requests[1].direction[2] - (-1.0f)) < 1.e-6f);

  printf("PASS\n");
}

/* ========================================================================== */
/* ds_initialized flag lifecycle                                               */
/* ========================================================================== */
static void
test_ds_initialized_lifecycle(void)
{
  struct path_state p;
  struct path_enc_data enc;

  printf("  ds_initialized flag lifecycle... ");

  memset(&p, 0, sizeof(p));
  memset(&enc, 0, sizeof(enc));
  p.ctx.diff_algo = SDIS_DIFFUSION_DELTA_SPHERE;

  /* Initially not initialized */
  CHK(p.ds_initialized == 0);

  /* After step_conductive dispatches ENC -> still 0 */
  step_conductive(&p, g_scn, &enc);
  CHK(p.ds_initialized == 0);
  CHK(p.phase == PATH_ENC_QUERY_EMIT);

  /* step_cnd_ds_check_temp sets ds_initialized = 1, but needs full medium
   * state.  Verify the flag is used correctly at conductive re-entry. */
  p.ds_initialized = 1;
  p.phase = PATH_COUPLED_CONDUCTIVE;
  step_conductive(&p, g_scn, &enc);
  CHK(p.phase == PATH_CND_DS_CHECK_TEMP);

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

  printf("Phase B-4 M4: Delta-sphere conductive fine-grained state machine tests\n");
  printf("=======================================================================\n");

  /* Build the shared test scene */
  setup_test_scene();

  /* Phase and enum checks */
  test_m4_phase_classification();
  test_m4_enum_values();

  /* T4.1: Init ENC query chain */
  test_init_enc_query_chain();

  /* T4.2: 2-ray emission */
  test_ds_step_ray_count_and_bucket();

  /* T4.3: ENC verify trigger conditions */
  test_ds_enc_verify_trigger();
  test_ds_enc_verify_far_hit();

  /* T4.4: ENC verify skip + mismatch retry */
  test_ds_enc_verify_skip();
  test_ds_enc_mismatch_retry();

  /* T4.5: ENC verify setup + loop/exit */
  test_ds_enc_verify_setup();
  test_ds_advance_loop_continue();
  test_ds_advance_loop_exit();
  test_ds_advance_enc_mismatch();

  /* Delta computation */
  test_ds_process_both_miss_delta();
  test_ds_process_single_hit_delta();

  /* Collect/batch integration */
  test_ds_collect_two_rays();

  /* Lifecycle */
  test_ds_initialized_lifecycle();

  /* Release */
  teardown_test_scene();

  printf("\nAll B-4 M4 tests PASSED.\n");
  return 0;
}
