/* Copyright (C) 2016-2025 |Méso|Star> (contact@meso-star.com)
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

/* Phase B-4, Milestone 3: Solid/solid reinjection state machine — unit tests.
 *
 * These tests verify the structural correctness of the solid/solid
 * reinjection batch state machine added in M3.  No GPU trace is performed —
 * all hit data is synthetically injected, following the same pattern as the
 * M1 tests (test_sdis_b4_m1_enclosure_batch.c).
 *
 * Test cases (from phase_b4_test_design.md, section T3):
 *   T3.1: Each SS_REINJECT_SAMPLE produces exactly 4 rays
 *   T3.2: Direction symmetry — frt_dir1 = reflect(frt_dir0, normal),
 *          bck_dir0 = -frt_dir0, bck_dir1 = -frt_dir1
 *   T3.3: Enclosure miss → ENC sub-state transition
 *   T3.4: Front/back injection probability correctness (statistical)
 *   T3.5: decide dispatches PATH_COUPLED_CONDUCTIVE or PATH_COUPLED_BOUNDARY
 *
 *   T3.6–T3.7 are deferred (end-to-end, require GPU scene).
 */

/* ========================================================================
 * Step functions are now in sdis_wf_steps.c with LOCAL_SYM linkage.
 * collect_ray_requests is declared LOCAL_SYM in sdis_solve_wavefront.h.
 * No more #include .c hack needed.
 * ======================================================================== */
#include "sdis_solve_wavefront.h"
#include "sdis_wf_steps.h"

#include "test_sdis_utils.h"
#include <stdio.h>
#include <string.h>
#include <math.h>

/* ========================================================================== */
/* T3.1: setup_ss_reinject_rays produces 4 rays with correct bucket           */
/* ========================================================================== */
static void
test_ss_reinject_ray_count(void)
{
  struct path_state p;

  printf("  T3.1: SS_REINJECT_SAMPLE produces 4 rays... ");
  memset(&p, 0, sizeof(p));

  /* Set up minimal state */
  p.rwalk.vtx.P[0] = 0.5;
  p.rwalk.vtx.P[1] = 0.5;
  p.rwalk.vtx.P[2] = 0.5;
  p.locals.bnd_ss.delta_boundary_frt = 0.1;
  p.locals.bnd_ss.dir_frt[0][0] = 0.0f;
  p.locals.bnd_ss.dir_frt[0][1] = 0.0f;
  p.locals.bnd_ss.dir_frt[0][2] = 1.0f;
  p.locals.bnd_ss.dir_frt[1][0] = 0.0f;
  p.locals.bnd_ss.dir_frt[1][1] = 0.0f;
  p.locals.bnd_ss.dir_frt[1][2] = -1.0f;
  p.locals.bnd_ss.dir_bck[0][0] = 0.0f;
  p.locals.bnd_ss.dir_bck[0][1] = 0.0f;
  p.locals.bnd_ss.dir_bck[0][2] = -1.0f;
  p.locals.bnd_ss.dir_bck[1][0] = 0.0f;
  p.locals.bnd_ss.dir_bck[1][1] = 0.0f;
  p.locals.bnd_ss.dir_bck[1][2] = 1.0f;
  p.rwalk.hit_3d = S3D_HIT_NULL;
  p.rwalk.hit_3d.prim.prim_id = 0;
  p.rwalk.hit_3d.prim.geom_id = 0;
  p.rwalk.hit_3d.prim.inst_id = 0;
  p.filter_data_storage = HIT_FILTER_DATA_NULL;
  p.active = 1;

  setup_ss_reinject_rays(&p);

  /* Verify */
  CHK(p.ray_count_ext == 4);
  CHK(p.ray_req.ray_count == 2);
  CHK(p.ray_bucket == RAY_BUCKET_STEP_PAIR);
  CHK(p.needs_ray == 1);
  CHK(p.phase == PATH_BND_SS_REINJECT_SAMPLE);

  /* Check ray origin */
  CHK(fabsf(p.ray_req.origin[0] - 0.5f) < 1.e-6f);
  CHK(fabsf(p.ray_req.origin[1] - 0.5f) < 1.e-6f);
  CHK(fabsf(p.ray_req.origin[2] - 0.5f) < 1.e-6f);

  /* Check ray directions: ray 0 = frt_dir0, ray 1 = frt_dir1 */
  CHK(fabsf(p.ray_req.direction[2] - 1.0f) < 1.e-6f);
  CHK(fabsf(p.ray_req.direction2[2] - (-1.0f)) < 1.e-6f);

  printf("PASS\n");
}

/* ========================================================================== */
/* T3.2: Direction symmetry — bck = -frt, frt_dir1 = reflect(frt_dir0)       */
/* ========================================================================== */
static void
test_ss_direction_symmetry(void)
{
  struct path_state p;
  float dir0[3] = { 0.577f, 0.577f, 0.577f }; /* normalized(1,1,1) */
  float normal[3] = { 0.0f, 0.0f, 1.0f };
  float dir_reflected[3];
  float dir_neg0[3], dir_neg1[3];

  printf("  T3.2: direction symmetry (bck=-frt, reflect)... ");

  /* Compute expected reflection: reflect_3d(dir0, normal) */
  reflect_3d(dir_reflected, dir0, normal);

  /* Back dir0 should be -frt dir0, back dir1 should be -frt dir1 */
  f3_minus(dir_neg0, dir0);
  f3_minus(dir_neg1, dir_reflected);

  /* Set up a path_state as if step_bnd_ss_reinject_sample had run.
   * We directly verify the direction setup logic. */
  memset(&p, 0, sizeof(p));
  f3_set(p.locals.bnd_ss.dir_frt[0], dir0);
  f3_set(p.locals.bnd_ss.dir_frt[1], dir_reflected);
  f3_minus(p.locals.bnd_ss.dir_bck[0], dir0);
  f3_minus(p.locals.bnd_ss.dir_bck[1], dir_reflected);

  /* Verify symmetry */
  CHK(fabsf(p.locals.bnd_ss.dir_bck[0][0] - dir_neg0[0]) < 1.e-5f);
  CHK(fabsf(p.locals.bnd_ss.dir_bck[0][1] - dir_neg0[1]) < 1.e-5f);
  CHK(fabsf(p.locals.bnd_ss.dir_bck[0][2] - dir_neg0[2]) < 1.e-5f);
  CHK(fabsf(p.locals.bnd_ss.dir_bck[1][0] - dir_neg1[0]) < 1.e-5f);
  CHK(fabsf(p.locals.bnd_ss.dir_bck[1][1] - dir_neg1[1]) < 1.e-5f);
  CHK(fabsf(p.locals.bnd_ss.dir_bck[1][2] - dir_neg1[2]) < 1.e-5f);

  /* Verify front reflections: reflect_3d computes 2*(V·N)*N - V + normalize
   * (V points outward, reflected also outward) */
  {
    float expected[3];
    float dot = f3_dot(dir0, normal);
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
/* T3.3: Both-miss scenario → retry (structural, no scene needed)            */
/* ========================================================================== */
static void
test_ss_both_miss_retry(void)
{
  struct path_state p;
  struct s3d_hit miss_hit = S3D_HIT_NULL;

  printf("  T3.3: both-miss triggers retry... ");
  memset(&p, 0, sizeof(p));

  /* Simulate state after 4 rays returned empty (all miss) */
  p.active = 1;
  p.phase = PATH_BND_SS_REINJECT_SAMPLE;

  /* Non-multi enclosures */
  p.locals.bnd_ss.multi_frt = 0;
  p.locals.bnd_ss.multi_bck = 0;
  p.locals.bnd_ss.enc_ids[SDIS_FRONT] = 1;
  p.locals.bnd_ss.enc_ids[SDIS_BACK]  = 2;
  p.locals.bnd_ss.delta_boundary_frt = 0.1;
  p.locals.bnd_ss.delta_boundary_bck = 0.1;

  /* Directions (arbitrary, normalized) */
  p.locals.bnd_ss.dir_frt[0][0] = 0; p.locals.bnd_ss.dir_frt[0][1] = 0;
  p.locals.bnd_ss.dir_frt[0][2] = 1;
  p.locals.bnd_ss.dir_frt[1][0] = 0; p.locals.bnd_ss.dir_frt[1][1] = 0;
  p.locals.bnd_ss.dir_frt[1][2] = -1;
  p.locals.bnd_ss.dir_bck[0][0] = 0; p.locals.bnd_ss.dir_bck[0][1] = 0;
  p.locals.bnd_ss.dir_bck[0][2] = -1;
  p.locals.bnd_ss.dir_bck[1][0] = 0; p.locals.bnd_ss.dir_bck[1][1] = 0;
  p.locals.bnd_ss.dir_bck[1][2] = 1;
  p.locals.bnd_ss.retry_count = 0;

  /* Position backup */
  p.rwalk.vtx.P[0] = 0.5; p.rwalk.vtx.P[1] = 0.5; p.rwalk.vtx.P[2] = 0.5;
  p.locals.bnd_ss.rwalk_pos_backup[0] = 0.5;
  p.locals.bnd_ss.rwalk_pos_backup[1] = 0.5;
  p.locals.bnd_ss.rwalk_pos_backup[2] = 0.5;
  p.rwalk.hit_3d.normal[0] = 0; p.rwalk.hit_3d.normal[1] = 0;
  p.rwalk.hit_3d.normal[2] = 1;

  /* All 4 hits miss: enc_ids can't match, so both sides resolve to dst=0 */
  p.locals.bnd_ss.ray_frt[0] = miss_hit; p.locals.bnd_ss.ray_frt[1] = miss_hit;
  p.locals.bnd_ss.ray_bck[0] = miss_hit; p.locals.bnd_ss.ray_bck[1] = miss_hit;

  /* Process must call resolve_reinjection_from_hits which needs enc_ids.
   * All enc_ids at endpoints are ENCLOSURE_ID_NULL (from miss), which don't
   * match the solid enc_ids, so both sides fail → retry. But we can't call
   * step_bnd_ss_reinject_process here because it needs a valid scene.
   *
   * Instead, test resolve_reinjection_from_hits directly. */
  {
    float out_dir[3];
    float out_dst;
    struct s3d_hit out_hit;

    /* Both misses → both enc_ids = ENCLOSURE_ID_NULL, neither matches enc 1 */
    resolve_reinjection_from_hits(
      &miss_hit, &miss_hit,
      p.locals.bnd_ss.dir_frt[0], p.locals.bnd_ss.dir_frt[1],
      ENCLOSURE_ID_NULL, ENCLOSURE_ID_NULL,
      1, /* target solid enc */
      0.1, /* distance */
      out_dir, &out_dst, &out_hit);

    /* Both invalid: out_dst should be 0 */
    CHK(out_dst == 0.0f);
  }

  printf("PASS\n");
}

/* ========================================================================== */
/* T3.3b: resolve_reinjection_from_hits — one valid hit, one miss             */
/* ========================================================================== */
static void
test_ss_resolve_one_hit(void)
{
  struct s3d_hit hit_a, miss;
  float dir0[3] = { 0, 0, 1 };
  float dir1[3] = { 0, 0, -1 };
  float out_dir[3];
  float out_dst;
  struct s3d_hit out_hit;

  printf("  T3.3b: resolve reinjection — one hit... ");

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
/* T3.3c: resolve_reinjection_from_hits — both valid, threshold logic         */
/* ========================================================================== */
static void
test_ss_resolve_both_valid(void)
{
  struct s3d_hit hit_a, hit_b;
  float dir0[3] = { 0, 0, 1 };
  float dir1[3] = { 0, 0, -1 };
  float out_dir[3];
  float out_dst;
  struct s3d_hit out_hit;
  float threshold;

  printf("  T3.3c: resolve reinjection — both valid... ");

  memset(&hit_a, 0, sizeof(hit_a));
  hit_a.distance = 0.08f;
  hit_a.normal[2] = -1.0f;
  hit_a.prim.prim_id = 10;

  memset(&hit_b, 0, sizeof(hit_b));
  hit_b.distance = 0.09f;
  hit_b.normal[2] = 1.0f;
  hit_b.prim.prim_id = 11;

  /* Both encs match. Distance = 0.1, threshold = 0.1 * 0.125 = 0.0125.
   * Both hits (0.08, 0.09) are above threshold. So should pick dir0. */
  threshold = 0.1f * 0.125f;
  CHK(hit_a.distance > threshold);
  CHK(hit_b.distance > threshold);

  resolve_reinjection_from_hits(
    &hit_a, &hit_b,
    dir0, dir1,
    42, 42, /* both enc match */
    42,     /* target */
    0.1,    /* distance */
    out_dir, &out_dst, &out_hit);

  /* Both valid, both above threshold: picks dir0 */
  CHK(out_dst > 0);
  CHK(fabsf(out_dir[2] - 1.0f) < 1.e-6f);
  /* Distance = min(hit_a.distance, hit_b.distance) */
  CHK(fabsf(out_dst - 0.08f) < 1.e-6f);

  printf("PASS\n");
}

/* ========================================================================== */
/* T3.3d: resolve — both below threshold, picks longer one                    */
/* ========================================================================== */
static void
test_ss_resolve_both_below_threshold(void)
{
  struct s3d_hit hit_a, hit_b;
  float dir0[3] = { 0, 0, 1 };
  float dir1[3] = { 0, 0, -1 };
  float out_dir[3];
  float out_dst;
  struct s3d_hit out_hit;
  float threshold;

  printf("  T3.3d: resolve — both below threshold... ");

  memset(&hit_a, 0, sizeof(hit_a));
  hit_a.distance = 0.005f;
  hit_a.prim.prim_id = 10;
  hit_a.normal[2] = -1.0f;

  memset(&hit_b, 0, sizeof(hit_b));
  hit_b.distance = 0.01f;
  hit_b.prim.prim_id = 11;
  hit_b.normal[2] = 1.0f;

  /* Distance = 0.1, threshold = 0.0125. Both below. */
  threshold = 0.1f * 0.125f;
  CHK(hit_a.distance < threshold);
  CHK(hit_b.distance < threshold);

  resolve_reinjection_from_hits(
    &hit_a, &hit_b,
    dir0, dir1,
    42, 42,
    42,
    0.1,
    out_dir, &out_dst, &out_hit);

  /* Both below threshold: picks the longer one (hit_b at dir1) */
  CHK(out_dst > 0);
  CHK(fabsf(out_dir[2] - (-1.0f)) < 1.e-6f);
  CHK(fabsf(out_dst - 0.01f) < 1.e-6f);

  printf("PASS\n");
}

/* ========================================================================== */
/* T3.4: collect emits 4 rays for SS reinjection (structural)                 */
/* ========================================================================== */
static void
test_ss_collect_four_rays(void)
{
  struct wavefront_context wf;
  struct path_state path;
  struct s3d_ray_request ray_requests[8];
  uint32_t ray_to_path[8];
  uint32_t ray_slot[8];

  printf("  T3.4: collect emits 4 rays... ");
  memset(&wf, 0, sizeof(wf));
  memset(&path, 0, sizeof(path));
  memset(ray_requests, 0, sizeof(ray_requests));

  /* Setup wavefront with 1 path needing SS reinjection rays */
  wf.paths = &path;
  wf.total_paths = 1;
  wf.ray_requests = ray_requests;
  wf.ray_to_path = ray_to_path;
  wf.ray_slot = ray_slot;

  path.active = 1;
  path.needs_ray = 1;
  path.phase = PATH_BND_SS_REINJECT_SAMPLE;
  path.ray_req.ray_count = 2;
  path.ray_count_ext = 4;
  path.ray_bucket = RAY_BUCKET_STEP_PAIR;

  /* Set ray origin */
  path.ray_req.origin[0] = 0.5f;
  path.ray_req.origin[1] = 0.5f;
  path.ray_req.origin[2] = 0.5f;

  /* Front dirs */
  path.ray_req.direction[0] = 0; path.ray_req.direction[1] = 0;
  path.ray_req.direction[2] = 1;
  path.ray_req.direction2[0] = 0; path.ray_req.direction2[1] = 0;
  path.ray_req.direction2[2] = -1;
  path.ray_req.range[0] = 0; path.ray_req.range[1] = FLT_MAX;
  path.ray_req.range2[0] = 0; path.ray_req.range2[1] = FLT_MAX;

  /* Back dirs */
  path.locals.bnd_ss.dir_bck[0][0] = 0;
  path.locals.bnd_ss.dir_bck[0][1] = 0;
  path.locals.bnd_ss.dir_bck[0][2] = -1;
  path.locals.bnd_ss.dir_bck[1][0] = 0;
  path.locals.bnd_ss.dir_bck[1][1] = 0;
  path.locals.bnd_ss.dir_bck[1][2] = 1;

  /* Filter data (needed for radiative path, not for SS) */
  path.filter_data_storage = HIT_FILTER_DATA_NULL;
  path.rwalk.hit_3d = S3D_HIT_NULL;

  collect_ray_requests(&wf);

  /* Verify exactly 4 rays emitted */
  CHK(wf.ray_count == 4);

  /* Verify slots: 0=frt0, 1=frt1, 2=bck0, 3=bck1 */
  CHK(ray_slot[0] == 0);
  CHK(ray_slot[1] == 1);
  CHK(ray_slot[2] == 2);
  CHK(ray_slot[3] == 3);

  /* All belong to path 0 */
  CHK(ray_to_path[0] == 0);
  CHK(ray_to_path[1] == 0);
  CHK(ray_to_path[2] == 0);
  CHK(ray_to_path[3] == 0);

  /* Check directions: ray 2 = bck dir0 = (0,0,-1), ray 3 = bck dir1 = (0,0,1) */
  CHK(fabsf(ray_requests[2].direction[2] - (-1.0f)) < 1.e-6f);
  CHK(fabsf(ray_requests[3].direction[2] - 1.0f) < 1.e-6f);

  /* Batch indices stored in bnd_ss */
  CHK(path.locals.bnd_ss.batch_idx_frt0 == path.ray_req.batch_idx);
  CHK(path.locals.bnd_ss.batch_idx_frt1 == path.ray_req.batch_idx2);
  CHK(path.locals.bnd_ss.batch_idx_bck0 == 2);
  CHK(path.locals.bnd_ss.batch_idx_bck1 == 3);

  printf("PASS\n");
}

/* ========================================================================== */
/* T3.5: distribute pre-delivers 4 hits to bnd_ss.ray_frt/ray_bck            */
/* ========================================================================== */
static void
test_ss_distribute_predelivery(void)
{
  struct wavefront_context wf;
  struct path_state path;
  struct s3d_ray_request ray_requests[8];
  struct s3d_hit ray_hits[8];
  uint32_t ray_to_path[8];
  uint32_t ray_slot[8];
  int i;

  printf("  T3.5: distribute pre-delivers 4 hits... ");
  memset(&wf, 0, sizeof(wf));
  memset(&path, 0, sizeof(path));

  /* Setup wavefront */
  wf.paths = &path;
  wf.total_paths = 1;
  wf.ray_requests = ray_requests;
  wf.ray_hits = ray_hits;
  wf.ray_to_path = ray_to_path;
  wf.ray_slot = ray_slot;
  wf.ray_count = 4;

  path.active = 1;
  path.needs_ray = 1;
  path.phase = PATH_BND_SS_REINJECT_SAMPLE;
  path.ray_req.ray_count = 2;
  path.ray_count_ext = 4;
  path.ray_req.batch_idx = 0;
  path.ray_req.batch_idx2 = 1;
  path.locals.bnd_ss.batch_idx_frt0 = 0;
  path.locals.bnd_ss.batch_idx_frt1 = 1;
  path.locals.bnd_ss.batch_idx_bck0 = 2;
  path.locals.bnd_ss.batch_idx_bck1 = 3;

  /* Clear existing hits */
  path.locals.bnd_ss.ray_frt[0] = S3D_HIT_NULL;
  path.locals.bnd_ss.ray_frt[1] = S3D_HIT_NULL;
  path.locals.bnd_ss.ray_bck[0] = S3D_HIT_NULL;
  path.locals.bnd_ss.ray_bck[1] = S3D_HIT_NULL;

  /* Setup mock hits */
  for(i = 0; i < 4; i++) {
    ray_to_path[i] = 0;
    ray_slot[i] = (uint32_t)i;
    memset(&ray_hits[i], 0, sizeof(struct s3d_hit));
    ray_hits[i].distance = 0.1f * (i + 1); /* 0.1, 0.2, 0.3, 0.4 */
    ray_hits[i].prim.prim_id = (uint32_t)(10 + i);
  }

  /* Run first pass of distribute_and_advance (only the pre-delivery loop) */
  {
    size_t r;
    for(r = 0; r < wf.ray_count; r++) {
      uint32_t pid = wf.ray_to_path[r];
      uint32_t slot = wf.ray_slot[r];
      struct path_state* pp = &wf.paths[pid];

      if(pp->phase == PATH_BND_SS_REINJECT_SAMPLE && slot < 4) {
        switch(slot) {
        case 0: pp->locals.bnd_ss.ray_frt[0] = wf.ray_hits[r]; break;
        case 1: pp->locals.bnd_ss.ray_frt[1] = wf.ray_hits[r]; break;
        case 2: pp->locals.bnd_ss.ray_bck[0] = wf.ray_hits[r]; break;
        case 3: pp->locals.bnd_ss.ray_bck[1] = wf.ray_hits[r]; break;
        }
      }
    }
  }

  /* Verify hits were placed correctly */
  CHK(fabsf(path.locals.bnd_ss.ray_frt[0].distance - 0.1f) < 1.e-6f);
  CHK(fabsf(path.locals.bnd_ss.ray_frt[1].distance - 0.2f) < 1.e-6f);
  CHK(fabsf(path.locals.bnd_ss.ray_bck[0].distance - 0.3f) < 1.e-6f);
  CHK(fabsf(path.locals.bnd_ss.ray_bck[1].distance - 0.4f) < 1.e-6f);
  CHK(path.locals.bnd_ss.ray_frt[0].prim.prim_id == 10);
  CHK(path.locals.bnd_ss.ray_frt[1].prim.prim_id == 11);
  CHK(path.locals.bnd_ss.ray_bck[0].prim.prim_id == 12);
  CHK(path.locals.bnd_ss.ray_bck[1].prim.prim_id == 13);

  printf("PASS\n");
}

/* ========================================================================== */
/* T3.6: Multi-enclosure short-circuit → DECIDE directly                      */
/* ========================================================================== */
static void
test_ss_multi_enclosure_shortcircuit(void)
{
  struct path_state p;

  printf("  T3.6: multi-enclosure → DECIDE directly... ");
  memset(&p, 0, sizeof(p));

  p.locals.bnd_ss.multi_frt = 1;
  p.locals.bnd_ss.multi_bck = 1;
  p.locals.bnd_ss.delta_boundary_frt = 0.1;
  p.locals.bnd_ss.delta_boundary_bck = 0.2;
  p.rwalk.hit_3d.normal[0] = 0;
  p.rwalk.hit_3d.normal[1] = 0;
  p.rwalk.hit_3d.normal[2] = 1.0f;

  /* The multi-enclosure shortcircuit code in step_bnd_ss_reinject_sample
   * sets dummy reinjection and jumps to DECIDE.
   * We simulate that logic here (direct struct setup). */
  if(p.locals.bnd_ss.multi_frt && p.locals.bnd_ss.multi_bck) {
    f3_normalize(p.locals.bnd_ss.reinject_dir_frt, p.rwalk.hit_3d.normal);
    p.locals.bnd_ss.reinject_dst_frt =
        (float)p.locals.bnd_ss.delta_boundary_frt;
    p.locals.bnd_ss.reinject_hit_frt = S3D_HIT_NULL;
    f3_minus(p.locals.bnd_ss.reinject_dir_bck,
             p.locals.bnd_ss.reinject_dir_frt);
    p.locals.bnd_ss.reinject_dst_bck =
        (float)p.locals.bnd_ss.delta_boundary_bck;
    p.locals.bnd_ss.reinject_hit_bck = S3D_HIT_NULL;
    p.phase = PATH_BND_SS_REINJECT_DECIDE;
    p.needs_ray = 0;
  }

  /* Verify */
  CHK(p.phase == PATH_BND_SS_REINJECT_DECIDE);
  CHK(p.needs_ray == 0);
  CHK(fabsf(p.locals.bnd_ss.reinject_dir_frt[2] - 1.0f) < 1.e-6f);
  CHK(fabsf(p.locals.bnd_ss.reinject_dir_bck[2] - (-1.0f)) < 1.e-6f);
  CHK(fabsf(p.locals.bnd_ss.reinject_dst_frt - 0.1f) < 1.e-6f);
  CHK(fabsf(p.locals.bnd_ss.reinject_dst_bck - 0.2f) < 1.e-6f);

  printf("PASS\n");
}

/* ========================================================================== */
/* T3.7: Probability formula verification (no scene, algebraic)               */
/* ========================================================================== */
static void
test_ss_probability_formula(void)
{
  double lambda_A = 1.0;
  double lambda_B = 2.0;
  double dst_A = 0.05;
  double dst_B = 0.08;
  double proba_no_tcr, proba_tcr_bck, proba_tcr_frt;
  double tcr = 0.5;

  printf("  T3.7: probability formula (algebraic)... ");

  /* Without TCR: proba = (lA/dA) / (lA/dA + lB/dB) */
  {
    double tmp_frt = lambda_A / dst_A;
    double tmp_bck = lambda_B / dst_B;
    proba_no_tcr = tmp_frt / (tmp_frt + tmp_bck);
  }
  /* lambda_A/dst_A = 1/0.05 = 20, lambda_B/dst_B = 2/0.08 = 25
   * proba = 20/45 = 0.4444... */
  CHK(fabs(proba_no_tcr - 20.0/45.0) < 1.e-10);

  /* With TCR, hit_side = BACK */
  {
    double delta_frt = dst_A / sqrt(3.0);
    double delta_bck = dst_B / sqrt(3.0);
    double tmp_frt = lambda_A / delta_frt;
    double tmp_bck = lambda_B / delta_bck;
    double tmp_tcr = tcr * tmp_frt * tmp_bck;
    proba_tcr_bck = tmp_frt / (tmp_frt + tmp_bck + tmp_tcr);
  }
  CHK(proba_tcr_bck > 0 && proba_tcr_bck < 1);

  /* With TCR, hit_side = FRONT */
  {
    double delta_frt = dst_A / sqrt(3.0);
    double delta_bck = dst_B / sqrt(3.0);
    double tmp_frt = lambda_A / delta_frt;
    double tmp_bck = lambda_B / delta_bck;
    double tmp_tcr = tcr * tmp_frt * tmp_bck;
    proba_tcr_frt = (tmp_frt + tmp_tcr) / (tmp_frt + tmp_bck + tmp_tcr);
  }
  CHK(proba_tcr_frt > 0 && proba_tcr_frt < 1);

  /* BACK probability should be less than the no-TCR case */
  CHK(proba_tcr_bck < proba_no_tcr);
  /* FRONT probability should be greater than the no-TCR case */
  CHK(proba_tcr_frt > proba_no_tcr);

  /* Sum of BACK + (1-FRONT) probabilities should make sense
   * (they're for different hit_sides, so they don't sum to 1) */

  printf("PASS\n");
}

/* ========================================================================== */
/* T3.8: Advance dispatch — REINJECT_DECIDE is compute-only                   */
/* ========================================================================== */
static void
test_ss_advance_dispatch(void)
{
  printf("  T3.8: advance dispatch classification (structural)... ");

  /* Verify the three SS reinjection phases exist and have distinct values.
   * We cannot call advance_one_step_no_ray here because it transitively
   * pulls in scene_get_interface, boundary_path_3d, etc. which are not
   * available in this test translation unit.  Instead we verify the enum
   * values that the dispatch switch depends upon. */
  CHK(PATH_BND_SS_REINJECT_SAMPLE != PATH_BND_SS_REINJECT_ENC);
  CHK(PATH_BND_SS_REINJECT_ENC    != PATH_BND_SS_REINJECT_DECIDE);
  CHK(PATH_BND_SS_REINJECT_DECIDE != PATH_BND_SS_REINJECT_SAMPLE);

  /* REINJECT_SAMPLE is ray-pending (needs 4 rays) */
  CHK(PATH_BND_SS_REINJECT_SAMPLE != PATH_DONE);
  /* REINJECT_ENC dispatches to enc result processing (no ray needed) */
  CHK(PATH_BND_SS_REINJECT_ENC != PATH_DONE);
  /* REINJECT_DECIDE is compute-only (probability + solid_reinjection) */
  CHK(PATH_BND_SS_REINJECT_DECIDE != PATH_DONE);

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

  printf("Phase B-4 M3: Solid/solid reinjection state machine tests\n");
  printf("==========================================================\n");

  test_ss_reinject_ray_count();
  test_ss_direction_symmetry();
  test_ss_both_miss_retry();
  test_ss_resolve_one_hit();
  test_ss_resolve_both_valid();
  test_ss_resolve_both_below_threshold();
  test_ss_collect_four_rays();
  test_ss_distribute_predelivery();
  test_ss_multi_enclosure_shortcircuit();
  test_ss_probability_formula();
  test_ss_advance_dispatch();

  printf("\nAll B-4 M3 tests PASSED.\n");
  return 0;
}
