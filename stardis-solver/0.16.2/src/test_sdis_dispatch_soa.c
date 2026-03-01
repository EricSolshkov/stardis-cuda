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

/* P1: Dispatch SoA — unit tests.
 *
 * Tests:
 *   T1: alloc/free lifecycle
 *   T2: sync_from_path correctness
 *   T3: sync_to_path correctness
 *   T4: round-trip sync preserves all fields
 *
 * Note: compact_active_paths is static in the wavefront .c file, so
 *       integration tests for compact must live inside the existing
 *       B-4 test suite (e.g. test_sdis_b4_m2_ray_bucketing).
 */

#include "sdis_wf_soa.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

/* Minimal assertion macro for standalone test */
#define T_ASSERT(cond) do { \
  if(!(cond)) { \
    fprintf(stderr, "FAIL: %s:%d: %s\n", __FILE__, __LINE__, #cond); \
    return 1; \
  } \
} while(0)

#define T_ASSERT_EQ_INT(a, b) do { \
  int _a = (int)(a), _b = (int)(b); \
  if(_a != _b) { \
    fprintf(stderr, "FAIL: %s:%d: %s == %d, expected %d\n", \
            __FILE__, __LINE__, #a, _a, _b); \
    return 1; \
  } \
} while(0)

#define T_ASSERT_EQ_SIZE(a, b) do { \
  size_t _a = (size_t)(a), _b = (size_t)(b); \
  if(_a != _b) { \
    fprintf(stderr, "FAIL: %s:%d: %s == %llu, expected %llu\n", \
            __FILE__, __LINE__, #a, \
            (unsigned long long)_a, (unsigned long long)_b); \
    return 1; \
  } \
} while(0)

/*******************************************************************************
 * T1: alloc / free lifecycle
 ******************************************************************************/
static int test_alloc_free(void)
{
  struct dispatch_soa soa;
  res_T res;

  memset(&soa, 0, sizeof(soa));

  /* Allocate */
  res = dispatch_soa_alloc(&soa, 1024);
  T_ASSERT(res == RES_OK);
  T_ASSERT(soa.phase != NULL);
  T_ASSERT(soa.active != NULL);
  T_ASSERT(soa.needs_ray != NULL);
  T_ASSERT(soa.ray_bucket != NULL);
  T_ASSERT(soa.ray_count_ext != NULL);
  T_ASSERT_EQ_SIZE(soa.count, 1024);

  /* All zeroed by calloc */
  T_ASSERT_EQ_INT(soa.phase[0], PATH_INIT);
  T_ASSERT_EQ_INT(soa.active[0], 0);
  T_ASSERT_EQ_INT(soa.needs_ray[0], 0);
  T_ASSERT_EQ_INT(soa.ray_bucket[0], RAY_BUCKET_RADIATIVE);
  T_ASSERT_EQ_INT(soa.ray_count_ext[0], 0);

  /* Free */
  dispatch_soa_free(&soa);
  T_ASSERT(soa.phase == NULL);
  T_ASSERT(soa.active == NULL);
  T_ASSERT_EQ_SIZE(soa.count, 0);

  printf("  T1: alloc/free lifecycle             PASS\n");
  return 0;
}

/*******************************************************************************
 * T2: sync_from_path correctness
 ******************************************************************************/
static int test_sync_from_path(void)
{
  struct dispatch_soa soa;
  struct path_state p;
  res_T res;

  memset(&p, 0, sizeof(p));
  p.phase = PATH_RAD_TRACE_PENDING;
  p.active = 1;
  p.needs_ray = 1;
  p.ray_bucket = RAY_BUCKET_STEP_PAIR;
  p.ray_count_ext = 2;

  res = dispatch_soa_alloc(&soa, 8);
  T_ASSERT(res == RES_OK);

  dispatch_soa_sync_from_path(&soa, 3, &p);

  T_ASSERT_EQ_INT(soa.phase[3], PATH_RAD_TRACE_PENDING);
  T_ASSERT_EQ_INT(soa.active[3], 1);
  T_ASSERT_EQ_INT(soa.needs_ray[3], 1);
  T_ASSERT_EQ_INT(soa.ray_bucket[3], RAY_BUCKET_STEP_PAIR);
  T_ASSERT_EQ_INT(soa.ray_count_ext[3], 2);

  /* Other slots untouched */
  T_ASSERT_EQ_INT(soa.phase[0], PATH_INIT);
  T_ASSERT_EQ_INT(soa.active[0], 0);

  dispatch_soa_free(&soa);
  printf("  T2: sync_from_path correctness       PASS\n");
  return 0;
}

/*******************************************************************************
 * T3: sync_to_path correctness
 ******************************************************************************/
static int test_sync_to_path(void)
{
  struct dispatch_soa soa;
  struct path_state p;
  res_T res;

  memset(&p, 0, sizeof(p));

  res = dispatch_soa_alloc(&soa, 4);
  T_ASSERT(res == RES_OK);

  soa.phase[2] = PATH_COUPLED_COND_DS_PENDING;
  soa.active[2] = 1;
  soa.needs_ray[2] = 1;
  soa.ray_bucket[2] = RAY_BUCKET_SHADOW;
  soa.ray_count_ext[2] = 6;

  dispatch_soa_sync_to_path(&soa, 2, &p);

  T_ASSERT_EQ_INT(p.phase, PATH_COUPLED_COND_DS_PENDING);
  T_ASSERT_EQ_INT(p.active, 1);
  T_ASSERT_EQ_INT(p.needs_ray, 1);
  T_ASSERT_EQ_INT(p.ray_bucket, RAY_BUCKET_SHADOW);
  T_ASSERT_EQ_INT(p.ray_count_ext, 6);

  dispatch_soa_free(&soa);
  printf("  T3: sync_to_path correctness         PASS\n");
  return 0;
}

/*******************************************************************************
 * T4: round-trip sync preserves all fields
 ******************************************************************************/
static int test_round_trip(void)
{
  struct dispatch_soa soa;
  struct path_state p, p2;
  res_T res;

  memset(&p, 0, sizeof(p));
  memset(&p2, 0, sizeof(p2));
  p.phase = PATH_BND_SFN_COMPUTE_Ti_RESUME;
  p.active = 1;
  p.needs_ray = 0;
  p.ray_bucket = RAY_BUCKET_ENCLOSURE;
  p.ray_count_ext = 6;

  res = dispatch_soa_alloc(&soa, 2);
  T_ASSERT(res == RES_OK);

  /* path_state → SoA → path_state2 */
  dispatch_soa_sync_from_path(&soa, 1, &p);
  dispatch_soa_sync_to_path(&soa, 1, &p2);

  T_ASSERT_EQ_INT(p2.phase, PATH_BND_SFN_COMPUTE_Ti_RESUME);
  T_ASSERT_EQ_INT(p2.active, 1);
  T_ASSERT_EQ_INT(p2.needs_ray, 0);
  T_ASSERT_EQ_INT(p2.ray_bucket, RAY_BUCKET_ENCLOSURE);
  T_ASSERT_EQ_INT(p2.ray_count_ext, 6);

  dispatch_soa_free(&soa);
  printf("  T4: round-trip sync                  PASS\n");
  return 0;
}

/*******************************************************************************
 * Main
 ******************************************************************************/
int main(void)
{
  int fail = 0;

  printf("=== P1 Dispatch SoA Tests ===\n");
  fail |= test_alloc_free();
  fail |= test_sync_from_path();
  fail |= test_sync_to_path();
  fail |= test_round_trip();

  if(fail) {
    printf("SOME TESTS FAILED\n");
    return 1;
  }
  printf("ALL TESTS PASSED\n");
  return 0;
}
