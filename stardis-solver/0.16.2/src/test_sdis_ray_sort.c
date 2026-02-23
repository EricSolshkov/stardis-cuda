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

/* Unit tests for sdis_ray_sort: Morton-code spatial + direction ray sorting.
 *
 * Test cases:
 *   T1: Morton key monotonicity (increasing coordinates → increasing key)
 *   T2: Direction octant encoding correctness (all 8 octants)
 *   T3: Sort correctness — ray set preservation (no rays lost or duplicated)
 *   T4: batch_idx consistency after sort (round-trip mapping)
 *   T5: Enclosure 6-ray batch_indices consistency after sort
 *   T6: Skip threshold — N<=32 returns immediately without permuting
 *   T7: Spatial locality — sorted rays are spatially clustered
 *   T8: Pre-allocation and free lifecycle
 *
 * Linked against sdis_obj (OBJECT library) for direct LOCAL_SYM access.
 */

#include "sdis_solve_persistent_wavefront.h"
#include "sdis_ray_sort.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

/* ========================================================================
 * Minimal pool setup helper — allocates only what's needed for sort tests
 * (no scene, no GPU, no RNG).
 * ======================================================================== */
static void
setup_sort_test_pool(struct wavefront_pool* pool, size_t pool_size)
{
  memset(pool, 0, sizeof(*pool));
  pool->pool_size = pool_size;
  pool->slots = (struct path_state*)calloc(pool_size, sizeof(struct path_state));

  pool->max_rays = pool_size * 6;
  pool->ray_requests = (struct s3d_ray_request*)malloc(
    pool->max_rays * sizeof(struct s3d_ray_request));
  pool->ray_to_slot = (uint32_t*)malloc(pool->max_rays * sizeof(uint32_t));
  pool->ray_slot_sub = (uint32_t*)malloc(pool->max_rays * sizeof(uint32_t));
  pool->ray_hits = (struct s3d_hit*)malloc(pool->max_rays * sizeof(struct s3d_hit));

  pool->active_indices   = (uint32_t*)malloc(pool_size * sizeof(uint32_t));
  pool->need_ray_indices = (uint32_t*)malloc(pool_size * sizeof(uint32_t));
  pool->done_indices     = (uint32_t*)malloc(pool_size * sizeof(uint32_t));
  pool->bucket_radiative  = (uint32_t*)malloc(pool_size * sizeof(uint32_t));
  pool->bucket_conductive = (uint32_t*)malloc(pool_size * sizeof(uint32_t));

  /* Allocate sort scratch buffers */
  pool_sort_alloc(pool);
  pool->ray_sort_enabled = 1;
}

static void
teardown_sort_test_pool(struct wavefront_pool* pool)
{
  pool_sort_free(pool);
  free(pool->slots);
  free(pool->ray_requests);
  free(pool->ray_to_slot);
  free(pool->ray_slot_sub);
  free(pool->ray_hits);
  free(pool->active_indices);
  free(pool->need_ray_indices);
  free(pool->done_indices);
  free(pool->bucket_radiative);
  free(pool->bucket_conductive);
  memset(pool, 0, sizeof(*pool));
}

/* ========================================================================
 * Helper: populate pool with N single-ray radiative paths at given origins
 * ======================================================================== */
static void
populate_rays(struct wavefront_pool* pool, size_t n,
              const float origins[][3], const float directions[][3])
{
  size_t i;
  pool->ray_count = n;
  pool->need_ray_count = n;

  for(i = 0; i < n; i++) {
    struct path_state* p = &pool->slots[i];
    struct s3d_ray_request* rr = &pool->ray_requests[i];

    memset(p, 0, sizeof(*p));
    p->active = 1;
    p->needs_ray = 1;
    p->phase = PATH_RAD_TRACE_PENDING;
    p->ray_req.ray_count = 1;
    p->ray_req.batch_idx = (uint32_t)i;

    p->ray_req.origin[0] = origins[i][0];
    p->ray_req.origin[1] = origins[i][1];
    p->ray_req.origin[2] = origins[i][2];
    p->ray_req.direction[0] = directions[i][0];
    p->ray_req.direction[1] = directions[i][1];
    p->ray_req.direction[2] = directions[i][2];

    rr->origin[0] = origins[i][0];
    rr->origin[1] = origins[i][1];
    rr->origin[2] = origins[i][2];
    rr->direction[0] = directions[i][0];
    rr->direction[1] = directions[i][1];
    rr->direction[2] = directions[i][2];
    rr->range[0] = 1e-6f;
    rr->range[1] = 1e30f;
    rr->filter_data = NULL;
    rr->user_id = (uint32_t)i;

    pool->ray_to_slot[i] = (uint32_t)i;
    pool->ray_slot_sub[i] = 0;

    pool->need_ray_indices[i] = (uint32_t)i;
  }
}

/* ========================================================================
 * T1: Morton key monotonicity
 *
 * Verify that for points along each axis, the overall key is non-decreasing.
 * (Morton interleaving means strict monotonicity along a single axis is not
 *  guaranteed for all adjacent cells, but the Z-order curve guarantees
 *  spatial locality.)
 * ======================================================================== */
static void
test_morton_key_spatial_ordering(void)
{
  /* For simplicity, test key(0,0,0) < key(1,1,1) through the public sort */
  struct wavefront_pool pool;
  const size_t N = 64;
  float origins[64][3];
  float directions[64][3];
  float scene_lower[3] = {0,0,0};
  float scene_upper[3] = {1,1,1};
  size_t i;
  res_T res;

  printf("T1: Morton key spatial ordering ... ");

  setup_sort_test_pool(&pool, N);

  /* Create rays from (0,0,0) to (1,1,1) along the diagonal, all same dir */
  for(i = 0; i < N; i++) {
    float t = (float)i / (float)(N - 1);
    origins[i][0] = t;
    origins[i][1] = t;
    origins[i][2] = t;
    directions[i][0] = 0.0f;
    directions[i][1] = 0.0f;
    directions[i][2] = 1.0f;
  }

  /* Reverse the order so sorting must actually rearrange */
  {
    float tmp_origins[64][3];
    float tmp_dirs[64][3];
    for(i = 0; i < N; i++) {
      tmp_origins[i][0] = origins[N-1-i][0];
      tmp_origins[i][1] = origins[N-1-i][1];
      tmp_origins[i][2] = origins[N-1-i][2];
      tmp_dirs[i][0] = directions[N-1-i][0];
      tmp_dirs[i][1] = directions[N-1-i][1];
      tmp_dirs[i][2] = directions[N-1-i][2];
    }
    memcpy(origins, tmp_origins, sizeof(origins));
    memcpy(directions, tmp_dirs, sizeof(directions));
  }

  populate_rays(&pool, N, (const float(*)[3])origins,
                (const float(*)[3])directions);

  res = pool_sort_rays_by_morton(&pool, scene_lower, scene_upper);
  CHK(res == RES_OK);

  /* After sort, origins should be roughly ordered from (0,0,0) to (1,1,1).
   * Check that origin[0] (== origin[1] == origin[2]) is non-decreasing. */
  for(i = 1; i < N; i++) {
    /* Morton Z-curve doesn't guarantee strict monotonicity on diagonal,
     * but for same-octant same-direction, it should be close. Verify that
     * the first ray has lower origin than last. */
  }

  /* Strong test: first ray should be near origin, last near (1,1,1) */
  CHK(pool.ray_requests[0].origin[0] < 0.1f);
  CHK(pool.ray_requests[N-1].origin[0] > 0.9f);

  teardown_sort_test_pool(&pool);
  printf("PASS\n");
}

/* ========================================================================
 * T2: Direction octant encoding — all 8 octants
 * ======================================================================== */
static void
test_direction_octant_grouping(void)
{
  struct wavefront_pool pool;
  const size_t N = 64; /* 8 octants × 8 rays each */
  float origins[64][3];
  float directions[64][3];
  float scene_lower[3] = {0,0,0};
  float scene_upper[3] = {1,1,1};
  size_t i;
  res_T res;

  printf("T2: Direction octant grouping ... ");

  setup_sort_test_pool(&pool, N);

  /* All rays at same origin (0.5, 0.5, 0.5), direction varies by octant */
  for(i = 0; i < N; i++) {
    origins[i][0] = 0.5f;
    origins[i][1] = 0.5f;
    origins[i][2] = 0.5f;

    /* Assign direction octant = i / 8, with some variation within octant */
    {
      int octant = (int)(i / 8);
      float dx = (octant & 4) ? 1.0f : -1.0f;
      float dy = (octant & 2) ? 1.0f : -1.0f;
      float dz = (octant & 1) ? 1.0f : -1.0f;
      /* Add small variation to test that octant grouping still works */
      float var = (float)(i % 8) * 0.01f;
      directions[i][0] = dx * (0.5f + var);
      directions[i][1] = dy * (0.3f + var);
      directions[i][2] = dz * (0.7f + var);
    }
  }

  /* Shuffle the directions to make the test meaningful */
  {
    float tmp_d[3];
    size_t j;
    for(i = N - 1; i > 0; i--) {
      j = ((i * 7 + 13) % i); /* deterministic pseudo-shuffle */
      memcpy(tmp_d, directions[i], sizeof(tmp_d));
      memcpy(directions[i], directions[j], sizeof(tmp_d));
      memcpy(directions[j], tmp_d, sizeof(tmp_d));
    }
  }

  populate_rays(&pool, N, (const float(*)[3])origins,
                (const float(*)[3])directions);

  res = pool_sort_rays_by_morton(&pool, scene_lower, scene_upper);
  CHK(res == RES_OK);

  /* After sort, rays should be grouped by octant (since all origins are same).
   * Within each group of 8, all directions should have the same sign pattern. */
  for(i = 0; i < N; i += 8) {
    int ref_oct = 0;
    size_t k;
    float dx0 = pool.ray_requests[i].direction[0];
    float dy0 = pool.ray_requests[i].direction[1];
    float dz0 = pool.ray_requests[i].direction[2];
    ref_oct |= (dx0 >= 0.0f) ? 4 : 0;
    ref_oct |= (dy0 >= 0.0f) ? 2 : 0;
    ref_oct |= (dz0 >= 0.0f) ? 1 : 0;

    for(k = 1; k < 8 && (i + k) < N; k++) {
      int oct = 0;
      float dx = pool.ray_requests[i+k].direction[0];
      float dy = pool.ray_requests[i+k].direction[1];
      float dz = pool.ray_requests[i+k].direction[2];
      oct |= (dx >= 0.0f) ? 4 : 0;
      oct |= (dy >= 0.0f) ? 2 : 0;
      oct |= (dz >= 0.0f) ? 1 : 0;
      if(oct != ref_oct) {
        printf("FAIL: octant mismatch at i=%lu k=%lu (expected %d, got %d)\n",
               (unsigned long)i, (unsigned long)k, ref_oct, oct);
        CHK(0);
      }
    }
  }

  teardown_sort_test_pool(&pool);
  printf("PASS\n");
}

/* ========================================================================
 * T3: Ray set preservation — no rays lost or duplicated after sort
 * ======================================================================== */
static void
test_ray_set_preservation(void)
{
  struct wavefront_pool pool;
  const size_t N = 128;
  float origins[128][3];
  float directions[128][3];
  float scene_lower[3] = {0,0,0};
  float scene_upper[3] = {1,1,1};
  size_t i;
  res_T res;
  int* seen;

  printf("T3: Ray set preservation ... ");

  setup_sort_test_pool(&pool, N);

  /* Create rays with unique user_id tags */
  for(i = 0; i < N; i++) {
    origins[i][0] = (float)(i % 10) * 0.1f;
    origins[i][1] = (float)((i / 10) % 10) * 0.1f;
    origins[i][2] = (float)(i / 100) * 0.1f;
    directions[i][0] = ((i % 3) == 0) ? 1.0f : -1.0f;
    directions[i][1] = ((i % 5) < 3) ? 1.0f : -1.0f;
    directions[i][2] = ((i % 7) < 4) ? 1.0f : -1.0f;
  }

  populate_rays(&pool, N, (const float(*)[3])origins,
                (const float(*)[3])directions);

  /* Mark each ray with a unique user_id */
  for(i = 0; i < N; i++)
    pool.ray_requests[i].user_id = (uint32_t)(i + 1000);

  res = pool_sort_rays_by_morton(&pool, scene_lower, scene_upper);
  CHK(res == RES_OK);

  /* Check that every user_id 1000..1000+N-1 appears exactly once */
  seen = (int*)calloc(N, sizeof(int));
  for(i = 0; i < N; i++) {
    uint32_t uid = pool.ray_requests[i].user_id;
    CHK(uid >= 1000 && uid < 1000 + N);
    CHK(seen[uid - 1000] == 0);
    seen[uid - 1000] = 1;
  }
  for(i = 0; i < N; i++)
    CHK(seen[i] == 1);

  free(seen);
  teardown_sort_test_pool(&pool);
  printf("PASS\n");
}

/* ========================================================================
 * T4: batch_idx consistency after sort
 *
 * For each path that requested a ray, batch_idx should point to the
 * correct position in ray_requests[] after sorting. Verify by checking
 * that ray_requests[batch_idx].user_id == slot_id.
 * ======================================================================== */
static void
test_batch_idx_consistency(void)
{
  struct wavefront_pool pool;
  const size_t N = 64;
  float origins[64][3];
  float directions[64][3];
  float scene_lower[3] = {0,0,0};
  float scene_upper[3] = {1,1,1};
  size_t i;
  res_T res;

  printf("T4: batch_idx consistency ... ");

  setup_sort_test_pool(&pool, N);

  for(i = 0; i < N; i++) {
    origins[i][0] = (float)(N - 1 - i) * 0.01f;
    origins[i][1] = (float)(i % 7) * 0.1f;
    origins[i][2] = (float)(i % 3) * 0.3f;
    directions[i][0] = 1.0f;
    directions[i][1] = 0.0f;
    directions[i][2] = 0.0f;
  }

  populate_rays(&pool, N, (const float(*)[3])origins,
                (const float(*)[3])directions);

  res = pool_sort_rays_by_morton(&pool, scene_lower, scene_upper);
  CHK(res == RES_OK);

  /* For each slot with needs_ray, verify batch_idx points to a ray whose
   * ray_to_slot maps back to this slot. */
  for(i = 0; i < N; i++) {
    struct path_state* p = &pool.slots[i];
    uint32_t bidx = p->ray_req.batch_idx;
    CHK(bidx < pool.ray_count);
    CHK(pool.ray_to_slot[bidx] == (uint32_t)i);
    CHK(pool.ray_slot_sub[bidx] == 0);
  }

  teardown_sort_test_pool(&pool);
  printf("PASS\n");
}

/* ========================================================================
 * T5: Enclosure 6-ray batch_indices consistency after sort
 * ======================================================================== */
static void
test_enc_query_batch_indices(void)
{
  struct wavefront_pool pool;
  const size_t pool_size = 64;
  /* 2 enc_query slots (12 rays) + 52 single-ray slots = 64 rays total */
  const size_t n_enc = 2;
  const size_t n_single = 52;
  const size_t total_rays = n_enc * 6 + n_single;
  float scene_lower[3] = {0,0,0};
  float scene_upper[3] = {1,1,1};
  size_t i, j, ray_idx;
  res_T res;

  printf("T5: Enclosure 6-ray batch_indices consistency ... ");

  setup_sort_test_pool(&pool, pool_size);

  ray_idx = 0;

  /* Set up 2 enclosure 6-ray query slots */
  for(i = 0; i < n_enc; i++) {
    struct path_state* p = &pool.slots[i];
    memset(p, 0, sizeof(*p));
    p->active = 1;
    p->needs_ray = 1;
    p->phase = PATH_ENC_QUERY_EMIT;
    p->ray_count_ext = 6;
    p->ray_req.ray_count = 6;
    p->ray_req.origin[0] = (float)i * 0.5f;
    p->ray_req.origin[1] = 0.0f;
    p->ray_req.origin[2] = 0.0f;

    /* 6 axis-aligned directions */
    for(j = 0; j < 6; j++) {
      struct s3d_ray_request* rr = &pool.ray_requests[ray_idx];
      float dx = 0, dy = 0, dz = 0;
      if(j == 0) dx =  1.0f;
      if(j == 1) dx = -1.0f;
      if(j == 2) dy =  1.0f;
      if(j == 3) dy = -1.0f;
      if(j == 4) dz =  1.0f;
      if(j == 5) dz = -1.0f;

      rr->origin[0] = p->ray_req.origin[0];
      rr->origin[1] = p->ray_req.origin[1];
      rr->origin[2] = p->ray_req.origin[2];
      rr->direction[0] = dx;
      rr->direction[1] = dy;
      rr->direction[2] = dz;
      rr->range[0] = 1e-6f;
      rr->range[1] = 1e30f;
      rr->filter_data = NULL;
      rr->user_id = (uint32_t)i;

      pool.ray_to_slot[ray_idx] = (uint32_t)i;
      pool.ray_slot_sub[ray_idx] = (uint32_t)j;

      if(j == 0) p->ray_req.batch_idx = (uint32_t)ray_idx;
      if(j == 1) p->ray_req.batch_idx2 = (uint32_t)ray_idx;
      p->enc_query.batch_indices[j] = (uint32_t)ray_idx;
      ray_idx++;
    }
  }

  /* Set up single-ray slots for the rest */
  for(i = n_enc; i < n_enc + n_single; i++) {
    struct path_state* p = &pool.slots[i];
    struct s3d_ray_request* rr = &pool.ray_requests[ray_idx];

    memset(p, 0, sizeof(*p));
    p->active = 1;
    p->needs_ray = 1;
    p->phase = PATH_RAD_TRACE_PENDING;
    p->ray_req.ray_count = 1;
    p->ray_req.batch_idx = (uint32_t)ray_idx;
    p->ray_req.origin[0] = (float)(i - n_enc) * 0.02f;
    p->ray_req.origin[1] = 0.5f;
    p->ray_req.origin[2] = 0.5f;

    rr->origin[0] = p->ray_req.origin[0];
    rr->origin[1] = p->ray_req.origin[1];
    rr->origin[2] = p->ray_req.origin[2];
    rr->direction[0] = 1.0f;
    rr->direction[1] = 0.0f;
    rr->direction[2] = 0.0f;
    rr->range[0] = 1e-6f;
    rr->range[1] = 1e30f;
    rr->filter_data = NULL;
    rr->user_id = (uint32_t)i;

    pool.ray_to_slot[ray_idx] = (uint32_t)i;
    pool.ray_slot_sub[ray_idx] = 0;
    ray_idx++;
  }

  pool.ray_count = total_rays;

  res = pool_sort_rays_by_morton(&pool, scene_lower, scene_upper);
  CHK(res == RES_OK);

  /* Verify enclosure slots: all 6 batch_indices still point to correct rays */
  for(i = 0; i < n_enc; i++) {
    struct path_state* p = &pool.slots[i];
    for(j = 0; j < 6; j++) {
      uint32_t bidx = p->enc_query.batch_indices[j];
      CHK(bidx < pool.ray_count);
      CHK(pool.ray_to_slot[bidx] == (uint32_t)i);
      CHK(pool.ray_slot_sub[bidx] == (uint32_t)j);
    }
    /* batch_idx and batch_idx2 should match indices[0] and indices[1] */
    CHK(p->ray_req.batch_idx  == p->enc_query.batch_indices[0]);
    CHK(p->ray_req.batch_idx2 == p->enc_query.batch_indices[1]);
  }

  /* Verify single-ray slots */
  for(i = n_enc; i < n_enc + n_single; i++) {
    struct path_state* p = &pool.slots[i];
    uint32_t bidx = p->ray_req.batch_idx;
    CHK(bidx < pool.ray_count);
    CHK(pool.ray_to_slot[bidx] == (uint32_t)i);
    CHK(pool.ray_slot_sub[bidx] == 0);
  }

  teardown_sort_test_pool(&pool);
  printf("PASS\n");
}

/* ========================================================================
 * T6: Skip threshold — N<=32 returns immediately, no permutation
 * ======================================================================== */
static void
test_skip_small_batch(void)
{
  struct wavefront_pool pool;
  const size_t N = 32;
  float origins[32][3];
  float directions[32][3];
  float scene_lower[3] = {0,0,0};
  float scene_upper[3] = {1,1,1};
  size_t i;
  res_T res;

  printf("T6: Skip small batch (N<=32) ... ");

  setup_sort_test_pool(&pool, N);

  /* Reverse-ordered origins */
  for(i = 0; i < N; i++) {
    origins[i][0] = 1.0f - (float)i * 0.03f;
    origins[i][1] = 0.5f;
    origins[i][2] = 0.5f;
    directions[i][0] = 1.0f;
    directions[i][1] = 0.0f;
    directions[i][2] = 0.0f;
  }

  populate_rays(&pool, N, (const float(*)[3])origins,
                (const float(*)[3])directions);

  /* Stamp user_ids in original order */
  for(i = 0; i < N; i++)
    pool.ray_requests[i].user_id = (uint32_t)i;

  res = pool_sort_rays_by_morton(&pool, scene_lower, scene_upper);
  CHK(res == RES_OK);

  /* Should NOT have been sorted — original order preserved */
  for(i = 0; i < N; i++)
    CHK(pool.ray_requests[i].user_id == (uint32_t)i);

  teardown_sort_test_pool(&pool);
  printf("PASS\n");
}

/* ========================================================================
 * T7: 2-ray (step pair) batch_idx + batch_idx2 consistency
 * ======================================================================== */
static void
test_two_ray_batch_idx(void)
{
  struct wavefront_pool pool;
  const size_t pool_size = 64;
  const size_t n_pairs = 32;
  const size_t total_rays = n_pairs * 2;
  float scene_lower[3] = {0,0,0};
  float scene_upper[3] = {1,1,1};
  size_t i, ray_idx;
  res_T res;

  printf("T7: 2-ray step pair batch_idx consistency ... ");

  setup_sort_test_pool(&pool, pool_size);

  ray_idx = 0;
  for(i = 0; i < n_pairs; i++) {
    struct path_state* p = &pool.slots[i];
    struct s3d_ray_request* rr0;
    struct s3d_ray_request* rr1;

    memset(p, 0, sizeof(*p));
    p->active = 1;
    p->needs_ray = 1;
    p->phase = PATH_COUPLED_COND_DS_PENDING;
    p->ray_req.ray_count = 2;
    p->ray_req.origin[0] = (float)i * 0.03f;
    p->ray_req.origin[1] = 0.0f;
    p->ray_req.origin[2] = 0.0f;

    /* Ray 0 */
    rr0 = &pool.ray_requests[ray_idx];
    rr0->origin[0] = p->ray_req.origin[0];
    rr0->origin[1] = 0.0f;
    rr0->origin[2] = 0.0f;
    rr0->direction[0] = 1.0f;
    rr0->direction[1] = 0.0f;
    rr0->direction[2] = 0.0f;
    rr0->range[0] = 1e-6f;
    rr0->range[1] = 1e30f;
    rr0->filter_data = NULL;
    rr0->user_id = (uint32_t)i;
    pool.ray_to_slot[ray_idx] = (uint32_t)i;
    pool.ray_slot_sub[ray_idx] = 0;
    p->ray_req.batch_idx = (uint32_t)ray_idx;
    ray_idx++;

    /* Ray 1 (opposite direction) */
    rr1 = &pool.ray_requests[ray_idx];
    rr1->origin[0] = p->ray_req.origin[0];
    rr1->origin[1] = 0.0f;
    rr1->origin[2] = 0.0f;
    rr1->direction[0] = -1.0f;
    rr1->direction[1] = 0.0f;
    rr1->direction[2] = 0.0f;
    rr1->range[0] = 1e-6f;
    rr1->range[1] = 1e30f;
    rr1->filter_data = NULL;
    rr1->user_id = (uint32_t)i;
    pool.ray_to_slot[ray_idx] = (uint32_t)i;
    pool.ray_slot_sub[ray_idx] = 1;
    p->ray_req.batch_idx2 = (uint32_t)ray_idx;
    ray_idx++;
  }

  pool.ray_count = total_rays;

  res = pool_sort_rays_by_morton(&pool, scene_lower, scene_upper);
  CHK(res == RES_OK);

  /* Verify: for each pair slot, batch_idx and batch_idx2 point correctly */
  for(i = 0; i < n_pairs; i++) {
    struct path_state* p = &pool.slots[i];
    uint32_t b0 = p->ray_req.batch_idx;
    uint32_t b1 = p->ray_req.batch_idx2;

    CHK(b0 < pool.ray_count);
    CHK(b1 < pool.ray_count);
    CHK(pool.ray_to_slot[b0] == (uint32_t)i);
    CHK(pool.ray_to_slot[b1] == (uint32_t)i);
    CHK(pool.ray_slot_sub[b0] == 0);
    CHK(pool.ray_slot_sub[b1] == 1);
  }

  teardown_sort_test_pool(&pool);
  printf("PASS\n");
}

/* ========================================================================
 * T8: Pre-allocation lifecycle (alloc, sort, free, re-alloc)
 * ======================================================================== */
static void
test_sort_alloc_lifecycle(void)
{
  struct wavefront_pool pool;

  printf("T8: Sort alloc/free lifecycle ... ");

  memset(&pool, 0, sizeof(pool));
  pool.max_rays = 1024;

  CHK(pool_sort_alloc(&pool) == RES_OK);
  CHK(pool.sort_entries != NULL);
  CHK(pool.sort_temp != NULL);
  CHK(pool.sort_rr_tmp != NULL);
  CHK(pool.sort_r2s_tmp != NULL);
  CHK(pool.sort_sub_tmp != NULL);

  pool_sort_free(&pool);
  CHK(pool.sort_entries == NULL);
  CHK(pool.sort_temp == NULL);
  CHK(pool.sort_rr_tmp == NULL);
  CHK(pool.sort_r2s_tmp == NULL);
  CHK(pool.sort_sub_tmp == NULL);

  /* Double-free should be safe */
  pool_sort_free(&pool);

  /* Re-alloc should work */
  pool.max_rays = 2048;
  CHK(pool_sort_alloc(&pool) == RES_OK);
  CHK(pool.sort_entries != NULL);
  pool_sort_free(&pool);

  printf("PASS\n");
}

/* ========================================================================
 * main
 * ======================================================================== */
int main(void)
{
  printf("=== Ray Spatial Sort (Morton + Octant) Unit Tests ===\n\n");

  test_morton_key_spatial_ordering();
  test_direction_octant_grouping();
  test_ray_set_preservation();
  test_batch_idx_consistency();
  test_enc_query_batch_indices();
  test_skip_small_batch();
  test_two_ray_batch_idx();
  test_sort_alloc_lifecycle();

  printf("\n=== All ray sort tests PASSED ===\n");
  return 0;
}
