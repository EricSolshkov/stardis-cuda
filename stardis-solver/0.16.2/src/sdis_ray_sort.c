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

/* sdis_ray_sort.c — Morton-code spatial + direction ray sorting
 *
 * Implements hybrid 30-bit Morton-code radix sort for GPU warp coherence:
 *   [31..5] = 27-bit origin Morton code  (9 bits/axis, 512^3 cells)
 *   [4..2]  = 3-bit direction octant     (sign(dx), sign(dy), sign(dz))
 *   [1..0]  = reserved (zero)
 *
 * Algorithm: 4-pass LSB radix sort (8-bit radix, uint32_t keys), O(N).
 * Estimated overhead for N=128K: ~0.11ms total (key + sort + permute + batch_idx).
 *
 * Reference: optimization/gpu_L2_pressure/ray_spatial_sort_design.md
 */

#include "sdis_ray_sort.h"
#include "sdis_solve_persistent_wavefront.h"
#include "sdis_scene_c.h"  /* sdis_scene -> dev for logging */
#include "sdis_log.h"

#include <star/s3d.h>  /* s3d_ray_request */

#include <stdlib.h>
#include <string.h>
#include <math.h>

/* ========================================================================
 * Morton-code encoding helpers
 * ======================================================================== */

/* Expand a 9-bit integer to a 27-bit Morton component via magic-number
 * bit interleave.  The result has 2 zero-bits between each original bit:
 *   bits x_8..x_0  →  ...00x_8 00x_7 00x_6 ... 00x_1 00x_0   (27 bits) */
static INLINE uint32_t
expand_bits_9(uint32_t v)
{
  v &= 0x1FFu;                          /* mask to 9 bits */
  v = (v | (v << 16)) & 0x010000FFu;
  v = (v | (v <<  8)) & 0x0100F00Fu;
  v = (v | (v <<  4)) & 0x010C30C3u;
  v = (v | (v <<  2)) & 0x09249249u;
  return v;
}

/* Compute 27-bit origin Morton code from 3 normalised [0,1] floats.
 * 9 bits/axis → 512^3 cells.  Inputs are clamped to [0, 511]. */
static INLINE uint32_t
morton_origin_27(float nx, float ny, float nz)
{
  uint32_t ix = (uint32_t)(fminf(fmaxf(nx * 512.0f, 0.0f), 511.0f));
  uint32_t iy = (uint32_t)(fminf(fmaxf(ny * 512.0f, 0.0f), 511.0f));
  uint32_t iz = (uint32_t)(fminf(fmaxf(nz * 512.0f, 0.0f), 511.0f));
  return (expand_bits_9(ix) << 2)
       | (expand_bits_9(iy) << 1)
       |  expand_bits_9(iz);
}

/* Encode direction into 3-bit octant.
 * sign(dx) << 2 | sign(dy) << 1 | sign(dz)
 *
 * This completely determines front-to-back child visit order at every
 * BVH internal node.  Rays in the same octant share identical
 * traversalStack ordering → zero warp divergence from direction. */
static INLINE uint32_t
encode_direction_octant(float dx, float dy, float dz)
{
  uint32_t oct = 0u;
  oct |= (dx >= 0.0f) ? 4u : 0u;
  oct |= (dy >= 0.0f) ? 2u : 0u;
  oct |= (dz >= 0.0f) ? 1u : 0u;
  return oct;  /* 3 bits [2..0] */
}

/* Compute full 32-bit hybrid sort key:
 *   [31..5] = 27-bit origin Morton code  (9 bits/axis, 512^3)
 *   [4..2]  = 3-bit direction octant
 *   [1..0]  = 0 (reserved) */
static INLINE uint32_t
ray_sort_key_32(float nx, float ny, float nz,
                float dx, float dy, float dz)
{
  uint32_t origin_key = morton_origin_27(nx, ny, nz);
  uint32_t oct_key    = encode_direction_octant(dx, dy, dz);
  return (origin_key << 5) | (oct_key << 2);
}

/* ========================================================================
 * Radix sort — 4-pass LSB, 8-bit radix, on ray_sort_entry {key, orig_idx}
 * ======================================================================== */

/* Sort entries[] in-place by key via 4-pass uint32_t radix sort.
 * temp[] must be at least N entries.  Ping-pong between entries and temp. */
static void
radix_sort_entries(struct ray_sort_entry* entries,
                   struct ray_sort_entry* temp,
                   size_t N)
{
  size_t pass;
  for(pass = 0; pass < 4; pass++) {
    int shift = (int)(pass * 8);
    size_t counts[256];
    size_t offsets[256];
    size_t k;

    memset(counts, 0, sizeof(counts));

    /* Histogram */
    for(k = 0; k < N; k++) {
      uint32_t digit = (entries[k].key >> shift) & 0xFFu;
      counts[digit]++;
    }

    /* Prefix sum */
    offsets[0] = 0;
    for(k = 1; k < 256; k++)
      offsets[k] = offsets[k - 1] + counts[k - 1];

    /* Scatter (stable) */
    for(k = 0; k < N; k++) {
      uint32_t digit = (entries[k].key >> shift) & 0xFFu;
      temp[offsets[digit]++] = entries[k];
    }

    /* Ping-pong: copy temp back to entries for next pass */
    memcpy(entries, temp, N * sizeof(*entries));
  }
}

/* ========================================================================
 * Permutation application and batch_idx update
 * ======================================================================== */

/* Apply the sorted permutation to ray_requests[], ray_to_slot[], ray_slot_sub[].
 *
 * After sorting, entries[i].orig_idx gives the original index of the ray
 * that should now occupy position i.  We permute all three arrays to match. */
static void
apply_ray_permutation(struct wavefront_pool* pool,
                      const struct ray_sort_entry* entries,
                      size_t N)
{
  size_t i;

  /* Use pre-allocated temporaries from pool if available,
   * otherwise fall back to malloc. */
  struct s3d_ray_request* tmp_rr = pool->sort_rr_tmp;
  uint32_t* tmp_r2s              = pool->sort_r2s_tmp;
  uint32_t* tmp_sub              = pool->sort_sub_tmp;
  int need_free = 0;

  if(!tmp_rr || !tmp_r2s || !tmp_sub) {
    tmp_rr  = (struct s3d_ray_request*)malloc(N * sizeof(*tmp_rr));
    tmp_r2s = (uint32_t*)malloc(N * sizeof(*tmp_r2s));
    tmp_sub = (uint32_t*)malloc(N * sizeof(*tmp_sub));
    need_free = 1;
    if(!tmp_rr || !tmp_r2s || !tmp_sub) {
      free(tmp_rr); free(tmp_r2s); free(tmp_sub);
      return; /* silent fail — only possible if pool alloc was skipped */
    }
  }

  /* Permute into temporaries */
  for(i = 0; i < N; i++) {
    uint32_t src = entries[i].orig_idx;
    tmp_rr[i]  = pool->ray_requests[src];
    tmp_r2s[i] = pool->ray_to_slot[src];
    tmp_sub[i] = pool->ray_slot_sub[src];
  }

  /* Copy back */
  memcpy(pool->ray_requests, tmp_rr,  N * sizeof(*tmp_rr));
  memcpy(pool->ray_to_slot,  tmp_r2s, N * sizeof(*tmp_r2s));
  memcpy(pool->ray_slot_sub, tmp_sub, N * sizeof(*tmp_sub));

  if(need_free) {
    free(tmp_rr);
    free(tmp_r2s);
    free(tmp_sub);
  }
}

/* Update batch_idx back-references stored in path_state.
 *
 * After permutation, ray_to_slot[new_idx] = slot_id gives us the slot that
 * requested this ray, and ray_slot_sub[new_idx] = sub tells us which sub-ray
 * it is (0=primary, 1=secondary, 0..5=enclosure).
 *
 * We write the new position back so that pool_distribute_ray_results() can
 * index ray_hits[] correctly via batch_idx. */
static void
update_batch_indices(struct wavefront_pool* pool, size_t N)
{
  size_t i;
  for(i = 0; i < N; i++) {
    uint32_t slot_id = pool->ray_to_slot[i];
    uint32_t sub     = pool->ray_slot_sub[i];
    struct path_state* p = &pool->slots[slot_id];

    if(p->phase == PATH_ENC_QUERY_EMIT && p->ray_count_ext == 6) {
      /* Enclosure 6-ray query:  batch_indices[0..5] map sub 0..5.
       * Also update batch_idx / batch_idx2 for sub 0 and 1. */
      p->enc_query.batch_indices[sub] = (uint32_t)i;
      if(sub == 0)
        p->ray_req.batch_idx = (uint32_t)i;
      else if(sub == 1)
        p->ray_req.batch_idx2 = (uint32_t)i;
    } else {
      /* Standard 1- or 2-ray request */
      if(sub == 0) {
        p->ray_req.batch_idx = (uint32_t)i;
      } else if(sub == 1) {
        p->ray_req.batch_idx2 = (uint32_t)i;
      }
    }
  }
}

/* ========================================================================
 * Pre-allocation / deallocation
 * ======================================================================== */

LOCAL_SYM res_T
pool_sort_alloc(struct wavefront_pool* pool)
{
  size_t max = pool->max_rays;

  pool->sort_entries = (struct ray_sort_entry*)
    malloc(max * sizeof(struct ray_sort_entry));
  pool->sort_temp = (struct ray_sort_entry*)
    malloc(max * sizeof(struct ray_sort_entry));
  pool->sort_rr_tmp = (struct s3d_ray_request*)
    malloc(max * sizeof(struct s3d_ray_request));
  pool->sort_r2s_tmp = (uint32_t*)malloc(max * sizeof(uint32_t));
  pool->sort_sub_tmp = (uint32_t*)malloc(max * sizeof(uint32_t));

  if(!pool->sort_entries || !pool->sort_temp
  || !pool->sort_rr_tmp  || !pool->sort_r2s_tmp
  || !pool->sort_sub_tmp) {
    pool_sort_free(pool);
    return RES_MEM_ERR;
  }

  pool->time_sort_s = 0.0;
  pool->total_sorted_rays = 0;

  return RES_OK;
}

LOCAL_SYM void
pool_sort_free(struct wavefront_pool* pool)
{
  free(pool->sort_entries);  pool->sort_entries = NULL;
  free(pool->sort_temp);     pool->sort_temp    = NULL;
  free(pool->sort_rr_tmp);   pool->sort_rr_tmp  = NULL;
  free(pool->sort_r2s_tmp);  pool->sort_r2s_tmp = NULL;
  free(pool->sort_sub_tmp);  pool->sort_sub_tmp = NULL;
}

/* ========================================================================
 * Main sort entry point
 * ======================================================================== */

LOCAL_SYM res_T
pool_sort_rays_by_morton(struct wavefront_pool* pool,
                         const float scene_lower[3],
                         const float scene_upper[3])
{
  const size_t N = pool->ray_count;
  size_t i;
  float inv_extent[3];
  struct ray_sort_entry* entries;
  struct ray_sort_entry* temp;

  /* Below warp size, sort has no GPU benefit */
  if(N <= 32) return RES_OK;

  /* 1. Compute scene extent inverse for normalisation */
  for(i = 0; i < 3; i++) {
    float ext = scene_upper[i] - scene_lower[i];
    inv_extent[i] = (ext > 1e-20f) ? 1.0f / ext : 0.0f;
  }

  /* 2. Get sort buffers (pre-allocated or fallback malloc) */
  entries = pool->sort_entries;
  temp    = pool->sort_temp;

  if(!entries || !temp) {
    entries = (struct ray_sort_entry*)malloc(N * sizeof(*entries));
    temp    = (struct ray_sort_entry*)malloc(N * sizeof(*temp));
    if(!entries || !temp) {
      free(entries); free(temp);
      return RES_MEM_ERR;
    }
  }

  /* 3. Compute hybrid sort keys (origin Morton + direction octant) */
  for(i = 0; i < N; i++) {
    const struct s3d_ray_request* rr = &pool->ray_requests[i];
    float nx = (rr->origin[0] - scene_lower[0]) * inv_extent[0];
    float ny = (rr->origin[1] - scene_lower[1]) * inv_extent[1];
    float nz = (rr->origin[2] - scene_lower[2]) * inv_extent[2];
    entries[i].key      = ray_sort_key_32(nx, ny, nz,
                                          rr->direction[0],
                                          rr->direction[1],
                                          rr->direction[2]);
    entries[i].orig_idx = (uint32_t)i;
  }

  /* 4. Radix sort by hybrid key (4-pass, 8-bit radix, O(N)) */
  radix_sort_entries(entries, temp, N);

  /* 4b. Diagnostic — verify sort effectiveness */
  {
    size_t moved = 0;
    size_t unique_keys = 0;
    uint32_t prev_key;
    size_t k;
    for(k = 0; k < N; k++) {
      if(entries[k].orig_idx != (uint32_t)k)
        moved++;
    }
    /* Count unique keys (entries are now sorted, so just check adjacency) */
    prev_key = entries[0].key;
    unique_keys = 1;
    for(k = 1; k < N; k++) {
      if(entries[k].key != prev_key) {
        unique_keys++;
        prev_key = entries[k].key;
      }
    }
    pool->sort_step_count++;
    /* Detailed log for first 5 steps, then every 100 */
    if(pool->scn && pool->scn->dev &&
       (pool->sort_step_count <= 5 ||
        pool->sort_step_count % 100 == 0)) {
      log_info(pool->scn->dev,
        "ray_sort[step %lu]: N=%lu moved=%lu(%.1f%%) "
        "unique_keys=%lu key_range=[0x%08X..0x%08X]\n",
        (unsigned long)pool->sort_step_count,
        (unsigned long)N,
        (unsigned long)moved,
        N > 0 ? 100.0 * (double)moved / (double)N : 0.0,
        (unsigned long)unique_keys,
        (unsigned)entries[0].key,
        (unsigned)entries[N - 1].key);
    }
  }

  /* 5. Apply permutation to ray_requests[], ray_to_slot[], ray_slot_sub[] */
  apply_ray_permutation(pool, entries, N);

  /* 6. Update batch_idx back-references in path_state */
  update_batch_indices(pool, N);

  /* 7. Accumulate statistics */
  pool->total_sorted_rays += N;

  /* Free fallback allocations (no-op if using pre-allocated buffers) */
  if(entries != pool->sort_entries) {
    free(entries);
    free(temp);
  }

  return RES_OK;
}
