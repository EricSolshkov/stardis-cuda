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

/* sdis_ray_sort.h — Morton-code spatial + direction ray sorting
 *
 * Sorts ray_requests[] by a hybrid 30-bit key:
 *   [31..5] = 27-bit origin Morton code  (9 bits/axis, 512^3 cells)
 *   [4..2]  = 3-bit direction octant     (sign(dx), sign(dy), sign(dz))
 *   [1..0]  = reserved (zero)
 *
 * Purpose: reduce L2 crossbar pressure by improving BVH traversal coherence
 * within GPU warps.  Adjacent 32 threads (one warp) that share similar
 * origins AND direction octants traverse overlapping BVH sub-trees with
 * identical front-to-back child ordering, minimising thread divergence and
 * L2 cache-line conflict.
 *
 * The sort is inserted between pool_collect_ray_requests_bucketed() and
 * s3d_scene_view_trace_rays_batch_ctx() in the wavefront main loop.
 *
 * Reference: optimization/gpu_L2_pressure/ray_spatial_sort_design.md
 */
#ifndef SDIS_RAY_SORT_H
#define SDIS_RAY_SORT_H

#include <rsys/rsys.h>
#include <stdint.h>
#include <stddef.h>

struct wavefront_pool;  /* forward declaration */

/* --------------------------------------------------------------------------
 * Sort key entry — pairs a 32-bit hybrid Morton key with its original index.
 * Used internally by radix sort; pre-allocated in the wavefront_pool to
 * avoid per-step malloc.
 * -------------------------------------------------------------------------- */
struct ray_sort_entry {
  uint32_t key;       /* 32-bit: origin Morton(27b) | octant(3b) | pad(2b) */
  uint32_t orig_idx;  /* original index in ray_requests[] before sort */
};

/* --------------------------------------------------------------------------
 * pool_sort_rays_by_morton — spatial+direction sort of ray_requests[]
 *
 * Sorts pool->ray_requests[0..ray_count-1] by a 30-bit hybrid Morton key
 * (27-bit origin + 3-bit direction octant).  All batch_idx / ray_to_slot /
 * ray_slot_sub mappings are updated to maintain correctness.
 *
 * scene_lower[3], scene_upper[3]: scene AABB for coordinate normalisation.
 *   Obtained once via s3d_scene_view_get_aabb() before the main loop.
 *
 * Pre-allocated scratch buffers are stored in the pool struct (see
 * pool_sort_alloc / pool_sort_free).  If those are NULL the function
 * falls back to internal malloc (slower, acceptable for testing).
 *
 * Returns RES_OK on success.
 * -------------------------------------------------------------------------- */
extern LOCAL_SYM res_T
pool_sort_rays_by_morton(struct wavefront_pool* pool,
                         const float scene_lower[3],
                         const float scene_upper[3]);

/* --------------------------------------------------------------------------
 * Pre-allocation / deallocation of sort scratch buffers.
 * Called from pool_create / pool_destroy.
 * -------------------------------------------------------------------------- */
extern LOCAL_SYM res_T
pool_sort_alloc(struct wavefront_pool* pool);

extern LOCAL_SYM void
pool_sort_free(struct wavefront_pool* pool);

#endif /* SDIS_RAY_SORT_H */
