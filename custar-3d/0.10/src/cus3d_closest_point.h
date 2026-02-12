/**
 * @file cus3d_closest_point.h
 * @brief GPU-accelerated Batch Closest Point Query for custar-3d
 *
 * Provides BVH-accelerated closest point queries on the GPU via
 * cuBQL::shrinkingRadiusQuery::forEachPrim.  Replaces the brute-force
 * CPU implementation in s3d_scene_view_closest_point.cpp with a
 * massively parallel GPU kernel for batch queries.
 *
 * Architecture mirrors cus3d_trace.h:
 *   - cus3d_cp_batch       : SoA GPU buffers for query positions/radii
 *   - cus3d_closest_point_batch : kernel launch + D2H download
 */

#ifndef CUS3D_CLOSEST_POINT_H
#define CUS3D_CLOSEST_POINT_H

#include "cus3d_device.h"
#include "cus3d_bvh.h"
#include "cus3d_geom_store.h"
#include "cus3d_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * GPU result of a closest point query.
 *
 * Similar to cus3d_hit_result but with closest_pos instead of ray fields.
 * distance stores the Euclidean distance (not squared) to match s3d_hit.
 */
struct cus3d_cp_result {
    int32_t   prim_id;       /**< Global primitive ID; -1 if no result */
    int32_t   geom_idx;      /**< Geometry entry index */
    int32_t   inst_id;       /**< Instance ID; -1 for non-instanced */
    float     distance;      /**< Euclidean distance to closest point */
    float     closest_pos[3];/**< World-space closest point on surface */
    float     normal[3];     /**< Geometric normal (un-normalized, CW convention) */
    float     uv[2];         /**< Barycentric coordinates on primitive */
};

/**
 * SoA GPU buffers for batch closest point queries.
 */
struct cus3d_cp_batch {
    struct gpu_buffer_float3  d_positions;   /**< Query points (N × float3) */
    float*                    d_radii;       /**< Search radii (N × float, device) */
    size_t                    d_radii_capacity;
    size_t                    count;         /**< Number of active queries */
};

/**
 * Allocate GPU buffers for up to @p max_queries closest point queries.
 */
res_T cus3d_cp_batch_create(struct cus3d_cp_batch* batch, size_t max_queries);

/**
 * Free GPU buffers.
 */
void cus3d_cp_batch_destroy(struct cus3d_cp_batch* batch);

/**
 * Execute a batch of closest point queries on the GPU.
 *
 * Launches the closest_point_kernel:
 *   - Single-level BVH: uses cuBQL::shrinkingRadiusQuery::forEachPrim
 *   - Two-level BVH (instanced): manual TLAS→BLAS traversal
 *
 * @param bvh       Built BVH (single or two-level)
 * @param store     Geometry data store
 * @param dev       CUDA device context
 * @param queries   GPU-side query batch (positions + radii already uploaded)
 * @param h_results Host-side output array (caller-allocated, size ≥ queries->count)
 * @return RES_OK on success
 */
res_T cus3d_closest_point_batch(
    const struct cus3d_bvh* bvh,
    const struct cus3d_geom_store* store,
    struct cus3d_device* dev,
    const struct cus3d_cp_batch* queries,
    struct cus3d_cp_result* h_results);

#ifdef __cplusplus
}
#endif

#endif /* CUS3D_CLOSEST_POINT_H */
