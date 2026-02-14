/**
 * @file cus3d_find_enclosure.h
 * @brief GPU-accelerated Batch Point-in-Enclosure Query for custar-3d
 *
 * Phase B-4 M10: Replaces the CPU 6-ray enclosure query + brute-force
 * fallback with a GPU BVH-accelerated approach.
 *
 * For each query point P:
 *   1. BVH nearest-neighbor finds the closest primitive (prim_id +
 *      distance) for degenerate detection.
 *   2. If distance >= threshold, cast up to 6 PI/4-rotated axis-aligned
 *      rays through the BVH, using dot(ray_dir, hit_normal) to determine
 *      front/back side — exactly matching the CPU algorithm
 *      scene_get_enclosure_id_in_closed_boundaries().
 *   3. Early-exit on first valid ray hit (average ~1.5 rays).
 *
 * The 6-ray approach fixes the concave corner precision issue where the
 * original dot(P-Q, N) method would find the wrong nearest face at
 * concave geometry, producing incorrect enclosure IDs.
 *
 * Architecture mirrors cus3d_closest_point.h but outputs enclosure-
 * relevant data (prim_id + signed side) instead of full hit records.
 *
 * Shares the same BVH infrastructure as the batch closest point module.
 * M9 (WoS) and M10 both build on top of the BVH traversal kernels.
 */

#ifndef CUS3D_FIND_ENCLOSURE_H
#define CUS3D_FIND_ENCLOSURE_H

#include "cus3d_device.h"
#include "cus3d_bvh.h"
#include "cus3d_geom_store.h"
#include "cus3d_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * GPU result of a point-in-enclosure query.
 *
 * The kernel finds the closest primitive and determines which side of
 * that primitive the query point lies on.  The caller maps prim_id →
 * enclosure via the scene's prim_props table (front/back enc_ids).
 */
struct cus3d_enc_result {
    int32_t   prim_id;       /**< Global primitive ID; -1 if degenerate */
    int32_t   geom_idx;      /**< Geometry entry index */
    int32_t   inst_id;       /**< Instance ID; -1 for non-instanced */
    float     distance;      /**< Euclidean distance to closest point */
    float     normal[3];     /**< Geometric normal at closest point */
    float     closest_pos[3];/**< World-space closest point on surface */
    int32_t   side;          /**< 0 = front (P on outward side of N),
                                  1 = back  (P on inward side of N),
                                 -1 = degenerate (distance < threshold) */
};

/**
 * SoA GPU buffers for batch enclosure queries.
 *
 * Reuses the same gpu_buffer_float3 pattern as cus3d_cp_batch.
 * No search radius is needed — we use FLT_MAX as initial radius.
 */
struct cus3d_enc_batch {
    struct gpu_buffer_float3  d_positions;   /**< Query points (N × float3) */
    size_t                    count;         /**< Number of active queries */
};

/**
 * Allocate GPU buffers for up to @p max_queries enclosure queries.
 */
res_T cus3d_enc_batch_create(struct cus3d_enc_batch* batch, size_t max_queries);

/**
 * Free GPU buffers.
 */
void cus3d_enc_batch_destroy(struct cus3d_enc_batch* batch);

/**
 * Execute a batch of point-in-enclosure queries on the GPU.
 *
 * For each query point:
 *   1. BVH closest-primitive traversal (shrinkingRadiusQuery)
 *   2. Compute dot(P - Q, N) to determine front/back side
 *   3. Output prim_id + side for the caller to resolve enc_id
 *
 * @param bvh       Built BVH (single or two-level)
 * @param store     Geometry data store
 * @param dev       CUDA device context
 * @param queries   GPU-side query batch (positions already uploaded)
 * @param h_results Host-side output array (caller-allocated, size >= queries->count)
 * @return RES_OK on success
 */
res_T cus3d_find_enclosure_batch(
    const struct cus3d_bvh* bvh,
    const struct cus3d_geom_store* store,
    struct cus3d_device* dev,
    const struct cus3d_enc_batch* queries,
    struct cus3d_enc_result* h_results);

#ifdef __cplusplus
}
#endif

#endif /* CUS3D_FIND_ENCLOSURE_H */
