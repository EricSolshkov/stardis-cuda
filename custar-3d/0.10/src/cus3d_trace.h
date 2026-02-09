/**
 * @file cus3d_trace.h
 * @brief Ray Tracing Engine for custar-3d
 * GPU-based ray tracing using cuBQL BVH traversal.
 * Replaces rtcIntersect1 and rtcIntersectArguments.
 */

#ifndef CUS3D_TRACE_H
#define CUS3D_TRACE_H

#include "cus3d_device.h"
#include "cus3d_bvh.h"
#include "cus3d_geom_store.h"
#include "cus3d_types.h"

#ifdef __cplusplus
extern "C" {
#endif

struct cus3d_ray_batch {
    struct gpu_buffer_float3   d_origins;
    struct gpu_buffer_float3   d_directions;
    struct gpu_buffer_float2   d_ranges;
    struct gpu_buffer_uint32   d_ray_data_offsets;
    size_t                     count;
};

res_T cus3d_trace_ray_batch(
    const struct cus3d_bvh* bvh,
    const struct cus3d_geom_store* store,
    struct cus3d_device* dev,
    const struct cus3d_ray_batch* rays,
    struct cus3d_hit_result* h_results);

res_T cus3d_trace_ray_single(
    const struct cus3d_bvh* bvh,
    const struct cus3d_geom_store* store,
    struct cus3d_device* dev,
    const float origin[3],
    const float direction[3],
    const float range[2],
    struct cus3d_hit_result* result);

/**
 * Trace a single ray and return up to @p max_hits nearest intersections
 * sorted by distance (ascending).  CPU-side filter code iterates
 * result->hits[0..count-1], accepting the first that passes the filter.
 *
 * @param max_hits  Desired count, clamped to CUS3D_MAX_MULTI_HITS.
 * @param result    Output: count + sorted hit array.
 */
res_T cus3d_trace_ray_single_multi(
    const struct cus3d_bvh* bvh,
    const struct cus3d_geom_store* store,
    struct cus3d_device* dev,
    const float origin[3],
    const float direction[3],
    const float range[2],
    int max_hits,
    struct cus3d_multi_hit_result* result);

res_T cus3d_ray_batch_create(struct cus3d_ray_batch* batch, size_t max_rays);
void  cus3d_ray_batch_destroy(struct cus3d_ray_batch* batch);

#ifdef __cplusplus
}
#endif

#endif /* CUS3D_TRACE_H */
