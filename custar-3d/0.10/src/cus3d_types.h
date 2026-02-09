/**
 * @file cus3d_types.h
 * @brief Common type definitions for custar-3d GPU modules
 */

#ifndef CUS3D_TYPES_H
#define CUS3D_TYPES_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    PRIM_TRIANGLE = 0,
    PRIM_SPHERE   = 1
} prim_type;

typedef enum {
    CUS3D_BUILD_LOW    = 0,
    CUS3D_BUILD_MEDIUM = 1,
    CUS3D_BUILD_HIGH   = 2
} cus3d_build_quality;

struct box3f {
    float lower[3];
    float upper[3];
};

struct sphere_gpu {
    float cx, cy, cz;
    float radius;
};

struct cus3d_hit_result {
    int32_t   prim_id;
    int32_t   geom_idx;
    int32_t   inst_id;
    float     distance;
    float     normal[3];
    float     uv[2];
};

/**
 * Maximum number of hits returned by Top-K multi-hit trace.
 * K=8 provides ample margin: typical self-intersection avoidance
 * needs only 2 candidates, and each slot is ~40 bytes.
 */
#define CUS3D_MAX_MULTI_HITS 8

/**
 * Result of a Top-K multi-hit trace.
 * hits[0..count-1] are sorted by distance in ascending order.
 */
struct cus3d_multi_hit_result {
    int32_t   count;                                     /**< Actual hit count [0, K] */
    struct cus3d_hit_result hits[CUS3D_MAX_MULTI_HITS];  /**< Sorted by distance ascending */
};

struct geom_gpu_entry {
    uint32_t prim_offset;
    uint32_t prim_count;
    uint32_t vertex_offset;
    uint8_t  type;
    uint8_t  is_enabled;
    uint8_t  flip_surface;
    uint8_t  has_filter;
};

struct instance_gpu_data {
    float    transform[12];
    float    inv_transform[12];
    uint32_t child_bvh_idx;
    uint32_t geom_store_offset;
    /* Per-instance GPU array pointers (device memory).
     * Used by instanced trace kernels to access the correct geometry
     * store for each instance, supporting mixed direct+instanced scenes. */
    const void*  inst_vertices;     /* float3* on GPU */
    const void*  inst_indices;      /* uint3*  on GPU */
    const void*  inst_spheres;      /* struct sphere_gpu* on GPU */
    const void*  inst_prim_to_geom; /* unsigned int* on GPU */
    const void*  inst_geom_entries; /* struct geom_gpu_entry* on GPU */
    uint32_t     inst_tri_count;
    uint32_t     inst_total_prims;
};

#ifdef __cplusplus
}
#endif

#endif /* CUS3D_TYPES_H */
