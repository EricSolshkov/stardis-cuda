/**
 * @file cus3d_bvh_internal.h
 * @brief Internal header exposing cus3d_bvh struct definition
 *
 * This header is for use by other cus3d CUDA modules (cus3d_trace.cu)
 * that need direct access to the cuBQL BinaryBVH stored inside
 * struct cus3d_bvh.  It must NOT be included from public headers or
 * from non-CUDA translation units.
 *
 * The struct definitions here must stay in sync with cus3d_bvh.cu.
 */

#ifndef CUS3D_BVH_INTERNAL_H
#define CUS3D_BVH_INTERNAL_H

#include <cuBQL/bvh.h>
#include "cus3d_bvh.h"
#include "cus3d_mem.h"

struct geometry; /* forward declaration (defined in s3d_geometry.h) */

/**
 * Per-instance child BVH data (internal).
 */
struct instance_bvh_entry {
    cuBQL::BinaryBVH<float, 3>  child_bvh;
    float                       transform[12];      /* 3x4 column-major */
    float                       inv_transform[12];
    uint32_t                    child_store_offset;
    uint32_t                    child_store_count;
    struct geometry*            inst_geom;          /* s3d geometry (GEOM_INSTANCE), NULL for pseudo-instance */
    int                         valid;              /* non-zero if built */
    int                         owns_child_bvh;     /* 1 if this entry owns child_bvh GPU memory */
    struct cus3d_geom_store*    child_store;         /* geometry store for this instance's prims */
};

/**
 * Full definition of the opaque cus3d_bvh struct.
 */
struct cus3d_bvh {
    /* Primary bottom-level BVH */
    cuBQL::BinaryBVH<float, 3>  bvh;
    int                         valid;

    /* Build configuration -- kept for potential rebuild */
    cuBQL::BuildConfig          config;

    /* Instance support (two-level traversal) */
    instance_bvh_entry*         instance_bvhs;
    size_t                      instance_count;
    size_t                      instance_capacity;

    cuBQL::BinaryBVH<float, 3>  tlas;       /* Top-level AS */
    int                         tlas_valid;

    /* TLAS compact index -> original instance_bvhs index mapping.
     * Built by cus3d_bvh_build_tlas; used by trace launch code and
     * cus3d_prim to resolve GPU hit inst_id (TLAS space) back to the
     * original instance_bvhs slot. */
    uint32_t*                   tlas_to_orig;
    uint32_t                    tlas_count;

    /* GPU buffer holding instance-level AABBs for TLAS build */
    struct gpu_buffer_box3f     d_instance_boxes;

    /* Cached scene AABB (host-side, copied from root node after build) */
    float                       lower[3];
    float                       upper[3];
};

#endif /* CUS3D_BVH_INTERNAL_H */
