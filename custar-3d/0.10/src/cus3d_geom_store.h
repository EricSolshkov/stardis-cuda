/**
 * @file cus3d_geom_store.h
 * @brief Geometry Data Store for custar-3d
 *
 * Maintains flattened GPU arrays of all scene primitives.
 * Replaces s3d_geometry and RTCGeometry.
 *
 * All triangles come first in the global primitive array, followed by all
 * spheres.  This layout lets the trace kernel distinguish types with a
 * single comparison (primID < total_tris).
 *
 *   primIDs [0 .. total_tris-1]           -> triangles
 *   primIDs [total_tris .. total_prims-1] -> spheres
 */

#ifndef CUS3D_GEOM_STORE_H
#define CUS3D_GEOM_STORE_H

#include "cus3d_device.h"
#include "cus3d_mem.h"
#include "cus3d_types.h"

#ifdef __cplusplus
extern "C" {
#endif

struct s3d_scene;
struct s3d_shape;

/* The real s3d_hit_filter_function_T lives in s3d.h; when building the
 * cuBQL modules in isolation we need a compatible forward declaration.  */
#ifndef S3D_H
typedef int (*s3d_hit_filter_function_T)(
    const struct s3d_hit*, const float*, const float*, const float*, void*, void*);
#endif

/**
 * Per-geometry host-side metadata.
 *
 * Each geom_entry corresponds to one s3d_shape (mesh or sphere) that has
 * been flattened into the global GPU arrays.  The prim_offset / prim_count
 * pair locates this shape's primitives in the unified table.
 */
struct geom_entry {
    uint32_t    shape_name;         /**< s3d shape ID (fid index) */
    prim_type   type;               /**< PRIM_TRIANGLE or PRIM_SPHERE */
    uint32_t    prim_offset;        /**< Start index in global prim array */
    uint32_t    prim_count;         /**< Number of primitives */
    uint32_t    vertex_offset;      /**< Start in vertex array (mesh only) */
    uint32_t    vertex_count;       /**< Number of vertices (mesh only) */
    int         is_enabled;
    int         flip_surface;
    /* Hit filter -- kept on host; GPU-side filter uses type-dispatch */
    s3d_hit_filter_function_T filter_func;
    void*       filter_data;
    struct s3d_shape* shape;        /**< Back-pointer for primitive queries */
};

/**
 * The geometry data store.
 *
 * Owns all GPU buffers for scene primitives and maintains host-side
 * metadata for CPU queries (primitive lookup, attribute access).
 */
struct cus3d_geom_store {
    /* ---- Host-side metadata ---- */
    struct geom_entry*   entries;
    size_t               entry_count;
    size_t               entry_capacity;

    uint32_t             total_tris;
    uint32_t             total_spheres;
    uint32_t             total_prims;      /* tris + spheres */
    uint32_t             total_verts;

    /* ---- GPU geometry data ---- */
    struct gpu_buffer_float3   d_vertices;  /**< All mesh vertices */
    struct gpu_buffer_uint3    d_indices;   /**< All mesh triangle indices */
    struct gpu_buffer_box3f    d_boxes;     /**< AABB for every primitive */

    /* Sphere data stored as sphere_gpu {cx,cy,cz,radius}.
     * Managed via raw cudaMalloc (simple POD, infrequent resize). */
    struct sphere_gpu*         d_spheres;
    size_t                     d_spheres_capacity;

    /* GPU lookup table: primID -> geom_entry index */
    struct gpu_buffer_uint32   d_prim_to_geom;

    /* GPU-side compact geometry metadata for kernels */
    struct geom_gpu_entry*     d_geom_entries;
    size_t                     d_geom_entries_capacity;

    /* ---- Dirty tracking ---- */
    int                  needs_rebuild;
};

/* ---- Lifecycle ---- */

res_T cus3d_geom_store_create(struct cus3d_geom_store** store);
void  cus3d_geom_store_destroy(struct cus3d_geom_store* store,
                               struct cus3d_device* dev);

/**
 * Full rebuild: iterate all shapes in @p scene, flatten their geometry
 * into contiguous GPU arrays, and upload.
 *
 * After this call the caller should invoke cus3d_geom_store_compute_bounds()
 * and then build the BVH.
 */
res_T cus3d_geom_store_sync(
    struct cus3d_geom_store* store,
    struct s3d_scene* scene,
    struct cus3d_device* dev);

/**
 * Launch GPU kernels to compute the AABB of every primitive.
 * Results are written to store->d_boxes.
 */
res_T cus3d_geom_store_compute_bounds(
    struct cus3d_geom_store* store,
    struct cus3d_device* dev);

/**
 * Given a global primID, return the corresponding geom_entry (host side).
 * Returns NULL if primID is out of range.
 */
const struct geom_entry* cus3d_geom_store_lookup(
    const struct cus3d_geom_store* store,
    uint32_t primID);

/**
 * Direct entry access by geom_idx (as returned in cus3d_hit_result::geom_idx).
 * Returns NULL if geom_idx is out of range.
 */
const struct geom_entry* cus3d_geom_store_get_entry(
    const struct cus3d_geom_store* store,
    uint32_t geom_idx);

#ifdef __cplusplus
}
#endif

#endif /* CUS3D_GEOM_STORE_H */
