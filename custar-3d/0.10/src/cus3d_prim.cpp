/**
 * @file cus3d_prim.cpp
 * @brief Primitive & Attribute Query -- convert GPU hit results to s3d types
 *
 * These are **CPU-side** operations.  After the GPU trace kernel writes a
 * cus3d_hit_result, the host calls these helpers to produce the
 * s3d_primitive / s3d_hit that the rest of the Star-3D code expects.
 *
 * Key mapping:
 *   cus3d_hit_result::prim_id   -> global primID in the unified table
 *   cus3d_hit_result::geom_idx  -> index into geom_store->entries[]
 *   geom_entry::shape           -> back-pointer to the s3d_shape
 *   geom_entry::prim_offset     -> subtracted to get a shape-local prim_id
 *
 * The s3d_primitive::shape__ field is set to the geom_entry::shape pointer.
 * Downstream code (s3d_primitive_get_attrib, etc.) casts shape__ and uses
 * the s3d_shape's data union to access mesh/sphere data for interpolation.
 */

#include "s3d.h"
#include "cus3d_prim.h"
#include "cus3d_bvh.h"
#include "s3d_geometry.h"

#include <float.h>

void
cus3d_hit_to_primitive(
    const struct cus3d_geom_store* store,
    const struct cus3d_bvh* bvh,
    const struct cus3d_hit_result* gpu_hit,
    struct s3d_primitive* prim)
{
    if (!store || !gpu_hit || !prim)
        return;

    if (gpu_hit->prim_id < 0) {
        *prim = S3D_PRIMITIVE_NULL;
        return;
    }

    /* Resolve the correct geometry store for this hit.
     * For instanced hits, geom_idx is relative to the child store;
     * for direct-geometry (or pseudo-instance) hits, it's the parent store. */
    const struct cus3d_geom_store* resolved_store = store;
    if (gpu_hit->inst_id >= 0) {
        const struct cus3d_geom_store* cs =
            cus3d_bvh_get_instance_store(bvh, (unsigned)gpu_hit->inst_id);
        if (cs)
            resolved_store = cs;
    }

    const struct geom_entry* ge = &resolved_store->entries[gpu_hit->geom_idx];

    /* prim_id is shape-local; scene_prim_id is the global unified-table ID */
    prim->prim_id       = gpu_hit->prim_id - (int)ge->prim_offset;
    prim->geom_id       = ge->shape_name;
    prim->scene_prim_id = (unsigned)gpu_hit->prim_id;
    prim->shape__       = ge->shape;

    if (gpu_hit->inst_id >= 0) {
        struct geometry* ig = cus3d_bvh_get_instance_geometry(
            bvh, (unsigned)gpu_hit->inst_id);
        prim->inst_id = ig ? ig->name : S3D_INVALID_ID;
        prim->inst__  = ig;
    } else {
        prim->inst_id = S3D_INVALID_ID;
        prim->inst__  = NULL;
    }
}

void
cus3d_hit_to_s3d_hit(
    const struct cus3d_geom_store* store,
    const struct cus3d_bvh* bvh,
    const struct cus3d_hit_result* gpu_hit,
    struct s3d_hit* hit)
{
    if (!store || !gpu_hit || !hit)
        return;

    /* Miss */
    if (gpu_hit->prim_id < 0) {
        *hit = S3D_HIT_NULL;
        return;
    }

    /* Populate the primitive sub-struct */
    cus3d_hit_to_primitive(store, bvh, gpu_hit, &hit->prim);

    /* Copy hit geometry from the GPU result */
    hit->distance  = gpu_hit->distance;
    hit->normal[0] = gpu_hit->normal[0];
    hit->normal[1] = gpu_hit->normal[1];
    hit->normal[2] = gpu_hit->normal[2];
    hit->uv[0]     = gpu_hit->uv[0];
    hit->uv[1]     = gpu_hit->uv[1];
}
