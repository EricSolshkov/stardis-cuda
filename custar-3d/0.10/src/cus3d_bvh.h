/**
 * @file cus3d_bvh.h
 * @brief BVH Manager for custar-3d using cuBQL
 * Replaces rtcNewScene, rtcCommitScene, and rtcGetSceneBounds.
 */

#ifndef CUS3D_BVH_H
#define CUS3D_BVH_H

#include "cus3d_device.h"
#include "cus3d_geom_store.h"
#include "cus3d_types.h"

#ifdef __cplusplus
extern "C" {
#endif

struct cus3d_bvh;

res_T cus3d_bvh_create(struct cus3d_bvh** bvh);
void  cus3d_bvh_destroy(struct cus3d_bvh* bvh, struct cus3d_device* dev);

res_T cus3d_bvh_build(
    struct cus3d_bvh* bvh,
    const struct cus3d_geom_store* store,
    struct cus3d_device* dev,
    cus3d_build_quality quality);

res_T cus3d_bvh_build_instance(
    struct cus3d_bvh* bvh,
    uint32_t instance_idx,
    const struct cus3d_geom_store* child_store,
    struct cus3d_device* dev);

res_T cus3d_bvh_build_tlas(
    struct cus3d_bvh* bvh,
    struct cus3d_device* dev);

/**
 * Set the 3x4 column-major forward and inverse transforms for an instance.
 * Must be called after cus3d_bvh_build_instance and before cus3d_bvh_build_tlas.
 */
res_T cus3d_bvh_set_instance_transform(
    struct cus3d_bvh* bvh,
    uint32_t instance_idx,
    const float forward[12],
    const float inverse[12]);

void cus3d_bvh_get_bounds(
    const struct cus3d_bvh* bvh,
    float lower[3],
    float upper[3]);

int cus3d_bvh_is_valid(const struct cus3d_bvh* bvh);

/**
 * Store the s3d instance geometry pointer for a given instance slot.
 * Must be called after cus3d_bvh_build_instance and before tracing.
 * @param instance_idx  Original (non-compact) instance index.
 * @param geom          Pointer to the GEOM_INSTANCE geometry, or NULL.
 */
res_T cus3d_bvh_set_instance_geometry(
    struct cus3d_bvh* bvh,
    unsigned instance_idx,
    struct geometry* geom);

/**
 * Set the child geometry store for an instance slot.
 * Must be called after cus3d_bvh_build_instance and before tracing.
 */
res_T cus3d_bvh_set_instance_child_store(
    struct cus3d_bvh* bvh,
    unsigned instance_idx,
    struct cus3d_geom_store* child_store);

/**
 * Look up the s3d instance geometry pointer using a TLAS compact index
 * (as returned by the GPU trace kernel in cus3d_hit_result::inst_id).
 * Returns NULL if the index is out of range or bvh is NULL.
 */
struct geometry* cus3d_bvh_get_instance_geometry(
    const struct cus3d_bvh* bvh,
    unsigned tlas_idx);

/**
 * Reset all instance entries, freeing owned child BVHs and TLAS.
 * Call before re-registering instances on a rebuild.
 */
res_T cus3d_bvh_reset_instances(
    struct cus3d_bvh* bvh,
    struct cus3d_device* dev);

/**
 * Register a pseudo-instance that wraps the parent BLAS with an identity
 * transform.  Used for mixed scenes (direct geometry + instances) so that
 * all geometry is accessible through the TLAS two-level traversal.
 *
 * The pseudo-instance does NOT own the child BVH memory (it is a
 * non-owning reference to the parent BLAS).
 *
 * @param parent_store  The parent geometry store (for child_store backref).
 * @return RES_OK on success.
 */
res_T cus3d_bvh_register_pseudo_instance(
    struct cus3d_bvh* bvh,
    uint32_t instance_idx,
    struct cus3d_geom_store* parent_store);

/**
 * Get the child geometry store for a given TLAS compact index.
 * Returns NULL if the index is out of range or no child store is set.
 */
const struct cus3d_geom_store* cus3d_bvh_get_instance_store(
    const struct cus3d_bvh* bvh,
    unsigned tlas_idx);

#ifdef __cplusplus
}
#endif

#endif /* CUS3D_BVH_H */
