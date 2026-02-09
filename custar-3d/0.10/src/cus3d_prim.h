/**
 * @file cus3d_prim.h
 * @brief Primitive & Attribute Query for custar-3d
 * Convert GPU hit results to s3d_primitive and s3d_hit structures.
 */

#ifndef CUS3D_PRIM_H
#define CUS3D_PRIM_H

#include "cus3d_geom_store.h"
#include "cus3d_types.h"

#ifdef __cplusplus
extern "C" {
#endif

struct s3d_primitive;
struct s3d_hit;
struct cus3d_bvh;

void cus3d_hit_to_primitive(
    const struct cus3d_geom_store* store,
    const struct cus3d_bvh* bvh,
    const struct cus3d_hit_result* gpu_hit,
    struct s3d_primitive* prim);

void cus3d_hit_to_s3d_hit(
    const struct cus3d_geom_store* store,
    const struct cus3d_bvh* bvh,
    const struct cus3d_hit_result* gpu_hit,
    struct s3d_hit* hit);

#ifdef __cplusplus
}
#endif

#endif /* CUS3D_PRIM_H */
