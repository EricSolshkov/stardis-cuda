#ifndef CUS3D_TRACE_UTIL_H
#define CUS3D_TRACE_UTIL_H

#include "s3d_scene_view_c.h"
#include "cus3d_geom_store.h"

/**
 * @brief Fixes UV and normal conventions for s3d_hit.
 *
 * Converts Moller-Trumbore UV to s3d barycentric convention and flips normals
 * to match s3d's CW-winding convention.
 *
 * @param hit Pointer to the s3d_hit structure to fix.
 * @param ge Pointer to the geometry entry associated with the hit.
 */
static INLINE void
trace_hit_fixup
  (struct s3d_hit* hit,
   const struct geom_entry* ge)
{
  ASSERT(hit && ge);

  if(ge->type == PRIM_TRIANGLE) {
    float mt_u = hit->uv[0];
    float mt_v = hit->uv[1];
    float w = 1.0f - mt_u - mt_v;
    if(w < 0.0f) {
      if(mt_u > mt_v) mt_u += w;
      else mt_v += w;
      w = 0.0f;
    }
    hit->uv[0] = w;
    hit->uv[1] = mt_u;

    hit->normal[0] = -hit->normal[0];
    hit->normal[1] = -hit->normal[1];
    hit->normal[2] = -hit->normal[2];
  }
}

#endif /* CUS3D_TRACE_UTIL_H */