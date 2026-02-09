/* Copyright (C) 2015-2023 |Méso|Star> (contact@meso-star.com)
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>. */

/*
 * cuBQL backend for s3d_scene_view_closest_point.
 *
 * Replaces the Embree rtcPointQuery-based implementation with a brute-force
 * host-side search over all cached geometries (Phase 4 strategy A).
 *
 * For each mesh, every triangle is tested; for each sphere, a direct
 * distance computation is performed.
 *
 * Instance support: the query position is transformed into instance-local
 * space via 3x3 inverse of the instance transform.  The child scene_view's
 * cached_geoms are iterated and distances are scaled back to world space.
 *
 * Filter functions are honoured: each candidate closest point is checked
 * against the geometry's hit_filter before being accepted.
 */

#include "s3d.h"
#include "s3d_c.h"
#include "s3d_device_c.h"
#include "s3d_instance.h"
#include "s3d_geometry.h"
#include "s3d_mesh.h"
#include "s3d_sphere.h"
#include "s3d_scene_view_c.h"

#include <rsys/float2.h>
#include <rsys/float3.h>
#include <rsys/double2.h>
#include <rsys/double3.h>
#include <rsys/float33.h>
#include <float.h>
#include <math.h>

/*******************************************************************************
 * Helper: closest point on a triangle (double precision)
 *
 * Pure math -- no Embree dependency.  Retained verbatim from the original
 * implementation.
 ******************************************************************************/
static INLINE double*
closest_point_triangle
  (const double p[3], /* Point */
   const double a[3], /* 1st triangle vertex */
   const double b[3], /* 2nd triangle vertex */
   const double c[3], /* 3rd triangle vertex */
    double closest_pt[3], /* Closest position */
    double uv[2]) /* UV of the closest position */
{
  double ab[3], ac[3], ap[3], bp[3], cp[3];
  double d1, d2, d3, d4, d5, d6;
  double va, vb, vc;
  double rcp_triangle_area;
  double v, w;
  ASSERT(p && a && b && c && closest_pt && uv);

  d3_sub(ab, b, a);
  d3_sub(ac, c, a);

  /* Check if the closest point is the triangle vertex 'a' */
  d3_sub(ap, p, a);
  d1 = d3_dot(ab, ap);
  d2 = d3_dot(ac, ap);
  if(d1 <= 0 && d2 <= 0) {
    uv[0] = 1;
    uv[1] = 0;
    return d3_set(closest_pt, a);
  }

  /* Check if the closest point is the triangle vertex 'b' */
  d3_sub(bp, p, b);
  d3 = d3_dot(ab, bp);
  d4 = d3_dot(ac, bp);
  if(d3 >= 0 && d4 <= d3) {
    uv[0] = 0;
    uv[1] = 1;
    return d3_set(closest_pt, b);
  }

  /* Check if the closest point is the triangle vertex 'c' */
  d3_sub(cp, p, c);
  d5 = d3_dot(ab, cp);
  d6 = d3_dot(ac, cp);
  if(d6 >= 0 && d5 <= d6) {
    uv[0] = 0;
    uv[1] = 0;
    return d3_set(closest_pt, c);
  }

  /* Check if the closest point is on the triangle edge 'ab' */
  vc = d1*d4 - d3*d2;
  if(vc <= 0 && d1 >= 0 && d3 <= 0) {
    const double s = d1 / (d1 - d3);
    closest_pt[0] = a[0] + s*ab[0];
    closest_pt[1] = a[1] + s*ab[1];
    closest_pt[2] = a[2] + s*ab[2];
    uv[0] = 1 - s;
    uv[1] = s;
    return closest_pt;
  }

  /* Check if the closest point is on the triangle edge 'ac' */
  vb = d5*d2 - d1*d6;
  if(vb <= 0 && d2 >= 0 && d6 <= 0) {
    const double s = d2 / (d2 - d6);
    closest_pt[0] = a[0] + s*ac[0];
    closest_pt[1] = a[1] + s*ac[1];
    closest_pt[2] = a[2] + s*ac[2];
    uv[0] = 1 - s;
    uv[1] = 0;
    return closest_pt;
  }

  /* Check if the closest point is on the triangle edge 'bc' */
  va = d3*d6 - d5*d4;
  if(va <= 0 && (d4 - d3) >= 0 && (d5 - d6) >= 0) {
    const double s = (d4 - d3) / ((d4 - d3) + (d5 - d6));
    closest_pt[0] = b[0] + s*(c[0] - b[0]);
    closest_pt[1] = b[1] + s*(c[1] - b[1]);
    closest_pt[2] = b[2] + s*(c[2] - b[2]);
    uv[0] = 0;
    uv[1] = 1 - s;
    return closest_pt;
  }

  /* The closest point lies in the triangle: compute its barycentric
   * coordinates */
  rcp_triangle_area = 1 / (va + vb + vc);
  v = vb * rcp_triangle_area;
  w = vc * rcp_triangle_area;

  /* Save the uv barycentric coordinates */
  uv[0] = 1 - v - w;
  uv[1] = v;
  ASSERT(eq_eps(uv[0] + uv[1] + w, 1, 1.e-4));

  if(uv[0] < 0) { /* Handle precision issues */
    if(uv[1] > w) uv[1] += uv[0];
    uv[0] = 0;
  }

  /* Use the barycentric coordinates to compute the world space position of the
   * closest point onto the triangle */
  closest_pt[0] = a[0] + v*ab[0] + w*ac[0];
  closest_pt[1] = a[1] + v*ab[1] + w*ac[1];
  closest_pt[2] = a[2] + v*ab[2] + w*ac[2];
  return closest_pt;
}

/*******************************************************************************
 * Brute-force closest point on a mesh
 *
 * Iterates every triangle of `geom->data.mesh`, computes the closest point,
 * checks the filter function, and updates `best_hit` / `cur_radius` if closer.
 *
 * `inst` is non-NULL when the mesh belongs to an instanced sub-scene.  In
 * that case `query_pos_ws` is in world space, mesh vertices are in local
 * space, and `inst->data.instance->transform` provides the local-to-world
 * 3x4 column-major matrix.
 ******************************************************************************/
static void
closest_point_mesh_bruteforce
  (const float query_pos_ws[3],
   struct geometry* geom,
   struct geometry* inst,    /* Can be NULL */
   float* cur_radius,        /* In/out: current search radius (world space) */
   const float user_radius,  /* Original user-submitted radius */
   void* query_data,
   struct s3d_hit* best_hit)
{
  size_t ntris, itri;
  const uint32_t* all_ids = NULL;
  const float* all_pos = NULL;
  double query_ls[3]; /* Local-space query position */
  int flip_surface = 0;
  float similarity_scale = 1.0f;
  ASSERT(geom && geom->type == GEOM_MESH);

  if(!geom->data.mesh->indices || !geom->data.mesh->attribs[S3D_POSITION])
    return;

  ntris = mesh_get_ntris(geom->data.mesh);
  if(ntris == 0) return;

  all_ids = mesh_get_ids(geom->data.mesh);
  all_pos = mesh_get_pos(geom->data.mesh);
  ASSERT(geom->data.mesh->attribs_type[S3D_POSITION] == S3D_FLOAT3);

  /* Transform query position to local space if instanced */
  if(inst) {
    const float* xf = inst->data.instance->transform;
    float inv33[9];
    float det;

    /* The instance transform is 3x4 column-major:
     *   cols [0..2] = 3x3 linear part, col [3] = translation.
     * Compute the 3x3 inverse. */
    det = f33_inverse(inv33, xf);
    if(fabsf(det) < 1.e-30f) return; /* Degenerate transform */

    /* Approximate similarity scale for distance conversion.
     * For uniform-scale transforms this is exact. */
    similarity_scale = (float)cbrt((double)fabsf(det));

    /* query_ls = inv33 * (query_ws - translation)
     * Use double-precision arithmetic for the subtraction and matrix-vector
     * multiply to match the accuracy of the CPU/Embree path (which promotes
     * to double via d3_muld / d3_set_f3).  The inv33 elements are float
     * (same source precision as Embree's world2inst), but the multiply-add
     * chain is carried out in double to avoid catastrophic cancellation when
     * the distance is much smaller than the coordinate magnitudes. */
    {
      double tw[3];
      tw[0] = (double)query_pos_ws[0] - (double)xf[9];
      tw[1] = (double)query_pos_ws[1] - (double)xf[10];
      tw[2] = (double)query_pos_ws[2] - (double)xf[11];
      /* inv33 is column-major: col0=[0,1,2], col1=[3,4,5], col2=[6,7,8]
       * row i of the matrix = inv33[i], inv33[3+i], inv33[6+i] */
      query_ls[0] = (double)inv33[0]*tw[0] + (double)inv33[3]*tw[1] + (double)inv33[6]*tw[2];
      query_ls[1] = (double)inv33[1]*tw[0] + (double)inv33[4]*tw[1] + (double)inv33[7]*tw[2];
      query_ls[2] = (double)inv33[2]*tw[0] + (double)inv33[5]*tw[1] + (double)inv33[8]*tw[2];
    }

    flip_surface = inst->flip_surface;
  } else {
    query_ls[0] = (double)query_pos_ws[0];
    query_ls[1] = (double)query_pos_ws[1];
    query_ls[2] = (double)query_pos_ws[2];
  }

  for(itri = 0; itri < ntris; itri++) {
    const uint32_t* ids = all_ids + itri * 3;
    double v0[3], v1[3], v2[3];
    double closest_pt[3], uv[2], vec[3];
    float Ng[3], E0[3], E1[3];
    float dst;
    int tri_flip;

    /* Fetch triangle vertices in double precision for robust computation */
    d3_set_f3(v0, all_pos + ids[0] * 3);
    d3_set_f3(v1, all_pos + ids[1] * 3);
    d3_set_f3(v2, all_pos + ids[2] * 3);

    /* Compute the closest point on the triangle */
    closest_point_triangle(query_ls, v0, v1, v2, closest_pt, uv);

    /* Distance in local space */
    d3_sub(vec, closest_pt, query_ls);
    dst = (float)d3_len(vec);

    /* Transform distance to world space */
    if(inst) {
      ASSERT(similarity_scale > 0);
      dst /= similarity_scale;
    }

    /* Quick reject: farther than current best */
    if(dst >= *cur_radius) continue;

    /* Compute the triangle normal in world space (left-hand / CW convention).
     * Keep it in float to avoid double-cast accuracy loss. */
    f3_sub(E0, all_pos + ids[1] * 3, all_pos + ids[0] * 3);
    f3_sub(E1, all_pos + ids[2] * 3, all_pos + ids[0] * 3);
    f3_cross(Ng, E1, E0);

    /* Flip the geometric normal wrt the flip_surface flags */
    tri_flip = flip_surface ^ geom->flip_surface;
    if(tri_flip) f3_minus(Ng, Ng);

    /* Build candidate hit */
    {
      struct s3d_hit tmp_hit = S3D_HIT_NULL;
      float dir[3], range[2], pos_f[3];

      tmp_hit.prim.shape__ = geom;
      tmp_hit.prim.inst__ = inst;
      tmp_hit.distance = dst;
      tmp_hit.uv[0] = (float)uv[0];
      tmp_hit.uv[1] = (float)uv[1];
      tmp_hit.normal[0] = Ng[0];
      tmp_hit.normal[1] = Ng[1];
      tmp_hit.normal[2] = Ng[2];
      tmp_hit.prim.prim_id = (unsigned)itri;
      tmp_hit.prim.geom_id = geom->name;
      tmp_hit.prim.inst_id = inst ? inst->name : S3D_INVALID_ID;
      tmp_hit.prim.scene_prim_id =
        tmp_hit.prim.prim_id
      + geom->scene_prim_id_offset
      + (inst ? inst->scene_prim_id_offset : 0);

      /* `dir' is the direction from query to closest point.
       * `pos' is the world-space query position. */
      f3_set_d3(dir, vec);
      if(inst) {
        f3_set(pos_f, query_pos_ws);
      } else {
        f3_set_d3(pos_f, query_ls);
      }

      range[0] = 0;
      range[1] = user_radius;

      /* [10 chapter] Call the filter function before accepting */
      {
        struct hit_filter* filter = &geom->data.mesh->filter;
        if(filter->func
        && filter->func(&tmp_hit, pos_f, dir, range,
                        query_data, filter->data)) {
          continue; /* Rejected by filter */
        }
      }

      /* Accept candidate: update best hit and shrink search radius */
      *best_hit = tmp_hit;
      *cur_radius = dst;
    }
  }
}

/*******************************************************************************
 * Brute-force closest point on a sphere
 ******************************************************************************/
static void
closest_point_sphere_direct
  (const float query_pos_ws[3],
   struct geometry* geom,
   struct geometry* inst,    /* Can be NULL */
   float* cur_radius,
   const float user_radius,
   void* query_data,
   struct s3d_hit* best_hit)
{
  float sphere_pos[3];
  float Ng[3];
  float uv[2];
  float dst, len;
  int flip_surface = 0;
  ASSERT(geom && geom->type == GEOM_SPHERE);

  if(sphere_is_degenerated(geom->data.sphere)) return;

  f3_set(sphere_pos, geom->data.sphere->pos);

  if(inst) {
    /* Transform sphere center to world space */
    const float* xf = inst->data.instance->transform;
    f33_mulf3(sphere_pos, xf, sphere_pos);
    f3_add(sphere_pos, xf + 9, sphere_pos);

    flip_surface = inst->flip_surface;
  }

  /* Direction from sphere center to query position */
  f3_sub(Ng, query_pos_ws, sphere_pos);
  len = f3_len(Ng);

  /* Distance from query position to sphere surface */
  dst = fabsf(len - geom->data.sphere->radius);

  if(dst >= *cur_radius) return;

  /* Normalize the hit normal */
  if(len > 0) {
    f3_divf(Ng, Ng, len);
  } else {
    /* Query at sphere center: arbitrarily choose +X */
    Ng[0] = 1.f;
    Ng[1] = 0.f;
    Ng[2] = 0.f;
  }

  /* Compute UV */
  sphere_normal_to_uv(Ng, uv);

  /* Flip normal */
  flip_surface ^= geom->flip_surface;
  if(flip_surface) f3_minus(Ng, Ng);

  /* Build candidate hit */
  {
    struct s3d_hit tmp_hit = S3D_HIT_NULL;
    float dir[3], range[2];

    tmp_hit.prim.shape__ = geom;
    tmp_hit.prim.inst__ = inst;
    tmp_hit.distance = dst;
    tmp_hit.uv[0] = uv[0];
    tmp_hit.uv[1] = uv[1];
    tmp_hit.normal[0] = Ng[0];
    tmp_hit.normal[1] = Ng[1];
    tmp_hit.normal[2] = Ng[2];
    tmp_hit.prim.prim_id = 0; /* Sphere has a single primitive */
    tmp_hit.prim.geom_id = geom->name;
    tmp_hit.prim.inst_id = inst ? inst->name : S3D_INVALID_ID;
    tmp_hit.prim.scene_prim_id =
      tmp_hit.prim.prim_id
    + geom->scene_prim_id_offset
    + (inst ? inst->scene_prim_id_offset : 0);

    range[0] = 0;
    range[1] = user_radius;

    /* Use the reversed geometric normal as the direction, consistent with
     * the original implementation (direction from surface to query). */
    f3_minus(dir, Ng);

    /* [10 chapter] Call the filter function before accepting */
    {
      struct hit_filter* filter = &geom->data.sphere->filter;
      if(filter->func
      && filter->func(&tmp_hit, query_pos_ws, dir, range,
                      query_data, filter->data)) {
        return; /* Rejected by filter */
      }
    }

    /* Accept */
    *best_hit = tmp_hit;
    *cur_radius = dst;
  }
}

/*******************************************************************************
 * Internal: iterate over a scene_view's cached geometries
 *
 * Used both for top-level geometries and for instance children.  When `inst`
 * is non-NULL the caller passes the child scene_view together with the
 * instance geometry.
 ******************************************************************************/
static void
closest_point_iterate_geoms
  (struct s3d_scene_view* scnview,
   const float query_pos_ws[3],
   struct geometry* inst,     /* NULL for top-level */
   float* cur_radius,
   const float user_radius,
   void* query_data,
   struct s3d_hit* best_hit)
{
  struct htable_geom_iterator it, end;

  htable_geom_begin(&scnview->cached_geoms, &it);
  htable_geom_end(&scnview->cached_geoms, &end);

  while(!htable_geom_iterator_eq(&it, &end)) {
    struct geometry* geom = *htable_geom_iterator_data_get(&it);
    htable_geom_iterator_next(&it);

    if(!geom->is_enabled) continue;

    switch(geom->type) {
      case GEOM_MESH:
        closest_point_mesh_bruteforce(
          query_pos_ws, geom, inst,
          cur_radius, user_radius, query_data, best_hit);
        break;
      case GEOM_SPHERE:
        closest_point_sphere_direct(
          query_pos_ws, geom, inst,
          cur_radius, user_radius, query_data, best_hit);
        break;
      case GEOM_INSTANCE:
        /* Only one level of instancing supported: do not recurse into
         * instances that are children of another instance. */
        if(!inst && geom->data.instance->scnview) {
          closest_point_iterate_geoms(
            geom->data.instance->scnview,
            query_pos_ws, geom,
            cur_radius, user_radius, query_data, best_hit);
        }
        break;
      default: break;
    }
  }
}

/*******************************************************************************
 * Exported function
 ******************************************************************************/
res_T
s3d_scene_view_closest_point
  (struct s3d_scene_view* scnview,
   const float pos[3],
   const float radius,
   void* query_data,
   struct s3d_hit* hit)
{
  float cur_radius;

  if(!scnview || !pos || radius <= 0 || !hit)
    return RES_BAD_ARG;
  if((scnview->mask & S3D_TRACE) == 0) {
    log_error(scnview->scn->dev,
      "%s: the S3D_TRACE flag is not active onto the submitted scene view.\n",
      FUNC_NAME);
    return RES_BAD_OP;
  }

  *hit = S3D_HIT_NULL;
  cur_radius = radius;

  /* Brute-force: iterate all cached geometries in this scene_view */
  closest_point_iterate_geoms(
    scnview, pos, NULL, &cur_radius, radius, query_data, hit);

  return RES_OK;
}

