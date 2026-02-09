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

#include "s3d.h"
#include "s3d_c.h"
#include "s3d_device_c.h"
#include "s3d_scene_c.h"
#include "s3d_scene_view_c.h"
#include "s3d_shape_c.h"
#include "s3d_geometry.h"
#include "s3d_mesh.h"
#include "s3d_sphere.h"
#include "s3d_instance.h"
#include "cus3d_device.h"
#include "cus3d_geom_store.h"
#include "cus3d_bvh.h"

#include <rsys/algorithm.h>
#include <rsys/float3.h>
#include <rsys/float33.h>
#include <rsys/mem_allocator.h>

/*******************************************************************************
 * Helper functions
 ******************************************************************************/
static FINLINE int
aabb_is_degenerated(const float lower[3], const float upper[3])
{
  ASSERT(lower && upper);
  return lower[0] > upper[0] || lower[1] > upper[1] || lower[2] > upper[2];
}

static INLINE int
cmp_float(const void* a, const void* b)
{
  const float key = *(const float*)a;
  const float val = *(const float*)b;
  if(key < val) return -1;
  if(key > val) return +1;
  return 0;
}

static INLINE int
cmp_float_to_fltui(const void* a, const void* b)
{
  const float key = *(const float*)a;
  const struct fltui* fltui = (const struct fltui*)b;
  if(key < fltui->flt) return -1;
  if(key > fltui->flt) return +1;
  return 0;
}

static INLINE int
cmp_size_t_to_nprims_cdf(const void* a, const void* b)
{
  const size_t key = *(const size_t*)a;
  const struct nprims_cdf* nprims_cdf = (const struct nprims_cdf*)b;
  if(key < nprims_cdf->nprims-1) return -1;
  if(key > nprims_cdf->nprims-1) return +1;
  return 0;
}

/* cuBQL version: geometry detach just marks GPU dirty and drops the ref.
 * Replaces the Embree version that called rtcDetachGeometry/rtcReleaseGeometry. */
static INLINE void
scene_view_destroy_geometry_cu
  (struct s3d_scene_view* scnview,
   struct geometry* geom)
{
  ASSERT(geom);
  scnview->gpu_dirty = 1;
  geometry_ref_put(geom);
}

static void
on_shape_detach
  (const struct s3d_scene* scn,
   const struct s3d_shape* shape,
   void* data)
{
  struct geometry** pgeom;
  struct geometry* geom;
  struct s3d_scene_view* scnview = (struct s3d_scene_view*)data;
  unsigned shape_id;
  ASSERT(scn && shape && data);
  (void)scn;

  S3D(shape_get_id(shape, &shape_id));
  pgeom = htable_geom_find(&scnview->cached_geoms, &shape_id);

  /* The scnview did not register a geometry for this shape. Ignore the signal */
  if(!pgeom) return;

  geom = *pgeom;
  if(scnview->mask == 0) {
    /* The scnview is NOT in use. Directly rm the cached geometry and notify
     * that the scene AABB must be reevaluated */
    size_t n; (void)n;
    scene_view_destroy_geometry_cu(scnview, geom);
    n = htable_geom_erase(&scnview->cached_geoms, &shape_id);
    ASSERT(n == 1);
    scnview->aabb_update = 1;
  } else {
    res_T res = darray_uint_push_back(&scnview->detached_shapes, &shape_id);
    if(res != RES_OK) FATAL("Insufficient memory.\n");
  }
}

/*******************************************************************************
 * cuBQL scene setup — replaces scene_view_setup_embree
 ******************************************************************************/
static res_T
scene_view_setup_cubql
  (struct s3d_scene_view* scnview,
   const struct s3d_accel_struct_conf* accel_struct_conf)
{
  cus3d_build_quality quality;
  res_T res = RES_OK;
  ASSERT(scnview);

  switch(accel_struct_conf->quality) {
    case S3D_ACCEL_STRUCT_QUALITY_LOW:    quality = CUS3D_BUILD_LOW;    break;
    case S3D_ACCEL_STRUCT_QUALITY_HIGH:   quality = CUS3D_BUILD_HIGH;   break;
    case S3D_ACCEL_STRUCT_QUALITY_MEDIUM:
    default:                              quality = CUS3D_BUILD_MEDIUM; break;
  }

  if(!scnview->geom_store) {
    res = cus3d_geom_store_create(&scnview->geom_store);
    if(res != RES_OK) goto error;
  }
  if(!scnview->bvh) {
    res = cus3d_bvh_create(&scnview->bvh);
    if(res != RES_OK) goto error;
  }

  if(!scnview->gpu_dirty && cus3d_bvh_is_valid(scnview->bvh))
    goto exit;

  res = cus3d_geom_store_sync(scnview->geom_store,
                               scnview->scn,
                               scnview->scn->dev->gpu);
  if(res != RES_OK) goto error;

  res = cus3d_geom_store_compute_bounds(scnview->geom_store,
                                         scnview->scn->dev->gpu);
  if(res != RES_OK) goto error;

  res = cus3d_bvh_build(scnview->bvh,
                         scnview->geom_store,
                         scnview->scn->dev->gpu,
                         quality);
  if(res != RES_OK) goto error;

  /* Instance two-level BVH support:
   * Walk cached geometries, find GEOM_INSTANCE entries.  For each one:
   *   1) Ensure the child scene's geom_store + BVH are built (recurse).
   *   2) Build a child BVH slot inside the parent's cus3d_bvh.
   *   3) Set the forward/inverse 3x4 transforms.
   *
   * For mixed scenes (direct geometry + instances), the parent BLAS is
   * registered as a "pseudo-instance" with identity transform so that
   * the TLAS two-level traversal covers all geometry.  Each instance
   * carries per-instance GPU array pointers to its own geometry store,
   * eliminating the need to merge stores.
   *
   * Then build the TLAS over all instance world-space AABBs.
   */
  {
    struct htable_geom_iterator it, end;
    uint32_t inst_idx = 0;
    int has_instances = 0;
    int has_direct_geometry = (scnview->geom_store->total_prims > 0);

    /* Reset instance state from previous builds */
    cus3d_bvh_reset_instances(scnview->bvh, scnview->scn->dev->gpu);

    /* First pass: build child BVHs and set transforms */
    htable_geom_begin(&scnview->cached_geoms, &it);
    htable_geom_end(&scnview->cached_geoms, &end);
    while(!htable_geom_iterator_eq(&it, &end)) {
      struct geometry* geom = *htable_geom_iterator_data_get(&it);
      htable_geom_iterator_next(&it);

      if(geom->type != GEOM_INSTANCE) continue;
      if(!geom->is_enabled) continue;

      struct instance* inst = geom->data.instance;
      struct s3d_scene_view* child_view = inst->scnview;
      ASSERT(child_view);

      has_instances = 1;

      /* Recursively ensure the child scene's GPU data is ready */
      if(!child_view->geom_store) {
        res = cus3d_geom_store_create(&child_view->geom_store);
        if(res != RES_OK) goto error;
      }
      if(!child_view->bvh) {
        res = cus3d_bvh_create(&child_view->bvh);
        if(res != RES_OK) goto error;
      }

      res = cus3d_geom_store_sync(child_view->geom_store,
                                   child_view->scn,
                                   scnview->scn->dev->gpu);
      if(res != RES_OK) goto error;

      res = cus3d_geom_store_compute_bounds(child_view->geom_store,
                                             scnview->scn->dev->gpu);
      if(res != RES_OK) goto error;

      /* Build child BVH (BLAS for this instance) in the parent bvh */
      res = cus3d_bvh_build_instance(scnview->bvh,
                                      inst_idx,
                                      child_view->geom_store,
                                      scnview->scn->dev->gpu);
      if(res != RES_OK) goto error;

      /* Store child geometry store pointer for per-instance array access */
      cus3d_bvh_set_instance_child_store(scnview->bvh,
                                          inst_idx,
                                          child_view->geom_store);

      /* Compute inverse of the 3x3 rotation part */
      {
        float inv33[9];
        float inv_transform[12];
        f33_inverse(inv33, inst->transform);
        /* inv_transform 3x4: rotation = inv33, translation = -inv33 * t */
        memcpy(inv_transform, inv33, 9 * sizeof(float));
        /* t = transform[9..11] */
        inv_transform[9]  = -(inv33[0]*inst->transform[9]
                            + inv33[3]*inst->transform[10]
                            + inv33[6]*inst->transform[11]);
        inv_transform[10] = -(inv33[1]*inst->transform[9]
                            + inv33[4]*inst->transform[10]
                            + inv33[7]*inst->transform[11]);
        inv_transform[11] = -(inv33[2]*inst->transform[9]
                            + inv33[5]*inst->transform[10]
                            + inv33[8]*inst->transform[11]);

        res = cus3d_bvh_set_instance_transform(scnview->bvh,
                                                inst_idx,
                                                inst->transform,
                                                inv_transform);
        if(res != RES_OK) goto error;
      }

      /* Record the s3d instance geometry so the host can resolve
       * TLAS index -> geometry* (for inst__ and inst_id). */
      res = cus3d_bvh_set_instance_geometry(scnview->bvh,
                                             inst_idx,
                                             geom);
      if(res != RES_OK) goto error;

      inst_idx++;
    }

    /* Mixed scene: direct geometry + instances.
     * Register the parent BLAS as a pseudo-instance with identity transform
     * so the TLAS two-level traversal covers both direct and instanced
     * geometry.  The pseudo-instance has inst_geom == NULL, which causes
     * cus3d_prim to set inst_id = S3D_INVALID_ID (direct hit). */
    if(has_instances && has_direct_geometry) {
      res = cus3d_bvh_register_pseudo_instance(scnview->bvh,
                                                inst_idx,
                                                scnview->geom_store);
      if(res != RES_OK) goto error;
      inst_idx++;
    }

    /* Build TLAS from all instance world-space AABBs */
    if(has_instances) {
      res = cus3d_bvh_build_tlas(scnview->bvh, scnview->scn->dev->gpu);
      if(res != RES_OK) goto error;
    }
  }

  scnview->gpu_dirty = 0;
  scnview->build_quality = (int)quality;

exit:
  return res;
error:
  goto exit;
}

static res_T
scene_view_register_mesh_cu
  (struct s3d_scene_view* scnview,
   struct s3d_shape* shape)
{
  struct geometry** pgeom = NULL;
  struct geometry* geom = NULL;
  size_t iattr;
  unsigned shape_id;
  int is_valid;
  res_T res = RES_OK;
  ASSERT(scnview && shape && shape->type == GEOM_MESH);

  is_valid = shape->data.mesh->indices
          && shape->data.mesh->attribs[S3D_POSITION];

  S3D(shape_get_id(shape, &shape_id));
  pgeom = htable_geom_find(&scnview->cached_geoms, &shape_id);

  if(pgeom) {
    geom = *pgeom;
    if(!is_valid) {
      scene_view_destroy_geometry_cu(scnview, geom);
      htable_geom_erase(&scnview->cached_geoms, &shape_id);
      scnview->aabb_update = 1;
      goto exit;
    }
  } else if(is_valid) {
    res = geometry_create(scnview->scn->dev, &geom);
    if(res != RES_OK) goto error;
    res = mesh_create(scnview->scn->dev, &geom->data.mesh);
    if(res != RES_OK) goto error;
    geom->type = GEOM_MESH;
    res = htable_geom_set(&scnview->cached_geoms, &shape_id, &geom);
    if(res != RES_OK) goto error;
    geom->name = shape->id.index;
    scnview->gpu_dirty = 1;
  }

  if(!is_valid) goto exit;

  if(geom->data.mesh->indices != shape->data.mesh->indices) {
    if(geom->data.mesh->indices)
      index_buffer_ref_put(geom->data.mesh->indices);
    index_buffer_ref_get(shape->data.mesh->indices);
    geom->data.mesh->indices = shape->data.mesh->indices;
    scnview->gpu_dirty = 1;
  }

  FOR_EACH(iattr, 0, S3D_ATTRIBS_COUNT__) {
    if(geom->data.mesh->attribs[iattr] == shape->data.mesh->attribs[iattr])
      continue;

    if(geom->data.mesh->attribs[iattr])
      vertex_buffer_ref_put(geom->data.mesh->attribs[iattr]);

    if(!shape->data.mesh->attribs[iattr]) {
      geom->data.mesh->attribs[iattr] = NULL;
      continue;
    }

    vertex_buffer_ref_get(shape->data.mesh->attribs[iattr]);
    geom->data.mesh->attribs[iattr] = shape->data.mesh->attribs[iattr];
    geom->data.mesh->attribs_type[iattr] = shape->data.mesh->attribs_type[iattr];

    if(iattr == S3D_POSITION) {
      scnview->gpu_dirty = 1;
      scnview->aabb_update = 1;
    }
  }

  if(geom->is_enabled != shape->is_enabled) {
    geom->is_enabled = shape->is_enabled;
    scnview->gpu_dirty = 1;
    scnview->aabb_update = 1;
  }

  geom->data.mesh->filter = shape->data.mesh->filter;
  geom->flip_surface = shape->flip_surface;

exit:
  return res;
error:
  goto exit;
}

static res_T
scene_view_register_sphere_cu
  (struct s3d_scene_view* scnview,
   struct s3d_shape* shape)
{
  struct geometry** pgeom = NULL;
  struct geometry* geom = NULL;
  unsigned shape_id;
  int is_valid;
  res_T res = RES_OK;
  ASSERT(scnview && shape && shape->type == GEOM_SPHERE);

  is_valid = !sphere_is_degenerated(shape->data.sphere);

  S3D(shape_get_id(shape, &shape_id));
  pgeom = htable_geom_find(&scnview->cached_geoms, &shape_id);

  if(pgeom) {
    geom = *pgeom;
    if(!is_valid) {
      scene_view_destroy_geometry_cu(scnview, geom);
      htable_geom_erase(&scnview->cached_geoms, &shape_id);
      scnview->aabb_update = 1;
      goto exit;
    }
  } else if(is_valid) {
    res = geometry_create(scnview->scn->dev, &geom);
    if(res != RES_OK) goto error;
    res = sphere_create(scnview->scn->dev, &geom->data.sphere);
    if(res != RES_OK) goto error;
    geom->type = GEOM_SPHERE;
    res = htable_geom_set(&scnview->cached_geoms, &shape_id, &geom);
    if(res != RES_OK) goto error;
    geom->name = shape->id.index;
    scnview->gpu_dirty = 1;
  }

  if(!is_valid) goto exit;

  if(geom->data.sphere->radius != shape->data.sphere->radius) {
    geom->data.sphere->radius = shape->data.sphere->radius;
    scnview->gpu_dirty = 1;
    scnview->aabb_update = 1;
  }
  if(!f3_eq(geom->data.sphere->pos, shape->data.sphere->pos)) {
    f3_set(geom->data.sphere->pos, shape->data.sphere->pos);
    scnview->gpu_dirty = 1;
    scnview->aabb_update = 1;
  }

  if(geom->is_enabled != shape->is_enabled) {
    geom->is_enabled = shape->is_enabled;
    scnview->gpu_dirty = 1;
    scnview->aabb_update = 1;
  }

  geom->data.sphere->filter = shape->data.sphere->filter;
  geom->flip_surface = shape->flip_surface;

exit:
  return res;
error:
  goto exit;
}

static res_T
scene_view_register_instance_cu
  (struct s3d_scene_view* scnview,
   struct s3d_shape* shape,
   const int mask)
{
  struct geometry** pgeom = NULL;
  struct geometry* geom = NULL;
  struct s3d_scene_view** pview = NULL;
  struct s3d_scene_view* view = NULL;
  unsigned shape_id;
  res_T res = RES_OK;
  ASSERT(scnview && shape && shape->type == GEOM_INSTANCE);

  if(shape->data.instance->scene->instances_count != 0) {
    res = RES_BAD_ARG;
    goto error;
  }

  pview = htable_instview_find
    (&scnview->instviews, &shape->data.instance->scene);
  if(pview) {
    view = *pview;
  } else {
    res = s3d_scene_view_create(shape->data.instance->scene, mask, &view);
    if(res != RES_OK) goto error;
    res = htable_instview_set
      (&scnview->instviews, &shape->data.instance->scene, &view);
    if(res != RES_OK) goto error;
  }

  S3D(shape_get_id(shape, &shape_id));
  pgeom = htable_geom_find(&scnview->cached_geoms, &shape_id);
  if(pgeom) {
    geom = *pgeom;
  } else {
    res = geometry_create(scnview->scn->dev, &geom);
    if(res != RES_OK) goto error;
    geom->type = GEOM_INSTANCE;
    res = instance_create(shape->data.instance->scene, &geom->data.instance);
    if(res != RES_OK) goto error;
    res = htable_geom_set(&scnview->cached_geoms, &shape_id, &geom);
    if(res != RES_OK) goto error;
    geom->name = shape->id.index;
    scnview->gpu_dirty = 1;
    scnview->aabb_update = 1;
  }
  ASSERT(geom->data.instance->scene == shape->data.instance->scene);
  geom->data.instance->scnview = view;

  if(!f33_eq(shape->data.instance->transform, geom->data.instance->transform)
  || !f3_eq(shape->data.instance->transform+9, geom->data.instance->transform+9)) {
    f33_set(geom->data.instance->transform, shape->data.instance->transform);
    f3_set(geom->data.instance->transform+9, shape->data.instance->transform+9);
    scnview->gpu_dirty = 1;
    scnview->aabb_update = 1;
  }

  if(geom->is_enabled != shape->is_enabled) {
    geom->is_enabled = shape->is_enabled;
    scnview->gpu_dirty = 1;
    scnview->aabb_update = 1;
  }

  geom->flip_surface = shape->flip_surface;

exit:
  return res;
error:
  goto exit;
}

static res_T
scene_view_compute_cdf(struct s3d_scene_view* scnview)
{
  struct htable_geom_iterator it, end;
  size_t len;
  float area = 0.f;
  res_T res = RES_OK;
  ASSERT(scnview);
  ASSERT(darray_fltui_size_get(&scnview->cdf) == 0);

  htable_geom_begin(&scnview->cached_geoms, &it);
  htable_geom_end(&scnview->cached_geoms, &end);

  while(!htable_geom_iterator_eq(&it, &end)) {
    const unsigned* shape_id = htable_geom_iterator_key_get(&it);
    struct geometry* geom = *htable_geom_iterator_data_get(&it);
    struct fltui fltui;

    htable_geom_iterator_next(&it);

    if(!geom->is_enabled) continue;

    switch(geom->type) {
      case GEOM_MESH:
        res = mesh_compute_cdf(geom->data.mesh);
        if(res != RES_OK) goto error;
        len = darray_float_size_get(&geom->data.mesh->cdf);
        if(len) {
          area += darray_float_cdata_get(&geom->data.mesh->cdf)[len - 1];
        }
        break;
      case GEOM_SPHERE:
        len = 1;
        area += sphere_compute_area(geom->data.sphere);
        break;
      case GEOM_INSTANCE:
        /* The instance CDF was computed during its scnview  synchronisation */
        len = darray_fltui_size_get(&geom->data.instance->scnview->cdf);
        if(len) {
          area += darray_fltui_cdata_get
            (&geom->data.instance->scnview->cdf)[len - 1].flt;
        }
        break;
      default: FATAL("Unreachable code\n"); break;
    }
    fltui.ui = *shape_id;
    fltui.flt = area;
    if(len) {
      res = darray_fltui_push_back(&scnview->cdf, &fltui);
      if(res != RES_OK) goto error;
    }
  }
exit:
  return res;
error:
  darray_fltui_clear(&scnview->cdf);
  goto exit;
}

static res_T
scene_view_compute_nprims_cdf
  (struct s3d_scene_view* scnview,
   const char store_cdf)
{
  struct htable_geom_iterator it, end;
  size_t len;
  unsigned nprims;
  res_T res = RES_OK;
  ASSERT(scnview);
  ASSERT(darray_nprims_cdf_size_get(&scnview->nprims_cdf) == 0);

  htable_geom_begin(&scnview->cached_geoms, &it);
  htable_geom_end(&scnview->cached_geoms, &end);

  nprims = 0;
  while(!htable_geom_iterator_eq(&it, &end)) {
    const unsigned* shape_id = htable_geom_iterator_key_get(&it);
    struct geometry* geom = *htable_geom_iterator_data_get(&it);
    struct nprims_cdf cdf;

    htable_geom_iterator_next(&it);

    if(!geom->is_enabled) continue;

    geom->scene_prim_id_offset = nprims;
    switch(geom->type) {
      case GEOM_MESH:
        len = mesh_get_ntris(geom->data.mesh);
        nprims += (unsigned)len;
        break;
      case GEOM_SPHERE:
        len = 1;
        nprims += 1;
        break;
      case GEOM_INSTANCE:
        /* The instance CDF was computed during its scnview synchronisation */
        len = darray_nprims_cdf_size_get
          (&geom->data.instance->scnview->nprims_cdf);
        if(len) {
          nprims += darray_nprims_cdf_cdata_get
            (&geom->data.instance->scnview->nprims_cdf)[len - 1].nprims;
        }
        break;
      default: FATAL("Unreachable code\n"); break;
    }

    cdf.nprims = nprims;
    cdf.ishape = *shape_id;
    if(store_cdf && len) {
      res = darray_nprims_cdf_push_back(&scnview->nprims_cdf, &cdf);
      if(res != RES_OK) goto error;
    }
  }
exit:
  return res;
error:
  darray_nprims_cdf_clear(&scnview->nprims_cdf);
  goto exit;
}

static void
scene_view_compute_scene_aabb(struct s3d_scene_view* scnview)
{
  struct htable_geom_iterator it, end;
  float lower[3], upper[3];

  ASSERT(scnview->lower[0] == FLT_MAX && scnview->upper[0] == -FLT_MAX);
  ASSERT(scnview->lower[1] == FLT_MAX && scnview->upper[1] == -FLT_MAX);
  ASSERT(scnview->lower[2] == FLT_MAX && scnview->upper[2] == -FLT_MAX);

  htable_geom_begin(&scnview->cached_geoms, &it);
  htable_geom_end(&scnview->cached_geoms, &end);

  while(!htable_geom_iterator_eq(&it, &end)) {
    struct instance* inst;
    struct geometry* geom = *htable_geom_iterator_data_get(&it);

    htable_geom_iterator_next(&it);

    if(!geom->is_enabled) continue;

    switch(geom->type) {
      case GEOM_MESH:
        mesh_compute_aabb(geom->data.mesh, lower, upper);
        break;
      case GEOM_SPHERE:
        sphere_compute_aabb(geom->data.sphere, lower, upper);
        break;
      case GEOM_INSTANCE:
        /* Note that the instance AABB was computed during its scnview
         * synchronisation. */
        inst = geom->data.instance;
        if(aabb_is_degenerated(inst->scnview->lower, inst->scnview->upper)) {
          /* Empty scene */
          f3_splat(lower, FLT_MAX);
          f3_splat(upper,-FLT_MAX);
        } else {
          /* Transform local scene AABB bounds in world space */
          f33_mulf3(lower, inst->transform, inst->scnview->lower);
          f33_mulf3(upper, inst->transform, inst->scnview->upper);
          f3_add(lower, inst->transform + 9, lower);
          f3_add(upper, inst->transform + 9, upper);
          /* Define the world space AABB of the transformed local scene AABB */
          if(lower[0] > upper[0]) SWAP(float, lower[0], upper[0]);
          if(lower[1] > upper[1]) SWAP(float, lower[1], upper[1]);
          if(lower[2] > upper[2]) SWAP(float, lower[2], upper[2]);
        }
        break;
      default: FATAL("Unreachable code\n"); break;
    }
    f3_min(scnview->lower, scnview->lower, lower);
    f3_max(scnview->upper, scnview->upper, upper);
  }
}

static float
scene_view_compute_volume
  (struct s3d_scene_view* scnview,
   const char flip_surface)
{
  struct htable_geom_iterator it, end;
  float volume;

  ASSERT(scnview);
  htable_geom_begin(&scnview->cached_geoms, &it);
  htable_geom_end(&scnview->cached_geoms, &end);

  volume = 0.f;
  while(!htable_geom_iterator_eq(&it, &end)) {
    struct geometry* geom = *htable_geom_iterator_data_get(&it);
    const char flip = geom->flip_surface ^ flip_surface;

    htable_geom_iterator_next(&it);

    if(!geom->is_enabled) continue;

    switch(geom->type) {
      case GEOM_MESH:
        volume += mesh_compute_volume(geom->data.mesh, flip);
        break;
      case GEOM_SPHERE:
        volume += flip
          ? -sphere_compute_volume(geom->data.sphere)
          :  sphere_compute_volume(geom->data.sphere);
        break;
      case GEOM_INSTANCE:
        volume += scene_view_compute_volume(geom->data.instance->scnview, flip);
        break;
      default: FATAL("Unreachable code\n"); break;
    }
  }

  if(volume < 0.f) {
    log_warning(scnview->scn->dev,
"%s:\n"
"\tthe volume is negative. The scene shapes might not represent closed 2D\n"
"\tmanifold volumes, or their surface normal might not point inward the volume.\n",
      FUNC_NAME);
  }

  return volume;
}

static res_T
scene_view_sync
  (struct s3d_scene_view* scnview,
   const int mask,
   const struct s3d_accel_struct_conf* accel_struct_conf)
{
  struct htable_shape_iterator it, end;
  res_T res = RES_OK;

  ASSERT(scnview && accel_struct_conf);

  /* Commit the scene shape to the scnview */
  htable_shape_begin(&scnview->scn->shapes, &it);
  htable_shape_end(&scnview->scn->shapes, &end);
  while(!htable_shape_iterator_eq(&it, &end)) {
    struct s3d_shape** pshape = htable_shape_iterator_data_get(&it);
    struct s3d_shape* shape = *pshape;

    switch(shape->type) {
      case GEOM_INSTANCE:
        res = scene_view_register_instance_cu(scnview, shape, mask);
        break;
      case GEOM_MESH:
        res = scene_view_register_mesh_cu(scnview, shape);
        break;
      case GEOM_SPHERE:
        res = scene_view_register_sphere_cu(scnview, shape);
        break;
      default: FATAL("Unreachable code\n"); break;
    }
    if(res != RES_OK) goto error;
    htable_shape_iterator_next(&it);
  }

  /* Setup the scene for the S3D_TRACE scnview */
  if((mask & S3D_TRACE) != 0) {
    res = scene_view_setup_cubql(scnview, accel_struct_conf);
    if(res != RES_OK) goto error;
  }
  /* Setup the scene for the S3D_SAMPLE scnview */
  if((mask & S3D_SAMPLE) != 0) {
    res = scene_view_compute_cdf(scnview);
    if(res != RES_OK) goto error;
  }

  /* Compute the scene AABB if it is updated */
  if(scnview->aabb_update) {
    if((mask & S3D_TRACE) == 0) {
      /* Compute from scratch */
      f3_splat(scnview->lower, FLT_MAX);
      f3_splat(scnview->upper,-FLT_MAX);
      scene_view_compute_scene_aabb(scnview);
    } else {
      cus3d_bvh_get_bounds(scnview->bvh, scnview->lower, scnview->upper);
    }
    scnview->aabb_update = 0;
  }

  /* Setup the scene for the scene_primitive_id/S3D_GET_PRIMITIVE scnview */
  res = scene_view_compute_nprims_cdf(scnview, (mask & S3D_GET_PRIMITIVE)!=0);
  if(res != RES_OK) goto error;

  scnview->mask = mask;

exit:
  return res;
error:
  goto exit;
}

static res_T
scene_view_create(struct s3d_scene* scn, struct s3d_scene_view** out_scnview)
{
  struct s3d_scene_view* scnview = NULL;
  res_T res = RES_OK;
  ASSERT(scn && out_scnview);

  if(!is_list_empty(&scn->scnviews)) {
    /* Retrieve an already allocated scnview */
    scnview = CONTAINER_OF(list_head(&scn->scnviews), struct s3d_scene_view, node);
    list_del(&scnview->node);
    ref_get(&scnview->ref);
  } else {
    scnview = (struct s3d_scene_view*)MEM_CALLOC
      (scn->dev->allocator, 1, sizeof(struct s3d_scene_view));
    if(!scnview) {
      res = RES_MEM_ERR;
      goto error;
    }
    list_init(&scnview->node);
    htable_geom_init(scn->dev->allocator, &scnview->cached_geoms);
    darray_fltui_init(scn->dev->allocator, &scnview->cdf);
    darray_nprims_cdf_init(scn->dev->allocator, &scnview->nprims_cdf);
    htable_instview_init(scn->dev->allocator, &scnview->instviews);
    darray_uint_init(scn->dev->allocator, &scnview->detached_shapes);
    f3_splat(scnview->lower, FLT_MAX);
    f3_splat(scnview->upper,-FLT_MAX);
    ref_init(&scnview->ref);
    scnview->build_quality = (int)CUS3D_BUILD_MEDIUM;
    scnview->gpu_dirty = 1;
    scnview->geom_store = NULL;
    scnview->bvh = NULL;

    CLBK_INIT(&scnview->on_shape_detach_cb);
    CLBK_SETUP(&scnview->on_shape_detach_cb, on_shape_detach, scnview);
    SIG_CONNECT_CLBK(&scn->sig_shape_detach, &scnview->on_shape_detach_cb);
  }
  S3D(scene_ref_get(scn));
  scnview->scn = scn;
exit:
  *out_scnview = scnview;
  return res;
error:
  if(scnview) {
    S3D(scene_view_ref_put(scnview));
    scnview = NULL;
  }
  goto exit;
}

static void
scene_view_release(ref_T* ref)
{
  struct htable_instview_iterator it_view, end_view;
  struct htable_geom_iterator it_geom, end_geom;
  struct s3d_scene_view* scnview = CONTAINER_OF(ref, struct s3d_scene_view, ref);
  size_t i, n;
  ASSERT(ref);

  /* Release the scnview of the instances */
  htable_instview_begin(&scnview->instviews, &it_view);
  htable_instview_end(&scnview->instviews, &end_view);
  while(!htable_instview_iterator_eq(&it_view, &end_view)) {
    struct s3d_scene_view* view = *htable_instview_iterator_data_get(&it_view);
    htable_instview_iterator_next(&it_view);
    S3D(scene_view_ref_put(view));
  }
  htable_instview_clear(&scnview->instviews);

  /* Reset the scnview of the cached instance. Note that this step is actually
   * not necessary but this reset is "appreciated" in debug */
  htable_geom_begin(&scnview->cached_geoms, &it_geom);
  htable_geom_end(&scnview->cached_geoms, &end_geom);
  while(!htable_geom_iterator_eq(&it_geom, &end_geom)) {
    struct geometry* geom = *htable_geom_iterator_data_get(&it_geom);
    htable_geom_iterator_next(&it_geom);
    if(geom->type != GEOM_INSTANCE) continue;
    geom->data.instance->scnview = NULL;
  }

  /* Remove the geometry of the shapes detached while the scnview was active */
  n = darray_uint_size_get(&scnview->detached_shapes);
  FOR_EACH(i, 0, n) {
    const unsigned shape_id = darray_uint_cdata_get(&scnview->detached_shapes)[i];
    struct geometry** pgeom = htable_geom_find(&scnview->cached_geoms, &shape_id);
    struct geometry* geom = *pgeom;
    size_t tmp; (void)tmp;
    scene_view_destroy_geometry_cu(scnview, geom);
    tmp = htable_geom_erase(&scnview->cached_geoms, &shape_id);
    ASSERT(tmp == 1);
    scnview->aabb_update = 1;
  }
  darray_uint_clear(&scnview->detached_shapes);

  darray_fltui_clear(&scnview->cdf);
  darray_nprims_cdf_clear(&scnview->nprims_cdf);
  scnview->mask = 0;
  scnview->gpu_dirty = 1;

  /* Do not physically release the memory space of the scnview. Add it to the
   * available scnviews pool of the scene */
  list_add(&scnview->scn->scnviews, &scnview->node);
  S3D(scene_ref_put(scnview->scn));
}

/*******************************************************************************
 * Exported functions
 ******************************************************************************/
res_T
s3d_scene_view_create
  (struct s3d_scene* scn,
   const int mask,
   struct s3d_scene_view** out_scnview)
{
  return s3d_scene_view_create2
    (scn, mask, &S3D_ACCEL_STRUCT_CONF_DEFAULT, out_scnview);
}

res_T
s3d_scene_view_create2
  (struct s3d_scene* scn,
   const int mask,
   const struct s3d_accel_struct_conf* cfg,
   struct s3d_scene_view** out_scnview)
{
  struct s3d_scene_view* scnview = NULL;
  const struct s3d_accel_struct_conf* accel_struct_conf = cfg;
  res_T res = RES_OK;

  if(!scn || !out_scnview) {
    res = RES_BAD_ARG;
    goto error;
  }

  if(!accel_struct_conf && (mask & S3D_TRACE)) {
    accel_struct_conf = &S3D_ACCEL_STRUCT_CONF_DEFAULT;
  }

  res = scene_view_create(scn, &scnview);
  if(res != RES_OK) goto error;

  res = scene_view_sync(scnview, mask, accel_struct_conf);
  if(res != RES_OK) goto error;

exit:
  if(out_scnview) *out_scnview = scnview;
  return res;
error:
  if(scnview) {
    S3D(scene_view_ref_put(scnview));
    scnview = NULL;
  }
  goto exit;
}

res_T
s3d_scene_view_ref_get(struct s3d_scene_view* scnview)
{
  if(!scnview) return RES_BAD_ARG;
  ref_get(&scnview->ref);
  return RES_OK;
}

res_T
s3d_scene_view_ref_put(struct s3d_scene_view* scnview)
{
  if(!scnview) return RES_BAD_ARG;
  ref_put(&scnview->ref, scene_view_release);
  return RES_OK;
}

res_T
s3d_scene_view_get_mask(struct s3d_scene_view* scnview, int* mask)
{
  if(!scnview || !mask) return RES_BAD_ARG;
  *mask = scnview->mask;
  return RES_OK;
}

res_T
s3d_scene_view_sample
  (struct s3d_scene_view* scnview,
   const float u,
   const float v,
   const float w,
   struct s3d_primitive* primitive, /* sampled primitive */
   float st[2])
{
  struct geometry** pgeom;
  struct geometry* geom;
  size_t sz;
  size_t i;
  const struct fltui* fltui, *fltui_found;
  const float* flt, *flt_found;
  unsigned ishape;
  float f;
  res_T res = RES_OK;

  if(!scnview || !primitive || !st) {
    res = RES_BAD_ARG;
    goto error;
  }
  /* Expecting canonic numbers */
  if(u < 0.f || u >= 1.f || v < 0.f || v >= 1.f || w < 0.f || w >= 1.f) {
    log_error(scnview->scn->dev,
      "%s: the submitted numbers are not canonical, i.e. they are not in [0, 1[.\n",
      FUNC_NAME);
    res = RES_BAD_ARG;
    goto error;
  }
  if((scnview->mask & S3D_SAMPLE) == 0) {
    log_error(scnview->scn->dev,
      "%s: no active S3D_SAMPLE scnview on the submitted scene.\n",
      FUNC_NAME);
    res = RES_BAD_OP;
    goto error;
  }

  /* Find the sampled geometry */
  if(darray_fltui_size_get(&scnview->cdf) == 0) {
    /* No geometry to sample */
    *primitive = S3D_PRIMITIVE_NULL;
    goto exit;
  } else if(darray_fltui_size_get(&scnview->cdf) == 1) {
    ishape = darray_fltui_cdata_get(&scnview->cdf)[0].ui;
    /* Map u to the CDF bounds */
    f = u * darray_fltui_cdata_get(&scnview->cdf)[0].flt;
  } else {
    fltui = darray_fltui_cdata_get(&scnview->cdf);
    sz = darray_fltui_size_get(&scnview->cdf);
    f = u * fltui[sz-1].flt; /* Map u to [0, SceneArea[ */
    fltui_found = static_cast<const struct fltui*>(search_lower_bound(&f, fltui, sz, sizeof(*fltui), cmp_float_to_fltui));
    ASSERT(fltui_found);

    /* search_lower_bound returns the first entry that is not less than `f'.
     * The following code discards entries that are also `equal' to `f' */
    i = (size_t)(fltui_found - fltui);
    while(fltui[i].flt == f && i < sz) ++i;
    ASSERT(i < sz);

    fltui_found = fltui + i;
    ishape = fltui_found->ui;

    /* Map f to [0, <Shape|Instance>Area[ */
    if(i) f -= fltui_found[-1].flt;
  }
  pgeom = htable_geom_find(&scnview->cached_geoms, &ishape);
  ASSERT(pgeom);
  geom = *pgeom;

  if(geom->type == GEOM_MESH || geom->type == GEOM_SPHERE) {
    primitive->inst__ = NULL;
    primitive->inst_id = S3D_INVALID_ID;
    primitive->scene_prim_id = 0;
  } else {
    /* Find the sampled instantiated geometry */
    ASSERT(geom->type == GEOM_INSTANCE);
    primitive->inst__ = geom;
    primitive->inst_id = geom->name;
    primitive->scene_prim_id = geom->scene_prim_id_offset;
    if(darray_fltui_size_get(&geom->data.instance->scnview->cdf) == 1) {
      ishape = darray_fltui_cdata_get(&geom->data.instance->scnview->cdf)[0].ui;
    } else {
      fltui = darray_fltui_cdata_get(&geom->data.instance->scnview->cdf);
      sz = darray_fltui_size_get(&geom->data.instance->scnview->cdf);
      fltui_found = static_cast<const struct fltui*>(search_lower_bound(&f, fltui, sz, sizeof(*fltui), cmp_float_to_fltui));
      ASSERT(fltui_found);

      /* search_lower_bound returns the first entry that is not less than `f'.
       * The following code discards entries that are also `equal' to `f' */
      i = (size_t)(fltui_found - fltui);
      while(fltui[i].flt == f && i < sz) ++i;
      ASSERT(i < sz);

      fltui_found = fltui + i;
      ishape = fltui_found->ui;

      /* Map `f' to [0, ShapeArea[ */
      if(i) f -= fltui_found[-1].flt;
    }
    pgeom = htable_geom_find(&geom->data.instance->scnview->cached_geoms, &ishape);
    ASSERT(pgeom);
    geom = *pgeom;
  }

  /* Find the sampled primitive */
  primitive->shape__ = geom;
  primitive->geom_id = geom->name;
  primitive->scene_prim_id += geom->scene_prim_id_offset;
  if(geom->type == GEOM_SPHERE) {
    primitive->prim_id = 0;
  } else if(geom->type == GEOM_MESH) {
    flt = darray_float_cdata_get(&geom->data.mesh->cdf);
    sz = darray_float_size_get(&geom->data.mesh->cdf);
    flt_found = static_cast<const float*>(search_lower_bound(&f, flt, sz, sizeof(*flt), cmp_float));
    ASSERT(flt_found != NULL);
    i = (size_t)(flt_found - flt);

    /* search_lower_bound returns the first entry that is not less than `f'.
     * The following code discards entries that are also `equal' to `f' */
    while(flt[i] == f && i < sz) ++i;
    ASSERT(i < sz);

    primitive->prim_id = (unsigned)i;

  } else {
    FATAL("Unreachable code\n");
  }
  primitive->scene_prim_id += primitive->prim_id;
  S3D(primitive_sample(primitive, v, w, st));

exit:
  return res;
error:
  goto exit;
}

res_T
s3d_scene_view_get_primitive
  (struct s3d_scene_view* scnview,
   const unsigned iprim,
   struct s3d_primitive* prim)
{
  struct geometry** pgeom;
  struct geometry* geom;
  const struct nprims_cdf* begin, *found;
  size_t sz;
  size_t nprims;
  unsigned ishape;
  size_t i;
  res_T res = RES_OK;

  if(!scnview || !prim) {
    res = RES_BAD_ARG;
    goto error;
  }
  if((scnview->mask & S3D_GET_PRIMITIVE) == 0) {
    log_error(scnview->scn->dev,
      "%s: no active S3D_GET_PRIMITIVE scnview on the submitted scene.\n",
      FUNC_NAME);
    res = RES_BAD_OP;
    goto error;
  }
  S3D(scene_view_primitives_count(scnview, &nprims));
  if(iprim  >= nprims) {
    log_error(scnview->scn->dev,
      "%s: the primitive index %u exceeds the number of scene primitives %u.\n",
      FUNC_NAME, iprim, (unsigned)nprims);
    res = RES_BAD_ARG;
    goto error;
  }

  i = iprim;
  if(darray_nprims_cdf_size_get(&scnview->nprims_cdf) == 1) {
    ishape = darray_nprims_cdf_cdata_get(&scnview->nprims_cdf)[0].ishape;
  } else {
    begin = darray_nprims_cdf_cdata_get(&scnview->nprims_cdf);
    sz = darray_nprims_cdf_size_get(&scnview->nprims_cdf);
    found = static_cast<const struct nprims_cdf*>(search_lower_bound(&i, begin, sz, sizeof(*begin), cmp_size_t_to_nprims_cdf));
    ASSERT(found != NULL);
    ishape = found->ishape;
    if(found != begin) {
      ASSERT(i >= found[-1].nprims);
      i -= found[-1].nprims;
    }
  }
  pgeom = htable_geom_find(&scnview->cached_geoms, &ishape);
  ASSERT(pgeom);
  geom = *pgeom;

  if(geom->type == GEOM_MESH || geom->type == GEOM_SPHERE) {
    prim->inst__ = NULL;
    prim->inst_id = S3D_INVALID_ID;
    prim->scene_prim_id = 0;
  } else {
    ASSERT(geom->type == GEOM_INSTANCE);
    prim->inst__ = geom;
    prim->inst_id = geom->name;
    prim->scene_prim_id = geom->scene_prim_id_offset;
    if(darray_nprims_cdf_size_get(&geom->data.instance->scnview->nprims_cdf)==1) {
      ishape = darray_nprims_cdf_cdata_get
        (&geom->data.instance->scnview->nprims_cdf)[0].ishape;
    } else {
      begin = darray_nprims_cdf_cdata_get
        (&geom->data.instance->scnview->nprims_cdf);
      sz = darray_nprims_cdf_size_get
        (&geom->data.instance->scnview->nprims_cdf);
      found = static_cast<const struct nprims_cdf*>(search_lower_bound(&i, begin, sz, sizeof(*begin), cmp_size_t_to_nprims_cdf));
      ASSERT(found != NULL);
      ishape = found->ishape;
      if(found != begin) {
        ASSERT(i >= found[-1].nprims);
        i -= found[-1].nprims;
      }
    }
    pgeom = htable_geom_find(&geom->data.instance->scnview->cached_geoms, &ishape);
    ASSERT(pgeom);
    geom = *pgeom;
  }
  ASSERT(geom->type == GEOM_MESH || geom->type == GEOM_SPHERE);
  ASSERT(geom->type != GEOM_MESH || i < mesh_get_ntris(geom->data.mesh));
  prim->shape__ = geom;
  prim->geom_id = geom->name;
  prim->prim_id = (unsigned)i;
  prim->scene_prim_id += geom->scene_prim_id_offset;
  prim->scene_prim_id += prim->prim_id;

exit:
  return res;
error:
  goto exit;
}

res_T
s3d_scene_view_primitives_count(struct s3d_scene_view* scnview, size_t* prims_count)
{
  res_T res = RES_OK;

  if(!scnview || !prims_count) {
    res = RES_BAD_ARG;
    goto error;
  }
  if((scnview->mask & S3D_GET_PRIMITIVE) != 0) {
    const size_t len = darray_nprims_cdf_size_get(&scnview->nprims_cdf);
    if(!len) {
      *prims_count = 0;
    } else {
      *prims_count = darray_nprims_cdf_cdata_get
        (&scnview->nprims_cdf)[len - 1].nprims;
    }
  } else {
    struct htable_geom_iterator it, end;
    size_t inst_count;

    htable_geom_begin(&scnview->cached_geoms, &it);
    htable_geom_end(&scnview->cached_geoms, &end);
    *prims_count = 0;
    while(!htable_geom_iterator_eq(&it, &end)) {
      struct geometry** pgeom = htable_geom_iterator_data_get(&it);
      struct geometry* geom = *pgeom;
      htable_geom_iterator_next(&it);

      if(!geom->is_enabled) continue;

      switch(geom->type) {
        case GEOM_MESH:
          *prims_count += mesh_get_ntris(geom->data.mesh);
          break;
        case GEOM_SPHERE:
          *prims_count += 1;
          break;
        case GEOM_INSTANCE:
          S3D(scene_view_primitives_count(geom->data.instance->scnview, &inst_count));
          *prims_count += inst_count;
          break;
        default: FATAL("Unreachable code\n"); break;
      }
    }
  }
exit:
  return res;
error:
  goto exit;
}

res_T
s3d_scene_view_compute_area(struct s3d_scene_view* scnview, float* out_area)
{
  float area;
  res_T res = RES_OK;

  if(!scnview || !out_area) {
    res = RES_BAD_ARG;
    goto error;
  }
  if((scnview->mask & S3D_SAMPLE) != 0) {
    /* Retrieve the overall scene area from the scene cumulative distribution
     * function */
    size_t len = darray_fltui_size_get(&scnview->cdf);
    if(!len) {
      area = 0.f;
    } else {
      area = darray_fltui_cdata_get(&scnview->cdf)[len - 1].flt;
    }
  } else {
    struct htable_geom_iterator it, end;
    float inst_area;

    htable_geom_begin(&scnview->cached_geoms, &it);
    htable_geom_end(&scnview->cached_geoms, &end);

    area = 0.f;
    while(!htable_geom_iterator_eq(&it, &end)) {
      struct geometry** pgeom = htable_geom_iterator_data_get(&it);
      struct geometry* geom = *pgeom;

      htable_geom_iterator_next(&it);

      if(!geom->is_enabled) continue;

      switch(geom->type) {
        case GEOM_MESH:
          area += mesh_compute_area(geom->data.mesh);
          break;
        case GEOM_SPHERE:
          area += sphere_compute_area(geom->data.sphere);
          break;
        case GEOM_INSTANCE:
          /* TODO take into account the instance scale factor */
          S3D(scene_view_compute_area(geom->data.instance->scnview, &inst_area));
          area += inst_area;
          break;
        default: FATAL("Unreachable code\n"); break;
      }
    }
  }

exit:
  if(out_area) *out_area = area;
  return res;
error:
  area = -1.f;
  goto exit;
}

res_T
s3d_scene_view_compute_volume(struct s3d_scene_view* scnview, float* out_volume)
{
  if(!scnview || !out_volume) return RES_BAD_ARG;
  *out_volume = scene_view_compute_volume(scnview, 0/*No initial flip_surface*/);
  return RES_OK;
}

res_T
s3d_scene_view_get_aabb(struct s3d_scene_view* scnview, float lower[3], float upper[3])
{
  if(!scnview || !lower || !upper) return RES_BAD_ARG;
  f3_set(lower, scnview->lower);
  f3_set(upper, scnview->upper);
  return RES_OK;
}

/*******************************************************************************
 * Local functions
 ******************************************************************************/
void
scene_view_destroy(struct s3d_scene_view* scnview)
{
  struct htable_geom_iterator it, end;
  ASSERT(scnview && !is_list_empty(&scnview->node)/*Not in use*/);
  ASSERT(scnview->mask == 0);

  /* Delete the cached geometries */
  htable_geom_begin(&scnview->cached_geoms, &it);
  htable_geom_end(&scnview->cached_geoms, &end);
  while(!htable_geom_iterator_eq(&it, &end)) {
    struct geometry** pgeom = htable_geom_iterator_data_get(&it);
    struct geometry* geom = *pgeom;
    scene_view_destroy_geometry_cu(scnview, geom);
    htable_geom_iterator_next(&it);
  }

  if(scnview->geom_store) {
    cus3d_geom_store_destroy(scnview->geom_store, scnview->scn->dev->gpu);
    scnview->geom_store = NULL;
  }
  if(scnview->bvh) {
    cus3d_bvh_destroy(scnview->bvh, scnview->scn->dev->gpu);
    scnview->bvh = NULL;
  }

  /* Release internal data structure */
  htable_geom_release(&scnview->cached_geoms);
  darray_fltui_release(&scnview->cdf);
  darray_nprims_cdf_release(&scnview->nprims_cdf);
  htable_instview_release(&scnview->instviews);
  darray_uint_release(&scnview->detached_shapes);

  /* Remove the scnview from its pool */
  list_del(&scnview->node);
  CLBK_DISCONNECT(&scnview->on_shape_detach_cb);

  /* Free the scnview memory space */
  MEM_RM(scnview->scn->dev->allocator, scnview);
}

