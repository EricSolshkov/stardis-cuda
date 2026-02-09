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

#include "s3d_device_c.h"
#include "s3d_geometry.h"
#include "s3d_instance.h"
#include "s3d_mesh.h"
#include "s3d_sphere.h"

#include <rsys/mem_allocator.h>

/*******************************************************************************
 * Helper functions
 ******************************************************************************/
static void
geometry_release(ref_T* ref)
{
  struct geometry* geom;
  struct s3d_device* dev;

  geom = CONTAINER_OF(ref, struct geometry, ref);
  dev = geom->dev;
  switch(geom->type) {
    case GEOM_MESH:
      if(geom->data.mesh) mesh_ref_put(geom->data.mesh);
      break;
    case GEOM_INSTANCE:
      if(geom->data.instance) instance_ref_put(geom->data.instance);
      break;
    case GEOM_SPHERE:
      if(geom->data.sphere) sphere_ref_put(geom->data.sphere);
      break;
    default: FATAL("Unreachable code\n"); break;
  }
  MEM_RM(dev->allocator, geom);
  S3D(device_ref_put(dev));
}

/*******************************************************************************
 * Local functions
 ******************************************************************************/
res_T
geometry_create
  (struct s3d_device* dev,
   struct geometry** out_geom)
{
  struct geometry* geom = NULL;
  res_T res = RES_OK;
  ASSERT(dev && out_geom);

  geom = (struct geometry*)MEM_CALLOC
    (dev->allocator, 1, sizeof(struct geometry));
  if(!geom) {
    res = RES_MEM_ERR;
    goto error;
  }
  ref_init(&geom->ref);
  S3D(device_ref_get(dev));
  geom->dev = dev;
  geom->name = S3D_INVALID_ID;
  geom->flip_surface = 0;
  geom->is_enabled = 1;
  geom->type = GEOM_NONE;
  geom->data.mesh = NULL;

exit:
  *out_geom = geom;
  return res;
error:
  if(geom) {
    geometry_ref_put(geom);
    geom = NULL;
  }
  goto exit;
}

void
geometry_ref_get(struct geometry* geom)
{
  ASSERT(geom);
  ref_get(&geom->ref);
}

void
geometry_ref_put(struct geometry* geom)
{
  ASSERT(geom);
  ref_put(&geom->ref, geometry_release);
}

