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

#ifndef S3D_GEOMETRY_H
#define S3D_GEOMETRY_H

#include "s3d.h"
#include <rsys/float3.h>
#include <rsys/ref_count.h>

enum geometry_type {
  GEOM_MESH,
  GEOM_INSTANCE,
  GEOM_SPHERE,
  GEOM_TYPES_COUNT__,
  GEOM_NONE = GEOM_TYPES_COUNT__
};

struct geometry {
  unsigned name;
  unsigned scene_prim_id_offset;

  char flip_surface;
  char is_enabled;

  enum geometry_type type;
  union {
    struct instance* instance;
    struct mesh* mesh;
    struct sphere* sphere;
  } data;

  struct s3d_device* dev;
  ref_T ref;
};

extern LOCAL_SYM res_T
geometry_create
  (struct s3d_device* dev,
   struct geometry** geom);

extern LOCAL_SYM void
geometry_ref_get
  (struct geometry* geometry);

extern LOCAL_SYM void
geometry_ref_put
  (struct geometry* geometry);

#endif /* S3D_GEOMETRY_H */
