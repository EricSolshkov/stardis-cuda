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

#ifndef S3D_C_H
#define S3D_C_H

#include "s3d.h"

#include <rsys/rsys.h>

/* Filter function and its associated user defined data */
struct hit_filter {
  s3d_hit_filter_function_T func;
  void* data;
};

static INLINE unsigned
s3d_type_get_dimension(const enum s3d_type type)
{
  switch(type) {
    case S3D_FLOAT: return 1;
    case S3D_FLOAT2: return 2;
    case S3D_FLOAT3: return 3;
    case S3D_FLOAT4: return 4;
    default: FATAL("Unreachable code\n"); break;
  }
}

#endif /* S3D_C_H */

