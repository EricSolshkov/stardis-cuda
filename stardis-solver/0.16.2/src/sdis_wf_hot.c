/* Copyright (C) 2016-2025 |Méso|Star> (contact@meso-star.com)
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

/* P0_OPT: path_hot array -- alloc / free implementation. */

#include "sdis_wf_hot.h"
#include <stdlib.h>
#include <string.h>

res_T
path_hot_arr_alloc(struct path_hot** out, size_t pool_size)
{
  struct path_hot* arr;
  ASSERT(out && pool_size > 0);

  arr = (struct path_hot*)calloc(pool_size, sizeof(struct path_hot));
  if(!arr) return RES_MEM_ERR;

  *out = arr;
  return RES_OK;
}

void
path_hot_arr_free(struct path_hot** arr)
{
  if(!arr) return;
  free(*arr);
  *arr = NULL;
}
