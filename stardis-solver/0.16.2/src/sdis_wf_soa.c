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

/* P1: Dispatch-layer SoA arrays — alloc / free implementation. */

#include "sdis_wf_soa.h"
#include <stdlib.h>
#include <string.h>

res_T
dispatch_soa_alloc(struct dispatch_soa* soa, size_t pool_size)
{
  ASSERT(soa && pool_size > 0);
  memset(soa, 0, sizeof(*soa));
  soa->count = pool_size;

  soa->phase = (enum path_phase*)calloc(pool_size, sizeof(enum path_phase));
  if(!soa->phase) goto fail;

  soa->active = (int*)calloc(pool_size, sizeof(int));
  if(!soa->active) goto fail;

  soa->needs_ray = (int*)calloc(pool_size, sizeof(int));
  if(!soa->needs_ray) goto fail;

  soa->ray_bucket = (enum ray_bucket_type*)calloc(
    pool_size, sizeof(enum ray_bucket_type));
  if(!soa->ray_bucket) goto fail;

  soa->ray_count_ext = (int*)calloc(pool_size, sizeof(int));
  if(!soa->ray_count_ext) goto fail;

  return RES_OK;

fail:
  dispatch_soa_free(soa);
  return RES_MEM_ERR;
}

void
dispatch_soa_free(struct dispatch_soa* soa)
{
  if(!soa) return;
  free(soa->phase);
  free(soa->active);
  free(soa->needs_ray);
  free(soa->ray_bucket);
  free(soa->ray_count_ext);
  memset(soa, 0, sizeof(*soa));
}
