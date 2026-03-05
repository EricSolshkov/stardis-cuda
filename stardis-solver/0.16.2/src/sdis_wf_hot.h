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

/* P0_OPT: path_hot -- compact 8-byte AoS of the 5 hottest dispatch fields.
 *
 * Replaces the P1 dispatch_soa (5 independent SoA arrays + sync layer)
 * with a single contiguous array of 8-byte structs.  Step functions
 * receive a `struct path_hot*` parameter and read/write directly --
 * no sync points needed.
 *
 * Memory footprint: pool_size * 8B  (e.g. 16384 * 8 = 128 KB, fits L2).
 */

#ifndef SDIS_WF_HOT_H
#define SDIS_WF_HOT_H

#include "sdis.h"            /* res_T */
#include <stdint.h>
#include <stddef.h>          /* size_t */

/*******************************************************************************
 * path_hot -- compact dispatch-hot fields (8 bytes, 8 slots / cache line)
 *
 * Extracted from path_state.  Step functions write hot->phase etc. directly.
 ******************************************************************************/
struct path_hot {
    uint8_t  phase;          /* enum path_phase,      max ~50  */
    uint8_t  active;         /* 0 or 1                         */
    uint8_t  needs_ray;      /* 0 or 1                         */
    uint8_t  ray_bucket;     /* enum ray_bucket_type,  max 5   */
    uint8_t  ray_count_ext;  /* {0, 1, 2, 4, 6}                */
    uint8_t  _pad[3];        /* -> 8B total, natural alignment  */
};

/* Compile-time size assertion (C89-friendly) */
typedef char path_hot_size_check_[(sizeof(struct path_hot) == 8) ? 1 : -1];

/* Lifecycle */
extern res_T path_hot_arr_alloc(struct path_hot** out, size_t pool_size);
extern void  path_hot_arr_free (struct path_hot** arr);

#endif /* SDIS_WF_HOT_H */
