/* Copyright (C) 2016-2026 |Méso|Star> (contact@meso-star.com)
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

/* Thin ssp_rng adapter for per-path CBRNG — C++ implementation.
 *
 * This file MUST be compiled as C++ because it includes ssp_rng_c.h which
 * requires C++ headers (<random>, Random123/conventional/Engine.hpp).
 *
 * It creates ssp_rng instances whose desc.get function pointer delegates
 * to wf_rng_get() on the wf_rng embedded in each path_state slot.
 * This makes all existing ssp_rng_canonical(p->rng) calls transparently
 * use per-path independent CBRNG without any call-site changes.
 *
 * Reference: debug_issues/block_firefly_noise/fix_per_path_cbrng.md §4.1
 */

/* Prevent Windows min/max macro pollution */
#ifdef _MSC_VER
#  ifndef NOMINMAX
#    define NOMINMAX
#  endif
#endif

#include "ssp_rng_c.h"       /* struct ssp_rng, struct rng_desc (C++ only)  */
#include "sdis_wf_rng.h"     /* struct wf_rng, wf_rng_get()                 */

#include <rsys/ref_count.h>  /* ref_init                                     */
#include <stdlib.h>          /* calloc, free                                 */
#include <string.h>          /* memset                                       */
#include <stdint.h>          /* UINT64_MAX                                   */
#include <limits.h>

/* ========================================================================= */
/* Adapter get() — called via ssp_rng->desc.get(ssp_rng->state)             */
/* ========================================================================= */
static uint64_t
wf_rng_adapter_get(void* state)
{
  return wf_rng_get(static_cast<struct wf_rng*>(state));
}

/* ========================================================================= */
/* Public API (extern "C")                                                   */
/* ========================================================================= */

extern "C" res_T
wf_rng_create_thin_ssp_rngs(size_t pool_size,
                             struct wf_rng** rng_states,
                             struct ssp_rng** slot_rngs,
                             struct ssp_rng** out_storage)
{
  struct ssp_rng* arr = NULL;
  size_t i;

  if(!out_storage || !slot_rngs || !rng_states || pool_size == 0) return RES_BAD_ARG;

  arr = static_cast<struct ssp_rng*>(calloc(pool_size, sizeof(struct ssp_rng)));
  if(!arr) return RES_MEM_ERR;

  for(i = 0; i < pool_size; i++) {
    struct ssp_rng* r = &arr[i];

    /* Zero-initialise the full rng_desc; only fill the fields we use */
    memset(&r->desc, 0, sizeof(r->desc));
    r->desc.get          = wf_rng_adapter_get;
    r->desc.min          = 0;
    r->desc.max          = UINT64_MAX;
    r->desc.sizeof_state = sizeof(struct wf_rng);
    r->desc.alignof_state = 8;
    /* desc.init, desc.release, desc.set, desc.discard etc. left NULL —
     * thin wrappers are managed externally, not through ssp_rng lifecycle */

    r->type      = SSP_RNG_THREEFRY;              /* cosmetic */
    r->state     = rng_states[i];                   /* points into path_state */
    r->allocator = NULL;

    /* Precomputed constants for ssp_rng_canonical() fast path.
     * For Threefry: min=0, max=UINT64_MAX
     *   r = max - min + 1 = 2^64
     *   dbl_k = max(1, ceil((53 + 64) / 64)) = 1   (since 117/64 = 1.828)
     *   flt_k = max(1, ceil((24 + 64) / 64)) = 1   (since  88/64 = 1.375)
     * Actually the formula is max(1, (bits + log2(r)) / log2(r)):
     *   dbl_k = max(1, (53 + 64) / 64) = max(1, 1) = 1  (integer division)
     *   flt_k = max(1, (24 + 64) / 64) = max(1, 1) = 1  (integer division)
     */
    r->r     = (long double)UINT64_MAX + 1.0L;    /* = 2^64 */
    r->dbl_k = 1;
    r->flt_k = 1;

    ref_init(&r->ref);

    /* Populate the caller's pointer array */
    slot_rngs[i] = r;
  }

  *out_storage = arr;
  return RES_OK;
}

extern "C" void
wf_rng_destroy_thin_ssp_rngs(struct ssp_rng* rngs)
{
  /* Do NOT call desc.release or free individual state pointers —
   * they point to embedded wf_rng fields inside path_state slots,
   * which are owned by the pool's slots[] array. */
  free(rngs);
}
