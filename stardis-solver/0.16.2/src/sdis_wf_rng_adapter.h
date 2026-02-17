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

/* Thin ssp_rng adapter for per-path CBRNG (wf_rng).
 *
 * Provides C-linkage functions to create/destroy ssp_rng instances that
 * internally delegate to wf_rng.  This allows existing code that calls
 * ssp_rng_canonical(p->rng) to transparently use per-path CBRNG without
 * any call-site modifications.
 *
 * The adapter is implemented in C++ (sdis_wf_rng_adapter.cpp) because
 * struct ssp_rng's full definition requires C++ headers (ssp_rng_c.h).
 *
 * Reference: debug_issues/block_firefly_noise/fix_per_path_cbrng.md §4.1
 */

#ifndef SDIS_WF_RNG_ADAPTER_H
#define SDIS_WF_RNG_ADAPTER_H

#include <rsys/rsys.h>  /* res_T */
#include <stddef.h>     /* size_t */

/* Forward declarations (C-compatible) */
struct ssp_rng;
struct wf_rng;

#ifdef __cplusplus
extern "C" {
#endif

/* Create per-path CBRNG thin wrappers and populate slot_rngs.
 *
 * Allocates pool_size thin ssp_rng instances internally.  Each wrapper's
 * desc.get function delegates to wf_rng_get() on the corresponding
 * wf_rng state.
 *
 * On success, slot_rngs[0..pool_size-1] are set to point to the thin
 * wrappers, and *out_storage receives an opaque handle that must be freed
 * with wf_rng_destroy_thin_ssp_rngs().  Individual slot_rngs entries
 * must NOT be passed to ssp_rng_ref_put().
 *
 * @param pool_size    Number of slots in the wavefront pool.
 * @param rng_states   Array of wf_rng pointers [pool_size] — each entry
 *                     is &slots[i].rng_state.
 * @param slot_rngs    Pointer array [pool_size] to populate.
 * @param out_storage  Receives opaque handle for the allocated thin RNG array.
 * @return RES_OK on success, RES_MEM_ERR on allocation failure.
 */
extern LOCAL_SYM res_T
wf_rng_create_thin_ssp_rngs(size_t pool_size,
                             struct wf_rng** rng_states,
                             struct ssp_rng** slot_rngs,
                             struct ssp_rng** out_storage);

/* Destroy the thin ssp_rng array created by wf_rng_create_thin_ssp_rngs().
 *
 * Frees the contiguous array.  Does NOT call ssp_rng_ref_put (the state
 * pointers target embedded wf_rng fields, not heap-allocated memory).
 *
 * @param rngs       Array returned by wf_rng_create_thin_ssp_rngs, or NULL.
 */
extern LOCAL_SYM void
wf_rng_destroy_thin_ssp_rngs(struct ssp_rng* rngs);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* SDIS_WF_RNG_ADAPTER_H */
