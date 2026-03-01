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

/* P1: Dispatch-layer SoA arrays — mirror of hot fields from path_state.
 *
 * These arrays are indexed by slot_id [0..pool_size).  Values are kept
 * in sync with the corresponding path_state fields by the dispatch layer.
 *
 * Step functions continue to use struct path_state* and do NOT touch
 * these arrays directly.  Sync happens at well-defined boundaries:
 *   - After cascade:   phase, active, needs_ray  -> written back to SoA
 *   - After init:      all 5 fields              -> written to SoA
 *   - Before collect:  read from SoA (not path_state)
 *   - Before compact:  read from SoA (not path_state)
 */

#ifndef SDIS_WF_SOA_H
#define SDIS_WF_SOA_H

#include "sdis_wf_types.h"   /* enum path_phase, enum ray_bucket_type */
#include "sdis_wf_state.h"   /* struct path_state */
#include "sdis.h"            /* INLINE, res_T */
#include <stdint.h>

/*******************************************************************************
 * dispatch_soa -- SoA mirror of dispatch-hot fields from path_state
 ******************************************************************************/
struct dispatch_soa {
  enum path_phase*        phase;          /* [pool_size] */
  int*                    active;         /* [pool_size] */
  int*                    needs_ray;      /* [pool_size] */
  enum ray_bucket_type*   ray_bucket;     /* [pool_size] */
  int*                    ray_count_ext;  /* [pool_size] */
  size_t                  count;          /* = pool_size */
};

/* Lifecycle */
extern res_T dispatch_soa_alloc(struct dispatch_soa* soa, size_t pool_size);
extern void  dispatch_soa_free (struct dispatch_soa* soa);

/*******************************************************************************
 * Single-slot sync: path_state -> SoA (after init / after cascade per-path)
 ******************************************************************************/
static INLINE void
dispatch_soa_sync_from_path(struct dispatch_soa* soa,
                            uint32_t idx,
                            const struct path_state* p)
{
  soa->phase[idx]         = p->phase;
  soa->active[idx]        = p->active;
  soa->needs_ray[idx]     = p->needs_ray;
  soa->ray_bucket[idx]    = p->ray_bucket;
  soa->ray_count_ext[idx] = p->ray_count_ext;
}

/*******************************************************************************
 * Single-slot sync: SoA -> path_state (if needed before step_* that reads)
 ******************************************************************************/
static INLINE void
dispatch_soa_sync_to_path(const struct dispatch_soa* soa,
                          uint32_t idx,
                          struct path_state* p)
{
  p->phase         = soa->phase[idx];
  p->active        = soa->active[idx];
  p->needs_ray     = soa->needs_ray[idx];
  p->ray_bucket    = soa->ray_bucket[idx];
  p->ray_count_ext = soa->ray_count_ext[idx];
}

/*******************************************************************************
 * Debug consistency check: verify SoA matches AoS for a single slot.
 * Active only in debug builds (NDEBUG not defined).
 ******************************************************************************/
#ifndef NDEBUG
static INLINE void
dispatch_soa_assert_consistent(const struct dispatch_soa* soa,
                               uint32_t idx,
                               const struct path_state* p)
{
  ASSERT(soa->phase[idx]         == p->phase);
  ASSERT(soa->active[idx]        == p->active);
  ASSERT(soa->needs_ray[idx]     == p->needs_ray);
  ASSERT(soa->ray_bucket[idx]    == p->ray_bucket);
  ASSERT(soa->ray_count_ext[idx] == p->ray_count_ext);
}
#else
#define dispatch_soa_assert_consistent(soa, idx, p) ((void)0)
#endif

#endif /* SDIS_WF_SOA_H */
