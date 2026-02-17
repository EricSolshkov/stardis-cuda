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

/* Per-path Counter-Based RNG (CBRNG) using Threefry4x64.
 *
 * Each Monte-Carlo path in the wavefront pool gets its own independent
 * random number stream keyed by (pixel_x, pixel_y, spp_idx, global_seed).
 * This eliminates the RNG sharing / interleaving problem that causes
 * 32x32-pixel block noise patterns in wavefront rendering.
 *
 * The implementation uses Random123's Threefry4x64-20 in counter mode:
 *   - 4x64-bit key: uniquely identifies each path
 *   - 4x64-bit counter: incremented every 4 draws
 *   - Each threefry4x64() call produces 4 uint64 random numbers
 *
 * All functions are static inline for zero call overhead.
 *
 * Reference: debug_issues/block_firefly_noise/fix_per_path_cbrng.md
 */

#ifndef SDIS_WF_RNG_H
#define SDIS_WF_RNG_H

#include <rsys/rsys.h>           /* INLINE, FINLINE, ASSERT              */
#include <Random123/threefry.h>  /* threefry4x64, threefry4x64_key_t etc */
#include <stdint.h>
#include <string.h>

/*******************************************************************************
 * struct wf_rng — inline CBRNG state (100 bytes)
 *
 * Embedded directly in path_state to avoid extra heap allocation.
 ******************************************************************************/
struct wf_rng {
  threefry4x64_key_t key;    /* 32B: determined by (px, py, spp_idx, seed)  */
  threefry4x64_ctr_t ctr;    /* 32B: incremented per 4 draws                */
  uint64_t           buf[4]; /* 32B: current threefry4x64() output buffer   */
  int                buf_idx;/* 4B:  buffer consumption position 0..3,
                              *      >=4 means buffer needs refresh          */
};
/* Total: 32 + 32 + 32 + 4 = 100 bytes (+ padding to 104 for alignment)    */

/*******************************************************************************
 * wf_rng_seed — initialise RNG for a specific path
 *
 * Each unique (px, py, spp_idx, global_seed) tuple produces a statistically
 * independent random number stream.  Threefry4x64's 20-round mixing provides
 * cryptographic-strength independence between different keys.
 ******************************************************************************/
static INLINE void
wf_rng_seed(struct wf_rng* r,
            uint32_t px, uint32_t py, uint32_t spp_idx,
            uint64_t global_seed)
{
  memset(r, 0, sizeof(*r));
  r->key.v[0] = ((uint64_t)py << 32) | (uint64_t)px;
  r->key.v[1] = (uint64_t)spp_idx;
  r->key.v[2] = global_seed;
  r->key.v[3] = 0;  /* reserved for future extension */
  /* ctr starts at 0 (already zeroed by memset) */
  r->buf_idx = 4;   /* force buffer refresh on first draw */
}

/*******************************************************************************
 * wf_rng_get — raw 64-bit random number
 *
 * Returns one uint64_t from the Threefry4x64 stream.  Internally buffers
 * 4 values per threefry4x64() call, so amortised cost is ~1/4 of a full
 * Threefry round-function evaluation.
 ******************************************************************************/
static INLINE uint64_t
wf_rng_get(struct wf_rng* r)
{
  if(r->buf_idx >= 4) {
    threefry4x64_ctr_t out = threefry4x64(r->ctr, r->key);
    r->buf[0] = out.v[0];
    r->buf[1] = out.v[1];
    r->buf[2] = out.v[2];
    r->buf[3] = out.v[3];
    r->buf_idx = 0;
    /* 128-bit counter increment: v[0]++, carry into v[1] on overflow */
    if(++r->ctr.v[0] == 0) r->ctr.v[1]++;
  }
  return r->buf[r->buf_idx++];
}

/*******************************************************************************
 * wf_rng_canonical — uniform double in [0, 1)
 *
 * Mathematically equivalent to ssp_rng_canonical() for Threefry (dbl_k=1):
 *   canonical = (double)(get() - min) / (max - min + 1)
 *             = (double)get() * 2^-64
 *
 * Uses 53-bit mantissa precision (full IEEE 754 double).
 ******************************************************************************/
static INLINE double
wf_rng_canonical(struct wf_rng* r)
{
  /* 0x1p-64 = 1.0 / 2^64 = 5.421010862427522e-20 */
  return (double)wf_rng_get(r) * 0x1p-64;
}

/*******************************************************************************
 * wf_rng_canonical_float — uniform float in [0, 1)
 *
 * Takes the high 24 bits of a uint64 and converts to [0, 1) float.
 * This guarantees uniform distribution over all representable floats
 * in [0, 1) with 24-bit mantissa precision.
 ******************************************************************************/
static INLINE float
wf_rng_canonical_float(struct wf_rng* r)
{
  return (float)(wf_rng_get(r) >> 40) * 0x1p-24f;
}

/*******************************************************************************
 * wf_rng_uniform_double — uniform double in [lo, hi)
 ******************************************************************************/
static INLINE double
wf_rng_uniform_double(struct wf_rng* r, double lo, double hi)
{
  return lo + wf_rng_canonical(r) * (hi - lo);
}

/*******************************************************************************
 * wf_rng_uniform_float — uniform float in [lo, hi)
 ******************************************************************************/
static INLINE float
wf_rng_uniform_float(struct wf_rng* r, float lo, float hi)
{
  return lo + wf_rng_canonical_float(r) * (hi - lo);
}

/*******************************************************************************
 * wf_rng_discard — O(1) skip forward by n draws
 *
 * Advances the counter state as if n draws had been consumed.
 * Useful for reproducibility / debugging.
 ******************************************************************************/
static INLINE void
wf_rng_discard(struct wf_rng* r, uint64_t n)
{
  /* Each counter increment produces 4 draws.
   * Current buffer has (4 - buf_idx) remaining draws. */
  uint64_t remaining_in_buf = (uint64_t)(4 - r->buf_idx);

  if(n <= remaining_in_buf) {
    r->buf_idx += (int)n;
    return;
  }

  n -= remaining_in_buf;

  /* Full counter increments needed */
  {
    uint64_t full_blocks = n / 4;
    int leftover = (int)(n % 4);

    /* Add full_blocks to the 128-bit counter */
    {
      uint64_t old_v0 = r->ctr.v[0];
      r->ctr.v[0] += full_blocks;
      if(r->ctr.v[0] < old_v0) r->ctr.v[1]++;  /* carry */
    }

    if(leftover > 0) {
      /* Need to generate the next block and consume leftover draws */
      r->buf_idx = 4;  /* force refresh */
      (void)wf_rng_get(r);  /* generates block, consumes 1 */
      r->buf_idx = leftover; /* skip to the right position */
    } else {
      r->buf_idx = 4;  /* force refresh on next get */
    }
  }
}

#endif /* SDIS_WF_RNG_H */
