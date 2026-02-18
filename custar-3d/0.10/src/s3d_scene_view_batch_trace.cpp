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

/*
 * GPU-accelerated batch ray tracing with CPU post-processing.
 *
 * Wraps cus3d_trace_ray_batch_multi (<<<grid, 256>>>, Top-K) and adds:
 *   - cus3d_hit_to_s3d_hit conversion
 *   - trace_hit_fixup (UV/normal convention transform)
 *   - CPU-side filter evaluation against Top-K candidates
 *   - Selective re-trace via s3d_scene_view_trace_ray only when all K
 *     candidates are rejected (rare).
 */

#include "s3d.h"
#include "s3d_c.h"
#include "s3d_device_c.h"
#include "s3d_scene_view_c.h"
#include "cus3d_trace.h"
#include "cus3d_prim.h"
#include "cus3d_geom_store.h"
#include "cus3d_bvh.h"
#include "cus3d_trace_util.h"

#include <rsys/float3.h>
#include <float.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <assert.h>

#ifdef _WIN32
#include <windows.h>
#else
#include <time.h>
#endif

static double get_time_ms(void)
{
#ifdef _WIN32
    LARGE_INTEGER freq, cnt;
    QueryPerformanceFrequency(&freq);
    QueryPerformanceCounter(&cnt);
    return (double)cnt.QuadPart / (double)freq.QuadPart * 1000.0;
#else
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (double)ts.tv_sec * 1000.0 + (double)ts.tv_nsec / 1e6;
#endif
}

/*******************************************************************************
 * Batch trace context
 ******************************************************************************/
/* Number of Top-K candidates per ray in batch mode.
 * Must cover: 1 self-hit (#1) + ~2 shared-edge hits (#3) + ~1 boundary
 * hit (#4) + margin.  K=8 (CUS3D_MAX_MULTI_HITS) provides ample room
 * and the GPU insertion-sort cost is negligible at K<=8. */
#define BATCH_TOPK_COUNT CUS3D_MAX_MULTI_HITS

/* Maximum fallback re-trace depth when all K candidates are rejected.
 * Matches MAX_FALLBACK_DEPTH in s3d_scene_view_trace_ray.cpp. */
#define BATCH_MAX_FALLBACK_DEPTH 4

struct s3d_batch_trace_context {
    struct cus3d_ray_batch          gpu_batch;
    struct cus3d_multi_hit_result*  h_multi_results;
    size_t                          max_rays;
    int                             initialized;
};

res_T
s3d_batch_trace_context_create
  (struct s3d_batch_trace_context** out_ctx,
   size_t max_rays)
{
    struct s3d_batch_trace_context* ctx;

    if(!out_ctx) return RES_BAD_ARG;
    *out_ctx = NULL;
    if(max_rays == 0) return RES_BAD_ARG;

    ctx = (struct s3d_batch_trace_context*)calloc(1, sizeof(*ctx));
    if(!ctx) return RES_MEM_ERR;

    res_T res = cus3d_ray_batch_create(&ctx->gpu_batch, max_rays);
    if(res != RES_OK) {
        free(ctx);
        return res;
    }

    ctx->h_multi_results = (struct cus3d_multi_hit_result*)malloc(
        max_rays * sizeof(struct cus3d_multi_hit_result));
    if(!ctx->h_multi_results) {
        cus3d_ray_batch_destroy(&ctx->gpu_batch);
        free(ctx);
        return RES_MEM_ERR;
    }

    ctx->max_rays    = max_rays;
    ctx->initialized = 1;
    *out_ctx = ctx;
    return RES_OK;
}

void
s3d_batch_trace_context_destroy
  (struct s3d_batch_trace_context* ctx)
{
    if(!ctx) return;
    cus3d_ray_batch_destroy(&ctx->gpu_batch);
    free(ctx->h_multi_results);
    free(ctx);
}

/*******************************************************************************
 * Batch trace diagnostics (compile-time toggleable)
 ******************************************************************************/
#ifndef BATCH_TRACE_DIAG
#define BATCH_TRACE_DIAG 0   /* Set to 0 to silence all diagnostics */
#endif

/* How often (in batch calls) to print a summary line */
#define DIAG_SUMMARY_INTERVAL 50

/* Max detailed per-ray dumps per batch call (to avoid flooding) */
#define DIAG_MAX_DETAIL_DUMPS 3

/* Cumulative counters across batch calls (file-scoped) */
static size_t g_diag_batch_calls       = 0;
static size_t g_diag_total_rays        = 0;
static size_t g_diag_total_accepted0   = 0;   /* accepted at candidate[0] */
static size_t g_diag_total_topk_saved  = 0;   /* accepted at candidate[1..K-1] */
static size_t g_diag_total_all_reject  = 0;   /* all K rejected → retrace */
static size_t g_diag_total_miss        = 0;   /* count==0, no candidates at all */
static size_t g_diag_total_no_filter   = 0;   /* no filter_data → auto-accept */
static size_t g_diag_total_retrace_ok  = 0;
static size_t g_diag_total_retrace_miss = 0;
/* Per-candidate-index acceptance histogram */
static size_t g_diag_accept_hist[CUS3D_MAX_MULTI_HITS] = {0};

/*******************************************************************************
 * Core batch trace implementation (Top-K)
 *
 * Flow:
 *   1. AoS→SoA transpose + gpu_buffer upload to cus3d_ray_batch
 *   2. cus3d_trace_ray_batch_multi → cus3d_multi_hit_result[] (Top-K per ray)
 *   3. CPU post-process: iterate Top-K candidates per ray, accept first
 *      that passes filter (cus3d_hit_to_s3d_hit + trace_hit_fixup + filter)
 *   4. Rare fallback: re-trace only rays where ALL K candidates were rejected
 ******************************************************************************/
static res_T
trace_rays_batch_impl
  (struct s3d_scene_view* view,
   struct cus3d_ray_batch* gpu_batch,
   struct cus3d_multi_hit_result* h_multi_results,
   const struct s3d_ray_request* requests,
   size_t nrays,
   struct s3d_hit* hits,
   struct s3d_batch_trace_stats* stats)
{
    struct cus3d_geom_store* store = view->geom_store;
    struct cus3d_bvh* bvh = view->bvh;
    struct s3d_device* dev_s3d = view->scn->dev;
    struct cus3d_device* dev = dev_s3d->gpu;
    res_T res;
    size_t i;
    int j_hist;
    double t0, t1;

    size_t stat_accepted = 0, stat_topk_accepted = 0;
    size_t stat_rejected = 0;
    size_t stat_retrace_ok = 0, stat_retrace_miss = 0;

    uint8_t* needs_retrace = (uint8_t*)calloc(nrays, 1);
    if(!needs_retrace) return RES_MEM_ERR;

    /* Step 1: AoS → SoA transpose + upload to GPU */
    t0 = stats ? get_time_ms() : 0;
    {
        float3* h_origins    = (float3*)malloc(nrays * sizeof(float3));
        float3* h_directions = (float3*)malloc(nrays * sizeof(float3));
        float2* h_ranges     = (float2*)malloc(nrays * sizeof(float2));

        if(!h_origins || !h_directions || !h_ranges) {
            free(h_origins); free(h_directions); free(h_ranges);
            free(needs_retrace);
            return RES_MEM_ERR;
        }

        for(i = 0; i < nrays; i++) {
            h_origins[i]    = make_float3(requests[i].origin[0],
                                          requests[i].origin[1],
                                          requests[i].origin[2]);
            h_directions[i] = make_float3(requests[i].direction[0],
                                          requests[i].direction[1],
                                          requests[i].direction[2]);
            h_ranges[i]     = make_float2(requests[i].range[0],
                                          requests[i].range[1]);
        }

        cudaStream_t s = dev->stream;
        gpu_buffer_float3_upload(&gpu_batch->d_origins, h_origins, nrays, s);
        gpu_buffer_float3_upload(&gpu_batch->d_directions, h_directions, nrays, s);
        gpu_buffer_float2_upload(&gpu_batch->d_ranges, h_ranges, nrays, s);
        cudaStreamSynchronize(s);

        free(h_origins);
        free(h_directions);
        free(h_ranges);
    }
    gpu_batch->count = nrays;

    /* Step 2: GPU batch Top-K trace — up to BATCH_TOPK_COUNT hits per ray */
    res = cus3d_trace_ray_batch_multi(
        bvh, store, dev, gpu_batch, BATCH_TOPK_COUNT, h_multi_results);
    if(res != RES_OK) { free(needs_retrace); return res; }

    t1 = stats ? get_time_ms() : 0;
    if(stats) stats->batch_time_ms = t1 - t0;

    /* Step 3: CPU post-processing — iterate Top-K candidates per ray */
    t0 = stats ? get_time_ms() : 0;

    size_t diag_miss_count = 0;      /* count==0 rays this batch */
    size_t diag_no_filter = 0;       /* accepted w/o filter this batch */
    size_t diag_detail_dumps = 0;    /* per-ray detail dumps emitted */
    /* Per-candidate-index acceptance this batch */
    size_t diag_accept_at[CUS3D_MAX_MULTI_HITS] = {0};

    for(i = 0; i < nrays; i++) {
        const struct cus3d_multi_hit_result* multi = &h_multi_results[i];
        int accepted = 0;
        int j;

        if(multi->count == 0) {
            hits[i] = S3D_HIT_NULL;
            stat_accepted++;
            diag_miss_count++;
            continue;
        }

        /* Iterate candidates in ascending distance order */
        for(j = 0; j < multi->count; j++) {
            const struct cus3d_hit_result* candidate = &multi->hits[j];

            if(candidate->prim_id < 0) {
                continue;
            }

            /* Resolve geometry store (instanced vs direct) */
            const struct cus3d_geom_store* resolved_store = store;
            if(candidate->inst_id >= 0) {
                const struct cus3d_geom_store* cs =
                    cus3d_bvh_get_instance_store(bvh,
                                                 (unsigned)candidate->inst_id);
                if(cs) resolved_store = cs;
            }

            cus3d_hit_to_s3d_hit(resolved_store, bvh, candidate, &hits[i]);

            const struct geom_entry* ge =
                cus3d_geom_store_get_entry(resolved_store,
                                           (uint32_t)candidate->geom_idx);
            if(!ge) continue;

            trace_hit_fixup(&hits[i], ge);

            /* Evaluate filter if present */
            if(requests[i].filter_data != NULL && ge->filter_func != NULL) {
                int rejected = ge->filter_func(
                    &hits[i],
                    requests[i].origin,
                    requests[i].direction,
                    requests[i].range,
                    requests[i].filter_data,
                    ge->filter_data);

                if(rejected)
                    continue; /* Try next Top-K candidate */
            } else if(requests[i].filter_data == NULL) {
                diag_no_filter++;
            }

            /* This candidate accepted */
            accepted = 1;
            diag_accept_at[j]++;
            if(j == 0)
                stat_accepted++;
            else
                stat_topk_accepted++;
            break;
        }

        if(!accepted) {
            /* All candidates rejected by filter.
             * If count < BATCH_TOPK_COUNT the GPU exhausted every
             * intersection in the BVH — retrace will find the same
             * (or fewer) candidates and also fail.  Short-circuit. */
            if(multi->count < BATCH_TOPK_COUNT) {
                hits[i] = S3D_HIT_NULL;
                stat_rejected++;
                /* No retrace needed — BVH is exhausted */
            } else {
                /* K slots were full — there may be further intersections
                 * beyond the K-th candidate.  Retrace can succeed. */
                needs_retrace[i] = 1;
                stat_rejected++;
            }

#if BATCH_TRACE_DIAG
            /* Dump detailed info for the first few all-rejected rays */
            if(diag_detail_dumps < DIAG_MAX_DETAIL_DUMPS) {
                diag_detail_dumps++;
                fprintf(stderr,
                    "[BATCH_DIAG] ALL-REJECT ray %zu: "
                    "org=(%.6g,%.6g,%.6g) dir=(%.6g,%.6g,%.6g) "
                    "range=[%.6g,%.6g] K=%d has_filter=%d\n",
                    i,
                    requests[i].origin[0], requests[i].origin[1],
                    requests[i].origin[2],
                    requests[i].direction[0], requests[i].direction[1],
                    requests[i].direction[2],
                    requests[i].range[0], requests[i].range[1],
                    multi->count,
                    requests[i].filter_data != NULL ? 1 : 0);

                for(j = 0; j < multi->count; j++) {
                    const struct cus3d_hit_result* c = &multi->hits[j];
                    fprintf(stderr,
                        "  candidate[%d]: prim=%d geom=%d inst=%d "
                        "dist=%.8g normal=(%.4g,%.4g,%.4g) uv=(%.4g,%.4g)\n",
                        j, c->prim_id, c->geom_idx, c->inst_id,
                        c->distance,
                        c->normal[0], c->normal[1], c->normal[2],
                        c->uv[0], c->uv[1]);
                }
            }
#endif
        }
    }

    t1 = stats ? get_time_ms() : 0;
    if(stats) stats->postprocess_time_ms = t1 - t0;

    /* Step 4: Re-trace only rays where ALL K candidates were rejected */
    t0 = stats ? get_time_ms() : 0;

    for(i = 0; i < nrays; i++) {
        if(!needs_retrace[i]) continue;

        res = s3d_scene_view_trace_ray(
            view,
            requests[i].origin,
            requests[i].direction,
            requests[i].range,
            requests[i].filter_data,
            &hits[i]);

        if(res != RES_OK) {
            free(needs_retrace);
            return res;
        }

        if(S3D_HIT_NONE(&hits[i]))
            stat_retrace_miss++;
        else
            stat_retrace_ok++;
    }

    t1 = stats ? get_time_ms() : 0;
    if(stats) stats->retrace_time_ms = t1 - t0;

    if(stats) {
        stats->total_rays       = nrays;
        stats->batch_accepted   = stat_accepted + stat_topk_accepted;
        stats->filter_rejected  = stat_rejected;
        stats->retrace_accepted = stat_retrace_ok;
        stats->retrace_missed   = stat_retrace_miss;
    }

#if BATCH_TRACE_DIAG
    /* Update cumulative counters */
    g_diag_batch_calls++;
    g_diag_total_rays       += nrays;
    g_diag_total_accepted0  += stat_accepted;
    g_diag_total_topk_saved += stat_topk_accepted;
    g_diag_total_all_reject += stat_rejected;
    g_diag_total_miss       += diag_miss_count;
    g_diag_total_no_filter  += diag_no_filter;
    g_diag_total_retrace_ok += stat_retrace_ok;
    g_diag_total_retrace_miss += stat_retrace_miss;
    for(j_hist = 0; j_hist < CUS3D_MAX_MULTI_HITS; j_hist++)
        g_diag_accept_hist[j_hist] += diag_accept_at[j_hist];

    /* Periodic summary */
    if(g_diag_batch_calls % DIAG_SUMMARY_INTERVAL == 0) {
        double reject_pct = g_diag_total_rays > 0
            ? 100.0 * (double)g_diag_total_all_reject / (double)g_diag_total_rays
            : 0.0;
        double topk_pct = g_diag_total_rays > 0
            ? 100.0 * (double)g_diag_total_topk_saved / (double)g_diag_total_rays
            : 0.0;
        fprintf(stderr,
            "[BATCH_DIAG] === SUMMARY after %zu calls, %zu total rays ===\n"
            "  accept@[0]: %zu | topk_saved: %zu (%.1f%%) | "
            "all_reject: %zu (%.1f%%) | miss: %zu | no_filter: %zu\n"
            "  retrace_ok: %zu | retrace_miss: %zu\n",
            g_diag_batch_calls, g_diag_total_rays,
            g_diag_total_accepted0, g_diag_total_topk_saved, topk_pct,
            g_diag_total_all_reject, reject_pct,
            g_diag_total_miss, g_diag_total_no_filter,
            g_diag_total_retrace_ok, g_diag_total_retrace_miss);

        /* Acceptance histogram */
        fprintf(stderr, "  accept_hist:");
        for(j_hist = 0; j_hist < CUS3D_MAX_MULTI_HITS; j_hist++) {
            if(g_diag_accept_hist[j_hist] > 0)
                fprintf(stderr, " [%d]=%zu", j_hist, g_diag_accept_hist[j_hist]);
        }
        fprintf(stderr, "\n");
    }
#endif

    free(needs_retrace);
    return RES_OK;
}

/*******************************************************************************
 * Public API
 ******************************************************************************/
res_T
s3d_scene_view_trace_rays_batch
  (struct s3d_scene_view* scnview,
   const struct s3d_ray_request* requests,
   size_t nrays,
   struct s3d_hit* hits,
   struct s3d_batch_trace_stats* stats)
{
    if(!scnview || !requests || !hits)
        return RES_BAD_ARG;
    if(nrays == 0)
        return RES_OK;
    if(!(scnview->mask & S3D_TRACE))
        return RES_BAD_ARG;
    if(nrays == 1) {
        fprintf(stderr,
            "[BATCH_TRACE] ABORT: batch size == 1 in "
            "s3d_scene_view_trace_rays_batch — inefficient GPU launch!\n");
        abort();
    }

    struct cus3d_ray_batch gpu_batch;
    res_T res = cus3d_ray_batch_create(&gpu_batch, nrays);
    if(res != RES_OK) return res;

    struct cus3d_multi_hit_result* h_multi_results =
        (struct cus3d_multi_hit_result*)malloc(
            nrays * sizeof(struct cus3d_multi_hit_result));
    if(!h_multi_results) {
        cus3d_ray_batch_destroy(&gpu_batch);
        return RES_MEM_ERR;
    }

    res = trace_rays_batch_impl(
        scnview, &gpu_batch, h_multi_results, requests, nrays, hits, stats);

    free(h_multi_results);
    cus3d_ray_batch_destroy(&gpu_batch);
    return res;
}

res_T
s3d_scene_view_trace_rays_batch_ctx
  (struct s3d_scene_view* scnview,
   struct s3d_batch_trace_context* ctx,
   const struct s3d_ray_request* requests,
   size_t nrays,
   struct s3d_hit* hits,
   struct s3d_batch_trace_stats* stats)
{
    if(!scnview || !ctx || !requests || !hits)
        return RES_BAD_ARG;
    if(nrays == 0)
        return RES_OK;
    if(!(scnview->mask & S3D_TRACE))
        return RES_BAD_ARG;
    if(nrays > ctx->max_rays)
        return RES_BAD_ARG;
    if(nrays == 1) {
        fprintf(stderr,
            "[BATCH_TRACE] ABORT: batch size == 1 in "
            "s3d_scene_view_trace_rays_batch_ctx — inefficient GPU launch!\n");
        abort();
    }

    return trace_rays_batch_impl(
        scnview, &ctx->gpu_batch, ctx->h_multi_results,
        requests, nrays, hits, stats);
}
