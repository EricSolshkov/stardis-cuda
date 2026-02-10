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
 * Wraps cus3d_trace_ray_batch (<<<grid, 256>>>) and adds:
 *   - cus3d_hit_to_s3d_hit conversion
 *   - trace_hit_fixup (UV/normal convention transform)
 *   - CPU-side filter evaluation
 *   - Selective re-trace via s3d_scene_view_trace_ray for rejected rays
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
struct s3d_batch_trace_context {
    struct cus3d_ray_batch   gpu_batch;
    struct cus3d_hit_result* h_results;
    size_t                   max_rays;
    int                      initialized;
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

    ctx->h_results = (struct cus3d_hit_result*)malloc(
        max_rays * sizeof(struct cus3d_hit_result));
    if(!ctx->h_results) {
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
    free(ctx->h_results);
    free(ctx);
}

/*******************************************************************************
 * Core batch trace implementation
 *
 * Flow:
 *   1. AoS→SoA transpose + gpu_buffer upload to cus3d_ray_batch
 *   2. cus3d_trace_ray_batch → cus3d_hit_result[] (nearest hit per ray)
 *   3. CPU post-process: cus3d_hit_to_s3d_hit + trace_hit_fixup + filter
 *   4. Selective re-trace for filter-rejected rays via s3d_scene_view_trace_ray
 ******************************************************************************/
static res_T
trace_rays_batch_impl
  (struct s3d_scene_view* view,
   struct cus3d_ray_batch* gpu_batch,
   struct cus3d_hit_result* h_results,
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
    double t0, t1;

    size_t stat_accepted = 0, stat_rejected = 0;
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

    /* Step 2: GPU batch trace — nearest hit per ray */
    res = cus3d_trace_ray_batch(bvh, store, dev, gpu_batch, h_results);
    if(res != RES_OK) { free(needs_retrace); return res; }

    t1 = stats ? get_time_ms() : 0;
    if(stats) stats->batch_time_ms = t1 - t0;

    /* Step 3: CPU post-processing — type conversion + fixup + filter */
    t0 = stats ? get_time_ms() : 0;

    for(i = 0; i < nrays; i++) {
        const struct cus3d_hit_result* gpu_hit = &h_results[i];

        if(gpu_hit->prim_id < 0) {
            hits[i] = S3D_HIT_NULL;
            stat_accepted++;
            continue;
        }

        /* Resolve geometry store (instanced vs direct) */
        const struct cus3d_geom_store* resolved_store = store;
        if(gpu_hit->inst_id >= 0) {
            const struct cus3d_geom_store* cs =
                cus3d_bvh_get_instance_store(bvh, (unsigned)gpu_hit->inst_id);
            if(cs) resolved_store = cs;
        }

        cus3d_hit_to_s3d_hit(resolved_store, bvh, gpu_hit, &hits[i]);

        const struct geom_entry* ge =
            cus3d_geom_store_get_entry(resolved_store,
                                       (uint32_t)gpu_hit->geom_idx);
        if(!ge) {
            hits[i] = S3D_HIT_NULL;
            stat_accepted++;
            continue;
        }

        trace_hit_fixup(&hits[i], ge);

        if(requests[i].filter_data != NULL && ge->filter_func != NULL) {
            int rejected = ge->filter_func(
                &hits[i],
                requests[i].origin,
                requests[i].direction,
                requests[i].range,
                requests[i].filter_data,
                ge->filter_data);

            if(rejected) {
                needs_retrace[i] = 1;
                stat_rejected++;
                continue;
            }
        }

        stat_accepted++;
    }

    t1 = stats ? get_time_ms() : 0;
    if(stats) stats->postprocess_time_ms = t1 - t0;

    /* Step 4: Re-trace rejected rays via Top-K single trace (full filter) */
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
        stats->batch_accepted   = stat_accepted;
        stats->filter_rejected  = stat_rejected;
        stats->retrace_accepted = stat_retrace_ok;
        stats->retrace_missed   = stat_retrace_miss;
    }

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

    struct cus3d_ray_batch gpu_batch;
    res_T res = cus3d_ray_batch_create(&gpu_batch, nrays);
    if(res != RES_OK) return res;

    struct cus3d_hit_result* h_results =
        (struct cus3d_hit_result*)malloc(nrays * sizeof(struct cus3d_hit_result));
    if(!h_results) {
        cus3d_ray_batch_destroy(&gpu_batch);
        return RES_MEM_ERR;
    }

    res = trace_rays_batch_impl(
        scnview, &gpu_batch, h_results, requests, nrays, hits, stats);

    free(h_results);
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

    return trace_rays_batch_impl(
        scnview, &ctx->gpu_batch, ctx->h_results,
        requests, nrays, hits, stats);
}
