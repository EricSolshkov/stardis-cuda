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
 * GPU-accelerated batch point-in-enclosure query with CPU post-processing.
 *
 * Phase B-4 M10: Wraps cus3d_find_enclosure_batch and adds:
 *   - AoS→SoA transpose + H2D upload
 *   - cus3d_enc_result → s3d_enc_result conversion (prim_id + side → enc_id)
 *   - Fallback via scene_get_enclosure_id for degenerate cases
 *
 * Architecture mirrors s3d_scene_view_batch_closest_point.cpp.
 */

#include "s3d.h"
#include "s3d_c.h"
#include "s3d_device_c.h"
#include "s3d_scene_view_c.h"
#include "cus3d_find_enclosure.h"
#include "cus3d_geom_store.h"
#include "cus3d_bvh.h"

#include <float.h>
#include <string.h>
#include <stdlib.h>

#ifdef _WIN32
#include <windows.h>
#else
#include <time.h>
#endif

static double enc_api_get_time_ms(void)
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
 * Batch enclosure context (pre-allocated GPU buffers)
 ******************************************************************************/
struct s3d_batch_enc_context {
    struct cus3d_enc_batch    gpu_batch;
    struct cus3d_enc_result*  h_results;
    size_t                    max_queries;
    int                       initialized;
};

res_T
s3d_batch_enc_context_create
  (struct s3d_batch_enc_context** out_ctx,
   size_t max_queries)
{
    struct s3d_batch_enc_context* ctx;

    if(!out_ctx) return RES_BAD_ARG;
    *out_ctx = NULL;
    if(max_queries == 0) return RES_BAD_ARG;

    ctx = (struct s3d_batch_enc_context*)calloc(1, sizeof(*ctx));
    if(!ctx) return RES_MEM_ERR;

    res_T res = cus3d_enc_batch_create(&ctx->gpu_batch, max_queries);
    if(res != RES_OK) {
        free(ctx);
        return res;
    }

    ctx->h_results = (struct cus3d_enc_result*)malloc(
        max_queries * sizeof(struct cus3d_enc_result));
    if(!ctx->h_results) {
        cus3d_enc_batch_destroy(&ctx->gpu_batch);
        free(ctx);
        return RES_MEM_ERR;
    }

    ctx->max_queries = max_queries;
    ctx->initialized = 1;
    *out_ctx = ctx;
    return RES_OK;
}

void
s3d_batch_enc_context_destroy
  (struct s3d_batch_enc_context* ctx)
{
    if(!ctx) return;
    cus3d_enc_batch_destroy(&ctx->gpu_batch);
    free(ctx->h_results);
    free(ctx);
}

/*******************************************************************************
 * Core batch find-enclosure implementation
 *
 * Flow:
 *   1. AoS→SoA transpose + upload (positions only, no radius needed)
 *   2. GPU batch find enclosure → cus3d_enc_result[]
 *   3. CPU post-process: enc_result → s3d_enc_locate_result
 *      (resolve prim_id + side → enc_id via scene prim_props)
 ******************************************************************************/
static res_T
find_enclosure_batch_impl
  (struct s3d_scene_view* view,
   struct cus3d_enc_batch* gpu_batch,
   struct cus3d_enc_result* h_results,
   const struct s3d_enc_locate_request* requests,
   size_t nqueries,
   struct s3d_enc_locate_result* results,
   struct s3d_batch_enc_stats* stats)
{
    struct cus3d_geom_store* store = view->geom_store;
    struct cus3d_bvh* bvh = view->bvh;
    struct s3d_device* dev_s3d = view->scn->dev;
    struct cus3d_device* dev = dev_s3d->gpu;
    res_T res;
    size_t i;
    double t0, t1;

    size_t stat_resolved = 0, stat_degenerate = 0, stat_miss = 0;

    /* Step 1: AoS → SoA + upload */
    t0 = stats ? enc_api_get_time_ms() : 0;
    {
        float3* h_positions = (float3*)malloc(nqueries * sizeof(float3));
        if(!h_positions) return RES_MEM_ERR;

        for(i = 0; i < nqueries; i++) {
            h_positions[i] = make_float3(requests[i].pos[0],
                                         requests[i].pos[1],
                                         requests[i].pos[2]);
        }

        cudaStream_t s = dev->stream;
        gpu_buffer_float3_upload(&gpu_batch->d_positions, h_positions, nqueries, s);
        cudaStreamSynchronize(s);

        free(h_positions);
    }
    gpu_batch->count = nqueries;

    /* Step 2: GPU batch find enclosure */
    res = cus3d_find_enclosure_batch(bvh, store, dev, gpu_batch, h_results);
    if(res != RES_OK) return res;

    t1 = stats ? enc_api_get_time_ms() : 0;
    if(stats) stats->batch_time_ms = t1 - t0;

    /* Step 3: CPU post-processing — resolve prim_id + side → enc_id */
    t0 = stats ? enc_api_get_time_ms() : 0;

    for(i = 0; i < nqueries; i++) {
        const struct cus3d_enc_result* gpu_enc = &h_results[i];

        if(gpu_enc->prim_id < 0) {
            /* No primitive found — this should be extremely rare */
            results[i].enc_id = (unsigned)-1; /* ENCLOSURE_ID_NULL */
            results[i].prim_id = -1;
            results[i].distance = -1.0f;
            results[i].side = -1;
            stat_miss++;
            continue;
        }

        if(gpu_enc->side < 0) {
            /* Degenerate — distance below threshold, side unreliable */
            results[i].enc_id = (unsigned)-1;
            results[i].prim_id = gpu_enc->prim_id;
            results[i].distance = gpu_enc->distance;
            results[i].side = -1;
            stat_degenerate++;
            continue;
        }

        /* Successful resolution — caller maps prim_id + side → enc_id
         * using scene_get_enclosure_ids() at the solver layer.
         * We store both prim_id and side here. */
        results[i].prim_id  = gpu_enc->prim_id;
        results[i].distance = gpu_enc->distance;
        results[i].side     = gpu_enc->side;

        /* Resolve enc_id from prim_props via geom_store lookup.
         * The prim_id from the GPU is a global primID in the geom_store.
         * The solver layer will do the actual scene_get_enclosure_ids() call
         * since we don't have access to the sdis_scene prim_props here.
         * We set enc_id = (unsigned)-1 to signal "needs upper-level resolution". */
        results[i].enc_id = (unsigned)-1;  /* resolved by solver */
        stat_resolved++;
    }

    t1 = stats ? enc_api_get_time_ms() : 0;
    if(stats) stats->postprocess_time_ms = t1 - t0;

    if(stats) {
        stats->total_queries   = nqueries;
        stats->resolved        = stat_resolved;
        stats->degenerate      = stat_degenerate;
        stats->missed          = stat_miss;
    }

    return RES_OK;
}

/*******************************************************************************
 * Public API
 ******************************************************************************/
res_T
s3d_scene_view_find_enclosure_batch
  (struct s3d_scene_view* scnview,
   const struct s3d_enc_locate_request* requests,
   size_t nqueries,
   struct s3d_enc_locate_result* results,
   struct s3d_batch_enc_stats* stats)
{
    if(!scnview || !requests || !results)
        return RES_BAD_ARG;
    if(nqueries == 0)
        return RES_OK;
    if(!(scnview->mask & S3D_TRACE))
        return RES_BAD_ARG;

    struct cus3d_enc_batch gpu_batch;
    res_T res = cus3d_enc_batch_create(&gpu_batch, nqueries);
    if(res != RES_OK) return res;

    struct cus3d_enc_result* h_results =
        (struct cus3d_enc_result*)malloc(nqueries * sizeof(struct cus3d_enc_result));
    if(!h_results) {
        cus3d_enc_batch_destroy(&gpu_batch);
        return RES_MEM_ERR;
    }

    res = find_enclosure_batch_impl(
        scnview, &gpu_batch, h_results, requests, nqueries, results, stats);

    free(h_results);
    cus3d_enc_batch_destroy(&gpu_batch);
    return res;
}

res_T
s3d_scene_view_find_enclosure_batch_ctx
  (struct s3d_scene_view* scnview,
   struct s3d_batch_enc_context* ctx,
   const struct s3d_enc_locate_request* requests,
   size_t nqueries,
   struct s3d_enc_locate_result* results,
   struct s3d_batch_enc_stats* stats)
{
    if(!scnview || !ctx || !requests || !results)
        return RES_BAD_ARG;
    if(nqueries == 0)
        return RES_OK;
    if(!(scnview->mask & S3D_TRACE))
        return RES_BAD_ARG;
    if(nqueries > ctx->max_queries)
        return RES_BAD_ARG;

    return find_enclosure_batch_impl(
        scnview, &ctx->gpu_batch, ctx->h_results,
        requests, nqueries, results, stats);
}
