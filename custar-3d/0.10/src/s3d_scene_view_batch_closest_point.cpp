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
 * GPU-accelerated batch closest point query with CPU post-processing.
 *
 * Wraps cus3d_closest_point_batch and adds:
 *   - AoS→SoA transpose + H2D upload
 *   - cus3d_cp_result → s3d_hit conversion
 *   - CPU-side filter evaluation
 *   - Fallback re-query via s3d_scene_view_closest_point for rejected hits
 *
 * Architecture mirrors s3d_scene_view_batch_trace.cpp.
 */

#include "s3d.h"
#include "s3d_c.h"
#include "s3d_device_c.h"
#include "s3d_scene_view_c.h"
#include "cus3d_closest_point.h"
#include "cus3d_prim.h"
#include "cus3d_geom_store.h"
#include "cus3d_bvh.h"

#include <rsys/float3.h>
#include <float.h>
#include <string.h>
#include <stdlib.h>

#ifdef _WIN32
#include <windows.h>
#else
#include <time.h>
#endif

static double cp_get_time_ms(void)
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
 * Batch closest point context
 ******************************************************************************/
struct s3d_batch_cp_context {
    struct cus3d_cp_batch    gpu_batch;
    struct cus3d_cp_result*  h_results;
    size_t                   max_queries;
    int                      initialized;
};

res_T
s3d_batch_cp_context_create
  (struct s3d_batch_cp_context** out_ctx,
   size_t max_queries)
{
    struct s3d_batch_cp_context* ctx;

    if(!out_ctx) return RES_BAD_ARG;
    *out_ctx = NULL;
    if(max_queries == 0) return RES_BAD_ARG;

    ctx = (struct s3d_batch_cp_context*)calloc(1, sizeof(*ctx));
    if(!ctx) return RES_MEM_ERR;

    res_T res = cus3d_cp_batch_create(&ctx->gpu_batch, max_queries);
    if(res != RES_OK) {
        free(ctx);
        return res;
    }

    ctx->h_results = (struct cus3d_cp_result*)malloc(
        max_queries * sizeof(struct cus3d_cp_result));
    if(!ctx->h_results) {
        cus3d_cp_batch_destroy(&ctx->gpu_batch);
        free(ctx);
        return RES_MEM_ERR;
    }

    ctx->max_queries = max_queries;
    ctx->initialized = 1;
    *out_ctx = ctx;
    return RES_OK;
}

void
s3d_batch_cp_context_destroy
  (struct s3d_batch_cp_context* ctx)
{
    if(!ctx) return;
    cus3d_cp_batch_destroy(&ctx->gpu_batch);
    free(ctx->h_results);
    free(ctx);
}

/*******************************************************************************
 * Convert a cus3d_cp_result to s3d_hit
 *
 * Similar to cus3d_hit_to_s3d_hit but uses closest_point fields.
 * The UV from the GPU kernel uses the same barycentric convention as the
 * brute-force CPU implementation (w = 1-u-v, u, v in [uv[0], uv[1]]),
 * so no fixup is needed (unlike trace which needs Moller-Trumbore → s3d).
 ******************************************************************************/
static void
cp_result_to_s3d_hit(
    const struct cus3d_geom_store* store,
    const struct cus3d_bvh* bvh,
    const struct cus3d_cp_result* cp,
    struct s3d_hit* hit)
{
    ASSERT(store && bvh && cp && hit);

    if(cp->prim_id < 0) {
        *hit = S3D_HIT_NULL;
        return;
    }

    /* Use cus3d_hit_to_s3d_hit for the primitive resolution part.
     * Build a temporary cus3d_hit_result and convert fields. */
    struct cus3d_hit_result tmp;
    tmp.prim_id  = cp->prim_id;
    tmp.geom_idx = cp->geom_idx;
    tmp.inst_id  = cp->inst_id;
    tmp.distance = cp->distance;
    tmp.normal[0] = cp->normal[0];
    tmp.normal[1] = cp->normal[1];
    tmp.normal[2] = cp->normal[2];
    tmp.uv[0]    = cp->uv[0];
    tmp.uv[1]    = cp->uv[1];

    cus3d_hit_to_s3d_hit(store, bvh, &tmp, hit);
}

/*******************************************************************************
 * closest_point fixup — adjust UV and normal conventions.
 *
 * The GPU closest-point kernel computes barycentric coords in the same
 * convention as the brute-force CPU version:
 *   uv[0] = w  (weight of vertex A)
 *   uv[1] = u  (weight of vertex B)
 *   v = 1 - uv[0] - uv[1] (weight of vertex C)
 *
 * Since s3d_hit.uv = [w, u] (matching the CPU closest_point output),
 * NO Moller-Trumbore → s3d conversion is needed (unlike trace_hit_fixup).
 *
 * Normal: The kernel computes cross(E1, E0) = cross(v2-v0, v1-v0) which
 * is the s3d CW convention.  The CPU brute-force uses the same formula.
 * So no normal negation is needed either.
 ******************************************************************************/
static void
cp_hit_fixup(struct s3d_hit* hit, const struct geom_entry* ge)
{
    /* No UV conversion needed (already in s3d convention) */
    /* No normal negation needed (same CW convention as CPU) */
    (void)hit;
    (void)ge;
}

/*******************************************************************************
 * Core batch closest point implementation
 *
 * Flow:
 *   1. AoS→SoA transpose + upload (positions, radii)
 *   2. GPU batch closest point → cus3d_cp_result[]
 *   3. CPU post-process: cp_result → s3d_hit + filter
 *   4. Selective re-query for filter-rejected via s3d_scene_view_closest_point
 ******************************************************************************/
static res_T
closest_point_batch_impl
  (struct s3d_scene_view* view,
   struct cus3d_cp_batch* gpu_batch,
   struct cus3d_cp_result* h_results,
   const struct s3d_cp_request* requests,
   size_t nqueries,
   struct s3d_hit* hits,
   struct s3d_batch_cp_stats* stats)
{
    struct cus3d_geom_store* store = view->geom_store;
    struct cus3d_bvh* bvh = view->bvh;
    struct s3d_device* dev_s3d = view->scn->dev;
    struct cus3d_device* dev = dev_s3d->gpu;
    res_T res;
    size_t i;
    double t0, t1;

    size_t stat_accepted = 0, stat_rejected = 0;
    size_t stat_requery_ok = 0, stat_requery_miss = 0;

    uint8_t* needs_requery = (uint8_t*)calloc(nqueries, 1);
    if(!needs_requery) return RES_MEM_ERR;

    /* Step 1: AoS → SoA + upload */
    t0 = stats ? cp_get_time_ms() : 0;
    {
        float3* h_positions = (float3*)malloc(nqueries * sizeof(float3));
        float*  h_radii     = (float*)malloc(nqueries * sizeof(float));

        if(!h_positions || !h_radii) {
            free(h_positions); free(h_radii);
            free(needs_requery);
            return RES_MEM_ERR;
        }

        for(i = 0; i < nqueries; i++) {
            h_positions[i] = make_float3(requests[i].pos[0],
                                         requests[i].pos[1],
                                         requests[i].pos[2]);
            h_radii[i] = requests[i].radius;
        }

        cudaStream_t s = dev->stream;
        gpu_buffer_float3_upload(&gpu_batch->d_positions, h_positions, nqueries, s);
        cudaMemcpyAsync(gpu_batch->d_radii, h_radii,
                        nqueries * sizeof(float),
                        cudaMemcpyHostToDevice, s);
        cudaStreamSynchronize(s);

        free(h_positions);
        free(h_radii);
    }
    gpu_batch->count = nqueries;

    /* Step 2: GPU batch closest point */
    res = cus3d_closest_point_batch(bvh, store, dev, gpu_batch, h_results);
    if(res != RES_OK) { free(needs_requery); return res; }

    t1 = stats ? cp_get_time_ms() : 0;
    if(stats) stats->batch_time_ms = t1 - t0;

    /* Step 3: CPU post-processing — convert + filter */
    t0 = stats ? cp_get_time_ms() : 0;

    for(i = 0; i < nqueries; i++) {
        const struct cus3d_cp_result* gpu_cp = &h_results[i];

        if(gpu_cp->prim_id < 0) {
            hits[i] = S3D_HIT_NULL;
            stat_accepted++;
            continue;
        }

        /* Resolve geometry store (instanced vs direct) */
        const struct cus3d_geom_store* resolved_store = store;
        if(gpu_cp->inst_id >= 0) {
            const struct cus3d_geom_store* cs =
                cus3d_bvh_get_instance_store(bvh, (unsigned)gpu_cp->inst_id);
            if(cs) resolved_store = cs;
        }

        cp_result_to_s3d_hit(resolved_store, bvh, gpu_cp, &hits[i]);

        const struct geom_entry* ge =
            cus3d_geom_store_get_entry(resolved_store,
                                       (uint32_t)gpu_cp->geom_idx);
        if(!ge) {
            hits[i] = S3D_HIT_NULL;
            stat_accepted++;
            continue;
        }

        cp_hit_fixup(&hits[i], ge);

        /* Filter evaluation */
        if(requests[i].query_data != NULL && ge->filter_func != NULL) {
            /* Build a direction from query point to closest point */
            float dir[3], range[2];
            dir[0] = gpu_cp->closest_pos[0] - requests[i].pos[0];
            dir[1] = gpu_cp->closest_pos[1] - requests[i].pos[1];
            dir[2] = gpu_cp->closest_pos[2] - requests[i].pos[2];
            range[0] = 0;
            range[1] = requests[i].radius;

            int rejected = ge->filter_func(
                &hits[i],
                requests[i].pos,
                dir,
                range,
                requests[i].query_data,
                ge->filter_data);

            if(rejected) {
                needs_requery[i] = 1;
                stat_rejected++;
                continue;
            }
        }

        stat_accepted++;
    }

    t1 = stats ? cp_get_time_ms() : 0;
    if(stats) stats->postprocess_time_ms = t1 - t0;

    /* Step 4: Re-query rejected via CPU brute-force closest point */
    t0 = stats ? cp_get_time_ms() : 0;

    for(i = 0; i < nqueries; i++) {
        if(!needs_requery[i]) continue;

        res = s3d_scene_view_closest_point(
            view,
            requests[i].pos,
            requests[i].radius,
            requests[i].query_data,
            &hits[i]);

        if(res != RES_OK) {
            free(needs_requery);
            return res;
        }

        if(S3D_HIT_NONE(&hits[i]))
            stat_requery_miss++;
        else
            stat_requery_ok++;
    }

    t1 = stats ? cp_get_time_ms() : 0;
    if(stats) stats->requery_time_ms = t1 - t0;

    if(stats) {
        stats->total_queries     = nqueries;
        stats->batch_accepted    = stat_accepted;
        stats->filter_rejected   = stat_rejected;
        stats->requery_accepted  = stat_requery_ok;
        stats->requery_missed    = stat_requery_miss;
    }

    free(needs_requery);
    return RES_OK;
}

/*******************************************************************************
 * Public API
 ******************************************************************************/
res_T
s3d_scene_view_closest_point_batch
  (struct s3d_scene_view* scnview,
   const struct s3d_cp_request* requests,
   size_t nqueries,
   struct s3d_hit* hits,
   struct s3d_batch_cp_stats* stats)
{
    if(!scnview || !requests || !hits)
        return RES_BAD_ARG;
    if(nqueries == 0)
        return RES_OK;
    if(!(scnview->mask & S3D_TRACE))
        return RES_BAD_ARG;

    struct cus3d_cp_batch gpu_batch;
    res_T res = cus3d_cp_batch_create(&gpu_batch, nqueries);
    if(res != RES_OK) return res;

    struct cus3d_cp_result* h_results =
        (struct cus3d_cp_result*)malloc(nqueries * sizeof(struct cus3d_cp_result));
    if(!h_results) {
        cus3d_cp_batch_destroy(&gpu_batch);
        return RES_MEM_ERR;
    }

    res = closest_point_batch_impl(
        scnview, &gpu_batch, h_results, requests, nqueries, hits, stats);

    free(h_results);
    cus3d_cp_batch_destroy(&gpu_batch);
    return res;
}

res_T
s3d_scene_view_closest_point_batch_ctx
  (struct s3d_scene_view* scnview,
   struct s3d_batch_cp_context* ctx,
   const struct s3d_cp_request* requests,
   size_t nqueries,
   struct s3d_hit* hits,
   struct s3d_batch_cp_stats* stats)
{
    if(!scnview || !ctx || !requests || !hits)
        return RES_BAD_ARG;
    if(nqueries == 0)
        return RES_OK;
    if(!(scnview->mask & S3D_TRACE))
        return RES_BAD_ARG;
    if(nqueries > ctx->max_queries)
        return RES_BAD_ARG;

    return closest_point_batch_impl(
        scnview, &ctx->gpu_batch, ctx->h_results,
        requests, nqueries, hits, stats);
}
