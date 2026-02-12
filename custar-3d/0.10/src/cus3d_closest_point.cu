/**
 * @file cus3d_closest_point.cu
 * @brief GPU closest point kernels + host-side batch API
 *
 * Implements BVH-accelerated closest point queries using
 * cuBQL::shrinkingRadiusQuery::forEachPrim.
 *
 * Architecture parallels cus3d_trace.cu:
 *   - closest_point_kernel (single-level BLAS)
 *   - closest_point_instanced_kernel (two-level TLAS+BLAS)
 *   - cus3d_cp_batch_create / destroy
 *   - cus3d_closest_point_batch (host-side launch + download)
 *
 * Closest point on triangle uses double-precision math (matching the CPU
 * brute-force implementation) to satisfy the project's 1e-6 tolerance
 * requirement.  The BVH traversal itself operates in float (squared
 * distances), only the per-candidate distance test promotes to double.
 */

#include "cus3d_closest_point.h"
#include "cus3d_bvh_internal.h"
#include "cus3d_mem.h"
#include "cus3d_math.cuh"

#include <cuBQL/bvh.h>
#include <cuBQL/traversal/shrinkingRadiusQuery.h>

#include <cuda_runtime.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

/* ------------------------------------------------------------------ */
/*  Error handling (mirrors cus3d_trace.cu)                           */
/* ------------------------------------------------------------------ */

#ifdef _MSC_VER
  static __declspec(thread) char s_cp_error_msg[512] = {0};
#else
  static __thread char s_cp_error_msg[512] = {0};
#endif

static void cp_clear_error(void) { s_cp_error_msg[0] = '\0'; }

static void cp_set_error(const char* msg) {
    if (msg) {
        strncpy(s_cp_error_msg, msg, sizeof(s_cp_error_msg) - 1);
        s_cp_error_msg[sizeof(s_cp_error_msg) - 1] = '\0';
    }
}

#define CP_CUDA_CHECK(call, context) \
    do { \
        cudaError_t err__ = (call); \
        if (err__ != cudaSuccess) { \
            char buf__[512]; \
            snprintf(buf__, sizeof(buf__), \
                     "%s failed: %s (%s:%d)", \
                     context, cudaGetErrorString(err__), __FILE__, __LINE__); \
            cp_set_error(buf__); \
            return RES_ERR; \
        } \
    } while (0)

/* ------------------------------------------------------------------ */
/*  Device: double-precision closest point on triangle                */
/*                                                                    */
/*  Verbatim port of closest_point_triangle() from                    */
/*  s3d_scene_view_closest_point.cpp (the canonical reference).       */
/*  Returns squared distance in float for BVH radius shrinking,      */
/*  but computes in double for precision.                             */
/* ------------------------------------------------------------------ */

struct cp_triangle_result {
    float  sqr_dist;       /* squared Euclidean distance */
    float  closest_pt[3];  /* world-space closest point */
    float  uv[2];          /* barycentric coordinates [w, u] */
};

__device__ static struct cp_triangle_result
closest_point_triangle_gpu(
    const float3& query,   /* query position */
    const float3& va,      /* triangle vertex A */
    const float3& vb,      /* triangle vertex B */
    const float3& vc)      /* triangle vertex C */
{
    struct cp_triangle_result res;
    res.sqr_dist = INFINITY;

    /* Promote to double for precision */
    double p[3] = { (double)query.x, (double)query.y, (double)query.z };
    double a[3] = { (double)va.x, (double)va.y, (double)va.z };
    double b[3] = { (double)vb.x, (double)vb.y, (double)vb.z };
    double c[3] = { (double)vc.x, (double)vc.y, (double)vc.z };

    double ab[3] = { b[0]-a[0], b[1]-a[1], b[2]-a[2] };
    double ac[3] = { c[0]-a[0], c[1]-a[1], c[2]-a[2] };
    double ap[3] = { p[0]-a[0], p[1]-a[1], p[2]-a[2] };

    double d1 = ab[0]*ap[0] + ab[1]*ap[1] + ab[2]*ap[2];
    double d2 = ac[0]*ap[0] + ac[1]*ap[1] + ac[2]*ap[2];

    double cp_d[3]; /* closest point in double */
    double uv0, uv1; /* barycentric */

    /* Vertex A */
    if (d1 <= 0.0 && d2 <= 0.0) {
        cp_d[0] = a[0]; cp_d[1] = a[1]; cp_d[2] = a[2];
        uv0 = 1.0; uv1 = 0.0;
        goto done;
    }

    /* Vertex B */
    {
        double bp[3] = { p[0]-b[0], p[1]-b[1], p[2]-b[2] };
        double d3 = ab[0]*bp[0] + ab[1]*bp[1] + ab[2]*bp[2];
        double d4 = ac[0]*bp[0] + ac[1]*bp[1] + ac[2]*bp[2];
        if (d3 >= 0.0 && d4 <= d3) {
            cp_d[0] = b[0]; cp_d[1] = b[1]; cp_d[2] = b[2];
            uv0 = 0.0; uv1 = 1.0;
            goto done;
        }

        /* Edge AB */
        double vc_ = d1*d4 - d3*d2;
        if (vc_ <= 0.0 && d1 >= 0.0 && d3 <= 0.0) {
            double s = d1 / (d1 - d3);
            cp_d[0] = a[0] + s*ab[0];
            cp_d[1] = a[1] + s*ab[1];
            cp_d[2] = a[2] + s*ab[2];
            uv0 = 1.0 - s; uv1 = s;
            goto done;
        }

        /* Vertex C */
        double cp_arr[3] = { p[0]-c[0], p[1]-c[1], p[2]-c[2] };
        double d5 = ab[0]*cp_arr[0] + ab[1]*cp_arr[1] + ab[2]*cp_arr[2];
        double d6 = ac[0]*cp_arr[0] + ac[1]*cp_arr[1] + ac[2]*cp_arr[2];
        if (d6 >= 0.0 && d5 <= d6) {
            cp_d[0] = c[0]; cp_d[1] = c[1]; cp_d[2] = c[2];
            uv0 = 0.0; uv1 = 0.0;
            goto done;
        }

        /* Edge AC */
        double vb_ = d5*d2 - d1*d6;
        if (vb_ <= 0.0 && d2 >= 0.0 && d6 <= 0.0) {
            double s = d2 / (d2 - d6);
            cp_d[0] = a[0] + s*ac[0];
            cp_d[1] = a[1] + s*ac[1];
            cp_d[2] = a[2] + s*ac[2];
            uv0 = 1.0 - s; uv1 = 0.0;
            goto done;
        }

        /* Edge BC */
        double va_ = d3*d6 - d5*d4;
        if (va_ <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0) {
            double s = (d4 - d3) / ((d4 - d3) + (d5 - d6));
            cp_d[0] = b[0] + s*(c[0] - b[0]);
            cp_d[1] = b[1] + s*(c[1] - b[1]);
            cp_d[2] = b[2] + s*(c[2] - b[2]);
            uv0 = 0.0; uv1 = 1.0 - s;
            goto done;
        }

        /* Interior point */
        {
            double rcp_area = 1.0 / (va_ + vb_ + vc_);
            double v = vb_ * rcp_area;
            double w = vc_ * rcp_area;
            uv0 = 1.0 - v - w;
            uv1 = v;
            if (uv0 < 0.0) {
                if (uv1 > w) uv1 += uv0;
                uv0 = 0.0;
            }
            cp_d[0] = a[0] + v*ab[0] + w*ac[0];
            cp_d[1] = a[1] + v*ab[1] + w*ac[1];
            cp_d[2] = a[2] + v*ab[2] + w*ac[2];
        }
    }

done:
    {
        double dx = cp_d[0] - p[0];
        double dy = cp_d[1] - p[1];
        double dz = cp_d[2] - p[2];
        res.sqr_dist = (float)(dx*dx + dy*dy + dz*dz);
        res.closest_pt[0] = (float)cp_d[0];
        res.closest_pt[1] = (float)cp_d[1];
        res.closest_pt[2] = (float)cp_d[2];
        res.uv[0] = (float)uv0;
        res.uv[1] = (float)uv1;
    }
    return res;
}

/* ------------------------------------------------------------------ */
/*  Device: closest point on sphere                                   */
/* ------------------------------------------------------------------ */

struct cp_sphere_result {
    float sqr_dist;
    float closest_pt[3];
    float normal[3];
    float uv[2];
};

__device__ static struct cp_sphere_result
closest_point_sphere_gpu(
    const float3& query,
    const struct sphere_gpu& sp)
{
    struct cp_sphere_result res;
    float3 diff = make_float3(query.x - sp.cx, query.y - sp.cy, query.z - sp.cz);
    float len = length(diff);
    float dst = fabsf(len - sp.radius);

    res.sqr_dist = dst * dst;

    /* Normal = direction from sphere center to query */
    float3 N;
    if (len > 0.0f) {
        N = make_float3(diff.x / len, diff.y / len, diff.z / len);
    } else {
        N = make_float3(1.0f, 0.0f, 0.0f);
    }
    res.normal[0] = N.x;
    res.normal[1] = N.y;
    res.normal[2] = N.z;

    /* Closest point on sphere surface */
    res.closest_pt[0] = sp.cx + N.x * sp.radius;
    res.closest_pt[1] = sp.cy + N.y * sp.radius;
    res.closest_pt[2] = sp.cz + N.z * sp.radius;

    /* UV from normal */
    float2 suv = sphere_normal_to_uv(N);
    res.uv[0] = suv.x;
    res.uv[1] = suv.y;

    return res;
}

/* ------------------------------------------------------------------ */
/*  Kernel: single-level closest point (BLAS only)                    */
/* ------------------------------------------------------------------ */

__global__ void closest_point_kernel(
    cuBQL::BinaryBVH<float, 3>               bvh,
    const float3* __restrict__               vertices,
    const uint3*  __restrict__               indices,
    const struct sphere_gpu* __restrict__    spheres,
    const unsigned int* __restrict__         prim_to_geom,
    const struct geom_gpu_entry* __restrict__ geom_entries,
    uint32_t                                 tri_count,
    const float3* __restrict__               query_positions,
    const float*  __restrict__               query_radii,
    uint32_t                                 num_queries,
    struct cus3d_cp_result*                  results)
{
    uint32_t tid = blockIdx.x * blockDim.x + threadIdx.x;
    if (tid >= num_queries) return;

    float3 pos   = query_positions[tid];
    float radius = query_radii[tid];

    struct cus3d_cp_result hit;
    hit.prim_id  = -1;
    hit.geom_idx = -1;
    hit.inst_id  = -1;
    hit.distance = radius;
    hit.closest_pos[0] = 0.0f;
    hit.closest_pos[1] = 0.0f;
    hit.closest_pos[2] = 0.0f;
    hit.normal[0] = 0.0f; hit.normal[1] = 0.0f; hit.normal[2] = 0.0f;
    hit.uv[0] = 0.0f; hit.uv[1] = 0.0f;

    float best_sqr_dist = radius * radius;

    auto candidate = [&](uint32_t primID) -> float {
        uint32_t geom_idx = prim_to_geom[primID];
        struct geom_gpu_entry ge = geom_entries[geom_idx];

        if (!ge.is_enabled)
            return best_sqr_dist;

        if (primID < tri_count) {
            /* Triangle closest point */
            uint32_t local_id = primID - ge.prim_offset;
            uint3 tri_idx = indices[ge.prim_offset + local_id];
            float3 v0 = vertices[tri_idx.x];
            float3 v1 = vertices[tri_idx.y];
            float3 v2 = vertices[tri_idx.z];

            struct cp_triangle_result tr = closest_point_triangle_gpu(pos, v0, v1, v2);

            if (tr.sqr_dist < best_sqr_dist) {
                best_sqr_dist = tr.sqr_dist;
                float dst = sqrtf(tr.sqr_dist);

                /* Compute geometric normal (CW convention: E1 × E0) */
                float3 e0 = v1 - v0;
                float3 e1 = v2 - v0;
                float3 N = cross(e1, e0);
                if (ge.flip_surface) {
                    N.x = -N.x; N.y = -N.y; N.z = -N.z;
                }

                hit.prim_id  = (int32_t)primID;
                hit.geom_idx = (int32_t)geom_idx;
                hit.distance = dst;
                hit.closest_pos[0] = tr.closest_pt[0];
                hit.closest_pos[1] = tr.closest_pt[1];
                hit.closest_pos[2] = tr.closest_pt[2];
                hit.normal[0] = N.x;
                hit.normal[1] = N.y;
                hit.normal[2] = N.z;
                hit.uv[0]     = tr.uv[0];
                hit.uv[1]     = tr.uv[1];
            }
        } else {
            /* Sphere closest point */
            uint32_t sphere_idx = primID - tri_count;
            struct sphere_gpu sp = spheres[sphere_idx];

            struct cp_sphere_result sr = closest_point_sphere_gpu(pos, sp);

            if (sr.sqr_dist < best_sqr_dist) {
                best_sqr_dist = sr.sqr_dist;
                float dst = sqrtf(sr.sqr_dist);

                float3 N = make_float3(sr.normal[0], sr.normal[1], sr.normal[2]);
                if (ge.flip_surface) {
                    N.x = -N.x; N.y = -N.y; N.z = -N.z;
                }

                hit.prim_id  = (int32_t)primID;
                hit.geom_idx = (int32_t)geom_idx;
                hit.distance = dst;
                hit.closest_pos[0] = sr.closest_pt[0];
                hit.closest_pos[1] = sr.closest_pt[1];
                hit.closest_pos[2] = sr.closest_pt[2];
                hit.normal[0] = N.x;
                hit.normal[1] = N.y;
                hit.normal[2] = N.z;
                hit.uv[0]     = sr.uv[0];
                hit.uv[1]     = sr.uv[1];
            }
        }
        return best_sqr_dist;
    };

    cuBQL::shrinkingRadiusQuery::forEachPrim(
        candidate,
        bvh,
        cuBQL::vec3f(pos.x, pos.y, pos.z),
        best_sqr_dist);

    results[tid] = hit;
}

/* ------------------------------------------------------------------ */
/*  Kernel: two-level closest point (TLAS + BLAS)                     */
/*                                                                    */
/*  cuBQL::shrinkingRadiusQuery has no twoLevel variant, so we        */
/*  manually iterate TLAS leaves, transform the query into instance   */
/*  local space, and run shrinkingRadiusQuery on the child BVH.       */
/*  Only instances whose world-space AABB overlaps the query sphere   */
/*  are visited (TLAS provides that filtering).                       */
/* ------------------------------------------------------------------ */

__global__ void closest_point_instanced_kernel(
    cuBQL::BinaryBVH<float, 3>               tlas,
    const cuBQL::BinaryBVH<float, 3>* __restrict__ blas_array,
    const struct instance_gpu_data*  __restrict__  instances,
    uint32_t                                 num_instances,
    const float3* __restrict__               vertices,
    const uint3*  __restrict__               indices,
    const struct sphere_gpu* __restrict__    spheres,
    const unsigned int* __restrict__         prim_to_geom,
    const struct geom_gpu_entry* __restrict__ geom_entries,
    uint32_t                                 tri_count,
    const float3* __restrict__               query_positions,
    const float*  __restrict__               query_radii,
    uint32_t                                 num_queries,
    struct cus3d_cp_result*                  results)
{
    uint32_t tid = blockIdx.x * blockDim.x + threadIdx.x;
    if (tid >= num_queries) return;

    float3 pos_ws = query_positions[tid];
    float  radius = query_radii[tid];

    struct cus3d_cp_result hit;
    hit.prim_id  = -1;
    hit.geom_idx = -1;
    hit.inst_id  = -1;
    hit.distance = radius;
    hit.closest_pos[0] = 0.0f;
    hit.closest_pos[1] = 0.0f;
    hit.closest_pos[2] = 0.0f;
    hit.normal[0] = 0.0f; hit.normal[1] = 0.0f; hit.normal[2] = 0.0f;
    hit.uv[0] = 0.0f; hit.uv[1] = 0.0f;

    float best_sqr_dist = radius * radius;

    /* Use TLAS traversal to find candidate instances */
    auto tlas_candidate = [&](uint32_t instID) -> float {
        if (instID >= num_instances)
            return best_sqr_dist;

        struct instance_gpu_data inst = instances[instID];

        /* Transform query position to instance local space */
        const float* T = inst.inv_transform;
        float lx = T[0]*pos_ws.x + T[3]*pos_ws.y + T[6]*pos_ws.z + T[9];
        float ly = T[1]*pos_ws.x + T[4]*pos_ws.y + T[7]*pos_ws.z + T[10];
        float lz = T[2]*pos_ws.x + T[5]*pos_ws.y + T[8]*pos_ws.z + T[11];
        float3 pos_ls = make_float3(lx, ly, lz);

        /* Approximate similarity scale for distance conversion.
         * For uniform-scale transforms this is exact.
         * det(3x3) = forward transform determinant */
        const float* Tf = inst.transform;
        float det = Tf[0]*(Tf[4]*Tf[8] - Tf[5]*Tf[7])
                  - Tf[3]*(Tf[1]*Tf[8] - Tf[2]*Tf[7])
                  + Tf[6]*(Tf[1]*Tf[5] - Tf[2]*Tf[4]);
        float sim_scale = cbrtf(fabsf(det));
        if (sim_scale < 1e-30f) return best_sqr_dist;

        /* Scale the search radius to local space */
        float local_radius_sq = best_sqr_dist / (sim_scale * sim_scale);

        /* Get the child BVH and per-instance geometry arrays */
        cuBQL::BinaryBVH<float, 3> child_bvh = blas_array[inst.child_bvh_idx];
        const float3* cur_verts = (const float3*)inst.inst_vertices;
        const uint3*  cur_idxs  = (const uint3*)inst.inst_indices;
        const struct sphere_gpu*  cur_sph = (const struct sphere_gpu*)inst.inst_spheres;
        const unsigned int*       cur_ptg = (const unsigned int*)inst.inst_prim_to_geom;
        const struct geom_gpu_entry* cur_ge = (const struct geom_gpu_entry*)inst.inst_geom_entries;
        uint32_t cur_tri_count = inst.inst_tri_count;

        auto child_candidate = [&](uint32_t primID) -> float {
            uint32_t geom_idx = cur_ptg[primID];
            struct geom_gpu_entry ge = cur_ge[geom_idx];

            if (!ge.is_enabled)
                return local_radius_sq;

            if (primID < cur_tri_count) {
                uint32_t local_id = primID - ge.prim_offset;
                uint3 tri_idx = cur_idxs[ge.prim_offset + local_id];
                float3 v0 = cur_verts[tri_idx.x];
                float3 v1 = cur_verts[tri_idx.y];
                float3 v2 = cur_verts[tri_idx.z];

                struct cp_triangle_result tr = closest_point_triangle_gpu(pos_ls, v0, v1, v2);

                if (tr.sqr_dist < local_radius_sq) {
                    /* Convert distance to world space */
                    float ws_sqr_dist = tr.sqr_dist * sim_scale * sim_scale;
                    if (ws_sqr_dist < best_sqr_dist) {
                        best_sqr_dist = ws_sqr_dist;
                        local_radius_sq = tr.sqr_dist;
                        float dst = sqrtf(ws_sqr_dist);

                        /* Normal in local space, transform to world */
                        float3 e0 = v1 - v0;
                        float3 e1 = v2 - v0;
                        float3 N = cross(e1, e0);
                        if (ge.flip_surface) {
                            N.x = -N.x; N.y = -N.y; N.z = -N.z;
                        }
                        float3 wN;
                        wN.x = Tf[0]*N.x + Tf[3]*N.y + Tf[6]*N.z;
                        wN.y = Tf[1]*N.x + Tf[4]*N.y + Tf[7]*N.z;
                        wN.z = Tf[2]*N.x + Tf[5]*N.y + Tf[8]*N.z;

                        /* Transform closest point to world space */
                        float3 ws_cp;
                        ws_cp.x = Tf[0]*tr.closest_pt[0] + Tf[3]*tr.closest_pt[1] + Tf[6]*tr.closest_pt[2] + Tf[9];
                        ws_cp.y = Tf[1]*tr.closest_pt[0] + Tf[4]*tr.closest_pt[1] + Tf[7]*tr.closest_pt[2] + Tf[10];
                        ws_cp.z = Tf[2]*tr.closest_pt[0] + Tf[5]*tr.closest_pt[1] + Tf[8]*tr.closest_pt[2] + Tf[11];

                        hit.prim_id  = (int32_t)primID;
                        hit.geom_idx = (int32_t)geom_idx;
                        hit.inst_id  = (int32_t)instID;
                        hit.distance = dst;
                        hit.closest_pos[0] = ws_cp.x;
                        hit.closest_pos[1] = ws_cp.y;
                        hit.closest_pos[2] = ws_cp.z;
                        hit.normal[0] = wN.x;
                        hit.normal[1] = wN.y;
                        hit.normal[2] = wN.z;
                        hit.uv[0]     = tr.uv[0];
                        hit.uv[1]     = tr.uv[1];
                    }
                }
            } else {
                uint32_t sphere_idx = primID - cur_tri_count;
                struct sphere_gpu sp = cur_sph[sphere_idx];

                struct cp_sphere_result sr = closest_point_sphere_gpu(pos_ls, sp);

                if (sr.sqr_dist < local_radius_sq) {
                    float ws_sqr_dist = sr.sqr_dist * sim_scale * sim_scale;
                    if (ws_sqr_dist < best_sqr_dist) {
                        best_sqr_dist = ws_sqr_dist;
                        local_radius_sq = sr.sqr_dist;
                        float dst = sqrtf(ws_sqr_dist);

                        float3 N = make_float3(sr.normal[0], sr.normal[1], sr.normal[2]);
                        if (ge.flip_surface) {
                            N.x = -N.x; N.y = -N.y; N.z = -N.z;
                        }
                        float3 wN;
                        wN.x = Tf[0]*N.x + Tf[3]*N.y + Tf[6]*N.z;
                        wN.y = Tf[1]*N.x + Tf[4]*N.y + Tf[7]*N.z;
                        wN.z = Tf[2]*N.x + Tf[5]*N.y + Tf[8]*N.z;

                        float3 ws_cp;
                        ws_cp.x = Tf[0]*sr.closest_pt[0] + Tf[3]*sr.closest_pt[1] + Tf[6]*sr.closest_pt[2] + Tf[9];
                        ws_cp.y = Tf[1]*sr.closest_pt[0] + Tf[4]*sr.closest_pt[1] + Tf[7]*sr.closest_pt[2] + Tf[10];
                        ws_cp.z = Tf[2]*sr.closest_pt[0] + Tf[5]*sr.closest_pt[1] + Tf[8]*sr.closest_pt[2] + Tf[11];

                        hit.prim_id  = (int32_t)primID;
                        hit.geom_idx = (int32_t)geom_idx;
                        hit.inst_id  = (int32_t)instID;
                        hit.distance = dst;
                        hit.closest_pos[0] = ws_cp.x;
                        hit.closest_pos[1] = ws_cp.y;
                        hit.closest_pos[2] = ws_cp.z;
                        hit.normal[0] = wN.x;
                        hit.normal[1] = wN.y;
                        hit.normal[2] = wN.z;
                        hit.uv[0]     = sr.uv[0];
                        hit.uv[1]     = sr.uv[1];
                    }
                }
            }
            return local_radius_sq;
        };

        cuBQL::shrinkingRadiusQuery::forEachPrim(
            child_candidate,
            child_bvh,
            cuBQL::vec3f(pos_ls.x, pos_ls.y, pos_ls.z),
            local_radius_sq);

        return best_sqr_dist;
    };

    /* Use TLAS shrinkingRadiusQuery to prune instances whose AABB
     * doesn't overlap the query sphere.  Each TLAS leaf primID
     * corresponds to an instance index. */
    cuBQL::shrinkingRadiusQuery::forEachPrim(
        tlas_candidate,
        tlas,
        cuBQL::vec3f(pos_ws.x, pos_ws.y, pos_ws.z),
        best_sqr_dist);

    results[tid] = hit;
}

/* ================================================================== */
/*  Host-side API                                                     */
/* ================================================================== */

res_T
cus3d_cp_batch_create(struct cus3d_cp_batch* batch, size_t max_queries)
{
    cp_clear_error();

    if (!batch) {
        cp_set_error("cus3d_cp_batch_create: batch is NULL");
        return RES_ERR;
    }

    memset(batch, 0, sizeof(*batch));

    if (max_queries == 0)
        return RES_OK;

    cudaStream_t s = (cudaStream_t)0;
    res_T res;

    res = gpu_buffer_float3_alloc(&batch->d_positions, max_queries, s);
    if (res != RES_OK) return res;

    cudaError_t cerr = cudaMalloc(&batch->d_radii,
                                  max_queries * sizeof(float));
    if (cerr != cudaSuccess) {
        gpu_buffer_float3_free(&batch->d_positions, s);
        cp_set_error("cus3d_cp_batch_create: cudaMalloc d_radii failed");
        return RES_MEM_ERR;
    }
    batch->d_radii_capacity = max_queries;

    batch->count = 0;
    return RES_OK;
}

void
cus3d_cp_batch_destroy(struct cus3d_cp_batch* batch)
{
    if (!batch) return;

    cudaStream_t s = (cudaStream_t)0;
    gpu_buffer_float3_free(&batch->d_positions, s);
    if (batch->d_radii) {
        cudaFree(batch->d_radii);
        batch->d_radii = NULL;
    }
    batch->d_radii_capacity = 0;
    batch->count = 0;
}

res_T
cus3d_closest_point_batch(
    const struct cus3d_bvh* bvh,
    const struct cus3d_geom_store* store,
    struct cus3d_device* dev,
    const struct cus3d_cp_batch* queries,
    struct cus3d_cp_result* h_results)
{
    cp_clear_error();

    if (!bvh || !store || !dev || !queries || !h_results) {
        cp_set_error("cus3d_closest_point_batch: NULL argument");
        return RES_ERR;
    }

    if (queries->count == 0)
        return RES_OK;

    if (!bvh->valid && !(bvh->tlas_valid && bvh->instance_count > 0)) {
        cp_set_error("cus3d_closest_point_batch: BVH not built");
        return RES_ERR;
    }

    cudaStream_t s = dev->stream;
    uint32_t num_queries = (uint32_t)queries->count;

    /* Allocate device results */
    struct cus3d_cp_result* d_results = NULL;
    CP_CUDA_CHECK(
        cudaMallocAsync(&d_results,
                        num_queries * sizeof(struct cus3d_cp_result), s),
        "alloc d_results");

    const uint32_t block_size = 256;
    uint32_t grid_size = (num_queries + block_size - 1) / block_size;

    if (bvh->tlas_valid && bvh->tlas_count > 0) {
        /* Two-level (instanced): prepare BLAS array + instance data */
        uint32_t n_inst = (uint32_t)bvh->tlas_count;

        cuBQL::BinaryBVH<float, 3>* h_blas_array =
            (cuBQL::BinaryBVH<float, 3>*)malloc(
                n_inst * sizeof(cuBQL::BinaryBVH<float, 3>));
        struct instance_gpu_data* h_instances =
            (struct instance_gpu_data*)malloc(
                n_inst * sizeof(struct instance_gpu_data));
        if (!h_blas_array || !h_instances) {
            free(h_blas_array);
            free(h_instances);
            cudaFreeAsync(d_results, s);
            cp_set_error("cus3d_closest_point_batch: host malloc failed");
            return RES_ERR;
        }

        for (uint32_t i = 0; i < n_inst; ++i) {
            uint32_t orig = bvh->tlas_to_orig[i];
            const instance_bvh_entry* ie = &bvh->instance_bvhs[orig];
            h_blas_array[i] = ie->child_bvh;
            memcpy(h_instances[i].transform, ie->transform, 12 * sizeof(float));
            memcpy(h_instances[i].inv_transform, ie->inv_transform, 12 * sizeof(float));
            h_instances[i].child_bvh_idx     = i;
            h_instances[i].geom_store_offset = ie->child_store_offset;
            const struct cus3d_geom_store* cs = ie->child_store;
            h_instances[i].inst_vertices     = cs ? cs->d_vertices.data : NULL;
            h_instances[i].inst_indices      = cs ? cs->d_indices.data  : NULL;
            h_instances[i].inst_spheres      = cs ? cs->d_spheres       : NULL;
            h_instances[i].inst_prim_to_geom = cs ? cs->d_prim_to_geom.data : NULL;
            h_instances[i].inst_geom_entries = cs ? cs->d_geom_entries   : NULL;
            h_instances[i].inst_tri_count    = cs ? cs->total_tris       : 0;
            h_instances[i].inst_total_prims  = cs ? cs->total_prims      : 0;
        }

        cuBQL::BinaryBVH<float, 3>* d_blas_array = NULL;
        CP_CUDA_CHECK(
            cudaMallocAsync(&d_blas_array,
                            n_inst * sizeof(cuBQL::BinaryBVH<float, 3>), s),
            "alloc d_blas_array");
        CP_CUDA_CHECK(
            cudaMemcpyAsync(d_blas_array, h_blas_array,
                            n_inst * sizeof(cuBQL::BinaryBVH<float, 3>),
                            cudaMemcpyHostToDevice, s),
            "upload blas_array");

        struct instance_gpu_data* d_instances = NULL;
        CP_CUDA_CHECK(
            cudaMallocAsync(&d_instances,
                            n_inst * sizeof(struct instance_gpu_data), s),
            "alloc d_instances");
        CP_CUDA_CHECK(
            cudaMemcpyAsync(d_instances, h_instances,
                            n_inst * sizeof(struct instance_gpu_data),
                            cudaMemcpyHostToDevice, s),
            "upload instances");

        free(h_blas_array);
        free(h_instances);

        closest_point_instanced_kernel<<<grid_size, block_size, 0, s>>>(
            bvh->tlas,
            d_blas_array,
            d_instances,
            n_inst,
            store->d_vertices.data,
            store->d_indices.data,
            store->d_spheres,
            store->d_prim_to_geom.data,
            store->d_geom_entries,
            store->total_tris,
            queries->d_positions.data,
            queries->d_radii,
            num_queries,
            d_results);

        CP_CUDA_CHECK(cudaGetLastError(), "launch closest_point_instanced_kernel");
        CP_CUDA_CHECK(cudaStreamSynchronize(s), "sync after instanced cp");

        cudaFreeAsync(d_blas_array, s);
        cudaFreeAsync(d_instances, s);

    } else {
        /* Single-level (direct geometry) */
        closest_point_kernel<<<grid_size, block_size, 0, s>>>(
            bvh->bvh,
            store->d_vertices.data,
            store->d_indices.data,
            store->d_spheres,
            store->d_prim_to_geom.data,
            store->d_geom_entries,
            store->total_tris,
            queries->d_positions.data,
            queries->d_radii,
            num_queries,
            d_results);

        CP_CUDA_CHECK(cudaGetLastError(), "launch closest_point_kernel");
        CP_CUDA_CHECK(cudaStreamSynchronize(s), "sync after cp");
    }

    /* Download results */
    CP_CUDA_CHECK(
        cudaMemcpyAsync(h_results, d_results,
                        num_queries * sizeof(struct cus3d_cp_result),
                        cudaMemcpyDeviceToHost, s),
        "download cp results");
    CP_CUDA_CHECK(cudaStreamSynchronize(s), "sync after result download");

    cudaFreeAsync(d_results, s);

    return RES_OK;
}
