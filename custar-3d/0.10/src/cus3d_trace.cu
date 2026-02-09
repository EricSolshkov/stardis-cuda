#include "cus3d_bvh_internal.h"
#include "cus3d_trace.h"
#include "cus3d_math.cuh"

#include <cuBQL/traversal/rayQueries.h>
#include <cuBQL/math/Ray.h>
#include <device_launch_parameters.h>

#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <stdarg.h>

#define TRACE_ERR_BUF_SIZE 512

#ifdef _MSC_VER
  static __declspec(thread) char tls_trace_error[TRACE_ERR_BUF_SIZE] = {0};
#else
  static __thread char tls_trace_error[TRACE_ERR_BUF_SIZE] = {0};
#endif

static void
trace_set_error(const char* fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    vsnprintf(tls_trace_error, TRACE_ERR_BUF_SIZE, fmt, args);
    va_end(args);
}

static void
trace_clear_error(void)
{
    tls_trace_error[0] = '\0';
}

#define TRACE_CUDA_CHECK(call, context)                                         \
    do {                                                                        \
        cudaError_t err__ = (call);                                             \
        if (err__ != cudaSuccess) {                                             \
            trace_set_error("cus3d_trace: %s failed -- %s",                     \
                            (context), cudaGetErrorString(err__));               \
            return RES_ERR;                                                     \
        }                                                                       \
    } while (0)

/*******************************************************************************
 * Single-level trace kernel (BLAS only, no instances)
 *
 * Each thread processes one ray.  cuBQL::shrinkingRayQuery::forEachPrim
 * drives BVH traversal; the lambda tests triangle or sphere intersection
 * for each candidate primID and returns the updated tMax to shrink the
 * search range.
 ******************************************************************************/
__global__ void trace_rays_kernel(
    cuBQL::BinaryBVH<float, 3>      bvh,
    const float3* __restrict__      vertices,
    const uint3*  __restrict__      indices,
    const struct sphere_gpu* __restrict__ spheres,
    const unsigned int* __restrict__ prim_to_geom,
    const struct geom_gpu_entry* __restrict__ geom_entries,
    uint32_t                        tri_count,
    const float3* __restrict__      ray_origins,
    const float3* __restrict__      ray_dirs,
    const float2* __restrict__      ray_ranges,
    uint32_t                        num_rays,
    struct cus3d_hit_result*        results)
{
    uint32_t tid = blockIdx.x * blockDim.x + threadIdx.x;
    if (tid >= num_rays) return;

    float3 org = ray_origins[tid];
    float3 dir = ray_dirs[tid];
    float  tmin = ray_ranges[tid].x;
    float  tmax = ray_ranges[tid].y;

    struct cus3d_hit_result hit;
    hit.prim_id  = -1;
    hit.geom_idx = -1;
    hit.inst_id  = -1;
    hit.distance = tmax;
    hit.normal[0] = 0.0f; hit.normal[1] = 0.0f; hit.normal[2] = 0.0f;
    hit.uv[0] = 0.0f; hit.uv[1] = 0.0f;

    cuBQL::ray3f ray(
        cuBQL::vec3f(org.x, org.y, org.z),
        cuBQL::vec3f(dir.x, dir.y, dir.z),
        tmin, tmax);

    auto intersect_prim = [&](uint32_t primID) -> float {
        uint32_t geom_idx = prim_to_geom[primID];
        struct geom_gpu_entry ge = geom_entries[geom_idx];

        if (!ge.is_enabled)
            return ray.tMax;

        if (primID < tri_count) {
            uint32_t local_id = primID - ge.prim_offset;
            uint3 tri_idx = indices[ge.prim_offset + local_id];
            float3 v0 = vertices[tri_idx.x];
            float3 v1 = vertices[tri_idx.y];
            float3 v2 = vertices[tri_idx.z];

            float t, u, v;
            if (ray_triangle_intersect(org, dir, v0, v1, v2,
                                       tmin, ray.tMax, &t, &u, &v)) {
                float3 e1 = v1 - v0;
                float3 e2 = v2 - v0;
                float3 N = cross(e1, e2);
                if (ge.flip_surface) {
                    N.x = -N.x; N.y = -N.y; N.z = -N.z;
                }

                hit.prim_id   = primID;
                hit.geom_idx  = geom_idx;
                hit.distance  = t;
                hit.normal[0] = N.x;
                hit.normal[1] = N.y;
                hit.normal[2] = N.z;
                hit.uv[0]     = u;
                hit.uv[1]     = v;
                ray.tMax       = t;
            }
        } else {
            uint32_t sphere_idx = primID - tri_count;
            struct sphere_gpu sp = spheres[sphere_idx];

            float t;
            float3 N;
            if (ray_sphere_intersect(org, dir, sp.cx, sp.cy, sp.cz,
                                     sp.radius, tmin, ray.tMax, &t, &N)) {
                if (ge.flip_surface) {
                    N.x = -N.x; N.y = -N.y; N.z = -N.z;
                }
                float2 suv = sphere_normal_to_uv(N);

                hit.prim_id   = primID;
                hit.geom_idx  = geom_idx;
                hit.distance  = t;
                hit.normal[0] = N.x;
                hit.normal[1] = N.y;
                hit.normal[2] = N.z;
                hit.uv[0]     = suv.x;
                hit.uv[1]     = suv.y;
                ray.tMax       = t;
            }
        }
        return ray.tMax;
    };

    cuBQL::shrinkingRayQuery::forEachPrim(intersect_prim, bvh, ray);

    results[tid] = hit;
}

/*******************************************************************************
 * Two-level trace kernel (TLAS + BLAS, for instanced scenes)
 *
 * cuBQL twoLevel::forEachPrim handles the TLAS->BLAS transition.
 * enterBlas transforms the ray into instance-local space and provides
 * the child BVH.  The per-prim lambda tests intersection in local space
 * and transforms the resulting normal back to world space.
 ******************************************************************************/
__global__ void trace_rays_instanced_kernel(
    cuBQL::BinaryBVH<float, 3>      tlas,
    const cuBQL::BinaryBVH<float, 3>* __restrict__ blas_array,
    const struct instance_gpu_data*  __restrict__ instances,
    uint32_t                         num_instances,
    const float3* __restrict__       vertices,
    const uint3*  __restrict__       indices,
    const struct sphere_gpu* __restrict__ spheres,
    const unsigned int* __restrict__ prim_to_geom,
    const struct geom_gpu_entry* __restrict__ geom_entries,
    uint32_t                         tri_count,
    const float3* __restrict__       ray_origins,
    const float3* __restrict__       ray_dirs,
    const float2* __restrict__       ray_ranges,
    uint32_t                         num_rays,
    struct cus3d_hit_result*         results)
{
    uint32_t tid = blockIdx.x * blockDim.x + threadIdx.x;
    if (tid >= num_rays) return;

    float3 org = ray_origins[tid];
    float3 dir = ray_dirs[tid];
    float  tmin = ray_ranges[tid].x;
    float  tmax = ray_ranges[tid].y;

    struct cus3d_hit_result hit;
    hit.prim_id  = -1;
    hit.geom_idx = -1;
    hit.inst_id  = -1;
    hit.distance = tmax;
    hit.normal[0] = 0.0f; hit.normal[1] = 0.0f; hit.normal[2] = 0.0f;
    hit.uv[0] = 0.0f; hit.uv[1] = 0.0f;

    int current_inst_id = -1;

    cuBQL::ray3f ray(
        cuBQL::vec3f(org.x, org.y, org.z),
        cuBQL::vec3f(dir.x, dir.y, dir.z),
        tmin, tmax);

    auto enterBlas = [&](cuBQL::ray3f& r,
                         cuBQL::BinaryBVH<float, 3>& blas,
                         int instID) {
        if (instID < 0 || instID >= (int)num_instances) return;
        struct instance_gpu_data inst = instances[instID];
        current_inst_id = instID;

        /* Transform ray to instance local space via inv_transform (3x4 col-major) */
        const float* T = inst.inv_transform;
        float lox = T[0]*org.x + T[3]*org.y + T[6]*org.z + T[9];
        float loy = T[1]*org.x + T[4]*org.y + T[7]*org.z + T[10];
        float loz = T[2]*org.x + T[5]*org.y + T[8]*org.z + T[11];

        float ldx = T[0]*dir.x + T[3]*dir.y + T[6]*dir.z;
        float ldy = T[1]*dir.x + T[4]*dir.y + T[7]*dir.z;
        float ldz = T[2]*dir.x + T[5]*dir.y + T[8]*dir.z;

        r.origin    = cuBQL::vec3f(lox, loy, loz);
        r.direction = cuBQL::vec3f(ldx, ldy, ldz);
        blas = blas_array[inst.child_bvh_idx];
    };

    auto leaveBlas = [&]() {
        current_inst_id = -1;
    };

    auto intersect_prim = [&](uint32_t primID) -> float {
        /* Use per-instance GPU arrays (supports mixed direct+instanced scenes) */
        const struct instance_gpu_data& ci = instances[current_inst_id];
        const unsigned int* cur_ptg = (const unsigned int*)ci.inst_prim_to_geom;
        const struct geom_gpu_entry* cur_ge = (const struct geom_gpu_entry*)ci.inst_geom_entries;
        const float3* cur_verts = (const float3*)ci.inst_vertices;
        const uint3*  cur_idxs  = (const uint3*)ci.inst_indices;
        const struct sphere_gpu* cur_sph = (const struct sphere_gpu*)ci.inst_spheres;
        uint32_t cur_tri_count = ci.inst_tri_count;

        uint32_t geom_idx = cur_ptg[primID];
        struct geom_gpu_entry ge = cur_ge[geom_idx];

        if (!ge.is_enabled)
            return ray.tMax;

        float3 local_org = make_float3(ray.origin.x, ray.origin.y, ray.origin.z);
        float3 local_dir = make_float3(ray.direction.x, ray.direction.y, ray.direction.z);

        if (primID < cur_tri_count) {
            uint32_t local_id = primID - ge.prim_offset;
            uint3 tri_idx = cur_idxs[ge.prim_offset + local_id];
            float3 v0 = cur_verts[tri_idx.x];
            float3 v1 = cur_verts[tri_idx.y];
            float3 v2 = cur_verts[tri_idx.z];

            float t, u, v;
            if (ray_triangle_intersect(local_org, local_dir, v0, v1, v2,
                                       tmin, ray.tMax, &t, &u, &v)) {
                float3 e1 = v1 - v0;
                float3 e2 = v2 - v0;
                float3 N = cross(e1, e2);
                if (ge.flip_surface) {
                    N.x = -N.x; N.y = -N.y; N.z = -N.z;
                }

                /* Transform normal to world space via forward 3x3 rotation */
                if (current_inst_id >= 0) {
                    const float* Tf = instances[current_inst_id].transform;
                    float3 wN;
                    wN.x = Tf[0]*N.x + Tf[3]*N.y + Tf[6]*N.z;
                    wN.y = Tf[1]*N.x + Tf[4]*N.y + Tf[7]*N.z;
                    wN.z = Tf[2]*N.x + Tf[5]*N.y + Tf[8]*N.z;
                    N = wN;
                }

                hit.prim_id   = primID;
                hit.geom_idx  = geom_idx;
                hit.inst_id   = current_inst_id;
                hit.distance  = t;
                hit.normal[0] = N.x;
                hit.normal[1] = N.y;
                hit.normal[2] = N.z;
                hit.uv[0]     = u;
                hit.uv[1]     = v;
                ray.tMax       = t;
            }
        } else {
            uint32_t sphere_idx = primID - cur_tri_count;
            struct sphere_gpu sp = cur_sph[sphere_idx];

            float t;
            float3 N;
            if (ray_sphere_intersect(local_org, local_dir, sp.cx, sp.cy, sp.cz,
                                     sp.radius, tmin, ray.tMax, &t, &N)) {
                if (ge.flip_surface) {
                    N.x = -N.x; N.y = -N.y; N.z = -N.z;
                }

                if (current_inst_id >= 0) {
                    const float* Tf = instances[current_inst_id].transform;
                    float3 wN;
                    wN.x = Tf[0]*N.x + Tf[3]*N.y + Tf[6]*N.z;
                    wN.y = Tf[1]*N.x + Tf[4]*N.y + Tf[7]*N.z;
                    wN.z = Tf[2]*N.x + Tf[5]*N.y + Tf[8]*N.z;
                    N = wN;
                }

                float2 suv = sphere_normal_to_uv(N);

                hit.prim_id   = primID;
                hit.geom_idx  = geom_idx;
                hit.inst_id   = current_inst_id;
                hit.distance  = t;
                hit.normal[0] = N.x;
                hit.normal[1] = N.y;
                hit.normal[2] = N.z;
                hit.uv[0]     = suv.x;
                hit.uv[1]     = suv.y;
                ray.tMax       = t;
            }
        }
        return ray.tMax;
    };

    cuBQL::shrinkingRayQuery::twoLevel::forEachPrim(
        enterBlas, leaveBlas, intersect_prim, tlas, ray);

    results[tid] = hit;
}

/*******************************************************************************
 * Top-K single-level trace kernel
 *
 * Maintains a small sorted array of the K nearest hits.
 * ray.tMax shrinks to the K-th hit distance, enabling BVH culling.
 ******************************************************************************/
__global__ void trace_rays_topk_kernel(
    cuBQL::BinaryBVH<float, 3>      bvh,
    const float3* __restrict__      vertices,
    const uint3*  __restrict__      indices,
    const struct sphere_gpu* __restrict__ spheres,
    const unsigned int* __restrict__ prim_to_geom,
    const struct geom_gpu_entry* __restrict__ geom_entries,
    uint32_t                        tri_count,
    const float3* __restrict__      ray_origins,
    const float3* __restrict__      ray_dirs,
    const float2* __restrict__      ray_ranges,
    uint32_t                        num_rays,
    int                             max_k,
    struct cus3d_multi_hit_result*  results)
{
    uint32_t tid = blockIdx.x * blockDim.x + threadIdx.x;
    if (tid >= num_rays) return;

    float3 org = ray_origins[tid];
    float3 dir = ray_dirs[tid];
    float  tmin = ray_ranges[tid].x;
    float  tmax = ray_ranges[tid].y;

    struct cus3d_hit_result slots[CUS3D_MAX_MULTI_HITS];
    int num_hits = 0;
    int K = max_k < CUS3D_MAX_MULTI_HITS ? max_k : CUS3D_MAX_MULTI_HITS;
    if (K < 1) K = 1;

    cuBQL::ray3f ray(
        cuBQL::vec3f(org.x, org.y, org.z),
        cuBQL::vec3f(dir.x, dir.y, dir.z),
        tmin, tmax);

    auto intersect_prim = [&](uint32_t primID) -> float {
        uint32_t geom_idx = prim_to_geom[primID];
        struct geom_gpu_entry ge = geom_entries[geom_idx];

        if (!ge.is_enabled)
            return ray.tMax;

        float t = -1.0f, u = 0.0f, v = 0.0f;
        float Nx = 0.0f, Ny = 0.0f, Nz = 0.0f;
        bool did_hit = false;

        if (primID < tri_count) {
            uint32_t local_id = primID - ge.prim_offset;
            uint3 tri_idx = indices[ge.prim_offset + local_id];
            float3 v0 = vertices[tri_idx.x];
            float3 v1 = vertices[tri_idx.y];
            float3 v2 = vertices[tri_idx.z];

            if (ray_triangle_intersect(org, dir, v0, v1, v2,
                                       tmin, ray.tMax, &t, &u, &v)) {
                float3 e1 = v1 - v0;
                float3 e2 = v2 - v0;
                float3 N = cross(e1, e2);
                if (ge.flip_surface) { N.x = -N.x; N.y = -N.y; N.z = -N.z; }
                Nx = N.x; Ny = N.y; Nz = N.z;
                did_hit = true;
            }
        } else {
            uint32_t sphere_idx = primID - tri_count;
            struct sphere_gpu sp = spheres[sphere_idx];

            float3 N;
            if (ray_sphere_intersect(org, dir, sp.cx, sp.cy, sp.cz,
                                     sp.radius, tmin, ray.tMax, &t, &N)) {
                if (ge.flip_surface) { N.x = -N.x; N.y = -N.y; N.z = -N.z; }
                float2 suv = sphere_normal_to_uv(N);
                u = suv.x; v = suv.y;
                Nx = N.x; Ny = N.y; Nz = N.z;
                did_hit = true;
            }
        }

        if (!did_hit)
            return ray.tMax;

        /* Insert into sorted Top-K array (insertion sort, K<=8) */
        if (num_hits < K) {
            int pos = num_hits;
            while (pos > 0 && slots[pos - 1].distance > t) {
                slots[pos] = slots[pos - 1];
                pos--;
            }
            slots[pos].prim_id   = (int32_t)primID;
            slots[pos].geom_idx  = (int32_t)geom_idx;
            slots[pos].inst_id   = -1;
            slots[pos].distance  = t;
            slots[pos].normal[0] = Nx;
            slots[pos].normal[1] = Ny;
            slots[pos].normal[2] = Nz;
            slots[pos].uv[0]     = u;
            slots[pos].uv[1]     = v;
            num_hits++;

            if (num_hits == K)
                ray.tMax = slots[K - 1].distance;

        } else if (t < slots[K - 1].distance) {
            int pos = K - 1;
            while (pos > 0 && slots[pos - 1].distance > t) {
                slots[pos] = slots[pos - 1];
                pos--;
            }
            slots[pos].prim_id   = (int32_t)primID;
            slots[pos].geom_idx  = (int32_t)geom_idx;
            slots[pos].inst_id   = -1;
            slots[pos].distance  = t;
            slots[pos].normal[0] = Nx;
            slots[pos].normal[1] = Ny;
            slots[pos].normal[2] = Nz;
            slots[pos].uv[0]     = u;
            slots[pos].uv[1]     = v;

            ray.tMax = slots[K - 1].distance;
        }

        return ray.tMax;
    };

    cuBQL::shrinkingRayQuery::forEachPrim(intersect_prim, bvh, ray);

    results[tid].count = num_hits;
    for (int i = 0; i < num_hits; i++)
        results[tid].hits[i] = slots[i];
    for (int i = num_hits; i < CUS3D_MAX_MULTI_HITS; i++)
        results[tid].hits[i].prim_id = -1;
}

/*******************************************************************************
 * Two-level Top-K trace kernel (TLAS + BLAS, for instanced scenes)
 *
 * Combines the two-level traversal of trace_rays_instanced_kernel with the
 * Top-K sorted candidate tracking of trace_rays_topk_kernel.
 * Uses cuBQL::shrinkingRayQuery::twoLevel::forEachPrim.
 ******************************************************************************/
__global__ void trace_rays_instanced_topk_kernel(
    cuBQL::BinaryBVH<float, 3>      tlas,
    const cuBQL::BinaryBVH<float, 3>* __restrict__ blas_array,
    const struct instance_gpu_data*  __restrict__ instances,
    uint32_t                         num_instances,
    const float3* __restrict__       vertices,
    const uint3*  __restrict__       indices,
    const struct sphere_gpu* __restrict__ spheres,
    const unsigned int* __restrict__ prim_to_geom,
    const struct geom_gpu_entry* __restrict__ geom_entries,
    uint32_t                         tri_count,
    const float3* __restrict__       ray_origins,
    const float3* __restrict__       ray_dirs,
    const float2* __restrict__       ray_ranges,
    uint32_t                         num_rays,
    int                              max_k,
    struct cus3d_multi_hit_result*   results)
{
    uint32_t tid = blockIdx.x * blockDim.x + threadIdx.x;
    if (tid >= num_rays) return;

    float3 org = ray_origins[tid];
    float3 dir = ray_dirs[tid];
    float  tmin = ray_ranges[tid].x;
    float  tmax = ray_ranges[tid].y;

    struct cus3d_hit_result slots[CUS3D_MAX_MULTI_HITS];
    int num_hits = 0;
    int K = max_k < CUS3D_MAX_MULTI_HITS ? max_k : CUS3D_MAX_MULTI_HITS;
    if (K < 1) K = 1;

    int current_inst_id = -1;

    cuBQL::ray3f ray(
        cuBQL::vec3f(org.x, org.y, org.z),
        cuBQL::vec3f(dir.x, dir.y, dir.z),
        tmin, tmax);

    auto enterBlas = [&](cuBQL::ray3f& r,
                         cuBQL::BinaryBVH<float, 3>& blas,
                         int instID) {
        if (instID < 0 || instID >= (int)num_instances) return;
        struct instance_gpu_data inst = instances[instID];
        current_inst_id = instID;

        /* Transform ray to instance local space via inv_transform (3x4 col-major) */
        const float* T = inst.inv_transform;
        float lox = T[0]*org.x + T[3]*org.y + T[6]*org.z + T[9];
        float loy = T[1]*org.x + T[4]*org.y + T[7]*org.z + T[10];
        float loz = T[2]*org.x + T[5]*org.y + T[8]*org.z + T[11];

        float ldx = T[0]*dir.x + T[3]*dir.y + T[6]*dir.z;
        float ldy = T[1]*dir.x + T[4]*dir.y + T[7]*dir.z;
        float ldz = T[2]*dir.x + T[5]*dir.y + T[8]*dir.z;

        r.origin    = cuBQL::vec3f(lox, loy, loz);
        r.direction = cuBQL::vec3f(ldx, ldy, ldz);
        blas = blas_array[inst.child_bvh_idx];
    };

    auto leaveBlas = [&]() {
        current_inst_id = -1;
    };

    auto intersect_prim = [&](uint32_t primID) -> float {
        /* Use per-instance GPU arrays (supports mixed direct+instanced scenes) */
        const struct instance_gpu_data& ci = instances[current_inst_id];
        const unsigned int* cur_ptg = (const unsigned int*)ci.inst_prim_to_geom;
        const struct geom_gpu_entry* cur_ge = (const struct geom_gpu_entry*)ci.inst_geom_entries;
        const float3* cur_verts = (const float3*)ci.inst_vertices;
        const uint3*  cur_idxs  = (const uint3*)ci.inst_indices;
        const struct sphere_gpu* cur_sph = (const struct sphere_gpu*)ci.inst_spheres;
        uint32_t cur_tri_count = ci.inst_tri_count;

        uint32_t geom_idx = cur_ptg[primID];
        struct geom_gpu_entry ge = cur_ge[geom_idx];

        if (!ge.is_enabled)
            return ray.tMax;

        /* Use local-space ray from cuBQL (already transformed by enterBlas) */
        float3 local_org = make_float3(ray.origin.x, ray.origin.y, ray.origin.z);
        float3 local_dir = make_float3(ray.direction.x, ray.direction.y, ray.direction.z);

        float t = -1.0f, u = 0.0f, v = 0.0f;
        float Nx = 0.0f, Ny = 0.0f, Nz = 0.0f;
        bool did_hit = false;

        if (primID < cur_tri_count) {
            uint32_t local_id = primID - ge.prim_offset;
            uint3 tri_idx = cur_idxs[ge.prim_offset + local_id];
            float3 v0 = cur_verts[tri_idx.x];
            float3 v1 = cur_verts[tri_idx.y];
            float3 v2 = cur_verts[tri_idx.z];

            if (ray_triangle_intersect(local_org, local_dir, v0, v1, v2,
                                       tmin, ray.tMax, &t, &u, &v)) {
                float3 e1 = v1 - v0;
                float3 e2 = v2 - v0;
                float3 N = cross(e1, e2);
                if (ge.flip_surface) { N.x = -N.x; N.y = -N.y; N.z = -N.z; }

                /* Transform normal to world space via forward 3x3 rotation */
                if (current_inst_id >= 0) {
                    const float* Tf = instances[current_inst_id].transform;
                    float3 wN;
                    wN.x = Tf[0]*N.x + Tf[3]*N.y + Tf[6]*N.z;
                    wN.y = Tf[1]*N.x + Tf[4]*N.y + Tf[7]*N.z;
                    wN.z = Tf[2]*N.x + Tf[5]*N.y + Tf[8]*N.z;
                    N = wN;
                }
                Nx = N.x; Ny = N.y; Nz = N.z;
                did_hit = true;
            }
        } else {
            uint32_t sphere_idx = primID - cur_tri_count;
            struct sphere_gpu sp = cur_sph[sphere_idx];

            float3 N;
            if (ray_sphere_intersect(local_org, local_dir, sp.cx, sp.cy, sp.cz,
                                     sp.radius, tmin, ray.tMax, &t, &N)) {
                if (ge.flip_surface) { N.x = -N.x; N.y = -N.y; N.z = -N.z; }

                if (current_inst_id >= 0) {
                    const float* Tf = instances[current_inst_id].transform;
                    float3 wN;
                    wN.x = Tf[0]*N.x + Tf[3]*N.y + Tf[6]*N.z;
                    wN.y = Tf[1]*N.x + Tf[4]*N.y + Tf[7]*N.z;
                    wN.z = Tf[2]*N.x + Tf[5]*N.y + Tf[8]*N.z;
                    N = wN;
                }
                float2 suv = sphere_normal_to_uv(N);
                u = suv.x; v = suv.y;
                Nx = N.x; Ny = N.y; Nz = N.z;
                did_hit = true;
            }
        }

        if (!did_hit)
            return ray.tMax;

        /* Insert into sorted Top-K array (insertion sort, K<=8) */
        if (num_hits < K) {
            int pos = num_hits;
            while (pos > 0 && slots[pos - 1].distance > t) {
                slots[pos] = slots[pos - 1];
                pos--;
            }
            slots[pos].prim_id   = (int32_t)primID;
            slots[pos].geom_idx  = (int32_t)geom_idx;
            slots[pos].inst_id   = current_inst_id;
            slots[pos].distance  = t;
            slots[pos].normal[0] = Nx;
            slots[pos].normal[1] = Ny;
            slots[pos].normal[2] = Nz;
            slots[pos].uv[0]     = u;
            slots[pos].uv[1]     = v;
            num_hits++;

            if (num_hits == K)
                ray.tMax = slots[K - 1].distance;

        } else if (t < slots[K - 1].distance) {
            int pos = K - 1;
            while (pos > 0 && slots[pos - 1].distance > t) {
                slots[pos] = slots[pos - 1];
                pos--;
            }
            slots[pos].prim_id   = (int32_t)primID;
            slots[pos].geom_idx  = (int32_t)geom_idx;
            slots[pos].inst_id   = current_inst_id;
            slots[pos].distance  = t;
            slots[pos].normal[0] = Nx;
            slots[pos].normal[1] = Ny;
            slots[pos].normal[2] = Nz;
            slots[pos].uv[0]     = u;
            slots[pos].uv[1]     = v;

            ray.tMax = slots[K - 1].distance;
        }

        return ray.tMax;
    };

    cuBQL::shrinkingRayQuery::twoLevel::forEachPrim(
        enterBlas, leaveBlas, intersect_prim, tlas, ray);

    results[tid].count = num_hits;
    for (int i = 0; i < num_hits; i++)
        results[tid].hits[i] = slots[i];
    for (int i = num_hits; i < CUS3D_MAX_MULTI_HITS; i++)
        results[tid].hits[i].prim_id = -1;
}

/*******************************************************************************
 * Host-side API
 ******************************************************************************/

res_T
cus3d_ray_batch_create(struct cus3d_ray_batch* batch, size_t max_rays)
{
    trace_clear_error();

    if (!batch) {
        trace_set_error("cus3d_ray_batch_create: batch is NULL");
        return RES_ERR;
    }

    memset(batch, 0, sizeof(*batch));

    if (max_rays == 0)
        return RES_OK;

    cudaStream_t s = (cudaStream_t)0;
    res_T res;

    res = gpu_buffer_float3_alloc(&batch->d_origins, max_rays, s);
    if (res != RES_OK) return res;

    res = gpu_buffer_float3_alloc(&batch->d_directions, max_rays, s);
    if (res != RES_OK) {
        gpu_buffer_float3_free(&batch->d_origins, s);
        return res;
    }

    res = gpu_buffer_float2_alloc(&batch->d_ranges, max_rays, s);
    if (res != RES_OK) {
        gpu_buffer_float3_free(&batch->d_origins, s);
        gpu_buffer_float3_free(&batch->d_directions, s);
        return res;
    }

    res = gpu_buffer_uint32_alloc(&batch->d_ray_data_offsets, max_rays, s);
    if (res != RES_OK) {
        gpu_buffer_float3_free(&batch->d_origins, s);
        gpu_buffer_float3_free(&batch->d_directions, s);
        gpu_buffer_float2_free(&batch->d_ranges, s);
        return res;
    }

    batch->count = 0;
    return RES_OK;
}

void
cus3d_ray_batch_destroy(struct cus3d_ray_batch* batch)
{
    if (!batch) return;

    cudaStream_t s = (cudaStream_t)0;
    gpu_buffer_float3_free(&batch->d_origins, s);
    gpu_buffer_float3_free(&batch->d_directions, s);
    gpu_buffer_float2_free(&batch->d_ranges, s);
    gpu_buffer_uint32_free(&batch->d_ray_data_offsets, s);
    batch->count = 0;
}

res_T
cus3d_trace_ray_batch(
    const struct cus3d_bvh* bvh,
    const struct cus3d_geom_store* store,
    struct cus3d_device* dev,
    const struct cus3d_ray_batch* rays,
    struct cus3d_hit_result* h_results)
{
    trace_clear_error();

    if (!bvh || !store || !dev || !rays || !h_results) {
        trace_set_error("cus3d_trace_ray_batch: NULL argument");
        return RES_ERR;
    }

    if (rays->count == 0)
        return RES_OK;

    if (!bvh->valid) {
        trace_set_error("cus3d_trace_ray_batch: BVH not built");
        return RES_ERR;
    }

    cudaStream_t s = dev->stream;
    uint32_t num_rays = (uint32_t)rays->count;

    struct cus3d_hit_result* d_results = NULL;
    TRACE_CUDA_CHECK(
        cudaMallocAsync(&d_results,
                        num_rays * sizeof(struct cus3d_hit_result), s),
        "alloc d_results");

    const uint32_t block_size = 256;
    uint32_t grid_size = (num_rays + block_size - 1) / block_size;

    if (bvh->tlas_valid && bvh->tlas_count > 0) {
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
            trace_set_error("cus3d_trace_ray_batch: host malloc failed");
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
            /* Per-instance GPU array pointers for mixed scenes */
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
        TRACE_CUDA_CHECK(
            cudaMallocAsync(&d_blas_array,
                            n_inst * sizeof(cuBQL::BinaryBVH<float, 3>), s),
            "alloc d_blas_array");
        TRACE_CUDA_CHECK(
            cudaMemcpyAsync(d_blas_array, h_blas_array,
                            n_inst * sizeof(cuBQL::BinaryBVH<float, 3>),
                            cudaMemcpyHostToDevice, s),
            "upload blas_array");

        struct instance_gpu_data* d_instances = NULL;
        TRACE_CUDA_CHECK(
            cudaMallocAsync(&d_instances,
                            n_inst * sizeof(struct instance_gpu_data), s),
            "alloc d_instances");
        TRACE_CUDA_CHECK(
            cudaMemcpyAsync(d_instances, h_instances,
                            n_inst * sizeof(struct instance_gpu_data),
                            cudaMemcpyHostToDevice, s),
            "upload instances");

        free(h_blas_array);
        free(h_instances);

        trace_rays_instanced_kernel<<<grid_size, block_size, 0, s>>>(
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
            rays->d_origins.data,
            rays->d_directions.data,
            rays->d_ranges.data,
            num_rays,
            d_results);

        TRACE_CUDA_CHECK(cudaGetLastError(), "launch trace_rays_instanced_kernel");
        TRACE_CUDA_CHECK(cudaStreamSynchronize(s), "sync after instanced trace");

        cudaFreeAsync(d_blas_array, s);
        cudaFreeAsync(d_instances, s);

    } else {
        trace_rays_kernel<<<grid_size, block_size, 0, s>>>(
            bvh->bvh,
            store->d_vertices.data,
            store->d_indices.data,
            store->d_spheres,
            store->d_prim_to_geom.data,
            store->d_geom_entries,
            store->total_tris,
            rays->d_origins.data,
            rays->d_directions.data,
            rays->d_ranges.data,
            num_rays,
            d_results);

        TRACE_CUDA_CHECK(cudaGetLastError(), "launch trace_rays_kernel");
        TRACE_CUDA_CHECK(cudaStreamSynchronize(s), "sync after trace");
    }

    TRACE_CUDA_CHECK(
        cudaMemcpyAsync(h_results, d_results,
                        num_rays * sizeof(struct cus3d_hit_result),
                        cudaMemcpyDeviceToHost, s),
        "download hit results");
    TRACE_CUDA_CHECK(cudaStreamSynchronize(s), "sync after result download");

    cudaFreeAsync(d_results, s);

    return RES_OK;
}

res_T
cus3d_trace_ray_single(
    const struct cus3d_bvh* bvh,
    const struct cus3d_geom_store* store,
    struct cus3d_device* dev,
    const float origin[3],
    const float direction[3],
    const float range[2],
    struct cus3d_hit_result* result)
{
    trace_clear_error();

    if (!bvh || !store || !dev || !origin || !direction || !range || !result) {
        trace_set_error("cus3d_trace_ray_single: NULL argument");
        return RES_ERR;
    }

    if (!bvh->valid) {
        result->prim_id = -1;
        return RES_OK;
    }

    cudaStream_t s = dev->stream;

    float3 h_org = make_float3(origin[0], origin[1], origin[2]);
    float3 h_dir = make_float3(direction[0], direction[1], direction[2]);
    float2 h_rng = make_float2(range[0], range[1]);

    float3* d_org = NULL;
    float3* d_dir = NULL;
    float2* d_rng = NULL;
    struct cus3d_hit_result* d_res = NULL;

    TRACE_CUDA_CHECK(cudaMallocAsync(&d_org, sizeof(float3), s), "alloc d_org");
    TRACE_CUDA_CHECK(cudaMallocAsync(&d_dir, sizeof(float3), s), "alloc d_dir");
    TRACE_CUDA_CHECK(cudaMallocAsync(&d_rng, sizeof(float2), s), "alloc d_rng");
    TRACE_CUDA_CHECK(cudaMallocAsync(&d_res, sizeof(struct cus3d_hit_result), s),
                     "alloc d_res");

    TRACE_CUDA_CHECK(
        cudaMemcpyAsync(d_org, &h_org, sizeof(float3), cudaMemcpyHostToDevice, s),
        "upload origin");
    TRACE_CUDA_CHECK(
        cudaMemcpyAsync(d_dir, &h_dir, sizeof(float3), cudaMemcpyHostToDevice, s),
        "upload direction");
    TRACE_CUDA_CHECK(
        cudaMemcpyAsync(d_rng, &h_rng, sizeof(float2), cudaMemcpyHostToDevice, s),
        "upload range");

    if (bvh->tlas_valid && bvh->tlas_count > 0) {
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
            cudaFreeAsync(d_org, s);
            cudaFreeAsync(d_dir, s);
            cudaFreeAsync(d_rng, s);
            cudaFreeAsync(d_res, s);
            trace_set_error("cus3d_trace_ray_single: host malloc failed");
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
            /* Per-instance GPU array pointers for mixed scenes */
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
        TRACE_CUDA_CHECK(
            cudaMallocAsync(&d_blas_array,
                            n_inst * sizeof(cuBQL::BinaryBVH<float, 3>), s),
            "alloc d_blas_array");
        TRACE_CUDA_CHECK(
            cudaMemcpyAsync(d_blas_array, h_blas_array,
                            n_inst * sizeof(cuBQL::BinaryBVH<float, 3>),
                            cudaMemcpyHostToDevice, s),
            "upload blas_array");

        struct instance_gpu_data* d_instances = NULL;
        TRACE_CUDA_CHECK(
            cudaMallocAsync(&d_instances,
                            n_inst * sizeof(struct instance_gpu_data), s),
            "alloc d_instances");
        TRACE_CUDA_CHECK(
            cudaMemcpyAsync(d_instances, h_instances,
                            n_inst * sizeof(struct instance_gpu_data),
                            cudaMemcpyHostToDevice, s),
            "upload instances");

        free(h_blas_array);
        free(h_instances);

        trace_rays_instanced_kernel<<<1, 1, 0, s>>>(
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
            d_org, d_dir, d_rng,
            1,
            d_res);

        TRACE_CUDA_CHECK(cudaGetLastError(),
                         "launch trace_rays_instanced_kernel (single)");
        TRACE_CUDA_CHECK(cudaStreamSynchronize(s),
                         "sync after instanced trace (single)");

        cudaFreeAsync(d_blas_array, s);
        cudaFreeAsync(d_instances, s);

    } else {
        trace_rays_kernel<<<1, 1, 0, s>>>(
            bvh->bvh,
            store->d_vertices.data,
            store->d_indices.data,
            store->d_spheres,
            store->d_prim_to_geom.data,
            store->d_geom_entries,
            store->total_tris,
            d_org, d_dir, d_rng,
            1,
            d_res);

        TRACE_CUDA_CHECK(cudaGetLastError(),
                         "launch trace_rays_kernel (single)");
        TRACE_CUDA_CHECK(cudaStreamSynchronize(s),
                         "sync after trace (single)");
    }

    TRACE_CUDA_CHECK(
        cudaMemcpyAsync(result, d_res, sizeof(struct cus3d_hit_result),
                        cudaMemcpyDeviceToHost, s),
        "download single hit result");
    TRACE_CUDA_CHECK(cudaStreamSynchronize(s),
                     "sync after single result download");

    cudaFreeAsync(d_org, s);
    cudaFreeAsync(d_dir, s);
    cudaFreeAsync(d_rng, s);
    cudaFreeAsync(d_res, s);

    return RES_OK;
}

res_T
cus3d_trace_ray_single_multi(
    const struct cus3d_bvh* bvh,
    const struct cus3d_geom_store* store,
    struct cus3d_device* dev,
    const float origin[3],
    const float direction[3],
    const float range[2],
    int max_hits,
    struct cus3d_multi_hit_result* result)
{
    trace_clear_error();

    if (!bvh || !store || !dev || !origin || !direction || !range || !result) {
        trace_set_error("cus3d_trace_ray_single_multi: NULL argument");
        return RES_ERR;
    }

    if (max_hits < 1) max_hits = 1;
    if (max_hits > CUS3D_MAX_MULTI_HITS) max_hits = CUS3D_MAX_MULTI_HITS;

    if (!bvh->valid && !(bvh->tlas_valid && bvh->instance_count > 0)) {
        result->count = 0;
        return RES_OK;
    }

    cudaStream_t s = dev->stream;

    float3 h_org = make_float3(origin[0], origin[1], origin[2]);
    float3 h_dir = make_float3(direction[0], direction[1], direction[2]);
    float2 h_rng = make_float2(range[0], range[1]);

    float3* d_org = NULL;
    float3* d_dir = NULL;
    float2* d_rng = NULL;
    struct cus3d_multi_hit_result* d_res = NULL;

    TRACE_CUDA_CHECK(cudaMallocAsync(&d_org, sizeof(float3), s), "alloc d_org");
    TRACE_CUDA_CHECK(cudaMallocAsync(&d_dir, sizeof(float3), s), "alloc d_dir");
    TRACE_CUDA_CHECK(cudaMallocAsync(&d_rng, sizeof(float2), s), "alloc d_rng");
    TRACE_CUDA_CHECK(cudaMallocAsync(&d_res, sizeof(struct cus3d_multi_hit_result), s),
                     "alloc d_res");

    TRACE_CUDA_CHECK(
        cudaMemcpyAsync(d_org, &h_org, sizeof(float3), cudaMemcpyHostToDevice, s),
        "upload origin");
    TRACE_CUDA_CHECK(
        cudaMemcpyAsync(d_dir, &h_dir, sizeof(float3), cudaMemcpyHostToDevice, s),
        "upload direction");
    TRACE_CUDA_CHECK(
        cudaMemcpyAsync(d_rng, &h_rng, sizeof(float2), cudaMemcpyHostToDevice, s),
        "upload range");

    if (bvh->tlas_valid && bvh->tlas_count > 0) {
        /* --- Two-level instanced Top-K path --- */
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
            cudaFreeAsync(d_org, s);
            cudaFreeAsync(d_dir, s);
            cudaFreeAsync(d_rng, s);
            cudaFreeAsync(d_res, s);
            trace_set_error("cus3d_trace_ray_single_multi: host malloc failed");
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
            /* Per-instance GPU array pointers for mixed scenes */
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
        TRACE_CUDA_CHECK(
            cudaMallocAsync(&d_blas_array,
                            n_inst * sizeof(cuBQL::BinaryBVH<float, 3>), s),
            "alloc d_blas_array");
        TRACE_CUDA_CHECK(
            cudaMemcpyAsync(d_blas_array, h_blas_array,
                            n_inst * sizeof(cuBQL::BinaryBVH<float, 3>),
                            cudaMemcpyHostToDevice, s),
            "upload blas_array");

        struct instance_gpu_data* d_instances = NULL;
        TRACE_CUDA_CHECK(
            cudaMallocAsync(&d_instances,
                            n_inst * sizeof(struct instance_gpu_data), s),
            "alloc d_instances");
        TRACE_CUDA_CHECK(
            cudaMemcpyAsync(d_instances, h_instances,
                            n_inst * sizeof(struct instance_gpu_data),
                            cudaMemcpyHostToDevice, s),
            "upload instances");

        free(h_blas_array);
        free(h_instances);

        trace_rays_instanced_topk_kernel<<<1, 1, 0, s>>>(
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
            d_org, d_dir, d_rng,
            1,
            max_hits,
            d_res);

        TRACE_CUDA_CHECK(cudaGetLastError(),
                         "launch trace_rays_instanced_topk_kernel (single)");
        TRACE_CUDA_CHECK(cudaStreamSynchronize(s),
                         "sync after instanced topk trace (single)");

        cudaFreeAsync(d_blas_array, s);
        cudaFreeAsync(d_instances, s);

    } else {
        /* --- Single-level BLAS-only Top-K path --- */
        trace_rays_topk_kernel<<<1, 1, 0, s>>>(
            bvh->bvh,
            store->d_vertices.data,
            store->d_indices.data,
            store->d_spheres,
            store->d_prim_to_geom.data,
            store->d_geom_entries,
            store->total_tris,
            d_org, d_dir, d_rng,
            1,
            max_hits,
            d_res);

        TRACE_CUDA_CHECK(cudaGetLastError(),
                         "launch trace_rays_topk_kernel (single)");
        TRACE_CUDA_CHECK(cudaStreamSynchronize(s),
                         "sync after topk trace (single)");
    }

    TRACE_CUDA_CHECK(
        cudaMemcpyAsync(result, d_res, sizeof(struct cus3d_multi_hit_result),
                        cudaMemcpyDeviceToHost, s),
        "download multi hit result");
    TRACE_CUDA_CHECK(cudaStreamSynchronize(s),
                     "sync after multi result download");

    cudaFreeAsync(d_org, s);
    cudaFreeAsync(d_dir, s);
    cudaFreeAsync(d_rng, s);
    cudaFreeAsync(d_res, s);

    return RES_OK;
}
