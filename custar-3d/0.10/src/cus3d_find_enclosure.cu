/**
 * @file cus3d_find_enclosure.cu
 * @brief GPU point-in-enclosure kernels + host-side batch API
 *
 * Phase B-4 M10: BVH-accelerated point-in-enclosure query.
 *
 * Uses a two-phase approach per query point:
 *   1. BVH closest-primitive (shrinkingRadiusQuery) for distance/degenerate
 *      detection.
 *   2. 6-ray side determination: cast up to 6 PI/4-rotated axis-aligned
 *      rays through the same BVH, using dot(ray_dir, hit_normal) to
 *      determine front/back side — matching the CPU algorithm
 *      scene_get_enclosure_id_in_closed_boundaries().
 *
 * The 6-ray approach replaces the original dot(P-Q, N) method which
 * failed at concave corners where the nearest face is not the
 * enclosing face.  With ray casting, the ray penetrates the correct
 * face regardless of concavity.
 *
 * The caller (solver layer) maps prim_id + side to enclosure ID via
 * scene_get_enclosure_ids().
 *
 * Architecture parallels cus3d_closest_point.cu:
 *   - find_enclosure_kernel (single-level BLAS)
 *   - find_enclosure_instanced_kernel (two-level TLAS+BLAS)
 *   - cus3d_enc_batch_create / destroy
 *   - cus3d_find_enclosure_batch (host-side launch + download)
 */

#include "cus3d_find_enclosure.h"
#include "cus3d_bvh_internal.h"
#include "cus3d_mem.h"
#include "cus3d_math.cuh"

#include <cuBQL/bvh.h>
#include <cuBQL/traversal/shrinkingRadiusQuery.h>
#include <cuBQL/traversal/rayQueries.h>

#include <cuda_runtime.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <float.h>

/* ------------------------------------------------------------------ */
/*  Error handling (mirrors cus3d_closest_point.cu)                   */
/* ------------------------------------------------------------------ */

#ifdef _MSC_VER
  static __declspec(thread) char s_enc_error_msg[512] = {0};
#else
  static __thread char s_enc_error_msg[512] = {0};
#endif

static void enc_clear_error(void) { s_enc_error_msg[0] = '\0'; }

static void enc_set_error(const char* msg) {
    if (msg) {
        strncpy(s_enc_error_msg, msg, sizeof(s_enc_error_msg) - 1);
        s_enc_error_msg[sizeof(s_enc_error_msg) - 1] = '\0';
    }
}

#define ENC_CUDA_CHECK(call, context) \
    do { \
        cudaError_t err__ = (call); \
        if (err__ != cudaSuccess) { \
            char buf__[512]; \
            snprintf(buf__, sizeof(buf__), \
                     "%s failed: %s (%s:%d)", \
                     context, cudaGetErrorString(err__), __FILE__, __LINE__); \
            enc_set_error(buf__); \
            return RES_ERR; \
        } \
    } while (0)

/* ------------------------------------------------------------------ */
/*  Import closest-point device functions                             */
/*                                                                    */
/*  These are identical to cus3d_closest_point.cu but we duplicate    */
/*  them here to avoid cross-TU device function linkage issues.       */
/*  Both use double-precision math for the 1e-6 tolerance requirement.*/
/* ------------------------------------------------------------------ */

struct enc_triangle_result {
    float  sqr_dist;
    float  closest_pt[3];
};

__device__ static struct enc_triangle_result
enc_closest_point_triangle(
    const float3& query,
    const float3& va,
    const float3& vb,
    const float3& vc)
{
    struct enc_triangle_result res;
    res.sqr_dist = INFINITY;

    double p[3] = { (double)query.x, (double)query.y, (double)query.z };
    double a[3] = { (double)va.x, (double)va.y, (double)va.z };
    double b[3] = { (double)vb.x, (double)vb.y, (double)vb.z };
    double c[3] = { (double)vc.x, (double)vc.y, (double)vc.z };

    double ab[3] = { b[0]-a[0], b[1]-a[1], b[2]-a[2] };
    double ac[3] = { c[0]-a[0], c[1]-a[1], c[2]-a[2] };
    double ap[3] = { p[0]-a[0], p[1]-a[1], p[2]-a[2] };

    double d1 = ab[0]*ap[0] + ab[1]*ap[1] + ab[2]*ap[2];
    double d2 = ac[0]*ap[0] + ac[1]*ap[1] + ac[2]*ap[2];

    if (d1 <= 0.0 && d2 <= 0.0) {
        /* Vertex A region */
        double dx = p[0]-a[0], dy = p[1]-a[1], dz = p[2]-a[2];
        res.sqr_dist = (float)(dx*dx + dy*dy + dz*dz);
        res.closest_pt[0] = (float)a[0];
        res.closest_pt[1] = (float)a[1];
        res.closest_pt[2] = (float)a[2];
        return res;
    }

    double bp[3] = { p[0]-b[0], p[1]-b[1], p[2]-b[2] };
    double d3 = ab[0]*bp[0] + ab[1]*bp[1] + ab[2]*bp[2];
    double d4 = ac[0]*bp[0] + ac[1]*bp[1] + ac[2]*bp[2];

    if (d3 >= 0.0 && d4 <= d3) {
        double dx = p[0]-b[0], dy = p[1]-b[1], dz = p[2]-b[2];
        res.sqr_dist = (float)(dx*dx + dy*dy + dz*dz);
        res.closest_pt[0] = (float)b[0];
        res.closest_pt[1] = (float)b[1];
        res.closest_pt[2] = (float)b[2];
        return res;
    }

    double vc_ = d1*d4 - d3*d2;
    if (vc_ <= 0.0 && d1 >= 0.0 && d3 <= 0.0) {
        double v = d1 / (d1 - d3);
        double q[3] = { a[0]+v*ab[0], a[1]+v*ab[1], a[2]+v*ab[2] };
        double dx = p[0]-q[0], dy = p[1]-q[1], dz = p[2]-q[2];
        res.sqr_dist = (float)(dx*dx + dy*dy + dz*dz);
        res.closest_pt[0] = (float)q[0];
        res.closest_pt[1] = (float)q[1];
        res.closest_pt[2] = (float)q[2];
        return res;
    }

    double cp_[3] = { p[0]-c[0], p[1]-c[1], p[2]-c[2] };
    double d5 = ab[0]*cp_[0] + ab[1]*cp_[1] + ab[2]*cp_[2];
    double d6 = ac[0]*cp_[0] + ac[1]*cp_[1] + ac[2]*cp_[2];

    if (d6 >= 0.0 && d5 <= d6) {
        double dx = p[0]-c[0], dy = p[1]-c[1], dz = p[2]-c[2];
        res.sqr_dist = (float)(dx*dx + dy*dy + dz*dz);
        res.closest_pt[0] = (float)c[0];
        res.closest_pt[1] = (float)c[1];
        res.closest_pt[2] = (float)c[2];
        return res;
    }

    double vb_ = d5*d2 - d1*d6;
    if (vb_ <= 0.0 && d2 >= 0.0 && d6 <= 0.0) {
        double w = d2 / (d2 - d6);
        double q[3] = { a[0]+w*ac[0], a[1]+w*ac[1], a[2]+w*ac[2] };
        double dx = p[0]-q[0], dy = p[1]-q[1], dz = p[2]-q[2];
        res.sqr_dist = (float)(dx*dx + dy*dy + dz*dz);
        res.closest_pt[0] = (float)q[0];
        res.closest_pt[1] = (float)q[1];
        res.closest_pt[2] = (float)q[2];
        return res;
    }

    double va_ = d3*d6 - d5*d4;
    if (va_ <= 0.0 && (d4-d3) >= 0.0 && (d5-d6) >= 0.0) {
        double w = (d4-d3) / ((d4-d3) + (d5-d6));
        double q[3] = { b[0]+w*(c[0]-b[0]), b[1]+w*(c[1]-b[1]), b[2]+w*(c[2]-b[2]) };
        double dx = p[0]-q[0], dy = p[1]-q[1], dz = p[2]-q[2];
        res.sqr_dist = (float)(dx*dx + dy*dy + dz*dz);
        res.closest_pt[0] = (float)q[0];
        res.closest_pt[1] = (float)q[1];
        res.closest_pt[2] = (float)q[2];
        return res;
    }

    /* Interior of triangle */
    double denom = 1.0 / (va_ + vb_ + vc_);
    double v = vb_ * denom;
    double w = vc_ * denom;
    double q[3] = { a[0]+ab[0]*v+ac[0]*w, a[1]+ab[1]*v+ac[1]*w, a[2]+ab[2]*v+ac[2]*w };
    double dx = p[0]-q[0], dy = p[1]-q[1], dz = p[2]-q[2];
    res.sqr_dist = (float)(dx*dx + dy*dy + dz*dz);
    res.closest_pt[0] = (float)q[0];
    res.closest_pt[1] = (float)q[1];
    res.closest_pt[2] = (float)q[2];
    return res;
}

__device__ static struct enc_triangle_result
enc_closest_point_sphere(
    const float3& query,
    const struct sphere_gpu& sp)
{
    struct enc_triangle_result res;
    float dx = query.x - sp.cx;
    float dy = query.y - sp.cy;
    float dz = query.z - sp.cz;
    float dist = sqrtf(dx*dx + dy*dy + dz*dz);

    if (dist < 1e-30f) {
        /* Query at center — project to +X surface point */
        res.closest_pt[0] = sp.cx + sp.radius;
        res.closest_pt[1] = sp.cy;
        res.closest_pt[2] = sp.cz;
        res.sqr_dist = sp.radius * sp.radius;
    } else {
        float inv = sp.radius / dist;
        res.closest_pt[0] = sp.cx + dx * inv;
        res.closest_pt[1] = sp.cy + dy * inv;
        res.closest_pt[2] = sp.cz + dz * inv;
        float d = dist - sp.radius;
        res.sqr_dist = d * d;
    }
    return res;
}

/* Degenerate distance threshold — below this, the side determination
 * is unreliable and we mark side = -1 (degenerate). */
#define ENC_DEGENERATE_THRESHOLD  1.e-6f

/* ------------------------------------------------------------------ */
/*  6-ray side determination (mirrors CPU algorithm)                   */
/*                                                                    */
/*  Instead of dot(P-Q, N) which fails at concave corners, cast up    */
/*  to 6 PI/4-rotated axis-aligned rays through the BVH and use      */
/*  dot(ray_dir, hit_normal) to determine front/back side — exactly   */
/*  matching scene_get_enclosure_id_in_closed_boundaries().           */
/* ------------------------------------------------------------------ */

/* Pre-computed rotation matrix matching CPU f33_rotation(PI/4, PI/4, PI/4).
 * Row-major 3x3.  The CPU rsys f33_rotation(pitch,yaw,roll) stores in
 * column-major order; here we transpose to row-major for standard M*v.
 *
 * CPU column-major frame[9] has:
 *   frame = { c2c3, c1s3+c3s1s2, s1s3-c1c3s2,   (col 0)
 *            -c2s3, c1c3-s1s2s3, c1s2s3+c3s1,    (col 1)
 *             s2,  -c2s1,        c1c2 }           (col 2)
 * where c1=c2=c3=s1=s2=s3 = sqrt(2)/2 for PI/4.
 *
 * Logical matrix M(row,col) = frame[col*3 + row], so row-major is:
 *   Row i = { frame[0*3+i], frame[1*3+i], frame[2*3+i] }
 *
 * Numerical values (verified by test_enc_rot_matrix_consistency):
 *   Row 0: [  0.5,                -0.5,                 sqrt(2)/2      ]
 *   Row 1: [  (1+sqrt(2))/2^1.5,  (2-sqrt(2))/2^1.5,  -0.5           ]
 *   Row 2: [  (2-sqrt(2))/2^1.5,  (1+sqrt(2))/2^1.5,   0.5           ]
 */
__device__ static const float ENC_ROT_MATRIX[9] = {
    /* Row 0: */  0.5f,               -0.5f,                0.70710678118f,
    /* Row 1: */  0.85355339059f,      0.14644660941f,     -0.5f,
    /* Row 2: */  0.14644660941f,      0.85355339059f,      0.5f
};

/* 6 axis-aligned directions: +X, -X, +Y, -Y, +Z, -Z */
__device__ static const float ENC_AXIS_DIRS[6][3] = {
    { 1, 0, 0}, {-1, 0, 0},
    { 0, 1, 0}, { 0,-1, 0},
    { 0, 0, 1}, { 0, 0,-1}
};

/* hit_on_edge: check if a ray hit lies on a triangle edge/vertex.
 * Uses sub-triangle area ratio < ON_EDGE_EPSILON, matching CPU hit_on_edge. */
#define ON_EDGE_EPSILON 1.e-4f

__device__ static bool
enc_hit_on_edge(
    const float3& hit_pos,
    const float3& v0,
    const float3& v1,
    const float3& v2)
{
    /* Triangle area × 2 */
    float3 e0 = v1 - v0;
    float3 e1 = v2 - v0;
    float3 tri_N = cross(e0, e1);
    float tri_2area = sqrtf(tri_N.x*tri_N.x + tri_N.y*tri_N.y + tri_N.z*tri_N.z);
    if (tri_2area < 1e-30f) return true;  /* degenerate triangle */

    /* Sub-triangle areas formed by hit_pos with each edge */
    float3 d0, d1;
    float3 sub_N;
    float sub_area;

    d0 = v0 - hit_pos; d1 = v1 - hit_pos;
    sub_N = cross(d0, d1);
    sub_area = sqrtf(sub_N.x*sub_N.x + sub_N.y*sub_N.y + sub_N.z*sub_N.z);
    if (sub_area / tri_2area < ON_EDGE_EPSILON) return true;

    d0 = v1 - hit_pos; d1 = v2 - hit_pos;
    sub_N = cross(d0, d1);
    sub_area = sqrtf(sub_N.x*sub_N.x + sub_N.y*sub_N.y + sub_N.z*sub_N.z);
    if (sub_area / tri_2area < ON_EDGE_EPSILON) return true;

    d0 = v2 - hit_pos; d1 = v0 - hit_pos;
    sub_N = cross(d0, d1);
    sub_area = sqrtf(sub_N.x*sub_N.x + sub_N.y*sub_N.y + sub_N.z*sub_N.z);
    if (sub_area / tri_2area < ON_EDGE_EPSILON) return true;

    return false;
}

/* Rotate a direction vector by the PI/4 rotation matrix */
__device__ static float3
enc_rotate_dir(const float raw_dir[3])
{
    float3 r;
    r.x = ENC_ROT_MATRIX[0]*raw_dir[0] + ENC_ROT_MATRIX[1]*raw_dir[1] + ENC_ROT_MATRIX[2]*raw_dir[2];
    r.y = ENC_ROT_MATRIX[3]*raw_dir[0] + ENC_ROT_MATRIX[4]*raw_dir[1] + ENC_ROT_MATRIX[5]*raw_dir[2];
    r.z = ENC_ROT_MATRIX[6]*raw_dir[0] + ENC_ROT_MATRIX[7]*raw_dir[1] + ENC_ROT_MATRIX[8]*raw_dir[2];
    return r;
}

/* ------------------------------------------------------------------ */
/*  Kernel: single-level find enclosure                               */
/* ------------------------------------------------------------------ */

__global__ void find_enclosure_kernel(
    cuBQL::BinaryBVH<float, 3>               bvh,
    const float3* __restrict__               vertices,
    const uint3*  __restrict__               indices,
    const struct sphere_gpu* __restrict__    spheres,
    const unsigned int* __restrict__         prim_to_geom,
    const struct geom_gpu_entry* __restrict__ geom_entries,
    uint32_t                                 tri_count,
    const float3* __restrict__               query_positions,
    uint32_t                                 num_queries,
    struct cus3d_enc_result*                 results)
{
    uint32_t tid = blockIdx.x * blockDim.x + threadIdx.x;
    if (tid >= num_queries) return;

    float3 pos = query_positions[tid];

    struct cus3d_enc_result hit;
    hit.prim_id  = -1;
    hit.geom_idx = -1;
    hit.inst_id  = -1;
    hit.distance = FLT_MAX;
    hit.normal[0] = 0.0f; hit.normal[1] = 0.0f; hit.normal[2] = 0.0f;
    hit.closest_pos[0] = 0.0f;
    hit.closest_pos[1] = 0.0f;
    hit.closest_pos[2] = 0.0f;
    hit.side = -1;

    float best_sqr_dist = FLT_MAX;

    auto candidate = [&](uint32_t primID) -> float {
        uint32_t geom_idx = prim_to_geom[primID];
        struct geom_gpu_entry ge = geom_entries[geom_idx];

        if (!ge.is_enabled)
            return best_sqr_dist;

        if (primID < tri_count) {
            /* Triangle */
            uint32_t local_id = primID - ge.prim_offset;
            uint3 tri_idx = indices[ge.prim_offset + local_id];
            float3 v0 = vertices[tri_idx.x];
            float3 v1 = vertices[tri_idx.y];
            float3 v2 = vertices[tri_idx.z];

            struct enc_triangle_result tr = enc_closest_point_triangle(pos, v0, v1, v2);

            if (tr.sqr_dist < best_sqr_dist) {
                best_sqr_dist = tr.sqr_dist;
                float dst = sqrtf(tr.sqr_dist);

                /* Geometric normal (CW convention: cross(E1, E0)) */
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
            }
        } else {
            /* Sphere */
            uint32_t sphere_idx = primID - tri_count;
            struct sphere_gpu sp = spheres[sphere_idx];

            struct enc_triangle_result sr = enc_closest_point_sphere(pos, sp);

            if (sr.sqr_dist < best_sqr_dist) {
                best_sqr_dist = sr.sqr_dist;
                float dst = sqrtf(sr.sqr_dist);

                /* Sphere normal: from center to closest point */
                float nx = sr.closest_pt[0] - sp.cx;
                float ny = sr.closest_pt[1] - sp.cy;
                float nz = sr.closest_pt[2] - sp.cz;
                if (ge.flip_surface) {
                    nx = -nx; ny = -ny; nz = -nz;
                }

                hit.prim_id  = (int32_t)primID;
                hit.geom_idx = (int32_t)geom_idx;
                hit.distance = dst;
                hit.closest_pos[0] = sr.closest_pt[0];
                hit.closest_pos[1] = sr.closest_pt[1];
                hit.closest_pos[2] = sr.closest_pt[2];
                hit.normal[0] = nx;
                hit.normal[1] = ny;
                hit.normal[2] = nz;
            }
        }
        return best_sqr_dist;
    };

    cuBQL::shrinkingRadiusQuery::forEachPrim(
        candidate,
        bvh,
        cuBQL::vec3f(pos.x, pos.y, pos.z),
        best_sqr_dist);

    /* ---- 6-ray side determination (replaces dot(P-Q, N)) ---- */
    if (hit.prim_id >= 0 && hit.distance >= ENC_DEGENERATE_THRESHOLD) {
        int resolved_side = -1;
        int resolved_prim = -1;

        for (int idir = 0; idir < 6 && resolved_side < 0; ++idir) {
            float3 dir = enc_rotate_dir(ENC_AXIS_DIRS[idir]);

            cuBQL::ray3f ray(
                cuBQL::vec3f(pos.x, pos.y, pos.z),
                cuBQL::vec3f(dir.x, dir.y, dir.z),
                FLT_MIN, FLT_MAX);

            int    ray_hit_prim = -1;
            int    ray_hit_geom = -1;
            float  ray_hit_t    = FLT_MAX;
            float3 ray_hit_N    = make_float3(0,0,0);
            float  ray_hit_u    = 0, ray_hit_v = 0;

            auto ray_intersect = [&](uint32_t primID) -> float {
                uint32_t gidx = prim_to_geom[primID];
                struct geom_gpu_entry ge = geom_entries[gidx];
                if (!ge.is_enabled) return ray.tMax;

                if (primID < tri_count) {
                    uint32_t lid = primID - ge.prim_offset;
                    uint3 ti = indices[ge.prim_offset + lid];
                    float3 va = vertices[ti.x];
                    float3 vb = vertices[ti.y];
                    float3 vc = vertices[ti.z];
                    float t, u, v;
                    if (ray_triangle_intersect(pos, dir, va, vb, vc,
                                              FLT_MIN, ray.tMax, &t, &u, &v)) {
                        float3 e1 = vb - va;
                        float3 e2 = vc - va;
                        float3 N = cross(e1, e2);
                        if (ge.flip_surface) { N.x=-N.x; N.y=-N.y; N.z=-N.z; }
                        ray_hit_prim = (int)primID;
                        ray_hit_geom = (int)gidx;
                        ray_hit_t    = t;
                        ray_hit_N    = N;
                        ray_hit_u    = u;
                        ray_hit_v    = v;
                        ray.tMax     = t;
                    }
                } else {
                    uint32_t si = primID - tri_count;
                    struct sphere_gpu sp = spheres[si];
                    float t; float3 N;
                    if (ray_sphere_intersect(pos, dir, sp.cx, sp.cy, sp.cz,
                                            sp.radius, FLT_MIN, ray.tMax, &t, &N)) {
                        if (ge.flip_surface) { N.x=-N.x; N.y=-N.y; N.z=-N.z; }
                        ray_hit_prim = (int)primID;
                        ray_hit_geom = (int)gidx;
                        ray_hit_t    = t;
                        ray_hit_N    = N;
                        ray.tMax     = t;
                    }
                }
                return ray.tMax;
            };

            cuBQL::shrinkingRayQuery::forEachPrim(ray_intersect, bvh, ray);

            if (ray_hit_prim < 0) continue;       /* miss */
            if (ray_hit_t <= 1.e-6f) continue;     /* too close */

            /* Edge detection for triangles */
            if ((uint32_t)ray_hit_prim < tri_count) {
                uint32_t gidx = prim_to_geom[(uint32_t)ray_hit_prim];
                struct geom_gpu_entry ge = geom_entries[gidx];
                uint32_t lid = (uint32_t)ray_hit_prim - ge.prim_offset;
                uint3 ti = indices[ge.prim_offset + lid];
                float3 va = vertices[ti.x];
                float3 vb = vertices[ti.y];
                float3 vc = vertices[ti.z];
                float3 hp = make_float3(pos.x + dir.x*ray_hit_t,
                                        pos.y + dir.y*ray_hit_t,
                                        pos.z + dir.z*ray_hit_t);
                if (enc_hit_on_edge(hp, va, vb, vc)) continue;
            }

            /* cos(N, dir) check */
            float Nlen = sqrtf(ray_hit_N.x*ray_hit_N.x + ray_hit_N.y*ray_hit_N.y
                             + ray_hit_N.z*ray_hit_N.z);
            if (Nlen < 1e-30f) continue;
            float cos_N_dir = (ray_hit_N.x*dir.x + ray_hit_N.y*dir.y
                             + ray_hit_N.z*dir.z) / Nlen;
            if (fabsf(cos_N_dir) <= 1.e-2f) continue;  /* grazing */

            resolved_side = (cos_N_dir < 0.0f) ? 0 : 1;
            resolved_prim = ray_hit_prim;
        }

        if (resolved_side >= 0) {
            hit.side    = resolved_side;
            hit.prim_id = resolved_prim;
        }
        /* else: all 6 rays failed → side stays -1 (degenerate) */
    }

    results[tid] = hit;
}

/* ------------------------------------------------------------------ */
/*  Kernel: two-level find enclosure (TLAS + BLAS)                    */
/* ------------------------------------------------------------------ */

__global__ void find_enclosure_instanced_kernel(
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
    uint32_t                                 num_queries,
    struct cus3d_enc_result*                 results)
{
    uint32_t tid = blockIdx.x * blockDim.x + threadIdx.x;
    if (tid >= num_queries) return;

    float3 pos_ws = query_positions[tid];

    struct cus3d_enc_result hit;
    hit.prim_id  = -1;
    hit.geom_idx = -1;
    hit.inst_id  = -1;
    hit.distance = FLT_MAX;
    hit.normal[0] = 0.0f; hit.normal[1] = 0.0f; hit.normal[2] = 0.0f;
    hit.closest_pos[0] = 0.0f;
    hit.closest_pos[1] = 0.0f;
    hit.closest_pos[2] = 0.0f;
    hit.side = -1;

    float best_sqr_dist = FLT_MAX;

    auto tlas_candidate = [&](uint32_t instID) -> float {
        if (instID >= num_instances)
            return best_sqr_dist;

        struct instance_gpu_data inst = instances[instID];

        /* Transform query to instance local space */
        const float* T = inst.inv_transform;
        float lx = T[0]*pos_ws.x + T[3]*pos_ws.y + T[6]*pos_ws.z + T[9];
        float ly = T[1]*pos_ws.x + T[4]*pos_ws.y + T[7]*pos_ws.z + T[10];
        float lz = T[2]*pos_ws.x + T[5]*pos_ws.y + T[8]*pos_ws.z + T[11];
        float3 pos_ls = make_float3(lx, ly, lz);

        /* Similarity scale for distance conversion */
        const float* Tf = inst.transform;
        float det = Tf[0]*(Tf[4]*Tf[8] - Tf[5]*Tf[7])
                  - Tf[3]*(Tf[1]*Tf[8] - Tf[2]*Tf[7])
                  + Tf[6]*(Tf[1]*Tf[5] - Tf[2]*Tf[4]);
        float sim_scale = cbrtf(fabsf(det));
        if (sim_scale < 1e-30f) return best_sqr_dist;

        float local_radius_sq = best_sqr_dist / (sim_scale * sim_scale);

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

                struct enc_triangle_result tr = enc_closest_point_triangle(pos_ls, v0, v1, v2);

                if (tr.sqr_dist < local_radius_sq) {
                    float ws_sqr_dist = tr.sqr_dist * sim_scale * sim_scale;
                    if (ws_sqr_dist < best_sqr_dist) {
                        best_sqr_dist = ws_sqr_dist;
                        local_radius_sq = tr.sqr_dist;
                        float dst = sqrtf(ws_sqr_dist);

                        float3 e0 = v1 - v0;
                        float3 e1 = v2 - v0;
                        float3 N = cross(e1, e0);
                        if (ge.flip_surface) {
                            N.x = -N.x; N.y = -N.y; N.z = -N.z;
                        }
                        /* Transform normal to world space */
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
                    }
                }
            } else {
                uint32_t sphere_idx = primID - cur_tri_count;
                struct sphere_gpu sp = cur_sph[sphere_idx];

                struct enc_triangle_result sr = enc_closest_point_sphere(pos_ls, sp);

                if (sr.sqr_dist < local_radius_sq) {
                    float ws_sqr_dist = sr.sqr_dist * sim_scale * sim_scale;
                    if (ws_sqr_dist < best_sqr_dist) {
                        best_sqr_dist = ws_sqr_dist;
                        local_radius_sq = sr.sqr_dist;
                        float dst = sqrtf(ws_sqr_dist);

                        float nx = sr.closest_pt[0] - sp.cx;
                        float ny = sr.closest_pt[1] - sp.cy;
                        float nz = sr.closest_pt[2] - sp.cz;
                        if (ge.flip_surface) {
                            nx = -nx; ny = -ny; nz = -nz;
                        }
                        float3 wN;
                        wN.x = Tf[0]*nx + Tf[3]*ny + Tf[6]*nz;
                        wN.y = Tf[1]*nx + Tf[4]*ny + Tf[7]*nz;
                        wN.z = Tf[2]*nx + Tf[5]*ny + Tf[8]*nz;

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

    cuBQL::shrinkingRadiusQuery::forEachPrim(
        tlas_candidate,
        tlas,
        cuBQL::vec3f(pos_ws.x, pos_ws.y, pos_ws.z),
        best_sqr_dist);

    /* ---- 6-ray side determination (instanced, replaces dot(P-Q, N)) ---- */
    if (hit.prim_id >= 0 && hit.distance >= ENC_DEGENERATE_THRESHOLD) {
        int resolved_side = -1;
        int resolved_prim = -1;
        int resolved_inst = -1;
        int resolved_geom = -1;

        for (int idir = 0; idir < 6 && resolved_side < 0; ++idir) {
            float3 dir_ws = enc_rotate_dir(ENC_AXIS_DIRS[idir]);

            /* Ray-based side determination via two-level BVH traversal.
             * We inline the TLAS/BLAS traversal here because cuBQL
             * twoLevel::forEachPrim needs enterBlas/leaveBlas lambdas
             * that are too complex for the generic wrapper. */
            int    ray_best_prim = -1;
            int    ray_best_geom = -1;
            int    ray_best_inst = -1;
            float  ray_best_t    = FLT_MAX;
            float3 ray_best_N    = make_float3(0,0,0);

            /* Iterate over instances via TLAS shrinkingRadiusQuery as a
             * conservative bounding volume test, then do ray traversal
             * against each candidate instance's BLAS. */
            for (uint32_t instID = 0; instID < num_instances; ++instID) {
                struct instance_gpu_data inst = instances[instID];

                /* Transform ray to instance local space */
                const float* Ti = inst.inv_transform;
                float lox = Ti[0]*pos_ws.x + Ti[3]*pos_ws.y + Ti[6]*pos_ws.z + Ti[9];
                float loy = Ti[1]*pos_ws.x + Ti[4]*pos_ws.y + Ti[7]*pos_ws.z + Ti[10];
                float loz = Ti[2]*pos_ws.x + Ti[5]*pos_ws.y + Ti[8]*pos_ws.z + Ti[11];
                float ldx = Ti[0]*dir_ws.x + Ti[3]*dir_ws.y + Ti[6]*dir_ws.z;
                float ldy = Ti[1]*dir_ws.x + Ti[4]*dir_ws.y + Ti[7]*dir_ws.z;
                float ldz = Ti[2]*dir_ws.x + Ti[5]*dir_ws.y + Ti[8]*dir_ws.z;

                /* Similarity scale for t conversion */
                float dir_ls_len = sqrtf(ldx*ldx + ldy*ldy + ldz*ldz);
                if (dir_ls_len < 1e-30f) continue;
                float inv_dir_ls_len = 1.0f / dir_ls_len;
                /* Normalize local direction */
                ldx *= inv_dir_ls_len;
                ldy *= inv_dir_ls_len;
                ldz *= inv_dir_ls_len;

                float3 lo = make_float3(lox, loy, loz);
                float3 ld = make_float3(ldx, ldy, ldz);

                /* local tMax = world tMax × dir_ls_len (shorter if instance is scaled up) */
                float local_tmax = ray_best_t * dir_ls_len;

                cuBQL::ray3f local_ray(
                    cuBQL::vec3f(lo.x, lo.y, lo.z),
                    cuBQL::vec3f(ld.x, ld.y, ld.z),
                    FLT_MIN, local_tmax);

                cuBQL::BinaryBVH<float, 3> child_bvh = blas_array[inst.child_bvh_idx];
                const float3* cv = (const float3*)inst.inst_vertices;
                const uint3*  ci = (const uint3*)inst.inst_indices;
                const struct sphere_gpu* cs = (const struct sphere_gpu*)inst.inst_spheres;
                const unsigned int*      cp = (const unsigned int*)inst.inst_prim_to_geom;
                const struct geom_gpu_entry* cg = (const struct geom_gpu_entry*)inst.inst_geom_entries;
                uint32_t ctri = inst.inst_tri_count;

                int    local_hit_prim = -1;
                int    local_hit_geom = -1;
                float  local_hit_t    = local_tmax;
                float3 local_hit_N    = make_float3(0,0,0);

                auto local_intersect = [&](uint32_t primID) -> float {
                    uint32_t gidx = cp[primID];
                    struct geom_gpu_entry ge = cg[gidx];
                    if (!ge.is_enabled) return local_ray.tMax;

                    if (primID < ctri) {
                        uint32_t lid = primID - ge.prim_offset;
                        uint3 tidx = ci[ge.prim_offset + lid];
                        float3 va = cv[tidx.x];
                        float3 vb = cv[tidx.y];
                        float3 vc = cv[tidx.z];
                        float t, u, v;
                        if (ray_triangle_intersect(lo, ld, va, vb, vc,
                                                  FLT_MIN, local_ray.tMax, &t, &u, &v)) {
                            float3 e1 = vb - va;
                            float3 e2 = vc - va;
                            float3 N = cross(e1, e2);
                            if (ge.flip_surface) { N.x=-N.x; N.y=-N.y; N.z=-N.z; }
                            local_hit_prim = (int)primID;
                            local_hit_geom = (int)gidx;
                            local_hit_t    = t;
                            local_hit_N    = N;
                            local_ray.tMax = t;
                        }
                    } else {
                        uint32_t si = primID - ctri;
                        struct sphere_gpu sp = cs[si];
                        float t; float3 N;
                        if (ray_sphere_intersect(lo, ld, sp.cx, sp.cy, sp.cz,
                                                sp.radius, FLT_MIN, local_ray.tMax, &t, &N)) {
                            if (ge.flip_surface) { N.x=-N.x; N.y=-N.y; N.z=-N.z; }
                            local_hit_prim = (int)primID;
                            local_hit_geom = (int)gidx;
                            local_hit_t    = t;
                            local_hit_N    = N;
                            local_ray.tMax = t;
                        }
                    }
                    return local_ray.tMax;
                };

                cuBQL::shrinkingRayQuery::forEachPrim(local_intersect, child_bvh, local_ray);

                if (local_hit_prim < 0) continue;

                /* Convert local t to world t */
                float ws_t = local_hit_t * inv_dir_ls_len;
                if (ws_t >= ray_best_t) continue;
                if (ws_t <= 1.e-6f) continue;

                /* Transform normal to world space */
                const float* Tf = inst.transform;
                float3 wN;
                wN.x = Tf[0]*local_hit_N.x + Tf[3]*local_hit_N.y + Tf[6]*local_hit_N.z;
                wN.y = Tf[1]*local_hit_N.x + Tf[4]*local_hit_N.y + Tf[7]*local_hit_N.z;
                wN.z = Tf[2]*local_hit_N.x + Tf[5]*local_hit_N.y + Tf[8]*local_hit_N.z;

                /* Edge detection for triangles */
                if ((uint32_t)local_hit_prim < ctri) {
                    struct geom_gpu_entry ge = cg[cp[(uint32_t)local_hit_prim]];
                    uint32_t lid = (uint32_t)local_hit_prim - ge.prim_offset;
                    uint3 tidx = ci[ge.prim_offset + lid];
                    float3 va = cv[tidx.x];
                    float3 vb = cv[tidx.y];
                    float3 vc = cv[tidx.z];
                    float3 hp = make_float3(lo.x + ld.x*local_hit_t,
                                            lo.y + ld.y*local_hit_t,
                                            lo.z + ld.z*local_hit_t);
                    if (enc_hit_on_edge(hp, va, vb, vc)) continue;
                }

                ray_best_t    = ws_t;
                ray_best_prim = local_hit_prim;
                ray_best_geom = local_hit_geom;
                ray_best_inst = (int)instID;
                ray_best_N    = wN;
            } /* end instance loop */

            if (ray_best_prim < 0) continue;  /* miss on all instances */

            /* cos(N, dir) check */
            float Nlen = sqrtf(ray_best_N.x*ray_best_N.x + ray_best_N.y*ray_best_N.y
                             + ray_best_N.z*ray_best_N.z);
            if (Nlen < 1e-30f) continue;
            float cos_N_dir = (ray_best_N.x*dir_ws.x + ray_best_N.y*dir_ws.y
                             + ray_best_N.z*dir_ws.z) / Nlen;
            if (fabsf(cos_N_dir) <= 1.e-2f) continue;  /* grazing */

            resolved_side = (cos_N_dir < 0.0f) ? 0 : 1;
            resolved_prim = ray_best_prim;
            resolved_inst = ray_best_inst;
            resolved_geom = ray_best_geom;
        }

        if (resolved_side >= 0) {
            hit.side    = resolved_side;
            hit.prim_id = resolved_prim;
            hit.inst_id = resolved_inst;
            hit.geom_idx = resolved_geom;
        }
        /* else: all 6 rays failed → side stays -1 (degenerate) */
    }

    results[tid] = hit;
}

/* ================================================================== */
/*  Host-side API                                                     */
/* ================================================================== */

res_T
cus3d_enc_batch_create(struct cus3d_enc_batch* batch, size_t max_queries)
{
    enc_clear_error();

    if (!batch) {
        enc_set_error("cus3d_enc_batch_create: batch is NULL");
        return RES_ERR;
    }

    memset(batch, 0, sizeof(*batch));

    if (max_queries == 0)
        return RES_OK;

    cudaStream_t s = (cudaStream_t)0;
    res_T res;

    res = gpu_buffer_float3_alloc(&batch->d_positions, max_queries, s);
    if (res != RES_OK) return res;

    batch->count = 0;
    return RES_OK;
}

void
cus3d_enc_batch_destroy(struct cus3d_enc_batch* batch)
{
    if (!batch) return;

    cudaStream_t s = (cudaStream_t)0;
    gpu_buffer_float3_free(&batch->d_positions, s);
    batch->count = 0;
}

res_T
cus3d_find_enclosure_batch(
    const struct cus3d_bvh* bvh,
    const struct cus3d_geom_store* store,
    struct cus3d_device* dev,
    const struct cus3d_enc_batch* queries,
    struct cus3d_enc_result* h_results)
{
    enc_clear_error();

    if (!bvh || !store || !dev || !queries || !h_results) {
        enc_set_error("cus3d_find_enclosure_batch: NULL argument");
        return RES_ERR;
    }

    if (queries->count == 0)
        return RES_OK;

    if (!bvh->valid && !(bvh->tlas_valid && bvh->instance_count > 0)) {
        enc_set_error("cus3d_find_enclosure_batch: BVH not built");
        return RES_ERR;
    }

    cudaStream_t s = dev->stream;
    uint32_t num_queries = (uint32_t)queries->count;

    /* Allocate device results */
    struct cus3d_enc_result* d_results = NULL;
    ENC_CUDA_CHECK(
        cudaMallocAsync(&d_results,
                        num_queries * sizeof(struct cus3d_enc_result), s),
        "alloc d_results");

    const uint32_t block_size = 256;
    uint32_t grid_size = (num_queries + block_size - 1) / block_size;

    if (bvh->tlas_valid && bvh->tlas_count > 0) {
        /* Two-level (instanced) */
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
            enc_set_error("cus3d_find_enclosure_batch: host malloc failed");
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
        ENC_CUDA_CHECK(
            cudaMallocAsync(&d_blas_array,
                            n_inst * sizeof(cuBQL::BinaryBVH<float, 3>), s),
            "alloc d_blas_array");
        ENC_CUDA_CHECK(
            cudaMemcpyAsync(d_blas_array, h_blas_array,
                            n_inst * sizeof(cuBQL::BinaryBVH<float, 3>),
                            cudaMemcpyHostToDevice, s),
            "upload blas_array");

        struct instance_gpu_data* d_instances = NULL;
        ENC_CUDA_CHECK(
            cudaMallocAsync(&d_instances,
                            n_inst * sizeof(struct instance_gpu_data), s),
            "alloc d_instances");
        ENC_CUDA_CHECK(
            cudaMemcpyAsync(d_instances, h_instances,
                            n_inst * sizeof(struct instance_gpu_data),
                            cudaMemcpyHostToDevice, s),
            "upload instances");

        free(h_blas_array);
        free(h_instances);

        find_enclosure_instanced_kernel<<<grid_size, block_size, 0, s>>>(
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
            num_queries,
            d_results);

        ENC_CUDA_CHECK(cudaGetLastError(), "launch find_enclosure_instanced_kernel");
        ENC_CUDA_CHECK(cudaStreamSynchronize(s), "sync after instanced enc");

        cudaFreeAsync(d_blas_array, s);
        cudaFreeAsync(d_instances, s);

    } else {
        /* Single-level (direct geometry) */
        find_enclosure_kernel<<<grid_size, block_size, 0, s>>>(
            bvh->bvh,
            store->d_vertices.data,
            store->d_indices.data,
            store->d_spheres,
            store->d_prim_to_geom.data,
            store->d_geom_entries,
            store->total_tris,
            queries->d_positions.data,
            num_queries,
            d_results);

        ENC_CUDA_CHECK(cudaGetLastError(), "launch find_enclosure_kernel");
        ENC_CUDA_CHECK(cudaStreamSynchronize(s), "sync after enc");
    }

    /* Download results */
    ENC_CUDA_CHECK(
        cudaMemcpyAsync(h_results, d_results,
                        num_queries * sizeof(struct cus3d_enc_result),
                        cudaMemcpyDeviceToHost, s),
        "download enc results");
    ENC_CUDA_CHECK(cudaStreamSynchronize(s), "sync after result download");

    cudaFreeAsync(d_results, s);

    return RES_OK;
}
