#include "cus3d_math.cuh"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
/*******************************************************************************
 * helper functions
 ******************************************************************************/
 __device__ __forceinline__
int maxAbsAxis(const float3& v)
{
    float ax = fabsf(v.x);
    float ay = fabsf(v.y);
    float az = fabsf(v.z);

    return (ax > ay)
        ? ((ax > az) ? 0 : 2)
        : ((ay > az) ? 1 : 2);
}

__device__ __forceinline__
float getComponent(const float3& v, int idx)
{
    return (idx == 0) ? v.x : (idx == 1) ? v.y : v.z;
}

/*******************************************************************************
 * Ray-Triangle intersection (Möller-Trumbore algorithm)
 *
 * Returns barycentric coordinates (u, v) for edges e1=(v1-v0) and e2=(v2-v0).
 * The third barycentric coordinate is w = 1 - u - v.
 * The hit point = v0 + u*e1 + v*e2 = (1-u-v)*v0 + u*v1 + v*v2.
 ******************************************************************************/
__device__ bool ray_triangle_intersectMT(
    const float3& org, const float3& dir,
    const float3& v0, const float3& v1, const float3& v2,
    float tmin, float tmax,
    float* t_out, float* u_out, float* v_out)
{
    float3 e1 = v1 - v0;
    float3 e2 = v2 - v0;

    float3 h = cross(dir, e2);
    float a = dot(e1, h);

    /* Ray parallel to triangle plane */
    if (a > -1e-12f && a < 1e-12f)
        return false;

    float f = 1.0f / a;
    float3 s = org - v0;
    float u = f * dot(s, h);
    if (u < 0.0f || u > 1.0f)
        return false;

    float3 q = cross(s, e1);
    float v = f * dot(dir, q);
    if (v < 0.0f || u + v > 1.0f)
        return false;

    float t = f * dot(e2, q);
    if (t <= tmin || t >= tmax)
        return false;

    *t_out = t;
    *u_out = u;
    *v_out = v;
    return true;
}

/*******************************************************************************
 * Ray-Triangle intersection (Möller-Trumbore algorithm)
 *
 * Returns barycentric coordinates (u, v) for edges e1=(v1-v0) and e2=(v2-v0).
 * The third barycentric coordinate is w = 1 - u - v.
 * The hit point = v0 + u*e1 + v*e2 = (1-u-v)*v0 + u*v1 + v*v2.
 ******************************************************************************/
__device__ bool ray_triangle_intersectWBW(
    const float3& org, const float3& dir,
    const float3& v0, const float3& v1, const float3& v2,
    float tmin, float tmax,
    float* t_out, float* u_out, float* v_out)
{
    // ---- 1. Choose dominant axis ----
    int kz = maxAbsAxis(dir);
    int kx = (kz + 1) % 3;
    int ky = (kx + 1) % 3;

    float dirKz = getComponent(dir, kz);

    constexpr float EPS_DIR = 1e-8f;

    // Preserve winding: swap kx/ky when dominant axis is negative
    if (dirKz < 0.0f) {
        int tmp = kx; kx = ky; ky = tmp;
    }

    // ---- 2. Near-parallel reject ----
    if (fabsf(dirKz) < EPS_DIR)
        return false;

    // ---- 3. Shear constants ----
    float Sz = 1.0f / dirKz;
    float Sx = getComponent(dir, kx) * Sz;
    float Sy = getComponent(dir, ky) * Sz;

    // ---- 4. Translate vertices ----
    float3 A = make_float3(v0.x - org.x, v0.y - org.y, v0.z - org.z);
    float3 B = make_float3(v1.x - org.x, v1.y - org.y, v1.z - org.z);
    float3 C = make_float3(v2.x - org.x, v2.y - org.y, v2.z - org.z);

    // ---- 5. Shear & project to 2D ----
    float Ax = getComponent(A, kx) - Sx * getComponent(A, kz);
    float Ay = getComponent(A, ky) - Sy * getComponent(A, kz);
    float Bx = getComponent(B, kx) - Sx * getComponent(B, kz);
    float By = getComponent(B, ky) - Sy * getComponent(B, kz);
    float Cx = getComponent(C, kx) - Sx * getComponent(C, kz);
    float Cy = getComponent(C, ky) - Sy * getComponent(C, kz);

    // ---- 6. Edge functions (scaled barycentrics) ----
    float U = Cx * By - Cy * Bx;
    float V = Ax * Cy - Ay * Cx;
    float W = Bx * Ay - By * Ax;

    // ---- 7. Inside test (watertight, half-open) ----
    if ((U < 0.0f || V < 0.0f || W < 0.0f) &&
        (U > 0.0f || V > 0.0f || W > 0.0f))
        return false;

    float det = U + V + W;

    // ---- 8. Degenerate triangle ----
    constexpr float EPS_DET = 1e-8f;
    if (fabsf(det) < EPS_DET)
        return false;

    // ---- 9. Compute hit distance ----
    float Az = getComponent(A, kz) * Sz;
    float Bz = getComponent(B, kz) * Sz;
    float Cz = getComponent(C, kz) * Sz;

    float T = U * Az + V * Bz + W * Cz;
    float invDet = 1.0f / det;

    float tHit = T * invDet;
    if (tHit <= tmin || tHit >= tmax)
        return false;

    // ---- 10. Output ----
    *t_out = tHit;
    // *u_out = U * invDet;
    // *v_out = V * invDet;
    *u_out = V * invDet;   // λ₁ = v1 的权重
    *v_out = W * invDet;   // λ₂ = v2 的权重

    return true;
}

/******************************************************************************* 
 * Ray-Triangle intersection transmitter
 ******************************************************************************/

__device__ bool ray_triangle_intersect(
    const float3& org, const float3& dir,
    const float3& v0, const float3& v1, const float3& v2,
    float tmin, float tmax,
    float* t_out, float* u_out, float* v_out){
    return ray_triangle_intersectWBW(org, dir, v0, v1, v2, tmin, tmax, t_out, u_out, v_out);
}

/*******************************************************************************
 * Ray-Sphere intersection (analytic quadratic equation)
 *
 * Solves |org + t*dir - center|^2 = radius^2 for t.
 * Returns the nearest valid t in (tmin, tmax) and the outward unit normal.
 ******************************************************************************/
__device__ bool ray_sphere_intersect(
    const float3& org, const float3& dir,
    float cx, float cy, float cz, float radius,
    float tmin, float tmax,
    float* t_out, float3* normal_out)
{
    float3 center = make_float3(cx, cy, cz);
    float3 oc = org - center;

    float a = dot(dir, dir);
    float b = 2.0f * dot(oc, dir);
    float c = dot(oc, oc) - radius * radius;
    float discriminant = b * b - 4.0f * a * c;

    if (discriminant < 0.0f)
        return false;

    float sqrtDisc = sqrtf(discriminant);
    float inv2a = 0.5f / a;
    float t0 = (-b - sqrtDisc) * inv2a;   /* near intersection */
    float t1 = (-b + sqrtDisc) * inv2a;   /* far  intersection */

    /* Try the nearer root first */
    if (t0 > tmin && t0 < tmax) {
        *t_out = t0;
        float3 hitPoint = org + dir * t0;
        float3 N = hitPoint - center;
        float len = length(N);
        if (len > 1e-8f)
            N = N * (1.0f / len);
        *normal_out = N;
        return true;
    }
    /* Fall back to the farther root (ray origin inside sphere) */
    if (t1 > tmin && t1 < tmax) {
        *t_out = t1;
        float3 hitPoint = org + dir * t1;
        float3 N = hitPoint - center;
        float len = length(N);
        if (len > 1e-8f)
            N = N * (1.0f / len);
        *normal_out = N;
        return true;
    }
    return false;
}

/*******************************************************************************
 * Sphere normal → UV mapping  (matches star-3d CPU convention)
 *
 *   v = (1 - N.z) / 2              θ mapping, v ∈ [0,1]
 *   u = atan2(N.y, N.x) / (2π)     φ mapping, u ∈ [0,1]
 *
 * Poles (N.z == ±1) → u = 0 to avoid atan2 instability.
 ******************************************************************************/
__device__ float2 sphere_normal_to_uv(const float3& N) {
    float u, v;
    float cos_theta = N.z;

    v = (1.0f - cos_theta) * 0.5f;

    if (fabsf(cos_theta) >= 1.0f) {
        u = 0.0f;
    } else if (fabsf(N.x) < 1.e-6f) {
        u = (N.y > 0.0f) ? 0.25f : 0.75f;
    } else {
        float phi = atan2f(N.y, N.x);
        if (phi < 0.0f)
            phi += 2.0f * (float)M_PI;
        u = phi / (2.0f * (float)M_PI);
    }
    return make_float2(u, v);
}

/*******************************************************************************
 * 3×4 column-major affine transforms
 *
 * Layout: [col0(3) | col1(3) | col2(3) | translation(3)]
 *   M[0],M[1],M[2]   = column 0  (m00, m10, m20)
 *   M[3],M[4],M[5]   = column 1  (m01, m11, m21)
 *   M[6],M[7],M[8]   = column 2  (m02, m12, m22)
 *   M[9],M[10],M[11]  = translation (tx, ty, tz)
 *
 * result = M_3x3 * point + T      (transform_point)
 * result = M_3x3 * vec            (transform_vector, no translation)
 * result = M_invT * n             (transform_normal, pre-computed inv-transpose)
 ******************************************************************************/
__device__ float3 transform_point(const float transform[12], const float3& p) {
    return make_float3(
        transform[0] * p.x + transform[3] * p.y + transform[6] * p.z + transform[9],
        transform[1] * p.x + transform[4] * p.y + transform[7] * p.z + transform[10],
        transform[2] * p.x + transform[5] * p.y + transform[8] * p.z + transform[11]
    );
}

__device__ float3 transform_vector(const float transform[12], const float3& v) {
    return make_float3(
        transform[0] * v.x + transform[3] * v.y + transform[6] * v.z,
        transform[1] * v.x + transform[4] * v.y + transform[7] * v.z,
        transform[2] * v.x + transform[5] * v.y + transform[8] * v.z
    );
}

__device__ float3 transform_normal(const float inv_transpose[12], const float3& n) {
    return make_float3(
        inv_transpose[0] * n.x + inv_transpose[3] * n.y + inv_transpose[6] * n.z,
        inv_transpose[1] * n.x + inv_transpose[4] * n.y + inv_transpose[7] * n.z,
        inv_transpose[2] * n.x + inv_transpose[5] * n.y + inv_transpose[8] * n.z
    );
}
