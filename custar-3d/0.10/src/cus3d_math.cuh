/**
 * @file cus3d_math.cuh
 * @brief Device math helpers for custar-3d CUDA kernels
 * Ray-primitive intersection, transforms, vector operations.
 */

#ifndef CUS3D_MATH_CUH
#define CUS3D_MATH_CUH

#include <cuda_runtime.h>

__device__ inline float3 operator+(const float3& a, const float3& b) {
    return make_float3(a.x + b.x, a.y + b.y, a.z + b.z);
}

__device__ inline float3 operator-(const float3& a, const float3& b) {
    return make_float3(a.x - b.x, a.y - b.y, a.z - b.z);
}

__device__ inline float3 operator*(const float3& a, float s) {
    return make_float3(a.x * s, a.y * s, a.z * s);
}

__device__ inline float3 operator*(float s, const float3& a) {
    return make_float3(a.x * s, a.y * s, a.z * s);
}

__device__ inline float dot(const float3& a, const float3& b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

__device__ inline float3 cross(const float3& a, const float3& b) {
    return make_float3(
        a.y * b.z - a.z * b.y,
        a.z * b.x - a.x * b.z,
        a.x * b.y - a.y * b.x
    );
}

__device__ inline float length(const float3& v) {
    return sqrtf(dot(v, v));
}

__device__ inline float3 normalize(const float3& v) {
    float len = length(v);
    return make_float3(v.x / len, v.y / len, v.z / len);
}

__device__ inline float3 fminf(const float3& a, const float3& b) {
    return make_float3(fminf(a.x, b.x), fminf(a.y, b.y), fminf(a.z, b.z));
}

__device__ inline float3 fmaxf(const float3& a, const float3& b) {
    return make_float3(fmaxf(a.x, b.x), fmaxf(a.y, b.y), fmaxf(a.z, b.z));
}

__device__ bool ray_triangle_intersect(
    const float3& org, const float3& dir,
    const float3& v0, const float3& v1, const float3& v2,
    float tmin, float tmax,
    float* t_out, float* u_out, float* v_out);

__device__ bool ray_sphere_intersect(
    const float3& org, const float3& dir,
    float cx, float cy, float cz, float radius,
    float tmin, float tmax,
    float* t_out, float3* normal_out);

__device__ float2 sphere_normal_to_uv(const float3& N);

__device__ float3 transform_point(const float transform[12], const float3& p);
__device__ float3 transform_vector(const float transform[12], const float3& v);
__device__ float3 transform_normal(const float inv_transpose[12], const float3& n);

#endif /* CUS3D_MATH_CUH */
