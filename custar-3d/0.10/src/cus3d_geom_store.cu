#include "cus3d_geom_store.h"
#include "cus3d_math.cuh"
#include "cus3d_types.h"

#include <cuda_runtime.h>
#include <device_launch_parameters.h>
#include <stdio.h>
#include <stdarg.h>

#define GEOM_CU_ERR_BUF_SIZE 512

#ifdef _MSC_VER
  static __declspec(thread) char tls_geom_cu_error[GEOM_CU_ERR_BUF_SIZE] = {0};
#else
  static __thread char tls_geom_cu_error[GEOM_CU_ERR_BUF_SIZE] = {0};
#endif

static void
geom_cu_set_error(const char* fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    vsnprintf(tls_geom_cu_error, GEOM_CU_ERR_BUF_SIZE, fmt, args);
    va_end(args);
}

#define GEOM_CU_CHECK(call, context)                                           \
    do {                                                                       \
        cudaError_t err__ = (call);                                            \
        if (err__ != cudaSuccess) {                                            \
            geom_cu_set_error("cus3d_geom_store.cu: %s failed -- %s",         \
                              (context), cudaGetErrorString(err__));            \
            return RES_ERR;                                                    \
        }                                                                      \
    } while (0)

/* Compute AABB for each triangle.
 * One thread per triangle.  Writes to boxes[prim_offset + tid]. */
__global__ void compute_triangle_bounds_kernel(
    const float3* __restrict__ vertices,
    const uint3*  __restrict__ indices,
    struct box3f* __restrict__ boxes,
    uint32_t      ntris,
    uint32_t      prim_offset)
{
    uint32_t tid = blockIdx.x * blockDim.x + threadIdx.x;
    if (tid >= ntris) return;

    uint3 tri = indices[prim_offset + tid];
    float3 v0 = vertices[tri.x];
    float3 v1 = vertices[tri.y];
    float3 v2 = vertices[tri.z];

    float3 lo = fminf(fminf(v0, v1), v2);
    float3 hi = fmaxf(fmaxf(v0, v1), v2);

    uint32_t dst = prim_offset + tid;
    boxes[dst].lower[0] = lo.x;
    boxes[dst].lower[1] = lo.y;
    boxes[dst].lower[2] = lo.z;
    boxes[dst].upper[0] = hi.x;
    boxes[dst].upper[1] = hi.y;
    boxes[dst].upper[2] = hi.z;
}

/* Compute AABB for each sphere.
 * One thread per sphere.  Writes to boxes[prim_offset + tid]. */
__global__ void compute_sphere_bounds_kernel(
    const struct sphere_gpu* __restrict__ spheres,
    struct box3f* __restrict__            boxes,
    uint32_t                              nspheres,
    uint32_t                              prim_offset)
{
    uint32_t tid = blockIdx.x * blockDim.x + threadIdx.x;
    if (tid >= nspheres) return;

    struct sphere_gpu sp = spheres[tid];
    float r = sp.radius;

    uint32_t dst = prim_offset + tid;
    boxes[dst].lower[0] = sp.cx - r;
    boxes[dst].lower[1] = sp.cy - r;
    boxes[dst].lower[2] = sp.cz - r;
    boxes[dst].upper[0] = sp.cx + r;
    boxes[dst].upper[1] = sp.cy + r;
    boxes[dst].upper[2] = sp.cz + r;
}

/* Host entry point: launch both AABB kernels */
extern "C" res_T
cus3d_geom_store_compute_bounds(
    struct cus3d_geom_store* store,
    struct cus3d_device*     dev)
{
    if (!store || !dev)
        return RES_ERR;

    if (store->total_prims == 0)
        return RES_OK;

    const unsigned int block_size = 256;
    cudaStream_t s = dev->stream;

    if (store->total_tris > 0) {
        unsigned int grid = (store->total_tris + block_size - 1) / block_size;
        compute_triangle_bounds_kernel<<<grid, block_size, 0, s>>>(
            store->d_vertices.data,
            store->d_indices.data,
            store->d_boxes.data,
            store->total_tris,
            0);
        GEOM_CU_CHECK(cudaGetLastError(), "launch compute_triangle_bounds");
    }

    if (store->total_spheres > 0) {
        unsigned int grid =
            (store->total_spheres + block_size - 1) / block_size;
        compute_sphere_bounds_kernel<<<grid, block_size, 0, s>>>(
            store->d_spheres,
            store->d_boxes.data,
            store->total_spheres,
            store->total_tris);
        GEOM_CU_CHECK(cudaGetLastError(), "launch compute_sphere_bounds");
    }

    GEOM_CU_CHECK(cudaStreamSynchronize(s), "sync after bounds computation");

    return RES_OK;
}
