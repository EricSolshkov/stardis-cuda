/**
 * @file cus3d_bvh.cu
 * @brief BVH Manager implementation using cuBQL
 *
 * Wraps cuBQL's GPU BVH builder to construct BinaryBVH<float,3> from the
 * bounding boxes produced by cus3d_geom_store.  Supports:
 *   - Primary BLAS build from a geometry store's d_boxes array
 *   - Per-instance child BVH build (two-level instancing)
 *   - Top-level AS (TLAS) build from transformed instance AABBs
 *   - Scene AABB extraction from the BVH root node
 *
 * The file is compiled as CUDA (.cu) because cuBQL is header-only and
 * its builder templates require __CUDACC__.
 *
 * Error reporting uses the same thread-local pattern as cus3d_device.cpp.
 */

/*
 * Pull in the cuBQL GPU builder implementation.  This macro must be
 * defined in exactly ONE translation unit that calls gpuBuilder().
 */
#define CUBQL_GPU_BUILDER_IMPLEMENTATION 1

#include "cus3d_bvh_internal.h"

#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include <float.h>

static_assert(sizeof(struct box3f) == sizeof(cuBQL::box_t<float, 3>),
              "box3f must be layout-compatible with cuBQL::box_t<float,3>");

static_assert(sizeof(struct box3f) == 6 * sizeof(float),
              "box3f must be 24 bytes (6 floats)");
#define BVH_ERR_BUF_SIZE 512

#ifdef _MSC_VER
  static __declspec(thread) char tls_bvh_error[BVH_ERR_BUF_SIZE] = {0};
#else
  static __thread char tls_bvh_error[BVH_ERR_BUF_SIZE] = {0};
#endif

static void
bvh_set_error(const char* fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    vsnprintf(tls_bvh_error, BVH_ERR_BUF_SIZE, fmt, args);
    va_end(args);
}

static void
bvh_clear_error(void)
{
    tls_bvh_error[0] = '\0';
}

#define BVH_CUDA_CHECK(call, context)                                          \
    do {                                                                       \
        cudaError_t err__ = (call);                                            \
        if (err__ != cudaSuccess) {                                            \
            bvh_set_error("cus3d_bvh: %s failed -- %s",                        \
                          (context), cudaGetErrorString(err__));                 \
            return RES_ERR;                                                    \
        }                                                                      \
    } while (0)

#define BVH_CUDA_CHECK_VOID(call, context)                                     \
    do {                                                                       \
        cudaError_t err__ = (call);                                            \
        if (err__ != cudaSuccess) {                                            \
            bvh_set_error("cus3d_bvh: %s failed -- %s",                        \
                          (context), cudaGetErrorString(err__));                 \
        }                                                                      \
    } while (0)

/*******************************************************************************
 * Internal helpers
 ******************************************************************************/

/**
 * Map cus3d_build_quality to cuBQL::BuildConfig.
 *
 *   LOW    -> SPATIAL_MEDIAN with large leaf threshold (fast build)
 *   MEDIUM -> SPATIAL_MEDIAN with default threshold
 *   HIGH   -> SAH (best query quality, slowest build)
 */
static cuBQL::BuildConfig
quality_to_config(cus3d_build_quality quality)
{
    cuBQL::BuildConfig cfg;
    switch (quality) {
    case CUS3D_BUILD_LOW:
        cfg.buildMethod       = cuBQL::BuildConfig::SPATIAL_MEDIAN;
        cfg.makeLeafThreshold = 8;
        break;
    case CUS3D_BUILD_HIGH:
        cfg.enableSAH();
        cfg.makeLeafThreshold = 1;
        break;
    case CUS3D_BUILD_MEDIUM:
    default:
        cfg.buildMethod       = cuBQL::BuildConfig::SPATIAL_MEDIAN;
        cfg.makeLeafThreshold = 1;
        break;
    }
    return cfg;
}

/**
 * Read the root node bounds from a device-side BVH and store them in
 * host-side lower[3]/upper[3] arrays.
 *
 * The root node is always at index 0 in bvh.nodes[].  We copy just
 * that one node to host memory, then extract lower/upper.
 */
static res_T
read_root_bounds(const cuBQL::BinaryBVH<float, 3>& bvh,
                 float lower[3], float upper[3],
                 cudaStream_t s)
{
    if (bvh.numNodes == 0 || bvh.nodes == nullptr) {
        /* Empty BVH -- set degenerate bounds */
        lower[0] = lower[1] = lower[2] =  FLT_MAX;
        upper[0] = upper[1] = upper[2] = -FLT_MAX;
        return RES_OK;
    }

    cuBQL::BinaryBVH<float, 3>::Node root_node;
    BVH_CUDA_CHECK(
        cudaMemcpyAsync(&root_node, bvh.nodes,
                        sizeof(root_node),
                        cudaMemcpyDeviceToHost, s),
        "read root node bounds");
    BVH_CUDA_CHECK(cudaStreamSynchronize(s), "sync after root bounds read");

    lower[0] = root_node.bounds.lower.x;
    lower[1] = root_node.bounds.lower.y;
    lower[2] = root_node.bounds.lower.z;
    upper[0] = root_node.bounds.upper.x;
    upper[1] = root_node.bounds.upper.y;
    upper[2] = root_node.bounds.upper.z;

    return RES_OK;
}

/**
 * Free a cuBQL BinaryBVH (if it has allocated memory).
 */
static void
free_bvh(cuBQL::BinaryBVH<float, 3>& bvh, cudaStream_t s)
{
    if (bvh.nodes != nullptr || bvh.primIDs != nullptr) {
        cuBQL::cuda::free(bvh, s);
    }
    bvh.nodes    = nullptr;
    bvh.primIDs  = nullptr;
    bvh.numNodes = 0;
    bvh.numPrims = 0;
}

/*******************************************************************************
 * Exported functions
 ******************************************************************************/

res_T
cus3d_bvh_create(struct cus3d_bvh** out)
{
    bvh_clear_error();

    if (!out) {
        bvh_set_error("cus3d_bvh_create: out is NULL");
        return RES_ERR;
    }

    struct cus3d_bvh* b =
        (struct cus3d_bvh*)calloc(1, sizeof(struct cus3d_bvh));
    if (!b) {
        bvh_set_error("cus3d_bvh_create: host calloc failed");
        return RES_ERR;
    }

    /* cuBQL BinaryBVH is zero-initialised by calloc -- nodes/primIDs
     * are already NULL.  Explicitly mark as invalid. */
    b->valid      = 0;
    b->tlas_valid = 0;

    /* Degenerate scene bounds */
    b->lower[0] = b->lower[1] = b->lower[2] =  FLT_MAX;
    b->upper[0] = b->upper[1] = b->upper[2] = -FLT_MAX;

    *out = b;
    return RES_OK;
}

void
cus3d_bvh_destroy(struct cus3d_bvh* bvh, struct cus3d_device* dev)
{
    if (!bvh) return;
    bvh_clear_error();

    cudaStream_t s = dev ? dev->stream : (cudaStream_t)0;

    /* Free primary BLAS */
    free_bvh(bvh->bvh, s);
    bvh->valid = 0;

    /* Free instance child BVHs (only if owned) */
    if (bvh->instance_bvhs) {
        for (size_t i = 0; i < bvh->instance_count; ++i) {
            if (bvh->instance_bvhs[i].valid && bvh->instance_bvhs[i].owns_child_bvh)
                free_bvh(bvh->instance_bvhs[i].child_bvh, s);
        }
        free(bvh->instance_bvhs);
        bvh->instance_bvhs    = NULL;
        bvh->instance_count   = 0;
        bvh->instance_capacity = 0;
    }

    /* Free TLAS */
    free_bvh(bvh->tlas, s);
    bvh->tlas_valid = 0;

    /* Free TLAS compact mapping */
    free(bvh->tlas_to_orig);
    bvh->tlas_to_orig = NULL;
    bvh->tlas_count   = 0;

    /* Free instance AABB buffer */
    gpu_buffer_box3f_free(&bvh->d_instance_boxes, s);

    free(bvh);
}

res_T
cus3d_bvh_build(struct cus3d_bvh* bvh,
                const struct cus3d_geom_store* store,
                struct cus3d_device* dev,
                cus3d_build_quality quality)
{
    bvh_clear_error();

    if (!bvh || !store || !dev) {
        bvh_set_error("cus3d_bvh_build: NULL argument");
        return RES_ERR;
    }

    cudaStream_t s = dev->stream;

    /* 1. Free existing BVH if valid */
    if (bvh->valid) {
        free_bvh(bvh->bvh, s);
        bvh->valid = 0;
    }

    /* 2. Empty scene -- nothing to build */
    if (store->total_prims == 0 || store->d_boxes.data == NULL) {
        bvh->lower[0] = bvh->lower[1] = bvh->lower[2] =  FLT_MAX;
        bvh->upper[0] = bvh->upper[1] = bvh->upper[2] = -FLT_MAX;
        return RES_OK;
    }

    /* 3. Configure build quality */
    bvh->config = quality_to_config(quality);

    /* 4. Build BVH on GPU
     *
     * cuBQL expects box_t<float,3>*.  Our d_boxes.data is struct box3f*
     * which is layout-compatible (verified by static_assert above).
     * We reinterpret_cast the device pointer. */
    const cuBQL::box_t<float, 3>* d_cubql_boxes =
        reinterpret_cast<const cuBQL::box_t<float, 3>*>(store->d_boxes.data);

    try {
        cuBQL::gpuBuilder(bvh->bvh,
                          d_cubql_boxes,
                          (uint32_t)store->total_prims,
                          bvh->config,
                          s);
    } catch (const std::exception& e) {
        bvh_set_error("cus3d_bvh_build: cuBQL gpuBuilder threw: %s", e.what());
        return RES_ERR;
    } catch (...) {
        bvh_set_error("cus3d_bvh_build: cuBQL gpuBuilder threw unknown exception");
        return RES_ERR;
    }

    /* 5. Extract root AABB */
    res_T res = read_root_bounds(bvh->bvh, bvh->lower, bvh->upper, s);
    if (res != RES_OK) return res;

    bvh->valid = 1;
    return RES_OK;
}

res_T
cus3d_bvh_build_instance(struct cus3d_bvh* bvh,
                         uint32_t instance_idx,
                         const struct cus3d_geom_store* child_store,
                         struct cus3d_device* dev)
{
    bvh_clear_error();

    if (!bvh || !child_store || !dev) {
        bvh_set_error("cus3d_bvh_build_instance: NULL argument");
        return RES_ERR;
    }

    cudaStream_t s = dev->stream;

    /* Grow the instance array if needed */
    size_t needed = (size_t)instance_idx + 1;
    if (needed > bvh->instance_capacity) {
        size_t new_cap = bvh->instance_capacity ? bvh->instance_capacity * 2 : 8;
        while (new_cap < needed)
            new_cap *= 2;

        instance_bvh_entry* tmp = (instance_bvh_entry*)realloc(
            bvh->instance_bvhs, new_cap * sizeof(instance_bvh_entry));
        if (!tmp) {
            bvh_set_error("cus3d_bvh_build_instance: host realloc failed");
            return RES_ERR;
        }
        /* Zero-init new entries */
        memset(tmp + bvh->instance_capacity, 0,
               (new_cap - bvh->instance_capacity) * sizeof(instance_bvh_entry));
        bvh->instance_bvhs    = tmp;
        bvh->instance_capacity = new_cap;
    }
    if (needed > bvh->instance_count)
        bvh->instance_count = needed;

    instance_bvh_entry* ie = &bvh->instance_bvhs[instance_idx];

    /* Free previous child BVH if valid and owned */
    if (ie->valid && ie->owns_child_bvh) {
        free_bvh(ie->child_bvh, s);
    }
    ie->valid = 0;

    /* Empty child store -- mark invalid, not an error */
    if (child_store->total_prims == 0 || child_store->d_boxes.data == NULL) {
        return RES_OK;
    }

    /* Build child BVH using default (MEDIUM) quality */
    cuBQL::BuildConfig child_config = quality_to_config(CUS3D_BUILD_MEDIUM);

    const cuBQL::box_t<float, 3>* d_cubql_boxes =
        reinterpret_cast<const cuBQL::box_t<float, 3>*>(
            child_store->d_boxes.data);

    try {
        cuBQL::gpuBuilder(ie->child_bvh,
                          d_cubql_boxes,
                          (uint32_t)child_store->total_prims,
                          child_config,
                          s);
    } catch (const std::exception& e) {
        bvh_set_error("cus3d_bvh_build_instance: cuBQL gpuBuilder threw: %s",
                      e.what());
        return RES_ERR;
    } catch (...) {
        bvh_set_error("cus3d_bvh_build_instance: cuBQL gpuBuilder threw "
                      "unknown exception");
        return RES_ERR;
    }

    ie->child_store_offset = 0;
    ie->child_store_count  = child_store->total_prims;
    ie->valid              = 1;
    ie->owns_child_bvh     = 1;

    return RES_OK;
}

/*******************************************************************************
 * Set forward and inverse 3x4 transforms for an instance slot.
 * Must be called after cus3d_bvh_build_instance (which allocated the slot)
 * and before cus3d_bvh_build_tlas (which reads the transforms).
 ******************************************************************************/
res_T
cus3d_bvh_set_instance_transform(struct cus3d_bvh* bvh,
                                 uint32_t instance_idx,
                                 const float forward[12],
                                 const float inverse[12])
{
    bvh_clear_error();

    if (!bvh || !forward || !inverse) {
        bvh_set_error("cus3d_bvh_set_instance_transform: NULL argument");
        return RES_ERR;
    }
    if (instance_idx >= bvh->instance_count) {
        bvh_set_error("cus3d_bvh_set_instance_transform: index %u out of range "
                      "(count=%zu)", instance_idx, bvh->instance_count);
        return RES_ERR;
    }

    instance_bvh_entry* ie = &bvh->instance_bvhs[instance_idx];
    memcpy(ie->transform,     forward, 12 * sizeof(float));
    memcpy(ie->inv_transform, inverse, 12 * sizeof(float));

    return RES_OK;
}

/*******************************************************************************
 * TLAS kernel: compute world-space AABB for each instance by transforming
 * the child BVH root bounds through the instance's 3x4 transform.
 *
 * A 3x4 column-major transform T is stored as:
 *   T[0..2]  = column 0 (x-axis)
 *   T[3..5]  = column 1 (y-axis)
 *   T[6..8]  = column 2 (z-axis)
 *   T[9..11] = column 3 (translation)
 *
 * To compute the AABB of a transformed box, we use the AABB-from-OBB
 * method: for each axis, accumulate signed extents from the transform
 * columns times the local half-extents.
 ******************************************************************************/
__global__ static void
compute_instance_bounds_kernel(
    const struct box3f* __restrict__  child_root_bounds,
    const float*        __restrict__  transforms,
    struct box3f*       __restrict__  out_boxes,
    uint32_t                         num_instances)
{
    uint32_t tid = blockIdx.x * blockDim.x + threadIdx.x;
    if (tid >= num_instances) return;

    struct box3f local = child_root_bounds[tid];
    const float* T = transforms + tid * 12;

    /* Local box center and half-extents */
    float cx = 0.5f * (local.lower[0] + local.upper[0]);
    float cy = 0.5f * (local.lower[1] + local.upper[1]);
    float cz = 0.5f * (local.lower[2] + local.upper[2]);
    float hx = 0.5f * (local.upper[0] - local.lower[0]);
    float hy = 0.5f * (local.upper[1] - local.lower[1]);
    float hz = 0.5f * (local.upper[2] - local.lower[2]);

    /* Transform center: c_world = T * [cx, cy, cz, 1] */
    float wcx = T[0]*cx + T[3]*cy + T[6]*cz + T[9];
    float wcy = T[1]*cx + T[4]*cy + T[7]*cz + T[10];
    float wcz = T[2]*cx + T[5]*cy + T[8]*cz + T[11];

    /* World-space half-extents (AABB from OBB via absolute transform) */
    float whx = fabsf(T[0])*hx + fabsf(T[3])*hy + fabsf(T[6])*hz;
    float why = fabsf(T[1])*hx + fabsf(T[4])*hy + fabsf(T[7])*hz;
    float whz = fabsf(T[2])*hx + fabsf(T[5])*hy + fabsf(T[8])*hz;

    out_boxes[tid].lower[0] = wcx - whx;
    out_boxes[tid].lower[1] = wcy - why;
    out_boxes[tid].lower[2] = wcz - whz;
    out_boxes[tid].upper[0] = wcx + whx;
    out_boxes[tid].upper[1] = wcy + why;
    out_boxes[tid].upper[2] = wcz + whz;
}

res_T
cus3d_bvh_build_tlas(struct cus3d_bvh* bvh,
                     struct cus3d_device* dev)
{
    bvh_clear_error();

    if (!bvh || !dev) {
        bvh_set_error("cus3d_bvh_build_tlas: NULL argument");
        return RES_ERR;
    }

    cudaStream_t s = dev->stream;

    /* Free previous TLAS */
    if (bvh->tlas_valid) {
        free_bvh(bvh->tlas, s);
        bvh->tlas_valid = 0;
    }
    free(bvh->tlas_to_orig);
    bvh->tlas_to_orig = NULL;
    bvh->tlas_count   = 0;

    /* Count valid instances and build compact mapping */
    uint32_t n_valid = 0;
    for (size_t i = 0; i < bvh->instance_count; ++i) {
        if (bvh->instance_bvhs[i].valid)
            n_valid++;
    }

    if (n_valid == 0)
        return RES_OK;

    bvh->tlas_to_orig = (uint32_t*)malloc(n_valid * sizeof(uint32_t));
    if (!bvh->tlas_to_orig) {
        bvh_set_error("cus3d_bvh_build_tlas: tlas_to_orig malloc failed");
        return RES_ERR;
    }
    {
        uint32_t slot = 0;
        for (size_t i = 0; i < bvh->instance_count; ++i) {
            if (bvh->instance_bvhs[i].valid)
                bvh->tlas_to_orig[slot++] = (uint32_t)i;
        }
    }
    bvh->tlas_count = n_valid;

    /* Allocate GPU buffer for world-space instance AABBs */
    gpu_buffer_box3f_free(&bvh->d_instance_boxes, s);
    res_T res = gpu_buffer_box3f_alloc(&bvh->d_instance_boxes, n_valid, s);
    if (res != RES_OK) return res;

    /* Stage child root bounds and transforms on host, then upload */
    struct box3f* h_child_bounds =
        (struct box3f*)malloc(n_valid * sizeof(struct box3f));
    float* h_transforms =
        (float*)malloc(n_valid * 12 * sizeof(float));
    if (!h_child_bounds || !h_transforms) {
        free(h_child_bounds);
        free(h_transforms);
        bvh_set_error("cus3d_bvh_build_tlas: host malloc failed");
        return RES_ERR;
    }

    /* Read each valid instance's child BVH root bounds (using tlas_to_orig) */
    for (uint32_t slot = 0; slot < n_valid; ++slot) {
        uint32_t orig = bvh->tlas_to_orig[slot];
        instance_bvh_entry* ie = &bvh->instance_bvhs[orig];

        float lo[3], hi[3];
        res = read_root_bounds(ie->child_bvh, lo, hi, s);
        if (res != RES_OK) {
            free(h_child_bounds);
            free(h_transforms);
            return res;
        }
        memcpy(h_child_bounds[slot].lower, lo, 3 * sizeof(float));
        memcpy(h_child_bounds[slot].upper, hi, 3 * sizeof(float));
        memcpy(h_transforms + slot * 12, ie->transform, 12 * sizeof(float));
    }

    /* Upload child root bounds to a temp GPU buffer */
    struct gpu_buffer_box3f d_child_bounds;
    memset(&d_child_bounds, 0, sizeof(d_child_bounds));
    res = gpu_buffer_box3f_alloc(&d_child_bounds, n_valid, s);
    if (res != RES_OK) {
        free(h_child_bounds); free(h_transforms);
        return res;
    }
    res = gpu_buffer_box3f_upload(&d_child_bounds, h_child_bounds, n_valid, s);
    if (res != RES_OK) {
        free(h_child_bounds); free(h_transforms);
        gpu_buffer_box3f_free(&d_child_bounds, s);
        return res;
    }

    /* Upload transforms as raw float array */
    float* d_transforms = NULL;
    BVH_CUDA_CHECK(
        cudaMallocAsync(&d_transforms, n_valid * 12 * sizeof(float), s),
        "alloc d_transforms");
    BVH_CUDA_CHECK(
        cudaMemcpyAsync(d_transforms, h_transforms,
                        n_valid * 12 * sizeof(float),
                        cudaMemcpyHostToDevice, s),
        "upload transforms");

    free(h_child_bounds);
    free(h_transforms);

    /* Launch kernel to compute world-space instance AABBs */
    {
        uint32_t block = 256;
        uint32_t grid = (n_valid + block - 1) / block;
        compute_instance_bounds_kernel<<<grid, block, 0, s>>>(
            d_child_bounds.data,
            d_transforms,
            bvh->d_instance_boxes.data,
            n_valid);
    }

    BVH_CUDA_CHECK(cudaStreamSynchronize(s), "sync after instance bounds");

    /* Free temp buffers */
    gpu_buffer_box3f_free(&d_child_bounds, s);
    BVH_CUDA_CHECK_VOID(cudaFreeAsync(d_transforms, s), "free d_transforms");

    /* Build TLAS from instance bounding boxes */
    const cuBQL::box_t<float, 3>* d_cubql_boxes =
        reinterpret_cast<const cuBQL::box_t<float, 3>*>(
            bvh->d_instance_boxes.data);

    cuBQL::BuildConfig tlas_config;
    tlas_config.buildMethod       = cuBQL::BuildConfig::SPATIAL_MEDIAN;
    tlas_config.makeLeafThreshold = 1;

    try {
        cuBQL::gpuBuilder(bvh->tlas,
                          d_cubql_boxes,
                          n_valid,
                          tlas_config,
                          s);
    } catch (const std::exception& e) {
        bvh_set_error("cus3d_bvh_build_tlas: cuBQL gpuBuilder threw: %s",
                      e.what());
        return RES_ERR;
    } catch (...) {
        bvh_set_error("cus3d_bvh_build_tlas: cuBQL gpuBuilder threw "
                      "unknown exception");
        return RES_ERR;
    }

    bvh->tlas_valid = 1;

    /* Update the overall scene bounds to include instance world-space AABBs.
     * The BLAS bounds (bvh->lower/upper) only cover direct geometry; when the
     * parent scene has only instances, they remain FLT_MAX/-FLT_MAX.  Read the
     * TLAS root AABB and merge it in. */
    {
        float tlas_lo[3], tlas_hi[3];
        res_T r = read_root_bounds(bvh->tlas, tlas_lo, tlas_hi, s);
        if (r == RES_OK) {
            for (int k = 0; k < 3; ++k) {
                if (tlas_lo[k] < bvh->lower[k]) bvh->lower[k] = tlas_lo[k];
                if (tlas_hi[k] > bvh->upper[k]) bvh->upper[k] = tlas_hi[k];
            }
        }
    }

    return RES_OK;
}

void
cus3d_bvh_get_bounds(const struct cus3d_bvh* bvh,
                     float lower[3],
                     float upper[3])
{
    if (!bvh || !lower || !upper) return;

    lower[0] = bvh->lower[0];
    lower[1] = bvh->lower[1];
    lower[2] = bvh->lower[2];
    upper[0] = bvh->upper[0];
    upper[1] = bvh->upper[1];
    upper[2] = bvh->upper[2];
}

int
cus3d_bvh_is_valid(const struct cus3d_bvh* bvh)
{
    if (!bvh) return 0;
    return bvh->valid;
}

struct geometry*
cus3d_bvh_get_instance_geometry(const struct cus3d_bvh* bvh,
                                unsigned tlas_idx)
{
    if (!bvh || !bvh->tlas_to_orig || tlas_idx >= bvh->tlas_count)
        return NULL;
    uint32_t orig = bvh->tlas_to_orig[tlas_idx];
    if (orig >= bvh->instance_count)
        return NULL;
    return bvh->instance_bvhs[orig].inst_geom;
}

res_T
cus3d_bvh_set_instance_geometry(struct cus3d_bvh* bvh,
                                unsigned instance_idx,
                                struct geometry* geom)
{
    if (!bvh) return RES_ERR;
    if (instance_idx >= bvh->instance_count) return RES_ERR;
    bvh->instance_bvhs[instance_idx].inst_geom = geom;
    return RES_OK;
}

res_T
cus3d_bvh_set_instance_child_store(struct cus3d_bvh* bvh,
                                    unsigned instance_idx,
                                    struct cus3d_geom_store* child_store)
{
    if (!bvh) return RES_ERR;
    if (instance_idx >= bvh->instance_count) return RES_ERR;
    bvh->instance_bvhs[instance_idx].child_store = child_store;
    return RES_OK;
}

res_T
cus3d_bvh_reset_instances(struct cus3d_bvh* bvh,
                          struct cus3d_device* dev)
{
    bvh_clear_error();
    if (!bvh) return RES_ERR;

    cudaStream_t s = dev ? dev->stream : (cudaStream_t)0;

    /* Free owned child BVHs and invalidate all slots */
    for (size_t i = 0; i < bvh->instance_count; ++i) {
        if (bvh->instance_bvhs[i].valid && bvh->instance_bvhs[i].owns_child_bvh)
            free_bvh(bvh->instance_bvhs[i].child_bvh, s);
        memset(&bvh->instance_bvhs[i], 0, sizeof(bvh->instance_bvhs[i]));
    }
    bvh->instance_count = 0;

    /* Free TLAS */
    if (bvh->tlas_valid) {
        free_bvh(bvh->tlas, s);
        bvh->tlas_valid = 0;
    }
    free(bvh->tlas_to_orig);
    bvh->tlas_to_orig = NULL;
    bvh->tlas_count   = 0;

    return RES_OK;
}

res_T
cus3d_bvh_register_pseudo_instance(
    struct cus3d_bvh* bvh,
    uint32_t instance_idx,
    struct cus3d_geom_store* parent_store)
{
    bvh_clear_error();

    if (!bvh || !parent_store) {
        bvh_set_error("cus3d_bvh_register_pseudo_instance: NULL argument");
        return RES_ERR;
    }
    if (!bvh->valid) {
        bvh_set_error("cus3d_bvh_register_pseudo_instance: parent BLAS not built");
        return RES_ERR;
    }

    /* Grow instance array if needed */
    size_t needed = (size_t)instance_idx + 1;
    if (needed > bvh->instance_capacity) {
        size_t new_cap = bvh->instance_capacity ? bvh->instance_capacity * 2 : 8;
        while (new_cap < needed)
            new_cap *= 2;
        instance_bvh_entry* tmp = (instance_bvh_entry*)realloc(
            bvh->instance_bvhs, new_cap * sizeof(instance_bvh_entry));
        if (!tmp) {
            bvh_set_error("cus3d_bvh_register_pseudo_instance: realloc failed");
            return RES_ERR;
        }
        memset(tmp + bvh->instance_capacity, 0,
               (new_cap - bvh->instance_capacity) * sizeof(instance_bvh_entry));
        bvh->instance_bvhs     = tmp;
        bvh->instance_capacity = new_cap;
    }
    if (needed > bvh->instance_count)
        bvh->instance_count = needed;

    instance_bvh_entry* ie = &bvh->instance_bvhs[instance_idx];
    memset(ie, 0, sizeof(*ie));

    /* Share parent BLAS (non-owning reference) */
    ie->child_bvh        = bvh->bvh;
    ie->owns_child_bvh   = 0;
    ie->valid            = 1;
    ie->inst_geom        = NULL; /* NULL marks pseudo-instance */
    ie->child_store      = parent_store;
    ie->child_store_offset = 0;
    ie->child_store_count  = parent_store->total_prims;

    /* Identity 3x4 transform */
    static const float identity[12] = {
        1.f, 0.f, 0.f,
        0.f, 1.f, 0.f,
        0.f, 0.f, 1.f,
        0.f, 0.f, 0.f
    };
    memcpy(ie->transform,     identity, sizeof(identity));
    memcpy(ie->inv_transform, identity, sizeof(identity));

    return RES_OK;
}

const struct cus3d_geom_store*
cus3d_bvh_get_instance_store(const struct cus3d_bvh* bvh,
                             unsigned tlas_idx)
{
    if (!bvh || !bvh->tlas_to_orig || tlas_idx >= bvh->tlas_count)
        return NULL;
    uint32_t orig = bvh->tlas_to_orig[tlas_idx];
    if (orig >= bvh->instance_count)
        return NULL;
    return bvh->instance_bvhs[orig].child_store;
}
