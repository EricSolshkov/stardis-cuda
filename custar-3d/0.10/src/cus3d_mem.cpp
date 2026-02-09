/**
 * @file cus3d_mem.cpp
 * @brief GPU Memory Manager implementation
 *
 * Wraps cudaMallocAsync / cudaFreeAsync / cudaMemcpyAsync for typed
 * GPU buffers.  Error reporting uses the same thread-local pattern
 * as cus3d_device.cpp.
 */

#include "cus3d_mem.h"
#include <cuda_runtime.h>
#include <stdio.h>
#include <string.h>
#include <stdarg.h>

/*******************************************************************************
 * Thread-local error buffer  (mirrors cus3d_device.cpp)
 ******************************************************************************/
#define CUS3D_MEM_ERROR_BUF_SIZE 512

#ifdef _MSC_VER
  static __declspec(thread) char tls_mem_error[CUS3D_MEM_ERROR_BUF_SIZE] = {0};
#else
  static __thread char tls_mem_error[CUS3D_MEM_ERROR_BUF_SIZE] = {0};
#endif

static void
mem_set_error(const char* fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    vsnprintf(tls_mem_error, CUS3D_MEM_ERROR_BUF_SIZE, fmt, args);
    va_end(args);
}

static void
mem_clear_error(void)
{
    tls_mem_error[0] = '\0';
}

#define MEM_CUDA_CHECK(call, context)                                          \
    do {                                                                       \
        cudaError_t err__ = (call);                                            \
        if (err__ != cudaSuccess) {                                            \
            mem_set_error("cus3d_mem: %s failed -- %s",                        \
                          (context), cudaGetErrorString(err__));                \
            return RES_ERR;                                                    \
        }                                                                      \
    } while (0)

#define MEM_CUDA_CHECK_VOID(call, context)                                     \
    do {                                                                       \
        cudaError_t err__ = (call);                                            \
        if (err__ != cudaSuccess) {                                            \
            mem_set_error("cus3d_mem: %s failed -- %s",                        \
                          (context), cudaGetErrorString(err__));                \
        }                                                                      \
    } while (0)

/*******************************************************************************
 * Internal: generic buffer operations
 *
 * Each public function delegates to one of these using the appropriate
 * element size.  This avoids duplicating the cuda* call sequences for
 * every type.
 ******************************************************************************/

static res_T
buf_alloc(void** d_ptr, size_t* count_out, size_t* cap_out,
          size_t count, size_t elem_size, cudaStream_t s)
{
    mem_clear_error();

    if (!d_ptr) {
        mem_set_error("cus3d_mem: alloc: buffer pointer is NULL");
        return RES_ERR;
    }

    *d_ptr    = NULL;
    *count_out = 0;
    *cap_out   = 0;

    if (count == 0)
        return RES_OK;

    MEM_CUDA_CHECK(cudaMallocAsync(d_ptr, count * elem_size, s),
                   "cudaMallocAsync");

    *count_out = count;
    *cap_out   = count;
    return RES_OK;
}

static void
buf_free(void** d_ptr, size_t* count_out, size_t* cap_out, cudaStream_t s)
{
    mem_clear_error();

    if (!d_ptr || !*d_ptr)
        return;

    MEM_CUDA_CHECK_VOID(cudaFreeAsync(*d_ptr, s), "cudaFreeAsync");

    *d_ptr     = NULL;
    *count_out = 0;
    *cap_out   = 0;
}

static res_T
buf_upload(void* d_dst, const void* h_src,
           size_t count, size_t elem_size, cudaStream_t s)
{
    mem_clear_error();

    if (count == 0)
        return RES_OK;

    if (!d_dst || !h_src) {
        mem_set_error("cus3d_mem: upload: NULL pointer "
                      "(d_dst=%p, h_src=%p)", d_dst, h_src);
        return RES_ERR;
    }

    MEM_CUDA_CHECK(cudaMemcpyAsync(d_dst, h_src,
                                   count * elem_size,
                                   cudaMemcpyHostToDevice, s),
                   "cudaMemcpyAsync H2D");

    return RES_OK;
}

static res_T
buf_download(void* h_dst, const void* d_src,
             size_t count, size_t elem_size, cudaStream_t s)
{
    mem_clear_error();

    if (count == 0)
        return RES_OK;

    if (!h_dst || !d_src) {
        mem_set_error("cus3d_mem: download: NULL pointer "
                      "(h_dst=%p, d_src=%p)", h_dst, d_src);
        return RES_ERR;
    }

    MEM_CUDA_CHECK(cudaMemcpyAsync(h_dst, d_src,
                                   count * elem_size,
                                   cudaMemcpyDeviceToHost, s),
                   "cudaMemcpyAsync D2H");

    return RES_OK;
}

static res_T
buf_resize(void** d_ptr, size_t* count_out, size_t* cap_out,
           size_t new_count, size_t elem_size, cudaStream_t s)
{
    mem_clear_error();

    if (!d_ptr) {
        mem_set_error("cus3d_mem: resize: buffer pointer is NULL");
        return RES_ERR;
    }

    if (new_count == 0) {
        buf_free(d_ptr, count_out, cap_out, s);
        return RES_OK;
    }

    if (new_count <= *cap_out) {
        *count_out = new_count;
        return RES_OK;
    }

    void* new_ptr = NULL;
    MEM_CUDA_CHECK(cudaMallocAsync(&new_ptr, new_count * elem_size, s),
                   "cudaMallocAsync (resize)");

    if (*d_ptr && *count_out > 0) {
        size_t copy_count = (*count_out < new_count) ? *count_out : new_count;
        MEM_CUDA_CHECK(cudaMemcpyAsync(new_ptr, *d_ptr,
                                       copy_count * elem_size,
                                       cudaMemcpyDeviceToDevice, s),
                       "cudaMemcpyAsync D2D (resize)");

        MEM_CUDA_CHECK_VOID(cudaFreeAsync(*d_ptr, s),
                            "cudaFreeAsync (resize old)");
    }

    *d_ptr     = new_ptr;
    *count_out = new_count;
    *cap_out   = new_count;
    return RES_OK;
}

/*******************************************************************************
 * gpu_buffer_float3
 ******************************************************************************/

res_T
gpu_buffer_float3_alloc(struct gpu_buffer_float3* buf, size_t count,
                        cudaStream_t s)
{
    return buf_alloc((void**)&buf->data, &buf->count, &buf->capacity,
                     count, sizeof(float3), s);
}

void
gpu_buffer_float3_free(struct gpu_buffer_float3* buf, cudaStream_t s)
{
    if (!buf) return;
    buf_free((void**)&buf->data, &buf->count, &buf->capacity, s);
}

res_T
gpu_buffer_float3_upload(struct gpu_buffer_float3* dst, const float3* h_src,
                         size_t count, cudaStream_t s)
{
    return buf_upload(dst->data, h_src, count, sizeof(float3), s);
}

res_T
gpu_buffer_float3_download(float3* h_dst,
                           const struct gpu_buffer_float3* src,
                           size_t count, cudaStream_t s)
{
    return buf_download(h_dst, src->data, count, sizeof(float3), s);
}

res_T
gpu_buffer_float3_resize(struct gpu_buffer_float3* buf, size_t new_count,
                         cudaStream_t s)
{
    return buf_resize((void**)&buf->data, &buf->count, &buf->capacity,
                      new_count, sizeof(float3), s);
}

/*******************************************************************************
 * gpu_buffer_float2
 ******************************************************************************/

res_T
gpu_buffer_float2_alloc(struct gpu_buffer_float2* buf, size_t count,
                        cudaStream_t s)
{
    return buf_alloc((void**)&buf->data, &buf->count, &buf->capacity,
                     count, sizeof(float2), s);
}

void
gpu_buffer_float2_free(struct gpu_buffer_float2* buf, cudaStream_t s)
{
    if (!buf) return;
    buf_free((void**)&buf->data, &buf->count, &buf->capacity, s);
}

res_T
gpu_buffer_float2_upload(struct gpu_buffer_float2* dst, const float2* h_src,
                         size_t count, cudaStream_t s)
{
    return buf_upload(dst->data, h_src, count, sizeof(float2), s);
}

res_T
gpu_buffer_float2_download(float2* h_dst,
                           const struct gpu_buffer_float2* src,
                           size_t count, cudaStream_t s)
{
    return buf_download(h_dst, src->data, count, sizeof(float2), s);
}

res_T
gpu_buffer_float2_resize(struct gpu_buffer_float2* buf, size_t new_count,
                         cudaStream_t s)
{
    return buf_resize((void**)&buf->data, &buf->count, &buf->capacity,
                      new_count, sizeof(float2), s);
}

/*******************************************************************************
 * gpu_buffer_uint3
 ******************************************************************************/

res_T
gpu_buffer_uint3_alloc(struct gpu_buffer_uint3* buf, size_t count,
                       cudaStream_t s)
{
    return buf_alloc((void**)&buf->data, &buf->count, &buf->capacity,
                     count, sizeof(uint3), s);
}

void
gpu_buffer_uint3_free(struct gpu_buffer_uint3* buf, cudaStream_t s)
{
    if (!buf) return;
    buf_free((void**)&buf->data, &buf->count, &buf->capacity, s);
}

res_T
gpu_buffer_uint3_upload(struct gpu_buffer_uint3* dst, const uint3* h_src,
                        size_t count, cudaStream_t s)
{
    return buf_upload(dst->data, h_src, count, sizeof(uint3), s);
}

res_T
gpu_buffer_uint3_download(uint3* h_dst,
                          const struct gpu_buffer_uint3* src,
                          size_t count, cudaStream_t s)
{
    return buf_download(h_dst, src->data, count, sizeof(uint3), s);
}

res_T
gpu_buffer_uint3_resize(struct gpu_buffer_uint3* buf, size_t new_count,
                        cudaStream_t s)
{
    return buf_resize((void**)&buf->data, &buf->count, &buf->capacity,
                      new_count, sizeof(uint3), s);
}

/*******************************************************************************
 * gpu_buffer_uint32
 ******************************************************************************/

res_T
gpu_buffer_uint32_alloc(struct gpu_buffer_uint32* buf, size_t count,
                        cudaStream_t s)
{
    return buf_alloc((void**)&buf->data, &buf->count, &buf->capacity,
                     count, sizeof(unsigned int), s);
}

void
gpu_buffer_uint32_free(struct gpu_buffer_uint32* buf, cudaStream_t s)
{
    if (!buf) return;
    buf_free((void**)&buf->data, &buf->count, &buf->capacity, s);
}

res_T
gpu_buffer_uint32_upload(struct gpu_buffer_uint32* dst,
                         const unsigned int* h_src,
                         size_t count, cudaStream_t s)
{
    return buf_upload(dst->data, h_src, count, sizeof(unsigned int), s);
}

res_T
gpu_buffer_uint32_download(unsigned int* h_dst,
                           const struct gpu_buffer_uint32* src,
                           size_t count, cudaStream_t s)
{
    return buf_download(h_dst, src->data, count, sizeof(unsigned int), s);
}

res_T
gpu_buffer_uint32_resize(struct gpu_buffer_uint32* buf, size_t new_count,
                         cudaStream_t s)
{
    return buf_resize((void**)&buf->data, &buf->count, &buf->capacity,
                      new_count, sizeof(unsigned int), s);
}

/*******************************************************************************
 * gpu_buffer_box3f
 ******************************************************************************/

res_T
gpu_buffer_box3f_alloc(struct gpu_buffer_box3f* buf, size_t count,
                       cudaStream_t s)
{
    return buf_alloc((void**)&buf->data, &buf->count, &buf->capacity,
                     count, sizeof(struct box3f), s);
}

void
gpu_buffer_box3f_free(struct gpu_buffer_box3f* buf, cudaStream_t s)
{
    if (!buf) return;
    buf_free((void**)&buf->data, &buf->count, &buf->capacity, s);
}

res_T
gpu_buffer_box3f_upload(struct gpu_buffer_box3f* dst,
                        const struct box3f* h_src,
                        size_t count, cudaStream_t s)
{
    return buf_upload(dst->data, h_src, count, sizeof(struct box3f), s);
}

res_T
gpu_buffer_box3f_download(struct box3f* h_dst,
                          const struct gpu_buffer_box3f* src,
                          size_t count, cudaStream_t s)
{
    return buf_download(h_dst, src->data, count, sizeof(struct box3f), s);
}

res_T
gpu_buffer_box3f_resize(struct gpu_buffer_box3f* buf, size_t new_count,
                        cudaStream_t s)
{
    return buf_resize((void**)&buf->data, &buf->count, &buf->capacity,
                      new_count, sizeof(struct box3f), s);
}
