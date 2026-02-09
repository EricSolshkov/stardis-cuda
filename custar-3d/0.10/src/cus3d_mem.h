/**
 * @file cus3d_mem.h
 * @brief GPU Memory Manager for custar-3d
 *
 * Typed GPU buffer allocation/deallocation with H2D/D2H transfer support.
 * Replaces Embree's buffer management and rtcNewSharedBuffer.
 *
 * Every buffer type follows the same pattern:
 *   alloc     – allocate device memory (sets count & capacity)
 *   free      – release device memory (zeros the struct)
 *   upload    – host-to-device async copy
 *   download  – device-to-host async copy
 *   resize    – grow/shrink, preserving existing data up to
 *               min(old_count, new_count)
 *
 * All async operations are submitted to the caller-provided cudaStream_t.
 * The caller must synchronize the stream before reading the results.
 */

#ifndef CUS3D_MEM_H
#define CUS3D_MEM_H

#include <cuda_runtime.h>
#include <stddef.h>
#include "cus3d_types.h"  /* struct box3f */

#ifdef __cplusplus
extern "C" {
#endif

#ifndef RES_OK
  typedef int res_T;
  #define RES_OK       0
  #define RES_BAD_ARG  1
  #define RES_MEM_ERR  2
  #define RES_ERR      (-1)
#endif

/* ------------------------------------------------------------------ */
/*  Typed GPU buffer structs                                          */
/* ------------------------------------------------------------------ */

struct gpu_buffer_float3 {
    float3*  data;
    size_t   count;
    size_t   capacity;
};

struct gpu_buffer_float2 {
    float2*  data;
    size_t   count;
    size_t   capacity;
};

struct gpu_buffer_uint3 {
    uint3*   data;
    size_t   count;
    size_t   capacity;
};

struct gpu_buffer_uint32 {
    unsigned int* data;
    size_t   count;
    size_t   capacity;
};

struct gpu_buffer_box3f {
    struct box3f* data;
    size_t   count;
    size_t   capacity;
};

/* ------------------------------------------------------------------ */
/*  gpu_buffer_float3                                                 */
/* ------------------------------------------------------------------ */

res_T gpu_buffer_float3_alloc(struct gpu_buffer_float3* buf, size_t count, cudaStream_t s);
void  gpu_buffer_float3_free(struct gpu_buffer_float3* buf, cudaStream_t s);
res_T gpu_buffer_float3_upload(struct gpu_buffer_float3* dst, const float3* h_src, size_t count, cudaStream_t s);
res_T gpu_buffer_float3_download(float3* h_dst, const struct gpu_buffer_float3* src, size_t count, cudaStream_t s);
res_T gpu_buffer_float3_resize(struct gpu_buffer_float3* buf, size_t new_count, cudaStream_t s);

/* ------------------------------------------------------------------ */
/*  gpu_buffer_float2                                                 */
/* ------------------------------------------------------------------ */

res_T gpu_buffer_float2_alloc(struct gpu_buffer_float2* buf, size_t count, cudaStream_t s);
void  gpu_buffer_float2_free(struct gpu_buffer_float2* buf, cudaStream_t s);
res_T gpu_buffer_float2_upload(struct gpu_buffer_float2* dst, const float2* h_src, size_t count, cudaStream_t s);
res_T gpu_buffer_float2_download(float2* h_dst, const struct gpu_buffer_float2* src, size_t count, cudaStream_t s);
res_T gpu_buffer_float2_resize(struct gpu_buffer_float2* buf, size_t new_count, cudaStream_t s);

/* ------------------------------------------------------------------ */
/*  gpu_buffer_uint3                                                  */
/* ------------------------------------------------------------------ */

res_T gpu_buffer_uint3_alloc(struct gpu_buffer_uint3* buf, size_t count, cudaStream_t s);
void  gpu_buffer_uint3_free(struct gpu_buffer_uint3* buf, cudaStream_t s);
res_T gpu_buffer_uint3_upload(struct gpu_buffer_uint3* dst, const uint3* h_src, size_t count, cudaStream_t s);
res_T gpu_buffer_uint3_download(uint3* h_dst, const struct gpu_buffer_uint3* src, size_t count, cudaStream_t s);
res_T gpu_buffer_uint3_resize(struct gpu_buffer_uint3* buf, size_t new_count, cudaStream_t s);

/* ------------------------------------------------------------------ */
/*  gpu_buffer_uint32                                                 */
/* ------------------------------------------------------------------ */

res_T gpu_buffer_uint32_alloc(struct gpu_buffer_uint32* buf, size_t count, cudaStream_t s);
void  gpu_buffer_uint32_free(struct gpu_buffer_uint32* buf, cudaStream_t s);
res_T gpu_buffer_uint32_upload(struct gpu_buffer_uint32* dst, const unsigned int* h_src, size_t count, cudaStream_t s);
res_T gpu_buffer_uint32_download(unsigned int* h_dst, const struct gpu_buffer_uint32* src, size_t count, cudaStream_t s);
res_T gpu_buffer_uint32_resize(struct gpu_buffer_uint32* buf, size_t new_count, cudaStream_t s);

/* ------------------------------------------------------------------ */
/*  gpu_buffer_box3f                                                  */
/* ------------------------------------------------------------------ */

res_T gpu_buffer_box3f_alloc(struct gpu_buffer_box3f* buf, size_t count, cudaStream_t s);
void  gpu_buffer_box3f_free(struct gpu_buffer_box3f* buf, cudaStream_t s);
res_T gpu_buffer_box3f_upload(struct gpu_buffer_box3f* dst, const struct box3f* h_src, size_t count, cudaStream_t s);
res_T gpu_buffer_box3f_download(struct box3f* h_dst, const struct gpu_buffer_box3f* src, size_t count, cudaStream_t s);
res_T gpu_buffer_box3f_resize(struct gpu_buffer_box3f* buf, size_t new_count, cudaStream_t s);

#ifdef __cplusplus
}
#endif

#endif /* CUS3D_MEM_H */
