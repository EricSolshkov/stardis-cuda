/**
 * @file cus3d_device.cpp
 * @brief CUDA Device & Context Management implementation
 *
 * Replaces Embree RTCDevice with CUDA context wrapper.
 * Manages CUDA device selection, dual-stream creation (compute + transfer),
 * device property query, synchronization, and thread-safe error reporting.
 */

#include "cus3d_device.h"
#include <cuda_runtime.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>

/*******************************************************************************
 * Thread-local error buffer
 *
 * Each thread gets its own error string so concurrent callers don't clobber
 * each other's diagnostics.  The size is generous enough for any CUDA error
 * string plus a short contextual prefix.
 ******************************************************************************/
#define CUS3D_ERROR_BUF_SIZE 512

#ifdef _MSC_VER
  static __declspec(thread) char tls_last_error[CUS3D_ERROR_BUF_SIZE] = {0};
#else
  static __thread char tls_last_error[CUS3D_ERROR_BUF_SIZE] = {0};
#endif

/*******************************************************************************
 * Internal helpers
 ******************************************************************************/

/** Store a formatted error message into the thread-local buffer. */
static void
set_error(const char* fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    vsnprintf(tls_last_error, CUS3D_ERROR_BUF_SIZE, fmt, args);
    va_end(args);
}

/** Clear the thread-local error buffer. */
static void
clear_error(void)
{
    tls_last_error[0] = '\0';
}

/**
 * Check a CUDA runtime call and, on failure, store the error string.
 * Returns RES_ERR from the enclosing function.
 */
#define CUDA_CHECK(call, context)                                              \
    do {                                                                       \
        cudaError_t err__ = (call);                                            \
        if (err__ != cudaSuccess) {                                            \
            set_error("cus3d_device: %s failed -- %s",                         \
                      (context), cudaGetErrorString(err__));                    \
            return RES_ERR;                                                    \
        }                                                                      \
    } while (0)

/**
 * Same as CUDA_CHECK but for void-returning functions (no return value).
 * Logs the error but continues execution.
 */
#define CUDA_CHECK_VOID(call, context)                                         \
    do {                                                                       \
        cudaError_t err__ = (call);                                            \
        if (err__ != cudaSuccess) {                                            \
            set_error("cus3d_device: %s failed -- %s",                         \
                      (context), cudaGetErrorString(err__));                    \
        }                                                                      \
    } while (0)

/*******************************************************************************
 * Exported functions
 ******************************************************************************/

res_T
cus3d_device_create(int device_id, struct cus3d_device** out)
{
    struct cus3d_device* dev = NULL;
    cudaDeviceProp props;
    int dev_count = 0;

    clear_error();

    if (!out) {
        set_error("cus3d_device_create: out parameter is NULL");
        return RES_ERR;
    }
    *out = NULL;

    /* --- Validate device availability --- */
    CUDA_CHECK(cudaGetDeviceCount(&dev_count), "cudaGetDeviceCount");

    if (dev_count == 0) {
        set_error("cus3d_device_create: no CUDA-capable devices found");
        return RES_ERR;
    }

    /* Resolve device_id: -1 means "pick the default (device 0)" */
    if (device_id < 0) {
        device_id = 0;
    }
    if (device_id >= dev_count) {
        set_error("cus3d_device_create: device_id %d out of range "
                  "(have %d device(s))", device_id, dev_count);
        return RES_ERR;
    }

    /* --- Select the device --- */
    CUDA_CHECK(cudaSetDevice(device_id), "cudaSetDevice");

    /* --- Query device properties --- */
    CUDA_CHECK(cudaGetDeviceProperties(&props, device_id),
               "cudaGetDeviceProperties");

    /* --- Allocate the handle --- */
    dev = (struct cus3d_device*)calloc(1, sizeof(struct cus3d_device));
    if (!dev) {
        set_error("cus3d_device_create: host memory allocation failed");
        return RES_ERR;
    }

    dev->cuda_device_id       = device_id;
    dev->total_mem            = props.totalGlobalMem;
    dev->sm_count             = props.multiProcessorCount;
    dev->max_threads_per_block = props.maxThreadsPerBlock;
    dev->stream               = NULL;
    dev->transfer_stream      = NULL;

    /* --- Create the primary compute stream --- */
    {
        cudaError_t err = cudaStreamCreate(&dev->stream);
        if (err != cudaSuccess) {
            set_error("cus3d_device_create: cudaStreamCreate (compute) "
                      "failed -- %s", cudaGetErrorString(err));
            free(dev);
            return RES_ERR;
        }
    }

    /* --- Create the async transfer stream --- */
    {
        cudaError_t err = cudaStreamCreate(&dev->transfer_stream);
        if (err != cudaSuccess) {
            set_error("cus3d_device_create: cudaStreamCreate (transfer) "
                      "failed -- %s", cudaGetErrorString(err));
            /* Clean up the compute stream before bailing out */
            cudaStreamDestroy(dev->stream);
            free(dev);
            return RES_ERR;
        }
    }

    *out = dev;
    return RES_OK;
}

void
cus3d_device_destroy(struct cus3d_device* dev)
{
    if (!dev) return;

    clear_error();

    /*
     * Synchronize both streams before tearing them down so that any
     * in-flight work completes.  We intentionally ignore errors here
     * because we're in a teardown path and there's nothing useful
     * the caller can do about a sync failure at this point.
     */
    if (dev->stream) {
        CUDA_CHECK_VOID(cudaStreamSynchronize(dev->stream),
                        "cudaStreamSynchronize (compute)");
        CUDA_CHECK_VOID(cudaStreamDestroy(dev->stream),
                        "cudaStreamDestroy (compute)");
        dev->stream = NULL;
    }

    if (dev->transfer_stream) {
        CUDA_CHECK_VOID(cudaStreamSynchronize(dev->transfer_stream),
                        "cudaStreamSynchronize (transfer)");
        CUDA_CHECK_VOID(cudaStreamDestroy(dev->transfer_stream),
                        "cudaStreamDestroy (transfer)");
        dev->transfer_stream = NULL;
    }

    free(dev);
}

void
cus3d_device_sync(struct cus3d_device* dev)
{
    if (!dev) return;

    clear_error();

    /*
     * Synchronize both streams.  The compute stream carries kernel
     * launches; the transfer stream carries async memcpys.
     */
    CUDA_CHECK_VOID(cudaStreamSynchronize(dev->stream),
                    "cudaStreamSynchronize (compute)");
    CUDA_CHECK_VOID(cudaStreamSynchronize(dev->transfer_stream),
                    "cudaStreamSynchronize (transfer)");
}

const char*
cus3d_get_last_error(void)
{
    return tls_last_error;
}
