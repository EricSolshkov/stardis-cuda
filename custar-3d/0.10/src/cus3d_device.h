/**
 * @file cus3d_device.h
 * @brief CUDA Device & Context Management for custar-3d
 *
 * Replaces RTCDevice from Embree with CUDA context wrapper.
 * Manages CUDA device selection, stream creation, and error reporting.
 */

#ifndef CUS3D_DEVICE_H
#define CUS3D_DEVICE_H

#include <cuda_runtime.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Forward declarations */
struct cus3d_device;

/*
 * When compiled inside the s3d library, res_T is already provided by
 * rsys/rsys.h (typedef int res_T; RES_OK=0).  For standalone tests or
 * other consumers that don't pull in rsys, we supply a minimal fallback.
 */
#ifndef RES_OK
  typedef int res_T;
  #define RES_OK       0
  #define RES_BAD_ARG  1
  #define RES_MEM_ERR  2
  #define RES_ERR      (-1)
#endif

/**
 * @brief CUDA device and context wrapper
 */
struct cus3d_device {
    int            cuda_device_id;     /**< Selected CUDA device */
    cudaStream_t   stream;             /**< Primary compute stream */
    cudaStream_t   transfer_stream;    /**< Async H2D/D2H transfer stream */
    size_t         total_mem;          /**< Device total memory */
    int            sm_count;           /**< Streaming multiprocessor count */
    int            max_threads_per_block;  /**< Max threads per block */
};

/**
 * @brief Create a CUDA device context
 * 
 * @param device_id CUDA device ID (0-based), -1 for default device
 * @param out Output device handle
 * @return RES_OK on success, RES_ERR on failure
 */
res_T cus3d_device_create(int device_id, struct cus3d_device** out);

/**
 * @brief Destroy a CUDA device context
 * 
 * @param dev Device to destroy
 */
void cus3d_device_destroy(struct cus3d_device* dev);

/**
 * @brief Synchronize device streams
 * 
 * @param dev Device to synchronize
 */
void cus3d_device_sync(struct cus3d_device* dev);

/**
 * @brief Get last CUDA error as string
 * 
 * @return Error message string
 */
const char* cus3d_get_last_error(void);

#ifdef __cplusplus
}
#endif

#endif /* CUS3D_DEVICE_H */
