/* Copyright (C) 2015-2023 |Méso|Star> (contact@meso-star.com)
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>. */

#include "s3d.h"
#include "s3d_c.h"
#include "s3d_device_c.h"
#include "cus3d_device.h"

#include <rsys/logger.h>
#include <rsys/mem_allocator.h>

/*******************************************************************************
 * Helper functions
 ******************************************************************************/
static INLINE void
log_msg
  (struct s3d_device* dev,
   const enum log_type stream,
   const char* msg,
   va_list vargs)
{
  ASSERT(dev && msg);
  if(dev->verbose) {
    res_T res; (void)res;
    res = logger_vprint(dev->logger, stream, msg, vargs);
    ASSERT(res == RES_OK);
  }
}

static void
device_release(ref_T* ref)
{
  struct s3d_device* dev;
  ASSERT(ref);
  dev = CONTAINER_OF(ref, struct s3d_device, ref);
  ASSERT(flist_name_is_empty(&dev->names) == 1);
  flist_name_release(&dev->names);
  cus3d_device_destroy(dev->gpu);
  dev->gpu = NULL;
  MEM_RM(dev->allocator, dev);
}

/*******************************************************************************
 * Exported s3d_device functions
 ******************************************************************************/
res_T
s3d_device_create
  (struct logger* logger,
   struct mem_allocator* mem_allocator,
   const int verbose,
   struct s3d_device** out_dev)
{
  struct s3d_device* dev = NULL;
  struct mem_allocator* allocator;
  res_T res = RES_OK;

  if(!out_dev) {
    res = RES_BAD_ARG;
    goto error;
  }

  allocator = mem_allocator ? mem_allocator : &mem_default_allocator;
  dev = (struct s3d_device*)MEM_CALLOC(allocator, 1, sizeof(struct s3d_device));
  if(!dev) {
    res = RES_MEM_ERR;
    goto error;
  }
  dev->logger = logger ? logger : LOGGER_DEFAULT;
  dev->allocator = allocator;
  dev->verbose = verbose;
  dev->gpu = NULL;
  flist_name_init(allocator, &dev->names);
  ref_init(&dev->ref);

  res = cus3d_device_create(-1, &dev->gpu);
  if(res != RES_OK) {
    log_error(dev, "Could not create the CUDA device -- %s.\n",
      cus3d_get_last_error());
    goto error;
  }

  if(dev->verbose) {
    log_warning(dev, "CUDA device %d: %zuMB, %d SMs, %d max threads/block.\n",
      dev->gpu->cuda_device_id,
      dev->gpu->total_mem / (1024 * 1024),
      dev->gpu->sm_count,
      dev->gpu->max_threads_per_block);
  }

exit:
  if(out_dev) *out_dev = dev;
  return res;
error:
  if(dev) {
    S3D(device_ref_put(dev));
    dev = NULL;
  }
  goto exit;
}

res_T
s3d_device_ref_get(struct s3d_device* dev)
{
  if(!dev) return RES_BAD_ARG;
  ref_get(&dev->ref);
  return RES_OK;
}

res_T
s3d_device_ref_put(struct s3d_device* dev)
{
  if(!dev) return RES_BAD_ARG;
  ref_put(&dev->ref, device_release);
  return RES_OK;
}

/*******************************************************************************
 * Local functions
 ******************************************************************************/
void
log_error(struct s3d_device* dev, const char* msg, ...)
{
  va_list vargs_list;
  ASSERT(dev && msg);

  va_start(vargs_list, msg);
  log_msg(dev, LOG_ERROR, msg, vargs_list);
  va_end(vargs_list);
}

void
log_warning(struct s3d_device* dev, const char* msg, ...)
{
  va_list vargs_list;
  ASSERT(dev && msg);

  va_start(vargs_list, msg);
  log_msg(dev, LOG_WARNING, msg, vargs_list);
  va_end(vargs_list);
}

