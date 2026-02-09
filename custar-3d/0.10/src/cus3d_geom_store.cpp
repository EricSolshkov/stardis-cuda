/**
 * @file cus3d_geom_store.cpp
 * @brief Geometry Data Store -- host-side implementation
 *
 * Collects shapes from an s3d_scene, flattens mesh vertices/indices and
 * sphere data into contiguous host arrays, uploads them to the GPU, and
 * builds the primID-to-geometry lookup table.
 *
 * AABB computation is delegated to GPU kernels in cus3d_geom_store.cu.
 */

#include "cus3d_geom_store.h"
#include "s3d_scene_c.h"
#include "s3d_shape_c.h"
#include "s3d_mesh.h"
#include "s3d_sphere.h"

#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <stdio.h>

/*******************************************************************************
 * Thread-local error buffer (same pattern as cus3d_device / cus3d_mem)
 ******************************************************************************/
#define GEOM_ERR_BUF_SIZE 512

#ifdef _MSC_VER
  static __declspec(thread) char tls_geom_error[GEOM_ERR_BUF_SIZE] = {0};
#else
  static __thread char tls_geom_error[GEOM_ERR_BUF_SIZE] = {0};
#endif

static void
geom_set_error(const char* fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    vsnprintf(tls_geom_error, GEOM_ERR_BUF_SIZE, fmt, args);
    va_end(args);
}

static void
geom_clear_error(void)
{
    tls_geom_error[0] = '\0';
}

#define GEOM_CUDA_CHECK(call, context)                                         \
    do {                                                                       \
        cudaError_t err__ = (call);                                            \
        if (err__ != cudaSuccess) {                                            \
            geom_set_error("cus3d_geom_store: %s failed -- %s",               \
                           (context), cudaGetErrorString(err__));               \
            return RES_ERR;                                                    \
        }                                                                      \
    } while (0)

/*******************************************************************************
 * Internal: geom_entry dynamic array helpers
 ******************************************************************************/

static res_T
entries_ensure_capacity(struct cus3d_geom_store* store, size_t needed)
{
    if (needed <= store->entry_capacity)
        return RES_OK;

    size_t new_cap = store->entry_capacity ? store->entry_capacity * 2 : 16;
    while (new_cap < needed)
        new_cap *= 2;

    struct geom_entry* tmp = (struct geom_entry*)realloc(
        store->entries, new_cap * sizeof(struct geom_entry));
    if (!tmp) {
        geom_set_error("cus3d_geom_store: host realloc failed");
        return RES_ERR;
    }
    store->entries = tmp;
    store->entry_capacity = new_cap;
    return RES_OK;
}

static struct geom_entry*
entries_push_back(struct cus3d_geom_store* store)
{
    if (entries_ensure_capacity(store, store->entry_count + 1) != RES_OK)
        return NULL;
    struct geom_entry* e = &store->entries[store->entry_count];
    memset(e, 0, sizeof(*e));
    store->entry_count++;
    return e;
}

/*******************************************************************************
 * Internal: raw device buffer helpers (sphere_gpu, geom_gpu_entry)
 *
 * These are simple POD types that don't appear often enough to warrant
 * full gpu_buffer_* wrappers in cus3d_mem.
 ******************************************************************************/

static res_T
raw_device_alloc(void** d_ptr, size_t* capacity, size_t count,
                 size_t elem_size, cudaStream_t s)
{
    if (count == 0) {
        *d_ptr = NULL;
        *capacity = 0;
        return RES_OK;
    }
    if (count <= *capacity && *d_ptr != NULL)
        return RES_OK;

    if (*d_ptr) {
        cudaError_t err = cudaFreeAsync(*d_ptr, s);
        if (err != cudaSuccess) {
            geom_set_error("cus3d_geom_store: cudaFreeAsync failed -- %s",
                           cudaGetErrorString(err));
            return RES_ERR;
        }
        *d_ptr = NULL;
    }
    cudaError_t err = cudaMallocAsync(d_ptr, count * elem_size, s);
    if (err != cudaSuccess) {
        geom_set_error("cus3d_geom_store: cudaMallocAsync failed -- %s",
                       cudaGetErrorString(err));
        return RES_ERR;
    }
    *capacity = count;
    return RES_OK;
}

static void
raw_device_free(void** d_ptr, size_t* capacity, cudaStream_t s)
{
    if (*d_ptr) {
        cudaFreeAsync(*d_ptr, s);
        *d_ptr = NULL;
    }
    *capacity = 0;
}

/*******************************************************************************
 * Lifecycle
 ******************************************************************************/

res_T
cus3d_geom_store_create(struct cus3d_geom_store** out)
{
    geom_clear_error();

    if (!out) {
        geom_set_error("cus3d_geom_store_create: out is NULL");
        return RES_ERR;
    }

    struct cus3d_geom_store* store =
        (struct cus3d_geom_store*)calloc(1, sizeof(struct cus3d_geom_store));
    if (!store) {
        geom_set_error("cus3d_geom_store_create: host calloc failed");
        return RES_ERR;
    }

    *out = store;
    return RES_OK;
}

void
cus3d_geom_store_destroy(struct cus3d_geom_store* store,
                         struct cus3d_device* dev)
{
    if (!store) return;

    cudaStream_t s = dev ? dev->stream : (cudaStream_t)0;

    gpu_buffer_float3_free(&store->d_vertices, s);
    gpu_buffer_uint3_free(&store->d_indices, s);
    gpu_buffer_box3f_free(&store->d_boxes, s);
    gpu_buffer_uint32_free(&store->d_prim_to_geom, s);

    raw_device_free((void**)&store->d_spheres,
                    &store->d_spheres_capacity, s);
    raw_device_free((void**)&store->d_geom_entries,
                    &store->d_geom_entries_capacity, s);

    free(store->entries);
    free(store);
}

/*******************************************************************************
 * Sync: flatten scene shapes into contiguous GPU arrays
 ******************************************************************************/

/*
 * Pass 1: walk the scene, count totals, and populate geom_entry metadata.
 *
 * Triangles are assigned primIDs [0 .. total_tris-1].
 * Spheres  are assigned primIDs [total_tris .. total_prims-1].
 */
static res_T
count_and_register_shapes(struct cus3d_geom_store* store,
                          struct s3d_scene* scene)
{
    struct htable_shape_iterator it, end;

    store->entry_count = 0;
    store->total_tris = 0;
    store->total_spheres = 0;
    store->total_verts = 0;

    htable_shape_begin(&scene->shapes, &it);
    htable_shape_end(&scene->shapes, &end);

    while (!htable_shape_iterator_eq(&it, &end)) {
        struct s3d_shape* shape = *htable_shape_iterator_data_get(&it);
        htable_shape_iterator_next(&it);

        /* Skip disabled shapes -- they must not participate in BVH
         * construction, AABB computation, or GPU data upload.  The
         * scene_view's cached_geoms still tracks the enable flag so that
         * gpu_dirty / aabb_update are set when the flag changes. */
        if (!shape->is_enabled) continue;

        if (shape->type == GEOM_MESH) {
            if (!shape->data.mesh->indices ||
                !shape->data.mesh->attribs[S3D_POSITION])
                continue;

            size_t ntris  = mesh_get_ntris(shape->data.mesh);
            size_t nverts = mesh_get_nverts(shape->data.mesh);
            if (ntris == 0 || nverts == 0) continue;

            struct geom_entry* e = entries_push_back(store);
            if (!e) return RES_ERR;

            e->shape_name    = shape->id.index;
            e->type          = PRIM_TRIANGLE;
            e->prim_count    = (uint32_t)ntris;
            e->vertex_offset = store->total_verts;
            e->vertex_count  = (uint32_t)nverts;
            e->is_enabled    = shape->is_enabled;
            e->flip_surface  = shape->flip_surface;
            e->filter_func   = shape->data.mesh->filter.func;
            e->filter_data   = shape->data.mesh->filter.data;
            e->shape         = shape;

            store->total_tris  += (uint32_t)ntris;
            store->total_verts += (uint32_t)nverts;

        } else if (shape->type == GEOM_SPHERE) {
            if (sphere_is_degenerated(shape->data.sphere))
                continue;

            struct geom_entry* e = entries_push_back(store);
            if (!e) return RES_ERR;

            e->shape_name    = shape->id.index;
            e->type          = PRIM_SPHERE;
            e->prim_count    = 1;
            e->vertex_offset = 0;
            e->vertex_count  = 0;
            e->is_enabled    = shape->is_enabled;
            e->flip_surface  = shape->flip_surface;
            e->filter_func   = shape->data.sphere->filter.func;
            e->filter_data   = shape->data.sphere->filter.data;
            e->shape         = shape;

            store->total_spheres += 1;
        }
        /* GEOM_INSTANCE is handled at scene_view level, not here */
    }

    store->total_prims = store->total_tris + store->total_spheres;
    return RES_OK;
}

/*
 * Pass 2: assign prim_offset for each entry.
 * Triangles: [0 .. total_tris).  Spheres: [total_tris .. total_prims).
 */
static void
assign_prim_offsets(struct cus3d_geom_store* store)
{
    uint32_t tri_offset    = 0;
    uint32_t sphere_offset = store->total_tris;

    for (size_t i = 0; i < store->entry_count; ++i) {
        struct geom_entry* e = &store->entries[i];
        if (e->type == PRIM_TRIANGLE) {
            e->prim_offset = tri_offset;
            tri_offset += e->prim_count;
        } else {
            e->prim_offset = sphere_offset;
            sphere_offset += e->prim_count;
        }
    }
}

/*
 * Pass 3: flatten mesh vertices and indices into host staging buffers,
 * adjusting index values by vertex_offset so the global vertex array
 * is self-consistent.
 */
static res_T
flatten_and_upload(struct cus3d_geom_store* store, struct cus3d_device* dev)
{
    cudaStream_t s = dev->transfer_stream;
    res_T res;

    /* ---------- Allocate GPU buffers ---------- */
    res = gpu_buffer_float3_alloc(&store->d_vertices, store->total_verts, s);
    if (res != RES_OK) return res;

    res = gpu_buffer_uint3_alloc(&store->d_indices, store->total_tris, s);
    if (res != RES_OK) return res;

    res = gpu_buffer_box3f_alloc(&store->d_boxes, store->total_prims, s);
    if (res != RES_OK) return res;

    res = gpu_buffer_uint32_alloc(&store->d_prim_to_geom, store->total_prims, s);
    if (res != RES_OK) return res;

    res = raw_device_alloc((void**)&store->d_spheres,
                           &store->d_spheres_capacity,
                           store->total_spheres,
                           sizeof(struct sphere_gpu), s);
    if (res != RES_OK) return res;

    res = raw_device_alloc((void**)&store->d_geom_entries,
                           &store->d_geom_entries_capacity,
                           store->entry_count,
                           sizeof(struct geom_gpu_entry), s);
    if (res != RES_OK) return res;

    /* ---------- Stage host data and upload per-entry ---------- */

    /* Host staging for the prim_to_geom lookup table */
    unsigned int* h_prim_to_geom = NULL;
    if (store->total_prims > 0) {
        h_prim_to_geom = (unsigned int*)malloc(
            store->total_prims * sizeof(unsigned int));
        if (!h_prim_to_geom) {
            geom_set_error("cus3d_geom_store: malloc h_prim_to_geom failed");
            return RES_ERR;
        }
    }

    /* Host staging for geom_gpu_entry array */
    struct geom_gpu_entry* h_geom_gpu = NULL;
    if (store->entry_count > 0) {
        h_geom_gpu = (struct geom_gpu_entry*)malloc(
            store->entry_count * sizeof(struct geom_gpu_entry));
        if (!h_geom_gpu) {
            free(h_prim_to_geom);
            geom_set_error("cus3d_geom_store: malloc h_geom_gpu failed");
            return RES_ERR;
        }
    }

    for (size_t i = 0; i < store->entry_count; ++i) {
        struct geom_entry* e = &store->entries[i];

        /* Fill prim_to_geom for this entry's range */
        for (uint32_t p = 0; p < e->prim_count; ++p)
            h_prim_to_geom[e->prim_offset + p] = (unsigned int)i;

        /* Fill compact GPU metadata */
        h_geom_gpu[i].prim_offset   = e->prim_offset;
        h_geom_gpu[i].prim_count    = e->prim_count;
        h_geom_gpu[i].vertex_offset = e->vertex_offset;
        h_geom_gpu[i].type          = (uint8_t)e->type;
        h_geom_gpu[i].is_enabled    = (uint8_t)(e->is_enabled ? 1 : 0);
        h_geom_gpu[i].flip_surface  = (uint8_t)(e->flip_surface ? 1 : 0);
        h_geom_gpu[i].has_filter    = (uint8_t)(e->filter_func ? 1 : 0);

        if (e->type == PRIM_TRIANGLE) {
            struct mesh* m = e->shape->data.mesh;
            float*    h_pos = mesh_get_pos(m);
            uint32_t* h_ids = mesh_get_ids(m);
            size_t ntris  = (size_t)e->prim_count;
            size_t nverts = (size_t)e->vertex_count;

            /* Upload vertices (float3 = 3 floats, mesh stores packed float[3]) */
            GEOM_CUDA_CHECK(
                cudaMemcpyAsync(
                    (float3*)store->d_vertices.data + e->vertex_offset,
                    h_pos,
                    nverts * sizeof(float3),
                    cudaMemcpyHostToDevice, s),
                "upload vertices");

            /* Build offset-adjusted index buffer on host, then upload.
             * The mesh stores indices as flat uint32[ntris*3]; we need uint3
             * with each component += vertex_offset. */
            uint3* h_adj_idx = (uint3*)malloc(ntris * sizeof(uint3));
            if (!h_adj_idx) {
                free(h_prim_to_geom);
                free(h_geom_gpu);
                geom_set_error("cus3d_geom_store: malloc h_adj_idx failed");
                return RES_ERR;
            }
            for (size_t t = 0; t < ntris; ++t) {
                h_adj_idx[t].x = h_ids[t * 3 + 0] + e->vertex_offset;
                h_adj_idx[t].y = h_ids[t * 3 + 1] + e->vertex_offset;
                h_adj_idx[t].z = h_ids[t * 3 + 2] + e->vertex_offset;
            }
            GEOM_CUDA_CHECK(
                cudaMemcpyAsync(
                    store->d_indices.data + e->prim_offset,
                    h_adj_idx,
                    ntris * sizeof(uint3),
                    cudaMemcpyHostToDevice, s),
                "upload indices");
            free(h_adj_idx);

        } else { /* PRIM_SPHERE */
            struct sphere* sp = e->shape->data.sphere;
            struct sphere_gpu h_sg;
            h_sg.cx     = sp->pos[0];
            h_sg.cy     = sp->pos[1];
            h_sg.cz     = sp->pos[2];
            h_sg.radius = sp->radius;

            uint32_t sphere_idx = e->prim_offset - store->total_tris;
            GEOM_CUDA_CHECK(
                cudaMemcpyAsync(
                    store->d_spheres + sphere_idx,
                    &h_sg,
                    sizeof(struct sphere_gpu),
                    cudaMemcpyHostToDevice, s),
                "upload sphere");
        }
    }

    /* Upload prim_to_geom and geom_gpu_entry arrays */
    if (store->total_prims > 0) {
        res = gpu_buffer_uint32_upload(
            &store->d_prim_to_geom, h_prim_to_geom,
            store->total_prims, s);
        free(h_prim_to_geom);
        if (res != RES_OK) { free(h_geom_gpu); return res; }
    }

    if (store->entry_count > 0) {
        GEOM_CUDA_CHECK(
            cudaMemcpyAsync(
                store->d_geom_entries, h_geom_gpu,
                store->entry_count * sizeof(struct geom_gpu_entry),
                cudaMemcpyHostToDevice, s),
            "upload geom_gpu_entries");
        free(h_geom_gpu);
    }

    /* Synchronize the transfer stream to ensure all uploads complete
     * before compute kernels (AABB, BVH) touch the data. */
    GEOM_CUDA_CHECK(cudaStreamSynchronize(s), "sync transfer stream");

    store->needs_rebuild = 1;
    return RES_OK;
}

res_T
cus3d_geom_store_sync(struct cus3d_geom_store* store,
                       struct s3d_scene* scene,
                       struct cus3d_device* dev)
{
    res_T res;
    geom_clear_error();

    if (!store || !scene || !dev) {
        geom_set_error("cus3d_geom_store_sync: NULL argument");
        return RES_ERR;
    }

    /* Free previous GPU data (we do a full rebuild each sync) */
    gpu_buffer_float3_free(&store->d_vertices, dev->stream);
    gpu_buffer_uint3_free(&store->d_indices, dev->stream);
    gpu_buffer_box3f_free(&store->d_boxes, dev->stream);
    gpu_buffer_uint32_free(&store->d_prim_to_geom, dev->stream);
    raw_device_free((void**)&store->d_spheres,
                    &store->d_spheres_capacity, dev->stream);
    raw_device_free((void**)&store->d_geom_entries,
                    &store->d_geom_entries_capacity, dev->stream);

    /* Pass 1: count shapes and build host metadata */
    res = count_and_register_shapes(store, scene);
    if (res != RES_OK) return res;

    if (store->total_prims == 0) {
        store->needs_rebuild = 0;
        return RES_OK;
    }

    /* Pass 2: assign prim offsets */
    assign_prim_offsets(store);

    /* Pass 3: flatten, upload */
    res = flatten_and_upload(store, dev);
    return res;
}

/*******************************************************************************
 * Lookup
 ******************************************************************************/

const struct geom_entry*
cus3d_geom_store_lookup(const struct cus3d_geom_store* store, uint32_t primID)
{
    if (!store || primID >= store->total_prims)
        return NULL;

    for (size_t i = 0; i < store->entry_count; ++i) {
        const struct geom_entry* e = &store->entries[i];
        if (primID >= e->prim_offset &&
            primID <  e->prim_offset + e->prim_count) {
            return e;
        }
    }
    return NULL;
}

const struct geom_entry*
cus3d_geom_store_get_entry(const struct cus3d_geom_store* store,
                           uint32_t geom_idx)
{
    if (!store || geom_idx >= (uint32_t)store->entry_count)
        return NULL;
    return &store->entries[geom_idx];
}
