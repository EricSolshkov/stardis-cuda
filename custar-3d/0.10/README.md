# CuStar-3D v0.10

GPU-accelerated 3D ray tracing library using NVIDIA cuBQL for STARDIS thermal solver.

## Overview

CuStar-3D is a GPU-accelerated implementation of the Star-3D ray tracing API, designed to replace Embree-based CPU ray tracing with cuBQL-based GPU acceleration for the STARDIS Monte Carlo thermal solver.

**Status**: Skeleton implementation - Headers and empty function stubs created, awaiting implementation.

## Architecture

Based on the module-wise architecture design documented in `guide/cus3d/cubql-module-architecture.md`, CuStar-3D consists of 8 core GPU modules:

### Core Modules

1. **cus3d_device** - CUDA context and device management
   - Replaces Embree's RTCDevice
   - Manages CUDA streams and device properties

2. **cus3d_mem** - GPU memory manager
   - Typed GPU buffer allocation
   - Host-to-Device / Device-to-Host transfers
   - Buffer resize and lifecycle management

3. **cus3d_types** - Common type definitions
   - Primitive types (triangle, sphere)
   - Build quality settings
   - Hit results and geometry metadata

4. **cus3d_geom_store** - Geometry data store
   - Flattened GPU arrays for all scene primitives
   - Replaces per-geometry RTCGeometry model
   - Unified primitive table for triangles and spheres

5. **cus3d_bvh** - BVH manager using cuBQL
   - Single BinaryBVH for all primitives
   - Two-level traversal for instancing
   - Configurable build quality (LOW/MEDIUM/HIGH)

6. **cus3d_trace** - Ray tracing engine
   - Batch ray tracing on GPU
   - Mixed geometry intersection (triangles + spheres)
   - Hit filter support

7. **cus3d_prim** - Primitive and attribute query
   - Convert GPU hit results to s3d_primitive
   - CPU-side attribute interpolation

8. **cus3d_math** - Device math helpers
   - Ray-primitive intersection algorithms
   - Transform utilities
   - Vector operations

## File Structure

```
custar-3d/0.10/
├── CMakeLists.txt          # Project build configuration
├── README.md               # This file
└── src/
    ├── cus3d_device.h/.cpp      # CUDA device management
    ├── cus3d_mem.h/.cpp          # GPU memory management
    ├── cus3d_types.h             # Type definitions
    ├── cus3d_geom_store.h/.cpp/.cu  # Geometry storage
    ├── cus3d_bvh.h/.cpp          # BVH management
    ├── cus3d_trace.h/.cu         # Ray tracing kernels
    ├── cus3d_prim.h/.cpp         # Hit/primitive conversion
    └── cus3d_math.cuh/.cu        # Device math utilities
```

## Dependencies

- **CUDA Toolkit** 12.6+ (for CUDA runtime and compilation)
- **cuBQL** (header-only, included in thirdparty/)
- **rsys** v0.15+ (base runtime system)
- **RTX 4090** or compatible GPU with Compute Capability 8.9+

## Build Requirements

- CMake 3.25+
- CUDA-capable compiler (nvcc)
- C++17 support
- CUDA architecture: sm_89 (RTX 4090), sm_86 (RTX 3090)

## Building

From stardis-cus3d workspace root:

```bash
mkdir build && cd build
cmake ..
cmake --build . --target custar3d
```

## Integration

CuStar-3D is integrated into the stardis-cus3d workspace at Layer 2 (Geometry & Sampling):

```
Layer 1: rsys (base runtime)
Layer 2: star-2d, star-3d, custar-3d, star-sp
Layer 3: star-enclosures-2d, star-enclosures-3d
...
```

## Design Philosophy

- **No RTCGeometry equivalent**: All primitives flattened into contiguous GPU arrays
- **No per-geometry BVH**: Single BinaryBVH covers all scene primitives
- **Batch-first ray tracing**: GPU kernels process ray batches, not single rays
- **CPU host control**: Public API remains CPU-side C interface

## Current Status

**Phase: Skeleton Complete**

✅ All header files created with API declarations
✅ All implementation files created with empty stubs
✅ CMake build system configured
✅ Workspace integration complete

⏳ Awaiting implementation of:
- CUDA device initialization
- GPU memory management
- Geometry flattening and upload
- BVH construction with cuBQL
- Ray tracing kernels
- Intersection algorithms
- Hit filtering

## Next Steps

1. Implement `cus3d_device` - CUDA context setup
2. Implement `cus3d_mem` - GPU buffer management
3. Implement `cus3d_geom_store` - Geometry flattening and bounds computation
4. Implement `cus3d_bvh` - cuBQL BVH construction
5. Implement `cus3d_math` - Intersection algorithms
6. Implement `cus3d_trace` - Ray tracing kernels
7. Implement `cus3d_prim` - Hit result conversion
8. Integration testing with existing star-3d API

## References

- Architecture design: `guide/cus3d/cubql-module-architecture.md`
- cuBQL API: `thirdparty/cuBQL/`
- Original star-3d: `star-3d/0.10/`

## License

Copyright (C) 2026 |Méso|Star> (contact@meso-star.com)

CuStar-3D is free software released under GPLv3+ license.
