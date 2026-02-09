# custar-3d Library - CMake Build Configuration
# Version: 0.10.0
# Description: 3D ray tracing and geometric operations
# Dependencies: rsys, cuBQL
# Rewritten: 2026-02-07 (fork from star-3d CMakeLists.txt)

cmake_minimum_required(VERSION 3.25)
project(custar-3d
    VERSION 0.10.0
    DESCRIPTION "custar-3d: GPU-accelerated Star-3D with cuBQL (standalone s3d replacement)"
    LANGUAGES C CXX CUDA
)
set(CMAKE_C_STANDARD 99)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CUDA_STANDARD 17)
set(CMAKE_CUDA_ARCHITECTURES "89;89-real;86;86-real")

# ============================================================================
# Load Helper Functions
# ============================================================================

if(NOT COMMAND add_module_tests)
    if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/../../cmake/HelperFunctions.cmake")
        include("${CMAKE_CURRENT_SOURCE_DIR}/../../cmake/HelperFunctions.cmake")
    else()
        message(FATAL_ERROR "Cannot find HelperFunctions.cmake - must build from workspace root")
    endif()
endif()

# ============================================================================
# Options (Standalone Build Support)
# ============================================================================

if(NOT DEFINED ENABLE_TESTS)
    option(ENABLE_TESTS "Enable building tests" ON)
endif()

if(NOT DEFINED BUILD_SHARED_LIBS)
    option(BUILD_SHARED_LIBS "Build shared library" ON)
endif()

# ============================================================================
# Source Files
# ============================================================================

set(S3D_CPP_SOURCES
    src/s3d_device.cpp
    src/s3d_scene.cpp
    src/s3d_scene_view.cpp
    src/s3d_shape.cpp
    src/s3d_mesh.cpp
    src/s3d_sphere.cpp
    src/s3d_instance.cpp
    src/s3d_primitive.cpp
    src/s3d_geometry.cpp
    src/s3d_scene_view_trace_ray.cpp
    src/s3d_scene_view_closest_point.cpp

    # CUDA host-side C++
    src/cus3d_device.cpp
    src/cus3d_mem.cpp
    src/cus3d_geom_store.cpp
    src/cus3d_prim.cpp
)

set(S3D_CUDA_SOURCES
    src/cus3d_geom_store.cu
    src/cus3d_bvh.cu
    src/cus3d_trace.cu
    src/cus3d_math.cu
)

validate_library_sources(S3D_CPP_SOURCES)
validate_library_sources(S3D_CUDA_SOURCES)

file(GLOB S3D_HEADERS CONFIGURE_DEPENDS src/*.h src/*.cuh)

# ============================================================================
# Library Target
# ============================================================================

if(BUILD_SHARED_LIBS)
    add_library(s3d SHARED ${S3D_CPP_SOURCES} ${S3D_CUDA_SOURCES})
    target_compile_definitions(s3d PRIVATE S3D_SHARED_BUILD)
else()
    add_library(s3d STATIC ${S3D_CPP_SOURCES} ${S3D_CUDA_SOURCES})
endif()

add_library(star-3d::s3d ALIAS s3d)

set_target_properties(s3d PROPERTIES
    VERSION ${PROJECT_VERSION}
    SOVERSION 0
    OUTPUT_NAME "s3d"
)

# ============================================================================
# Generate Namespace Headers (star/) - v1.1
# ============================================================================

# Public headers to expose
set(S3D_PUBLIC_HEADERS
    src/s3d.h
)

# v1.1: Use helper function to generate namespace structure
generate_namespace_headers(
    TARGET s3d
    NAMESPACE star
    HEADERS ${S3D_PUBLIC_HEADERS}
)

# ============================================================================
# Include Directories
# ============================================================================

target_include_directories(s3d
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
        $<INSTALL_INTERFACE:include>
)

# ============================================================================
# Dependencies
# ============================================================================

target_link_libraries(s3d PUBLIC rsys)

if(TARGET cubql)
    target_link_libraries(s3d PRIVATE cubql)
    target_compile_definitions(s3d PRIVATE S3D_USE_CUBQL_BACKEND)
    if(TARGET CUDA::cudart)
        target_link_libraries(s3d PRIVATE CUDA::cudart)
    else()
        find_package(CUDAToolkit REQUIRED)
        target_link_libraries(s3d PRIVATE CUDA::cudart)
    endif()
    message(STATUS "custar-3d: cuBQL GPU backend (standalone s3d replacement)")
else()
    message(FATAL_ERROR "cuBQL target not found. Ensure ThirdPartyRegistry.cmake is loaded.")
endif()

set_property(TARGET s3d PROPERTY CXX_STANDARD 17)
set_property(TARGET s3d PROPERTY CXX_STANDARD_REQUIRED ON)
set_property(TARGET s3d PROPERTY CUDA_STANDARD 17)
set_property(TARGET s3d PROPERTY CUDA_STANDARD_REQUIRED ON)

if(UNIX AND NOT APPLE)
    target_link_libraries(s3d PRIVATE m)
endif()

# ============================================================================
# Tests
# ============================================================================

if(ENABLE_TESTS)
    enable_testing()
    file(GLOB S3D_TEST_SOURCES src/test_*.c)
    if(S3D_TEST_SOURCES)
        add_module_tests(
            SOURCES ${S3D_TEST_SOURCES}
            LIBRARIES s3d
        )
    endif()
endif()

set_target_properties(s3d PROPERTIES
    CUDA_SEPARABLE_COMPILATION ON
)

# ============================================================================
# Installation
# ============================================================================

install(TARGETS s3d
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
)

install(FILES ${S3D_PUBLIC_HEADERS}
    DESTINATION include/star
)
