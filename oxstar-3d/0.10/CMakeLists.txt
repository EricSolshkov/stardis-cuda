# oxstar-3d Library - CMake Build Configuration
# Version: 0.10.0
# Description: 3D ray tracing and geometric operations (OptiX backend)
# Dependencies: rsys, OptiX SDK, CUDA Toolkit
# Created: 2026-02-23 (fork from custar-3d CMakeLists.txt, OptiX adaptation)

cmake_minimum_required(VERSION 3.25)
project(oxstar-3d
    VERSION 0.10.0
    DESCRIPTION "oxstar-3d: GPU-accelerated Star-3D with OptiX (standalone s3d replacement)"
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
# OptiX SDK
# ============================================================================

set(OptiX_INSTALL_DIR "C:/ProgramData/NVIDIA Corporation/OptiX SDK 9.1.0"
    CACHE PATH "Path to OptiX SDK installation")
set(OptiX_INCLUDE_DIR "${OptiX_INSTALL_DIR}/include")

if(NOT EXISTS "${OptiX_INCLUDE_DIR}/optix.h")
    message(FATAL_ERROR
        "OptiX SDK not found at: ${OptiX_INSTALL_DIR}\n"
        "Set -DOptiX_INSTALL_DIR=<path> to the OptiX SDK root directory.")
endif()
message(STATUS "oxstar-3d: OptiX SDK found: ${OptiX_INSTALL_DIR}")

# ============================================================================
# CUDA Toolkit
# ============================================================================

find_package(CUDAToolkit REQUIRED)
message(STATUS "oxstar-3d: CUDA Toolkit: ${CUDAToolkit_VERSION}")

# ============================================================================
# Find nvcc for PTX compilation
# ============================================================================

if(CUDAToolkit_NVCC_EXECUTABLE)
    set(NVCC_CMD "${CUDAToolkit_NVCC_EXECUTABLE}")
else()
    find_program(NVCC_CMD nvcc HINTS "${CUDAToolkit_BIN_DIR}")
endif()

if(NOT NVCC_CMD)
    message(FATAL_ERROR "nvcc not found. Make sure CUDA Toolkit is installed.")
endif()

# ============================================================================
# Compile OptiX Device Programs to PTX
# ============================================================================

# Ray tracing device programs
set(PTX_SOURCE "${CMAKE_CURRENT_SOURCE_DIR}/device/programs.cu")
set(PTX_OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/programs.ptx")

add_custom_command(
    OUTPUT "${PTX_OUTPUT}"
    COMMAND "${NVCC_CMD}"
        --ptx
        -I"${OptiX_INCLUDE_DIR}"
        -I"${CMAKE_CURRENT_SOURCE_DIR}/include"
        --machine=64
        -arch=compute_70
        --use_fast_math
        -std=c++17
        -D__CUDA_INCLUDE_COMPILER_INTERNAL_HEADERS__
        "${PTX_SOURCE}"
        -o "${PTX_OUTPUT}"
    DEPENDS
        "${PTX_SOURCE}"
        "${CMAKE_CURRENT_SOURCE_DIR}/include/unified_params.h"
        "${CMAKE_CURRENT_SOURCE_DIR}/include/ray_types.h"
        "${CMAKE_CURRENT_SOURCE_DIR}/include/nn_types.h"
    COMMENT "oxstar-3d: Compiling OptiX device programs to PTX (compute_70)"
    VERBATIM
)

# Nearest-neighbor device programs
set(NN_PTX_SOURCE "${CMAKE_CURRENT_SOURCE_DIR}/device/nn_programs.cu")
set(NN_PTX_OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/nn_programs.ptx")

add_custom_command(
    OUTPUT "${NN_PTX_OUTPUT}"
    COMMAND "${NVCC_CMD}"
        --ptx
        -I"${OptiX_INCLUDE_DIR}"
        -I"${CMAKE_CURRENT_SOURCE_DIR}/include"
        --machine=64
        -arch=compute_70
        --use_fast_math
        -std=c++17
        -D__CUDA_INCLUDE_COMPILER_INTERNAL_HEADERS__
        "${NN_PTX_SOURCE}"
        -o "${NN_PTX_OUTPUT}"
    DEPENDS
        "${NN_PTX_SOURCE}"
        "${CMAKE_CURRENT_SOURCE_DIR}/include/unified_params.h"
        "${CMAKE_CURRENT_SOURCE_DIR}/include/ray_types.h"
        "${CMAKE_CURRENT_SOURCE_DIR}/include/nn_types.h"
    COMMENT "oxstar-3d: Compiling NN device programs to PTX (compute_70)"
    VERBATIM
)

add_custom_target(oxs3d_ptx_programs ALL DEPENDS "${PTX_OUTPUT}" "${NN_PTX_OUTPUT}")

# ============================================================================
# Source Files
# ============================================================================

# s3d wrapper layer (public API, compatible with custar-3d)
set(S3D_WRAPPER_SOURCES
    s3d_wrapper/ox_s3d_device.cpp
    s3d_wrapper/ox_s3d_shape.cpp
    s3d_wrapper/ox_s3d_scene.cpp
    s3d_wrapper/ox_s3d_scene_view.cpp
    s3d_wrapper/ox_s3d_primitive.cpp
)

# OptiX host-side C++ (device/pipeline/geometry management)
set(S3D_CPP_SOURCES
    src/device_manager.cpp
    src/geometry_manager.cpp
    src/unified_tracer.cpp
    src/accel_manager.cpp
    src/pipeline_manager.cpp
    src/ray_tracer.cpp
    src/nn_query.cpp
)

# Regular CUDA kernels (non-OptiX)
set(S3D_CUDA_SOURCES
    device/kernels.cu
    device/nn_kernels.cu
)

validate_library_sources(S3D_WRAPPER_SOURCES)
validate_library_sources(S3D_CPP_SOURCES)
validate_library_sources(S3D_CUDA_SOURCES)

# ============================================================================
# Library Target
# ============================================================================

if(BUILD_SHARED_LIBS)
    add_library(s3d SHARED
        ${S3D_WRAPPER_SOURCES}
        ${S3D_CPP_SOURCES}
        ${S3D_CUDA_SOURCES}
    )
    target_compile_definitions(s3d PRIVATE S3D_SHARED_BUILD)
else()
    add_library(s3d STATIC
        ${S3D_WRAPPER_SOURCES}
        ${S3D_CPP_SOURCES}
        ${S3D_CUDA_SOURCES}
    )
endif()

add_library(star-3d::s3d ALIAS s3d)

set_target_properties(s3d PROPERTIES
    VERSION ${PROJECT_VERSION}
    SOVERSION 0
    OUTPUT_NAME "s3d"
)

# Ensure PTX is built before the library
add_dependencies(s3d oxs3d_ptx_programs)

# ============================================================================
# Generate Namespace Headers (star/) - v1.1
# ============================================================================

set(S3D_PUBLIC_HEADERS
    s3d_wrapper/s3d.h
)

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
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/s3d_wrapper>
        $<INSTALL_INTERFACE:include>
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${CMAKE_CURRENT_SOURCE_DIR}/device
        ${OptiX_INCLUDE_DIR}
)

# ============================================================================
# PTX File Path Definitions
# ============================================================================

string(REPLACE "\\" "/" PTX_PATH_FWD "${PTX_OUTPUT}")
string(REPLACE "\\" "/" NN_PTX_PATH_FWD "${NN_PTX_OUTPUT}")
target_compile_definitions(s3d PRIVATE
    OPTIX_PTX_FILE_PATH="${PTX_PATH_FWD}"
    OPTIX_NN_PTX_FILE_PATH="${NN_PTX_PATH_FWD}"
)

# ============================================================================
# Dependencies
# ============================================================================

target_link_libraries(s3d
    PUBLIC rsys
    PRIVATE CUDA::cudart
)

# OpenMP (optional — CPU post-process parallelization)
find_package(OpenMP)
if(OpenMP_CXX_FOUND)
    target_link_libraries(s3d PRIVATE OpenMP::OpenMP_CXX)
    # MSVC: ensure /openmp flag propagates via MSBuild properties
    set_source_files_properties(
        s3d_wrapper/ox_s3d_scene_view.cpp
        PROPERTIES COMPILE_OPTIONS "${OpenMP_CXX_FLAGS}"
    )
    message(STATUS "oxstar-3d: OpenMP enabled (flags: ${OpenMP_CXX_FLAGS})")
endif()

target_compile_definitions(s3d PRIVATE S3D_USE_OPTIX_BACKEND)

set_property(TARGET s3d PROPERTY CXX_STANDARD 17)
set_property(TARGET s3d PROPERTY CXX_STANDARD_REQUIRED ON)
set_property(TARGET s3d PROPERTY CUDA_STANDARD 17)
set_property(TARGET s3d PROPERTY CUDA_STANDARD_REQUIRED ON)
set_property(TARGET s3d PROPERTY CUDA_SEPARABLE_COMPILATION ON)

if(UNIX AND NOT APPLE)
    target_link_libraries(s3d PRIVATE m)
endif()

message(STATUS "oxstar-3d: OptiX GPU backend (standalone s3d replacement)")

# ============================================================================
# Deploy PTX files alongside library binary
# ============================================================================

add_custom_command(TARGET s3d POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${PTX_OUTPUT}"
        "$<TARGET_FILE_DIR:s3d>/programs.ptx"
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${NN_PTX_OUTPUT}"
        "$<TARGET_FILE_DIR:s3d>/nn_programs.ptx"
    COMMENT "oxstar-3d: Copying PTX files to output directory"
)

# ============================================================================
# Throughput Benchmark Executable (src/main.cpp)
# ============================================================================

add_executable(oxs3d_throughput
    src/main.cpp
    src/device_manager.cpp
    src/geometry_manager.cpp
    src/unified_tracer.cpp
    src/accel_manager.cpp
    src/pipeline_manager.cpp
    src/ray_tracer.cpp
    src/nn_query.cpp
    device/kernels.cu
    device/nn_kernels.cu
)

target_include_directories(oxs3d_throughput PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}/device
    ${OptiX_INCLUDE_DIR}
)

target_link_libraries(oxs3d_throughput PRIVATE
    rsys
    CUDA::cudart
)

set_target_properties(oxs3d_throughput PROPERTIES
    CUDA_ARCHITECTURES "${CMAKE_CUDA_ARCHITECTURES}"
    CXX_STANDARD 17
)

add_dependencies(oxs3d_throughput oxs3d_ptx_programs)

# Deploy PTX files alongside the benchmark executable
add_custom_command(TARGET oxs3d_throughput POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${PTX_OUTPUT}"
        "$<TARGET_FILE_DIR:oxs3d_throughput>/programs.ptx"
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${NN_PTX_OUTPUT}"
        "$<TARGET_FILE_DIR:oxs3d_throughput>/nn_programs.ptx"
    COMMENT "oxstar-3d: Copying PTX files for throughput benchmark"
)

# ============================================================================
# Tests
# ============================================================================

if(ENABLE_TESTS)
    enable_testing()

    set(S3D_TEST_DIR "${CMAKE_CURRENT_SOURCE_DIR}/tests")

    set(S3D_TEST_SOURCES
        test_s3d_accel_struct_conf.c
        test_s3d_batch_closest_point.c
        test_s3d_batch_trace.c
        test_s3d_closest_point.c
        test_s3d_device.c
        test_s3d_primitive.c
        test_s3d_sampler.c
        test_s3d_sample_sphere.c
        test_s3d_scene.c
        test_s3d_scene_view.c
        test_s3d_scene_view_aabb.c
        test_s3d_seams.c
        test_s3d_shape.c
        test_s3d_sphere.c
        test_s3d_sphere_box.c
        test_s3d_sphere_instance.c
        test_s3d_trace_ray.c
        test_s3d_trace_ray_instance.c
        test_s3d_trace_ray_sphere.c
    )

    foreach(test_src ${S3D_TEST_SOURCES})
        get_filename_component(test_name ${test_src} NAME_WE)
        add_executable(${test_name} ${S3D_TEST_DIR}/${test_src})
        target_include_directories(${test_name} PRIVATE
            "${CMAKE_CURRENT_SOURCE_DIR}/s3d_wrapper"
            "${S3D_TEST_DIR}"
        )
        target_link_libraries(${test_name} PRIVATE s3d)
        add_dependencies(${test_name} oxs3d_ptx_programs)

        # Windows: deploy runtime DLLs
        if(WIN32 AND CMAKE_VERSION VERSION_GREATER_EQUAL "3.21")
            add_custom_command(TARGET ${test_name} POST_BUILD
                COMMAND ${CMAKE_COMMAND} -E copy_if_different
                    $<TARGET_RUNTIME_DLLS:${test_name}>
                    $<TARGET_FILE_DIR:${test_name}>
                COMMAND_EXPAND_LISTS
                COMMENT "Deploying runtime DLLs for ${test_name}"
            )
        endif()

        # Deploy PTX files to test executable directory
        add_custom_command(TARGET ${test_name} POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
                "${PTX_OUTPUT}"
                "$<TARGET_FILE_DIR:${test_name}>/programs.ptx"
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
                "${NN_PTX_OUTPUT}"
                "$<TARGET_FILE_DIR:${test_name}>/nn_programs.ptx"
            COMMENT "Deploying PTX files for ${test_name}"
        )

        add_test(NAME ${test_name} COMMAND ${test_name}
            WORKING_DIRECTORY "$<TARGET_FILE_DIR:${test_name}>")
    endforeach()
endif()

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

# Install PTX files alongside the library
install(FILES
    "${PTX_OUTPUT}"
    "${NN_PTX_OUTPUT}"
    DESTINATION bin
)

# ============================================================================
# Configuration Summary
# ============================================================================

message(STATUS "")
message(STATUS "=== oxstar-3d Configuration ===")
message(STATUS "  Backend:        OptiX ${OptiX_INSTALL_DIR}")
message(STATUS "  CUDA Toolkit:   ${CUDAToolkit_VERSION}")
message(STATUS "  PTX Arch:       compute_70 (JIT for actual GPU)")
message(STATUS "  PTX Targets:    programs.ptx, nn_programs.ptx")
message(STATUS "  Public API:     s3d_wrapper/s3d.h")
message(STATUS "  Tests:          ${CMAKE_CURRENT_SOURCE_DIR}/tests/")
message(STATUS "===============================")
