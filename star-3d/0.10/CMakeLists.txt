# star-3d Library - CMake Build Configuration
# Version: 0.10.0
# Description: 3D ray tracing and geometric operations (cuBQL GPU backend)
# Dependencies: rsys, cuBQL, CUDA
# Rewritten: 2026-02-04 (cuBQL migration - no Embree fallback)
# Migration Strategy: Direct function-by-function migration to cuBQL
#   - Keep C API (extern "C") for compatibility
#   - Implement backend in C++/CUDA for cuBQL integration
#   - Remove all Embree dependencies

cmake_minimum_required(VERSION 3.25)
project(star-3d
    VERSION 0.10.0
    DESCRIPTION "star-3d: 3D ray tracing and geometric operations (cuBQL GPU backend)"
    LANGUAGES C CXX CUDA  # Add CXX for C++ backend implementation
)

# Set language standards
set(CMAKE_C_STANDARD 99)        # C API compatibility
set(CMAKE_CXX_STANDARD 17)      # C++ backend implementation
set(CMAKE_CUDA_STANDARD 17)     # CUDA for cuBQL
# Set CUDA architectures for RTX 4090 (sm_89) and compatibility
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

# cuBQL GPU backend is mandatory (no Embree fallback)
set(USE_CUBQL_BACKEND ON CACHE BOOL "Use cuBQL GPU backend (mandatory)" FORCE)
message(STATUS "star-3d: cuBQL GPU backend is mandatory (no Embree fallback)")



# ============================================================================
# Source Files
# ============================================================================

# Library sources (production code only - NO test_*.c files)
set(S3D_SOURCES
    src/s3d_device.cpp
    src/s3d_geometry.cpp
    src/s3d_instance.cpp
    src/s3d_mesh.cpp
    src/s3d_primitive.cpp
    src/s3d_scene.cpp
    src/s3d_scene_view.cpp
    src/s3d_scene_view_closest_point.cpp
    src/s3d_scene_view_trace_ray.cpp
    src/s3d_shape.cpp
    src/s3d_sphere.cpp
)

# v1.1: Validate no test files mixed in
validate_library_sources(S3D_SOURCES)

file(GLOB S3D_HEADERS CONFIGURE_DEPENDS src/*.h)

# ============================================================================
# Library Target
# ============================================================================

if(BUILD_SHARED_LIBS)
    add_library(s3d SHARED ${S3D_SOURCES})
    target_compile_definitions(s3d PRIVATE S3D_SHARED_BUILD)
else()
    add_library(s3d STATIC ${S3D_SOURCES})
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
        # Source directory (internal use only)
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
        $<INSTALL_INTERFACE:include>
)
# Note: Namespace headers auto-configured by generate_namespace_headers()

# ============================================================================
# Dependencies
# ============================================================================

# rsys (internal dependency)
target_link_libraries(s3d PUBLIC rsys)

# cuBQL GPU backend (mandatory, no Embree fallback)
if(TARGET cubql)
    target_link_libraries(s3d PRIVATE cubql)
    target_compile_definitions(s3d PRIVATE S3D_USE_CUBQL_BACKEND)
    # Link CUDA runtime for cuBQL
    if(TARGET CUDA::cudart)
        target_link_libraries(s3d PRIVATE CUDA::cudart)
    else()
        find_package(CUDAToolkit REQUIRED)
        target_link_libraries(s3d PRIVATE CUDA::cudart)
    endif()
    message(STATUS "star-3d: Using cuBQL GPU backend (mandatory)")
else()
    message(FATAL_ERROR "cuBQL target not found. Ensure ThirdPartyRegistry.cmake is loaded. Embree fallback is not available.")
endif()

# Enable C++ compilation for backend implementation files
# This allows mixing C and C++ source files in the same library
set_property(TARGET s3d PROPERTY CXX_STANDARD 17)
set_property(TARGET s3d PROPERTY CXX_STANDARD_REQUIRED ON)

# Math library (Unix only)
if(UNIX AND NOT APPLE)
    target_link_libraries(s3d PRIVATE m)
endif()

# ============================================================================
# Tests
# ============================================================================

if(ENABLE_TESTS)
    enable_testing()
    
    # Test sources discovered from src/test_*.c pattern
    file(GLOB S3D_TEST_SOURCES src/test_*.c)
    
    if(S3D_TEST_SOURCES)
        add_module_tests(
            SOURCES ${S3D_TEST_SOURCES}
            LIBRARIES s3d
        )
    endif()
endif()

# ============================================================================
# Installation
# ============================================================================

# Install library targets
install(TARGETS s3d
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
)

# Install public headers to include/star/
install(FILES ${S3D_PUBLIC_HEADERS}
    DESTINATION include/star
)
