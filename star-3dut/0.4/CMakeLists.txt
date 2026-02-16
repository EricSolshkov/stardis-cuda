# star-3dut Library - CMake Build Configuration
# Version: 0.4.0
# Description: 3D utility mesh generation library (sphere, cylinder, cuboid, super shape)
# Dependencies: rsys
# Rewritten: 2026-02-16 (Workspace template)

cmake_minimum_required(VERSION 3.25)
project(star-3dut
    VERSION 0.4.0
    DESCRIPTION "star-3dut: 3D utility mesh generation library"
    LANGUAGES C
)

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

set(S3DUT_SOURCES
    src/s3dut_cuboid.c
    src/s3dut_cylinder.c
    src/s3dut_mesh.c
    src/s3dut_sphere.c
    src/s3dut_super_shape.c
)

# ============================================================================
# Library Target
# ============================================================================

if(BUILD_SHARED_LIBS)
    add_library(s3dut SHARED ${S3DUT_SOURCES})
    target_compile_definitions(s3dut PRIVATE S3DUT_SHARED_BUILD)
else()
    add_library(s3dut STATIC ${S3DUT_SOURCES})
endif()

add_library(s3dut::s3dut ALIAS s3dut)

set_target_properties(s3dut PROPERTIES
    VERSION ${PROJECT_VERSION}
    SOVERSION 0
    OUTPUT_NAME "s3dut"
)

# ============================================================================
# Generate Standard Include Structure (star/ namespace)
# ============================================================================

set(S3DUT_GENERATED_INCLUDE_DIR "${CMAKE_CURRENT_BINARY_DIR}/include")

# Public header to expose under star/ namespace
set(S3DUT_PUBLIC_HEADERS
    src/s3dut.h
)

# Create include/star/ structure in build directory
foreach(header ${S3DUT_PUBLIC_HEADERS})
    get_filename_component(header_name ${header} NAME)
    configure_file(
        ${CMAKE_CURRENT_SOURCE_DIR}/${header}
        ${S3DUT_GENERATED_INCLUDE_DIR}/star/${header_name}
        COPYONLY
    )
endforeach()

# ============================================================================
# Include Directories
# ============================================================================

target_include_directories(s3dut
    PUBLIC
        # Generated structure for <star/s3dut.h>
        $<BUILD_INTERFACE:${S3DUT_GENERATED_INCLUDE_DIR}>
        # Source directory (internal use only)
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
        $<INSTALL_INTERFACE:include>
)

# ============================================================================
# Dependencies
# ============================================================================

target_link_libraries(s3dut PUBLIC rsys)

# Math library (Unix only)
if(UNIX AND NOT APPLE)
    target_link_libraries(s3dut PRIVATE m)
endif()

# ============================================================================
# Source Validation
# ============================================================================

validate_library_sources(
    TARGET s3dut
    SOURCES ${S3DUT_SOURCES}
)

# ============================================================================
# Tests
# ============================================================================

if(ENABLE_TESTS)
    enable_testing()

    set(S3DUT_TEST_SOURCES
        src/test_s3dut_cuboid.c
        src/test_s3dut_cylinder.c
        src/test_s3dut_hemisphere.c
        src/test_s3dut_sphere.c
        src/test_s3dut_super_shape.c
        src/test_s3dut_thick_cylinder.c
        src/test_s3dut_thick_truncated_sphere.c
        src/test_s3dut_thick_truncated_super_shape.c
        src/test_s3dut_thin_cylinder.c
        src/test_s3dut_truncated_sphere.c
    )

    add_module_tests(
        SOURCES ${S3DUT_TEST_SOURCES}
        LIBRARIES s3dut
    )
endif()

# ============================================================================
# Installation
# ============================================================================

# Install library targets
install(TARGETS s3dut
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
)

# Install public headers to include/star/
install(FILES ${S3DUT_PUBLIC_HEADERS}
    DESTINATION include/star
)
