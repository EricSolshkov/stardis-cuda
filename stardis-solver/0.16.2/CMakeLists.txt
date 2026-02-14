# stardis-solver Library - CMake Build Configuration
# Version: 0.16.2
# Description: Monte Carlo radiation transfer solver
# Dependencies: rsys, star-2d, star-3d, star-enclosures-2d, star-enclosures-3d, star-sp, star-wf, OpenMP
# Note: MPI support disabled for Windows migration
# Rewritten: 2026-01-21 (Workspace template)

cmake_minimum_required(VERSION 3.25)
project(stardis-solver
    VERSION 0.16.2
    DESCRIPTION "stardis-solver: Monte Carlo radiation transfer solver"
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
# Source Files (MPI sources excluded)
# ============================================================================

set(SDIS_SOURCES
    src/sdis.c
    src/sdis_brdf.c
    src/sdis_camera.c
    src/sdis_data.c
    src/sdis_device.c
    src/sdis_estimator.c
    src/sdis_estimator_buffer.c
    src/sdis_green.c
    src/sdis_heat_path.c
    src/sdis_heat_path_boundary.c
    src/sdis_heat_path_conductive.c
    src/sdis_interface.c
    src/sdis_log.c
    src/sdis_medium.c
    src/sdis_misc.c
    src/sdis_primkey.c
    src/sdis_radiative_env.c
    src/sdis_realisation.c
    src/sdis_scene.c
    src/sdis_solve.c
    src/sdis_solve_camera.c
    src/sdis_solve_wavefront.c
    src/sdis_wf_steps.c
    src/sdis_solve_persistent_wavefront.c
    src/sdis_source.c
    src/sdis_tile.c
)

# ============================================================================
# Library Target
# ============================================================================

# OBJECT library: compile sources once, link into both DLL and test targets.
# Tests link sdis_obj directly (all symbols visible, no DLL export needed).
add_library(sdis_obj OBJECT ${SDIS_SOURCES})
target_include_directories(sdis_obj
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
        $<INSTALL_INTERFACE:include>
)

# ============================================================================
# Dependencies (on the OBJECT lib so both sdis and tests inherit them)
# ============================================================================

find_package(OpenMP)

target_link_libraries(sdis_obj
    PUBLIC
        rsys
        s2d
        s3d
        senc2d
        senc3d
        ssp
        $<$<BOOL:${OpenMP_C_FOUND}>:OpenMP::OpenMP_C>
)

# Optional: star-wf
if(TARGET swf)
    target_link_libraries(sdis_obj PRIVATE swf)
endif()

# Math library (Unix only)
if(UNIX AND NOT APPLE)
    target_link_libraries(sdis_obj PRIVATE m)
endif()

# ============================================================================
# Shared / Static library (links the OBJECT lib)
# ============================================================================

# When building a DLL, EXPORT_SYM must expand to __declspec(dllexport)
# during compilation — so the define goes on sdis_obj, not sdis.
# For test EXEs the dllexport attribute is harmlessly ignored by the linker.
#
# When building STATIC, SDIS_STATIC_BUILD is PUBLIC so that consumers
# (stardis, tests) also see it and get  SDIS_API = extern LOCAL_SYM
# instead of extern __declspec(dllimport).
if(BUILD_SHARED_LIBS)
    target_compile_definitions(sdis_obj PRIVATE SDIS_SHARED_BUILD)
    add_library(sdis SHARED)
else()
    target_compile_definitions(sdis_obj PUBLIC  SDIS_STATIC_BUILD)
    add_library(sdis STATIC)
endif()

target_link_libraries(sdis PUBLIC sdis_obj)

add_library(sdis::sdis ALIAS sdis)

set_target_properties(sdis PROPERTIES
    VERSION ${PROJECT_VERSION}
    SOVERSION 0
    OUTPUT_NAME "sdis"
)

# ============================================================================
# Tests
# ============================================================================

if(ENABLE_TESTS)
    enable_testing()
    
    # Test utility source (shared by all tests)
    add_library(test_sdis_utils OBJECT src/test_sdis_utils.c)
    target_link_libraries(test_sdis_utils PUBLIC sdis rsys)
    
    # Regular tests
    set(SDIS_REGULAR_TESTS
        src/test_sdis_camera.c
        src/test_sdis_conducto_radiative.c
        src/test_sdis_conducto_radiative_2d.c
        src/test_sdis_contact_resistance.c
        src/test_sdis_contact_resistance_2.c
        src/test_sdis_convection.c
        src/test_sdis_convection_non_uniform.c
        src/test_sdis_data.c
        src/test_sdis_enclosure_limit_conditions.c
        src/test_sdis_flux.c
        src/test_sdis_flux2.c
        src/test_sdis_flux_with_h.c
        src/test_sdis_interface.c
        src/test_sdis_medium.c
        src/test_sdis_picard.c
        src/test_sdis_radiative_env.c
        src/test_sdis_solve_probe.c
        src/test_sdis_solve_probe_2d.c
        src/test_sdis_solve_probe2_2d.c
        src/test_sdis_solve_probe3_2d.c
        src/test_sdis_source.c
        src/test_sdis_transcient.c
        src/test_sdis_unsteady.c
        src/test_sdis_unsteady_1d.c
        src/test_sdis_unsteady_analytic_profile_2d.c
        src/test_sdis_unsteady_atm.c
        src/test_sdis_volumic_power.c
        src/test_sdis_volumic_power4.c
    )
    
    add_module_tests(
        SOURCES ${SDIS_REGULAR_TESTS}
        LIBRARIES test_sdis_utils sdis
    )
    
    # Long-running tests (with timeout)
    set(SDIS_LONG_TESTS
        src/test_sdis_volumic_power2.c
        src/test_sdis_volumic_power2_2d.c
        src/test_sdis_volumic_power3_2d.c
    #   src/test_sdis_wavefront_benchmark.c    disabled because unresolved symbols
    )
    
    foreach(test_src ${SDIS_LONG_TESTS})
        get_filename_component(test_name ${test_src} NAME_WE)
        add_executable(${test_name} ${test_src})
        target_link_libraries(${test_name} PRIVATE test_sdis_utils sdis)
        add_test(NAME ${test_name} COMMAND ${test_name})
        set_tests_properties(${test_name} PROPERTIES TIMEOUT 600)
    endforeach()

    # ---- B-4 tests link the OBJECT library directly ----
    # sdis_obj gives direct access to all symbols (LOCAL_SYM included)
    # without needing WINDOWS_EXPORT_ALL_SYMBOLS on the DLL.

    # M1 enclosure batch test (disabled: M1 6-ray ENC replaced by M10 BVH locate)
    # add_executable(test_sdis_b4_m1_enclosure_batch
    #     src/test_sdis_b4_m1_enclosure_batch.c)
    # target_link_libraries(test_sdis_b4_m1_enclosure_batch PRIVATE sdis_obj)
    # add_test(NAME test_sdis_b4_m1_enclosure_batch
    #          COMMAND test_sdis_b4_m1_enclosure_batch)

    add_executable(test_sdis_b4_m2_ray_bucketing
        src/test_sdis_b4_m2_ray_bucketing.c)
    target_link_libraries(test_sdis_b4_m2_ray_bucketing PRIVATE sdis_obj)
    add_test(NAME test_sdis_b4_m2_ray_bucketing
             COMMAND test_sdis_b4_m2_ray_bucketing)

    add_executable(test_sdis_b4_m3_solid_solid
        src/test_sdis_b4_m3_solid_solid.c)
    target_link_libraries(test_sdis_b4_m3_solid_solid PRIVATE sdis_obj)
    add_test(NAME test_sdis_b4_m3_solid_solid
             COMMAND test_sdis_b4_m3_solid_solid)

    add_executable(test_sdis_b4_m4_delta_sphere
        src/test_sdis_b4_m4_delta_sphere.c)
    target_link_libraries(test_sdis_b4_m4_delta_sphere PRIVATE sdis_obj)
    add_test(NAME test_sdis_b4_m4_delta_sphere
             COMMAND test_sdis_b4_m4_delta_sphere)

    add_executable(test_sdis_b4_m6_convective
        src/test_sdis_b4_m6_convective.c)
    target_link_libraries(test_sdis_b4_m6_convective PRIVATE sdis_obj)
    add_test(NAME test_sdis_b4_m6_convective
             COMMAND test_sdis_b4_m6_convective)

    add_executable(test_sdis_b4_m5_picard1
        src/test_sdis_b4_m5_picard1.c)
    target_link_libraries(test_sdis_b4_m5_picard1 PRIVATE sdis_obj)
    add_test(NAME test_sdis_b4_m5_picard1
             COMMAND test_sdis_b4_m5_picard1)

    add_executable(test_sdis_b4_m7_external_flux
        src/test_sdis_b4_m7_external_flux.c)
    target_link_libraries(test_sdis_b4_m7_external_flux PRIVATE sdis_obj)
    add_test(NAME test_sdis_b4_m7_external_flux
             COMMAND test_sdis_b4_m7_external_flux)

    add_executable(test_sdis_b4_m8_picardN
        src/test_sdis_b4_m8_picardN.c)
    target_link_libraries(test_sdis_b4_m8_picardN PRIVATE sdis_obj)
    add_test(NAME test_sdis_b4_m8_picardN
             COMMAND test_sdis_b4_m8_picardN)

    # Deploy runtime DLLs (Embree4, TBB, etc.) for all B4 test executables
    if(WIN32 AND COMMAND deploy_runtime_dependencies)
        # deploy_runtime_dependencies(test_sdis_b4_m1_enclosure_batch)
        deploy_runtime_dependencies(test_sdis_b4_m2_ray_bucketing)
        deploy_runtime_dependencies(test_sdis_b4_m3_solid_solid)
        deploy_runtime_dependencies(test_sdis_b4_m4_delta_sphere)
        deploy_runtime_dependencies(test_sdis_b4_m6_convective)
        deploy_runtime_dependencies(test_sdis_b4_m5_picard1)
        deploy_runtime_dependencies(test_sdis_b4_m7_external_flux)
        deploy_runtime_dependencies(test_sdis_b4_m8_picardN)
    endif()

endif()

# ============================================================================
# Installation
# ============================================================================

install(TARGETS sdis
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    PUBLIC_HEADER DESTINATION include
)
