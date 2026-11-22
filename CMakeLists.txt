cmake_minimum_required(VERSION 3.20)
project(AabbPhysics LANGUAGES CXX)

option(BUILD_GUI "Build SFML visualization (OFF keeps headless builds offline)" ON)

add_library(physics_core STATIC
    config.h config.cpp
    physics.h physics.cpp
    scenes.h scenes.cpp
    benchmark.h benchmark.cpp)
target_include_directories(physics_core PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")
target_compile_features(physics_core PUBLIC cxx_std_17)
set_target_properties(physics_core PROPERTIES CXX_EXTENSIONS OFF)
if(MSVC)
    target_compile_options(physics_core PRIVATE /W4 /permissive- /utf-8)
else()
    target_compile_options(physics_core PRIVATE -Wall -Wextra -Wpedantic)
endif()

include(CTest)
if(BUILD_TESTING)
    add_executable(tests_headless tests/tests_headless.cpp)
    target_link_libraries(tests_headless PRIVATE physics_core)
    foreach(case wall ccd multirate combined_fast_normal scheduler determinism stack config)
        add_test(NAME ${case} COMMAND tests_headless ${case})
    endforeach()
endif()

add_executable(benchmark_headless benchmark_main.cpp)
target_link_libraries(benchmark_headless PRIVATE physics_core)

add_library(demo_state STATIC demo_state.h demo_state.cpp)
target_link_libraries(demo_state PUBLIC physics_core)
if(BUILD_TESTING)
    add_executable(tests_stage2 tests/tests_stage2.cpp)
    target_link_libraries(tests_stage2 PRIVATE demo_state)
    add_test(NAME stage2 COMMAND tests_stage2)
    add_test(NAME benchmark_cli COMMAND benchmark_headless)
endif()

if(BUILD_GUI)
    include(FetchContent)
    set(BUILD_SHARED_LIBS OFF CACHE BOOL "Build static dependencies" FORCE)
    set(SFML_BUILD_AUDIO OFF CACHE BOOL "" FORCE)
    set(SFML_BUILD_NETWORK OFF CACHE BOOL "" FORCE)
    set(SFML_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
    set(SFML_BUILD_DOC OFF CACHE BOOL "" FORCE)
    set(SFML_BUILD_TEST_SUITE OFF CACHE BOOL "" FORCE)
    FetchContent_Declare(sfml
        URL https://github.com/SFML/SFML/archive/refs/tags/2.6.2.zip
        URL_HASH SHA256=19d6dbd9c901c74441d9888c13cb1399f614fe8993d59062a72cfbceb00fed04
        DOWNLOAD_EXTRACT_TIMESTAMP TRUE)
    FetchContent_MakeAvailable(sfml)

    # Console subsystem is intentional: B and --benchmark print the same report.
    add_executable(aabb_demo main.cpp)
    target_link_libraries(aabb_demo PRIVATE demo_state sfml-graphics sfml-window sfml-system)
    if(MSVC)
        target_compile_options(aabb_demo PRIVATE /W4 /permissive- /utf-8)
    endif()
    if(BUILD_TESTING)
        add_test(NAME gui_benchmark_cli COMMAND aabb_demo --benchmark)
    endif()
endif()
