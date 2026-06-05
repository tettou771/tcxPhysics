# =============================================================================
# tcxPhysics - TrussC 3D physics addon (Jolt Physics)
# =============================================================================

cmake_minimum_required(VERSION 3.16)

set(ADDON_NAME tcxPhysics)

# --- Fetch Jolt Physics ------------------------------------------------------
# Jolt's CMakeLists lives in the Build/ subdirectory, so we point FetchContent
# at it with SOURCE_SUBDIR. Disable every sample/test/tool target so only the
# `Jolt` library is built.
include(FetchContent)
FetchContent_Declare(
    JoltPhysics
    GIT_REPOSITORY https://github.com/jrouwe/JoltPhysics.git
    GIT_TAG v5.5.0
    SOURCE_SUBDIR Build
)

set(TARGET_UNIT_TESTS       OFF CACHE BOOL "" FORCE)
set(TARGET_HELLO_WORLD      OFF CACHE BOOL "" FORCE)
set(TARGET_PERFORMANCE_TEST OFF CACHE BOOL "" FORCE)
set(TARGET_SAMPLES          OFF CACHE BOOL "" FORCE)
set(TARGET_VIEWER           OFF CACHE BOOL "" FORCE)
set(ENABLE_ALL_WARNINGS     OFF CACHE BOOL "" FORCE)
# Don't let Jolt stomp on the global CXX flags TrussC already set; build it with
# the toolchain's normal flags (RTTI/exceptions on) so it integrates cleanly.
set(OVERRIDE_CXX_FLAGS      OFF CACHE BOOL "" FORCE)
# MSVC: Jolt defaults to the STATIC runtime (/MT) while TrussC + the app use the
# DYNAMIC runtime (/MD). Mismatched CRTs cause LNK2038 "RuntimeLibrary mismatch".
# Force Jolt onto the dynamic runtime to match.
set(USE_STATIC_MSVC_RUNTIME_LIBRARY OFF CACHE BOOL "" FORCE)
# Emscripten: build Jolt with wasm SIMD (128-bit). This is where most of Jolt's
# native-class throughput on the web comes from — without it the math drops to
# scalar and roughly halves.
if(EMSCRIPTEN)
    set(USE_WASM_SIMD       ON  CACHE BOOL "" FORCE)
endif()

FetchContent_MakeAvailable(JoltPhysics)

# --- Collect this addon's sources --------------------------------------------
file(GLOB ADDON_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)
file(GLOB ADDON_HEADERS ${CMAKE_CURRENT_SOURCE_DIR}/src/*.h)

add_library(${ADDON_NAME} STATIC ${ADDON_SOURCES} ${ADDON_HEADERS})

target_include_directories(${ADDON_NAME} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src)

# TrussC is the public dependency. Jolt is PRIVATE: it is only included from
# this addon's .cpp files (the public headers are Jolt-free via PIMPL), so apps
# that consume tcxPhysics never see Jolt's headers or its ABI-defines.
target_link_libraries(${ADDON_NAME} PUBLIC TrussC)
target_link_libraries(${ADDON_NAME} PRIVATE Jolt)
