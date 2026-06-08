# local.cmake — project-local CMake, auto-included by trussc_app() and (unlike the
# generated CMakeLists.txt / CMakePresets.json) COMMITTED, so it survives a
# `trusscli update` regeneration.
#
# This example opts into the advanced escape hatch (<tcxPhysicsJolt.h>), which
# pulls in Jolt headers directly. Link the SAME Jolt target the tcxPhysics addon
# built so the headers AND the JPH_* compile defines match — mismatched defines
# silently change struct layouts (ABI breakage). This one line is the entire
# build cost of reaching for the hatch.
if(TARGET Jolt)
    target_link_libraries(${PROJECT_NAME} PRIVATE Jolt)
endif()
