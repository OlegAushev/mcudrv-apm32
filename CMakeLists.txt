file(GLOB_RECURSE sources CONFIGURE_DEPENDS
    ${CMAKE_CURRENT_SOURCE_DIR}/apm32/*.cpp)

# OBJECT (not STATIC): consumers link in every .obj rather than letting the
# linker cherry-pick by demand. Translation units like
# `apm32/f4/can/can_irq_handlers.cpp` only export symbols referenced by the
# vector table in startup.S; in a STATIC archive those .obj are skipped
# because the weak Default_Handler aliases from startup.S already satisfy the
# vector table. With OBJECT every .obj reaches the executable, the strong
# handlers override the weak aliases, and --gc-sections (set in the toolchain
# file) removes everything that is genuinely unused.
add_library(mcudrv_apm32 OBJECT ${sources})

# PUBLIC propagates `#include <apm32/...>` to consumers.
target_include_directories(mcudrv_apm32 PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR})
