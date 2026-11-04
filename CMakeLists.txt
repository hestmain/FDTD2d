cmake_minimum_required(VERSION 3.20)
project(fdtd2d VERSION 0.1.0 DESCRIPTION "2D acoustic FDTD simulator" LANGUAGES C)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

option(FDTD2D_BUILD_TESTS "Build unit and validation tests" ON)
option(FDTD2D_WERROR "Treat compiler warnings as errors" OFF)
option(FDTD2D_SANITIZE "Build with AddressSanitizer and UBSan" OFF)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(PNG REQUIRED)

if(FDTD2D_SANITIZE)
  add_compile_options(-fsanitize=address,undefined -fno-omit-frame-pointer)
  add_link_options(-fsanitize=address,undefined)
endif()

# ---- third party: tomlc17 (MIT, vendored, unmodified) --------------------
add_library(tomlc17 STATIC third_party/tomlc17/tomlc17.c)
set_target_properties(tomlc17 PROPERTIES C_STANDARD 17 POSITION_INDEPENDENT_CODE ON)
target_include_directories(tomlc17 SYSTEM PUBLIC third_party/tomlc17)

# ---- core library ---------------------------------------------------------
add_library(fdtd2d_core STATIC
  src/common.c
  src/geometry.c
  src/medium.c
  src/waveform.c
  src/boundary.c
  src/config.c
  src/source.c
  src/model.c
  src/solver.c
  src/render.c
)
target_include_directories(fdtd2d_core PUBLIC src)
# j0/y0 (tests), mkdir and friends need X/Open + POSIX declarations.
target_compile_definitions(fdtd2d_core PUBLIC _XOPEN_SOURCE=700)
target_link_libraries(fdtd2d_core PUBLIC PNG::PNG m PRIVATE tomlc17)

set(FDTD2D_WARNINGS -Wall -Wextra -Wpedantic -Wshadow -Wstrict-prototypes
    -Wmissing-prototypes -Wformat=2 -Wundef)
if(FDTD2D_WERROR)
  list(APPEND FDTD2D_WARNINGS -Werror)
endif()
target_compile_options(fdtd2d_core PRIVATE ${FDTD2D_WARNINGS})

# ---- command line tool ----------------------------------------------------
add_executable(fdtd2d src/main.c)
target_link_libraries(fdtd2d PRIVATE fdtd2d_core)
target_compile_options(fdtd2d PRIVATE ${FDTD2D_WARNINGS})
install(TARGETS fdtd2d RUNTIME DESTINATION bin)

# ---- tests ------------------------------------------------------------------
if(FDTD2D_BUILD_TESTS)
  enable_testing()
  # Unit tests: fast. Validation tests: physics checks, slower.
  set(FDTD2D_UNIT_TESTS geometry config model source render solver)
  set(FDTD2D_VALIDATION_TESTS validation)
  foreach(name IN LISTS FDTD2D_UNIT_TESTS FDTD2D_VALIDATION_TESTS)
    add_executable(test_${name} tests/test_${name}.c)
    target_link_libraries(test_${name} PRIVATE fdtd2d_core)
    target_compile_options(test_${name} PRIVATE ${FDTD2D_WARNINGS})
    target_compile_definitions(test_${name} PRIVATE
      FDTD2D_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}"
      FDTD2D_TEST_TMPDIR="${CMAKE_CURRENT_BINARY_DIR}/test_tmp")
    add_test(NAME ${name} COMMAND test_${name})
    # Exit code 77 = test not implemented / skipped.
    set_tests_properties(${name} PROPERTIES SKIP_RETURN_CODE 77)
  endforeach()
  foreach(name IN LISTS FDTD2D_UNIT_TESTS)
    set_tests_properties(${name} PROPERTIES LABELS unit TIMEOUT 60)
  endforeach()
  foreach(name IN LISTS FDTD2D_VALIDATION_TESTS)
    set_tests_properties(${name} PROPERTIES LABELS validation TIMEOUT 600)
  endforeach()
  # End-to-end smoke test of the CLI on the shipped examples (--check only).
  file(GLOB FDTD2D_EXAMPLES CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/examples/*.toml)
  foreach(example IN LISTS FDTD2D_EXAMPLES)
    get_filename_component(example_name ${example} NAME_WE)
    add_test(NAME cli_check_${example_name} COMMAND fdtd2d --check ${example})
    set_tests_properties(cli_check_${example_name} PROPERTIES LABELS unit TIMEOUT 60)
  endforeach()
endif()
