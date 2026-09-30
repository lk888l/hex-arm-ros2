cmake_minimum_required(VERSION 3.16)
project(hex_arm_controller VERSION 0.1.0 LANGUAGES NONE)

find_package(ament_cmake REQUIRED)
find_program(CARGO_EXECUTABLE cargo REQUIRED)
option(HEX_ARM_ENABLE_CIA402 "Include the CiA402 motor backend in the production controller" ON)
option(HEX_ARM_BUILD_COMMISSIONING "Build/install historical commissioning executable" OFF)

set(cargo_build_args --no-default-features --bin hex_arm_controller)
set(cargo_test_args --no-default-features)
set(cargo_target_dir "${CMAKE_CURRENT_BINARY_DIR}/cargo")
set(controller_binary "${cargo_target_dir}/release/hex_arm_controller")
set(cargo_outputs "${controller_binary}")
if(HEX_ARM_BUILD_COMMISSIONING AND NOT HEX_ARM_ENABLE_CIA402)
  message(FATAL_ERROR "HEX_ARM_BUILD_COMMISSIONING requires HEX_ARM_ENABLE_CIA402=ON")
endif()
if(HEX_ARM_ENABLE_CIA402)
  list(APPEND cargo_build_args --features legacy)
  list(APPEND cargo_test_args --features legacy)
endif()
if(HEX_ARM_BUILD_COMMISSIONING)
  list(APPEND cargo_build_args --bin hex_arm_commission)
  list(APPEND cargo_outputs "${cargo_target_dir}/release/hex_arm_commission")
endif()

# Cargo owns source AND feature fingerprints. Always invoke it: an executable
# built with legacy enabled has the same filename and may be newer than all
# sources when the CMake option is switched OFF. Timestamp-only output rules
# could otherwise silently reinstall the wrong feature set.
add_custom_target(${PROJECT_NAME}_cargo ALL
  COMMAND ${CMAKE_COMMAND} -E env CARGO_TARGET_DIR=${cargo_target_dir}
    ${CARGO_EXECUTABLE} build --release --locked ${cargo_build_args}
  WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
  BYPRODUCTS ${cargo_outputs}
  USES_TERMINAL
  VERBATIM)

install(PROGRAMS ${cargo_outputs} DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY proto config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  add_test(NAME ${PROJECT_NAME}_rust_tests
    COMMAND ${CMAKE_COMMAND} -E env CARGO_TARGET_DIR=${cargo_target_dir}
      ${CARGO_EXECUTABLE} test --locked ${cargo_test_args})
  set_tests_properties(${PROJECT_NAME}_rust_tests PROPERTIES
    WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}" TIMEOUT 180)
endif()

ament_package()
