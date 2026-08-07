cmake_minimum_required(VERSION 3.16)
project(hex_arm_controller VERSION 0.1.0 LANGUAGES NONE)

find_package(ament_cmake REQUIRED)
find_program(CARGO_EXECUTABLE cargo REQUIRED)

file(GLOB_RECURSE rust_sources CONFIGURE_DEPENDS
  "${CMAKE_CURRENT_SOURCE_DIR}/src/*.rs"
  "${CMAKE_CURRENT_SOURCE_DIR}/vendor/hex-motor/src/*.rs")
set(cargo_target_dir "${CMAKE_CURRENT_BINARY_DIR}/cargo")
set(controller_binary "${cargo_target_dir}/release/hex_arm_controller")

add_custom_command(
  OUTPUT "${controller_binary}"
  COMMAND ${CMAKE_COMMAND} -E env CARGO_TARGET_DIR=${cargo_target_dir}
    ${CARGO_EXECUTABLE} build --release --locked
  WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
  DEPENDS ${rust_sources} Cargo.toml Cargo.lock build.rs proto/robot_api.proto
  USES_TERMINAL
  VERBATIM)
add_custom_target(${PROJECT_NAME}_cargo ALL DEPENDS "${controller_binary}")

install(PROGRAMS "${controller_binary}" DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY proto DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  add_test(NAME ${PROJECT_NAME}_rust_tests
    COMMAND ${CMAKE_COMMAND} -E env CARGO_TARGET_DIR=${cargo_target_dir}
      ${CARGO_EXECUTABLE} test --locked)
  set_tests_properties(${PROJECT_NAME}_rust_tests PROPERTIES
    WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}" TIMEOUT 180)
endif()

ament_package()

