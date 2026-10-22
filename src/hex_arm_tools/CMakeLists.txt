cmake_minimum_required(VERSION 3.16)
project(hex_arm_tools VERSION 0.1.0 LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(Protobuf REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)
find_package(hex_arm_protocol REQUIRED)

# Generate clients without requiring the Rust daemon package.
set(proto_file "${hex_arm_protocol_PROTO_FILE}")
get_filename_component(protocol_dir "${proto_file}" DIRECTORY)
set(generated_dir "${CMAKE_CURRENT_BINARY_DIR}/generated")
set(generated_python "${generated_dir}/robot_api_pb2.py")
add_custom_command(
  OUTPUT "${generated_python}"
  COMMAND ${CMAKE_COMMAND} -E make_directory "${generated_dir}"
  COMMAND protobuf::protoc
    --proto_path=${protocol_dir}
    --python_out=${generated_dir}
    "${proto_file}"
  DEPENDS "${proto_file}"
  VERBATIM)
add_custom_target(${PROJECT_NAME}_protobuf ALL DEPENDS "${generated_python}")

ament_python_install_package(${PROJECT_NAME})
install(FILES "${generated_python}"
  DESTINATION "${PYTHON_INSTALL_DIR}/${PROJECT_NAME}/pb")
install(PROGRAMS scripts/hex_arm_gravity_comp
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(gravity_comp_client test/test_gravity_comp.py TIMEOUT 30)
  ament_add_pytest_test(gravity_comp_integration test/test_gravity_comp_integration.py TIMEOUT 60)
endif()

ament_package()
