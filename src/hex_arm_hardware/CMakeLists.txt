cmake_minimum_required(VERSION 3.16)
project(hex_arm_hardware VERSION 0.1.0 LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(hex_arm_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(Protobuf REQUIRED)
# Match Rust Cargo.lock and eclipse-zenoh Python; refuse silent ABI upgrades.
set(HEX_ARM_ZENOH_PREFIX "/opt/hex-arm-zenoh" CACHE PATH "Pinned Zenoh C/C++ SDK prefix")
list(APPEND CMAKE_PREFIX_PATH "${HEX_ARM_ZENOH_PREFIX}")
find_package(zenohc 1.9.0 EXACT REQUIRED CONFIG)
find_package(zenohcxx 1.9.0 EXACT REQUIRED CONFIG)
protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS ../hex_arm_controller/proto/robot_api.proto)

add_library(${PROJECT_NAME} SHARED src/hex_arm_system.cpp src/zenoh_transport.cpp ${PROTO_SRCS})
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra -Wpedantic -Wconversion)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_CURRENT_BINARY_DIR})
target_link_libraries(${PROJECT_NAME} zenohcxx::zenohc protobuf::libprotobuf)
set_target_properties(${PROJECT_NAME} PROPERTIES INSTALL_RPATH "$ORIGIN")
ament_target_dependencies(${PROJECT_NAME}
  hardware_interface pluginlib rclcpp rclcpp_lifecycle sensor_msgs std_srvs hex_arm_msgs diagnostic_msgs)

pluginlib_export_plugin_description_file(hardware_interface hex_arm_hardware.xml)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
get_target_property(ZENOHC_LIBRARY zenohc::lib IMPORTED_LOCATION)
install(FILES "${ZENOHC_LIBRARY}" DESTINATION lib)
get_filename_component(ZENOHC_LIBRARY_DIR "${ZENOHC_LIBRARY}" DIRECTORY)
install(DIRECTORY "${ZENOHC_LIBRARY_DIR}/../share/hex_arm_zenoh"
  DESTINATION share/${PROJECT_NAME} OPTIONAL)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_transport_trace test/test_transport_trace.cpp)
  target_include_directories(test_transport_trace PRIVATE include)
  target_compile_features(test_transport_trace PRIVATE cxx_std_17)
  ament_add_gtest(test_snapshot_mailbox test/test_snapshot_mailbox.cpp)
  target_include_directories(test_snapshot_mailbox PRIVATE include)
  target_compile_features(test_snapshot_mailbox PRIVATE cxx_std_17)
  add_executable(transport_probe test/transport_probe.cpp)
  target_link_libraries(transport_probe ${PROJECT_NAME})
  # Exercise this plugin's Jazzy-compatible legacy exported handle interface.
  target_compile_options(transport_probe PRIVATE -Wno-deprecated-declarations)
  install(TARGETS transport_probe DESTINATION lib/${PROJECT_NAME})
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(hardware_interface pluginlib rclcpp sensor_msgs std_srvs)
ament_package()

