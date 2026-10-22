cmake_minimum_required(VERSION 3.16)
project(hex_arm_description VERSION 0.1.0 LANGUAGES NONE)

find_package(ament_cmake REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)
find_package(xpkg_urdf_firefly_y6 REQUIRED)

# Resolve the exported package, so isolated workspaces never need a sibling
# vendor checkout. Reconfigure when either the pinned model or generator changes.
set(vendor_urdf "${xpkg_urdf_firefly_y6_DIR}/../urdf/xpkg_urdf_firefly_y6.urdf")
set(generator "${CMAKE_CURRENT_SOURCE_DIR}/scripts/generate_geometry_macro.py")
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${vendor_urdf}" "${generator}")
execute_process(COMMAND "${Python3_EXECUTABLE}" "${generator}" "${vendor_urdf}"
  "${CMAKE_CURRENT_BINARY_DIR}/firefly_y6.geometry.xacro"
  COMMAND_ERROR_IS_FATAL ANY)

install(DIRECTORY urdf DESTINATION share/${PROJECT_NAME})
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/firefly_y6.geometry.xacro"
  DESTINATION share/${PROJECT_NAME}/urdf)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(description_contract test/test_description.py TIMEOUT 60)
endif()

ament_package()
