# Locate the installed schema, including when this package was relocated.
get_filename_component(hex_arm_protocol_SHARE_DIR "${CMAKE_CURRENT_LIST_DIR}/.." ABSOLUTE)
set(hex_arm_protocol_PROTO_FILE "${hex_arm_protocol_SHARE_DIR}/proto/robot_api.proto")
if(NOT EXISTS "${hex_arm_protocol_PROTO_FILE}")
  message(FATAL_ERROR "hex_arm_protocol schema is missing: ${hex_arm_protocol_PROTO_FILE}")
endif()
