# Audit metadata for hex_arm/internal/state. All ROS times use this bridge's
# ROS clock. JointState.header.stamp equals received_stamp, not motor sample time.
builtin_interfaces/Time stamp
builtin_interfaces/Time received_stamp
string instance
uint64 sequence
# Original driver timestamp: publication time since driver process startup.
# Never subtract it from ROS time or another host/process's monotonic time.
int64 source_stamp_ns
uint8 DRIVER_PROCESS_MONOTONIC=1
uint8 source_clock
# Local bridge steady-clock reception time and elapsed time since reception.
# This age excludes CAN acquisition and transport latency before reception.
int64 received_steady_ns
float64 receive_age_s
bool feedback_fresh
# Both are false until the protocol supplies verified acquisition/sync metadata.
bool motor_sample_time_available
bool clocks_synchronized
