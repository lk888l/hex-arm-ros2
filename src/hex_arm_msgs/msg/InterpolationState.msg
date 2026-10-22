# Diagnostic snapshot only; never a command or a motor sample timestamp.
# All *_ns below use the Rust driver's process-relative monotonic clock,
# except requested/effective_duration_ns, which are durations.
builtin_interfaces/Time stamp
string[] joint_names
uint64 source_stamp_ns
uint64 command_sequence
uint64 command_generation
uint64 segment_start_ns
uint64 requested_duration_ns
uint64 effective_duration_ns
uint64 sample_stamp_ns
float64[6] command_position
float64[6] command_velocity
float64[6] interpolated_position
float64[6] interpolated_velocity
