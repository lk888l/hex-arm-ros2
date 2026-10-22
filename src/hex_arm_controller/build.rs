use std::path::PathBuf;

fn main() {
    println!("cargo:rerun-if-env-changed=HEX_ARM_PROTO_FILE");
    // ROS builds use the installed protocol package. Standalone Cargo builds
    // use the compatibility symlink in a complete source checkout, or an
    // explicit schema path when the controller source is copied independently.
    let schema = std::env::var_os("HEX_ARM_PROTO_FILE")
        .map(PathBuf::from)
        .unwrap_or_else(|| PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("proto/robot_api.proto"));
    let schema = schema.canonicalize().unwrap_or_else(|error| {
        panic!("robot_api.proto unavailable ({error}); install hex_arm_protocol and set HEX_ARM_PROTO_FILE to its share/hex_arm_protocol/proto/robot_api.proto")
    });
    let include = schema.parent().expect("schema parent directory");
    let protoc = protoc_bin_vendored::protoc_bin_path().expect("vendored protoc unavailable");
    std::env::set_var("PROTOC", protoc);
    prost_build::Config::new()
        .compile_protos(&[&schema], &[include])
        .expect("robot_api protobuf generation failed");
    println!("cargo:rerun-if-changed={}", schema.display());
}
