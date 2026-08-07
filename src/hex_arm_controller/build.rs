fn main() {
    let protoc = protoc_bin_vendored::protoc_bin_path().expect("vendored protoc unavailable");
    std::env::set_var("PROTOC", protoc);
    prost_build::Config::new()
        .compile_protos(&["proto/robot_api.proto"], &["proto"])
        .expect("robot_api protobuf generation failed");
    println!("cargo:rerun-if-changed=proto/robot_api.proto");
}
