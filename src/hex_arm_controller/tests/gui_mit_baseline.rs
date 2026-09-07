//! Golden values independently evaluated from the GUI's unchanged Firefly URDF.
use hex_arm_controller::conversion::{motor_position_to_ros, ros_target_to_motor, RosTarget};
use hex_arm_controller::payload_dynamics::load_profile_dynamics;
use hex_arm_controller::profile::HardwareProfile;

fn reviewed_test_profile() -> HardwareProfile {
    let root = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../..");
    let contents =
        std::fs::read_to_string(root.join("config/hardware/firefly_y6.meow_mit.example.yaml"))
            .unwrap();
    let mut profile: HardwareProfile = serde_yaml::from_str(&contents).unwrap();
    assert!(!profile.validated && !profile.calibrated);
    profile.validated = true;
    profile.urdf_path = root
        .join("src/xpkg_urdf_firefly_y6/urdf/xpkg_urdf_firefly_y6.urdf")
        .to_string_lossy()
        .into_owned();
    profile.bus.expected_link.as_mut().unwrap().adapter.serial =
        "0123456789ABCDEF0123456789ABCDEF".into();
    for joint in &mut profile.joints {
        joint.identity.vendor_id = 0x0068_6578;
        joint.identity.product_code = 0x6c64_bc78;
        joint.identity.revision = 1;
        joint.identity.serial_number = joint.node_id as u32;
        joint.identity.model = "hexmeow 4310".into();
    }
    profile
}

#[test]
#[allow(clippy::approx_constant)]
fn gui_park_coordinates_and_gravity_match_independent_reference() {
    let profile = reviewed_test_profile();
    profile.validate().unwrap();
    profile.validate_single_turn_command_windows().unwrap();
    let q = [0.0, -1.57, 3.14, 0.0, 0.0, 0.0];
    let encoder = [0.000269, 0.249714, 0.251193, -0.00159, -0.004028, 0.000018];
    let expected_gravity = [0.0, 2.3794834, -4.3563867, -0.5624728, 0.0, 0.0];
    let expected_motor_ff = [0.0, -0.7138450, -3.0494707, -0.3937309, 0.0, 0.0];
    let dynamics = load_profile_dynamics(&profile).unwrap();
    let gravity = dynamics.gravity_torque_with(&q, profile.gravity_vector_base_m_s2);
    for (i, joint) in profile.joints.iter().enumerate() {
        assert!(
            (gravity[i] - expected_gravity[i]).abs() < 2.0e-4,
            "joint {} G(q)",
            i + 1
        );
        assert!((motor_position_to_ros(encoder[i], joint) - q[i]).abs() < 1.0e-6);
        let target = ros_target_to_motor(
            RosTarget {
                position_rad: q[i],
                velocity_rad_s: 0.0,
                torque_nm: joint
                    .clamp_gravity_feedforward(gravity[i] * joint.gravity_compensation_scale),
                kp_nm_rad: joint.default_kp,
                kd_nm_s_rad: joint.default_kd,
            },
            joint,
        );
        assert!((target.position_rev - encoder[i]).abs() < 1.0e-6);
        assert!((target.torque_nm - expected_motor_ff[i]).abs() < 2.0e-4);
        assert!((target.kp_nm_rev - 502.65482).abs() < 1.0e-3);
        assert!((target.kd_nm_s_rev - 94.24778).abs() < 1.0e-3);
    }
}

#[test]
fn meow_profile_does_not_inherit_legacy_seam_or_torque_calibration() {
    let mut profile = reviewed_test_profile();
    // A Meow multi-turn offset outside +/-0.5 Rev is still representable.
    profile.joints[0].zero_offset_rad = 7.0;
    profile.validate_single_turn_command_windows().unwrap();
    profile.joints[0].torque_scale = 0.85;
    assert!(profile
        .validate()
        .unwrap_err()
        .to_string()
        .contains("torque_scale"));
}
