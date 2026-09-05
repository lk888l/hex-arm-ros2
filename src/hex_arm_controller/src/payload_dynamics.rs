//! Build the controller gravity model from the arm URDF plus an optional fixed
//! tool payload recorded in the hardware profile.

use anyhow::{Context, Result};

use crate::profile::{HardwareProfile, TipPayloadProfile};

/// Read the arm URDF exactly once and build the model used by profile-only
/// validation, isolated commissioning, and the normal ROS/MoveIt runtime.
pub fn load_profile_dynamics(profile: &HardwareProfile) -> Result<hex_arm_dynamics::ArmDynamics> {
    let urdf_xml = std::fs::read_to_string(&profile.urdf_path)
        .with_context(|| format!("read dynamics URDF {}", profile.urdf_path))?;
    let dynamics_xml = match &profile.tip_payload {
        Some(payload) => merge_fixed_tip_payload(&urdf_xml, payload)?,
        None => urdf_xml,
    };
    hex_arm_dynamics::ArmDynamics::from_urdf_string(&dynamics_xml)
        .context("load fixed hex-arm-dynamics model")
}

fn merge_fixed_tip_payload(urdf_xml: &str, payload: &TipPayloadProfile) -> Result<String> {
    anyhow::ensure!(
        payload.mount_link == "link_6",
        "tip payload must mount in the serial arm tip link_6 frame"
    );
    let mut robot =
        urdf_rs::read_from_string(urdf_xml).context("parse arm URDF for tip payload")?;

    let mount_joint = robot
        .joints
        .iter()
        .find(|joint| joint.child.link == payload.mount_link)
        .with_context(|| {
            format!(
                "tip payload mount {} is not the child of a serial arm joint",
                payload.mount_link
            )
        })?;
    anyhow::ensure!(
        mount_joint.name == "joint_6"
            && matches!(
                mount_joint.joint_type,
                urdf_rs::JointType::Revolute | urdf_rs::JointType::Continuous
            ),
        "tip payload mount link_6 must be the revolute serial tip child of joint_6"
    );
    anyhow::ensure!(
        !robot
            .joints
            .iter()
            .any(|joint| joint.parent.link == payload.mount_link),
        "tip payload mount link_6 is not a terminal serial link"
    );

    let mount = robot
        .links
        .iter_mut()
        .find(|link| link.name == payload.mount_link)
        .with_context(|| format!("tip payload mount link {} is absent", payload.mount_link))?;
    let arm_mass = mount.inertial.mass.value;
    let arm_com = mount.inertial.origin.xyz.0;
    anyhow::ensure!(
        arm_mass.is_finite() && arm_mass >= 0.0 && arm_com.iter().all(|value| value.is_finite()),
        "tip payload mount link has invalid arm inertial data"
    );

    let payload_mass = f64::from(payload.mass_kg);
    let combined_mass = arm_mass + payload_mass;
    anyhow::ensure!(
        combined_mass.is_finite() && combined_mass > 0.0,
        "combined tip payload mass is invalid"
    );
    for (axis, combined) in mount.inertial.origin.xyz.iter_mut().enumerate() {
        *combined = (arm_mass * arm_com[axis]
            + payload_mass * f64::from(payload.center_of_mass_xyz_m[axis]))
            / combined_mass;
    }
    mount.inertial.mass.value = combined_mass;

    // hex-arm-dynamics currently consumes only mass and COM for G(q). Keep the
    // arm link inertia unchanged rather than fabricating a parallel-axis tensor
    // from trial payload data that explicitly has not been calibrated.
    urdf_rs::write_to_string(&robot).context("serialize arm URDF with fixed tip payload")
}

#[cfg(test)]
mod tests {
    use approx::assert_abs_diff_eq;

    use super::*;
    use crate::profile::IdentityFingerprint;

    fn payload() -> TipPayloadProfile {
        TipPayloadProfile {
            mount_link: "link_6".into(),
            auxiliary_node_id: 15,
            identity: IdentityFingerprint {
                vendor_id: 0x4859_444c,
                product_code: 0xaaaa_0001,
                revision: 9,
                serial_number: 0x2573_510b,
                model: "GR80 trial".into(),
            },
            mass_kg: 1.0,
            center_of_mass_xyz_m: [1.0, 0.0, 0.0],
            source_urdf_sha256: "f74b3e76b14175c788c5ef70dd0c1941958d229a8461da6f20e17543e4ba1114"
                .into(),
            inertial_calibrated: false,
        }
    }

    fn pendulum_urdf() -> &'static str {
        r#"<?xml version="1.0"?>
<robot name="payload_pendulum">
  <link name="base"/>
  <link name="link_6">
    <inertial>
      <origin xyz="0.5 0 0" rpy="0 0 0"/>
      <mass value="1"/>
      <inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/>
    </inertial>
  </link>
  <joint name="joint_6" type="revolute">
    <parent link="base"/>
    <child link="link_6"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-3.14" upper="3.14" effort="10" velocity="1"/>
  </joint>
</robot>"#
    }

    #[test]
    fn fixed_payload_mass_and_com_contribute_to_single_pendulum_gravity() {
        let merged = merge_fixed_tip_payload(pendulum_urdf(), &payload()).unwrap();
        let dynamics = hex_arm_dynamics::ArmDynamics::from_urdf_string(&merged).unwrap();

        // Existing 1 kg at x=.5 plus payload 1 kg at x=1 gives a 2 kg link
        // at x=.75. At q=0 its +Y-axis holding torque is -m*g*l.
        assert_abs_diff_eq!(
            dynamics.gravity_torque(&[0.0])[0],
            -14.715,
            epsilon = 1.0e-3
        );
    }

    #[test]
    fn fixed_payload_rejects_a_non_tip_mount() {
        let mut bad = payload();
        bad.mount_link = "base".into();
        let error = merge_fixed_tip_payload(pendulum_urdf(), &bad)
            .unwrap_err()
            .to_string();
        assert!(error.contains("link_6"));
    }

    #[test]
    #[allow(clippy::approx_constant)] // surveyed joint_3 value, deliberately not mathematical PI
    fn field_arm_trial_payload_changes_the_surveyed_pose_gravity_model() {
        let arm_path = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("../xpkg_urdf_firefly_y6/urdf/xpkg_urdf_firefly_y6.urdf");
        let arm_xml = std::fs::read_to_string(arm_path).unwrap();
        let nominal = hex_arm_dynamics::ArmDynamics::from_urdf_string(&arm_xml).unwrap();
        let mut trial = payload();
        trial.mass_kg = 0.41;
        trial.center_of_mass_xyz_m = [0.005_537_804_7, 0.000_026_829_268, 0.048_889_972];
        let merged = merge_fixed_tip_payload(&arm_xml, &trial).unwrap();
        let augmented = hex_arm_dynamics::ArmDynamics::from_urdf_string(&merged).unwrap();

        assert_eq!(augmented.dof(), 6);
        let surveyed_q = [0.0, -1.570, 3.140, 0.0, 0.0, 0.0];
        let nominal_gravity = nominal.gravity_torque(&surveyed_q);
        let augmented_gravity = augmented.gravity_torque(&surveyed_q);
        assert!(augmented_gravity.iter().all(|value| value.is_finite()));
        // Regression for the corrected joint-2 sign and conventional -Z
        // gravity candidate.  The previous +1.570 fit reversed J2/J3/J4 and
        // was able to pass the weaker "some axis changed" check below.
        let expected = [0.0, 1.865_3, -5.956_6, -1.157_3, 0.0, 0.0];
        for (actual, expected) in augmented_gravity.iter().zip(expected) {
            assert_abs_diff_eq!(*actual, expected, epsilon = 5.0e-3);
        }
        assert!(nominal_gravity
            .iter()
            .zip(&augmented_gravity)
            .any(|(before, after)| (after - before).abs() > 0.05));
    }
}
