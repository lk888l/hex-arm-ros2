//! One versioned startup recipe shared with the ROS trajectory client.
use anyhow::Result;
use serde::Deserialize;
use std::sync::LazyLock;

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct StartupRecipe {
    pub schema_version: u32,
    pub folded_position_rad: [f32; 6],
    pub ros_folded_tolerance_rad: [f32; 5],
    pub commissioning_folded_tolerance_rad: [f32; 5],
    pub stopped_velocity_rad_s: f32,
    pub steps: [StartupStep; 3],
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct StartupStep {
    pub joint_index: usize,
    pub target_rad: f32,
    pub duration_sec: f32,
}

impl StartupRecipe {
    pub fn parse(yaml: &str) -> Result<Self> {
        let recipe: Self = serde_yaml::from_str(yaml)?;
        anyhow::ensure!(recipe.schema_version == 1, "unsupported startup recipe");
        anyhow::ensure!(
            recipe.folded_position_rad.iter().all(|q| q.is_finite()),
            "nonfinite folded posture"
        );
        anyhow::ensure!(
            recipe
                .ros_folded_tolerance_rad
                .iter()
                .chain(&recipe.commissioning_folded_tolerance_rad)
                .chain(std::iter::once(&recipe.stopped_velocity_rad_s))
                .all(|v| v.is_finite() && *v > 0.0),
            "invalid startup tolerance"
        );
        for (step, expected_axis) in recipe.steps.iter().zip([1, 3, 2]) {
            anyhow::ensure!(
                step.joint_index == expected_axis,
                "startup must execute J2 -> J4 -> J3"
            );
            anyhow::ensure!(
                step.target_rad.is_finite()
                    && step.duration_sec.is_finite()
                    && step.duration_sec > 0.0,
                "invalid startup waypoint"
            );
        }
        Ok(recipe)
    }

    pub fn waypoints(&self) -> [(usize, f32, f32); 3] {
        std::array::from_fn(|i| {
            let step = &self.steps[i];
            (step.joint_index, step.target_rad, step.duration_sec)
        })
    }

    pub fn ready(&self) -> [f32; 6] {
        let mut q = self.folded_position_rad;
        for axis in [0, 4, 5] {
            q[axis] = 0.0;
        }
        for step in &self.steps {
            q[step.joint_index] = step.target_rad;
        }
        q
    }
}

pub const SOURCE: &str = include_str!("../config/startup.yaml");
pub static RECIPE: LazyLock<StartupRecipe> =
    LazyLock::new(|| StartupRecipe::parse(SOURCE).expect("invalid built-in startup recipe"));

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn canonical_sequence_has_the_verified_ready_pose() {
        assert_eq!(RECIPE.waypoints().map(|step| step.0), [1, 3, 2]);
        assert_eq!(RECIPE.ready(), [0.0, -1.35, 1.43, -0.30, 0.0, 0.0]);
    }
    #[test]
    fn rejects_reordering_nonfinite_and_unknown_fields() {
        for invalid in [
            SOURCE.replace("joint_index: 1", "joint_index: 2"),
            SOURCE.replace("duration_sec: 8.0", "duration_sec: 0"),
            SOURCE.replace("target_rad: -1.350", "target_rad: .nan"),
            SOURCE.replace("schema_version: 1", "schema_version: 2"),
            format!("{SOURCE}\nunknown: true\n"),
        ] {
            assert!(StartupRecipe::parse(&invalid).is_err());
        }
    }
}
