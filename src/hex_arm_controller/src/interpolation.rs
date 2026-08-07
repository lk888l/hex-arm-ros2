use crate::conversion::RosTarget;

#[derive(Debug, Clone)]
pub struct Interpolator {
    start: Vec<RosTarget>,
    goal: Vec<RosTarget>,
    started_ns: u64,
    duration_ns: u64,
}

impl Interpolator {
    pub fn hold(targets: Vec<RosTarget>, now_ns: u64) -> Self {
        Self {
            start: targets.clone(),
            goal: targets,
            started_ns: now_ns,
            duration_ns: 0,
        }
    }

    pub fn retarget(
        &mut self,
        goal: Vec<RosTarget>,
        now_ns: u64,
        duration_ns: u64,
    ) -> anyhow::Result<()> {
        if goal.len() != self.goal.len()
            || goal.iter().any(|target| {
                !target.position_rad.is_finite()
                    || !target.velocity_rad_s.is_finite()
                    || !target.torque_nm.is_finite()
                    || !target.kp_nm_rad.is_finite()
                    || !target.kd_nm_s_rad.is_finite()
            })
        {
            anyhow::bail!("interpolation target shape or value is invalid");
        }
        self.start = self.sample(now_ns);
        self.goal = goal;
        self.started_ns = now_ns;
        self.duration_ns = duration_ns;
        Ok(())
    }

    pub fn sample(&self, now_ns: u64) -> Vec<RosTarget> {
        let alpha = if self.duration_ns == 0 {
            1.0
        } else {
            (now_ns.saturating_sub(self.started_ns) as f32 / self.duration_ns as f32)
                .clamp(0.0, 1.0)
        };
        self.start
            .iter()
            .zip(&self.goal)
            .map(|(a, b)| RosTarget {
                position_rad: lerp(a.position_rad, b.position_rad, alpha),
                velocity_rad_s: lerp(a.velocity_rad_s, b.velocity_rad_s, alpha),
                torque_nm: lerp(a.torque_nm, b.torque_nm, alpha),
                kp_nm_rad: lerp(a.kp_nm_rad, b.kp_nm_rad, alpha),
                kd_nm_s_rad: lerp(a.kd_nm_s_rad, b.kd_nm_s_rad, alpha),
            })
            .collect()
    }
}

fn lerp(a: f32, b: f32, alpha: f32) -> f32 {
    a + (b - a) * alpha
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn interpolation_is_continuous_and_clamped() {
        let a = RosTarget {
            position_rad: 0.0,
            ..Default::default()
        };
        let b = RosTarget {
            position_rad: 2.0,
            ..Default::default()
        };
        let mut interpolator = Interpolator::hold(vec![a], 100);
        interpolator.retarget(vec![b], 100, 100).unwrap();
        assert_eq!(interpolator.sample(50)[0].position_rad, 0.0);
        assert_eq!(interpolator.sample(150)[0].position_rad, 1.0);
        assert_eq!(interpolator.sample(250)[0].position_rad, 2.0);
    }

    #[test]
    fn retarget_starts_at_current_sample() {
        let a = RosTarget {
            position_rad: 0.0,
            ..Default::default()
        };
        let b = RosTarget {
            position_rad: 2.0,
            ..Default::default()
        };
        let c = RosTarget {
            position_rad: 3.0,
            ..Default::default()
        };
        let mut interpolator = Interpolator::hold(vec![a], 0);
        interpolator.retarget(vec![b], 0, 100).unwrap();
        interpolator.retarget(vec![c], 50, 100).unwrap();
        assert_eq!(interpolator.sample(50)[0].position_rad, 1.0);
    }
}
