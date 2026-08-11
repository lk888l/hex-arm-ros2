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

    pub fn retarget_with_velocity_limits(
        &mut self,
        goal: Vec<RosTarget>,
        now_ns: u64,
        requested_duration_ns: u64,
        velocity_limits_rad_s: &[f32],
    ) -> anyhow::Result<()> {
        if velocity_limits_rad_s.len() != goal.len()
            || velocity_limits_rad_s
                .iter()
                .any(|limit| !limit.is_finite() || *limit <= 0.0)
        {
            anyhow::bail!("velocity limit shape or value is invalid");
        }

        let start = self.sample(now_ns);
        let minimum_duration_ns = start
            .iter()
            .zip(&goal)
            .zip(velocity_limits_rad_s)
            .map(|((current, target), limit)| {
                let seconds =
                    (target.position_rad - current.position_rad).abs() as f64 / *limit as f64;
                (seconds * 1_000_000_000.0).ceil() as u64
            })
            .max()
            .unwrap_or(0);

        self.retarget(goal, now_ns, requested_duration_ns.max(minimum_duration_ns))
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

    #[test]
    fn position_step_is_stretched_to_the_velocity_limit() {
        let start = RosTarget {
            position_rad: 0.0,
            ..Default::default()
        };
        let goal = RosTarget {
            position_rad: 1.0,
            ..Default::default()
        };
        let mut interpolator = Interpolator::hold(vec![start], 0);
        interpolator
            .retarget_with_velocity_limits(vec![goal], 0, 0, &[0.2])
            .unwrap();

        assert!((interpolator.sample(1_000_000_000)[0].position_rad - 0.2).abs() < 1e-6);
        assert_eq!(interpolator.sample(5_000_000_000)[0].position_rad, 1.0);
    }

    #[test]
    fn repeated_retargeting_remains_rate_limited() {
        let start = RosTarget {
            position_rad: 0.0,
            ..Default::default()
        };
        let goal = RosTarget {
            position_rad: 1.0,
            ..Default::default()
        };
        let mut interpolator = Interpolator::hold(vec![start], 0);
        interpolator
            .retarget_with_velocity_limits(vec![goal], 0, 10_000_000, &[0.2])
            .unwrap();
        interpolator
            .retarget_with_velocity_limits(vec![goal], 10_000_000, 10_000_000, &[0.2])
            .unwrap();

        assert!((interpolator.sample(20_000_000)[0].position_rad - 0.004).abs() < 1e-5);
    }

    #[test]
    fn rejects_invalid_velocity_limits() {
        let target = RosTarget::default();
        let mut interpolator = Interpolator::hold(vec![target], 0);
        assert!(interpolator
            .retarget_with_velocity_limits(vec![target], 0, 0, &[])
            .is_err());
        assert!(interpolator
            .retarget_with_velocity_limits(vec![target], 0, 0, &[0.0])
            .is_err());
        assert!(interpolator
            .retarget_with_velocity_limits(vec![target], 0, 0, &[f32::NAN])
            .is_err());
    }
}
