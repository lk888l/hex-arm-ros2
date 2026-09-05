use crate::conversion::RosTarget;

const NS_PER_SECOND: f64 = 1_000_000_000.0;
const BOUND_ROUNDOFF: f64 = 1.0e-9;

#[derive(Debug, Clone, Copy)]
struct CubicSegment {
    position_0: f64,
    velocity_0: f64,
    c2: f64,
    c3: f64,
    duration_sec: f64,
}

impl CubicSegment {
    fn new(start: RosTarget, goal: RosTarget, duration_ns: u64) -> Self {
        let duration_sec = duration_ns as f64 / NS_PER_SECOND;
        if duration_ns == 0 {
            return Self {
                position_0: goal.position_rad as f64,
                velocity_0: goal.velocity_rad_s as f64,
                c2: 0.0,
                c3: 0.0,
                duration_sec: 0.0,
            };
        }

        let position_0 = start.position_rad as f64;
        let velocity_0 = start.velocity_rad_s as f64;
        let position_1 = goal.position_rad as f64;
        let velocity_1 = goal.velocity_rad_s as f64;
        let displacement = position_1 - position_0;
        let duration_squared = duration_sec * duration_sec;
        let duration_cubed = duration_squared * duration_sec;
        let c2 = (3.0 * displacement - (2.0 * velocity_0 + velocity_1) * duration_sec)
            / duration_squared;
        let c3 = (-2.0 * displacement + (velocity_0 + velocity_1) * duration_sec) / duration_cubed;
        Self {
            position_0,
            velocity_0,
            c2,
            c3,
            duration_sec,
        }
    }

    fn sample(&self, elapsed_sec: f64) -> (f64, f64, f64) {
        let time = elapsed_sec.clamp(0.0, self.duration_sec);
        let position = self.position_0
            + self.velocity_0 * time
            + self.c2 * time * time
            + self.c3 * time * time * time;
        let velocity = self.velocity_0 + 2.0 * self.c2 * time + 3.0 * self.c3 * time * time;
        let acceleration = 2.0 * self.c2 + 6.0 * self.c3 * time;
        (position, velocity, acceleration)
    }

    /// Exact continuous-time maxima for this cubic. Acceleration is linear,
    /// and velocity is quadratic with at most one interior extremum.
    fn maximum_absolute_velocity_and_acceleration(&self) -> (f64, f64) {
        if self.duration_sec == 0.0 {
            return (self.velocity_0.abs(), 0.0);
        }
        let (_, velocity_0, acceleration_0) = self.sample(0.0);
        let (_, velocity_1, acceleration_1) = self.sample(self.duration_sec);
        let mut maximum_velocity = velocity_0.abs().max(velocity_1.abs());
        if self.c3 != 0.0 {
            let stationary_time = -self.c2 / (3.0 * self.c3);
            if stationary_time > 0.0 && stationary_time < self.duration_sec {
                maximum_velocity = maximum_velocity.max(self.sample(stationary_time).1.abs());
            }
        }
        (
            maximum_velocity,
            acceleration_0.abs().max(acceleration_1.abs()),
        )
    }
}

#[derive(Debug, Clone)]
pub struct Interpolator {
    start: Vec<RosTarget>,
    goal: Vec<RosTarget>,
    segments: Vec<CubicSegment>,
    velocity_limits_rad_s: Vec<f32>,
    started_ns: u64,
    duration_ns: u64,
}

impl Interpolator {
    pub fn hold(targets: Vec<RosTarget>, now_ns: u64) -> Self {
        let segments = targets
            .iter()
            .copied()
            .map(|target| CubicSegment::new(target, target, 0))
            .collect();
        Self {
            start: targets.clone(),
            goal: targets,
            segments,
            velocity_limits_rad_s: Vec::new(),
            started_ns: now_ns,
            duration_ns: 0,
        }
    }

    pub fn retarget_with_limits(
        &mut self,
        goal: Vec<RosTarget>,
        now_ns: u64,
        requested_duration_ns: u64,
        velocity_limits_rad_s: &[f32],
        acceleration_limits_rad_s2: &[f32],
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
        if velocity_limits_rad_s.len() != goal.len()
            || acceleration_limits_rad_s2.len() != goal.len()
            || velocity_limits_rad_s
                .iter()
                .chain(acceleration_limits_rad_s2)
                .any(|limit| !limit.is_finite() || *limit <= 0.0)
        {
            anyhow::bail!("velocity/acceleration limit shape or value is invalid");
        }

        let start = self.sample(now_ns);
        for ((current, target), velocity_limit) in
            start.iter().zip(&goal).zip(velocity_limits_rad_s)
        {
            anyhow::ensure!(
                current.velocity_rad_s.abs() <= *velocity_limit,
                "current interpolated velocity exceeds the new velocity limit"
            );
            anyhow::ensure!(
                target.velocity_rad_s.abs() <= *velocity_limit,
                "goal velocity exceeds the velocity limit"
            );
        }

        let mut duration_ns = requested_duration_ns;
        for (((current, target), velocity_limit), acceleration_limit) in start
            .iter()
            .zip(&goal)
            .zip(velocity_limits_rad_s)
            .zip(acceleration_limits_rad_s2)
        {
            duration_ns = duration_ns.max(minimum_duration_ns(
                *current,
                *target,
                *velocity_limit,
                *acceleration_limit,
            )?);
        }

        // Verify the common six-axis duration instead of assuming that a
        // larger duration remains safe after future trajectory changes.
        loop {
            let all_safe = start
                .iter()
                .zip(&goal)
                .zip(velocity_limits_rad_s)
                .zip(acceleration_limits_rad_s2)
                .all(
                    |(((current, target), velocity_limit), acceleration_limit)| {
                        segment_respects_limits(
                            *current,
                            *target,
                            duration_ns,
                            *velocity_limit,
                            *acceleration_limit,
                        )
                    },
                );
            if all_safe {
                break;
            }
            duration_ns = duration_ns
                .checked_mul(2)
                .filter(|duration| *duration > 0)
                .ok_or_else(|| anyhow::anyhow!("no finite safe interpolation duration"))?;
        }

        self.start = start;
        self.goal = goal;
        self.started_ns = now_ns;
        self.duration_ns = duration_ns;
        self.velocity_limits_rad_s = velocity_limits_rad_s.to_vec();
        self.segments = self
            .start
            .iter()
            .copied()
            .zip(self.goal.iter().copied())
            .map(|(start, goal)| CubicSegment::new(start, goal, duration_ns))
            .collect();
        Ok(())
    }

    pub fn sample(&self, now_ns: u64) -> Vec<RosTarget> {
        let elapsed_ns = now_ns.saturating_sub(self.started_ns).min(self.duration_ns);
        let elapsed_sec = elapsed_ns as f64 / NS_PER_SECOND;
        let alpha = if self.duration_ns == 0 {
            1.0
        } else {
            elapsed_ns as f32 / self.duration_ns as f32
        };
        self.start
            .iter()
            .zip(&self.goal)
            .zip(&self.segments)
            .enumerate()
            .map(|(index, ((start, goal), segment))| {
                let (position, velocity, _) = segment.sample(elapsed_sec);
                let velocity = self
                    .velocity_limits_rad_s
                    .get(index)
                    .map_or(velocity as f32, |limit| {
                        (velocity as f32).clamp(-*limit, *limit)
                    });
                RosTarget {
                    position_rad: position as f32,
                    velocity_rad_s: velocity,
                    torque_nm: lerp(start.torque_nm, goal.torque_nm, alpha),
                    kp_nm_rad: lerp(start.kp_nm_rad, goal.kp_nm_rad, alpha),
                    kd_nm_s_rad: lerp(start.kd_nm_s_rad, goal.kd_nm_s_rad, alpha),
                }
            })
            .collect()
    }

    #[cfg(test)]
    pub(crate) fn segment_bounds(&self, index: usize) -> (f64, f64) {
        self.segments[index].maximum_absolute_velocity_and_acceleration()
    }

    #[cfg(test)]
    pub(crate) fn sample_acceleration_rad_s2(&self, index: usize, now_ns: u64) -> f64 {
        let elapsed_ns = now_ns.saturating_sub(self.started_ns).min(self.duration_ns);
        self.segments[index]
            .sample(elapsed_ns as f64 / NS_PER_SECOND)
            .2
    }
}

fn segment_respects_limits(
    start: RosTarget,
    goal: RosTarget,
    duration_ns: u64,
    velocity_limit_rad_s: f32,
    acceleration_limit_rad_s2: f32,
) -> bool {
    if duration_ns == 0
        && (start.position_rad != goal.position_rad || start.velocity_rad_s != goal.velocity_rad_s)
    {
        return false;
    }
    let (maximum_velocity, maximum_acceleration) =
        CubicSegment::new(start, goal, duration_ns).maximum_absolute_velocity_and_acceleration();
    maximum_velocity <= velocity_limit_rad_s as f64 * (1.0 + BOUND_ROUNDOFF)
        && maximum_acceleration <= acceleration_limit_rad_s2 as f64 * (1.0 + BOUND_ROUNDOFF)
}

fn minimum_duration_ns(
    start: RosTarget,
    goal: RosTarget,
    velocity_limit_rad_s: f32,
    acceleration_limit_rad_s2: f32,
) -> anyhow::Result<u64> {
    if start.position_rad == goal.position_rad && start.velocity_rad_s == goal.velocity_rad_s {
        return Ok(0);
    }

    let mut safe_ns = 1_u64;
    while !segment_respects_limits(
        start,
        goal,
        safe_ns,
        velocity_limit_rad_s,
        acceleration_limit_rad_s2,
    ) {
        safe_ns = safe_ns
            .checked_mul(2)
            .ok_or_else(|| anyhow::anyhow!("no finite safe interpolation duration"))?;
    }

    let mut unsafe_ns = safe_ns / 2;
    while unsafe_ns + 1 < safe_ns {
        let midpoint = unsafe_ns + (safe_ns - unsafe_ns) / 2;
        if segment_respects_limits(
            start,
            goal,
            midpoint,
            velocity_limit_rad_s,
            acceleration_limit_rad_s2,
        ) {
            safe_ns = midpoint;
        } else {
            unsafe_ns = midpoint;
        }
    }
    Ok(safe_ns)
}

fn lerp(a: f32, b: f32, alpha: f32) -> f32 {
    a + (b - a) * alpha
}

#[cfg(test)]
mod tests {
    use super::*;

    fn target(position_rad: f32, velocity_rad_s: f32) -> RosTarget {
        RosTarget {
            position_rad,
            velocity_rad_s,
            ..Default::default()
        }
    }

    #[test]
    fn rest_to_rest_segment_has_analytically_proven_bounds() {
        let mut interpolator = Interpolator::hold(vec![target(0.0, 0.0)], 0);
        interpolator
            .retarget_with_limits(vec![target(1.0, 0.0)], 0, 0, &[0.2], &[0.1])
            .unwrap();
        let (maximum_velocity, maximum_acceleration) = interpolator.segment_bounds(0);
        assert!(maximum_velocity <= 0.2_f32 as f64 * (1.0 + BOUND_ROUNDOFF));
        assert!(maximum_acceleration <= 0.1_f32 as f64 * (1.0 + BOUND_ROUNDOFF));
        assert!(interpolator.duration_ns >= 7_745_000_000);
        assert_eq!(interpolator.sample(0)[0], target(0.0, 0.0));
        assert!((interpolator.sample(interpolator.duration_ns)[0].position_rad - 1.0).abs() < 1e-6);
    }

    #[test]
    fn repeated_retarget_preserves_position_and_velocity_without_exceeding_limits() {
        let mut interpolator = Interpolator::hold(vec![target(0.0, 0.0)], 0);
        interpolator
            .retarget_with_limits(vec![target(1.0, 0.1)], 0, 0, &[0.3], &[0.2])
            .unwrap();
        let retarget_ns = 700_000_000;
        let before = interpolator.sample(retarget_ns)[0];
        let old_acceleration = interpolator.sample_acceleration_rad_s2(0, retarget_ns);
        interpolator
            .retarget_with_limits(vec![target(-0.5, -0.1)], retarget_ns, 0, &[0.3], &[0.2])
            .unwrap();
        let after = interpolator.sample(retarget_ns)[0];
        let new_acceleration = interpolator.sample_acceleration_rad_s2(0, retarget_ns);
        let (maximum_velocity, maximum_acceleration) = interpolator.segment_bounds(0);
        assert!((after.position_rad - before.position_rad).abs() < 1e-6);
        assert!((after.velocity_rad_s - before.velocity_rad_s).abs() < 1e-6);
        assert!(old_acceleration.abs() <= 0.2_f32 as f64 * (1.0 + BOUND_ROUNDOFF));
        assert!(new_acceleration.abs() <= 0.2_f32 as f64 * (1.0 + BOUND_ROUNDOFF));
        assert!(maximum_velocity <= 0.3_f32 as f64 * (1.0 + BOUND_ROUNDOFF));
        assert!(maximum_acceleration <= 0.2_f32 as f64 * (1.0 + BOUND_ROUNDOFF));
    }

    #[test]
    fn zero_duration_position_change_is_stretched_by_both_limits() {
        let mut interpolator = Interpolator::hold(vec![target(0.0, 0.0)], 0);
        interpolator
            .retarget_with_limits(vec![target(0.01, 0.0)], 0, 0, &[0.1], &[0.1])
            .unwrap();
        assert!(interpolator.duration_ns > 0);
        assert_eq!(interpolator.sample(0)[0].position_rad, 0.0);
        let (maximum_velocity, maximum_acceleration) = interpolator.segment_bounds(0);
        assert!(maximum_velocity <= 0.1_f32 as f64 * (1.0 + BOUND_ROUNDOFF));
        assert!(maximum_acceleration <= 0.1_f32 as f64 * (1.0 + BOUND_ROUNDOFF));
    }

    #[test]
    fn requested_duration_is_kept_when_it_is_already_safe() {
        let mut interpolator = Interpolator::hold(vec![target(0.0, 0.0)], 0);
        interpolator
            .retarget_with_limits(vec![target(0.01, 0.0)], 0, 2_000_000_000, &[0.1], &[0.1])
            .unwrap();
        assert_eq!(interpolator.duration_ns, 2_000_000_000);
    }

    #[test]
    fn exact_velocity_boundary_and_clamped_time_are_safe() {
        let boundary = target(0.0, 0.2);
        let mut interpolator = Interpolator::hold(vec![boundary], 100);
        interpolator
            .retarget_with_limits(vec![target(0.2, 0.2)], 100, 1_000_000_000, &[0.2], &[0.1])
            .unwrap();
        assert_eq!(interpolator.sample(50)[0], boundary);
        let (maximum_velocity, maximum_acceleration) = interpolator.segment_bounds(0);
        assert!(maximum_velocity <= 0.2_f32 as f64 * (1.0 + BOUND_ROUNDOFF));
        assert!(maximum_acceleration <= 0.1_f32 as f64 * (1.0 + BOUND_ROUNDOFF));
    }

    #[test]
    fn rejects_invalid_limits_and_goal_values() {
        let finite = target(0.0, 0.0);
        let mut interpolator = Interpolator::hold(vec![finite], 0);
        for (velocity, acceleration) in [
            (vec![], vec![0.1]),
            (vec![0.1], vec![]),
            (vec![0.0], vec![0.1]),
            (vec![0.1], vec![0.0]),
            (vec![f32::NAN], vec![0.1]),
            (vec![0.1], vec![f32::INFINITY]),
        ] {
            assert!(interpolator
                .retarget_with_limits(vec![finite], 0, 0, &velocity, &acceleration)
                .is_err());
        }
        assert!(interpolator
            .retarget_with_limits(vec![target(f32::NAN, 0.0)], 0, 0, &[0.1], &[0.1])
            .is_err());
        assert!(interpolator
            .retarget_with_limits(vec![target(0.0, 0.2)], 0, 0, &[0.1], &[0.1])
            .is_err());
    }
}
