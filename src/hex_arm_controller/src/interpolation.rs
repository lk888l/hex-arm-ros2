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

        let duration_ns = safe_common_duration_ns(
            &start,
            &goal,
            requested_duration_ns,
            velocity_limits_rad_s,
            acceleration_limits_rad_s2,
        )?;

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

/// Feasibility is not monotone in duration when endpoint velocities are
/// nonzero: a 10 ms constant-velocity segment can be safe while 16 ms is
/// unsafe, and only become safe again near 3 s. Doubling and binary searching
/// therefore skips short feasible intervals and introduces seconds of lag.
/// The exact cubic bounds change feasibility only at the polynomial roots
/// below. Search their integer-nanosecond neighbours jointly for all axes.
fn safe_common_duration_ns(
    start: &[RosTarget],
    goal: &[RosTarget],
    requested_ns: u64,
    velocity_limits: &[f32],
    acceleration_limits: &[f32],
) -> anyhow::Result<u64> {
    let all_safe = |duration_ns| {
        start
            .iter()
            .zip(goal)
            .zip(velocity_limits)
            .zip(acceleration_limits)
            .all(|(((current, target), velocity), acceleration)| {
                segment_respects_limits(*current, *target, duration_ns, *velocity, *acceleration)
            })
    };
    if all_safe(requested_ns) {
        return Ok(requested_ns);
    }
    let mut candidates = vec![1];
    for (((current, target), velocity), acceleration) in start
        .iter()
        .zip(goal)
        .zip(velocity_limits)
        .zip(acceleration_limits)
    {
        let displacement = target.position_rad as f64 - current.position_rad as f64;
        let u = current.velocity_rad_s as f64;
        let v = target.velocity_rad_s as f64;
        for bound in [-(*acceleration as f64), *acceleration as f64] {
            // a(0) = 6d/T² - (4u+2v)/T; a(T) = -6d/T² + (2u+4v)/T.
            duration_roots(
                bound,
                4.0 * u + 2.0 * v,
                -6.0 * displacement,
                &mut candidates,
            );
            duration_roots(
                bound,
                -2.0 * u - 4.0 * v,
                6.0 * displacement,
                &mut candidates,
            );
        }
        for bound in [-(*velocity as f64), *velocity as f64] {
            // Interior velocity extremum u - c2²/(3c3) = bound.
            // Extra roots with the extremum outside [0,T] are harmless:
            // every candidate is checked against the exact continuous bounds.
            duration_roots(
                3.0 * (u + v) * (u - bound) - (2.0 * u + v).powi(2),
                6.0 * displacement * (u + v + bound),
                -9.0 * displacement.powi(2),
                &mut candidates,
            );
        }
    }
    candidates.sort_unstable();
    candidates.dedup();
    candidates
        .into_iter()
        .find(|duration| *duration >= requested_ns && all_safe(*duration))
        .ok_or_else(|| anyhow::anyhow!("no finite safe interpolation duration"))
}

fn duration_roots(a: f64, b: f64, c: f64, candidates: &mut Vec<u64>) {
    let mut add = |seconds: f64| {
        let nanoseconds = seconds * NS_PER_SECOND;
        if nanoseconds.is_finite() && nanoseconds > 0.0 && nanoseconds < u64::MAX as f64 {
            let below = nanoseconds.floor() as u64;
            // Check both sides of a root and absorb floating-point rounding.
            for delta in 0..=2 {
                candidates.push(below.saturating_add(delta));
            }
        }
    };
    if a == 0.0 {
        if b != 0.0 {
            add(-c / b);
        }
        return;
    }
    let discriminant = b.mul_add(b, -4.0 * a * c);
    if discriminant < 0.0 {
        return;
    }
    if discriminant == 0.0 {
        add(-b / (2.0 * a));
        return;
    }
    // Stable quadratic formula avoids cancellation for very short segments.
    let q = -0.5 * (b + discriminant.sqrt().copysign(b));
    add(q / a);
    if q != 0.0 {
        add(c / q);
    }
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
    fn full_range_stream_with_planning_acceleration_headroom_tracks_under_jitter() {
        let initial = [0.65_f32, -0.6, 0.7, -0.4, 0.25, 0.4];
        let final_q = [-0.65_f32, -0.8, 0.85, -0.4, -0.25, -0.4];
        let hardware_acceleration = 1.2566370614359172_f64 as f32;
        let intervals = [
            10_000_000, 12_000_000, 8_000_000, 10_000_000, 11_000_000, 9_000_000,
        ];
        for (hardware_velocity, planning_acceleration) in [
            (hardware_acceleration, 0.6_f64),
            (1.6755160819145563_f64 as f32, 0.75_f64),
            (2.2340214425527414_f64 as f32, 0.9375_f64),
        ] {
            let ramp_sec = (1.3 / planning_acceleration).sqrt();
            let total_sec = 2.0 * ramp_sec;
            let mut interpolator =
                Interpolator::hold(initial.iter().map(|q| target(*q, 0.0)).collect(), 0);
            let mut now_ns = 0_u64;
            let mut maximum_error = 0.0_f32;
            for tick in 0..450 {
                now_ns += intervals[tick % intervals.len()];
                let t = (now_ns as f64 / NS_PER_SECOND).min(total_sec);
                let (distance, speed) = if t < ramp_sec {
                    (
                        0.5 * planning_acceleration * t * t,
                        planning_acceleration * t,
                    )
                } else {
                    let remaining = total_sec - t;
                    (
                        1.3 - 0.5 * planning_acceleration * remaining * remaining,
                        planning_acceleration * remaining,
                    )
                };
                let goals: Vec<_> = initial
                    .iter()
                    .zip(final_q)
                    .map(|(start, end)| {
                        let delta = (end - start) as f64;
                        target(
                            (*start as f64 + delta * distance / 1.3) as f32,
                            (delta * speed / 1.3) as f32,
                        )
                    })
                    .collect();
                interpolator
                    .retarget_with_limits(
                        goals.clone(),
                        now_ns,
                        10_000_000,
                        &[hardware_velocity; 6],
                        &[hardware_acceleration; 6],
                    )
                    .unwrap();
                for (index, (actual, desired)) in
                    interpolator.sample(now_ns).iter().zip(&goals).enumerate()
                {
                    maximum_error =
                        maximum_error.max((actual.position_rad - desired.position_rad).abs());
                    let (velocity, acceleration) = interpolator.segment_bounds(index);
                    assert!(velocity <= hardware_velocity as f64 * (1.0 + BOUND_ROUNDOFF));
                    assert!(acceleration <= hardware_acceleration as f64 * (1.0 + BOUND_ROUNDOFF));
                }
            }
            assert!(
                maximum_error < 0.025,
                "stream error was {maximum_error} rad with planning acceleration {planning_acceleration} rad/s^2"
            );
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
    fn safe_ten_millisecond_constant_velocity_segment_keeps_its_duration() {
        let mut interpolator = Interpolator::hold(vec![target(0.0, 0.05)], 0);
        interpolator
            .retarget_with_limits(vec![target(0.0005, 0.05)], 0, 10_000_000, &[0.1], &[0.1])
            .unwrap();
        assert_eq!(interpolator.duration_ns, 10_000_000);
    }

    #[test]
    fn short_feasible_window_is_found_before_the_long_reverse_motion_solution() {
        let mut interpolator = Interpolator::hold(vec![target(0.0, 0.05)], 0);
        interpolator
            .retarget_with_limits(vec![target(0.0005, 0.05)], 0, 1, &[0.1], &[0.1])
            .unwrap();
        assert!((9_000_000..=10_000_000).contains(&interpolator.duration_ns));
        let (velocity, acceleration) = interpolator.segment_bounds(0);
        assert!(velocity <= 0.1_f32 as f64 * (1.0 + BOUND_ROUNDOFF));
        assert!(acceleration <= 0.1_f32 as f64 * (1.0 + BOUND_ROUNDOFF));
    }

    #[test]
    fn multi_axis_stream_with_jitter_tracks_without_relaxing_continuous_bounds() {
        let initial = [-0.0001, -1.568, 1.566, 0.001, 0.006, -0.00006];
        let final_q = [0.0, -1.35, 1.57, 0.0, 0.0, 0.0];
        let mut interpolator =
            Interpolator::hold(initial.iter().map(|q| target(*q, 0.0)).collect(), 0);
        let mut now_ns = 0_u64;
        let mut maximum_error = 0.0_f32;
        let intervals = [
            10_000_000, 12_000_000, 8_000_000, 10_000_000, 11_000_000, 9_000_000,
        ];
        for tick in 0..1000 {
            now_ns += intervals[tick % intervals.len()];
            let t = (now_ns as f64 / NS_PER_SECOND / 8.0).min(1.0);
            let alpha = 10.0 * t.powi(3) - 15.0 * t.powi(4) + 6.0 * t.powi(5);
            let rate = (30.0 * t.powi(2) - 60.0 * t.powi(3) + 30.0 * t.powi(4)) / 8.0;
            let goal: Vec<_> = initial
                .iter()
                .zip(final_q)
                .map(|(a, b)| {
                    target(
                        (*a as f64 + (b - a) as f64 * alpha) as f32,
                        ((b - a) as f64 * rate) as f32,
                    )
                })
                .collect();
            let before = interpolator.sample(now_ns);
            maximum_error =
                maximum_error.max((before[1].position_rad - goal[1].position_rad).abs());
            interpolator
                .retarget_with_limits(goal, now_ns, 10_000_000, &[0.1; 6], &[0.1; 6])
                .unwrap();
            let after = interpolator.sample(now_ns);
            for axis in 0..6 {
                assert!((before[axis].position_rad - after[axis].position_rad).abs() < 1e-6);
                assert!((before[axis].velocity_rad_s - after[axis].velocity_rad_s).abs() < 1e-6);
                let (velocity, acceleration) = interpolator.segment_bounds(axis);
                assert!(velocity <= 0.1_f32 as f64 * (1.0 + BOUND_ROUNDOFF));
                assert!(acceleration <= 0.1_f32 as f64 * (1.0 + BOUND_ROUNDOFF));
            }
        }
        assert!(maximum_error < 0.01, "maximum stream lag {maximum_error}");
        assert!((interpolator.sample(now_ns)[1].position_rad + 1.35).abs() < 1e-4);
    }

    #[test]
    fn common_duration_search_does_not_skip_feasible_intervals_in_dense_oracle() {
        let mut seed = 17_u64;
        let mut random = || {
            seed = seed.wrapping_mul(6364136223846793005).wrapping_add(1);
            ((seed >> 32) as u32) as f32 / u32::MAX as f32
        };
        for _ in 0..200 {
            let start: Vec<_> = (0..6)
                .map(|_| target(0.0, (random() - 0.5) * 0.18))
                .collect();
            let goal: Vec<_> = (0..6)
                .map(|_| target((random() - 0.5) * 0.04, (random() - 0.5) * 0.18))
                .collect();
            let requested = 1_000_000;
            let selected =
                safe_common_duration_ns(&start, &goal, requested, &[0.1; 6], &[0.1; 6]).unwrap();
            assert!(start
                .iter()
                .zip(&goal)
                .all(|(s, g)| segment_respects_limits(*s, *g, selected, 0.1, 0.1)));
            let mut probe = requested;
            while probe < selected {
                assert!(
                    !start
                        .iter()
                        .zip(&goal)
                        .all(|(s, g)| segment_respects_limits(*s, *g, probe, 0.1, 0.1)),
                    "missed feasible duration {probe} before {selected}"
                );
                probe = (probe as f64 * 1.01).ceil() as u64;
            }
        }
    }

    #[test]
    fn fixed_rate_ros_stream_with_independent_receive_jitter_tracks_j4() {
        // ROS samples at 100 Hz; its timestamp does not follow delivery delay.
        for seed in 0..32_u64 {
            let mut random = seed;
            let mut interpolator = Interpolator::hold(vec![target(0.0, 0.0)], 0);
            let mut maximum_error = 0.0_f32;
            let mut longest_segment = 0;
            for tick in 1..=1200 {
                let source_ns = tick * 10_000_000_u64;
                random = random.wrapping_mul(6364136223846793005).wrapping_add(1);
                let now_ns = source_ns + (random >> 32) % 5 * 2_000_000;
                let t = (source_ns as f64 / NS_PER_SECOND / 10.0).min(1.0);
                let alpha = 10.0 * t.powi(3) - 15.0 * t.powi(4) + 6.0 * t.powi(5);
                let rate = (30.0 * t.powi(2) - 60.0 * t.powi(3) + 30.0 * t.powi(4)) / 10.0;
                let goal = target((-0.3 * alpha) as f32, (-0.3 * rate) as f32);
                maximum_error = maximum_error
                    .max((interpolator.sample(now_ns)[0].position_rad - goal.position_rad).abs());
                interpolator
                    .retarget_with_limits(vec![goal], now_ns, 10_000_000, &[0.1], &[0.1])
                    .unwrap();
                longest_segment = longest_segment.max(interpolator.duration_ns);
                let (velocity, acceleration) = interpolator.segment_bounds(0);
                assert!(velocity <= 0.1_f32 as f64 * (1.0 + BOUND_ROUNDOFF));
                assert!(acceleration <= 0.1_f32 as f64 * (1.0 + BOUND_ROUNDOFF));
            }
            assert!(maximum_error < 0.005,
            "seed {seed}: maximum stream lag {maximum_error}, longest segment {longest_segment} ns");
        }
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
