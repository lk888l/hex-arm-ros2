"""Lossless trajectory/feedback records; all times identify their clock."""
import time


def point_record(point):
    return {
        "positions": list(point.positions), "velocities": list(point.velocities),
        "accelerations": list(point.accelerations), "effort": list(point.effort),
        "time_from_start_ns": point.time_from_start.sec * 1_000_000_000 + point.time_from_start.nanosec,
    }


def trajectory_record(trajectory):
    return {"joint_names": list(trajectory.joint_names),
            "header_stamp_ns": trajectory.header.stamp.sec * 1_000_000_000 + trajectory.header.stamp.nanosec,
            "points": [point_record(point) for point in trajectory.points]}


def feedback_record(feedback, label):
    desired, actual, error = (point_record(getattr(feedback, name))
                              for name in ("desired", "actual", "error"))
    return {"t": time.monotonic(), "step": label,
            "joint_names": list(feedback.joint_names),
            "header_stamp_ns": feedback.header.stamp.sec * 1_000_000_000 + feedback.header.stamp.nanosec,
            # Retain existing report keys for analysis scripts.
            "desired": desired["positions"], "desired_velocity": desired["velocities"],
            "actual": actual["positions"], "actual_velocity": actual["velocities"],
            "error": error["positions"], "velocity_error": error["velocities"],
            "points": {"desired": desired, "actual": actual, "error": error}}


def feedback_summary(samples, joint_names):
    summary = {"feedback_samples": len(samples)}
    for source, key in (("error", "max_tracking_error_rad"),
                        ("velocity_error", "max_velocity_error_rad_s")):
        values = []
        for sample in samples:
            names = sample["joint_names"]
            if len(sample[source]) == len(names) == len(joint_names) and set(names) == set(joint_names):
                values.append([sample[source][names.index(name)] for name in joint_names])
        if values:
            summary[key] = [max(abs(value[i]) for value in values) for i in range(len(joint_names))]
    return summary
