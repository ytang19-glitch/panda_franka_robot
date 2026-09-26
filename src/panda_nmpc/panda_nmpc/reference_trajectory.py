"""Validate and interpolate a MoveIt JointTrajectory for tracking diagnostics."""
import math


class ReferenceTrajectory:
    def __init__(self, joint_names):
        self.joint_names = tuple(joint_names)
        self.samples = ()

    def set_trajectory(self, trajectory):
        if len(self.joint_names) != 7 or len(set(self.joint_names)) != 7:
            raise ValueError("Expected seven distinct Panda arm joints")
        if (len(trajectory.joint_names) != 7 or
                set(trajectory.joint_names) != set(self.joint_names)):
            raise ValueError("Trajectory must contain exactly the seven Panda arm joints")
        if len(trajectory.points) < 2:
            raise ValueError("Trajectory requires at least two points")
        indices = [trajectory.joint_names.index(name) for name in self.joint_names]
        samples = []
        for point in trajectory.points:
            if len(point.positions) != 7:
                raise ValueError("Each point requires seven positions")
            t = point.time_from_start.sec + point.time_from_start.nanosec * 1e-9
            q = tuple(float(point.positions[i]) for i in indices)
            if not math.isfinite(t) or not all(map(math.isfinite, q)):
                raise ValueError("Trajectory contains nonfinite values")
            if samples and t <= samples[-1][0]:
                raise ValueError("Trajectory times must be strictly increasing")
            if t < 0:
                raise ValueError("Trajectory times must be nonnegative")
            samples.append((t, q))
        self.samples = tuple(samples)
        return self.duration

    @property
    def duration(self):
        return self.samples[-1][0] if self.samples else 0.0

    def sample(self, t):
        if not self.samples:
            raise ValueError("No reference trajectory loaded")
        if not math.isfinite(t):
            raise ValueError("Sample time must be finite")
        if t <= self.samples[0][0]:
            return self.samples[0][1]
        if t >= self.duration:
            return self.samples[-1][1]
        for (t0, q0), (t1, q1) in zip(self.samples, self.samples[1:]):
            if t <= t1:
                alpha = (t - t0) / (t1 - t0)
                return tuple(a + alpha * (b - a) for a, b in zip(q0, q1))
        return self.samples[-1][1]
