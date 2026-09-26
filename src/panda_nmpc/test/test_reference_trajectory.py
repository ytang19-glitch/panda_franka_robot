"""Tests use small message-shaped objects, without a running ROS graph."""
from types import SimpleNamespace

import pytest

from panda_nmpc.reference_trajectory import ReferenceTrajectory


JOINTS = [f"panda_joint{i}" for i in range(1, 8)]


def point(seconds, positions):
    return SimpleNamespace(
        time_from_start=SimpleNamespace(sec=seconds, nanosec=0),
        positions=positions,
    )


def test_reorders_and_interpolates_joint_positions():
    reference = ReferenceTrajectory(JOINTS)
    trajectory = SimpleNamespace(
        joint_names=list(reversed(JOINTS)),
        points=[point(0, [0.] * 7), point(2, [float(i) for i in range(7)])],
    )
    reference.set_trajectory(trajectory)
    assert reference.sample(1) == tuple(float(i) / 2 for i in reversed(range(7)))
    assert reference.sample(3) == reference.sample(2)


def test_rejects_nonincreasing_times():
    reference = ReferenceTrajectory(JOINTS)
    trajectory = SimpleNamespace(
        joint_names=JOINTS,
        points=[point(1, [0.] * 7), point(1, [1.] * 7)],
    )
    with pytest.raises(ValueError, match="strictly increasing"):
        reference.set_trajectory(trajectory)
