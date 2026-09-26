"""Input checks for read-only tracking experiments."""
import math


def ordered_joint_positions(message, joint_names):
    if len(message.name) != len(set(message.name)):
        return None
    positions = dict(zip(message.name, message.position))
    try:
        q = tuple(float(positions[name]) for name in joint_names)
    except (KeyError, TypeError, ValueError):
        return None
    return q if all(math.isfinite(value) for value in q) else None


def position_error_norm(measured, desired):
    if len(measured) != len(desired):
        raise ValueError("Measured and desired vectors differ in length")
    return math.sqrt(sum((a - b) ** 2 for a, b in zip(measured, desired)))
