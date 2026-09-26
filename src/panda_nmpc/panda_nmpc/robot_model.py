"""Robot model boundary for a future dynamics-based NMPC solver.

Use a validated Panda dynamics model here before enabling actuation. A simple
joint interpolator is not a substitute for the manipulator dynamics.
"""


class PandaDynamicsModel:
    def forward_dynamics(self, q, dq, torque):
        raise NotImplementedError("Connect a validated Panda dynamics model")
