"""Future solver interface. No actuator commands are implemented here."""


class NmpcOptimizer:
    def __init__(self, model, config):
        self.model = model
        self.config = config

    def solve(self, state, reference):
        raise NotImplementedError(
            "NMPC dynamics, constraints and solver must be implemented and validated"
        )
