"""Nonlinear MPC for hold-up, the Pi side of Controller_Remote(NMPC_Stabilize).

Solves the same "stay upright" problem as the constrained MPC, but with the
nonlinear model instead of a linearisation. Like the constrained MPC it works
in velocity form and takes the previous torque from the STM32's measurement
rather than remembering it.
"""
from solvers.nmpc.nmpc_solver import NmpcSolver


class Controller_NMPC:
    def __init__(self):
        self._solver = NmpcSolver()
        print(f"[NMPC] loaded: {self._solver.n_horizon} nodes, {self._solver.n_states} states")

    def compute(self, state: list[float], u_prev: float) -> float:
        """state: [theta, alpha, theta_dot, alpha_dot]; u_prev: measured torque [N.m].
        Returns the torque [N.m], already limited by the solver."""
        tau, status, t_us = self._solver.solve(state, u_prev)
        if status != 0:
            # The solver failed: hold the previous torque instead of applying a
            # bad solution.
            print(f"[NMPC] solver fault, status={status}, t={t_us:.0f} us: holding the previous torque")
            return u_prev
        return tau

    def reset(self):
        """Clears the solver's warm start, for a fresh start or after a fault."""
        self._solver.reset()

    def close(self):
        self._solver.close()
