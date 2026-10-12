"""Constrained MPC (hold-up), the Pi side of Controller_Remote(cMPC).

Wraps the generated solver behind one call, compute(state, u_prev) -> torque.

The solver works in velocity form: it plans torque increments, so it needs the
previous torque as part of its state. That torque is not remembered here. The
STM32 sends the torque measured on the motor with every request (u_prev), so the
solver always starts from what the motor really received, including after a
missed reply or a saturated command.
"""
from solvers.cmpc.cmpc_solver import CmpcSolver


class Controller_cMPC:
    def __init__(self):
        self._solver = CmpcSolver()
        print(f"[cMPC] loaded: {self._solver.n_horizon} nodes, {self._solver.n_states} states")

    def compute(self, state: list[float], u_prev: float) -> float:
        """state: [theta, alpha, theta_dot, alpha_dot]; u_prev: measured torque [N.m].
        Returns the torque [N.m], already limited by the solver."""
        tau, status, t_us = self._solver.solve(state, u_prev)
        if status != 0:
            # The solver failed: hold the previous torque instead of applying a
            # bad solution.
            print(f"[cMPC] solver fault, status={status}, t={t_us:.0f} us: holding the previous torque")
            return u_prev
        return tau

    def reset(self):
        """Clears the solver's warm start, for a fresh start or after a fault."""
        self._solver.reset()

    def close(self):
        self._solver.close()
