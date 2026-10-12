"""Nonlinear MPC for swing-up, the Pi side of Controller_Remote(NMPC_Swingup).

The state is [theta, alpha, theta_dot, alpha_dot] and the control is the torque
itself (no previous-torque state), because swing-up needs free torque reversals.

There are two kinds of tick.

Cold start: cold_start(state), called on a reset. After a reset the STM32
applies zero torque for a fixed time T_HOLD_S and sends nothing, then applies
the one reply this method returns. So the solver plans from where the system
will be at the end of that hold:
    predicted = predict(state, zero torque, T_HOLD_S)
    torque    = several solver iterations from the predicted state
The Pi is busy for a while here; the STM32 is not sending frames, so nothing
queues up behind it.

Normal tick: compute(state, u_applied). The torque for the coming 5 ms is
already fixed, so the plan starts from the state predicted one tick ahead:
    predicted = predict(state, u_applied, TICK_S)
followed by a single solver iteration that continues from the previous plan.
u_applied is sent by the STM32 and not remembered here, because a local copy
would not see missed replies or holds.

T_HOLD_S must equal the STM32's cold-start hold. If they differ, the cold plan
starts from the wrong state and nothing reports it.
"""
import time

from solvers.nmpc_sw.nmpc_sw_solver import NmpcSwSolver

T_HOLD_S = 0.115  # zero-torque hold after a reset [s]; must match the STM32
TICK_S = 0.005    # outer-loop period [s]

# A cold start slower than this risks missing the end of the STM32's hold [ms].
COLD_WARN_MS = 135.0


class Controller_NMPC_SW:
    def __init__(self):
        self._solver = NmpcSwSolver()
        print(f"[NMPC swing-up] loaded: {self._solver.n_horizon} nodes, hold {1e3 * T_HOLD_S:.0f} ms")

    def cold_start(self, state: list[float]) -> float:
        t0 = time.perf_counter()
        self._solver.reset()
        predicted = self._solver.predict(state, 0.0, T_HOLD_S)
        tau, status, _ = self._solver.solve(predicted, self._solver.cold_iters_default)
        wall_ms = 1e3 * (time.perf_counter() - t0)

        if status != 0:
            print("[NMPC swing-up] cold start failed, replying with zero torque")
            self._solver.reset()
            tau = 0.0

        late = "  (close to the end of the hold)" if wall_ms > COLD_WARN_MS else ""
        print(f"[NMPC swing-up] cold start took {wall_ms:.1f} ms{late}")
        return tau

    def compute(self, state: list[float], u_applied: float) -> float:
        """state: [theta, alpha, theta_dot, alpha_dot]; u_applied: torque for the coming tick [N.m]."""
        predicted = self._solver.predict(state, u_applied, TICK_S)
        tau, status, t_us = self._solver.solve(predicted, 1)

        if status != 0:
            # A failed solve leaves a degraded plan. Drop it so the next tick
            # starts again from the energy-based guess, one iteration only (a
            # full cold solve would not fit in 5 ms).
            print(f"[NMPC swing-up] solver fault, status={status}, t={t_us:.0f} us: zero torque, re-seeding")
            self._solver.reset()
            tau = 0.0
        return tau

    def reset(self):
        self._solver.reset()

    def close(self):
        self._solver.close()
