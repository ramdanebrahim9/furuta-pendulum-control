"""ctypes wrapper around libnmpc_solver.so, the generated nonlinear-MPC solver
(hold-up). Same interface as cmpc_solver.py.

    solver = NmpcSolver()
    tau, status, t_us = solver.solve([theta, alpha, theta_dot, alpha_dot], u_prev)

tau    torque [N.m], already limited to the actuator bound
status 0 means success; anything else is a solver fault
t_us   solve time on the Pi [microseconds]
"""
import ctypes
import os

_HERE = os.path.dirname(os.path.abspath(__file__))


class NmpcSolver:
    def __init__(self, lib_path=None):
        lib_path = lib_path or os.path.join(_HERE, "libnmpc_solver.so")
        self._lib = ctypes.CDLL(lib_path)

        self._lib.nmpc_init.restype = ctypes.c_int
        self._lib.nmpc_reset.restype = ctypes.c_int
        self._lib.nmpc_free.restype = None
        self._lib.nmpc_n_horizon.restype = ctypes.c_int
        self._lib.nmpc_n_states.restype = ctypes.c_int
        self._lib.nmpc_tau_max.restype = ctypes.c_double
        self._lib.nmpc_solve.restype = ctypes.c_int
        self._lib.nmpc_solve.argtypes = [
            ctypes.POINTER(ctypes.c_double),   # x[NX]
            ctypes.c_double,                   # u_prev
            ctypes.POINTER(ctypes.c_double),   # tau_out
            ctypes.POINTER(ctypes.c_int),      # status
            ctypes.POINTER(ctypes.c_double),   # t_us
        ]

        st = self._lib.nmpc_init()
        if st != 0:
            raise RuntimeError(f"nmpc_init failed, acados status {st}")

        self.n_horizon = self._lib.nmpc_n_horizon()
        self.n_states = self._lib.nmpc_n_states()
        self.tau_max = self._lib.nmpc_tau_max()

        # preallocated buffers: no per-tick allocation in the loop
        self._x = (ctypes.c_double * self.n_states)()
        self._tau = ctypes.c_double()
        self._status = ctypes.c_int()
        self._t_us = ctypes.c_double()

    def solve(self, x, u_prev):
        for i in range(self.n_states):
            self._x[i] = x[i]
        self._lib.nmpc_solve(self._x, float(u_prev),
                             ctypes.byref(self._tau),
                             ctypes.byref(self._status),
                             ctypes.byref(self._t_us))
        return self._tau.value, self._status.value, self._t_us.value



    def reset(self):
        """Clears the warm start: call for a fresh start or after a fault."""
        return self._lib.nmpc_reset()

    def close(self):
        self._lib.nmpc_free()
