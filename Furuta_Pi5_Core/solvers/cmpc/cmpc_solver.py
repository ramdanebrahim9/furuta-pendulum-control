"""ctypes wrapper around libcmpc_solver.so, the generated constrained-MPC solver.

    solver = CmpcSolver()                  # loads the library next to this file
    tau, status, t_us = solver.solve([theta, alpha, theta_dot, alpha_dot], u_prev)

tau    torque [N.m], already limited to the actuator bound
status 0 means success; anything else is a solver fault
t_us   solve time on the Pi [microseconds]
"""
import ctypes
import os

_HERE = os.path.dirname(os.path.abspath(__file__))


class CmpcSolver:
    def __init__(self, lib_path=None):
        lib_path = lib_path or os.path.join(_HERE, "libcmpc_solver.so")
        self._lib = ctypes.CDLL(lib_path)

        self._lib.cmpc_init.restype = ctypes.c_int
        self._lib.cmpc_reset.restype = ctypes.c_int
        self._lib.cmpc_free.restype = None
        self._lib.cmpc_n_horizon.restype = ctypes.c_int
        self._lib.cmpc_n_states.restype = ctypes.c_int
        self._lib.cmpc_tau_max.restype = ctypes.c_double
        self._lib.cmpc_solve.restype = ctypes.c_int
        self._lib.cmpc_solve.argtypes = [
            ctypes.POINTER(ctypes.c_double),   # x[NX]
            ctypes.c_double,                   # u_prev
            ctypes.POINTER(ctypes.c_double),   # tau_out
            ctypes.POINTER(ctypes.c_int),      # status
            ctypes.POINTER(ctypes.c_double),   # t_us
        ]

        st = self._lib.cmpc_init()
        if st != 0:
            raise RuntimeError(f"cmpc_init failed, acados status {st}")

        self.n_horizon = self._lib.cmpc_n_horizon()
        self.n_states = self._lib.cmpc_n_states()
        self.tau_max = self._lib.cmpc_tau_max()

        # preallocated buffers: no per-tick allocation in the loop
        self._x = (ctypes.c_double * self.n_states)()
        self._tau = ctypes.c_double()
        self._status = ctypes.c_int()
        self._t_us = ctypes.c_double()

    def solve(self, x, u_prev):
        for i in range(self.n_states):
            self._x[i] = x[i]
        self._lib.cmpc_solve(self._x, float(u_prev),
                             ctypes.byref(self._tau),
                             ctypes.byref(self._status),
                             ctypes.byref(self._t_us))
        return self._tau.value, self._status.value, self._t_us.value

    def reset(self):
        """Cold start: call on mode switch or after a fault reset."""
        return self._lib.cmpc_reset()

    def close(self):
        self._lib.cmpc_free()
