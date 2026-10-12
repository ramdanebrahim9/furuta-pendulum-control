"""ctypes wrapper around libnmpc_sw_solver.so, the generated swing-up NMPC solver.

    solver = NmpcSwSolver()
    solver.reset()                              # at a reset
    tau, status, t_us = solver.solve(x, iters=solver.cold_iters_default)   # first (cold) tick
    tau, status, t_us = solver.solve(x, iters=1)                           # every tick after

tau    torque [N.m], already limited to the actuator bound
status 0 means success; anything else is a solver fault
t_us   total solve time over `iters` iterations [microseconds]

cold_iters_default is the iteration count the design suggests for the first
tick. It is only a suggestion: solve() runs exactly `iters` iterations.

solve() uses the state it is given as is. To account for the delay of one tick,
predict first:
    x0 = solver.predict(x, tau, dt)    # plant integrated for dt with tau held
"""
import ctypes
import os

_HERE = os.path.dirname(os.path.abspath(__file__))

class NmpcSwSolver:
    def __init__(self, lib_path=None):
        lib_path = lib_path or os.path.join(_HERE, "libnmpc_sw_solver.so")
        self._lib = ctypes.CDLL(lib_path)

        self._lib.nmpc_sw_init.restype = ctypes.c_int
        self._lib.nmpc_sw_reset.restype = ctypes.c_int
        self._lib.nmpc_sw_free.restype = None
        self._lib.nmpc_sw_n_horizon.restype = ctypes.c_int
        self._lib.nmpc_sw_n_states.restype = ctypes.c_int
        self._lib.nmpc_sw_tau_max.restype = ctypes.c_double
        self._lib.nmpc_sw_cold_iters_default.restype = ctypes.c_int
        self._lib.nmpc_sw_solve.restype = ctypes.c_int
        self._lib.nmpc_sw_solve.argtypes = [
            ctypes.POINTER(ctypes.c_double),   # x[NX]
            ctypes.c_int,                      # iters
            ctypes.POINTER(ctypes.c_double),   # tau_out
            ctypes.POINTER(ctypes.c_int),      # status
            ctypes.POINTER(ctypes.c_double),   # t_us
        ]
        self._lib.nmpc_sw_predict.restype = ctypes.c_int
        self._lib.nmpc_sw_predict.argtypes = [
            ctypes.POINTER(ctypes.c_double),   # x[NX]
            ctypes.c_double,                   # tau (held)
            ctypes.c_double,                   # dt [s]
            ctypes.POINTER(ctypes.c_double),   # x_out[NX]
        ]

        st =self._lib.nmpc_sw_init()
        if st != 0:
            raise RuntimeError(f"nmpc_sw_init failed, acados status {st}")

        self.n_horizon = self._lib.nmpc_sw_n_horizon()
        self.n_states = self._lib.nmpc_sw_n_states()
        self.tau_max = self._lib.nmpc_sw_tau_max()
        self.cold_iters_default = self._lib.nmpc_sw_cold_iters_default()

        # preallocated buffers: no per-tick allocation in the loop
        self._x = (ctypes.c_double * self.n_states)()
        self._tau = ctypes.c_double()
        self._status = ctypes.c_int()
        self._t_us = ctypes.c_double()
        self._x_pred = (ctypes.c_double * self.n_states)()

    def predict(self, x, tau, dt):
        """Integrates the plant from x for dt seconds with tau held constant.
        Uses the same model as the solver. Returns a new list."""
        for i in range(self.n_states):
            self._x[i] = x[i]
        st = self._lib.nmpc_sw_predict(self._x, float(tau), float(dt), self._x_pred)
        if st != 0:
            raise ValueError(f"nmpc_sw_predict failed (dt={dt})")
        return list(self._x_pred)

    def solve(self, x, iters=1):
        for i in range(self.n_states):
            self._x[i] = x[i]
        self._lib.nmpc_sw_solve(self._x, int(iters),
                                ctypes.byref(self._tau),
                                ctypes.byref(self._status),
                                ctypes.byref(self._t_us))
        return self._tau.value, self._status.value, self._t_us.value

    def reset(self):
        """Clears the warm start. Call before the first solve of a new swing-up."""
        return self._lib.nmpc_sw_reset()

    def close(self):
        self._lib.nmpc_sw_free()
