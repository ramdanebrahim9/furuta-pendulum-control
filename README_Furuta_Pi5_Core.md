# Furuta Pendulum Control System: Raspberry Pi 5 solvers

The solver application that runs on the Raspberry Pi 5. It is the second half of
the system; the firmware is described in
[`README_Furuta_STM32_Core.md`](README_Furuta_STM32_Core.md).

The code is in [`Furuta_Pi5_Core/`](Furuta_Pi5_Core/). The STM32 control loop
sends the pendulum state over UART every 5 ms; this program runs the requested
optimisation and sends back the optimal torque.

```
STM32  --- state, previous torque --->  Pi: pick the solver, solve
STM32  <--------- torque -------------  Pi
```

The Pi never starts anything. It waits for a request, solves, and replies. The
STM32 does not wait for the answer: it applies the reply to the previous request
on the next tick, so a slow or missing reply never disturbs the 5 ms timing.
The message format and the reasoning are described in
[`Remote_Controllers/README.md`](Furuta_STM32_Core/Control_Outer_Loop/Controllers/Remote_Controllers/README.md).

## Solvers

| Mode | Solver | Used for |
|---|---|---|
| `cMPC` | constrained MPC | holding the pendulum up |
| `NMPC_Stabilize` | nonlinear MPC | holding the pendulum up |
| `NMPC_Swingup` | nonlinear MPC | swinging the pendulum up |

Each one is a solver generated from a MATLAB design, wrapped in a small C shim
and called from Python.

## Layout

```
main.py                 reads requests, dispatches, writes replies
controller_choices.py   which implementation serves each mode
controllers/            one class per solver: compute(state, u_prev) -> torque
solvers/
  cmpc/  nmpc/  nmpc_sw/
    *_shim.c              plain-C interface to the generated solver
    *_solver.py           ctypes wrapper around the compiled library
    generated/            generated solver code
```

`compute()` is the single call the dispatcher uses, the same shape as on the
STM32 side, so adding or swapping a solver changes only `controller_choices.py`.

## Details worth knowing

- **Warm start.** A solver starts each solve from its previous solution. After a
  fall and recovery that no longer fits, so the STM32 sends a reset and the
  solver clears it.
- **Swing-up cold start.** After a reset the swing-up solver cannot start from a
  flat guess, because the hanging position is a symmetric point of the cost. It
  starts from a rollout of the energy-based swing-up law, planned for the state
  the system will be in when the STM32's fixed zero-torque hold ends.
- **Faults.** If a solve fails, the previous torque is held instead of applying
  a bad solution.

## About this copy

This is a read-only showcase. The generated model, cost and constraint code
returns dummy values, and the plant parameters, cost weights, bounds and
tuning gains are withheld (shown as `0.0` placeholders). Build and deploy
scripts, experiment logs and compiled libraries are not included.
