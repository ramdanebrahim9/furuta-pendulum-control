# Remote controllers (Raspberry Pi offload)

Some controllers are too heavy for the STM32. This folder is how they run on a
Raspberry Pi 5 instead, while the STM32 keeps full control of timing and the
motor.

## The idea

The balance loop runs every 5 ms. A model predictive controller (MPC) solves an
optimisation problem every tick, and the nonlinear versions take longer than
5 ms on the STM32. A Pi 5 runs those solvers fast enough.

```
        STM32 (owns the clock)                    Raspberry Pi 5 (does the maths)
   ┌─────────────────────────────┐   state    ┌─────────────────────────────┐
   │ sensors -> estimator        │ ─────────> │ wait for a state            │
   │ outer loop, every 5 ms      │            │ solve the optimisation      │
   │ applies the torque          │ <───────── │ send back one torque        │
   └─────────────────────────────┘   torque   └─────────────────────────────┘
                     over UART, 3 wires, 460800 baud
```

The Pi never starts anything. It waits for a state, solves, and replies. If the
Pi is slow, crashes or is unplugged, the STM32 keeps ticking at exactly 5 ms.

## What is in this folder

| File | Purpose |
|---|---|
| `Controller_Remote.hpp/.cpp` | The controller itself. The outer loop calls `compute()` like on any other controller; inside, it talks to the Pi. |
| `RemoteProtocol.hpp` | The message format shared with the Pi side: the modes, the reply marker byte and the frame sizes. |
| `PiHandshake.hpp` | A ping/pong check run before an experiment, to confirm the Pi is up. |

One class, `Controller_Remote`, serves all three Pi solvers. Which one runs
depends on the `SolverMode` it was created with:

| Mode | Solver on the Pi | Used for |
|---|---|---|
| `cMPC` | constrained MPC | holding the pendulum up |
| `NMPC_Stabilize` | nonlinear MPC | holding the pendulum up |
| `NMPC_Swingup` | nonlinear MPC | swinging the pendulum up |

## What happens in one tick

Every 5 ms the outer loop calls `compute(state, measured_torque)`:

1. **Collect.** If the Pi's reply to the previous request has arrived, take its
   torque.
2. **Send.** Send the current state to the Pi. This does not wait.
3. **Return.** Return the torque from step 1.

The reply arrives on its own, through a UART interrupt, some time before the
next tick.

### Why the torque is one tick old

The torque applied now was solved from the state one tick (5 ms) ago. This is
on purpose. The STM32 never stops to wait for the Pi, so the 5 ms timing never
slips. The solvers on the Pi expect this and predict one step ahead to
compensate.

## The messages

Both messages have a fixed size, which keeps reading them simple.

```
Request   STM32 -> Pi   21 bytes   [ mode | state: 4 floats | u_in ]
Reply     Pi -> STM32    5 bytes   [ marker 0xA5 | torque ]
```

- **mode** says which solver to run, or carries a command (below).
- **u_in** is the previous torque, which the solver needs as part of its input.
- **marker** is a fixed first byte on every reply. The reply is sent by a
  Python program on the Pi, whose timing is not exact, so a reply can occasionally
  still be arriving when the STM32 starts listening for the next one. The
  result is a "torque" built from the halves of two different replies. The
  marker lets the STM32 notice a misaligned frame and throw it away.

## Special commands

Besides normal requests, the first byte can be a command:

| Command | When it is sent | What the Pi does |
|---|---|---|
| Reset (one per solver) | The controller is re-entered during a run, for example after the pendulum falls and recovers | Clears that solver's memory |
| `STOP_All` | The STM32 stops using the Pi entirely | Clears every solver |

A solver remembers its previous answer and uses it as a starting guess next
time (a "warm start"). That is useful in normal running, but after a fall the
remembered answer no longer fits the situation. The reset command clears it.

## What if the Pi misses a reply?

`lastRxOk()` returns false when the reply was not there in time. The controller
keeps applying the last torque, and the outer loop decides whether to stop the
experiment. The very first tick after a reset is never counted as a miss,
because no request has been sent yet.

## Before an experiment starts

`PiHandshake::waitForPi()` sends one byte and waits for one byte back. If a
Pi-based controller is selected but the Pi program is not running, this is
reported before the experiment starts, not discovered mid-run as zero torque.

## Case study: the solver that kept reporting errors

This is a real problem from this project, kept as an example of tracking down a
bug that looked like one thing and was another.

**Symptom.** After the pendulum was knocked down during a constrained-MPC run,
the Pi log filled with solver errors (`ACADOS_MINSTEP`, meaning the solver could
not make progress).

**First suspect, a real bug.** After a fall and recovery the Pi's solver was still
warm-starting from its state before the fall. The reset command above was added
to fix this. It was a genuine fix, but the errors continued, even on a solver
that had just been started and could not have been affected by any fall.

**Actual cause.** The optimisation problem limited the arm angle to a fixed
range of roughly -150 to +120 degrees. But the arm angle that is sent is a
running total of encoder counts, and nothing in the system pulls it back
towards zero. In normal use it drifts outside that range. The solver starts
each solve by fixing the first step of its plan to the measured state. A measured
angle outside the allowed range then makes the problem impossible before any
optimising starts, and no amount of tuning can fix a problem that is impossible
as stated.

**Fix.** The arm-angle limit was removed (set to infinity) in the solver design,
and the errors stopped.

**Open item.** The limit was the reason for choosing a constrained MPC in the
first place. The plan is to bring it back as a soft constraint: the arm may
leave the range, but doing so is penalised heavily. The problem then always has
a solution, and the controller still pushes the arm back into range. The
alternative of limiting the angle relative to where the arm was at each
re-entry was rejected, because after a fall the arm can be anywhere, and the
pendulum should balance around the true centre.

## See also

- `RemoteProtocol.hpp`: the exact message format.
- `Pendulum_Loop/ControllerChoices.hpp`: how a remote controller is selected.
- `README_Furuta_STM32_Core.md` and `README_Furuta_Pi5_Core.md` at the repository root: where this fits in the whole system.
