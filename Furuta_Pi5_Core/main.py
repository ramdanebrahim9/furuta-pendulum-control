#!/usr/bin/env python3
"""Pi 5 solver application.

The STM32 control loop sends the pendulum state over UART every 5 ms. This
program runs the requested solver and sends back the optimal torque. It never
starts anything on its own: it waits for a frame, solves, and replies.

Messages (sizes are fixed, which keeps reading simple):
    request  (STM32 -> Pi), 21 bytes:  [ mode | state: 4 floats | u_in ]
    reply    (Pi -> STM32),  5 bytes:  [ RX_MAGIC | torque ]

The first byte of a request is a mode:
    0x01 cMPC            constrained MPC (hold-up)
    0x02 NMPC_Stabilize  nonlinear MPC (hold-up)
    0x03 NMPC_Swingup    nonlinear MPC (swing-up)
    0x11 0x12 0x13       reset the cMPC / NMPC_Stabilize / NMPC_Swingup solver
    0x20 StopAll         reset every solver

A solver remembers its previous solution and starts the next solve from it (a
warm start). After a fall and recovery that memory no longer fits, so the
STM32 sends a reset in place of a normal request. Swing-up answers a reset with
a real torque (a cold start); the others answer with zero.

RX_MAGIC is the first byte of every reply. The STM32 uses it to reject a reply
that arrives misaligned, since Python's timing is not exact.

Before an experiment the STM32 sends one PING byte and waits for a PONG, to
confirm this program is running.

Which implementation serves each mode is set in controller_choices.py.
"""

import struct

import serial

import controller_choices as cc

PORT = "/dev/ttyAMA3"
BAUDRATE = 460800

RX_FRAME_SIZE = 21  # mode (1) + state (16) + u_in (4)
RX_MAGIC = 0xA5     # must match the STM32 side
PING_BYTE = 0x50    # 'P'
PONG_BYTE = 0x4B    # 'K'

# Short read timeout so Ctrl+C is handled promptly while no frames arrive.
# A frame is still returned the moment it is complete.
SER_READ_TIMEOUT_S = 0.05

# One instance per mode, built once: loading a solver is a real one-time cost.
_controllers = {
    cc.MODE_CMPC: cc.ACTIVE_CMPC(),
    cc.MODE_NMPC_STABILIZE: cc.ACTIVE_NMPC_STABILIZE(),
    cc.MODE_NMPC_SWINGUP: cc.ACTIVE_NMPC_SWINGUP(),
}

# Reset byte -> the mode whose solver it resets.
_RESET_MODES = {
    0x11: cc.MODE_CMPC,
    0x12: cc.MODE_NMPC_STABILIZE,
    0x13: cc.MODE_NMPC_SWINGUP,
}


def dispatch(mode: int, state: list[float], u_in: float) -> float:
    """Runs the request and returns the torque to send back."""
    if mode == cc.MODE_STOP_ALL:
        print("[Pi5] stop: resetting all solvers")
        for controller in _controllers.values():
            controller.reset()
        return 0.0

    reset_target = _RESET_MODES.get(mode)
    if reset_target is not None:
        controller = _controllers[reset_target]
        cold_start = getattr(controller, "cold_start", None)
        if cold_start is not None:
            # The swing-up solver plans ahead from the reset state, and its
            # answer is the reply.
            print(f"[Pi5] reset 0x{mode:02X}: cold start")
            return cold_start(state)
        print(f"[Pi5] reset 0x{mode:02X}")
        controller.reset()
        return 0.0

    controller = _controllers.get(mode)
    if controller is None:
        print(f"[Pi5] unknown mode 0x{mode:02X}, replying with zero torque")
        return 0.0
    return controller.compute(state, u_in)


def wait_for_ping(ser: serial.Serial) -> None:
    """Blocks until the STM32's startup check arrives, then answers it."""
    print("[Pi5] waiting for the STM32 handshake...")
    while True:
        byte = ser.read(1)
        if len(byte) == 1 and byte[0] == PING_BYTE:
            ser.write(bytes([PONG_BYTE]))
            print("[Pi5] handshake done\n")
            return


def main() -> None:
    print(f"[Pi5] opening {PORT} at {BAUDRATE} baud")
    ser = serial.Serial(PORT, BAUDRATE, timeout=SER_READ_TIMEOUT_S)
    wait_for_ping(ser)
    print("[Pi5] running\n")

    try:
        pending = bytearray()  # part of a frame read so far
        while True:
            # A read can time out part-way through a frame (the STM32 goes
            # quiet during a swing-up cold start), so keep what has arrived and
            # finish the frame on the next pass instead of losing alignment.
            pending += ser.read(RX_FRAME_SIZE - len(pending))
            if len(pending) != RX_FRAME_SIZE:
                continue
            frame = bytes(pending)
            pending.clear()

            mode = frame[0]
            state = struct.unpack_from("<4f", frame, offset=1)
            (u_in,) = struct.unpack_from("<f", frame, offset=17)

            tau = dispatch(mode, state, u_in)
            ser.write(struct.pack("<Bf", RX_MAGIC, tau))
    except KeyboardInterrupt:
        print("\n[Pi5] shutting down")
    finally:
        for controller in _controllers.values():
            controller.close()
        ser.close()


if __name__ == "__main__":
    main()
