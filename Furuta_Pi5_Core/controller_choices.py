"""Which implementation serves each solver mode.

The Pi-side counterpart of ControllerChoices.hpp on the STM32: one file says
what runs for each mode byte, and main.py only reads it. To swap an
implementation, change the ACTIVE_* line.

Mode bytes match Controller_Remote::SolverMode on the STM32 side:
    0x01  cMPC            constrained MPC (hold-up)
    0x02  NMPC_Stabilize  nonlinear MPC (hold-up)
    0x03  NMPC_Swingup    nonlinear MPC (swing-up)
    0x20  StopAll         not a controller: resets every solver
"""
from controllers.controller_cmpc import Controller_cMPC
from controllers.controller_nmpc import Controller_NMPC
from controllers.controller_nmpc_sw import Controller_NMPC_SW

MODE_CMPC = 0x01
MODE_NMPC_STABILIZE = 0x02
MODE_NMPC_SWINGUP = 0x03
MODE_STOP_ALL = 0x20

ACTIVE_CMPC = Controller_cMPC
ACTIVE_NMPC_STABILIZE = Controller_NMPC
ACTIVE_NMPC_SWINGUP = Controller_NMPC_SW
