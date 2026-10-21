ROBOT ARM V10 - FINAL BATTERY LIFT GUARD

BASED ON: V9 full project. Five Arduino source files are together in robot_arm/.
The battery positions, homing, gripper angles, existing XY/Z routes, and
continuous return sequence are unchanged.

CAUSE FROM SERIAL LOG
RUN: LAST BATTERY PLACE / FIRST RETURN
STATION: LIFT
LOADED XY SPEED CAP: 1.5x
XY HARD STOP: tracking error X -0.139 DEG, Y -1.354 DEG

ONLY NEW MOTION BEHAVIOR
The loaded upward XY lift immediately before the last battery place uses an
additional 0.75x maximum speed/acceleration scale (default). The selected
speed value is restored before XYZ transfer, descent and subsequent cycles.
All other loaded lifts keep the existing "loadspeed" cap. The arm still stops
if encoder following error exceeds 1 degree. No autonomous resume is made
from an interrupted position. Previously taught battery coordinates are intact.

SERIAL COMMANDS (115200 baud, Newline)
lastlift       show speed limit for the final loaded lift
lastlift 0.5   slower diagnosis if final lift still stops
lastlift 0.75  recommended first trial (default)
lastlift 1     slightly faster trial ONLY after stable tests
speed 1 / loadspeed 1 / diagboost 1 / cycles 1 / s / run

IMPORTANT
Power down safely, remove the held cell, restore the KNOWN original
mechanical start pose, then reboot/re-register before a new run.
Test without cells first. If the final lift still stops at 0.5x,
stop increasing speed; inspect Y motor torque and supply under load,
driver current limits, encoder coupling/wiring and interference between
the battery, tray and gripper. Never bypass the safety threshold.

Host C++ syntax checks and simulated tests do not prove physical performance
or confirm Arduino IDE compilation.
