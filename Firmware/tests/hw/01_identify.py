# /// script
# requires-python = ">=3.10"
# dependencies = ["harp>=0.5", "pyserial"]
# ///
"""Step 1: can we talk to the board? Prints WhoAmI, firmware, and the motion registers."""
import argparse
from stepper import add_port_arg, open_stepper, positions, read, sd

p = argparse.ArgumentParser(description=__doc__)
add_port_arg(p)
args = p.parse_args()

from harp.device.client import DeviceError

def read_or_na(reg):
    try:
        return read(dev, reg)
    except DeviceError:
        return "n/a (register not in this firmware)"

dev = open_stepper(args.port)
for m in range(4):
    print(f"motor {m}: step interval {read_or_na(getattr(sd, f'Motor{m}StepInterval'))} us, "
          f"start interval {read_or_na(getattr(sd, f'Motor{m}MaximumStepInterval'))} us, "
          f"acceleration {read_or_na(getattr(sd, f'Motor{m}Acceleration'))} steps/s^2")
print("accumulated steps:", positions(dev))
print(f"interlock mode: {int(read(dev, sd.InterlockEnabled))} (0 closed, 1 open), driver errors: {int(read(dev, sd.MotorRaisedError)):#x}")
dev.close()
