"""Shared helpers for the StepperDriver hardware tests.

Uses the `harp` Python package (https://github.com/harp-tech/python). The runnable
scripts declare it as an inline dependency, so `uv run <script>.py` sets everything up.
The register classes are built at run time from ../../../device.yml, so new
registers are available without regenerating anything.
"""
from __future__ import annotations

import argparse
import glob
import sys
import time
from pathlib import Path

from harp import serial
from harp.device import core, schema

DEVICE_YML = Path(__file__).resolve().parents[3] / "device.yml"
WHO_AM_I_STEPPER = 1130

sd = schema.create_device_module(DEVICE_YML.read_bytes())  # register classes by name


def default_port() -> str | None:
    ports = sorted(glob.glob("/dev/tty.usbserial-*") + glob.glob("/dev/ttyUSB*"))
    return ports[0] if ports else None


def add_port_arg(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--port", default=default_port(), help="serial port (default: first usbserial)")


def open_stepper(port: str):
    """Open the device, check WhoAmI and print the firmware version."""
    if not port:
        sys.exit("no serial port found; pass --port")
    dev = serial.open_device(sd, port=port)   # also checks WhoAmI against device.yml
    who = int(dev.read(core.WhoAmI).payload)
    if who != WHO_AM_I_STEPPER:
        dev.close()
        sys.exit(f"device on {port} is WhoAmI {who}, not a StepperDriver ({WHO_AM_I_STEPPER})")
    try:
        fw_h = int(dev.read(core.FirmwareVersionHigh).payload)
        fw_l = int(dev.read(core.FirmwareVersionLow).payload)
        print(f"StepperDriver on {port}, firmware {fw_h}.{fw_l}")
    except Exception:  # noqa: BLE001 - version is informational
        print(f"StepperDriver on {port}")
    set_active(dev)
    return dev


def set_active(dev) -> None:
    """Harp devices emit events only in Active mode; connecting does not switch them."""
    cur = dev.read(core.OperationControl).payload
    if cur.operation_mode != core.OperationMode.ACTIVE:
        dev.write(core.OperationControl, core.OperationControlPayload(
            operation_mode=core.OperationMode.ACTIVE,
            dump_registers=False,
            mute_replies=False,
            visual_indicators=cur.visual_indicators,
            operation_led=cur.operation_led,
            heartbeat=cur.heartbeat,
        ))
        time.sleep(0.05)


def write(dev, reg, value):
    """Write a register and return the reply. Raises harp DeviceError on an error reply."""
    return dev.write(reg, value)


def read(dev, reg):
    return dev.read(reg).payload


def positions(dev) -> list[int]:
    """AccumulatedSteps for all motors (the payload is a struct with motor0..motor3)."""
    return [int(v) for v in read(dev, sd.AccumulatedSteps).payload_array]


def position(dev, motor: int) -> int:
    return positions(dev)[motor]


def wait_stopped(dev, motor: int, timeout_s: float = 30.0, poll_s: float = 0.02) -> bool:
    """Poll AccumulatedSteps until it stops changing for 3 polls (or timeout)."""
    t0 = time.time()
    last = position(dev, motor)
    stable = 0
    while time.time() - t0 < timeout_s:
        time.sleep(poll_s)
        p = position(dev, motor)
        stable = stable + 1 if p == last else 0
        last = p
        if stable >= 3:
            return True
    return False


def configure_motor(dev, motor: int, *, step_interval_us: int, start_interval_us: int,
                    accel_steps_s2: int, current_a: float = 0.5, microstep: int = 0) -> None:
    """Set the trapezoid profile of one motor (step-unit registers)."""
    m = f"Motor{motor}"
    write(dev, getattr(sd, f"{m}StepInterval"), step_interval_us)
    write(dev, getattr(sd, f"{m}MaximumStepInterval"), start_interval_us)
    write(dev, getattr(sd, f"{m}Acceleration"), accel_steps_s2)
    write(dev, getattr(sd, f"{m}MaximumRunCurrent"), current_a)
    write(dev, getattr(sd, f"{m}MicrostepResolution"), microstep)


def add_interlock_arg(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--interlock", choices=["open", "closed"], default="open",
                        help="interlock connector state that enables the device (default: open = nothing plugged in)")


def set_interlock(dev, mode: str) -> None:
    """InterlockEnabled (reg 72): 0 = enabled when the connector is closed, 1 = when open.
    Drivers cannot be enabled unless the connector matches this setting."""
    write(dev, sd.InterlockEnabled, 1 if mode == "open" else 0)
    time.sleep(0.3)   # the firmware debounces the interlock state for 250 ms


def enable(dev, mask: int) -> None:
    from harp.device.client import DeviceError
    try:
        write(dev, sd.EnableDriver, mask)
    except DeviceError:
        err = int(read(dev, sd.MotorRaisedError))
        sys.exit(f"EnableDriver refused for mask {mask:#x}. Either the interlock connector does not match "
                 f"--interlock, or the driver reports an error (MotorRaisedError = {err:#x}).")


def disable(dev, mask: int) -> None:
    write(dev, sd.DisableDriver, mask)


def move_relative(dev, motor: int, steps: int) -> None:
    write(dev, getattr(sd, f"Motor{motor}MoveRelative"), steps)
