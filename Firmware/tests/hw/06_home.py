# /// script
# requires-python = ">=3.10"
# dependencies = ["harp>=0.5", "pyserial"]
# ///
"""Step 6: home one axis against its optical endstop at the negative end of travel.

The endstop is wired to a digital input configured to stop the motor on the sensor's
edge (the firmware stops at the next pulse). Sequence:
  1. if the sensor is already triggered, back off until it clears
  2. fast approach in the negative direction until the sensor stops the motor
  3. back off, slow approach again for repeatability
  4. define that point as position 0 and stay there (optionally set MaxPosition for the
     far end). MinPosition is set to -1 because 0 would disable it, so a negative command
     from home is refused instead of driving into the physical stop (the sensor only acts
     on an edge, and at home it is already triggered).

    uv run 06_home.py --motor 1 --input 1 --search 26000              # 30 mm stage: never look further than ~32 mm
    uv run 06_home.py --motor 1 --input 1 --search 26000 --active low  # sensor pulls the line low when blocked
    uv run 06_home.py --motor 1 --input 1 --search 26000 --max-travel-pulses 22400   # also set the far soft limit

Input polarity, as implemented by the firmware: the inputs are pulled up and the
DigitalInputState bit is 1 when the line is LOW. The register values StopMotorOnRising /
StopMotorOnFalling refer to that bit, so "Falling" (0x2m) stops when the line goes HIGH
and "Rising" (0x1m) when it goes LOW. This script hides that behind --active.
"""
import argparse
import sys
import time
from stepper import (add_port_arg, add_interlock_arg, set_interlock, open_stepper, configure_motor,
                     enable, disable, move_relative, position, wait_stopped, read, write, sd)

p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
add_port_arg(p)
add_interlock_arg(p)
p.add_argument("--motor", type=int, default=1)
p.add_argument("--input", type=int, default=None, help="digital input wired to this axis' endstop (default: same number as the motor)")
p.add_argument("--active", choices=["high", "low"], default="high",
               help="level of the input line when the flag is in the sensor (default high, as in the lab's existing configs)")
p.add_argument("--fast-us", type=float, default=100, help="approach: us between pulses (100 = 12.5 mm/s at 1.25 um)")
p.add_argument("--slow-us", type=float, default=500, help="second approach: us between pulses")
p.add_argument("--accel", type=int, default=100000, help="steps/s^2")
p.add_argument("--backoff", type=int, default=800, help="pulses to back off after each hit (800 = 1 mm at 1.25 um)")
p.add_argument("--search", type=int, required=True,
               help="maximum pulses to travel while looking for the sensor; set it a little above the stage travel (24000 = 30 mm at 1.25 um)")
p.add_argument("--max-travel-pulses", type=int, default=0, help="if > 0, set MaxPosition to this after homing")
p.add_argument("--park", type=int, default=0, help="pulses to move off the sensor after homing (default: stay at 0)")
p.add_argument("--no-soft-min", action="store_true", help="do not set MinPosition to -1 after homing")
p.add_argument("--current", type=float, default=0.5)
args = p.parse_args()

m = args.motor
k = args.input if args.input is not None else m
# Stop when the line reaches the active level: line going HIGH = state bit falling = 0x2m,
# line going LOW = state bit rising = 0x1m (see the note in the module docstring).
edge_code = (0x20 if args.active == "high" else 0x10) | m

dev = open_stepper(args.port)
set_interlock(dev, args.interlock)


def triggered() -> bool:
    """True when the input line is at the sensor's active level. DigitalInputState bit k is
    set when the line is LOW (pulled-up input)."""
    low = bool(int(read(dev, sd.DigitalInputState)) & (1 << k))
    return (not low) if args.active == "high" else low


def approach(interval_us: float) -> int:
    """Move negative until the sensor stops the motor. Returns pulses travelled."""
    configure_motor(dev, m, step_interval_us=int(round(interval_us)), start_interval_us=2000,
                    accel_steps_s2=args.accel, current_a=args.current)
    p0 = position(dev, m)
    move_relative(dev, m, -args.search)
    if not wait_stopped(dev, m, timeout_s=args.search * interval_us * 1e-6 + 10):
        sys.exit("timeout while approaching the sensor")
    travelled = p0 - position(dev, m)
    if not triggered():
        write(dev, sd.StopMotors, 1 << m)
        sys.exit(f"travelled {travelled} pulses without reaching the sensor on input {k}; check wiring, --input and --active")
    return travelled


# Input k stops motor m on the chosen edge
write(dev, getattr(sd, f"Input{k}OpMode"), edge_code)
write(dev, sd.EnableDigitalInputs, 1 << k)
enable(dev, 1 << m)
time.sleep(0.1)

configure_motor(dev, m, step_interval_us=int(round(args.fast_us)), start_interval_us=2000,
                accel_steps_s2=args.accel, current_a=args.current)
if triggered():
    print("sensor already triggered: backing off")
    for _ in range(5):
        move_relative(dev, m, args.backoff)
        wait_stopped(dev, m)
        if not triggered():
            break
    else:
        sys.exit("sensor stays triggered after backing off; check --active and the wiring")

t = approach(args.fast_us)
print(f"fast approach: sensor found after {t} pulses")
hit1 = position(dev, m)
move_relative(dev, m, args.backoff)
wait_stopped(dev, m)
approach(args.slow_us)
hit2 = position(dev, m)
print(f"slow approach: repeat offset {hit2 - hit1:+d} pulses between the two hits")

# Define home at the slow-approach hit point.
# (device.yml before 1.0 misspelled these registers "Mortor<n>AccumulatedSteps")
write(dev, getattr(sd, f"Motor{m}AccumulatedSteps", None) or getattr(sd, f"Mortor{m}AccumulatedSteps"), 0)
if not args.no_soft_min:
    write(dev, getattr(sd, f"Motor{m}MinPosition"), -1)     # 0 would mean "disabled"
if args.max_travel_pulses > 0:
    write(dev, getattr(sd, f"Motor{m}MaxPosition"), args.max_travel_pulses)
    print(f"MaxPosition set to {args.max_travel_pulses}")
if args.park:
    move_relative(dev, m, args.park)
    wait_stopped(dev, m)
print(f"homed: sensor edge is position 0, now at {position(dev, m)}")
disable(dev, 1 << m)
dev.close()
