# /// script
# requires-python = ">=3.10"
# dependencies = ["harp>=0.5", "pyserial"]
# ///
"""Step 8: home one or more axes with the firmware's HomeMotors register.

Configures the Homing* array registers for the selected motors, writes HomeMotors and
waits for HomingState to report the result. Then, for each homed axis, checks the zero:
move off by --backoff, approach again slowly with the input set to stop the motor
(InputNOpMode), and report where the sensor edge is now (it should be within a few pulses
of 0). Repeats --cycles times, moving --travel pulses out between homings.

    uv run 08_home_firmware.py --motors 0 --search 26000                 # motor 0 on input 0
    uv run 08_home_firmware.py --motors 0,1 --search 26000 --cycles 5    # two axes together
    uv run 08_home_firmware.py --motors 0,1 --search 26000 --late 1      # start motor 1 while 0 is homing
    uv run 08_home_firmware.py --motors 1 --inputs 3 --active low --search 26000

Input polarity, as implemented by the firmware: the inputs are pulled up; --active high
means the line goes HIGH when the flag is in the sensor (the lab's optical endstops).
"""
import argparse
import sys
import time

from harp.device.client import DeviceError
from stepper import (add_port_arg, add_interlock_arg, set_interlock, open_stepper, configure_motor,
                     enable, disable, move_relative, position, wait_stopped, read, write, sd)

p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
add_port_arg(p)
add_interlock_arg(p)
p.add_argument("--motors", default="0", help="comma-separated motors to home (default 0)")
p.add_argument("--inputs", default=None, help="comma-separated endstop inputs, one per motor (default: same numbers)")
p.add_argument("--active", choices=["high", "low"], default="high", help="input line level when the sensor is triggered")
p.add_argument("--positive", action="store_true", help="the endstop is at the positive end of travel")
p.add_argument("--search", type=int, required=True, help="HomingSearchDistance in pulses (a little above the stage travel)")
p.add_argument("--backoff", type=int, default=800, help="HomingBackoff in pulses (default 800 = 1 mm at 1.25 um)")
p.add_argument("--fast-us", type=int, default=100, help="HomingStepInterval (default 100 us = 12.5 mm/s at 1.25 um)")
p.add_argument("--slow-us", type=int, default=500, help="HomingSlowStepInterval (default 500 us)")
p.add_argument("--accel", type=int, default=100000, help="MotorNAcceleration, steps/s^2")
p.add_argument("--start-us", type=int, default=2000, help="MotorNMaximumStepInterval (start speed)")
p.add_argument("--current", type=float, default=0.5)
p.add_argument("--cycles", type=int, default=3, help="number of homings per axis")
p.add_argument("--travel", type=int, default=8000, help="pulses to move out between homings (default 8000 = 10 mm)")
p.add_argument("--late", type=int, default=None, help="start this motor (one of --motors) 200 ms after the others")
p.add_argument("--timeout", type=float, default=30.0)
args = p.parse_args()

motors = [int(m) for m in args.motors.split(",")]
inputs = [int(k) for k in args.inputs.split(",")] if args.inputs else list(motors)
if len(inputs) != len(motors):
    sys.exit("--inputs needs one input per motor")
if args.late is not None and args.late not in motors:
    sys.exit("--late must be one of --motors")
mask = sum(1 << m for m in motors)
away = 1 if not args.positive else -1          # sign of a move away from the sensor


def write_array(reg, payload_cls, values: dict[int, int]):
    """Read-modify-write one of the Homing* arrays (all four motors are written together)."""
    cur = [int(v) for v in read(dev, reg).payload_array]
    for m, v in values.items():
        cur[m] = v
    write(dev, reg, payload_cls(motor0=cur[0], motor1=cur[1], motor2=cur[2], motor3=cur[3]))


def homing_state() -> tuple[int, int]:
    st = read(dev, sd.HomingState)
    return int(st.homed), int(st.homing)


dev = open_stepper(args.port)
set_interlock(dev, args.interlock)

cfg = {m: k | (0x10 if args.positive else 0) | (0x20 if args.active == "low" else 0) for m, k in zip(motors, inputs)}
write_array(sd.HomingConfig, sd.HomingConfigPayload, cfg)
write_array(sd.HomingStepInterval, sd.HomingStepIntervalPayload, {m: args.fast_us for m in motors})
write_array(sd.HomingSlowStepInterval, sd.HomingSlowStepIntervalPayload, {m: args.slow_us for m in motors})
write_array(sd.HomingSearchDistance, sd.HomingSearchDistancePayload, {m: args.search for m in motors})
write_array(sd.HomingBackoff, sd.HomingBackoffPayload, {m: args.backoff for m in motors})
for m in motors:
    configure_motor(dev, m, step_interval_us=args.fast_us, start_interval_us=args.start_us,
                    accel_steps_s2=args.accel, current_a=args.current)

events: list[tuple[float, int, int]] = []
t_start = time.perf_counter()
dev.subscribe(sd.HomingState, lambda msg: events.append(
    (time.perf_counter() - t_start, int(msg.payload.homed), int(msg.payload.homing))))

enable(dev, mask)
time.sleep(0.1)


def home(which: list[int]) -> dict[int, float]:
    """Home `which` (optionally staggered) and return the time each one took, in seconds."""
    global t_start
    events.clear()
    t_start = time.perf_counter()
    first = [m for m in which if m != args.late]
    write(dev, sd.HomeMotors, sum(1 << m for m in first))
    if args.late in which:
        time.sleep(0.2)
        write(dev, sd.HomeMotors, 1 << args.late)
        homed, homing = homing_state()
        print(f"  motor {args.late} requested late: homing mask now {homing:#x}")
    done: dict[int, float] = {}
    while time.perf_counter() - t_start < args.timeout:
        time.sleep(0.02)
        homed, homing = homing_state()
        for m in which:
            if m not in done and not homing & (1 << m):
                done[m] = time.perf_counter() - t_start
        if len(done) == len(which):
            break
    else:
        write(dev, sd.StopMotors, sum(1 << m for m in which))
        sys.exit(f"timeout: HomingState homed {homed:#x}, homing {homing:#x}")
    homed, _ = homing_state()
    for m in which:
        if not homed & (1 << m):
            sys.exit(f"motor {m}: homing FAILED after {done[m]:.2f} s (check --inputs, --active, --search, --backoff)")
    return done


def edge_check(m: int, k: int) -> int:
    """Back off, then approach slowly with InputkOpMode stopping the motor on the sensor.
    Returns the position where it stopped (should be close to 0)."""
    op = (0x20 if args.active == "high" else 0x10) | m      # Falling = line going high
    configure_motor(dev, m, step_interval_us=args.slow_us, start_interval_us=max(args.start_us, args.slow_us),
                    accel_steps_s2=args.accel, current_a=args.current)
    move_relative(dev, m, away * args.backoff)
    wait_stopped(dev, m)
    write(dev, getattr(sd, f"Input{k}OpMode"), op)
    write(dev, sd.EnableDigitalInputs, 1 << k)
    move_relative(dev, m, -away * 2 * args.backoff)
    wait_stopped(dev, m)
    write(dev, getattr(sd, f"Input{k}OpMode"), 0)            # event only again
    return position(dev, m)


try:
    print(f"homing motors {motors} on inputs {inputs}: {args.fast_us} us search, {args.slow_us} us final, "
          f"backoff {args.backoff}, search {args.search}")
    for c in range(args.cycles):
        times = home(motors)
        line = []
        for m, k in zip(motors, inputs):
            pos0 = position(dev, m)
            edge = edge_check(m, k)
            line.append(f"m{m}: {times[m]:.2f} s, pos {pos0:+d}, edge {edge:+d}")
        print(f"cycle {c + 1}: " + "; ".join(line))
        print("  HomingState events: " + ", ".join(f"{t*1000:.0f} ms homed={h:#x} homing={g:#x}" for t, h, g in events))
        # move out for the next cycle, all axes together
        for m in motors:
            configure_motor(dev, m, step_interval_us=args.fast_us, start_interval_us=args.start_us,
                            accel_steps_s2=args.accel, current_a=args.current)
            move_relative(dev, m, away * args.travel)
        for m in motors:
            wait_stopped(dev, m)

    # A move written to a homing axis must be refused; StopMotors cancels homing
    m = motors[0]
    write(dev, sd.HomeMotors, 1 << m)
    time.sleep(0.05)
    try:
        move_relative(dev, m, 100)
        print(f"WARNING: a move to motor {m} was accepted while it was homing")
    except DeviceError:
        print(f"move to motor {m} refused while homing: OK")
    write(dev, sd.StopMotors, 1 << m)
    time.sleep(0.05)
    homed, homing = homing_state()
    print(f"after StopMotors: homed {homed:#x}, homing {homing:#x} (motor {m} should be neither)")
finally:
    disable(dev, mask)
    dev.close()
