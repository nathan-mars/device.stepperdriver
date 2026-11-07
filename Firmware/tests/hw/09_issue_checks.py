# /// script
# requires-python = ">=3.10"
# dependencies = ["harp>=0.5", "pyserial"]
# ///
"""Step 9: bench checks for upstream issues #31 and #34 and the step interval range.

Homes the motors with HomeMotors (endstops at the negative end, active high), then:

  #31    A move toward the endstop with InputNOpMode set to stop the motor must end with a
         MotorStopped event (upstream: no event when an end-of-travel input stopped it).
         MinPosition is set a little past the sensor as a backstop, so a missed input
         stop ends at that limit instead of the hard stop, and is reported as FAIL.
  #34    Quick movement (motors 1 and 2) at QuickMovementPulseDistance 2.5 and 1.25 with
         the same mm settings must move 1x and 2x the pulses (upstream: 1.25 did not move).
         A quick move faster than the CPU budget must be refused with an error reply.
  range  StepInterval 16000 is accepted and moves exact pulses; 16001 is refused.

    uv run 09_issue_checks.py --motors 0,1
    uv run 09_issue_checks.py --motors 2,3 --search 26000
"""
import argparse
import sys
import threading
import time

from harp.device.client import DeviceError
from stepper import (add_port_arg, add_interlock_arg, set_interlock, open_stepper, configure_motor,
                     enable, disable, position, wait_stopped, read, write, sd)

p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
add_port_arg(p)
add_interlock_arg(p)
p.add_argument("--motors", default="0,1", help="comma-separated motors; motor n uses input n as its endstop")
p.add_argument("--search", type=int, default=26000, help="HomingSearchDistance, pulses (stage travel is ~24000)")
p.add_argument("--current", type=float, default=0.5)
args = p.parse_args()
motors = [int(m) for m in args.motors.split(",")]
mask = sum(1 << m for m in motors)
results: list[tuple[str, bool, str]] = []


def record(name: str, ok: bool, detail: str) -> None:
    results.append((name, ok, detail))
    print(f"  {'OK  ' if ok else 'FAIL'} {name}: {detail}")


def write_array(reg, payload_cls, values: dict[int, int]):
    cur = [int(v) for v in read(dev, reg).payload_array]
    for m, v in values.items():
        cur[m] = v
    write(dev, reg, payload_cls(motor0=cur[0], motor1=cur[1], motor2=cur[2], motor3=cur[3]))


def refused(reg, value) -> bool:
    try:
        write(dev, reg, value)
        return False
    except DeviceError:
        return True


def move_abs(m: int, target: int) -> int:
    write(dev, getattr(sd, f"Motor{m}MoveAbsolute"), target)
    wait_stopped(dev, m)
    return position(dev, m)


dev = open_stepper(args.port)
set_interlock(dev, args.interlock)

stopped_events: list[tuple[float, int]] = []
stop_seen = threading.Event()


def on_stopped(msg) -> None:
    stopped_events.append((time.perf_counter(), int(msg.payload_bytes[0])))
    stop_seen.set()


dev.subscribe(sd.MotorStopped, on_stopped)


def profile(m: int, interval: int = 100) -> None:
    configure_motor(dev, m, step_interval_us=interval, start_interval_us=2000,
                    accel_steps_s2=100_000, current_a=args.current)


# ---- home ------------------------------------------------------------------------------
print("homing")
write_array(sd.HomingConfig, sd.HomingConfigPayload, {m: m for m in motors})
write_array(sd.HomingSearchDistance, sd.HomingSearchDistancePayload, {m: args.search for m in motors})
for m in motors:
    profile(m)
enable(dev, mask)
time.sleep(0.1)
write(dev, sd.HomeMotors, mask)
t0 = time.time()
while time.time() - t0 < 30:
    time.sleep(0.05)
    st = read(dev, sd.HomingState)
    if not int(st.homing) & mask:
        break
if int(read(dev, sd.HomingState).homed) & mask != mask:
    write(dev, sd.StopMotors, mask)
    sys.exit("homing failed; check the endstops before running this script")
for m in motors:
    print(f"  motor {m} homed, position {position(dev, m)}")

# ---- #31: input stop sends MotorStopped --------------------------------------------------
print("#31 MotorStopped after an endstop stop")
for m in motors:
    profile(m)
    move_abs(m, 4000)
    write(dev, getattr(sd, f"Input{m}OpMode"), 0x20 | m)      # stop motor m when the line goes high
    write(dev, sd.EnableDigitalInputs, 1 << m)
    write(dev, getattr(sd, f"Motor{m}MinPosition"), -800)     # backstop 1 mm past the sensor
    time.sleep(0.05)
    stopped_events.clear()
    stop_seen.clear()
    write(dev, getattr(sd, f"Motor{m}MoveAbsolute"), -800)
    got = stop_seen.wait(timeout=5)
    wait_stopped(dev, m)
    pos = position(dev, m)
    bit = any(b & (0x10 << m) for _, b in stopped_events)
    by_input = pos > -700
    record(f"#31 motor {m}", got and bit and by_input,
           f"event {'received' if got else 'MISSING'}"
           f"{' with motor bit' if bit else ' (motor bit not set)' if got else ''}, "
           f"stopped at {pos} ({'by the endstop' if by_input else 'by the MinPosition backstop: input did not stop it'})")
    write(dev, getattr(sd, f"Input{m}OpMode"), 0)
    write(dev, getattr(sd, f"Motor{m}MinPosition"), 0)
    move_abs(m, 2000)

# ---- #34: quick movement pulse distance ----------------------------------------------------
quick = [m for m in motors if m in (1, 2)]
if quick:
    print("#34 quick movement pulse distance")
for m in quick:
    q = f"Motor{m}QuickMovement"
    write(dev, getattr(sd, f"{q}NominalSpeed"), 10.0)       # mm/s
    write(dev, getattr(sd, f"{q}InitialSpeed"), 1.0)
    write(dev, getattr(sd, f"{q}Acceleration"), 0.5)        # m/s^2
    write(dev, getattr(sd, f"{q}Distance"), 2.0)            # mm
    for um in (2.5, 1.25):
        write(dev, getattr(sd, f"{q}PulseDistance"), um)
        expected = round(2.0 * 1000 / um)
        p0 = position(dev, m)
        stopped_events.clear()
        stop_seen.clear()
        try:
            write(dev, sd.TriggerQuickMovement, 1 << m)
            err = ""
        except DeviceError as e:
            err = f"trigger refused ({e})"
        got = stop_seen.wait(timeout=5)
        wait_stopped(dev, m)
        moved = position(dev, m) - p0
        record(f"#34 motor {m} at {um} um/pulse", not err and moved == expected and got,
               err or f"moved {moved} pulses, expected {expected}, MotorStopped {'yes' if got else 'MISSING'}")
        move_abs(m, 2000)
    # Too fast for the CPU budget at 1.25 um: 200 mm/s = 160k pulses/s. Must be refused, not ignored.
    write(dev, getattr(sd, f"{q}NominalSpeed"), 200.0)
    p0 = position(dev, m)
    was_refused = refused(sd.TriggerQuickMovement, 1 << m)
    wait_stopped(dev, m)
    moved = position(dev, m) - p0
    record(f"#34 motor {m} over budget", was_refused and moved == 0,
           f"{'refused with an error reply' if was_refused else 'ACCEPTED'}, moved {moved} pulses")
    write(dev, getattr(sd, f"{q}NominalSpeed"), 10.0)
    move_abs(m, 2000)

# ---- range: 16000 us accepted, 16001 refused ----------------------------------------------
print("step interval range")
for m in motors:
    si, mi = getattr(sd, f"Motor{m}StepInterval"), getattr(sd, f"Motor{m}MaximumStepInterval")
    r16001 = refused(si, 16001) and refused(mi, 16001)
    write(dev, si, 16000)
    write(dev, mi, 16000)
    p0 = position(dev, m)
    t = time.perf_counter()
    end = move_abs(m, p0 + 100)
    dt = time.perf_counter() - t
    record(f"range motor {m}", r16001 and end == p0 + 100 and 1.4 < dt < 2.2,
           f"16001 {'refused' if r16001 else 'ACCEPTED'}; 100 pulses at 16000 us -> {end - p0} pulses "
           f"in {dt:.2f} s (expect ~1.6 s plus settle polling)")
    profile(m)
    move_abs(m, 2000)

disable(dev, mask)
dev.close()
fails = [r for r in results if not r[1]]
print(f"\n{len(results) - len(fails)}/{len(results)} checks passed")
sys.exit(1 if fails else 0)
