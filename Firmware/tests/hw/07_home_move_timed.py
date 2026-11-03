# /// script
# requires-python = ">=3.10"
# dependencies = ["harp>=0.5", "pyserial"]
# ///
"""Step 7: home, timed out-and-back move, then use the endstop to check for lost steps.

  1. home against the optical endstop (fast approach, back off, slow approach); the
     slow-approach hit is position 0
  2. MoveAbsolute to +distance, then back to 0, at the given peak speed; each move is
     bracketed by Harp device timestamps. The endstop stays armed the whole time, so if
     steps were lost on the way out the sensor stops the return move before 0 (instead of
     the motor hitting the hard stop); the test reports that and stops.
  3. back off, slow approach again: the hit position is how many pulses the axis drifted
     (catches lost steps the early-trigger check cannot see, e.g. on the way back; compare
     with the homing repeat offset, which is the sensor's own repeatability)

    uv run 07_home_move_timed.py --motor 0                       # 15 mm at 100 mm/s, input 0
    uv run 07_home_move_timed.py --motor 0 --repeat 5 --mm 15 --mm-s 100

Timestamps, all from the device clock (Harp timestamps, seconds):
  before    reply to a read sent just before the move command
  command   reply to the MoveAbsolute write = when the firmware started the move
  stopped   MotorStopped event = when the last pulse was issued
  after     reply to a read sent just after the host saw MotorStopped
"stopped - command" is the move duration; "after - before" adds the USB round trips.
The sensor edge is position 0, so the return move may set it off before the count reaches
0: while decelerating, the rotor and carriage lead the step count (15-19 pulses at 2.5 m/s^2
on the test rig, with no steps lost). Only a stop more than --early-tolerance pulses before 0
counts as an early trigger; the final slow approach is the precise lost-step check.
With --return-to N > 0 the moves run between N and N + distance, so the return finishes on
its own deceleration short of the sensor; any endstop trip then means lost steps.
"""
import argparse
import sys
import threading
import time
from harp.device import core
from stepper import (add_port_arg, add_interlock_arg, set_interlock, open_stepper, configure_motor,
                     enable, disable, move_relative, position, wait_stopped, read, write, sd)

p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
add_port_arg(p)
add_interlock_arg(p)
p.add_argument("--motor", type=int, default=0)
p.add_argument("--input", type=int, default=None, help="endstop input (default: same number as the motor)")
p.add_argument("--active", choices=["high", "low"], default="high", help="endstop line level when triggered")
p.add_argument("--um-per-pulse", type=float, default=1.25)
p.add_argument("--mm", type=float, default=15.0, help="move distance from home")
p.add_argument("--mm-s", type=float, default=100.0, help="peak (cruise) speed")
p.add_argument("--accel", type=int, default=2_000_000, help="steps/s^2 (2e6 = 2.5 m/s^2 at 1.25 um)")
p.add_argument("--start-us", type=int, default=2000, help="start/stop: us between pulses")
p.add_argument("--repeat", type=int, default=1, help="out-and-back cycles before the endstop check")
p.add_argument("--search", type=int, default=26000, help="max pulses while homing (stage is ~24000)")
p.add_argument("--home-fast-us", type=int, default=100)
p.add_argument("--home-slow-us", type=int, default=500)
p.add_argument("--backoff", type=int, default=800, help="pulses to back off before each slow approach")
p.add_argument("--early-tolerance", type=int, default=40,
               help="pulses short of 0 that still count as reaching 0 on the return move")
p.add_argument("--return-to", type=int, default=0,
               help="pulses from home where the return moves end (0 = at the sensor edge)")
p.add_argument("--current", type=float, default=0.5)
args = p.parse_args()

m = args.motor
k = args.input if args.input is not None else m
edge_code = (0x20 if args.active == "high" else 0x10) | m   # see 06_home.py for the polarity note
pulses = round(args.mm * 1000 / args.um_per_pulse)
interval_us = max(10, round(1e6 * args.um_per_pulse / (args.mm_s * 1000)))   # register is whole us
peak_mm_s = args.um_per_pulse * 1e3 / interval_us

dev = open_stepper(args.port)
set_interlock(dev, args.interlock)
min_pos_reg = getattr(sd, f"Motor{m}MinPosition")
max_pos_reg = getattr(sd, f"Motor{m}MaxPosition")
write(dev, min_pos_reg, 0)   # soft limits off while homing and checking (0 = disabled)
write(dev, max_pos_reg, 0)


def triggered() -> bool:
    low = bool(int(read(dev, sd.DigitalInputState)) & (1 << k))   # bit set = line LOW
    return (not low) if args.active == "high" else low


def approach(interval: int, search: int) -> int:
    """Move negative until the endstop stops the motor; return the hit position."""
    configure_motor(dev, m, step_interval_us=interval, start_interval_us=args.start_us,
                    accel_steps_s2=100_000, current_a=args.current)
    move_relative(dev, m, -search)
    if not wait_stopped(dev, m, timeout_s=search * interval * 1e-6 + 10):
        sys.exit("timeout while approaching the endstop")
    if not triggered():
        sys.exit(f"travelled {search} pulses without reaching the endstop on input {k}")
    return position(dev, m)


def back_off() -> None:
    move_relative(dev, m, args.backoff)
    wait_stopped(dev, m)
    if triggered():
        sys.exit("endstop still triggered after backing off; check --active / --backoff")


# ---- 1. home ---------------------------------------------------------------------------
write(dev, getattr(sd, f"Input{k}OpMode"), edge_code)
write(dev, sd.EnableDigitalInputs, 1 << k)   # armed for the whole test
enable(dev, 1 << m)
time.sleep(0.1)
configure_motor(dev, m, step_interval_us=args.home_fast_us, start_interval_us=args.start_us,
                accel_steps_s2=100_000, current_a=args.current)
if triggered():
    back_off()
approach(args.home_fast_us, args.search)
hit1 = position(dev, m)
back_off()
hit2 = approach(args.home_slow_us, args.backoff + 400)
repeat_offset = hit2 - hit1
write(dev, getattr(sd, f"Motor{m}AccumulatedSteps"), 0)
print(f"homed on input {k} (fast/slow hit offset {repeat_offset:+d} pulses)")

# ---- 2. timed moves --------------------------------------------------------------------
configure_motor(dev, m, step_interval_us=interval_us, start_interval_us=args.start_us,
                accel_steps_s2=args.accel, current_a=args.current)
stopped = threading.Event()
stop_ts: list[float] = []


def on_stopped(msg) -> None:
    if msg.payload_bytes[0] & (0x10 << m):   # high nibble: motors that just stopped
        stop_ts.append(msg.timestamp)
        stopped.set()


sub = dev.subscribe(sd.MotorStopped, on_stopped)
move_reg = getattr(sd, f"Motor{m}MoveAbsolute")
print(f"move: {args.mm} mm = {pulses} pulses, peak {peak_mm_s:.1f} mm/s ({interval_us} us/pulse), "
      f"accel {args.accel * args.um_per_pulse * 1e-6:.2f} m/s^2")
print(f"{'target':>8} {'pos':>7} {'before':>14} {'command':>14} {'stopped':>14} {'after':>14} "
      f"{'move ms':>8} {'bracket ms':>10}")
R = args.return_to
if R:
    write(dev, move_reg, R)   # untimed: park at the return point first
    wait_stopped(dev, m)
moves = {"out": [], "back": []}   # (duration s, pulses travelled)
ok = True
early = None
return_stops: list[int] = []
for cycle in range(args.repeat):
    for name, target in (("out", R + pulses), ("back", R)):
        stopped.clear()
        stop_ts.clear()
        p0 = position(dev, m)
        t_before = dev.read(core.TimestampSeconds).timestamp
        t_cmd = write(dev, move_reg, target).timestamp
        if not stopped.wait(timeout=10):
            print("no MotorStopped event within 10 s"); ok = False
            wait_stopped(dev, m)
        t_after = dev.read(core.TimestampSeconds).timestamp
        pos = position(dev, m)
        t_stop = stop_ts[0] if stop_ts else float("nan")
        moves[name].append((t_stop - t_cmd, abs(pos - p0)))
        print(f"{target:>8} {pos:>7} {t_before:>14.6f} {t_cmd:>14.6f} {t_stop:>14.6f} {t_after:>14.6f} "
              f"{(t_stop - t_cmd) * 1e3:>8.2f} {(t_after - t_before) * 1e3:>10.2f}")
        if name == "back":
            return_stops.append(pos)
        if name == "back" and triggered() and (R > 0 or pos > args.early_tolerance):
            early = (cycle + 1, pos)
            break
        if pos != target and not (name == "back" and R == 0 and 0 <= pos <= args.early_tolerance):
            ok = False
    if early:
        break
sub.unsubscribe()


def expected_s(n: int) -> float:
    v0, vmax, a = 1e6 / args.start_us, 1e6 / interval_us, args.accel
    d_ramp = (vmax * vmax - v0 * v0) / (2 * a)
    if 2 * d_ramp < n:
        return 2 * (vmax - v0) / a + (n - 2 * d_ramp) / vmax
    return 2 * ((v0 * v0 + a * n) ** 0.5 - v0) / a


print(f"\nsummary ({args.mm} mm, peak {peak_mm_s:.1f} mm/s, expected {expected_s(pulses) * 1e3:.2f} ms per move)")
print(f"{'move':>5} {'n':>3} {'time ms (mean)':>15} {'min':>8} {'max':>8} {'avg speed mm/s':>15} {'expected ms':>12}")
for name, rs in moves.items():
    # complete moves with a timestamp (the return may stop up to --early-tolerance short at the edge)
    full = [r for r in rs if r[1] >= pulses - args.early_tolerance and r[0] == r[0]]
    if not full:
        continue
    ts = [r[0] * 1e3 for r in full]
    mean = sum(ts) / len(ts)
    speed = sum(r[1] for r in full) * args.um_per_pulse * 1e-3 / sum(r[0] for r in full)
    print(f"{name:>5} {len(ts):>3} {mean:>15.2f} {min(ts):>8.2f} {max(ts):>8.2f} "
          f"{speed:>15.1f} {expected_s(pulses) * 1e3:>12.2f}")
if return_stops:
    print(f"return moves stopped at (pulses short of 0, endstop trip while decelerating): {return_stops}" if R == 0
          else f"return moves stopped at (target {R}): {return_stops}")
if early:
    c, pos = early
    print(f"\nEARLY TRIGGER: on cycle {c} the endstop stopped the return move at {pos} "
          f"({(pos - R) * args.um_per_pulse:.1f} um before its target {R}).")
    # Lost steps, or the carriage leading the step count while decelerating? Measure the
    # edge again at the slow homing speed: ~pos means steps were lost, ~0 means lag only.
    time.sleep(0.2)
    back_off()
    hit = approach(args.home_slow_us, args.backoff + 400)
    verdict = ("steps were lost" if abs(hit) > max(abs(repeat_offset), args.early_tolerance)
               else "no steps lost; the early stop was dynamic (load angle / compliance while decelerating)")
    print(f"slow re-approach: edge at {hit:+d} pulses ({hit * args.um_per_pulse:+.2f} um) -> {verdict}. "
          f"Homing repeatability for reference: {repeat_offset:+d} pulses")
    write(dev, min_pos_reg, 0)
    disable(dev, 1 << m)
    dev.close()
    raise SystemExit(2)

# ---- 3. endstop check ------------------------------------------------------------------
back_off()
hit = approach(args.home_slow_us, args.backoff + R + 400)
print(f"endstop check: sensor edge now at {hit:+d} pulses (0 = no drift; "
      f"{hit * args.um_per_pulse:+.2f} um). Homing repeatability for reference: {repeat_offset:+d} pulses")
write(dev, getattr(sd, f"Motor{m}AccumulatedSteps"), 0)
write(dev, min_pos_reg, -1)   # as 06_home.py leaves it: home, negative moves refused
disable(dev, 1 << m)
dev.close()
raise SystemExit(0 if ok else 1)
