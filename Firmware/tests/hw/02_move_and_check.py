# /// script
# requires-python = ">=3.10"
# dependencies = ["harp>=0.5", "pyserial"]
# ///
"""Step 2: move one motor out and back, check AccumulatedSteps returns to start.

Example (1.25 um/pulse axis, 12000 pulses = 15 mm, 100 mm/s, 2.5 m/s^2):
    uv run 02_move_and_check.py --motor 1 --steps 12000 --speed-us 12.5 --accel 2000000
"""
import argparse
import time
from stepper import add_port_arg, add_interlock_arg, set_interlock, open_stepper, configure_motor, enable, disable, position, wait_stopped, sd, write

p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
add_port_arg(p)
add_interlock_arg(p)
p.add_argument("--motor", type=int, default=1)
p.add_argument("--steps", type=int, default=4000)
p.add_argument("--speed-us", type=float, default=100, help="cruise: us between pulses (>= 10)")
p.add_argument("--start-us", type=int, default=2000, help="start/stop: us between pulses")
p.add_argument("--accel", type=int, default=50000, help="steps/s^2")
p.add_argument("--current", type=float, default=0.5)
p.add_argument("--repeat", type=int, default=1)
args = p.parse_args()

dev = open_stepper(args.port)
set_interlock(dev, args.interlock)
m = args.motor
configure_motor(dev, m, step_interval_us=int(round(args.speed_us)), start_interval_us=args.start_us,
                accel_steps_s2=args.accel, current_a=args.current)
enable(dev, 1 << m)
stopped_at = []
# MotorStopped: low nibble = motors now stopped, high nibble = motors that just stopped
sub = dev.subscribe(sd.MotorStopped, lambda msg: stopped_at.append(msg.timestamp) if msg.payload_bytes[0] & (0x10 << m) else None)
time.sleep(0.1)
start = position(dev, m)
ok = True
for i in range(args.repeat):
    for sign in (+1, -1):
        p0 = position(dev, m)
        stopped_at.clear()
        t0 = time.time()
        reply = write(dev, getattr(sd, f"Motor{m}MoveRelative"), sign * args.steps)
        if not wait_stopped(dev, m):
            print("timeout waiting for the motor to stop"); ok = False
        dt = time.time() - t0
        p1 = position(dev, m)
        # device-side duration: write-reply timestamp -> MotorStopped event timestamp
        if stopped_at and reply.timestamp is not None:
            clock = f"{(stopped_at[0] - reply.timestamp) * 1e3:.0f} ms device clock"
        else:
            clock = ("no MotorStopped event received" if not stopped_at else "reply had no timestamp")
        print(f"move {sign * args.steps:+d}: position {p0} -> {p1} (delta {p1 - p0:+d}) in {dt * 1e3:.0f} ms host / {clock}")
        ok &= (p1 - p0) == sign * args.steps
end = position(dev, m)
print("back at start:", end == start, f"(start {start}, end {end})")
v0, vmax, a = 1e6 / args.start_us, 1e6 / args.speed_us, args.accel
d_ramp = (vmax * vmax - v0 * v0) / (2 * a)
if 2 * d_ramp < args.steps:
    t_exp = 2 * (vmax - v0) / a + (args.steps - 2 * d_ramp) / vmax
else:
    vpk = (v0 * v0 + a * args.steps) ** 0.5
    t_exp = 2 * (vpk - v0) / a
print(f"expected duration for this profile: {t_exp * 1e3:.0f} ms")
sub.unsubscribe()
disable(dev, 1 << m)
dev.close()
raise SystemExit(0 if ok and end == start else 1)
