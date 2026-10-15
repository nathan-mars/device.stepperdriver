# /// script
# requires-python = ">=3.10"
# dependencies = ["harp>=0.5", "pyserial"]
# ///
"""Step 5: measure the real pulse rate against the commanded one.

Runs the motor in constant-speed mode (StepRelative, no ramp) at a known interval for a
fixed time, then compares the pulses counted by the firmware with the expectation.
Also streams AccumulatedSteps events at 100 Hz and fits the slope, which is independent of
host round-trip latency.

    uv run 05_speed_check.py --motor 1 --interval-us 500 --seconds 5
Expected: 2000 pulses/s, so ~10000 pulses in 5 s.
"""
import argparse
import threading
import time
from stepper import add_port_arg, add_interlock_arg, set_interlock, open_stepper, configure_motor, enable, disable, position, sd, write

p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
add_port_arg(p)
add_interlock_arg(p)
p.add_argument("--motor", type=int, default=1)
p.add_argument("--interval-us", type=int, default=500, help="commanded us between pulses")
p.add_argument("--seconds", type=float, default=5.0)
p.add_argument("--current", type=float, default=0.5)
args = p.parse_args()

dev = open_stepper(args.port)
set_interlock(dev, args.interlock)
m = args.motor
configure_motor(dev, m, step_interval_us=250, start_interval_us=2000, accel_steps_s2=17778, current_a=args.current)
enable(dev, 1 << m)
time.sleep(0.1)

samples = []
lock = threading.Lock()
t0 = time.perf_counter()
sub = dev.subscribe(sd.AccumulatedSteps, lambda msg: samples.append((time.perf_counter() - t0, int(msg.payload.payload_array[m]))))
write(dev, sd.AccumulatedStepsSamplingRate, 3)        # 100 Hz
p_start = position(dev, m)
t_start = time.perf_counter()
write(dev, getattr(sd, f"Motor{m}StepRelative"), args.interval_us)   # positive direction jog
time.sleep(args.seconds)
write(dev, getattr(sd, f"Motor{m}StepRelative"), 0)                  # stop
t_stop = time.perf_counter()
time.sleep(0.3)
p_end = position(dev, m)
write(dev, sd.AccumulatedStepsSamplingRate, 0)
sub.unsubscribe()
disable(dev, 1 << m)
dev.close()

expected_rate = 1e6 / args.interval_us
elapsed = t_stop - t_start
print(f"commanded {expected_rate:.0f} pulses/s for {elapsed:.2f} s -> expected ~{expected_rate * elapsed:.0f} pulses")
print(f"firmware counted {p_end - p_start} pulses -> {(p_end - p_start) / elapsed:.0f} pulses/s ({(p_end - p_start) / (expected_rate * elapsed):.2f}x commanded)")
# slope of the event stream over the middle 60% of the run (steady state, no start/stop effects)
with lock:
    mid = [s for s in samples if 0.2 * args.seconds < s[0] - (t_start - t0) < 0.8 * args.seconds]
if len(mid) > 10:
    (ta, xa), (tb, xb) = mid[0], mid[-1]
    print(f"event-stream slope: {(xb - xa) / (tb - ta):.0f} pulses/s from {len(mid)} samples")
print("If the ratio is ~2x: either the timer runs twice as fast as assumed, or each pulse is counted twice.")
print("Measure the physical travel for a known pulse count to tell them apart (travel = pulses x distance per pulse).")
