# /// script
# requires-python = ">=3.10"
# dependencies = ["harp>=0.5", "pyserial"]
# ///
"""Step 4: travel limit and StopMotors during a move.

Sets MaxPosition to start+limit, commands a move past it, expects the motor to stop
exactly at the limit. Then starts a long move and stops it with StopMotors.
"""
import argparse
import time
from stepper import add_port_arg, add_interlock_arg, set_interlock, open_stepper, configure_motor, enable, disable, move_relative, position, wait_stopped, sd, write

p = argparse.ArgumentParser(description=__doc__)
add_port_arg(p)
add_interlock_arg(p)
p.add_argument("--motor", type=int, default=1)
p.add_argument("--limit", type=int, default=3000)
p.add_argument("--speed-us", type=float, default=50)
p.add_argument("--accel", type=int, default=200000)
args = p.parse_args()

dev = open_stepper(args.port)
set_interlock(dev, args.interlock)
m = args.motor
configure_motor(dev, m, step_interval_us=int(round(args.speed_us)), start_interval_us=2000, accel_steps_s2=args.accel)
enable(dev, 1 << m)
start = position(dev, m)
write(dev, getattr(sd, f"Motor{m}MaxPosition"), start + args.limit)
move_relative(dev, m, args.limit * 3)
wait_stopped(dev, m)
p1 = position(dev, m)
print(f"limit test: stopped at {p1}, expected {start + args.limit}:", "OK" if p1 == start + args.limit else "FAIL")
write(dev, getattr(sd, f"Motor{m}MaxPosition"), 0)

move_relative(dev, m, -args.limit * 3)
time.sleep(0.15)
write(dev, sd.StopMotors, 1 << m)
wait_stopped(dev, m)
p2 = position(dev, m)
print(f"stop test: stopped at {p2} ({p2 - p1:+d} from the limit), motor idle:", "OK" if p1 - args.limit * 3 < p2 < p1 else "FAIL")
move_relative(dev, m, start - p2)
wait_stopped(dev, m)
print("returned to start:", position(dev, m) == start)
disable(dev, 1 << m)
dev.close()
