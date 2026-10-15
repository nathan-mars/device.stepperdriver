# /// script
# requires-python = ">=3.10"
# dependencies = ["harp>=0.5", "pyserial"]
# ///
"""Step 3: record AccumulatedSteps (and the encoder, if wired) during a move and save a CSV
of time, position and speed, to check the trapezoid shape.

    uv run 03_log_profile.py --motor 1 --steps 12000 --speed-us 12.5 --accel 2000000 --out profile.csv
Plot with:
    uv run --with pandas --with matplotlib python -c "import pandas as pd; import matplotlib.pyplot as plt; d=pd.read_csv('profile.csv'); d.plot(x='t_s', y='speed_steps_s'); plt.show()"
"""
import argparse
import csv
import threading
import time
from stepper import add_port_arg, add_interlock_arg, set_interlock, open_stepper, configure_motor, enable, disable, move_relative, sd, write

p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
add_port_arg(p)
add_interlock_arg(p)
p.add_argument("--motor", type=int, default=1)
p.add_argument("--steps", type=int, default=12000)
p.add_argument("--speed-us", type=float, default=12.5)
p.add_argument("--start-us", type=int, default=1250)
p.add_argument("--accel", type=int, default=2000000)
p.add_argument("--current", type=float, default=0.5)
p.add_argument("--out", default="profile.csv")
args = p.parse_args()

dev = open_stepper(args.port)
set_interlock(dev, args.interlock)
m = args.motor
configure_motor(dev, m, step_interval_us=int(round(args.speed_us)), start_interval_us=args.start_us,
                accel_steps_s2=args.accel, current_a=args.current)

rows = []
lock = threading.Lock()
t0 = time.perf_counter()

def on_steps(msg):
    with lock:
        rows.append((time.perf_counter() - t0, int(msg.payload.payload_array[m])))

sub = dev.subscribe(sd.AccumulatedSteps, on_steps)
write(dev, sd.AccumulatedStepsSamplingRate, 3)        # 100 Hz events
enable(dev, 1 << m)
time.sleep(0.2)
move_relative(dev, m, args.steps)
time.sleep(0.5)
while True:                                           # wait until position stops changing
    with lock:
        tail = rows[-6:]
    if len(tail) == 6 and len({r[1] for r in tail}) == 1:
        break
    time.sleep(0.05)
write(dev, sd.AccumulatedStepsSamplingRate, 0)
sub.unsubscribe()
disable(dev, 1 << m)
dev.close()

with open(args.out, "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["t_s", "position", "speed_steps_s"])
    for i, (t, x) in enumerate(rows):
        v = (rows[i][1] - rows[i - 1][1]) / (rows[i][0] - rows[i - 1][0]) if i else 0.0
        w.writerow([f"{t:.4f}", x, f"{v:.0f}"])
print(f"{len(rows)} samples written to {args.out}; final position {rows[-1][1] if rows else 'n/a'}")
