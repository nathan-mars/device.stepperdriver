"""Simulate the old linear-period ramp and the new 1 kHz-planner trapezoid.
Units: steps, seconds. Output: lists of (t, v_steps_per_s, period_us)."""
import math

F_TICK = 4_000_000.0      # DIV8 at 32 MHz -> 0.25 us per tick
DT = 0.001                # planner tick

def old_ramp(total_steps, p_start_us, p_min_us, dp_us):
    """Firmware <=0.7: period -= dp each step until p_min; symmetric decel."""
    t = 0.0; out = []
    p = p_start_us
    ramp_steps = (p_start_us - p_min_us) // dp_us
    for n in range(total_steps):
        remaining = total_steps - n
        out.append((t, 1e6 / p, p))
        t += p * 1e-6
        if remaining <= ramp_steps and remaining <= n:
            p = min(p + dp_us, p_start_us)
        else:
            p = max(p - dp_us, p_min_us)
    return out

def new_trapezoid(total_steps, v0, vmax, a):
    """Firmware 1.0 model: step ISR counts; planner at 1 kHz sets period."""
    t = 0.0; out = []
    remaining = total_steps
    v = v0
    decel = False
    next_tick = DT              # firmware launches at v0; first planner update 1 ms later
    period_ticks = max(9, min(65535, round(F_TICK / v)))
    while remaining > 0:
        if t >= next_tick:               # planner
            next_tick += DT
            d_stop = (v*v - v0*v0) / (2*a) + v*DT
            if remaining <= d_stop:
                decel = True
            if decel:
                v = max(v0, math.sqrt(v0*v0 + 2*a*remaining))
            else:
                v = min(vmax, v + a*DT)
            period_ticks = max(9, min(65535, round(F_TICK / v)))
        p_us = period_ticks / F_TICK * 1e6
        out.append((t, 1e6 / p_us, p_us))
        t += p_us * 1e-6
        remaining -= 1
    return out

def summarize(name, data):
    ts = [d[0] for d in data]; vs = [d[1] for d in data]
    print(f"{name}: steps={len(data)} time={ts[-1]*1e3:.1f} ms vmax={max(vs):.0f} steps/s "
          f"v_first={vs[0]:.0f} v_last={vs[-1]:.0f}")

if __name__ == "__main__":
    # Same start/top speed for both: start 2000 us, top 100 us (defaults-ish, old mode max speed)
    old = old_ramp(4000, 2000, 100, 10)
    summarize("old", old)
    # equivalent average acceleration for the old ramp
    n = (2000-100)//10; T = sum(1e-6*(2000 - 10*i) for i in range(n))
    a_eq = (1e4 - 500)/T
    print(f"old ramp: {n} steps, T_ramp={T*1e3:.1f} ms, a_eq={a_eq:.0f} steps/s^2")
    new = new_trapezoid(4000, 500, 10000, a_eq)
    summarize("new-same", new)
    # Fast move: 15 mm at 1.25 um/pulse, 1 -> 100 mm/s, 2.5 m/s^2
    um = 1.25
    fast = new_trapezoid(int(15000/um), 1/um*1000, 100/um*1000, 2.5e6/um*1000/1000)
    summarize("fast", fast)
    short = new_trapezoid(int(1000/um), 1/um*1000, 100/um*1000, 2.5e6/um*1000/1000)
    summarize("short-1mm", short)
