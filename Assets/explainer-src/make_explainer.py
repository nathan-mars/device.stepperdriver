import math, sys, html
sys.path.insert(0, sys.argv[1])
from sim import old_ramp, new_trapezoid, F_TICK, DT
OUT = sys.argv[2]
UM = 1.25                         # um per pulse (8 microsteps, 2 mm lead screw)
def mmps(v_steps): return v_steps * UM / 1000.0

# ---------- data ----------
# A: same start/top speed (2000 us -> 100 us), 4000 pulses = 5 mm
old = old_ramp(4000, 2000, 100, 10)
n = (2000-100)//10; T = sum(1e-6*(2000-10*i) for i in range(n)); a_eq = (1e4-500)/T
new = new_trapezoid(4000, 500, 10000, a_eq)
# B: 15 mm, 1 -> 100 mm/s, 2.5 m/s^2
v0 = 1/UM*1000; vmax = 100/UM*1000; acc = 2.5e6/UM
fast = new_trapezoid(int(15000/UM), v0, vmax, acc)

def per_ms(data):
    """one sample per planner tick: (t_ms, v_mm_s, period_us)"""
    out=[]; nxt=0.0
    for t,v,p in data:
        if t >= nxt:
            out.append((t*1e3, mmps(v), p)); nxt += DT
    out.append((data[-1][0]*1e3, mmps(data[-1][1]), data[-1][2]))
    return out
A_old, A_new, B = per_ms(old), per_ms(new), per_ms(fast)

# ---------- svg helpers ----------
W, H = 720, 300
ML, MR, MT, MB = 56, 16, 20, 40
PW, PH = W-ML-MR, H-MT-MB
def scale(v, lo, hi, a, b): return a + (v-lo)/(hi-lo)*(b-a)
def nice_ticks(lo, hi, n=5):
    span = hi-lo; raw = span/n; mag = 10**math.floor(math.log10(raw))
    step = min([1,2,2.5,5,10], key=lambda s: abs(s*mag-raw))*mag
    t = math.ceil(lo/step)*step; out=[]
    while t <= hi+1e-9: out.append(round(t,10)); t += step
    return out
def path(pts, xlo, xhi, ylo, yhi, ylog=False):
    d=[]
    for i,(x,y) in enumerate(pts):
        X = scale(x, xlo, xhi, ML, ML+PW)
        yy = math.log10(y) if ylog else y
        Y = scale(yy, ylo, yhi, MT+PH, MT)
        d.append(f"{'M' if i==0 else 'L'}{X:.1f} {Y:.1f}")
    return " ".join(d)
def svg_plot(pid, title, xlabel, ylabel, series, xmax, ymax, ylog=False, ylo=0, end_labels=True):
    """series: list of (name, slot, pts)"""
    xt = nice_ticks(0, xmax); 
    if ylog:
        yt = [10**k for k in range(int(math.floor(ylo)), int(math.ceil(ymax))+1)]
        ylo_s, ymax_s = ylo, ymax
    else:
        yt = nice_ticks(ylo, ymax); ylo_s, ymax_s = ylo, ymax
    g = [f'<svg class="plot" id="{pid}" viewBox="0 0 {W} {H}" role="img" aria-labelledby="{pid}-t">']
    g.append(f'<title id="{pid}-t">{html.escape(title)}</title>')
    for t in yt:
        yy = math.log10(t) if ylog else t
        Y = scale(yy, ylo_s, ymax_s, MT+PH, MT)
        g.append(f'<line class="grid" x1="{ML}" x2="{ML+PW}" y1="{Y:.1f}" y2="{Y:.1f}"/>')
        lab = f"{t:g}"
        g.append(f'<text class="tick" x="{ML-8}" y="{Y+4:.1f}" text-anchor="end">{lab}</text>')
    for t in xt:
        X = scale(t, 0, xmax, ML, ML+PW)
        g.append(f'<text class="tick" x="{X:.1f}" y="{MT+PH+18}" text-anchor="middle">{t:g}</text>')
    g.append(f'<line class="axis" x1="{ML}" x2="{ML+PW}" y1="{MT+PH}" y2="{MT+PH}"/>')
    g.append(f'<text class="label" x="{ML+PW/2:.0f}" y="{H-6}" text-anchor="middle">{html.escape(xlabel)}</text>')
    g.append(f'<text class="label" transform="translate(14 {MT+PH/2:.0f}) rotate(-90)" text-anchor="middle">{html.escape(ylabel)}</text>')
    for name, slot, pts in series:
        g.append(f'<path class="line s{slot}" d="{path(pts, 0, xmax, ylo_s, ymax_s, ylog)}"/>')
        if end_labels:
            x,y = pts[-1]; yy = math.log10(y) if ylog else y
            X = scale(x,0,xmax,ML,ML+PW); Y = scale(yy, ylo_s, ymax_s, MT+PH, MT)
            g.append(f'<circle class="dot s{slot}" cx="{X:.1f}" cy="{Y:.1f}" r="4"/>')
    # hover layer
    g.append(f'<line class="xhair" x1="0" x2="0" y1="{MT}" y2="{MT+PH}" visibility="hidden"/>')
    g.append(f'<rect class="hit" x="{ML}" y="{MT}" width="{PW}" height="{PH}" fill="transparent"/>')
    g.append('</svg>')
    return "\n".join(g)

def series_json(series, xmax):
    import json
    return json.dumps({"xmax": xmax, "ML": ML, "PW": PW,
                       "series": [{"name": n, "pts": [[round(x,2), round(y,2)] for x,y in pts]} for n,_,pts in series]})

# Plot A
A_series = [("Firmware 0.9 and earlier", 2, [(t, v) for t,v,p in A_old]),
            ("New firmware (1.0)", 1, [(t, v) for t,v,p in A_new])]
A_xmax = math.ceil(max(A_old[-1][0], A_new[-1][0])/100)*100
plotA = svg_plot("plotA", "Speed over time, old vs new firmware, same settings", "time (ms)", "speed (mm/s)", A_series, A_xmax, 14)
# Plot B (speed) and B2 (interval, log)
B_series = [("New firmware, 15 mm move", 1, [(t, v) for t,v,p in B])]
B_xmax = math.ceil(B[-1][0]/20)*20
plotB = svg_plot("plotB", "Speed over time for a 15 mm move at up to 100 mm/s", "time (ms)", "speed (mm/s)", B_series, B_xmax, 110)
B2_series = [("Time between pulses", 1, [(t, p) for t,v,p in B])]
plotB2 = svg_plot("plotB2", "Time between pulses for the same move", "time (ms)", "µs between pulses (log)", B2_series, B_xmax, 4, ylog=True, ylo=1)

# table rows (every 10th sample)
def table_rows(samples, cols):
    rows=[]
    for i,s in enumerate(samples):
        if i % 10 == 0 or i == len(samples)-1:
            rows.append("<tr>" + "".join(f"<td>{c(s)}</td>" for c in cols) + "</tr>")
    return "\n".join(rows)
tabA = table_rows(list(zip(A_old, A_new + [A_new[-1]]*(len(A_old)-len(A_new)))) if len(A_old)>=len(A_new) else list(zip(A_old+[A_old[-1]]*(len(A_new)-len(A_old)), A_new)),
    [lambda s: f"{s[0][0]:.0f}", lambda s: f"{s[0][1]:.2f}", lambda s: f"{s[1][1]:.2f}"])
tabB = table_rows(B, [lambda s: f"{s[0]:.0f}", lambda s: f"{s[1]:.1f}", lambda s: f"{s[2]:.1f}"])

conv = [(1,0.5),(5,0.5),(10,0.5),(25,0.5),(50,0.5),(100,0.5)]
conv_rows = "\n".join(f"<tr><td>{v}</td><td>{v*1000/UM:,.0f}</td><td>{UM/v*1000:.1f}</td></tr>" for v,_ in conv)

old_T = A_old[-1][0]; new_T = A_new[-1][0]

page = f"""<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Trapezoidal Motion Explained</title>
<meta name="description" content="Why the Harp Stepper Driver can now run true trapezoidal speed profiles on a microcontroller without a floating-point unit, and what the admission rules for simultaneous moves are.">
<style>
:root {{
  color-scheme: light;
  --page: #f9f9f7; --surface: #fcfcfb; --ink: #0b0b0b; --ink-2: #52514e; --muted: #898781;
  --grid: #e1e0d9; --axis: #c3c2b7; --border: rgba(11,11,11,0.10);
  --s1: #2a78d6; --s2: #eb6834; --s1-wash: rgba(42,120,214,0.10); --s2-wash: rgba(235,104,52,0.10);
  --note: #eef4fc;
}}
@media (prefers-color-scheme: dark) {{
  :root:not([data-theme="light"]) {{
    color-scheme: dark;
    --page: #0d0d0d; --surface: #1a1a19; --ink: #f0efec; --ink-2: #c3c2b7; --muted: #898781;
    --grid: #2c2c2a; --axis: #383835; --border: rgba(255,255,255,0.10);
    --s1: #3987e5; --s2: #d95926; --s1-wash: rgba(57,135,229,0.12); --s2-wash: rgba(217,89,38,0.12);
    --note: #1f2630;
  }}
}}
:root[data-theme="dark"] {{
  color-scheme: dark;
  --page: #0d0d0d; --surface: #1a1a19; --ink: #f0efec; --ink-2: #c3c2b7; --muted: #898781;
  --grid: #2c2c2a; --axis: #383835; --border: rgba(255,255,255,0.10);
  --s1: #3987e5; --s2: #d95926; --s1-wash: rgba(57,135,229,0.12); --s2-wash: rgba(217,89,38,0.12);
  --note: #1f2630;
}}
* {{ box-sizing: border-box; }}
body {{ margin: 0; background: var(--page); color: var(--ink); font: 16px/1.55 system-ui, -apple-system, "Segoe UI", sans-serif; }}
main {{ max-width: 760px; margin: 0 auto; padding: 32px 16px 64px; }}
h1 {{ font-size: 28px; line-height: 1.2; margin: 0 0 4px; }}
.sub {{ color: var(--ink-2); margin: 0 0 24px; }}
h2 {{ font-size: 20px; margin: 36px 0 8px; }}
p {{ margin: 0 0 14px; }}
figure {{ margin: 20px 0; background: var(--surface); border: 1px solid var(--border); border-radius: 10px; padding: 12px 12px 8px; }}
figcaption {{ font-size: 14px; color: var(--ink-2); padding: 6px 4px 2px; }}
.plot {{ width: 100%; height: auto; display: block; }}
.grid {{ stroke: var(--grid); stroke-width: 1; }}
.axis {{ stroke: var(--axis); stroke-width: 1; }}
.tick {{ fill: var(--muted); font-size: 11px; font-variant-numeric: tabular-nums; }}
.label {{ fill: var(--ink-2); font-size: 12px; }}
.line {{ fill: none; stroke-width: 2; stroke-linejoin: round; stroke-linecap: round; }}
.line.s1, .dot.s1 {{ stroke: var(--s1); }} .dot.s1 {{ fill: var(--s1); }}
.line.s2, .dot.s2 {{ stroke: var(--s2); }} .dot.s2 {{ fill: var(--s2); }}
.dot {{ stroke: var(--surface); stroke-width: 2; }}
.xhair {{ stroke: var(--axis); stroke-width: 1; }}
.legend {{ display: flex; gap: 18px; flex-wrap: wrap; font-size: 13px; color: var(--ink-2); padding: 2px 4px 6px; }}
.legend span::before {{ content: ""; display: inline-block; width: 18px; height: 2px; vertical-align: middle; margin-right: 6px; border-radius: 1px; }}
.legend .k1::before {{ background: var(--s1); }} .legend .k2::before {{ background: var(--s2); }}
.readout {{ font-size: 13px; color: var(--ink-2); min-height: 18px; padding: 0 4px; font-variant-numeric: tabular-nums; }}
details {{ font-size: 13px; color: var(--ink-2); padding: 0 4px; }}
table {{ border-collapse: collapse; font-size: 13px; width: 100%; }}
.changes th, .changes td {{ text-align: left; vertical-align: top; font-variant-numeric: normal; }}
.changes code {{ font-size: 12px; }}
th, td {{ text-align: right; padding: 4px 8px; border-bottom: 1px solid var(--grid); font-variant-numeric: tabular-nums; }}
th:first-child, td:first-child {{ text-align: left; }}
.note {{ background: var(--note); border-radius: 10px; padding: 14px 16px; margin: 16px 0; }}
.note h3 {{ margin: 0 0 6px; font-size: 16px; }}
.diagram {{ width: 100%; height: auto; display: block; }}
.lane {{ fill: var(--ink-2); font-size: 12px; font-weight: 600; }}
.dtext {{ fill: var(--ink); font-size: 12px; }}
.dmuted {{ fill: var(--muted); font-size: 11px; }}
.blk1 {{ fill: var(--s1-wash); stroke: var(--s1); stroke-width: 1.5; }}
.blk2 {{ fill: var(--s2-wash); stroke: var(--s2); stroke-width: 1.5; }}
.pulse {{ stroke: var(--ink-2); stroke-width: 1.5; fill: none; }}
.rule {{ stroke: var(--axis); stroke-width: 1; }}
@media print {{ body {{ background: #fff; color: #000; }} main {{ padding: 0; max-width: none; }} figure {{ break-inside: avoid; border-color: #ccc; }} details {{ display: none; }} }}
</style>
</head>
<body>
<main>
<h1>Trapezoidal speed control on the Harp Stepper Driver</h1>
<p class="sub">Why it is possible on this microcontroller, what changed in firmware 1.0, and how fast several axes can move at once.</p>

<p>A stepper motor is moved by sending it a train of pulses; each pulse advances the shaft by one micro-step (1.25&nbsp;µm on our lead-screw axes). Speed is simply how closely the pulses are spaced: 100&nbsp;mm/s means a pulse every 12.5&nbsp;µs, 1&nbsp;mm/s means one every 1250&nbsp;µs. A motor cannot jump from standstill to 100&nbsp;mm/s; it must be accelerated, held at speed, and decelerated, so that speed plotted against time forms a trapezoid. Doing that in firmware means recomputing the pulse spacing as the speed changes, and the spacing is <em>one divided by the speed</em>. The driver's microcontroller (an 8-bit AVR with no floating-point hardware) needs about 20 to 28&nbsp;µs to perform that division. The original firmware performed it <em>after every pulse</em>, which is arithmetically natural but physically impossible once pulses are closer together than the division takes. Hence the conclusion that a trapezoid could only be approximated, and only slowly: the "quick movement" mode that shipped had to spread one division across eight pulses and switch to a crude fallback at high speed, and ordinary moves used a simpler scheme that is not a trapezoid at all (the orange curve below).</p>

<p>The resolution is to separate two jobs that had been fused together. The pulses themselves are produced by a hardware timer that runs with no software involvement; firmware is only interrupted once per pulse to count it, which takes about 2&nbsp;µs. The speed, on the other hand, does not need to be recomputed per pulse. Updating it one thousand times per second is more than enough: at 2.5&nbsp;m/s² the speed changes by only 2.5&nbsp;mm/s per millisecond, a step far smaller than the motor's own mechanical response can resolve. So the division (and one square root, used to decelerate so that the move lands on exactly the commanded pulse count) now runs once per millisecond in a low-priority "planner", costing about 50&nbsp;µs of every 1000, independent of motor speed. Position is still exact because every pulse is counted in hardware-triggered interrupts; only the speed is piecewise constant over 1&nbsp;ms windows. This is the same architecture used by essentially every CNC and 3D-printer controller on similar 8-bit processors, and it lets all four axes use true trapezoidal profiles, with travel limits and stop inputs honoured during the move, something the previous quick-movement mode could not do.</p>

<figure>
<div class="legend"><span class="k2">Firmware 0.9 and earlier</span><span class="k1">Firmware 1.0</span></div>
{plotA}
<div class="readout" id="plotA-r">Hover or tap the plot to read values.</div>
<figcaption>A 5&nbsp;mm move with identical settings (starting at 0.6&nbsp;mm/s, top speed 12.5&nbsp;mm/s, the fastest firmware 0.9 allowed for ordinary moves). The old ramp removed a fixed 10&nbsp;µs from the pulse spacing on every pulse, which is gentle at low speed and violent near the top; it took {old_T:.0f}&nbsp;ms. Firmware 1.0 reaches the same top speed with a constant acceleration matched to the old ramp's duration and finishes in {new_T:.0f}&nbsp;ms.</figcaption>
<details><summary>Data table (every 10&nbsp;ms)</summary><table><thead><tr><th>time (ms)</th><th>old (mm/s)</th><th>new (mm/s)</th></tr></thead><tbody>
{tabA}
</tbody></table></details>
</figure>

<figure>
{plotB}
<div class="readout" id="plotB-r">Hover or tap the plot to read values.</div>
{plotB2}
<div class="readout" id="plotB2-r"></div>
<figcaption>A 15&nbsp;mm move from 1&nbsp;mm/s to 100&nbsp;mm/s at 2.5&nbsp;m/s², as firmware 1.0 executes it, finishing in {B[-1][0]:.0f}&nbsp;ms (measured on the bench: 190&nbsp;ms by the device's own clock). The lower panel shows the same move as the time between consecutive pulses, from 1250&nbsp;µs at the start down to 12.5&nbsp;µs at full speed. The tiny stair-steps are the once-per-millisecond speed updates.</figcaption>
<details><summary>Data table (every 10&nbsp;ms)</summary><table><thead><tr><th>time (ms)</th><th>speed (mm/s)</th><th>µs between pulses</th></tr></thead><tbody>
{tabB}
</tbody></table></details>
</figure>

<h2>What runs when</h2>
<figure>
<svg class="diagram" viewBox="0 0 720 250" role="img" aria-label="Timeline comparing per-pulse work in the old and new firmware with the once-per-millisecond planner">
  <text class="lane" x="0" y="22">Pulses at 100 mm/s</text>
  <line class="rule" x1="150" x2="700" y1="40" y2="40"/>
  {"".join(f'<path class="pulse" d="M{150+i*62.5} 40 v-12 h6 v12"/>' for i in range(9))}
  <text class="dmuted" x="150" y="60">12.5 µs between pulses, generated by the hardware timer</text>

  <text class="lane" x="0" y="96">Old firmware, per pulse</text>
  {"".join(f'<rect class="blk2" x="{150+i*62.5}" y="80" width="{62.5*1.8:.0f}" height="20" rx="4"/>' for i in range(0,9,2))}
  <text class="dtext" x="150" y="122">compute 1 ÷ speed: 20–28 µs, longer than the gap between pulses, so it falls behind</text>

  <text class="lane" x="0" y="160">New firmware, per pulse</text>
  {"".join(f'<rect class="blk1" x="{150+i*62.5}" y="146" width="10" height="18" rx="3"/>' for i in range(9))}
  <text class="dtext" x="150" y="186">count one pulse, stop if it was the last: about 2 µs</text>

  <text class="lane" x="0" y="222">New firmware, per ms</text>
  <rect class="blk1" x="150" y="208" width="24" height="18" rx="3"/>
  <text class="dtext" x="184" y="222">planner: new speed → new pulse spacing → timer, about 50 µs in every 1000 µs</text>
</svg>
<figcaption>The division that defeated the per-pulse approach still happens, but once per millisecond instead of once per pulse. At 100&nbsp;mm/s that is 80 pulses for every planner update.</figcaption>
</figure>

<div class="note">
<h3>How fast can several axes move at the same time?</h3>
<p>Each pulse still costs the processor about 2.5&nbsp;µs, so the total pulse rate of all moving axes is limited. The firmware applies a simple admission rule when a move is requested, and refuses the request (the register write returns an error, nothing moves) rather than running it badly. For axes with 1.25&nbsp;µm per pulse the rule reads:</p>
<ul>
<li>No single axis faster than <strong>125&nbsp;mm/s</strong> (10&nbsp;µs between pulses).</li>
<li>The speeds of all axes moving at the same time add up to at most <strong>150&nbsp;mm/s</strong>. For example: one axis at 100&nbsp;mm/s plus another at 50; two axes at 75; four axes at 37 each. Slow moves (a few mm/s) on all four axes together are always accepted.</li>
</ul>
<p>Travel limits, stop inputs, the emergency input and position reporting work during every move, on every axis.</p>
</div>

<h2>Speed and pulse spacing, side by side</h2>
<table><thead><tr><th>speed (mm/s)</th><th>pulses per second</th><th>µs between pulses</th></tr></thead><tbody>
{conv_rows}
</tbody></table>
<p class="sub" style="margin-top:8px">For 1.25&nbsp;µm per pulse (1.8° motor, 8 micro-steps, 2&nbsp;mm lead). Other axes scale proportionally.</p>

<h2>Appendix: what changed from firmware 0.9 to 1.0</h2>
<p class="sub">For users of the Harp registers and the Bonsai package. Firmware 0.8 and 0.9 only updated the Harp core and the hardware target; the motion code was last changed in 0.7.</p>
<table class="changes"><thead><tr><th>Area</th><th>Firmware 0.9</th><th>Firmware 1.0</th></tr></thead><tbody>
<tr><td>Speed profile</td><td>Pulse interval shortened by a fixed amount per pulse (not a constant acceleration); quick-movement mode on motors 1 and 2 only</td><td>Constant-acceleration trapezoid for every move on all four motors</td></tr>
<tr><td>Acceleration</td><td><code>Motor{{n}}StepAccelerationInterval</code>, µs removed per pulse</td><td>New <code>Motor{{n}}Acceleration</code> (addresses 141–144) in steps/s². The old register still works: writing it sets an acceleration that reproduces the old ramp's duration</td></tr>
<tr><td>Speed range</td><td><code>StepInterval</code> ≥ 100 µs (≤ 12.5 mm/s at 1.25 µm/pulse)</td><td><code>StepInterval</code> ≥ 10 µs (up to 125 mm/s); <code>MaximumStepInterval</code> ≤ 16 ms</td></tr>
<tr><td>Several axes at once</td><td>No rule; fast quick-moves starved the rest of the firmware</td><td>Admission rule: speeds of moving axes add up to at most 150 mm/s (at 1.25 µm/pulse). A move that would exceed it is refused with an error reply</td></tr>
<tr><td>Quick-movement registers (130–140)</td><td>Separate code path, no travel limits, blocked other commands</td><td>Converted into the same engine; limits and stops apply. Unit descriptions corrected (µm per pulse, m/s²)</td></tr>
<tr><td>Constant-speed mode (<code>StepRelative</code>)</td><td>Immediate speed and direction changes</td><td>Speed changes within 1 ms; a direction change stops first; travel limits apply; subject to the admission rule</td></tr>
<tr><td>Stopping</td><td>Pulse train cut immediately</td><td>Stop completes at the next pulse, so the pulse count is always exact; at most one pulse interval of delay (16 ms at the slowest setting, typically ≤ 2 ms)</td></tr>
<tr><td>Travel limits (<code>Max/MinPosition</code>)</td><td>Abrupt stop when reached; not enforced in quick-movement mode</td><td>Target clamped so the move decelerates into the limit; enforced for every move. 0 still means disabled</td></tr>
<tr><td>Absolute moves while moving</td><td>Delta computed from a position that could be stale</td><td>Motor stops, position settles, then the exact delta is issued</td></tr>
<tr><td>Register names</td><td><code>Mortor{{n}}AccumulatedSteps</code> (typo, fixed in 0.9 too)</td><td><code>Motor{{n}}AccumulatedSteps</code></td></tr>
<tr><td>Position reporting</td><td>Updated on every pulse</td><td>Updated every millisecond (events are at most 100 Hz, so no visible difference)</td></tr>
<tr><td>Versions</td><td>Firmware 0.9, interface 0.4.0, Harp core 1.15</td><td>Firmware 1.0, interface 1.0.0, Harp core 1.15; reported hardware version selectable at build time (1.0 or 1.1)</td></tr>
</tbody></table>
<p>Unchanged: motor enable/disable, currents, microstep resolution, encoders, digital inputs and their stop modes, interlock, events, and every register address that existed before.</p>

<script>
const DATA = {{
  plotA: {series_json(A_series, A_xmax)},
  plotB: {series_json(B_series, B_xmax)},
  plotB2: {series_json(B2_series, B_xmax)}
}};
const UNITS = {{ plotA: " mm/s", plotB: " mm/s", plotB2: " µs" }};
for (const id of Object.keys(DATA)) {{
  const svg = document.getElementById(id), d = DATA[id], out = document.getElementById(id + "-r");
  const xh = svg.querySelector(".xhair"), hit = svg.querySelector(".hit");
  const toX = (ev) => {{ const pt = svg.createSVGPoint(); pt.x = ev.clientX; pt.y = ev.clientY; return pt.matrixTransform(svg.getScreenCTM().inverse()).x; }};
  const show = (ev) => {{
    const X = Math.min(Math.max(toX(ev), d.ML), d.ML + d.PW);
    const t = (X - d.ML) / d.PW * d.xmax;
    xh.setAttribute("x1", X); xh.setAttribute("x2", X); xh.setAttribute("visibility", "visible");
    out.textContent = "t = " + t.toFixed(0) + " ms: " + d.series.map(s => {{
      let best = s.pts[0]; for (const p of s.pts) if (Math.abs(p[0]-t) < Math.abs(best[0]-t)) best = p;
      return s.name + " " + best[1].toFixed(1) + UNITS[id];
    }}).join(", ");
  }};
  hit.addEventListener("pointermove", show); hit.addEventListener("pointerdown", show);
  hit.addEventListener("pointerleave", () => {{ xh.setAttribute("visibility", "hidden"); }});
}}
</script>
</main>
</body>
</html>
"""
open(OUT, "w", encoding="utf-8").write(page)
print(OUT, len(page), "bytes; old", round(old_T), "ms new", round(new_T), "ms fast", round(B[-1][0]), "ms; samples", len(A_old), len(A_new), len(B))
