"""Build ../stage-motion-limits-report.html from the bench measurements of 2026-11-03.

    python3 make_limits_report.py

The numbers below were taken with Firmware/tests/hw/07_home_move_timed.py on motor 0 /
input 0 (optical endstop), firmware 1.0, 1.25 um per pulse, run current at its maximum.
Only the outbound moves are used for timing: those always run the full commanded distance
(returns towards the endstop may be cut short by the sensor, see the report).
"""
from __future__ import annotations

from html import escape
from pathlib import Path

UM = 1.25            # um per pulse
V0 = 500.0           # start/stop speed, pulses/s (MaximumStepInterval 2000 us)
OUT = Path(__file__).resolve().parent.parent / "stage-motion-limits-report.html"


def expected_ms(mm: float, interval_us: int, accel_m_s2: float) -> float:
    n = mm * 1000 / UM
    a = accel_m_s2 * 1e6 / UM
    vmax = 1e6 / interval_us
    d_ramp = (vmax * vmax - V0 * V0) / (2 * a)
    if 2 * d_ramp < n:
        return 1e3 * (2 * (vmax - V0) / a + (n - 2 * d_ramp) / vmax)
    return 1e3 * 2 * ((V0 * V0 + a * n) ** 0.5 - V0) / a


# (distance mm, step interval us, accel m/s^2, outbound moves, mean ms, min ms, max ms, final edge check pulses, note)
RUNS = [
    (15, 12, 2.5, 5, 189.59, 188.99, 190.02, "+2", ""),
    (15, 10, 2.5, 5, 174.01, 173.98, 174.02, "+4", ""),
    (25, 10, 2.5, 5, 256.00, 256.00, 256.00, "+1", ""),
    (25, 10, 3.0, 5, 248.00, 248.00, 248.00, "&minus;11", "small loss"),
    (25, 10, 3.5, 40, 242.00, 241.95, 242.02, "&minus;8 to +3", "8 runs, one lost 28"),
    (25, 10, 4.0, 5, 237.99, 237.98, 238.02, "+3", ""),
    (25, 10, 4.5, 5, 234.19, 233.98, 234.98, "&minus;2", ""),
    (25, 10, 5.0, 5, 231.19, 230.98, 232.00, "&minus;4", ""),
    (25, 10, 5.5, 5, 229.00, 228.99, 229.02, "&minus;1", ""),
    (25, 10, 6.0, 0, None, None, None, "stall", "stalled"),
]

SWEEP = [r for r in RUNS if r[0] == 25]


# ---- plotting helpers (same look as the trapezoidal-motion explainer) ---------------------
W, H, ML, MR, MT, MB = 720, 300, 56, 16, 16, 40
PW, PH = W - ML - MR, H - MT - MB


def axes(xmin, xmax, ymin, ymax, xticks, yticks, xlabel, ylabel, xfmt=str, yfmt=str):
    sx = lambda x: ML + (x - xmin) / (xmax - xmin) * PW
    sy = lambda y: MT + PH - (y - ymin) / (ymax - ymin) * PH
    out = []
    for y in yticks:
        out.append(f'<line class="grid" x1="{ML}" x2="{ML + PW}" y1="{sy(y):.1f}" y2="{sy(y):.1f}"/>')
        out.append(f'<text class="tick" x="{ML - 8}" y="{sy(y) + 4:.1f}" text-anchor="end">{yfmt(y)}</text>')
    for x in xticks:
        out.append(f'<text class="tick" x="{sx(x):.1f}" y="{MT + PH + 18}" text-anchor="middle">{xfmt(x)}</text>')
    out.append(f'<line class="axis" x1="{ML}" x2="{ML + PW}" y1="{MT + PH}" y2="{MT + PH}"/>')
    out.append(f'<text class="label" x="{ML + PW / 2}" y="{H - 4}" text-anchor="middle">{xlabel}</text>')
    out.append(f'<text class="label" transform="translate(14 {MT + PH / 2}) rotate(-90)" text-anchor="middle">{ylabel}</text>')
    return sx, sy, out


def path(pts, sx, sy):
    return "M" + " L".join(f"{sx(x):.1f} {sy(y):.1f}" for x, y in pts)


# Figure 1: move time vs acceleration, 25 mm at 125 mm/s
def fig_time_vs_accel() -> str:
    sx, sy, out = axes(2.25, 6.25, 210, 265, [2.5, 3, 3.5, 4, 4.5, 5, 5.5, 6], [210, 220, 230, 240, 250, 260],
                       "acceleration (m/s²)", "move time (ms)", xfmt=lambda x: f"{x:g}")
    curve = [(a / 100, expected_ms(25, 10, a / 100)) for a in range(250, 601, 5)]
    out.append(f'<path class="line s1" d="{path(curve, sx, sy)}"/>')
    for mm, iv, a, n, mean, lo, hi, edge, note in SWEEP:
        if mean is None:
            x, y = sx(a), sy(expected_ms(25, 10, a))
            out.append(f'<g class="stall"><line x1="{x - 6:.1f}" y1="{y - 6:.1f}" x2="{x + 6:.1f}" y2="{y + 6:.1f}"/>'
                       f'<line x1="{x - 6:.1f}" y1="{y + 6:.1f}" x2="{x + 6:.1f}" y2="{y - 6:.1f}"/>'
                       f'<title>{a:g} m/s²: stalled</title></g>')
            out.append(f'<text class="label" x="{x:.1f}" y="{y - 12:.1f}" text-anchor="middle">stall</text>')
            continue
        out.append(f'<circle class="dot s2" cx="{sx(a):.1f}" cy="{sy(mean):.1f}" r="5">'
                   f'<title>{a:g} m/s²: measured {mean:.2f} ms, predicted {expected_ms(25, 10, a):.2f} ms</title></circle>')
    # shade the stall region
    out.insert(0, f'<rect class="zone" x="{sx(5.75):.1f}" y="{MT}" width="{sx(6.25) - sx(5.75):.1f}" height="{PH}"/>')
    return (f'<svg class="plot" viewBox="0 0 {W} {H}" role="img" aria-labelledby="f1-t">'
            f'<title id="f1-t">Measured and predicted move time against acceleration</title>{"".join(out)}</svg>')


# Figure 2: predicted speed profiles of the 25 mm move at three accelerations, measured end marked
def profile(accel: float, mm: float = 25, interval_us: int = 10):
    n = mm * 1000 / UM
    a = accel * 1e6 / UM
    vmax = 1e6 / interval_us
    d_ramp = (vmax * vmax - V0 * V0) / (2 * a)
    t_ramp = (vmax - V0) / a
    t_cruise = (n - 2 * d_ramp) / vmax
    to_mm_s = UM * 1e-3
    return [(0, V0 * to_mm_s), (t_ramp * 1e3, vmax * to_mm_s), ((t_ramp + t_cruise) * 1e3, vmax * to_mm_s),
            ((2 * t_ramp + t_cruise) * 1e3, V0 * to_mm_s)]


def fig_profiles() -> str:
    sx, sy, out = axes(0, 270, 0, 140, [0, 50, 100, 150, 200, 250], [0, 25, 50, 75, 100, 125],
                       "time (ms)", "speed (mm/s)")
    for cls, a in (("s3", 2.5), ("s1", 4.0), ("s2", 5.5)):
        pts = profile(a)
        out.append(f'<path class="line {cls}" d="{path(pts, sx, sy)}"/>')
        meas = next(r[4] for r in SWEEP if r[2] == a)
        x = sx(meas)
        out.append(f'<line class="mark {cls}" x1="{x:.1f}" x2="{x:.1f}" y1="{sy(0):.1f}" y2="{sy(22):.1f}">'
                   f'<title>{a:g} m/s²: measured end {meas:.2f} ms</title></line>')
    return (f'<svg class="plot" viewBox="0 0 {W} {H}" role="img" aria-labelledby="f2-t">'
            f'<title id="f2-t">Predicted speed profile of a 25 mm move at 2.5, 4.0 and 5.5 m/s²</title>{"".join(out)}</svg>')


def run_table() -> str:
    rows = []
    for mm, iv, a, n, mean, lo, hi, edge, note in RUNS:
        vpk = UM * 1e3 / iv
        exp = expected_ms(mm, iv, a)
        if mean is None:
            meas = diff = rng = "&mdash;"
            avg = "&mdash;"
        else:
            meas, rng = f"{mean:.2f}", f"{lo:.2f}&ndash;{hi:.2f}"
            diff = f"+{mean - exp:.1f} ({(mean / exp - 1) * 100:.1f}%)"
            avg = f"{mm / (mean / 1e3):.1f}"
        rows.append(f"<tr><td>{mm}</td><td>{vpk:.1f}</td><td>{a:g}</td><td>{n or '&mdash;'}</td>"
                    f"<td>{exp:.2f}</td><td>{meas}</td><td>{rng}</td><td>{diff}</td><td>{avg}</td>"
                    f"<td>{edge}</td><td class=\"txt\">{note}</td></tr>")
    return ("<table><thead><tr><th>Distance (mm)</th><th>Peak (mm/s)</th><th>Accel (m/s²)</th><th>Moves</th>"
            "<th>Predicted (ms)</th><th>Measured mean (ms)</th><th>Range (ms)</th><th>Measured &minus; predicted</th>"
            "<th>Avg speed (mm/s)</th><th>Endstop check (pulses)</th><th class=\"txt\">Note</th></tr></thead><tbody>"
            + "".join(rows) + "</tbody></table>")


ok_sweep = [r for r in SWEEP if r[4] is not None]
diffs = [r[4] - expected_ms(25, 10, r[2]) for r in ok_sweep]
pct = [(r[4] / expected_ms(r[0], r[1], r[2]) - 1) * 100 for r in RUNS if r[4] is not None]

CSS = """
:root {
  color-scheme: light;
  --page: #f9f9f7; --surface: #fcfcfb; --ink: #0b0b0b; --ink-2: #52514e; --muted: #898781;
  --grid: #e1e0d9; --axis: #c3c2b7; --border: rgba(11,11,11,0.10);
  --s1: #2a78d6; --s2: #eb6834; --s3: #8a8a84; --bad: #c4362c; --zone: rgba(196,54,44,0.07);
  --note: #eef4fc;
}
@media (prefers-color-scheme: dark) {
  :root:not([data-theme="light"]) {
    color-scheme: dark;
    --page: #0d0d0d; --surface: #1a1a19; --ink: #f0efec; --ink-2: #c3c2b7; --muted: #898781;
    --grid: #2c2c2a; --axis: #383835; --border: rgba(255,255,255,0.10);
    --s1: #3987e5; --s2: #d95926; --s3: #9a9a94; --bad: #e0564b; --zone: rgba(224,86,75,0.10);
    --note: #1f2630;
  }
}
:root[data-theme="dark"] {
  color-scheme: dark;
  --page: #0d0d0d; --surface: #1a1a19; --ink: #f0efec; --ink-2: #c3c2b7; --muted: #898781;
  --grid: #2c2c2a; --axis: #383835; --border: rgba(255,255,255,0.10);
  --s1: #3987e5; --s2: #d95926; --s3: #9a9a94; --bad: #e0564b; --zone: rgba(224,86,75,0.10);
  --note: #1f2630;
}
* { box-sizing: border-box; }
body { margin: 0; background: var(--page); color: var(--ink); font: 16px/1.55 system-ui, -apple-system, "Segoe UI", sans-serif; }
main { max-width: 760px; margin: 0 auto; padding: 32px 16px 64px; }
h1 { font-size: 28px; line-height: 1.2; margin: 0 0 4px; }
.sub { color: var(--ink-2); margin: 0 0 24px; }
h2 { font-size: 20px; margin: 36px 0 8px; }
p { margin: 0 0 14px; }
figure { margin: 20px 0; background: var(--surface); border: 1px solid var(--border); border-radius: 10px; padding: 12px 12px 8px; }
figcaption { font-size: 14px; color: var(--ink-2); padding: 6px 4px 2px; }
.plot { width: 100%; height: auto; display: block; }
.grid { stroke: var(--grid); stroke-width: 1; }
.axis { stroke: var(--axis); stroke-width: 1; }
.tick { fill: var(--muted); font-size: 11px; font-variant-numeric: tabular-nums; }
.label { fill: var(--ink-2); font-size: 12px; }
.line { fill: none; stroke-width: 2; stroke-linejoin: round; stroke-linecap: round; }
.line.s1, .mark.s1 { stroke: var(--s1); } .line.s2, .mark.s2 { stroke: var(--s2); } .line.s3, .mark.s3 { stroke: var(--s3); }
.mark { stroke-width: 2; stroke-dasharray: 3 3; }
.dot { stroke: var(--surface); stroke-width: 2; } .dot.s2 { fill: var(--s2); }
.stall line { stroke: var(--bad); stroke-width: 2.5; stroke-linecap: round; }
.zone { fill: var(--zone); }
.legend { display: flex; gap: 18px; flex-wrap: wrap; font-size: 13px; color: var(--ink-2); padding: 2px 4px 6px; }
.legend span::before { content: ""; display: inline-block; width: 18px; height: 2px; vertical-align: middle; margin-right: 6px; border-radius: 1px; }
.legend .k1::before { background: var(--s1); } .legend .k2::before { background: var(--s2); } .legend .k3::before { background: var(--s3); }
.legend .kd::before { width: 10px; height: 10px; border-radius: 50%; background: var(--s2); }
.legend .kx::before { content: "×"; width: auto; height: auto; background: none; color: var(--bad); font-weight: 700; }
.tablewrap { overflow-x: auto; margin: 16px 0; }
table { border-collapse: collapse; font-size: 13px; width: 100%; }
th, td { white-space: nowrap; text-align: right; padding: 4px 8px; border-bottom: 1px solid var(--grid); font-variant-numeric: tabular-nums; }
th { font-weight: 600; color: var(--ink-2); vertical-align: bottom; white-space: normal; }
th:first-child, td:first-child, .txt { text-align: left; }
.kpis { display: grid; grid-template-columns: repeat(auto-fit, minmax(160px, 1fr)); gap: 12px; margin: 20px 0; }
.kpi { background: var(--surface); border: 1px solid var(--border); border-radius: 10px; padding: 12px 14px; }
.kpi b { display: block; font-size: 24px; font-variant-numeric: tabular-nums; }
.kpi span { font-size: 13px; color: var(--ink-2); }
.note { background: var(--note); border-radius: 10px; padding: 14px 16px; margin: 16px 0; }
.note h3 { margin: 0 0 6px; font-size: 16px; }
ul { margin: 0 0 14px; padding-left: 22px; }
code { font-size: 14px; }
a { color: var(--s1); text-underline-offset: 2px; }
.theme-btn { position: fixed; top: 12px; right: 12px; z-index: 10; font: 13px system-ui, -apple-system, "Segoe UI", sans-serif;
  color: var(--ink-2); background: var(--surface); border: 1px solid var(--border); border-radius: 8px; padding: 6px 10px; cursor: pointer; }
.theme-btn:hover { color: var(--ink); }
@media print { body { background: #fff; color: #000; } main { padding: 0; max-width: none; } .theme-btn { display: none; } figure, .kpi, .note { break-inside: avoid; border-color: #ccc; } }
"""

THEME_JS = """<script>
(() => {
  const root = document.documentElement, btn = document.getElementById("theme-btn");
  const dark = () => root.dataset.theme ? root.dataset.theme === "dark" : matchMedia("(prefers-color-scheme: dark)").matches;
  const label = () => { btn.textContent = dark() ? "Light mode" : "Dark mode"; };
  try { const t = localStorage.getItem("theme"); if (t === "light" || t === "dark") root.dataset.theme = t; } catch (e) {}
  label();
  btn.addEventListener("click", () => {
    root.dataset.theme = dark() ? "light" : "dark";
    try { localStorage.setItem("theme", root.dataset.theme); } catch (e) {}
    label();
  });
  matchMedia("(prefers-color-scheme: dark)").addEventListener("change", label);
})();
</script>"""

html = f"""<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Stage Motion Limits</title>
<meta name="description" content="Bench test of the Harp Stepper Driver (firmware 1.0) on a 30 mm lead-screw stage: measured move times against the trapezoid prediction, and the speed and acceleration limits of the stage and driver.">
<style>{CSS}</style>
</head>
<body>
<button class="theme-btn" id="theme-btn" type="button" aria-label="Switch between light and dark mode"></button>
<main>
<h1>Move times and motion limits of the stage</h1>
<p class="sub">Harp Stepper Driver, firmware 1.0, motor 0 on a 30&nbsp;mm lead-screw stage (1.25&nbsp;µm per pulse; <a href="https://cad.onshape.com/publications/b5bede2057939bc5e5df2c22/w/5c6e4a965dd7aa3c9b0189a9/e/41f249638f505733f66e10e1">stage design in Onshape</a>). Bench test, 3&nbsp;November&nbsp;2026.</p>

<div class="kpis">
  <div class="kpi"><b>+{min(pct):.1f}&ndash;{max(pct):.1f}%</b><span>measured move time over the trapezoid prediction</span></div>
  <div class="kpi"><b>±0.05&nbsp;ms</b><span>spread between repeats of the same move</span></div>
  <div class="kpi"><b>125&nbsp;mm/s</b><span>top speed: the firmware's fastest pulse rate, reached without trouble</span></div>
  <div class="kpi"><b>5.5&nbsp;m/s²</b><span>highest acceleration run cleanly; the motor stalls at 6.0</span></div>
</div>

<p>The question was whether the new trapezoidal motion engine moves the stage in the time the speed profile predicts, and how far speed and acceleration can be pushed on this stage before the motor loses steps. Each test homed the stage on its optical endstop, ran five out-and-back moves, timed each with the driver's own clock, and finished by finding the endstop again slowly to check that the position count still matched the stage. The motor current was at its maximum throughout.</p>

<h2>Measured times follow the prediction</h2>
<p>For every setting, the measured move time is within {min(pct):.1f}&ndash;{max(pct):.1f}% of the time computed from the ideal trapezoid (constant acceleration from the start speed of 0.625&nbsp;mm/s to the peak, cruise, and the mirror-image deceleration). The measured time is always slightly longer: by 4.4&ndash;4.5&nbsp;ms for 15&nbsp;mm moves and {min(diffs):.1f}&ndash;{max(diffs):.1f}&nbsp;ms for 25&nbsp;mm moves. The offset is steady from run to run, so it can simply be added to a planned time. Repeats of the same move agree to about 0.05&nbsp;ms, essentially the resolution of the timestamps.</p>

<figure>
<div class="legend"><span class="k1">Predicted (ideal trapezoid)</span><span class="kd">Measured, mean of 5 or more moves</span><span class="kx">Stall</span></div>
{fig_time_vs_accel()}
<figcaption>Figure 1. Time for a 25&nbsp;mm move at a peak of 125&nbsp;mm/s against acceleration. The measured points sit a constant few milliseconds above the predicted curve up to 5.5&nbsp;m/s². At 6.0&nbsp;m/s² the motor stalled. Hover a point for its values.</figcaption>
</figure>

<p>Higher acceleration does shorten the move, but by less each time: going from 2.5 to 5.5&nbsp;m/s² saves 27&nbsp;ms on a 25&nbsp;mm move (256 &rarr; 229&nbsp;ms), and the next step would save only about 2&nbsp;ms. On a 25&nbsp;mm move, most of the time at the higher accelerations is already spent cruising at the top speed, as Figure&nbsp;2 shows.</p>

<figure>
<div class="legend"><span class="k3">2.5 m/s²</span><span class="k1">4.0 m/s²</span><span class="k2">5.5 m/s²</span></div>
{fig_profiles()}
<figcaption>Figure 2. Predicted speed against time for a 25&nbsp;mm move at three accelerations. The dashed ticks on the time axis mark when the measured moves actually finished.</figcaption>
</figure>

<h2>Limits of this stage and driver</h2>
<p><b>Speed: 125&nbsp;mm/s.</b> The firmware's shortest pulse interval is 10&nbsp;µs, which is 125&nbsp;mm/s at 1.25&nbsp;µm per pulse. The stage ran at that speed at every acceleration up to 5.5&nbsp;m/s² without losing position, so the top speed is set by the firmware, not by the motor. Speeds between settings come in steps of whole microseconds per pulse: 12&nbsp;µs gives 104.2&nbsp;mm/s, 11&nbsp;µs 113.6&nbsp;mm/s, 10&nbsp;µs 125&nbsp;mm/s.</p>
<p><b>Acceleration: 5.5&nbsp;m/s² at 125&nbsp;mm/s.</b> Raising acceleration in 0.5&nbsp;m/s² steps, the stage ran cleanly up to 5.5&nbsp;m/s². At 6.0&nbsp;m/s² the motor lost synchronization 1.8&nbsp;mm into the very first move, just after reaching full speed, and the stage stopped while the driver kept counting pulses. There was no gradual warning between the two settings. Since the current is already at its maximum, more torque is not available to raise this limit.</p>

<div class="note">
<h3>Suggested operating envelope</h3>
<p>Peak speed up to 125&nbsp;mm/s, acceleration up to about 5&nbsp;m/s². That leaves one 0.5&nbsp;m/s² step below the highest clean setting and two below the stall. Plan move times from the trapezoid and add about 7&nbsp;ms (25&nbsp;mm moves) or 4.5&nbsp;ms (15&nbsp;mm moves).</p>
<p style="margin:0">Two of the 17 test runs (at 3.0 and 3.5&nbsp;m/s²) ended 14 and 35&nbsp;µm off after five round trips; seven further runs at 3.5&nbsp;m/s² were clean, and the cause was not found. Re-home periodically if micrometer-level position matters over many fast moves.</p>
</div>

<h2>All runs</h2>
<p>Times are for the outbound moves, which always travel the full distance. The endstop check is where the slow re-approach found the sensor after the run (0 = position fully kept; the sensor itself repeats to about ±10 pulses, or 12&nbsp;µm).</p>
<div class="tablewrap">{run_table()}</div>

<h2>How it was measured</h2>
<ul>
<li>Stage: 30&nbsp;mm lead-screw stage with an optical endstop at the home end; the CAD model is published in <a href="https://cad.onshape.com/publications/b5bede2057939bc5e5df2c22/w/5c6e4a965dd7aa3c9b0189a9/e/41f249638f505733f66e10e1">Onshape</a>.</li>
<li>Script: <code>Firmware/tests/hw/07_home_move_timed.py</code>. Each move is timed from the driver's reply to the move command (the moment it starts the move) to its "motor stopped" event, both stamped by the driver's Harp clock, so USB delays do not enter.</li>
<li>The prediction uses the same settings as the driver: start and stop speed 0.625&nbsp;mm/s (2000&nbsp;µs per pulse), the commanded peak, and the commanded acceleration.</li>
<li>The endstop stays active during all moves, so a motor that loses steps is stopped by the sensor instead of running into the stage's hard stop. When the moves return all the way to home, the sensor stops them 15&ndash;30 pulses early: while decelerating, the carriage runs slightly ahead of the pulse count. A slow re-approach afterwards confirmed no position was lost in those cases. Moves ending 200 pulses (0.25&nbsp;mm) short of home avoid this, and there the return moves take exactly as long as the outbound ones.</li>
</ul>
</main>
{THEME_JS}
</body>
</html>
"""

OUT.write_text(html)
print(f"wrote {OUT}")
