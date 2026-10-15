# Explainer source

`trapezoidal-motion-explainer.html` in the parent folder is generated:

```sh
python3 make_explainer.py . ../trapezoidal-motion-explainer.html
```

A PDF can be printed from it locally (it is not committed):

```sh
"/Applications/Google Chrome.app/Contents/MacOS/Google Chrome" --headless=new --disable-gpu \
  --no-pdf-header-footer --print-to-pdf=../trapezoidal-motion-explainer.pdf \
  "file://$PWD/../trapezoidal-motion-explainer.html"
```

`sim.py` models the firmware 0.9 ramp and the firmware 1.0 planner (1 kHz speed updates,
0.25 µs timer ticks) and is also a convenient place to try other profiles. No dependencies
beyond the Python standard library.
