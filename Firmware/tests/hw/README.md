# Hardware tests

Python scripts that drive a connected StepperDriver through its USB serial port.
They need [uv](https://docs.astral.sh/uv/) (`brew install uv`); each script declares its
dependencies (`harp`, `pyserial`) inline, so `uv run` creates the environment on first use.

```sh
cd Firmware/tests/hw
uv run 01_identify.py              # finds the first /dev/tty.usbserial-*; or --port ...
uv run 02_move_and_check.py --motor 1 --steps 4000
uv run 02_move_and_check.py --motor 1 --steps 12000 --speed-us 12.5 --accel 2000000   # 15 mm at 100 mm/s
uv run 03_log_profile.py --motor 1 --out profile.csv
uv run 04_limits_and_stop.py --motor 1

# If `make flash` (Harp Toolkit) stops at 30 % with "did not respond with a bootloader
# message", find out what the board does after a reset, then flash with the same protocol:
uv run flash.py --probe
uv run flash.py ../../StepperDriver/build/StepperDriver-fw1.0-harp1.15-hw1.0-ass0.hex
```

Register classes come straight from `device.yml` (via `harp.device.schema`), so the
scripts follow the register names in that file (`Motor1StepInterval`,
`Motor1Acceleration`, `AccumulatedSteps`, ...). Keep the motor free to move and start
with a low current (`--current 0.5`).
