# Harp StepperDriver firmware

Firmware for the Harp Stepper Driver (ATxmega128A1U, four TMC2210 step/dir drivers).
The Microchip Studio project is `StepperDriver/StepperDriver.cproj`; this README covers
the command-line workflow on macOS/Linux, which needs no Microchip tools.

## One-time setup (macOS)

```sh
brew install avr-gcc@15 avr-binutils uv       # compiler + objcopy/size (avr-gcc@15 is keg-only); uv runs the Python tests
dotnet tool install -g Harp.Toolkit            # flashes .hex files over the USB serial port
dotnet tool install -g dotnet-t4               # only needed to regenerate the Bonsai interface
export PATH="$HOME/.dotnet/tools:$PATH"        # add to ~/.zshrc so harp.toolkit is found
```

The Makefile looks for the compiler at `/opt/homebrew/opt/avr-gcc@15/bin`. If yours is
somewhere else: `make AVR_BIN=/path/to/bin`.

## Build

```sh
cd Firmware/StepperDriver
make            # -> build/StepperDriver.hex plus the Harp-named release copy
                #    build/StepperDriver-fw1.0-harp1.15-hw1.0-ass0.hex
make HW=1.1     # same, for the revision 1.1 boards (enable-line/USB-C protection); the
                # device then reports hardware 1.1 and the file is named ...-hw1.1-...
make clean
```

The last line of the build reports SRAM. The chip has 8 KB; the Harp core library's
UART buffer uses most of it, so the build fails on purpose if less than 300 bytes are
left for the stack.

## Flash (Harp Toolkit over USB; no programmer needed)

1. Connect the board over USB and power it.
2. Find the serial port: `ls /dev/tty.usbserial-*` (Windows: a COM port).
3. `make flash PORT=/dev/tty.usbserial-XXXX`
   (this runs `harp.toolkit update build/StepperDriver-fw<...>.hex --port <port>` with
   `DOTNET_ROLL_FORWARD=Major`, because the toolkit targets .NET 8 and newer SDKs only
   ship their own runtime). `harp.toolkit list` prints the serial ports it can see.
4. Power-cycle the board. Reading its WhoAmI should return 1130 and the firmware
   version printed in `app.c`.

If a board has never had the Harp bootloader, the 2x3 header `PDI1` on the PCB accepts an
Atmel-ICE / AVRISP mkII in PDI mode:
`avrdude -c atmelice_pdi -p x128a1u -U application:w:build/StepperDriver.hex:i`
(`application:` writes only the application section and leaves a bootloader intact).

## Motion engine (firmware 1.0)

Every move is a trapezoid: `Motor{n}MaximumStepInterval` sets the start/stop speed,
`Motor{n}StepInterval` the cruise speed and `Motor{n}Acceleration` (steps/s², new in 1.0)
the ramp. The old `Motor{n}StepAccelerationInterval` register still works and is converted
to an equivalent acceleration. `StepRelative` (constant-speed jog) and the Quick* registers
use the same engine. Things worth knowing:

- Stops (end of move, `StopMotors`, inputs, emergency) take effect at the next pulse, so
  the pulse count is always exact; at the slowest configured speed that is up to 16 ms.
- A move request is refused (error reply) if the pulse rates of all moving motors would
  exceed the CPU budget: each motor costs ~2.5 µs per pulse and the total must stay
  under 30% (one motor at 10 µs between pulses, two at 17 µs, four at 34 µs).
- Travel limits (`Motor{n}MaxPosition` / `MinPosition`) clamp the target, so a move
  decelerates into the limit instead of stopping abruptly.
- The engine is `StepperDriver/motion.c` (no AVR code) + `stepper_control.c` (timers,
  interrupts). The design is explained in `../Assets/trapezoidal-motion-explainer.html`.

## Tests

```sh
make test       # host-side unit tests of the motion engine, built with clang/gcc
```

Hardware test scripts live in `tests/hw/` and run with uv, which installs the `harp`
package (https://github.com/harp-tech/python) on first use:

```sh
cd tests/hw && uv run 01_identify.py
```

## Regenerating the Bonsai interface after a `device.yml` change

```sh
dotnet build ../Generators/Generators.csproj
```
