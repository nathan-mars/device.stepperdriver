# /// script
# requires-python = ">=3.10"
# dependencies = ["harp>=0.5", "pyserial"]
# ///
"""Flash the StepperDriver over USB with the Harp ATxmega bootloader protocol, or
probe what answers after a reset. A Python port of the Harp Toolkit update logic with
tighter timing and verbose diagnostics, for when the toolkit stops at 30%.

    uv run flash.py --probe                 # reset the board and report what talks back
    uv run flash.py ../../StepperDriver/build/StepperDriver-fw1.0-harp1.15-hw1.0-ass0.hex
    uv run flash.py --force <hex>           # board already in bootloader (no Harp reply)

Protocol (from Harp.Toolkit.Core/Firmware/ATxmega/Bootloader.cs): 1 Mbaud 8N1, messages
are 13-byte header + data + 16-bit sum: 01 02 03 <opcode> <err> <addr:4> <len:4> ... <sum:2>.
Opcodes: 0x66 read page size, 0x00 write page at addr, 0x77 exit bootloader. The device
echoes the message with the error byte filled in.
"""
from __future__ import annotations

import argparse
import struct
import sys
import time

import serial as pyserial

from stepper import add_port_arg

BAUD = 1_000_000
HDR = 15
OP_WRITE, OP_PAGESIZE, OP_EXIT = 0x00, 0x66, 0x77


# ---------------------------------------------------------------- Intel HEX
def load_hex(path: str, page_size: int = 512) -> bytes:
    data = bytearray()
    base = 0
    for n, line in enumerate(open(path), 1):
        line = line.strip()
        if not line:
            continue
        if line[0] != ":":
            sys.exit(f"{path}:{n}: not an Intel HEX record")
        rec = bytes.fromhex(line[1:])
        count, addr, rtype = rec[0], (rec[1] << 8) | rec[2], rec[3]
        if (sum(rec) & 0xFF) != 0:
            sys.exit(f"{path}:{n}: bad record checksum")
        payload = rec[4:4 + count]
        if rtype == 0:
            a = base + addr
            if a + count > len(data):
                data.extend(b"\xff" * (a + count - len(data)))
            data[a:a + count] = payload
        elif rtype == 1:
            break
        elif rtype == 2:
            base = struct.unpack(">H", payload)[0] * 16
        elif rtype == 4:
            base = struct.unpack(">H", payload)[0] << 16
        else:
            sys.exit(f"{path}:{n}: unsupported record type {rtype}")
    pad = (-len(data)) % page_size
    data.extend(b"\xff" * pad)
    return bytes(data)


# ---------------------------------------------------------------- bootloader frames
def frame(opcode: int, address: int, payload: bytes = b"") -> bytes:
    body = bytes([1, 2, 3, opcode, 0]) + struct.pack("<II", address, len(payload)) + payload
    s = sum(body) & 0xFFFF
    return body + struct.pack("<H", s)


def command(port: pyserial.Serial, msg: bytes, timeout_s: float = 0.5) -> bytes:
    port.reset_input_buffer()
    port.write(msg)
    port.flush()
    port.timeout = timeout_s
    reply = port.read(len(msg))
    if len(reply) != len(msg):
        raise TimeoutError(f"expected {len(msg)} bytes, got {len(reply)}: {reply.hex(' ')}")
    if reply[:4] != bytes([1, 2, 3, msg[3]]):
        raise ValueError(f"not a bootloader reply: {reply[:16].hex(' ')}")
    if (sum(reply[:-2]) & 0xFFFF) != struct.unpack("<H", reply[-2:])[0]:
        raise ValueError("bad reply checksum")
    if reply[4] != 0:
        raise ValueError(f"bootloader error code {reply[4]}")
    return reply


def read_page_size(port: pyserial.Serial, timeout_s: float = 0.5) -> int:
    reply = command(port, frame(OP_PAGESIZE, 0), timeout_s)
    size = struct.unpack("<I", reply[9:13])[0]
    if size < 64 or size > 4096 or size & (size - 1):
        raise ValueError(f"invalid page size {size}")
    return size


# ---------------------------------------------------------------- Harp side
def harp_reset(port_name: str, timeout_ms: int) -> None:
    """Ask the running application to reset (keeping the current boot mode)."""
    from harp import serial as hserial
    from harp.device import core
    with hserial.open_device(None, port=port_name) as dev:
        dev.REPLY_TIMEOUT = timeout_ms / 1000.0
        who = int(dev.read(core.WhoAmI).payload)
        fw = f"{int(dev.read(core.FirmwareVersionHigh).payload)}.{int(dev.read(core.FirmwareVersionLow).payload)}"
        flags = int(dev.read(core.ResetDevice).payload)
        print(f"application answers: WhoAmI {who}, firmware {fw}, reset flags 0x{flags:02x}")
        if flags & core.ResetFlags.BOOT_FROM_EEPROM:
            dev.write(core.ResetDevice, core.ResetFlags.RESTORE_EEPROM)
        elif flags & core.ResetFlags.BOOT_FROM_DEFAULT:
            dev.write(core.ResetDevice, core.ResetFlags.RESTORE_DEFAULT)
        else:
            sys.exit("device reports an unexpected boot mode")
    print("reset command sent")


def probe(port_name: str, seconds: float) -> None:
    """Right after a reset, poll for a bootloader reply and report whatever arrives."""
    t0 = time.perf_counter()
    port = pyserial.Serial(port_name, BAUD, timeout=0.05)
    print(f"port reopened {1e3 * (time.perf_counter() - t0):.0f} ms after reset")
    msg = frame(OP_PAGESIZE, 0)
    while time.perf_counter() - t0 < seconds:
        port.reset_input_buffer()
        port.write(msg)
        reply = port.read(64)
        t = 1e3 * (time.perf_counter() - t0)
        if reply[:4] == bytes([1, 2, 3, OP_PAGESIZE]):
            size = struct.unpack("<I", reply[9:13])[0] if len(reply) >= 13 else -1
            print(f"{t:6.0f} ms: BOOTLOADER answered, page size {size}")
            return
        if reply:
            kind = "Harp application frame" if reply[0] in (1, 2, 3, 9, 10, 11) and len(reply) > 1 and reply[1] + 2 <= len(reply) + 8 else "unknown bytes"
            print(f"{t:6.0f} ms: {kind}: {reply[:24].hex(' ')}")
        time.sleep(0.02)
    print(f"no bootloader reply within {seconds:.1f} s")
    port.close()


def flash(port_name: str, image: bytes, page_size_hint: int = 512, attempts: int = 3) -> None:
    for attempt in range(1, attempts + 1):
        try:
            with pyserial.Serial(port_name, BAUD, timeout=0.5) as port:
                time.sleep(0.05)
                page = read_page_size(port)
                print(f"bootloader page size {page}; image {len(image)} bytes")
                if len(image) % page:
                    image = image + b"\xff" * (page - len(image) % page)
                for addr in range(0, len(image), page):
                    command(port, frame(OP_WRITE, addr, image[addr:addr + page]))
                    if (addr // page) % 8 == 0 or addr + page == len(image):
                        print(f"\r  written {addr + page}/{len(image)} bytes", end="", flush=True)
                print()
                # The bootloader restarts into the application while still echoing the exit
                # frame, so the echo is usually truncated; a valid header with error 0 is enough.
                exit_msg = frame(OP_EXIT, 0, b"\xff" * page)
                port.reset_input_buffer()
                port.write(exit_msg)
                port.flush()
                port.timeout = 1.0
                echo = port.read(len(exit_msg))
                if echo[:4] != bytes([1, 2, 3, OP_EXIT]) or (len(echo) > 4 and echo[4] != 0):
                    raise ValueError(f"exit not acknowledged: {echo[:16].hex(' ')}")
                print("exit acknowledged; the application starts within a few seconds")
                return
        except (TimeoutError, ValueError, OSError) as e:
            print(f"attempt {attempt}: {e}")
            time.sleep(0.5)
    sys.exit("flashing failed")


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    add_port_arg(p)
    p.add_argument("hex", nargs="?", help="firmware image (Intel HEX)")
    p.add_argument("--probe", action="store_true", help="reset the board and report what answers, no flashing")
    p.add_argument("--force", action="store_true", help="skip the Harp reset; the board is already in the bootloader")
    p.add_argument("--window", type=float, default=5.0, help="seconds to poll for the bootloader after reset")
    p.add_argument("--timeout", type=int, default=2000, help="Harp reply timeout in ms for the reset step")
    args = p.parse_args()
    if not args.port:
        sys.exit("no serial port found; pass --port")

    if args.probe:
        harp_reset(args.port, args.timeout)
        probe(args.port, args.window)
        return

    if not args.hex:
        sys.exit("give a .hex file, or use --probe")
    image = load_hex(args.hex)
    if not args.force:
        harp_reset(args.port, args.timeout)
    # Poll quickly: the bootloader only listens for a short time after reset.
    t0 = time.perf_counter()
    while True:
        try:
            with pyserial.Serial(args.port, BAUD, timeout=0.1) as port:
                read_page_size(port, 0.1)
            break
        except (TimeoutError, ValueError, OSError):
            if time.perf_counter() - t0 > args.window:
                sys.exit("no bootloader reply; run with --probe to see what the board does after a reset")
            time.sleep(0.02)
    print(f"bootloader found {1e3 * (time.perf_counter() - t0):.0f} ms after reset")
    flash(args.port, image)


if __name__ == "__main__":
    main()
