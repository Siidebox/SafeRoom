#!/usr/bin/env python3
"""
send_config.py — Send a .cfg file to the IWR6843 CLI UART port.

Usage:
    python send_config.py --port COM3 --cfg path/to/config.cfg
    python send_config.py --port /dev/ttyUSB0 --cfg SafeRoom_1p9m_4x6m.cfg

The IWR6843 exposes two serial ports:
    CLI port  (lower COM number): 115200 baud — send config commands here
    Data port (higher COM number): 921600 baud — read TLV output from here

Requirements:
    pip install pyserial
"""

import argparse
import sys
import time
import serial


CLI_BAUD = 115200
INTER_CMD_DELAY = 0.05   # seconds between commands (50ms)
READ_TIMEOUT   = 2.0     # seconds to wait for ACK per command


def send_config(port: str, cfg_path: str, verbose: bool = True) -> bool:
    """
    Open the CLI port and send each non-comment line from cfg_path.
    Returns True if all commands were acknowledged, False otherwise.
    """
    try:
        lines = _read_cfg(cfg_path)
    except FileNotFoundError:
        print(f"[ERROR] Config file not found: {cfg_path}")
        return False

    try:
        ser = serial.Serial(port, baudrate=CLI_BAUD, timeout=READ_TIMEOUT)
    except serial.SerialException as e:
        print(f"[ERROR] Cannot open port {port}: {e}")
        return False

    print(f"Opened {port} at {CLI_BAUD} baud")
    print(f"Sending {len(lines)} commands from {cfg_path}\n")

    success = True
    for cmd in lines:
        if verbose:
            print(f"  >> {cmd}", end="  ")

        ser.write((cmd + "\n").encode("utf-8"))
        time.sleep(INTER_CMD_DELAY)

        response = _read_response(ser)

        if verbose:
            # Print response on same line, strip whitespace
            resp_short = response.replace("\n", " ").replace("\r", "").strip()
            print(f"<< {resp_short}")

        if "Error" in response or "error" in response:
            print(f"[WARN] Command may have failed: {cmd!r}")
            print(f"       Response: {response!r}")
            success = False

    ser.close()
    print("\nDone." if success else "\nCompleted with warnings (see above).")
    return success


def _read_cfg(path: str) -> list[str]:
    """Return non-blank, non-comment lines from a .cfg file."""
    commands = []
    with open(path, "r", encoding="utf-8") as f:
        for line in f:
            stripped = line.strip()
            if stripped and not stripped.startswith("%"):
                commands.append(stripped)
    return commands


def _read_response(ser: serial.Serial) -> str:
    """Read all available bytes from serial until timeout."""
    response = b""
    deadline = time.time() + READ_TIMEOUT
    while time.time() < deadline:
        waiting = ser.in_waiting
        if waiting:
            response += ser.read(waiting)
            # Stop reading once we see a prompt or "Done"
            decoded = response.decode("utf-8", errors="replace")
            if "Done" in decoded or "mmwDemo:/" in decoded:
                break
        else:
            time.sleep(0.005)
    return response.decode("utf-8", errors="replace")


def main():
    parser = argparse.ArgumentParser(
        description="Send a .cfg file to the IWR6843 CLI UART port."
    )
    parser.add_argument(
        "--port", required=True,
        help="CLI serial port (e.g. COM3 on Windows, /dev/ttyUSB0 on Linux)"
    )
    parser.add_argument(
        "--cfg", required=True,
        help="Path to the .cfg file to send"
    )
    parser.add_argument(
        "--quiet", action="store_true",
        help="Suppress per-command output"
    )
    args = parser.parse_args()

    ok = send_config(args.port, args.cfg, verbose=not args.quiet)
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
