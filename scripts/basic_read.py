"""
basic_read.py - Initial communication test for IWR6843AOPEVM Rev G.

Sends a configuration to the radar via the CLI port and reads raw
data frames from the data port to verify that everything is working.

Usage:
    python scripts/basic_read.py
    python scripts/basic_read.py --config config/default.cfg
"""

import serial
import time
import struct
import sys
import argparse

# =====================================================
# CONFIGURATION - Adjusted for RPi5 + udev symlinks
# =====================================================
CLI_PORT = "/dev/mmwave_cli"
DATA_PORT = "/dev/mmwave_data"
CLI_BAUD = 115200
DATA_BAUD = 921600

# Magic word that marks the beginning of each radar data frame
MAGIC_WORD = b'\x02\x01\x04\x03\x06\x05\x08\x07'

# Default configuration for indoor presence detection
# Based on SDK 3.6, IWR6843AOP, ~10 fps, ~5m range
DEFAULT_CONFIG = [
    "sensorStop",
    "flushCfg",
    "dfeDataOutputMode 1",
    "channelCfg 15 7 0",
    "adcCfg 2 1",
    "adcbufCfg -1 0 1 1 1",
    "profileCfg 0 60 42.95 7 57.14 0 0 70 1 256 5209 0 0 48",
    "chirpCfg 0 0 0 0 0 0 0 1",
    "chirpCfg 1 1 0 0 0 0 0 2",
    "chirpCfg 2 2 0 0 0 0 0 4",
    "frameCfg 0 2 16 0 100 1 0",
    "lowPower 0 0",
    "guiMonitor -1 1 1 0 0 0 1",
    "cfarCfg -1 0 2 8 4 3 0 15 1",
    "cfarCfg -1 1 0 4 2 3 1 15 1",
    "multiObjBeamForming -1 1 0.5",
    "clutterRemoval -1 0",
    "calibDcRangeSig -1 0 -5 8 256",
    "extendedMaxVelocity -1 0",
    "bpmCfg -1 0 0 1",
    "lvdsStreamCfg -1 0 0 0",
    "compRangeBiasAndRxChanPhase 0.0 1 0 1 0 1 0 1 0 1 0 1 0 1 0 1 0 1 0 1 0 1 0 1 0",
    "measureRangeBiasAndRxChanPhase 0 1.5 0.2",
    "CQRxSatMonitor 0 3 5 121 0",
    "CQSigImgMonitor 0 127 4",
    "analogMonitor 0 0",
    "aoaFovCfg -1 -90 90 -90 90",
    "cfarFovCfg -1 0 0 5.0",
    "cfarFovCfg -1 1 -1 1",
    "calibData 0 0 0",
    "sensorStart",
]


def load_config_from_file(filepath):
    """Load configuration commands from a .cfg file."""
    commands = []
    with open(filepath, 'r') as f:
        for line in f:
            line = line.strip()
            if line and not line.startswith('%') and not line.startswith('#'):
                commands.append(line)
    return commands


def send_config(cli_port, commands):
    """
    Send configuration commands to the radar one by one.
    Returns True if all commands were accepted, False on error.
    """
    print(f"[INFO] Sending {len(commands)} configuration commands...")

    for line in commands:
        cli_port.write((line + '\n').encode('utf-8'))
        time.sleep(0.05)
        response = cli_port.read(cli_port.in_waiting).decode('utf-8', errors='replace')

        status = "OK" if "Done" in response or "sensorStart" in line else "??"
        if "Error" in response or "error" in response:
            status = "ERROR"
            print(f"  [ERROR] {line}")
            print(f"          Radar response: {response.strip()}")
            return False

        print(f"  [{status}] {line}")

    print("[INFO] Configuration sent successfully.\n")
    return True


def read_data_frames(data_port, num_frames=20):
    """
    Read data frames from the radar and display basic info.
    Looks for the magic word header to sync with the data stream.
    """
    print(f"[INFO] Waiting for radar data ({num_frames} frames)...")
    print(f"[INFO] If nothing appears within 5-10 seconds, check your configuration.\n")
    print(f"  {'Frame':>8s} | {'Size (bytes)':>12s} | {'Objects':>8s} | {'TLVs':>5s}")
    print(f"  {'-'*8}-+-{'-'*12}-+-{'-'*8}-+-{'-'*5}")

    buffer = b''
    frames_received = 0
    start_time = time.time()
    timeout = 15  # seconds

    while frames_received < num_frames:
        # Timeout check
        if time.time() - start_time > timeout and frames_received == 0:
            print("\n[ERROR] Timeout: no data received from the radar.")
            print("        Check that the configuration was sent correctly.")
            return False

        # Read available data
        if data_port.in_waiting > 0:
            buffer += data_port.read(data_port.in_waiting)
        else:
            time.sleep(0.01)
            continue

        # Search for magic word in buffer
        while True:
            idx = buffer.find(MAGIC_WORD)
            if idx == -1:
                buffer = buffer[-len(MAGIC_WORD):]
                break

            if idx > 0:
                buffer = buffer[idx:]

            # Need at least the full header (40 bytes)
            if len(buffer) < 40:
                break

            # Parse frame header
            try:
                total_len = struct.unpack('<I', buffer[12:16])[0]
                frame_num = struct.unpack('<I', buffer[20:24])[0]
                num_objects = struct.unpack('<I', buffer[28:32])[0]
                num_tlvs = struct.unpack('<I', buffer[32:36])[0]
            except struct.error:
                buffer = buffer[8:]
                continue

            # Sanity check
            if total_len > 100000 or total_len < 40:
                buffer = buffer[8:]
                continue

            # Wait for full frame
            if len(buffer) < total_len:
                break

            frames_received += 1
            print(f"  {frame_num:>8d} | {total_len:>12d} | {num_objects:>8d} | {num_tlvs:>5d}")

            buffer = buffer[total_len:]

    elapsed = time.time() - start_time
    fps = frames_received / elapsed if elapsed > 0 else 0
    print(f"\n[OK] Received {frames_received} frames in {elapsed:.1f}s (~{fps:.1f} fps)")
    return True


def main():
    parser = argparse.ArgumentParser(description="IWR6843AOPEVM basic communication test")
    parser.add_argument(
        '--config', '-c',
        type=str,
        default=None,
        help='Path to a .cfg file. If not provided, uses built-in default config.'
    )
    parser.add_argument(
        '--frames', '-n',
        type=int,
        default=20,
        help='Number of frames to read (default: 20)'
    )
    args = parser.parse_args()

    print("=" * 60)
    print("  IWR6843AOPEVM Rev G - Basic Communication Test")
    print("=" * 60)
    print()

    # Open serial ports
    try:
        cli = serial.Serial(CLI_PORT, CLI_BAUD, timeout=1)
        print(f"[OK] CLI port opened: {CLI_PORT} @ {CLI_BAUD} baud")
    except serial.SerialException as e:
        print(f"[ERROR] Could not open CLI port ({CLI_PORT}): {e}")
        print("  Check: Is the radar connected? Are you in the dialout group?")
        sys.exit(1)

    try:
        data = serial.Serial(DATA_PORT, DATA_BAUD, timeout=1)
        print(f"[OK] Data port opened: {DATA_PORT} @ {DATA_BAUD} baud")
    except serial.SerialException as e:
        print(f"[ERROR] Could not open data port ({DATA_PORT}): {e}")
        cli.close()
        sys.exit(1)

    print()

    # Clear buffers
    cli.reset_input_buffer()
    data.reset_input_buffer()

    # Load and send configuration
    if args.config:
        print(f"[INFO] Loading configuration from: {args.config}")
        commands = load_config_from_file(args.config)
    else:
        print("[INFO] Using built-in default configuration.")
        commands = DEFAULT_CONFIG

    if not send_config(cli, commands):
        cli.close()
        data.close()
        sys.exit(1)

    # Wait for radar to start
    time.sleep(0.5)

    # Read data frames
    try:
        success = read_data_frames(data, num_frames=args.frames)
    except KeyboardInterrupt:
        print("\n[INFO] Interrupted by user.")
        success = True

    # Stop sensor and close ports
    cli.write(b'sensorStop\n')
    time.sleep(0.1)
    cli.close()
    data.close()
    print("[INFO] Sensor stopped. Ports closed. Test complete.")

    sys.exit(0 if success else 1)


if __name__ == "__main__":
    main()
