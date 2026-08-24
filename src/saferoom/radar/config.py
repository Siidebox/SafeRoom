"""Sensor configuration over the CLI UART.

Reads a ``.cfg`` file from ``chirp_configs/`` and replays it line by line to
the radar's CLI port, which is how all runtime parameters (RF profile,
detection thresholds, tracker gains, scene boundaries) are set.
"""

import time

import serial

from saferoom.radar.tlv import CLI_BAUD, INTER_CMD_DELAY


def parse_boundary_box(cfg_path: str):
    """Return (xmin,xmax, ymin,ymax, zmin,zmax) from the first boundaryBox line in cfg.
    Returns None if cfg_path is None or the line is not found."""
    if cfg_path is None:
        return None
    try:
        with open(cfg_path, encoding='utf-8') as f:
            for line in f:
                line = line.strip()
                if line.startswith('boundaryBox'):
                    parts = line.split()
                    # boundaryBox <subframe> xmin xmax ymin ymax zmin zmax
                    # or without subframe: boundaryBox xmin xmax ymin ymax zmin zmax
                    nums = [float(p) for p in parts[1:] if p.lstrip('-').replace('.','').isdigit() or
                            (p.startswith('-') and p[1:].replace('.','').isdigit())]
                    # Skip subframe index (-1 or 0) if present
                    if len(nums) == 7:
                        nums = nums[1:]   # drop subframe
                    if len(nums) == 6:
                        return tuple(nums)  # xmin,xmax,ymin,ymax,zmin,zmax
    except (FileNotFoundError, ValueError):
        pass
    return None


def send_config(cli_port: str, cfg_path: str) -> bool:
    """Send a .cfg file to the sensor CLI port."""
    try:
        with open(cfg_path, encoding='utf-8') as f:
            lines = [l.strip() for l in f if l.strip() and not l.strip().startswith('%')]
    except FileNotFoundError:
        print(f'[ERROR] Config file not found: {cfg_path}')
        return False

    try:
        ser = serial.Serial(cli_port, baudrate=CLI_BAUD, timeout=2.0)
    except serial.SerialException as e:
        print(f'[ERROR] Cannot open CLI port {cli_port}: {e}')
        return False

    print(f'Sending {len(lines)} commands to {cli_port}...')
    ok = True
    for cmd in lines:
        ser.write((cmd + '\n').encode('utf-8'))
        time.sleep(INTER_CMD_DELAY)
        resp = b''
        deadline = time.time() + 2.0
        while time.time() < deadline:
            if ser.in_waiting:
                resp += ser.read(ser.in_waiting)
                if b'Done' in resp or b'mmwDemo:/' in resp:
                    break
            else:
                time.sleep(0.005)
        resp_str = resp.decode('utf-8', errors='replace').replace('\n', ' ').strip()
        print(f'  >> {cmd:<50}  << {resp_str[:80]}')
        if 'error' in resp_str.lower():
            print(f'     [WARN] possible error on command: {cmd!r}')
            ok = False

    ser.close()
    print('Config sent.\n')
    return ok


def main():
    """``saferoom-config`` — send a .cfg file to the radar CLI UART."""
    import argparse
    import sys

    parser = argparse.ArgumentParser(
        description='Send a .cfg file to the IWR6843 CLI UART port.')
    parser.add_argument('--port', required=True,
                        help='CLI serial port (e.g. COM4, /dev/ttyUSB0)')
    parser.add_argument('--cfg', required=True,
                        help='Path to the .cfg file to send')
    args = parser.parse_args()

    sys.exit(0 if send_config(args.port, args.cfg) else 1)


if __name__ == '__main__':
    main()
