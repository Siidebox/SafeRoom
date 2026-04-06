#!/usr/bin/env python3
"""
radar_reader.py — Read and parse TLV output from IWR6843 running 3D People Tracking.

Parses:
  TLV 1020: Compressed point cloud (spherical → Cartesian)
  TLV 1010: 3D Kalman tracker target list (pos, vel, acc)
  TLV 1011: Target index (point-to-track mapping)
  TLV 1012: Target height (maxZ, minZ per track)
  TLV 1021: Presence indication

Usage:
  # Send config and read data with live plot:
  python radar_reader.py --cli COM4 --data COM3 --cfg chirp_configs/SafeRoom_1p9m_4x6m.cfg --plot

  # Read only (config already loaded):
  python radar_reader.py --cli COM4 --data COM3

  # Log to CSV:
  python radar_reader.py --cli COM4 --data COM3 --cfg ... --log session.csv

  # Raspberry Pi:
  python radar_reader.py --cli /dev/ttyUSB0 --data /dev/ttyUSB1 --cfg ...

Requirements:
  pip install pyserial numpy pyqtgraph pyqt5
"""

import argparse
import collections
import csv
import math
import struct
import sys
import time
import threading

import numpy as np
import serial

# ── TLV type constants ──────────────────────────────────────────────────────
TLV_COMPRESSED_POINT_CLOUD = 1020
TLV_TRACK_LIST             = 1010
TLV_TARGET_INDEX           = 1011
TLV_TARGET_HEIGHT          = 1012
TLV_PRESENCE               = 1021

# ── Serial settings ─────────────────────────────────────────────────────────
CLI_BAUD  = 115200
DATA_BAUD = 921600
INTER_CMD_DELAY = 0.05   # seconds between CLI commands

# ── Frame magic word ─────────────────────────────────────────────────────────
MAGIC = bytes([0x02, 0x01, 0x04, 0x03, 0x06, 0x05, 0x08, 0x07])

# ── Struct formats (all little-endian) ───────────────────────────────────────
# Frame header: uint64 magic + 8×uint32 = 40 bytes
# Fields: magic, version, totalPacketLen, platform, frameNum,
#         timeCPUCycles, numDetectedObj, numTLVs, subFrameNum
HEADER_FMT  = '<Q8I'
HEADER_SIZE = struct.calcsize(HEADER_FMT)  # 40 bytes

TLV_HDR_FMT  = '<2I'   # type, length — 8 bytes
TLV_HDR_SIZE = struct.calcsize(TLV_HDR_FMT)

PC_UNIT_FMT  = '<5f'   # elevUnit, azUnit, dopplerUnit, rangeUnit, snrUnit — 20 bytes
PC_UNIT_SIZE = struct.calcsize(PC_UNIT_FMT)

# elevation(i8), azimuth(i8), doppler(i16), range(u16), snr(u16) — 8 bytes
PC_PT_FMT  = '<2bh2H'
PC_PT_SIZE = struct.calcsize(PC_PT_FMT)

# TID(u32) + posXYZ(3f) + velXYZ(3f) + accXYZ(3f) + ec[16](16f) + g(f) + conf(f) — 112 bytes
TRACK_FMT  = '<I27f'
TRACK_SIZE = struct.calcsize(TRACK_FMT)

# TID(u32) + maxZ(f) + minZ(f) — 12 bytes
HEIGHT_FMT  = '<I2f'
HEIGHT_SIZE = struct.calcsize(HEIGHT_FMT)


# ─────────────────────────────────────────────────────────────────────────────
# TLV parsers
# ─────────────────────────────────────────────────────────────────────────────

def _spherical_to_cartesian(rng, az_rad, el_rad):
    """Convert spherical (range, azimuth, elevation) to Cartesian (x, y, z) in sensor frame."""
    x = rng * np.sin(az_rad) * np.cos(el_rad)
    y = rng * np.cos(az_rad) * np.cos(el_rad)
    z = rng * np.sin(el_rad)
    return x, y, z


def sensor_to_world(x_s, y_s, z_s, height_m, tilt_deg):
    """
    Convert sensor-frame Cartesian to world-frame Cartesian.

    The sensor is mounted at height_m above the floor and tilted tilt_deg
    degrees downward (nose-down). The rotation is around the X-axis.

    World frame: X right, Y forward (ground level), Z up (Z=0 = floor).
    Sensor frame: X right, Y along boresight, Z perpendicular upward.
    """
    t = math.radians(tilt_deg)
    x_w = x_s
    y_w = math.cos(t) * y_s + math.sin(t) * z_s
    z_w = -math.sin(t) * y_s + math.cos(t) * z_s + height_m
    return x_w, y_w, z_w


def parse_point_cloud(data):
    """
    Parse TLV 1020 — compressed spherical point cloud.
    Returns list of dicts: {x, y, z, doppler, snr, range, azimuth, elevation}
    """
    if len(data) < PC_UNIT_SIZE:
        return []

    elev_unit, az_unit, doppler_unit, range_unit, snr_unit = struct.unpack(
        PC_UNIT_FMT, data[:PC_UNIT_SIZE]
    )
    data = data[PC_UNIT_SIZE:]

    num_points = len(data) // PC_PT_SIZE
    points = []
    for _ in range(num_points):
        elev_c, az_c, dop_c, rng_c, snr_c = struct.unpack(PC_PT_FMT, data[:PC_PT_SIZE])
        data = data[PC_PT_SIZE:]

        elev = elev_c * elev_unit
        az   = az_c   * az_unit
        rng  = rng_c  * range_unit
        dop  = dop_c  * doppler_unit
        snr  = snr_c  * snr_unit

        x, y, z = _spherical_to_cartesian(rng, az, elev)
        points.append({
            'x': float(x), 'y': float(y), 'z': float(z),
            'doppler': float(dop), 'snr': float(snr),
            'range': float(rng), 'azimuth': float(az), 'elevation': float(elev)
        })
    return points


def parse_tracks(data):
    """
    Parse TLV 1010 — 3D tracker target list.
    Returns list of dicts: {tid, x, y, z, vx, vy, vz, ax, ay, az, g, confidence}
    """
    num_tracks = len(data) // TRACK_SIZE
    tracks = []
    for _ in range(num_tracks):
        fields = struct.unpack(TRACK_FMT, data[:TRACK_SIZE])
        data = data[TRACK_SIZE:]
        tracks.append({
            'tid':        int(fields[0]),
            'x':          float(fields[1]),
            'y':          float(fields[2]),
            'z':          float(fields[3]),
            'vx':         float(fields[4]),
            'vy':         float(fields[5]),
            'vz':         float(fields[6]),
            'ax':         float(fields[7]),
            'ay':         float(fields[8]),
            'az':         float(fields[9]),
            # fields[10..25] = error covariance matrix (skipped)
            'g':          float(fields[26]),
            'confidence': float(fields[27]),
        })
    return tracks


def parse_heights(data):
    """
    Parse TLV 1012 — target height.
    Returns dict: {tid: {'maxZ': float, 'minZ': float}}
    """
    num = len(data) // HEIGHT_SIZE
    heights = {}
    for _ in range(num):
        tid, max_z, min_z = struct.unpack(HEIGHT_FMT, data[:HEIGHT_SIZE])
        data = data[HEIGHT_SIZE:]
        heights[int(tid)] = {'maxZ': float(max_z), 'minZ': float(min_z)}
    return heights


def parse_indices(data):
    """
    Parse TLV 1011 — target index per detected point.
    Returns list of uint8 (255 = unassigned).
    """
    return list(data)


def parse_presence(data):
    """
    Parse TLV 1021 — presence indication.
    Returns 1 if someone is present, 0 otherwise.
    """
    if len(data) < 4:
        return 0
    return int(struct.unpack('<I', data[:4])[0])


# ─────────────────────────────────────────────────────────────────────────────
# Frame reader
# ─────────────────────────────────────────────────────────────────────────────

class FrameReader:
    """Reads and parses complete frames from the data UART port."""

    def __init__(self, data_port: serial.Serial):
        self._ser = data_port
        self._buf = bytearray()

    def _read_bytes(self, n: int, timeout: float = 2.0) -> bytes:
        deadline = time.time() + timeout
        out = bytearray()
        while len(out) < n and time.time() < deadline:
            chunk = self._ser.read(n - len(out))
            if chunk:
                out.extend(chunk)
        return bytes(out)

    def _sync(self) -> bool:
        """Scan the stream until the magic word is found. Returns True on success."""
        magic_len = len(MAGIC)
        window = bytearray(magic_len)
        # Pre-fill window
        chunk = self._read_bytes(magic_len)
        if len(chunk) < magic_len:
            return False
        window[:] = chunk

        for _ in range(4096):  # scan up to 4KB before giving up
            if bytes(window) == MAGIC:
                return True
            b = self._read_bytes(1)
            if not b:
                return False
            window = window[1:] + bytearray(b)
        return False

    def read_frame(self):
        """
        Read one complete frame from the data port.
        Returns a dict with parsed TLV data, or None on error.
        """
        if not self._sync():
            return None

        # Read rest of header (HEADER_SIZE - 8 bytes for magic already consumed)
        rest = self._read_bytes(HEADER_SIZE - 8)
        if len(rest) < HEADER_SIZE - 8:
            return None

        raw_header = MAGIC + rest
        try:
            (magic, version, total_len, platform, frame_num,
             cpu_cycles, num_det_obj, num_tlvs, sub_frame) = struct.unpack(
                HEADER_FMT, raw_header)
        except struct.error:
            return None

        # Sanity check
        if total_len > 65536 or num_tlvs > 20:
            return None

        payload_len = total_len - HEADER_SIZE
        if payload_len <= 0:
            return None

        payload = self._read_bytes(payload_len)
        if len(payload) < payload_len:
            return None

        # Parse TLVs
        frame = {
            'frameNum':    frame_num,
            'numDetected': num_det_obj,
            'points':      [],
            'tracks':      [],
            'heights':     {},
            'indices':     [],
            'presence':    0,
        }

        offset = 0
        for _ in range(num_tlvs):
            if offset + TLV_HDR_SIZE > len(payload):
                break
            tlv_type, tlv_len = struct.unpack(
                TLV_HDR_FMT, payload[offset:offset + TLV_HDR_SIZE])
            offset += TLV_HDR_SIZE

            if offset + tlv_len > len(payload):
                break

            tlv_data = payload[offset:offset + tlv_len]
            offset += tlv_len

            if tlv_type == TLV_COMPRESSED_POINT_CLOUD:
                frame['points'] = parse_point_cloud(tlv_data)
            elif tlv_type == TLV_TRACK_LIST:
                frame['tracks'] = parse_tracks(tlv_data)
            elif tlv_type == TLV_TARGET_HEIGHT:
                frame['heights'] = parse_heights(tlv_data)
            elif tlv_type == TLV_TARGET_INDEX:
                frame['indices'] = parse_indices(tlv_data)
            elif tlv_type == TLV_PRESENCE:
                frame['presence'] = parse_presence(tlv_data)

        return frame


# ─────────────────────────────────────────────────────────────────────────────
# Fall detector
# ─────────────────────────────────────────────────────────────────────────────

class FallDetector:
    """
    Three-tier fall / faint detector using maxZ and Kalman vz.

    Tier 1 — FAST fall: sustained strong vz spike (≥ 5 frames at ≤ -0.80 m/s).
             No reference required, short warmup (5 frames).
             No maxZ ceiling constraint — the vz spike occurs while the person is
             still at mid-height (1.3–1.5 m). By the time maxZ < 1.0 m, the fall
             is over and vz has already decayed to near zero.

    Tier 2 — SLOW fall: person was standing (ref_maxz ≥ MIN_STANDING_MAXZ) and
             has been at floor level (maxZ < 60% of reference) for 20 s.
             No vz requirement — catches slow collapses and missed kinetic falls.

    Tier 3 — FAINT/unconscious: person is at floor level and maxZ is stable
             (variance < FAINT_STABILITY m over FAINT_WINDOW s) for FAINT_PERSIST s.
             Uses a session-level flag so a new TID that spawns on the floor still
             triggers if the person was seen standing earlier in the session.

    update() returns (is_fall: bool, is_faint: bool).

    Calibration from log data:
      sitting vz ~ -0.55 m/s, real fall vz ~ -0.94 to -1.58 m/s → gap at -0.80
      Kalman vz reports ~-0.30 m/s even when lying still → use maxZ variance instead
    """

    REFERENCE_WINDOW   = 30.0   # s — rolling max window for standing reference
    FALL_RATIO         = 0.60   # Tier 2: maxZ must drop to <60% of reference
    ABSOLUTE_CEILING   = 1.0    # m — used for Tier 2 low-posture condition
    MIN_STANDING_MAXZ  = 1.2    # m — reference for Tier 2; session flag for Tier 3
    COOLDOWN           = 8.0    # s between fall detections per track
    MIN_FRAMES         = 20     # warm-up for Tier 2/3

    # Tier 1 — fast fall (NO reference, NO maxZ ceiling, short warmup)
    # Calibration: sitting -0.55 m/s, real fall -0.94 to -1.58 m/s → threshold -0.80
    FAST_VZ_THRESHOLD  = -0.80  # m/s
    FAST_PERSIST       = 5      # frames (~0.25 s at 20fps)
    MIN_FRAMES_FAST    = 5      # warmup for Tier 1 only (track may spawn mid-fall)

    # Tier 2 — slow / gradual fall (reference required, no vz constraint)
    SLOW_PERSIST       = 400    # frames (20 s at 20fps)

    # Tier 3 — faint / unconscious (maxZ stability, session standing flag)
    FAINT_CEILING      = 0.80   # m — person must be this low
    FAINT_STABILITY    = 0.10   # m — max allowed maxZ std-dev over stability window
    FAINT_WINDOW       = 5.0    # s — stability measurement window
    FAINT_PERSIST      = 600    # frames (30 s at 20fps)
    FAINT_COOLDOWN     = 60.0   # s between faint detections per track

    def __init__(self, frame_period_s: float = 0.05):
        self._maxz_hist      = {}   # tid → deque of (maxZ, timestamp) — 30s window
        self._maxz_stab      = {}   # tid → deque of maxZ values — FAINT_WINDOW
        self._last_det       = {}   # tid → timestamp of last fall detection
        self._last_faint     = {}   # tid → timestamp of last faint detection
        self._frame_cnt      = {}   # tid → warm-up frames
        self._fast_count     = {}   # tid → consecutive Tier 1 frames
        self._slow_count     = {}   # tid → consecutive Tier 2 frames
        self._still_count    = {}   # tid → consecutive Tier 3 frames
        self._last_peak_vz   = {}   # tid → peak_vz of last fall (for logging)
        self._session_stood  = False  # True once any TID's ref_maxz ≥ MIN_STANDING_MAXZ

    def _init_tid(self, tid: int):
        if tid not in self._maxz_hist:
            self._maxz_hist[tid]   = collections.deque()
            self._maxz_stab[tid]   = collections.deque()
            self._last_det[tid]    = 0.0
            self._last_faint[tid]  = 0.0
            self._frame_cnt[tid]   = 0
            self._fast_count[tid]  = 0
            self._slow_count[tid]  = 0
            self._still_count[tid] = 0
            self._last_peak_vz[tid] = 0.0

    def update(self, tid: int, height: dict, track: dict):
        """
        Call once per frame per track when TLV 1012 data is available.
        height: {'maxZ': float, 'minZ': float}
        track:  {'vz': float, ...}
        Returns (is_fall, is_faint).
        """
        now   = time.time()
        max_z = height.get('maxZ', 0.0)
        vz    = track.get('vz', 0.0)

        self._init_tid(tid)
        self._frame_cnt[tid] += 1

        # ── Rolling reference window (30s max) ───────────────────────────
        buf = self._maxz_hist[tid]
        buf.append((max_z, now))
        cutoff = now - self.REFERENCE_WINDOW
        while buf and buf[0][1] < cutoff:
            buf.popleft()

        ref_maxz = max(z for z, _ in buf) if buf else 0.0

        # Update session-level standing flag
        if ref_maxz >= self.MIN_STANDING_MAXZ:
            self._session_stood = True

        # ── Stability window (FAINT_WINDOW s) ────────────────────────────
        stab = self._maxz_stab[tid]
        stab.append(max_z)
        max_stab_len = int(self.FAINT_WINDOW / 0.05)  # assume 20fps
        while len(stab) > max_stab_len:
            stab.popleft()

        # ── Tier 1 — fast fall (no reference, short warmup) ──────────────
        # vz spike is the primary signal; maxZ may still be > 1.0 m during fall
        if self._frame_cnt[tid] >= self.MIN_FRAMES_FAST:
            if vz <= self.FAST_VZ_THRESHOLD:
                self._fast_count[tid] += 1
            else:
                self._fast_count[tid] = 0

        if self._frame_cnt[tid] < self.MIN_FRAMES:
            return False, False

        # ── Tier 2 — slow / gradual fall (reference required) ────────────
        slow_cond = (
            ref_maxz >= self.MIN_STANDING_MAXZ
            and max_z < self.FALL_RATIO * ref_maxz
            and max_z < self.ABSOLUTE_CEILING
        )
        if slow_cond:
            self._slow_count[tid] += 1
        else:
            self._slow_count[tid] = 0

        # ── Tier 3 — faint / unconscious ─────────────────────────────────
        # Use maxZ variance over stability window, not Kalman vz (which drifts)
        if len(stab) >= max_stab_len:
            vals = list(stab)
            mean = sum(vals) / len(vals)
            std  = (sum((v - mean) ** 2 for v in vals) / len(vals)) ** 0.5
            still_cond = (
                self._session_stood
                and max_z < self.FAINT_CEILING
                and std < self.FAINT_STABILITY
            )
        else:
            still_cond = False

        if still_cond:
            self._still_count[tid] += 1
        else:
            self._still_count[tid] = 0

        # ── Detection ────────────────────────────────────────────────────
        fall_cooldown = (now - self._last_det[tid]) < self.COOLDOWN

        fast_fall = (
            not fall_cooldown
            and self._fast_count[tid] >= self.FAST_PERSIST
        )
        slow_fall = (
            not fall_cooldown
            and self._slow_count[tid] >= self.SLOW_PERSIST
        )
        fall = fast_fall or slow_fall

        faint = (
            not fall_cooldown
            and (now - self._last_faint[tid]) >= self.FAINT_COOLDOWN
            and self._still_count[tid] >= self.FAINT_PERSIST
        )

        if fall:
            self._last_peak_vz[tid] = vz
            self._last_det[tid]     = now
            self._fast_count[tid]   = 0
            self._slow_count[tid]   = 0

        if faint:
            self._last_faint[tid]  = now
            self._still_count[tid] = 0

        return fall, faint

    def bump_frame(self, tid: int):
        """Advance warm-up counter without updating maxZ history (no height data this frame)."""
        self._init_tid(tid)
        self._frame_cnt[tid] += 1

    def cleanup_old_tracks(self, active_tids: set):
        for tid in list(self._maxz_hist.keys()):
            if tid not in active_tids:
                del self._maxz_hist[tid]
                del self._last_det[tid]
                del self._last_faint[tid]
                del self._frame_cnt[tid]
                del self._fast_count[tid]
                del self._slow_count[tid]
                del self._maxz_stab[tid]
                del self._still_count[tid]
                del self._last_peak_vz[tid]

    def ref_maxz(self, tid: int) -> float:
        """Return current rolling reference maxZ for a track (for logging/debug)."""
        buf = self._maxz_hist.get(tid)
        return max((z for z, _ in buf), default=0.0) if buf else 0.0


# ─────────────────────────────────────────────────────────────────────────────
# Config sender
# ─────────────────────────────────────────────────────────────────────────────

def send_config(cli_port: str, cfg_path: str) -> bool:
    """Send a .cfg file to the sensor CLI port."""
    try:
        with open(cfg_path, 'r', encoding='utf-8') as f:
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


# ─────────────────────────────────────────────────────────────────────────────
# CSV logger
# ─────────────────────────────────────────────────────────────────────────────

class CsvLogger:
    def __init__(self, path: str):
        self._f = open(path, 'w', newline='')
        self._w = csv.writer(self._f)
        self._w.writerow([
            'timestamp', 'frameNum', 'presence',
            'tid', 'x', 'y', 'z', 'vx', 'vy', 'vz',
            'height_m', 'maxZ', 'minZ', 'maxZ_ref', 'peak_vz',
            'fall_detected', 'faint_detected'
        ])
        print(f'Logging to {path}')

    def log(self, frame: dict, fall_tids: set, faint_tids: set = None, fall_detector=None):
        ts = time.time()
        presence = frame['presence']
        fn = frame['frameNum']
        if not frame['tracks']:
            self._w.writerow([ts, fn, presence,
                              '', '', '', '', '', '', '',
                              '', '', '', '', ''])
        for t in frame['tracks']:
            tid = t['tid']
            h = frame['heights'].get(tid, {})
            max_z = h.get('maxZ', float('nan'))
            min_z = h.get('minZ', float('nan'))
            height_m = (max_z - min_z) if (not math.isnan(max_z) and not math.isnan(min_z)) else float('nan')
            ref      = round(fall_detector.ref_maxz(tid), 3)          if fall_detector else ''
            peak_vz  = round(fall_detector._last_peak_vz.get(tid, 0.0), 3) if fall_detector else ''
            self._w.writerow([
                ts, fn, presence,
                tid, round(t['x'], 3), round(t['y'], 3), round(t['z'], 3),
                round(t['vx'], 3), round(t['vy'], 3), round(t['vz'], 3),
                round(height_m, 3) if not math.isnan(height_m) else '',
                round(max_z, 3) if not math.isnan(max_z) else '',
                round(min_z, 3) if not math.isnan(min_z) else '',
                ref, peak_vz,
                1 if tid in fall_tids else 0,
                1 if (faint_tids and tid in faint_tids) else 0
            ])
        self._f.flush()

    def close(self):
        self._f.close()


# ─────────────────────────────────────────────────────────────────────────────
# PyQtGraph visualizer
# ─────────────────────────────────────────────────────────────────────────────

def _snr_to_rgba(snr_array: np.ndarray) -> np.ndarray:
    """Map SNR values (0–20) to RGBA: red=low, yellow=mid, green=high."""
    t = np.clip(snr_array / 20.0, 0.0, 1.0)
    r = np.where(t < 0.5, 255, (255 * (1.0 - t) * 2).clip(0, 255)).astype(np.uint8)
    g = np.where(t < 0.5, (255 * t * 2).clip(0, 255), 255).astype(np.uint8)
    b = np.zeros(len(t), dtype=np.uint8)
    a = np.full(len(t), 180, dtype=np.uint8)
    return np.column_stack([r, g, b, a])


class RadarWindow:
    """
    PyQtGraph-based real-time visualizer.

    Layout:
      [ Top view X-Y ] [ Side view X-Z ] [ Fall counter ]
      [ Status bar (spanning all columns) ]

    Driven by a QTimer that polls frame_queue every 30 ms (~33 fps redraw cap).
    The actual frame rate is limited by the radar (100 ms / 10 fps).
    """

    def __init__(self, frame_queue, stop_event):
        import pyqtgraph as pg
        from pyqtgraph.Qt import QtCore, QtWidgets

        self._queue = frame_queue
        self._stop  = stop_event
        self._total_falls  = 0
        self._total_faints = 0
        self._track_items_xy: list = []
        self._track_items_xz: list = []

        pg.setConfigOptions(antialias=True, background='w', foreground='k')

        self._app = pg.mkQApp('SafeRoom Radar')

        # ── Main window ──────────────────────────────────────────────────────
        self._win = QtWidgets.QWidget()
        self._win.setWindowTitle('SafeRoom Radar — IWR6843 3D People Tracking')
        self._win.resize(1200, 520)

        main_layout = QtWidgets.QHBoxLayout(self._win)
        main_layout.setContentsMargins(4, 4, 4, 4)
        main_layout.setSpacing(6)

        # ── Plots (left side) ─────────────────────────────────────────────
        self._glw = pg.GraphicsLayoutWidget()
        main_layout.addWidget(self._glw, stretch=10)

        # Top view X-Y
        self._plot_xy = self._glw.addPlot(row=0, col=0, title='Top view (X-Y)')
        self._plot_xy.setLabel('bottom', 'X (m)')
        self._plot_xy.setLabel('left',   'Y (m)')
        self._plot_xy.setXRange(-2, 2, padding=0)
        self._plot_xy.setYRange(0, 5, padding=0)
        self._plot_xy.setAspectLocked(True)
        self._plot_xy.showGrid(x=True, y=True, alpha=0.3)
        self._scat_xy = pg.ScatterPlotItem(size=7, pen=None)
        self._plot_xy.addItem(self._scat_xy)

        # Side view X-Z
        self._plot_xz = self._glw.addPlot(row=0, col=1, title='Side view (X-Z)')
        self._plot_xz.setLabel('bottom', 'X (m)')
        self._plot_xz.setLabel('left',   'Z (m)')
        self._plot_xz.setXRange(-2, 2, padding=0)
        self._plot_xz.setYRange(0, 2.5, padding=0)
        self._plot_xz.setAspectLocked(True)
        self._plot_xz.showGrid(x=True, y=True, alpha=0.3)
        self._scat_xz = pg.ScatterPlotItem(size=7, pen=None)
        self._plot_xz.addItem(self._scat_xz)

        # Status bar below both plots
        self._status = self._glw.addLabel(
            '', row=1, col=0, colspan=2,
            color=(80, 80, 80), size='9pt'
        )

        # ── Fall counter panel (right side) ──────────────────────────────
        cnt_widget = QtWidgets.QWidget()
        cnt_widget.setFixedWidth(120)
        cnt_widget.setStyleSheet('background-color: #fff0f0; border-radius: 6px;')
        cnt_layout = QtWidgets.QVBoxLayout(cnt_widget)
        cnt_layout.setAlignment(QtCore.Qt.AlignCenter)

        lbl_title = QtWidgets.QLabel('CAÍDAS\ndetectadas')
        lbl_title.setAlignment(QtCore.Qt.AlignCenter)
        lbl_title.setStyleSheet('color: darkred; font-weight: bold; font-size: 11px;')

        self._lbl_count = QtWidgets.QLabel('0')
        self._lbl_count.setAlignment(QtCore.Qt.AlignCenter)
        self._lbl_count.setStyleSheet(
            'color: red; font-size: 52px; font-weight: bold;')

        self._lbl_last = QtWidgets.QLabel('')
        self._lbl_last.setAlignment(QtCore.Qt.AlignCenter)
        self._lbl_last.setStyleSheet('color: gray; font-size: 9px;')

        lbl_faint_title = QtWidgets.QLabel('DESMAYOS\ndetectados')
        lbl_faint_title.setAlignment(QtCore.Qt.AlignCenter)
        lbl_faint_title.setStyleSheet(
            'color: darkorange; font-weight: bold; font-size: 11px; margin-top: 8px;')

        self._lbl_faint_count = QtWidgets.QLabel('0')
        self._lbl_faint_count.setAlignment(QtCore.Qt.AlignCenter)
        self._lbl_faint_count.setStyleSheet(
            'color: orange; font-size: 36px; font-weight: bold;')

        cnt_layout.addStretch()
        cnt_layout.addWidget(lbl_title)
        cnt_layout.addWidget(self._lbl_count)
        cnt_layout.addWidget(self._lbl_last)
        cnt_layout.addWidget(lbl_faint_title)
        cnt_layout.addWidget(self._lbl_faint_count)
        cnt_layout.addStretch()

        main_layout.addWidget(cnt_widget, stretch=1)

        # ── Timer ────────────────────────────────────────────────────────
        self._timer = QtCore.QTimer()
        self._timer.timeout.connect(self._update)
        self._timer.start(30)   # poll every 30 ms

    def _update(self):
        import pyqtgraph as pg
        import datetime

        if self._stop.is_set():
            self._timer.stop()
            self._app.quit()
            return

        if self._queue.empty():
            return

        frame, fall_tids, faint_tids = self._queue.get_nowait()
        pts     = frame['points']
        tracks  = frame['tracks']
        heights = frame['heights']
        presence = frame['presence']

        # ── Point cloud ───────────────────────────────────────────────────
        if pts:
            xs   = np.array([p['x']   for p in pts], dtype=np.float32)
            ys   = np.array([p['y']   for p in pts], dtype=np.float32)
            zs   = np.array([p['z']   for p in pts], dtype=np.float32)
            snrs = np.array([p['snr'] for p in pts], dtype=np.float32)
            colors = _snr_to_rgba(snrs)
            self._scat_xy.setData(x=xs, y=ys, brush=[pg.mkBrush(*c) for c in colors])
            self._scat_xz.setData(x=xs, y=zs, brush=[pg.mkBrush(*c) for c in colors])
        else:
            self._scat_xy.setData(x=[], y=[])
            self._scat_xz.setData(x=[], y=[])

        # ── Remove old track labels ───────────────────────────────────────
        for item in self._track_items_xy:
            self._plot_xy.removeItem(item)
        for item in self._track_items_xz:
            self._plot_xz.removeItem(item)
        self._track_items_xy.clear()
        self._track_items_xz.clear()

        # ── Draw tracks ───────────────────────────────────────────────────
        for t in tracks:
            tid = t['tid']
            is_fall  = tid in fall_tids
            is_faint = tid in faint_tids
            color = (220, 30, 30) if is_fall else (220, 140, 0) if is_faint else (30, 90, 200)

            h = heights.get(tid, {})
            cluster_h = h.get('maxZ', 0.0) - h.get('minZ', 0.0) if h else float('nan')
            label = f"T{tid}"
            if not math.isnan(cluster_h):
                label += f"\nh={cluster_h:.2f}m"
            if is_fall:
                label += "\n⚠ FALL"
            elif is_faint:
                label += "\n⚠ FAINT"

            txt_xy = pg.TextItem(label, color=color, anchor=(0.5, 1.0))
            txt_xy.setPos(t['x'], t['y'])
            self._plot_xy.addItem(txt_xy)
            self._track_items_xy.append(txt_xy)

            # t['z'] from TLV 1010 is at ground level (tracker projection).
            # Place label at maxZ (top of cluster) so it sits on the person.
            z_label = h.get('maxZ', t['z']) if h else t['z']
            txt_xz = pg.TextItem(label, color=color, anchor=(0.5, 1.0))
            txt_xz.setPos(t['x'], z_label)
            self._plot_xz.addItem(txt_xz)
            self._track_items_xz.append(txt_xz)

        # ── Fall / faint counters ─────────────────────────────────────────
        if fall_tids:
            self._total_falls += len(fall_tids)
            self._lbl_count.setText(str(self._total_falls))
            now_str = datetime.datetime.now().strftime('%H:%M:%S')
            self._lbl_last.setText(f'última\n{now_str}')
        if faint_tids:
            self._total_faints += len(faint_tids)
            self._lbl_faint_count.setText(str(self._total_faints))

        # ── Status bar ────────────────────────────────────────────────────
        pres_str = '● PRESENT' if presence else '○ empty'
        if fall_tids:
            alert_str = '  ⚠ FALL DETECTED'
            alert_color = '#cc0000'
        elif faint_tids:
            alert_str = '  ⚠ FAINT DETECTED'
            alert_color = '#cc7700'
        else:
            alert_str = ''
            alert_color = '#505050'
        self._status.setText(
            f"Frame {frame['frameNum']}  |  {len(pts)} pts  |  "
            f"{len(tracks)} tracks  |  {pres_str}{alert_str}",
            color=alert_color,
            size='9pt',
        )

    def show(self):
        self._win.show()

    def exec(self):
        self._app.exec()


# ─────────────────────────────────────────────────────────────────────────────
# Frame reading loop (runs in main thread or background thread)
# ─────────────────────────────────────────────────────────────────────────────

def _read_loop(reader, fall_detector, logger, frame_queue, stop_event,
               sensor_h, sensor_t, use_plot):
    frame_count  = 0
    fall_count   = 0
    faint_count  = 0
    print('\n--- Reading frames. Press Ctrl+C to stop. ---\n')
    try:
        while not stop_event.is_set():
            frame = reader.read_frame()
            if frame is None:
                print('[WARN] Failed to read frame — retrying...')
                time.sleep(0.1)
                continue

            # Transform point cloud from sensor frame to world frame (Z=0 = floor).
            for p in frame['points']:
                p['x'], p['y'], p['z'] = sensor_to_world(
                    p['x'], p['y'], p['z'], sensor_h, sensor_t)

            frame_count += 1
            tracks  = frame['tracks']
            heights = frame['heights']

            # Fall / faint detection
            fall_tids  = set()
            faint_tids = set()
            active_tids = {t['tid'] for t in tracks}
            for t in tracks:
                tid = t['tid']
                h = heights.get(tid)  # None if no TLV 1012 data this frame
                if h is None:
                    # No height data — skip to avoid false falls from h=0 fallback.
                    fall_detector.bump_frame(tid)
                    continue
                is_fall, is_faint = fall_detector.update(tid, h, t)
                if is_fall:
                    fall_tids.add(tid)
                    fall_count += 1
                    print(f'\n*** FALL DETECTED — Track {tid} at '
                          f'({t["x"]:.2f}, {t["y"]:.2f}, {t["z"]:.2f}) m ***\n')
                if is_faint:
                    faint_tids.add(tid)
                    faint_count += 1
                    print(f'\n*** FAINT DETECTED — Track {tid} at '
                          f'({t["x"]:.2f}, {t["y"]:.2f}, {t["z"]:.2f}) m ***\n')
            fall_detector.cleanup_old_tracks(active_tids)

            # Console
            pres = '● PRESENT' if frame['presence'] else '○ empty  '
            pts_n = len(frame['points'])
            trk_strs = []
            for t in tracks:
                tid = t['tid']
                h = heights.get(tid) or {}
                max_z = h.get('maxZ', float('nan'))
                ref   = fall_detector.ref_maxz(tid)
                flag  = ' ⚠FALL' if tid in fall_tids else (' ⚠FAINT' if tid in faint_tids else '')
                trk_strs.append(
                    f"T{tid}({t['x']:+.2f},{t['y']:+.2f}m "
                    f"maxZ={max_z:.2f} ref={ref:.2f} vz={t['vz']:+.2f}){flag}"
                )
            print(f"[{frame['frameNum']:06d}] {pres}  {pts_n:3d}pts  "
                  + (', '.join(trk_strs) if trk_strs else 'no tracks'))

            logger.log(frame, fall_tids, faint_tids, fall_detector)

            if use_plot and not frame_queue.full():
                frame_queue.put_nowait((frame, fall_tids, faint_tids))

    except KeyboardInterrupt:
        pass
    finally:
        stop_event.set()
        print(f'\nStopped. Frames: {frame_count}, Falls: {fall_count}, Faints: {faint_count}')


# ─────────────────────────────────────────────────────────────────────────────
# Main
# ─────────────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description='Read and parse TLV output from IWR6843 3D People Tracking.'
    )
    parser.add_argument('--cli',  required=True, help='CLI serial port  (e.g. COM4 or /dev/ttyUSB0)')
    parser.add_argument('--data', required=True, help='Data serial port (e.g. COM3 or /dev/ttyUSB1)')
    parser.add_argument('--cfg',  default=None,  help='Path to .cfg file to send (optional)')
    parser.add_argument('--plot', action='store_true', help='Show live PyQtGraph visualization')
    parser.add_argument('--log',  default=None,
                        help='Path to CSV log file. If omitted, auto-saves to logs/session_YYYYMMDD_HHMMSS.csv')
    parser.add_argument('--frame-period', type=float, default=0.05,
                        help='Expected frame period in seconds (default: 0.05 = 50ms = 20fps)')
    parser.add_argument('--sensor-height', type=float, default=1.90,
                        help='Sensor height above floor in meters (default: 1.90)')
    parser.add_argument('--sensor-tilt', type=float, default=15.0,
                        help='Sensor downward tilt in degrees (default: 15.0)')
    args = parser.parse_args()

    # Auto-generate log path if not provided
    if args.log is None:
        import os, datetime
        os.makedirs('logs', exist_ok=True)
        ts = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
        args.log = f'logs/session_{ts}.csv'
        print(f'Auto-logging to {args.log}')

    # Send config if provided
    if args.cfg:
        ok = send_config(args.cli, args.cfg)
        if not ok:
            print('[WARN] Config may have had errors — continuing anyway.')
        time.sleep(2.0)

    # Open data port
    try:
        data_ser = serial.Serial(args.data, baudrate=DATA_BAUD, timeout=2.0)
    except serial.SerialException as e:
        print(f'[ERROR] Cannot open data port {args.data}: {e}')
        sys.exit(1)
    print(f'Data port {args.data} opened at {DATA_BAUD} baud.')

    reader        = FrameReader(data_ser)
    fall_detector = FallDetector(frame_period_s=args.frame_period)
    logger        = CsvLogger(args.log)

    import queue
    frame_queue = queue.Queue(maxsize=4)
    stop_event  = threading.Event()

    if args.plot:
        # Qt event loop must run in the main thread.
        # Frame reader runs in a daemon background thread.
        window = RadarWindow(frame_queue, stop_event)
        window.show()

        reader_thread = threading.Thread(
            target=_read_loop,
            args=(reader, fall_detector, logger, frame_queue, stop_event,
                  args.sensor_height, args.sensor_tilt, True),
            daemon=True,
        )
        reader_thread.start()

        try:
            window.exec()   # blocks until window is closed
        finally:
            stop_event.set()
            data_ser.close()
            logger.close()
    else:
        try:
            _read_loop(reader, fall_detector, logger, frame_queue, stop_event,
                       args.sensor_height, args.sensor_tilt, False)
        finally:
            data_ser.close()
            logger.close()


if __name__ == '__main__':
    main()
