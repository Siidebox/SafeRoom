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
  pip install PyOpenGL          # only needed for --plot3d
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

# ── Optional ML components ───────────────────────────────────────────────────
try:
    from ml_logger import MlCsvLogger, compute_pc_stats
    _ML_LOGGER_AVAILABLE = True
except ImportError:
    _ML_LOGGER_AVAILABLE = False

try:
    from ml_inference import MlFallDetector
    _ML_INFERENCE_AVAILABLE = True
except ImportError:
    _ML_INFERENCE_AVAILABLE = False

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
    Two-tier fall / faint detector using maxZ and Kalman vz.

    Tier 1 — FAST fall: sustained vz spike below threshold for ≥ FAST_PERSIST frames.
             No reference required, short warmup. Catches kinetic falls.
             Calibrated: sitting -0.83 m/s, crouching -1.05 m/s, real fall -1.28+ m/s.

    Tier 2 — FAINT/unconscious: person at floor level (maxZ < 0.80 m) with stable
             height (std < 0.10 m) for 30 s. Requires session-level standing flag.
             Catches unconscious/immobile persons after any fall, including missed Tier-1.

    update() returns (is_fall: bool, is_faint: bool).
    """

    MIN_STANDING_MAXZ  = 1.2    # m — session standing flag threshold for Tier 2
    COOLDOWN           = 25.0   # s between fall detections per track
    MIN_FRAMES         = 20     # warm-up for Tier 2

    # Tier 1 — FAST fall
    FAST_VZ_THRESHOLD  = -1.15  # m/s — gap: crouching max -1.05, real fall -1.28+
    FAST_PERSIST       = 3      # consecutive frames required
    MIN_FRAMES_FAST    = 5      # shorter warmup (track may spawn mid-fall)

    # Tier 2 — FAINT/unconscious
    FAINT_CEILING      = 0.80   # m
    FAINT_STABILITY    = 0.10   # m — max maxZ std-dev over stability window
    FAINT_WINDOW       = 5.0    # s — stability measurement window
    FAINT_PERSIST      = 600    # frames (30 s at 20 fps)
    FAINT_COOLDOWN     = 60.0   # s between faint detections per track

    def __init__(self, frame_period_s: float = 0.05):
        self._maxz_hist    = {}   # tid → deque of (maxZ, timestamp)
        self._maxz_stab    = {}   # tid → deque of maxZ values — FAINT_WINDOW
        self._last_det     = {}   # tid → timestamp of last fall detection
        self._last_faint   = {}   # tid → timestamp of last faint detection
        self._frame_cnt    = {}   # tid → warm-up frames
        self._fast_count   = {}   # tid → consecutive Tier-1 frames
        self._still_count  = {}   # tid → consecutive Tier-2 frames
        self._last_peak_vz = {}   # tid → peak_vz of last fall (for logging)
        self._session_stood = False  # True once any TID's maxZ ≥ MIN_STANDING_MAXZ

    def _init_tid(self, tid: int):
        if tid not in self._maxz_hist:
            self._maxz_hist[tid]    = collections.deque()
            self._maxz_stab[tid]    = collections.deque()
            self._last_det[tid]     = 0.0
            self._last_faint[tid]   = 0.0
            self._frame_cnt[tid]    = 0
            self._fast_count[tid]   = 0
            self._still_count[tid]  = 0
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

        # Rolling maxZ history (for session standing flag)
        buf = self._maxz_hist[tid]
        buf.append((max_z, now))
        cutoff = now - 30.0
        while buf and buf[0][1] < cutoff:
            buf.popleft()

        if max_z >= self.MIN_STANDING_MAXZ:
            self._session_stood = True

        # Stability window for Tier-2
        stab = self._maxz_stab[tid]
        stab.append(max_z)
        max_stab_len = int(self.FAINT_WINDOW / 0.05)
        while len(stab) > max_stab_len:
            stab.popleft()

        # ── Tier 1 — FAST fall ───────────────────────────────────────────
        if self._frame_cnt[tid] >= self.MIN_FRAMES_FAST:
            if vz <= self.FAST_VZ_THRESHOLD:
                self._fast_count[tid] += 1
            else:
                self._fast_count[tid] = 0

        if self._frame_cnt[tid] < self.MIN_FRAMES:
            return False, False

        # ── Tier 2 — FAINT/unconscious ───────────────────────────────────
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

        fall = (
            not fall_cooldown
            and self._fast_count[tid] >= self.FAST_PERSIST
        )
        faint = (
            not fall_cooldown
            and (now - self._last_faint[tid]) >= self.FAINT_COOLDOWN
            and self._still_count[tid] >= self.FAINT_PERSIST
        )

        if fall:
            self._last_peak_vz[tid]  = vz
            self._last_det[tid]      = now
            self._fast_count[tid]    = 0

        if faint:
            self._last_faint[tid]   = now
            self._still_count[tid]  = 0

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

def parse_boundary_box(cfg_path: str):
    """Return (xmin,xmax, ymin,ymax, zmin,zmax) from the first boundaryBox line in cfg.
    Returns None if cfg_path is None or the line is not found."""
    if cfg_path is None:
        return None
    try:
        with open(cfg_path, 'r', encoding='utf-8') as f:
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

def _person_box_edges(cx: float, cy: float, minZ: float, maxZ: float,
                      hw: float = 0.2) -> np.ndarray:
    """
    Return (24, 3) array of line-segment endpoints forming a wireframe box
    around a person centred at (cx, cy) with half-width hw, from minZ to maxZ.
    Used for GLLinePlotItem with mode='lines' (each pair = one segment).
    """
    x0, x1 = cx - hw, cx + hw
    y0, y1 = cy - hw, cy + hw
    z0, z1 = minZ, maxZ
    return np.array([
        # bottom rectangle
        [x0, y0, z0], [x1, y0, z0],  [x1, y0, z0], [x1, y1, z0],
        [x1, y1, z0], [x0, y1, z0],  [x0, y1, z0], [x0, y0, z0],
        # top rectangle
        [x0, y0, z1], [x1, y0, z1],  [x1, y0, z1], [x1, y1, z1],
        [x1, y1, z1], [x0, y1, z1],  [x0, y1, z1], [x0, y0, z1],
        # vertical edges
        [x0, y0, z0], [x0, y0, z1],  [x1, y0, z0], [x1, y0, z1],
        [x1, y1, z0], [x1, y1, z1],  [x0, y1, z0], [x0, y1, z1],
    ], dtype=np.float32)


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

    TRAIL_LEN = 50   # positions to keep per track

    def __init__(self, frame_queue, stop_event, boundary_box=None, plot3d=False):
        import pyqtgraph as pg
        from pyqtgraph.Qt import QtCore, QtWidgets

        self._queue = frame_queue
        self._stop  = stop_event
        self._total_falls  = 0
        self._total_faints = 0
        # M1 — track trails and persistent scene items (reused each frame)
        self._trails: dict = {}        # tid -> deque of (x, y, z)
        self._trail_curves: dict = {}  # tid -> (PlotCurveItem_xy, PlotCurveItem_xz)
        self._track_labels: dict = {}  # tid -> (TextItem_xy, TextItem_xz)
        # 3D GL items (None if --plot3d not set)
        self._gl_view  = None
        self._gl_scat  = None
        self._gl_trails: dict = {}     # tid -> GLLinePlotItem (trail)
        self._gl_boxes:  dict = {}     # tid -> GLLinePlotItem (person wireframe)
        # M3 — flash alert
        self._flash_until = 0.0      # epoch time when flash expires
        self._flash_color = ''

        pg.setConfigOptions(antialias=True, background='w', foreground='k')

        self._app = pg.mkQApp('SafeRoom Radar')

        # ── Main window ──────────────────────────────────────────────────────
        self._win = QtWidgets.QWidget()
        self._win.setWindowTitle('SafeRoom Radar — IWR6843 3D People Tracking')
        self._win.resize(1600 if plot3d else 1200, 560)

        main_layout = QtWidgets.QHBoxLayout(self._win)
        main_layout.setContentsMargins(4, 4, 4, 4)
        main_layout.setSpacing(6)

        # ── Plots (left side) ─────────────────────────────────────────────
        self._glw = pg.GraphicsLayoutWidget()
        main_layout.addWidget(self._glw, stretch=7 if plot3d else 10)

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

        # M2 — Boundary box (static, drawn once)
        if boundary_box is not None:
            xmin, xmax, ymin, ymax, zmin, zmax = boundary_box
            box_pen = pg.mkPen(color=(80, 80, 200), width=1.5,
                               style=QtCore.Qt.DashLine)
            # Top view: X-Y rectangle
            bx_xy = np.array([xmin, xmax, xmax, xmin, xmin])
            by_xy = np.array([ymin, ymin, ymax, ymax, ymin])
            self._plot_xy.plot(bx_xy, by_xy, pen=box_pen)
            # Side view: X-Z rectangle
            bx_xz = np.array([xmin, xmax, xmax, xmin, xmin])
            bz_xz = np.array([zmin, zmin, zmax, zmax, zmin])
            self._plot_xz.plot(bx_xz, bz_xz, pen=box_pen)
            # Sensor marker (triangle ▲) in top view at origin
            self._plot_xy.plot([0], [0], pen=None,
                               symbol='t', symbolSize=12,
                               symbolBrush=(255, 100, 0), symbolPen=None)

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

        lbl_title = QtWidgets.QLabel('FALLS\ndetected')
        lbl_title.setAlignment(QtCore.Qt.AlignCenter)
        lbl_title.setStyleSheet('color: darkred; font-weight: bold; font-size: 11px;')

        self._lbl_count = QtWidgets.QLabel('0')
        self._lbl_count.setAlignment(QtCore.Qt.AlignCenter)
        self._lbl_count.setStyleSheet(
            'color: red; font-size: 52px; font-weight: bold;')

        self._lbl_last = QtWidgets.QLabel('')
        self._lbl_last.setAlignment(QtCore.Qt.AlignCenter)
        self._lbl_last.setStyleSheet('color: gray; font-size: 9px;')

        lbl_faint_title = QtWidgets.QLabel('FAINTS\ndetected')
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

        # ── Optional 3D GL view ──────────────────────────────────────────
        if plot3d:
            try:
                import pyqtgraph.opengl as gl

                gl_widget = gl.GLViewWidget()
                gl_widget.setMinimumWidth(380)
                gl_widget.setCameraPosition(distance=6.0, elevation=25, azimuth=225)
                # Insert between the 2D plots and the fall counter
                main_layout.insertWidget(1, gl_widget, stretch=5)
                self._gl_view = gl_widget

                # Floor grid centred on the room
                grid = gl.GLGridItem()
                grid.setSize(x=3, y=5)
                grid.setSpacing(x=0.5, y=0.5)
                grid.translate(0, 2.0, 0)
                gl_widget.addItem(grid)

                # Boundary box wireframe
                if boundary_box is not None:
                    xmin, xmax, ymin, ymax, zmin, zmax = boundary_box
                    box = gl.GLBoxItem(color=(80, 80, 200, 60))
                    box.setSize(x=xmax - xmin, y=ymax - ymin, z=zmax - zmin)
                    box.translate(xmin, ymin, zmin)
                    gl_widget.addItem(box)

                # Sensor marker at origin
                sensor_dot = gl.GLScatterPlotItem(
                    pos=np.array([[0.0, 0.0, 0.0]], dtype=np.float32),
                    color=np.array([[1.0, 0.4, 0.0, 1.0]], dtype=np.float32),
                    size=14, pxMode=True,
                )
                gl_widget.addItem(sensor_dot)

                # Point cloud scatter (starts empty)
                self._gl_scat = gl.GLScatterPlotItem(
                    pos=np.zeros((1, 3), dtype=np.float32),
                    color=np.zeros((1, 4), dtype=np.float32),
                    size=5, pxMode=True,
                )
                gl_widget.addItem(self._gl_scat)

            except ImportError:
                print('[WARN] --plot3d requires PyOpenGL: pip install PyOpenGL')
                self._gl_view = None

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
            # Pass Nx4 uint8 numpy array directly — avoids creating N QBrush objects
            self._scat_xy.setData(x=xs, y=ys, brush=colors)
            self._scat_xz.setData(x=xs, y=zs, brush=colors)
        else:
            self._scat_xy.setData(x=[], y=[])
            self._scat_xz.setData(x=[], y=[])

        # ── Update trail buffers + remove gone tracks ─────────────────────
        active_tids = {t['tid'] for t in tracks}
        for gone in list(self._trails.keys()):
            if gone not in active_tids:
                del self._trails[gone]
                if gone in self._trail_curves:
                    self._plot_xy.removeItem(self._trail_curves[gone][0])
                    self._plot_xz.removeItem(self._trail_curves[gone][1])
                    del self._trail_curves[gone]
                if gone in self._track_labels:
                    self._plot_xy.removeItem(self._track_labels[gone][0])
                    self._plot_xz.removeItem(self._track_labels[gone][1])
                    del self._track_labels[gone]

        # ── Draw tracks and trails (reuse existing scene items) ───────────
        for t in tracks:
            tid = t['tid']
            is_fall  = tid in fall_tids
            is_faint = tid in faint_tids
            color = (220, 30, 30) if is_fall else (220, 140, 0) if is_faint else (30, 90, 200)

            h = heights.get(tid, {})
            cluster_h = h.get('maxZ', 0.0) - h.get('minZ', 0.0) if h else float('nan')
            z_label = h.get('maxZ', t['z']) if h else t['z']

            # Append current position to trail
            if tid not in self._trails:
                self._trails[tid] = collections.deque(maxlen=self.TRAIL_LEN)
            self._trails[tid].append((t['x'], t['y'], z_label))

            # Trail: reuse PlotCurveItem, only addItem on first appearance
            trail = self._trails[tid]
            if len(trail) >= 2:
                txs = np.array([p[0] for p in trail], dtype=np.float32)
                tys = np.array([p[1] for p in trail], dtype=np.float32)
                tzs = np.array([p[2] for p in trail], dtype=np.float32)
                trail_pen = pg.mkPen(color=(*color, 80), width=1.5)
                if tid in self._trail_curves:
                    self._trail_curves[tid][0].setData(x=txs, y=tys)
                    self._trail_curves[tid][0].setPen(trail_pen)
                    self._trail_curves[tid][1].setData(x=txs, y=tzs)
                    self._trail_curves[tid][1].setPen(trail_pen)
                else:
                    ln_xy = pg.PlotCurveItem(x=txs, y=tys, pen=trail_pen)
                    ln_xz = pg.PlotCurveItem(x=txs, y=tzs, pen=trail_pen)
                    self._plot_xy.addItem(ln_xy)
                    self._plot_xz.addItem(ln_xz)
                    self._trail_curves[tid] = (ln_xy, ln_xz)

            label = f"T{tid}"
            if not math.isnan(cluster_h):
                label += f"\nh={cluster_h:.2f}m"
            if is_fall:
                label += "\n⚠ FALL"
            elif is_faint:
                label += "\n⚠ FAINT"

            # Labels: reuse TextItem, only addItem on first appearance
            # t['z'] from TLV 1010 is at ground level (tracker projection).
            # Place label at maxZ (top of cluster) so it sits on the person.
            if tid in self._track_labels:
                txt_xy, txt_xz = self._track_labels[tid]
                txt_xy.setText(label)
                txt_xy.setColor(color)
                txt_xy.setPos(t['x'], t['y'])
                txt_xz.setText(label)
                txt_xz.setColor(color)
                txt_xz.setPos(t['x'], z_label)
            else:
                txt_xy = pg.TextItem(label, color=color, anchor=(0.5, 1.0))
                txt_xy.setPos(t['x'], t['y'])
                self._plot_xy.addItem(txt_xy)
                txt_xz = pg.TextItem(label, color=color, anchor=(0.5, 1.0))
                txt_xz.setPos(t['x'], z_label)
                self._plot_xz.addItem(txt_xz)
                self._track_labels[tid] = (txt_xy, txt_xz)

        # ── 3D GL update (only when --plot3d) ────────────────────────────
        if self._gl_view is not None:
            import pyqtgraph.opengl as gl

            # Point cloud
            if pts:
                pos3d = np.column_stack([xs, ys, zs]).astype(np.float32)
                col3d = (colors.astype(np.float32) / 255.0)
                self._gl_scat.setData(pos=pos3d, color=col3d)
            else:
                self._gl_scat.setData(
                    pos=np.zeros((1, 3), dtype=np.float32),
                    color=np.zeros((1, 4), dtype=np.float32),
                )

            # Remove items for gone tracks
            for gone in [tid for tid in self._gl_trails if tid not in active_tids]:
                self._gl_view.removeItem(self._gl_trails.pop(gone))
            for gone in [tid for tid in self._gl_boxes if tid not in active_tids]:
                self._gl_view.removeItem(self._gl_boxes.pop(gone))

            for t in tracks:
                tid = t['tid']
                is_fall  = tid in fall_tids
                is_faint = tid in faint_tids
                r, g, b = (0.86, 0.12, 0.12) if is_fall else \
                          (0.86, 0.55, 0.0)  if is_faint else \
                          (0.12, 0.35, 0.78)

                # Trail
                trail = self._trails.get(tid)
                if trail and len(trail) >= 2:
                    pos3d = np.array([[p[0], p[1], p[2]] for p in trail],
                                     dtype=np.float32)
                    if tid in self._gl_trails:
                        self._gl_trails[tid].setData(pos=pos3d,
                                                     color=(r, g, b, 0.5))
                    else:
                        line = gl.GLLinePlotItem(pos=pos3d,
                                                 color=(r, g, b, 0.5),
                                                 width=2.0, antialias=True)
                        self._gl_view.addItem(line)
                        self._gl_trails[tid] = line

                # Person wireframe box (uses TLV 1012 height data)
                h = heights.get(tid)
                if h:
                    minZ = h.get('minZ', 0.0)
                    maxZ = h.get('maxZ', minZ + 0.3)
                    edges = _person_box_edges(t['x'], t['y'], minZ, maxZ)
                    if tid in self._gl_boxes:
                        self._gl_boxes[tid].setData(pos=edges,
                                                    color=(r, g, b, 0.9))
                    else:
                        box_line = gl.GLLinePlotItem(pos=edges,
                                                     color=(r, g, b, 0.9),
                                                     width=1.5,
                                                     antialias=True,
                                                     mode='lines')
                        self._gl_view.addItem(box_line)
                        self._gl_boxes[tid] = box_line

        # ── Fall / faint counters + M3 flash ─────────────────────────────
        if fall_tids:
            self._total_falls += len(fall_tids)
            self._lbl_count.setText(str(self._total_falls))
            now_str = datetime.datetime.now().strftime('%H:%M:%S')
            self._lbl_last.setText(f'last\n{now_str}')
            self._flash_until = time.time() + 2.0
            self._flash_color = '#ffcccc'
        if faint_tids:
            self._total_faints += len(faint_tids)
            self._lbl_faint_count.setText(str(self._total_faints))
            if not fall_tids:   # fall flash takes priority
                self._flash_until = time.time() + 2.0
                self._flash_color = '#ffe4b5'

        # Apply or clear flash background
        if time.time() < self._flash_until:
            self._win.setStyleSheet(f'background-color: {self._flash_color};')
        else:
            self._win.setStyleSheet('')

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
               sensor_h, sensor_t, use_plot, ml_detector=None, z_offset=0.0):
    frame_count  = 0
    fall_count   = 0
    faint_count  = 0
    ml_fall_count = 0
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
                p['z'] += z_offset

            # Apply the same z_offset to tracks and heights (firmware world frame).
            # Tracks come pre-transformed by the firmware using sensorPosition.
            # If sensorPosition height is off, z_offset corrects both together.
            if z_offset != 0.0:
                for t in frame['tracks']:
                    t['z'] += z_offset
                for h in frame['heights'].values():
                    h['maxZ'] += z_offset
                    h['minZ'] += z_offset

            frame_count += 1
            tracks  = frame['tracks']
            heights = frame['heights']

            # Fall / faint detection
            fall_tids  = set()
            faint_tids = set()
            ml_fall_tids = set()
            active_tids = {t['tid'] for t in tracks}
            for t in tracks:
                tid = t['tid']
                h = heights.get(tid)  # None if no TLV 1012 data this frame
                if h is None:
                    # No height data — skip to avoid false falls from h=0 fallback.
                    fall_detector.bump_frame(tid)
                    continue
                # Ignore ghost tracks near standing persons: small vertical extent
                # AND above floor level. A fallen person is also small vertically
                # (0.07–0.15 m) but at floor level (maxZ < 0.8 m) — must pass through.
                _h_extent = h.get('maxZ', 0) - h.get('minZ', 0)
                _max_z    = h.get('maxZ', 0)
                if _h_extent < 0.25 and _max_z > 0.8:
                    fall_detector.bump_frame(tid)
                    continue
                # Ignore tracks with too few points when above floor level.
                # Ghost tracks have <10 points and maxZ > 0.8m.
                # A person on the floor (faint) also has few points but maxZ < 0.8m — must pass through.
                _n_pts = sum(1 for idx in frame['indices'] if idx == tid)
                if _n_pts < 10 and _max_z > 0.8:
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

                # ML detector (runs alongside rule-based, independent)
                if ml_detector is not None and h is not None:
                    pc = compute_pc_stats(frame['points'], frame['indices'], tid)
                    is_fall_ml, prob_ml = ml_detector.update(tid, t, h, pc)
                    if is_fall_ml:
                        ml_fall_tids.add(tid)
                        ml_fall_count += 1
                        print(f'\n[ML] FALL DETECTED — Track {tid} '
                              f'(p={prob_ml:.2f})\n')

            fall_detector.cleanup_old_tracks(active_tids)
            if ml_detector is not None:
                ml_detector.cleanup_old_tracks(active_tids)

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
                if tid in ml_fall_tids:
                    flag += ' [ML⚠]'
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
        msg = f'\nStopped. Frames: {frame_count}, Falls: {fall_count}, Faints: {faint_count}'
        if ml_detector is not None:
            msg += f', ML falls: {ml_fall_count}'
        print(msg)


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
    parser.add_argument('--plot',   action='store_true', help='Show live 2D PyQtGraph visualization')
    parser.add_argument('--plot3d', action='store_true', help='Add 3D OpenGL panel (requires PyOpenGL)')
    parser.add_argument('--log',  default=None,
                        help='Path to CSV log file. If omitted, auto-saves to logs/session_YYYYMMDD_HHMMSS.csv')
    parser.add_argument('--frame-period', type=float, default=0.05,
                        help='Expected frame period in seconds (default: 0.05 = 50ms = 20fps)')
    parser.add_argument('--sensor-height', type=float, default=2.05,
                        help='Sensor height above floor in meters (default: 2.05)')
    parser.add_argument('--sensor-tilt', type=float, default=15.0,
                        help='Sensor downward tilt in degrees (default: 15.0)')
    parser.add_argument('--z-offset', type=float, default=0.0, metavar='METERS',
                        help='Z correction in metres applied to all points and tracks '
                             '(e.g. --z-offset -0.20 to shift down 20 cm). '
                             'Use to calibrate when objects appear at the wrong height.')
    # ML dataset collection
    parser.add_argument('--ml-log', action='store_true',
                        help='Use enhanced ML CSV logger (more columns: ax/ay/az, pc stats, label)')
    parser.add_argument('--label-mode', action='store_true',
                        help='Activate real-time keyboard labeling (requires --ml-log). '
                             'Keys: f=fall  n=normal  s=sitting  w=walking')
    # ML inference
    parser.add_argument('--ml-model', default=None, metavar='PATH',
                        help='Path to trained ML fall detector (.pkl or .pt). '
                             'Runs alongside rule-based detector when provided.')
    args = parser.parse_args()

    import os, datetime

    # Auto-generate log path
    if args.log is None:
        os.makedirs('logs', exist_ok=True)
        ts = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
        prefix = 'ml_session' if args.ml_log else 'session'
        args.log = f'logs/{prefix}_{ts}.csv'
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

    # Choose logger
    if args.ml_log:
        if not _ML_LOGGER_AVAILABLE:
            print('[ERROR] ml_logger.py not found in tools/. Cannot use --ml-log.')
            sys.exit(1)
        session_id = os.path.splitext(os.path.basename(args.log))[0]
        logger = MlCsvLogger(args.log, session_id=session_id,
                              label_mode=args.label_mode)
    else:
        if args.label_mode:
            print('[WARN] --label-mode has no effect without --ml-log.')
        logger = CsvLogger(args.log)

    # ML fall detector (optional, runs alongside rule-based)
    ml_detector = None
    if args.ml_model:
        if not _ML_INFERENCE_AVAILABLE:
            print('[WARN] ml_inference.py not found. Ignoring --ml-model.')
        elif not _ML_LOGGER_AVAILABLE:
            print('[WARN] ml_logger.py not found (needed for compute_pc_stats). '
                  'Ignoring --ml-model.')
        else:
            try:
                ml_detector = MlFallDetector(args.ml_model)
            except Exception as e:
                print(f'[WARN] Could not load ML model: {e}')

    import queue
    frame_queue = queue.Queue(maxsize=4)
    stop_event  = threading.Event()

    if args.plot or args.plot3d:
        # Qt event loop must run in the main thread.
        # Frame reader runs in a daemon background thread.
        bbox = parse_boundary_box(args.cfg)
        window = RadarWindow(frame_queue, stop_event, boundary_box=bbox,
                             plot3d=args.plot3d)
        window.show()

        reader_thread = threading.Thread(
            target=_read_loop,
            args=(reader, fall_detector, logger, frame_queue, stop_event,
                  args.sensor_height, args.sensor_tilt, True),
            kwargs={'ml_detector': ml_detector, 'z_offset': args.z_offset},
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
                       args.sensor_height, args.sensor_tilt, False,
                       ml_detector=ml_detector, z_offset=args.z_offset)
        finally:
            data_ser.close()
            logger.close()


if __name__ == '__main__':
    main()
