"""TLV frame parsing for the IWR6843 3D People Tracking firmware.

Wire format:

    [Magic 8 B][Header 32 B][TLV ...]

Parsed TLV types:

    1020  Compressed point cloud (spherical, unpacked to Cartesian)
    1010  3D Kalman tracker target list (position, velocity, acceleration)
    1011  Target index (point-to-track mapping)
    1012  Target height (maxZ, minZ per track)
    1021  Presence indication
"""

import math
import struct
import time

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

        # Timestamps captured AS SOON AS the magic word is locked, before parsing.
        # t_mono_ns is the shared clock across radar+IR for sync; t_wall is for
        # human-readable correlation. Both flow through frame[...] to the logger.
        t_mono_ns = time.monotonic_ns()
        t_wall    = time.time()

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
            't_mono_ns':   t_mono_ns,
            't_wall':      t_wall,
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
