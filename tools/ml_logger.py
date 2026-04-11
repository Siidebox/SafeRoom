#!/usr/bin/env python3
"""
ml_logger.py — Enhanced CSV logger for ML dataset collection.

Extends the basic CsvLogger with:
  - Kalman acceleration (ax, ay, az) and tracker metadata (g, confidence)
  - Point cloud statistics per track (TLV 1011 point-to-track mapping)
  - Frame-level statistics (total points, track count)
  - Ground-truth label column (keyboard annotation)

Usage:
    from ml_logger import MlCsvLogger

    logger = MlCsvLogger('logs/ml_session.csv', session_id='s01', label_mode=True)
    # In read loop:
    logger.log(frame, fall_tids, faint_tids, fall_detector)
    logger.close()

Keyboard labels (when label_mode=True):
    f → fall      n → normal      s → sitting      w → walking
"""

import csv
import math
import sys
import threading
import time

import numpy as np


# ── Point cloud aggregation ──────────────────────────────────────────────────

def compute_pc_stats(points: list, indices: list, tid: int) -> dict:
    """
    Compute point cloud statistics for points assigned to a given track ID.

    Uses TLV 1011 target index list to filter points belonging to `tid`.
    Returns a dict of per-track aggregated features.
    All values are 0 / 0.0 when no points are assigned.
    """
    assigned = []
    for i in range(min(len(points), len(indices))):
        if indices[i] == tid:
            assigned.append(points[i])

    if not assigned:
        return _empty_pc_stats()

    dopplers = np.array([p['doppler'] for p in assigned], dtype=np.float32)
    snrs     = np.array([p['snr']     for p in assigned], dtype=np.float32)
    zs       = np.array([p['z']       for p in assigned], dtype=np.float32)

    return {
        'pc_n_points':     len(assigned),
        'pc_mean_doppler': float(np.mean(dopplers)),
        'pc_std_doppler':  float(np.std(dopplers))  if len(dopplers) > 1 else 0.0,
        'pc_mean_snr':     float(np.mean(snrs)),
        'pc_std_snr':      float(np.std(snrs))      if len(snrs) > 1 else 0.0,
        'pc_mean_z':       float(np.mean(zs)),
        'pc_std_z':        float(np.std(zs))         if len(zs) > 1 else 0.0,
        'pc_z_span':       float(np.max(zs) - np.min(zs)),
    }


def _empty_pc_stats() -> dict:
    return {
        'pc_n_points': 0,
        'pc_mean_doppler': 0.0, 'pc_std_doppler': 0.0,
        'pc_mean_snr':     0.0, 'pc_std_snr':     0.0,
        'pc_mean_z':       0.0, 'pc_std_z':       0.0,
        'pc_z_span':       0.0,
    }


# ── Keyboard labeler ─────────────────────────────────────────────────────────

class KeyboardLabeler:
    """
    Non-blocking keyboard listener for real-time session labeling.

    Tries pynput first; falls back to msvcrt (Windows) or tty (Linux/macOS).
    Label persists until changed — press once, stays until next keypress.

    Key bindings:
        f → 'fall'      (press at fall onset, keep until person recovers)
        n → 'normal'    (default)
        s → 'sitting'
        w → 'walking'
    """

    LABEL_KEYS = {
        'f': 'fall',
        'n': 'normal',
        's': 'sitting',
        'w': 'walking',
    }

    def __init__(self):
        self._label   = 'normal'
        self._lock    = threading.Lock()
        self._running = False
        self._thread  = None

    @property
    def current_label(self) -> str:
        with self._lock:
            return self._label

    def _set_label(self, ch: str):
        if not isinstance(ch, str):
            return
        ch = ch.lower()
        if ch in self.LABEL_KEYS:
            with self._lock:
                self._label = self.LABEL_KEYS[ch]
            print(f'\n[LABEL] → {self.LABEL_KEYS[ch]}', flush=True)

    # ── backend implementations ──────────────────────────────────────────────

    def _listen_pynput(self):
        from pynput import keyboard

        def on_press(key):
            if not self._running:
                return False
            try:
                self._set_label(key.char)
            except AttributeError:
                pass  # special key (Shift, Ctrl, …)

        with keyboard.Listener(on_press=on_press) as listener:
            while self._running:
                time.sleep(0.1)

    def _listen_msvcrt(self):
        import msvcrt
        while self._running:
            if msvcrt.kbhit():
                try:
                    ch = msvcrt.getch().decode('utf-8', errors='ignore')
                    self._set_label(ch)
                except Exception:
                    pass
            time.sleep(0.05)

    def _listen_tty(self):
        import tty
        import termios
        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            while self._running:
                ch = sys.stdin.read(1)
                self._set_label(ch)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)

    # ── public interface ─────────────────────────────────────────────────────

    def start(self):
        self._running = True
        method = 'pynput'
        try:
            import pynput  # noqa: F401
            target = self._listen_pynput
        except ImportError:
            if sys.platform == 'win32':
                target = self._listen_msvcrt
                method = 'msvcrt'
            else:
                target = self._listen_tty
                method = 'tty'

        self._thread = threading.Thread(target=target, daemon=True)
        self._thread.start()
        print(
            f'[LABEL] Keyboard labeler active ({method}). '
            'Keys: f=fall  n=normal  s=sitting  w=walking'
        )

    def stop(self):
        self._running = False


# ── Column definition ────────────────────────────────────────────────────────

ML_COLUMNS = [
    # Identification
    'timestamp', 'session_id', 'frameNum', 'presence',
    # Track kinematics
    'tid', 'x', 'y', 'z', 'vx', 'vy', 'vz',
    # Kalman accelerations + tracker metadata (already parsed by parse_tracks,
    # but not logged by the original CsvLogger)
    'ax', 'ay', 'az', 'g', 'confidence',
    # Height envelope
    'height_m', 'maxZ', 'minZ', 'maxZ_ref', 'peak_vz',
    # Rule-based detector outputs (kept for comparison baseline)
    'fall_detected', 'faint_detected',
    # Point cloud stats (derived via TLV 1011 target index)
    'pc_n_points',
    'pc_mean_doppler', 'pc_std_doppler',
    'pc_mean_snr',     'pc_std_snr',
    'pc_mean_z',       'pc_std_z',
    'pc_z_span',
    # Frame-level
    'num_total_points', 'num_tracks',
    # Ground-truth label from keyboard annotation
    'label',
]


# ── Main logger class ────────────────────────────────────────────────────────

class MlCsvLogger:
    """
    Enhanced CSV logger that captures the full feature set for ML training.

    Has the same .log() / .close() interface as CsvLogger so it can be used
    as a drop-in replacement in the radar_reader _read_loop.

    Parameters
    ----------
    path : str
        Output CSV file path (auto-created, overwritten if exists).
    session_id : str, optional
        Short identifier embedded in every row (e.g. 's01', 'living_room').
        Defaults to the filename stem.
    label_mode : bool
        Start a KeyboardLabeler thread for real-time annotation.
    """

    def __init__(self, path: str, session_id: str = '', label_mode: bool = False):
        self._f          = open(path, 'w', newline='')
        self._w          = csv.writer(self._f)
        self._w.writerow(ML_COLUMNS)
        self._session_id = session_id
        self._labeler    = None

        if label_mode:
            self._labeler = KeyboardLabeler()
            self._labeler.start()

        print(f'[ML Logger] {path}  |  label_mode={label_mode}  |  {len(ML_COLUMNS)} columns')

    def log(self, frame: dict, fall_tids: set,
            faint_tids: set = None, fall_detector=None):
        """
        Write one row per track per frame to the CSV.

        When there are no tracks, writes a single presence-only row with
        empty track fields (same convention as CsvLogger).
        """
        ts       = time.time()
        presence = frame['presence']
        fn       = frame['frameNum']
        points   = frame['points']
        indices  = frame['indices']
        tracks   = frame['tracks']
        heights  = frame['heights']

        n_total_pts = len(points)
        n_tracks    = len(tracks)
        label       = self._labeler.current_label if self._labeler else 'unknown'

        if not tracks:
            self._w.writerow([
                ts, self._session_id, fn, presence,
                '', '', '', '', '', '', '',   # tid + kinematics (7)
                '', '', '', '', '',           # ax ay az g confidence (5)
                '', '', '', '', '',           # height_m maxZ minZ maxZ_ref peak_vz (5)
                0, 0,                         # fall_detected, faint_detected
                0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,  # pc stats (8)
                n_total_pts, n_tracks,
                label,
            ])
            self._f.flush()
            return

        for t in tracks:
            tid   = t['tid']
            h     = heights.get(tid, {})
            max_z = h.get('maxZ', float('nan'))
            min_z = h.get('minZ', float('nan'))
            height_m = (
                (max_z - min_z)
                if not (math.isnan(max_z) or math.isnan(min_z))
                else float('nan')
            )

            ref = (
                round(fall_detector.ref_maxz(tid), 4)
                if fall_detector else ''
            )
            peak_vz = (
                round(fall_detector._last_peak_vz.get(tid, 0.0), 4)
                if fall_detector else ''
            )

            pc = compute_pc_stats(points, indices, tid)

            self._w.writerow([
                ts, self._session_id, fn, presence,
                tid,
                round(t['x'],  4), round(t['y'],  4), round(t['z'],  4),
                round(t['vx'], 4), round(t['vy'], 4), round(t['vz'], 4),
                round(t['ax'], 4), round(t['ay'], 4), round(t['az'], 4),
                round(t['g'],  4), round(t['confidence'], 4),
                round(height_m, 4) if not math.isnan(height_m) else '',
                round(max_z, 4)    if not math.isnan(max_z)    else '',
                round(min_z, 4)    if not math.isnan(min_z)    else '',
                ref, peak_vz,
                1 if tid in fall_tids else 0,
                1 if (faint_tids and tid in faint_tids) else 0,
                pc['pc_n_points'],
                round(pc['pc_mean_doppler'], 4),
                round(pc['pc_std_doppler'],  4),
                round(pc['pc_mean_snr'],     4),
                round(pc['pc_std_snr'],      4),
                round(pc['pc_mean_z'],       4),
                round(pc['pc_std_z'],        4),
                round(pc['pc_z_span'],       4),
                n_total_pts, n_tracks,
                label,
            ])

        self._f.flush()

    def close(self):
        if self._labeler:
            self._labeler.stop()
        self._f.close()
