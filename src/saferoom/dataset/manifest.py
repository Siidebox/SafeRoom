"""
manifest_schema.py — Session manifest writer/reader for synchronized radar+IR.

A "session" is a directory under sessions/ containing:
    radar.csv      — MlCsvLogger output (one row per track per frame)
    thermal.npz    — MLX90640 frames + per-frame t_mono_ns + t_wall
    manifest.json  — metadata produced by Manifest.write() (this module)

Labels are stored as time-range entries keyed on t_mono_ns so a single source
of truth covers both radar.csv (per-row label column) and thermal.npz (per-frame
nearest-neighbour lookup).
"""

import json
import os
import time
from dataclasses import asdict, dataclass, field

SCHEMA_VERSION = 1


@dataclass
class RadarMeta:
    cfg_file: str = ""
    fps_target: float = 20.0
    fps_real: float = 0.0
    rows: int = 0
    frames: int = 0
    drops: int = 0


@dataclass
class ThermalMeta:
    refresh_hz: int = 16
    fps_real: float = 0.0
    frames: int = 0
    bad_frames: int = 0
    shape: list[int] = field(default_factory=lambda: [24, 32])


@dataclass
class SyncMeta:
    clock: str = "monotonic_ns"
    max_drift_ms_observed: float | None = None
    note: str = ""


@dataclass
class LabelSpan:
    t_start_mono_ns: int
    t_end_mono_ns: int
    label: str
    notes: str = ""


@dataclass
class Manifest:
    session_id: str
    started_at_wall: str = ""           # ISO8601 local time
    started_at_mono_ns: int = 0
    duration_s: float = 0.0
    subject: str = ""
    notes: str = ""
    radar: RadarMeta = field(default_factory=RadarMeta)
    thermal: ThermalMeta = field(default_factory=ThermalMeta)
    sync: SyncMeta = field(default_factory=SyncMeta)
    labels: list[LabelSpan] = field(default_factory=list)
    schema_version: int = SCHEMA_VERSION

    # ── io ──

    def write(self, session_dir: str):
        path = os.path.join(session_dir, 'manifest.json')
        with open(path, 'w', encoding='utf-8') as f:
            json.dump(asdict(self), f, indent=2)
        return path

    @classmethod
    def load(cls, session_dir: str) -> 'Manifest':
        path = os.path.join(session_dir, 'manifest.json')
        with open(path, encoding='utf-8') as f:
            d = json.load(f)
        m = cls(session_id=d['session_id'])
        m.started_at_wall    = d.get('started_at_wall', '')
        m.started_at_mono_ns = int(d.get('started_at_mono_ns', 0))
        m.duration_s         = float(d.get('duration_s', 0.0))
        m.subject            = d.get('subject', '')
        m.notes              = d.get('notes', '')
        m.radar              = RadarMeta(**d.get('radar', {}))
        m.thermal            = ThermalMeta(**d.get('thermal', {}))
        m.sync               = SyncMeta(**d.get('sync', {}))
        m.labels             = [LabelSpan(**lbl) for lbl in d.get('labels', [])]
        m.schema_version     = int(d.get('schema_version', 0))
        return m


# ── helpers ──────────────────────────────────────────────────────────────────

def make_session_dir(base: str = "sessions", name: str = "") -> str:
    """Create sessions/<YYYYMMDD_HHMMSS>[_name]/ and return its path."""
    stamp = time.strftime('%Y%m%d_%H%M%S')
    folder = stamp if not name else f"{stamp}_{name}"
    path = os.path.join(base, folder)
    os.makedirs(path, exist_ok=True)
    return path
