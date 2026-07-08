"""Tests for tools/replay_session.py — offline session replay driver."""
import json
import os

import numpy as np
import pandas as pd
import pytest

from replay_session import replay_rules, load_session, replay_session, fuse
from event_metrics import Detection
from ir_confirmer import ConfirmerResult

NS = int(1e9)
FRAME_S = 0.05  # 20 fps


def _make_radar_df(n_frames=120, fall_at=None, tid=1, t0_ns=0):
    """One-track radar DataFrame. Standing until `fall_at`, then a fast fall
    (vz -1.5 m/s for 4 frames) and floor level afterwards."""
    rows = []
    for i in range(n_frames):
        falling = fall_at is not None and fall_at <= i < fall_at + 4
        fallen = fall_at is not None and i >= fall_at + 4
        maxz = 0.4 if fallen else (0.9 if falling else 1.5)
        vz = -1.5 if falling else 0.0
        rows.append({
            't_mono_ns': t0_ns + int(i * FRAME_S * NS),
            'frameNum': i, 'tid': tid,
            'x': 0.0, 'y': 2.0, 'z': maxz / 2,
            'vx': 0.0, 'vy': 0.0, 'vz': vz,
            'ax': 0.0, 'ay': 0.0, 'az': -9.0 if falling else 0.0,
            'g': 1.0, 'confidence': 0.9,
            'maxZ': maxz, 'minZ': 0.1, 'height_m': maxz - 0.1,
            'pc_n_points': 30, 'pc_mean_doppler': vz, 'pc_std_doppler': 0.3,
            'pc_mean_snr': 150.0, 'pc_std_snr': 20.0,
            'pc_mean_z': maxz / 2, 'pc_std_z': 0.2, 'pc_z_span': 0.4,
        })
    return pd.DataFrame(rows)


class TestReplayRules:
    def test_detects_fast_fall(self):
        df = _make_radar_df(fall_at=60)
        dets = replay_rules(df)
        falls = [d for d in dets if d.extra.get('type') == 'fall']
        assert len(falls) == 1
        # Fires within the vz burst (frames 60-63), using session time
        t_fall_s = falls[0].t_mono_ns / NS
        assert 60 * FRAME_S <= t_fall_s <= 64 * FRAME_S

    def test_no_fall_when_standing(self):
        df = _make_radar_df(fall_at=None)
        assert replay_rules(df) == []

    def test_uses_session_time_not_wall_time(self):
        # Two falls 30 session-seconds apart (> 25 s cooldown) must BOTH be
        # detected even though the replay runs in milliseconds of wall time.
        df1 = _make_radar_df(n_frames=700, fall_at=60)
        df2 = _make_radar_df(n_frames=120, fall_at=40,
                             t0_ns=int(700 * FRAME_S * NS))
        df = pd.concat([df1, df2], ignore_index=True)
        dets = replay_rules(df)
        falls = [d for d in dets if d.extra.get('type') == 'fall']
        assert len(falls) == 2


@pytest.fixture
def session_dir(tmp_path):
    """Minimal session directory: radar.csv + manifest.json (no thermal)."""
    df = _make_radar_df(n_frames=200, fall_at=100)
    df['label'] = ['fall' if 100 <= i < 110 else 'walk' for i in range(200)]
    df.insert(1, 'timestamp', 1779206352.0 + df['t_mono_ns'] / NS)
    df.insert(2, 'session_id', 'test_sess')
    df.to_csv(tmp_path / 'radar.csv', index=False)

    manifest = {
        'session_id': 'test_sess',
        'started_at_mono_ns': 0,
        'duration_s': 10.0,
        'subject': 'test',
        'radar': {'rows': 200, 'frames': 200},
        'thermal': {'frames': 0},
        'labels': [{
            't_start_mono_ns': int(100 * FRAME_S * NS),
            't_end_mono_ns': int(110 * FRAME_S * NS),
            'label': 'fall', 'notes': '',
        }],
        'schema_version': 1,
    }
    (tmp_path / 'manifest.json').write_text(json.dumps(manifest))
    return str(tmp_path)


class TestReplaySession:
    def test_load_session(self, session_dir):
        sess = load_session(session_dir)
        assert len(sess.radar_df) == 200
        assert sess.manifest.duration_s == 10.0
        assert sess.thermal_frames is None       # no thermal.npz

    def test_rules_metrics_on_session(self, session_dir):
        result = replay_session(session_dir)
        m = result['detectors']['rules']['metrics']
        assert m['n_events'] == 1
        assert m['n_detected'] == 1
        assert m['event_recall'] == 1.0
        assert m['n_false_alarms'] == 0
        assert m['latency_median_s'] < 1.0

    def test_result_includes_session_metadata(self, session_dir):
        result = replay_session(session_dir)
        assert result['session_id'] == 'test_sess'
        assert result['duration_s'] == 10.0


class TestGtEventMerging:
    def test_fall_lying_extends_gt_event_for_matching(self, session_dir):
        # Re-label the manifest: short fall impact + long fall_lying tail.
        # A late detection during the lying tail (e.g. Tier-2 faint) must
        # match the event instead of counting as a false alarm.
        manifest = json.loads(
            open(os.path.join(session_dir, 'manifest.json')).read())
        manifest['labels'] = [
            {'t_start_mono_ns': int(5.0 * NS), 't_end_mono_ns': int(6.0 * NS),
             'label': 'fall', 'notes': ''},
            {'t_start_mono_ns': int(6.0 * NS), 't_end_mono_ns': int(9.5 * NS),
             'label': 'fall_lying', 'notes': ''},
        ]
        with open(os.path.join(session_dir, 'manifest.json'), 'w') as f:
            json.dump(manifest, f)

        from replay_session import build_gt_events
        from manifest_schema import Manifest
        events = build_gt_events(Manifest.load(session_dir))
        assert len(events) == 1
        assert events[0].t_start_mono_ns == int(5.0 * NS)
        assert events[0].t_end_mono_ns == int(9.5 * NS)

    def test_standalone_fall_not_extended(self, session_dir):
        from replay_session import build_gt_events
        from manifest_schema import Manifest
        events = build_gt_events(Manifest.load(session_dir))
        assert len(events) == 1
        assert events[0].t_end_mono_ns == int(110 * FRAME_S * NS)


class TestFuse:
    def test_vetoed_detections_are_suppressed(self):
        dets = [Detection(1 * NS, 'rules', 1, {'type': 'fall'}),
                Detection(2 * NS, 'rules', 1, {'type': 'fall'}),
                Detection(3 * NS, 'rules', 1, {'type': 'fall'})]
        ir_results = {
            id(dets[0]): ConfirmerResult(decision='confirmed', confidence=0.9),
            id(dets[1]): ConfirmerResult(decision='vetoed', confidence=0.0),
            id(dets[2]): ConfirmerResult(decision='failopen', confidence=0.0),
        }
        fused = fuse(dets, ir_results)
        assert [d.t_mono_ns for d in fused] == [1 * NS, 3 * NS]
        assert fused[0].source == 'rules+ir'
        assert fused[0].extra['ir_decision'] == 'confirmed'
        assert fused[1].extra['ir_decision'] == 'failopen'
