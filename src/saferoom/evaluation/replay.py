#!/usr/bin/env python3
"""
replay_session.py — Offline replay of recorded sessions through the detectors.

Replays sessions/<id>/ (radar.csv + thermal.npz + manifest.json) through:
    - FallDetector   (rule-based, radar_reader.py)
    - MlFallDetector (trained model, ml_inference.py)      [--ml-model]
    - IrConfirmer    (thermal confirmation, ir_confirmer.py)

and aggregates detections against the manifest LabelSpans into event-level
metrics (event recall, false alarms/hour, detection latency) via
event_metrics.py. This produces the rules vs ML vs fusion comparison on the
SAME recorded events — the paired evaluation a live A/B run cannot give.

Detector cooldowns run on session time (injected clock), so results are
identical to a live run regardless of replay speed.

Usage:
    python tools/replay_session.py sessions/synth_a
    python tools/replay_session.py sessions/* --ml-model models/fall_detector_xgb.pkl
    python tools/replay_session.py sessions/synth_a --json results.json
"""

import argparse
import glob
import json
import os
from dataclasses import dataclass

import numpy as np
import pandas as pd

from saferoom.dataset.manifest import Manifest
from saferoom.evaluation.metrics import Detection, GtEvent, compute_event_metrics
from saferoom.ir.confirmer import BackgroundModel, IrConfirmer, IrConfirmerParams
from saferoom.radar.detector import FallDetector

NS = int(1e9)

PC_STAT_COLS = ['pc_n_points', 'pc_mean_doppler', 'pc_std_doppler',
                'pc_mean_snr', 'pc_std_snr', 'pc_mean_z', 'pc_std_z',
                'pc_z_span']

TRACK_STALE_S = 2.0     # drop per-track detector state after this gap


class SessionClock:
    """Mutable clock driven by the replay loop (seconds)."""

    def __init__(self):
        self.t = 0.0

    def __call__(self) -> float:
        return self.t


@dataclass
class Session:
    session_dir: str
    radar_df: pd.DataFrame
    manifest: Manifest
    thermal_frames: np.ndarray | None     # (N, 24, 32) or None
    thermal_ts: np.ndarray | None         # (N,) t_mono_ns or None


def load_session(session_dir: str, ir_rotate_k: int = 0) -> Session:
    radar_df = pd.read_csv(os.path.join(session_dir, 'radar.csv'))
    radar_df = radar_df.sort_values('t_mono_ns', kind='stable').reset_index(drop=True)
    manifest = Manifest.load(session_dir)

    thermal_frames = thermal_ts = None
    npz_path = os.path.join(session_dir, 'thermal.npz')
    if os.path.isfile(npz_path):
        npz = np.load(npz_path)
        if 'frames' in npz and len(npz['frames']):
            thermal_frames = npz['frames'].astype(np.float32)
            if ir_rotate_k:
                # Live pushes rot90-ed frames to the confirmer (--ir-rotate
                # in radar_reader); recorded npz frames are raw, so replay
                # must apply the same mounting rotation.
                thermal_frames = np.rot90(thermal_frames, k=ir_rotate_k,
                                          axes=(1, 2)).copy()
            thermal_ts = npz['t_mono_ns'].astype(np.int64)
            order = np.argsort(thermal_ts)
            thermal_frames, thermal_ts = thermal_frames[order], thermal_ts[order]

    return Session(session_dir, radar_df, manifest, thermal_frames, thermal_ts)


# ── replay loops ─────────────────────────────────────────────────────────────

def _iter_rows(radar_df: pd.DataFrame):
    """Yield (t_s, t_mono_ns, tid, row) in time order.

    Frames with no active track carry an empty ``tid`` (NaN); they hold no
    per-track detection info, so skip them.
    """
    for row in radar_df.itertuples(index=False):
        tid = getattr(row, 'tid', float('nan'))
        if not np.isfinite(tid):
            continue
        t_ns = int(row.t_mono_ns)
        yield t_ns / NS, t_ns, int(tid), row


def _cleanup(detector, last_seen: dict, now_s: float):
    active = {tid for tid, t in last_seen.items() if now_s - t <= TRACK_STALE_S}
    if len(active) != len(last_seen):
        detector.cleanup_old_tracks(active)
        for tid in list(last_seen):
            if tid not in active:
                del last_seen[tid]


def replay_rules(radar_df: pd.DataFrame, clock: SessionClock | None = None
                 ) -> list[Detection]:
    """Run FallDetector over a session in session time."""
    clock = clock or SessionClock()
    fd = FallDetector(clock=clock)
    detections: list[Detection] = []
    last_seen: dict = {}

    for t_s, t_ns, tid, row in _iter_rows(radar_df):
        clock.t = t_s
        last_seen[tid] = t_s
        max_z = getattr(row, 'maxZ', float('nan'))
        if not np.isfinite(max_z):
            fd.bump_frame(tid)
            continue
        fall, faint = fd.update(
            tid,
            {'maxZ': float(max_z), 'minZ': float(getattr(row, 'minZ', 0.0))},
            {'vz': float(getattr(row, 'vz', 0.0))},
        )
        if fall:
            detections.append(Detection(t_ns, 'rules', tid, {'type': 'fall'}))
        if faint:
            detections.append(Detection(t_ns, 'rules', tid, {'type': 'faint'}))
        _cleanup(fd, last_seen, t_s)

    return detections


def replay_ml(radar_df: pd.DataFrame, model_path: str,
              window_size: int = 30, stride: int = 5,
              threshold: float = 0.5, cooldown: float = 5.0,
              clock: SessionClock | None = None) -> list[Detection]:
    """Run MlFallDetector over a session in session time."""
    from saferoom.ml.inference import MlFallDetector

    clock = clock or SessionClock()
    det = MlFallDetector(model_path, window_size=window_size, stride=stride,
                         threshold=threshold, cooldown=cooldown, clock=clock)
    detections: list[Detection] = []
    last_seen: dict = {}

    for t_s, t_ns, tid, row in _iter_rows(radar_df):
        clock.t = t_s
        last_seen[tid] = t_s
        track = {k: float(getattr(row, k, 0.0)) for k in
                 ('x', 'y', 'z', 'vx', 'vy', 'vz', 'ax', 'ay', 'az',
                  'g', 'confidence')}
        max_z = getattr(row, 'maxZ', float('nan'))
        height = {'maxZ': float(max_z) if np.isfinite(max_z) else 0.0,
                  'minZ': float(getattr(row, 'minZ', 0.0))}
        pc_stats = {k: float(getattr(row, k)) for k in PC_STAT_COLS
                    if hasattr(row, k) and np.isfinite(getattr(row, k))}
        is_fall, prob = det.update(tid, track, height, pc_stats)
        if is_fall:
            detections.append(Detection(t_ns, 'ml', tid,
                                        {'type': 'fall', 'prob': prob}))
        _cleanup(det, last_seen, t_s)

    return detections


def load_background_from_session(calib_dir: str,
                                 params: IrConfirmerParams | None = None,
                                 ir_rotate_k: int = 0) -> BackgroundModel:
    """Build a thermal BackgroundModel from a dedicated empty-room
    calibration session (60 s recorded at the start of each capture block).

    Mirrors live behavior — the confirmer calibrates once at startup —
    without wasting 30 s of empty room at the start of every session.

    Raises ValueError when thermal data is missing or the safeguard rejects
    the frames (someone was in the room during calibration).
    """
    npz_path = os.path.join(calib_dir, 'thermal.npz')
    if not os.path.isfile(npz_path):
        raise ValueError(f'No thermal.npz in calibration session: {calib_dir}')
    npz = np.load(npz_path)
    frames = npz['frames'].astype(np.float32)
    if not len(frames):
        raise ValueError(f'Empty thermal.npz in calibration session: {calib_dir}')
    if ir_rotate_k:
        frames = np.rot90(frames, k=ir_rotate_k, axes=(1, 2)).copy()

    bg = BackgroundModel(params or IrConfirmerParams())
    for f in frames:
        bg.feed(f)
    if not bg.finalize():
        raise ValueError(
            f'Calibration session rejected ({calib_dir}): '
            f'{bg.last_reject_reason}. The room must be empty while '
            f'recording the calib session.')
    return bg


def replay_ir(session: Session, detections: list[Detection],
              params: IrConfirmerParams | None = None,
              background: BackgroundModel | None = None) -> dict | None:
    """Evaluate the IrConfirmer at each detection timestamp.

    Offline we widen the ring buffer to the whole session so evaluation can
    happen after all frames are pushed. Without `background`, startup
    auto-calibration consumes the first calibration_seconds of frames
    exactly as live; with it (from load_background_from_session) the
    confirmer starts calibrated.

    Returns {'calibrated': bool, 'results': {id(det): ConfirmerResult}} or
    None when the session has no thermal data.
    """
    if session.thermal_frames is None:
        return None

    params = params or IrConfirmerParams()
    span_s = (int(session.thermal_ts[-1]) - int(session.thermal_ts[0])) / NS
    params.buffer_seconds = max(params.buffer_seconds, span_s + 1.0)

    confirmer = IrConfirmer(params=params, background=background)
    for frame, t_ns in zip(session.thermal_frames, session.thermal_ts, strict=False):
        confirmer.push(frame, int(t_ns))

    results = {}
    for d in detections:
        results[id(d)] = confirmer.evaluate(d.t_mono_ns)
    return {'calibrated': confirmer.is_calibrated(), 'results': results}


def fuse(detections: list[Detection], ir_results: dict) -> list[Detection]:
    """Keep detections the confirmer did not veto (confirmed or failopen)."""
    fused = []
    for d in detections:
        res = ir_results[id(d)]
        if res.decision != 'vetoed':
            fused.append(Detection(d.t_mono_ns, d.source + '+ir', d.tid,
                                   {**d.extra, 'ir_decision': res.decision}))
    return fused


# ── session-level driver ─────────────────────────────────────────────────────

GT_MERGE_GAP_NS = int(0.5 * NS)     # max gap between fall and its lying tail


def build_gt_events(manifest: Manifest) -> list[GtEvent]:
    """Ground-truth fall events for matching.

    A 'fall' span merged with any immediately following 'fall_lying' span(s):
    a late detection while the person is still on the floor (e.g. Tier-2
    faint) is a correct alarm for that fall, not a false alarm. Latency is
    still measured from the fall (impact) start by match_events.
    """
    spans = sorted(manifest.labels, key=lambda s: s.t_start_mono_ns)
    events: list[GtEvent] = []
    for s in spans:
        if s.label == 'fall':
            events.append(GtEvent(s.t_start_mono_ns, s.t_end_mono_ns, 'fall'))
        elif (s.label == 'fall_lying' and events
              and s.t_start_mono_ns - events[-1].t_end_mono_ns <= GT_MERGE_GAP_NS):
            events[-1].t_end_mono_ns = max(events[-1].t_end_mono_ns,
                                           s.t_end_mono_ns)
    return events


def replay_session(session_dir: str, ml_model: str | None = None,
                   ml_threshold: float = 0.5,
                   tol_pre_s: float = 1.0, tol_post_s: float = 2.0,
                   ir_params: IrConfirmerParams | None = None,
                   ir_calib_dir: str | None = None,
                   ir_rotate: int = 0) -> dict:
    ir_rotate_k = (ir_rotate // 90) % 4
    sess = load_session(session_dir, ir_rotate_k=ir_rotate_k)

    ir_background = None
    if ir_calib_dir:
        ir_background = load_background_from_session(ir_calib_dir, ir_params,
                                                     ir_rotate_k=ir_rotate_k)

    duration_s = sess.manifest.duration_s
    if duration_s <= 0 and len(sess.radar_df):
        duration_s = (int(sess.radar_df['t_mono_ns'].iloc[-1])
                      - int(sess.radar_df['t_mono_ns'].iloc[0])) / NS

    gt_events = build_gt_events(sess.manifest)

    def metrics(dets):
        return compute_event_metrics(gt_events, dets, duration_s,
                                     tol_pre_s, tol_post_s)

    detectors = {}

    rules_dets = replay_rules(sess.radar_df)
    detectors['rules'] = {'detections': rules_dets, 'metrics': metrics(rules_dets)}

    ml_dets = None
    if ml_model:
        ml_dets = replay_ml(sess.radar_df, ml_model, threshold=ml_threshold)
        detectors['ml'] = {'detections': ml_dets, 'metrics': metrics(ml_dets)}

    ir_summary = None
    if sess.thermal_frames is not None:
        all_dets = rules_dets + (ml_dets or [])
        ir = replay_ir(sess, all_dets, ir_params, background=ir_background)
        if ir is not None:
            detectors['rules+ir'] = _fused_entry(rules_dets, ir, metrics)
            if ml_dets is not None:
                detectors['ml+ir'] = _fused_entry(ml_dets, ir, metrics)
            ir_summary = {
                'calibrated': ir['calibrated'],
                'evaluations': [
                    {'t_mono_ns': d.t_mono_ns, 'source': d.source,
                     'decision': ir['results'][id(d)].decision,
                     'reason': ir['results'][id(d)].reason}
                    for d in all_dets
                ],
            }

    return {
        'session_id': sess.manifest.session_id,
        'session_dir': session_dir,
        'subject': sess.manifest.subject,
        'duration_s': duration_s,
        'n_gt_fall_events': len(gt_events),
        'detectors': detectors,
        'ir': ir_summary,
    }


def _fused_entry(base_dets, ir, metrics_fn):
    fused = fuse(base_dets, ir['results'])
    return {'detections': fused, 'metrics': metrics_fn(fused)}


# ── reporting ────────────────────────────────────────────────────────────────

def _fmt(v, spec='.2f'):
    return 'n/a' if v is None else format(v, spec)


def print_report(results: list[dict]):
    for r in results:
        print(f"\n=== {r['session_id']}  ({r['duration_s']:.0f} s, "
              f"{r['n_gt_fall_events']} GT fall events) ===")
        header = (f"{'detector':<10} {'events':>6} {'detected':>8} "
                  f"{'recall':>7} {'FA':>4} {'FA/h':>7} {'lat_med':>8}")
        print(header)
        for name, entry in r['detectors'].items():
            m = entry['metrics']
            print(f"{name:<10} {m['n_events']:>6} {m['n_detected']:>8} "
                  f"{_fmt(m['event_recall']):>7} {m['n_false_alarms']:>4} "
                  f"{m['fa_per_hour']:>7.2f} {_fmt(m['latency_median_s']):>7}s")
        if r['ir'] is not None:
            cal = 'calibrated' if r['ir']['calibrated'] else 'NOT calibrated (failopen)'
            print(f"IR confirmer: {cal}")
            for ev in r['ir']['evaluations']:
                print(f"  eval @{ev['t_mono_ns'] / NS:9.2f}s [{ev['source']}] "
                      f"-> {ev['decision']}  ({ev['reason']})")


def _jsonable(results: list[dict]) -> list:
    out = []
    for r in results:
        rr = dict(r)
        rr['detectors'] = {}
        for name, entry in r['detectors'].items():
            m = dict(entry['metrics'])
            m['missed_events'] = [
                {'t_start_mono_ns': e.t_start_mono_ns,
                 't_end_mono_ns': e.t_end_mono_ns} for e in m['missed_events']]
            m['false_alarm_detections'] = [
                {'t_mono_ns': d.t_mono_ns, 'tid': d.tid, **d.extra}
                for d in m['false_alarm_detections']]
            rr['detectors'][name] = {
                'detections': [{'t_mono_ns': d.t_mono_ns, 'source': d.source,
                                'tid': d.tid, **d.extra}
                               for d in entry['detections']],
                'metrics': m,
            }
        out.append(rr)
    return out


def main():
    ap = argparse.ArgumentParser(
        description='Offline replay of recorded sessions with event-level metrics.')
    ap.add_argument('sessions', nargs='+',
                    help='Session directories (globs allowed)')
    ap.add_argument('--ml-model', default=None,
                    help='Trained model (.pkl) to replay alongside the rules')
    ap.add_argument('--ml-threshold', type=float, default=0.5)
    ap.add_argument('--tol-pre', type=float, default=1.0,
                    help='Match tolerance before GT event start (s)')
    ap.add_argument('--tol-post', type=float, default=2.0,
                    help='Match tolerance after GT event end (s)')
    ap.add_argument('--ir-calib', default=None,
                    help='Empty-room calibration session dir; builds the IR '
                         'background from its thermal.npz instead of '
                         'per-session auto-calibration')
    ap.add_argument('--ir-rotate', type=int, default=90,
                    choices=[0, 90, 180, 270],
                    help='IR mounting rotation in degrees (default: 90, '
                         'matches current mount) — must match the --ir-rotate '
                         'used in radar_reader live mode')
    ap.add_argument('--json', default=None, help='Write results to JSON file')
    args = ap.parse_args()

    dirs = []
    for pattern in args.sessions:
        expanded = sorted(glob.glob(pattern)) or [pattern]
        dirs.extend(d for d in expanded if os.path.isdir(d))
    if not dirs:
        ap.error('no session directories found')

    results = []
    for d in dirs:
        try:
            results.append(replay_session(d, ml_model=args.ml_model,
                                          ml_threshold=args.ml_threshold,
                                          tol_pre_s=args.tol_pre,
                                          tol_post_s=args.tol_post,
                                          ir_calib_dir=args.ir_calib,
                                          ir_rotate=args.ir_rotate))
        except FileNotFoundError as e:
            print(f'[skip] {d}: {e}')

    print_report(results)

    if args.json:
        with open(args.json, 'w', encoding='utf-8') as f:
            json.dump(_jsonable(results), f, indent=2)
        print(f'\nResults written to {args.json}')


if __name__ == '__main__':
    main()
