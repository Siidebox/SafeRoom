"""
event_metrics.py — Window/frame detections → event-level metrics.

Fall detection quality must be reported per EVENT, not per window: a single
1 s fall produces many positive windows, and window-level precision/recall
does not translate to "false alarms per hour" for a caregiver.

This module matches discrete detector firings against ground-truth
LabelSpans (manifest.json) and computes:
    - event recall        (fraction of GT events with >= 1 detection)
    - false alarms / hour (unmatched detections over session duration)
    - detection latency   (detection time - event start), median

Used by tools/replay_session.py; pure functions, no I/O.
"""

import statistics
from dataclasses import dataclass, field


@dataclass
class GtEvent:
    """Ground-truth event from a manifest LabelSpan."""
    t_start_mono_ns: int
    t_end_mono_ns: int
    label: str = "fall"


@dataclass
class Detection:
    """A discrete detector firing (rules tier-1/2, ML threshold crossing...)."""
    t_mono_ns: int
    source: str = ""          # "rules" | "ml" | "ml+ir" ...
    tid: int = -1
    extra: dict = field(default_factory=dict)


@dataclass
class EventMatch:
    event: GtEvent
    detection: Detection | None = None     # None => missed
    latency_s: float | None = None         # detection - event start


def match_events(gt_events: list[GtEvent],
                 detections: list[Detection],
                 tol_pre_s: float = 1.0,
                 tol_post_s: float = 2.0) -> tuple[list[EventMatch], list[Detection]]:
    """
    Match detections to ground-truth events.

    A detection matches an event if it falls inside
    [t_start - tol_pre_s, t_end + tol_post_s]. Each event takes its earliest
    in-window detection; every detection inside the window of an already
    matched event is consumed (not a false alarm — e.g. cooldown re-fires).
    A detection can be consumed by only one event (earliest event wins).

    Returns (matches ordered like gt_events, false_alarms).
    """
    pre_ns  = int(tol_pre_s * 1e9)
    post_ns = int(tol_post_s * 1e9)

    events = sorted(gt_events, key=lambda e: e.t_start_mono_ns)
    dets   = sorted(detections, key=lambda d: d.t_mono_ns)

    consumed = set()                      # indices into dets
    matches_by_event = {}

    for ev in events:
        lo = ev.t_start_mono_ns - pre_ns
        hi = ev.t_end_mono_ns + post_ns
        first = None
        for i, d in enumerate(dets):
            if i in consumed:
                continue
            if lo <= d.t_mono_ns <= hi:
                consumed.add(i)
                if first is None:
                    first = d
        if first is not None:
            lat = (first.t_mono_ns - ev.t_start_mono_ns) / 1e9
            matches_by_event[id(ev)] = EventMatch(ev, first, lat)
        else:
            matches_by_event[id(ev)] = EventMatch(ev, None, None)

    matches = [matches_by_event[id(ev)] for ev in gt_events]
    false_alarms = [d for i, d in enumerate(dets) if i not in consumed]
    return matches, false_alarms


def compute_event_metrics(gt_events: list[GtEvent],
                          detections: list[Detection],
                          duration_s: float,
                          tol_pre_s: float = 1.0,
                          tol_post_s: float = 2.0) -> dict:
    """Aggregate matches into the metrics dict used for reporting."""
    matches, false_alarms = match_events(gt_events, detections,
                                         tol_pre_s, tol_post_s)
    latencies = [m.latency_s for m in matches if m.latency_s is not None]
    n_events = len(gt_events)
    n_detected = len(latencies)

    return {
        "n_events": n_events,
        "n_detected": n_detected,
        "event_recall": (n_detected / n_events) if n_events else None,
        "n_false_alarms": len(false_alarms),
        "fa_per_hour": (len(false_alarms) / (duration_s / 3600.0))
                       if duration_s > 0 else 0.0,
        "latencies_s": latencies,
        "latency_median_s": statistics.median(latencies) if latencies else None,
        "missed_events": [m.event for m in matches if m.detection is None],
        "false_alarm_detections": false_alarms,
    }
