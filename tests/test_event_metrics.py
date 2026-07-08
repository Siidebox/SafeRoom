"""Tests for tools/event_metrics.py — window→event aggregation and metrics."""
import pytest

from event_metrics import GtEvent, Detection, match_events, compute_event_metrics

NS = int(1e9)


def _gt(start_s, end_s, label="fall"):
    return GtEvent(t_start_mono_ns=int(start_s * NS),
                   t_end_mono_ns=int(end_s * NS),
                   label=label)


def _det(t_s, source="rules", tid=1):
    return Detection(t_mono_ns=int(t_s * NS), source=source, tid=tid)


class TestMatchEvents:
    def test_detection_inside_event_matches(self):
        gt = [_gt(10.0, 11.0)]
        dets = [_det(10.5)]
        matches, false_alarms = match_events(gt, dets)
        assert len(matches) == 1
        assert matches[0].event is gt[0]
        assert matches[0].detection is dets[0]
        assert false_alarms == []

    def test_detection_far_from_event_is_false_alarm(self):
        gt = [_gt(10.0, 11.0)]
        dets = [_det(30.0)]
        matches, false_alarms = match_events(gt, dets)
        assert matches[0].detection is None      # event missed
        assert false_alarms == [dets[0]]

    def test_detection_within_post_tolerance_matches(self):
        gt = [_gt(10.0, 11.0)]
        dets = [_det(12.5)]                      # 1.5 s after label end
        matches, _ = match_events(gt, dets, tol_post_s=2.0)
        assert matches[0].detection is dets[0]

    def test_detection_within_pre_tolerance_matches(self):
        gt = [_gt(10.0, 11.0)]
        dets = [_det(9.5)]                       # 0.5 s before label start
        matches, _ = match_events(gt, dets, tol_pre_s=1.0)
        assert matches[0].detection is dets[0]

    def test_earliest_detection_wins_and_extras_ignored(self):
        # Multiple detections of the same event (e.g. cooldown expiry) should
        # count once, matching the earliest, and NOT become false alarms.
        gt = [_gt(10.0, 15.0)]
        dets = [_det(10.4), _det(13.0)]
        matches, false_alarms = match_events(gt, dets)
        assert matches[0].detection is dets[0]
        assert false_alarms == []

    def test_detection_matches_only_one_event(self):
        gt = [_gt(10.0, 11.0), _gt(11.5, 12.5)]
        dets = [_det(11.2)]                      # in tolerance of both
        matches, false_alarms = match_events(gt, dets, tol_pre_s=1.0, tol_post_s=2.0)
        matched = [m for m in matches if m.detection is not None]
        assert len(matched) == 1
        assert false_alarms == []

    def test_latency_measured_from_event_start(self):
        gt = [_gt(10.0, 15.0)]
        dets = [_det(10.8)]
        matches, _ = match_events(gt, dets)
        assert matches[0].latency_s == pytest.approx(0.8)


class TestComputeEventMetrics:
    def test_perfect_detection(self):
        gt = [_gt(10.0, 11.0), _gt(40.0, 41.0)]
        dets = [_det(10.2), _det(40.3)]
        m = compute_event_metrics(gt, dets, duration_s=3600.0)
        assert m["n_events"] == 2
        assert m["n_detected"] == 2
        assert m["event_recall"] == 1.0
        assert m["n_false_alarms"] == 0
        assert m["fa_per_hour"] == 0.0
        assert m["latency_median_s"] == pytest.approx(0.25)

    def test_missed_event_and_false_alarm(self):
        gt = [_gt(10.0, 11.0), _gt(40.0, 41.0)]
        dets = [_det(10.2), _det(100.0)]
        m = compute_event_metrics(gt, dets, duration_s=1800.0)  # 0.5 h
        assert m["n_detected"] == 1
        assert m["event_recall"] == 0.5
        assert m["n_false_alarms"] == 1
        assert m["fa_per_hour"] == pytest.approx(2.0)

    def test_no_events_no_detections(self):
        m = compute_event_metrics([], [], duration_s=600.0)
        assert m["n_events"] == 0
        assert m["event_recall"] is None
        assert m["fa_per_hour"] == 0.0
        assert m["latency_median_s"] is None
