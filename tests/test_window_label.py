"""Tests for the window-labeling rule (M3) and fall_lying handling (M4).

Decisions (2026-07-08, pre-dataset):
- A window is positive only if >= min_fall_frames rows are labeled 'fall'
  (any-overlap labeling made a window with 1/30 fall frames as positive as
  one centered on the impact).
- Windows touching 'fall' or 'fall_lying' without reaching the positive
  threshold are EXCLUDED (label None) — boundary/post-impact ambiguity must
  not become noisy positives or contradictory negatives.
- Falling back to the rule-based 'fall_detected' column as ground truth is
  forbidden (baseline->ML label leakage).
"""
import pandas as pd
import pytest

from feature_engineering import _window_label


def _win(labels):
    return pd.DataFrame({'label': labels, 'vz': [0.0] * len(labels)})


class TestWindowLabel:
    def test_positive_when_enough_fall_frames(self):
        win = _win(['walk'] * 20 + ['fall'] * 10)
        assert _window_label(win, min_fall_frames=5) == 1

    def test_excluded_when_too_few_fall_frames(self):
        win = _win(['walk'] * 28 + ['fall'] * 2)
        assert _window_label(win, min_fall_frames=5) is None

    def test_negative_when_no_fall(self):
        win = _win(['walk'] * 30)
        assert _window_label(win, min_fall_frames=5) == 0

    def test_fall_lying_windows_are_excluded(self):
        win = _win(['fall_lying'] * 30)
        assert _window_label(win, min_fall_frames=5) is None

    def test_fall_plus_lying_positive_if_enough_fall(self):
        win = _win(['fall'] * 6 + ['fall_lying'] * 24)
        assert _window_label(win, min_fall_frames=5) == 1

    def test_rule_based_fallback_is_forbidden(self):
        win = pd.DataFrame({'fall_detected': [1] * 30})
        with pytest.raises(ValueError, match='fall_detected'):
            _window_label(win)


class TestLabelerVocabulary:
    def test_fall_lying_key_exists(self):
        from label_session_multimodal import KEY_TO_LABEL, LABEL_COLORS
        assert 'fall_lying' in KEY_TO_LABEL.values()
        assert 'fall_lying' in LABEL_COLORS
