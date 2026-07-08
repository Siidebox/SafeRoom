"""Clock injection tests — detectors must honor an injected clock so
offline replay (tools/replay_session.py) can drive cooldowns with session
time instead of wall time."""
import joblib
import numpy as np

from radar_reader import FallDetector
from ml_inference import MlFallDetector


class FakeClock:
    def __init__(self, t0=1000.0):
        self.t = t0

    def __call__(self):
        return self.t


class DummyAlwaysFallModel:
    """Sklearn-like model that always predicts fall with p=0.9."""
    def predict_proba(self, x):
        return np.array([[0.1, 0.9]] * len(x))


def _feed_fall(fd, tid=1, n_warmup=20, n_fall=3):
    """Warm up a track standing, then feed a fast-fall vz burst.
    Returns True if any update reported a fall."""
    fell = False
    for _ in range(n_warmup):
        fall, _ = fd.update(tid, {'maxZ': 1.5, 'minZ': 0.2}, {'vz': 0.0})
        fell = fell or fall
    for _ in range(n_fall):
        fall, _ = fd.update(tid, {'maxZ': 0.9, 'minZ': 0.1}, {'vz': -1.5})
        fell = fell or fall
    return fell


class TestFallDetectorClock:
    def test_cooldown_follows_injected_clock(self):
        clock = FakeClock()
        fd = FallDetector(clock=clock)

        assert _feed_fall(fd) is True

        # Immediately after: cooldown active under injected time
        assert _feed_fall(fd, n_warmup=0, n_fall=5) is False

        # Advance session time beyond COOLDOWN -> detects again
        clock.t += FallDetector.COOLDOWN + 1.0
        assert _feed_fall(fd, n_warmup=0, n_fall=5) is True


class TestMlFallDetectorClock:
    def test_cooldown_follows_injected_clock(self, tmp_path):
        model_path = tmp_path / 'dummy.pkl'
        joblib.dump(DummyAlwaysFallModel(), model_path)

        clock = FakeClock()
        det = MlFallDetector(str(model_path), window_size=5, stride=1,
                             threshold=0.5, cooldown=5.0, clock=clock)

        track = {'x': 0, 'y': 2, 'z': 1, 'vx': 0, 'vy': 0, 'vz': -1.5,
                 'ax': 0, 'ay': 0, 'az': -5, 'g': 1, 'confidence': 1}
        height = {'maxZ': 0.9, 'minZ': 0.1}

        results = [det.update(1, track, height, {})[0] for _ in range(5)]
        assert results[-1] is True          # first prediction fires

        # Same session time -> cooldown blocks
        assert det.update(1, track, height, {})[0] is False

        # Advance injected clock past cooldown -> fires again
        clock.t += 6.0
        assert det.update(1, track, height, {})[0] is True
