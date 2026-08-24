"""Rule-based two-tier fall detector.

Thresholds were calibrated on labelled hardware sessions recorded in the
deployment room; see ``docs/limitations.md`` for what the two tiers can and
cannot distinguish.
"""

import collections
import time

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

    def __init__(self, frame_period_s: float = 0.05, clock=None):
        # clock: callable returning seconds — injectable so offline replay
        # (replay_session.py) can drive cooldowns with session time.
        self._clock        = clock if clock is not None else time.time
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
            # -inf = "never detected": with an injected session clock that
            # starts near 0, initializing to 0.0 would leave the cooldown
            # active for the first COOLDOWN seconds of the session.
            self._last_det[tid]     = float('-inf')
            self._last_faint[tid]   = float('-inf')
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
        now   = self._clock()
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
