# sanad_perception/tracker.py
import numpy as np
from scipy.optimize import linear_sum_assignment
from dataclasses import dataclass, field
from typing import List, Optional


# ── Single track ─────────────────────────────────────────────────────────────

@dataclass
class Track:
    """
    Constant-velocity Kalman filter for one detected person.

    State vector:  x = [px, py, vx, vy]ᵀ   (position + velocity in XY)
    Measurement:   z = [px, py]ᵀ

    Matrices are class-level constants so every Track shares them —
    override per-instance if you need different noise levels.
    """

    track_id : int
    hits     : int = 0          # confirmed observations
    misses   : int = 0          # consecutive frames with no match
    age      : int = 0          # total frames since creation

    # Kalman state and covariance — initialised in __post_init__
    x : np.ndarray = field(default_factory=lambda: np.zeros(4))
    P : np.ndarray = field(
        default_factory=lambda: np.diag([1.0, 1.0, 5.0, 5.0]))

    # ── Constant-velocity process model ──────────────────────────────────────
    # F and H are the same for every track, so define them as class attributes
    H = np.array([[1, 0, 0, 0],
                  [0, 1, 0, 0]], dtype=np.float64)

    # Process noise — tune Q_vel_var if velocity estimates are noisy
    Q_pos_var : float = 0.02    # position process noise variance
    Q_vel_var : float = 0.10    # velocity process noise variance

    # Measurement noise — tune R_var if sensor is noisy
    R_var     : float = 0.05

    # ── Kalman predict ────────────────────────────────────────────────────────

    def predict(self, dt: float):
        """
        Project state forward by dt seconds.
        Call this once per frame for every track, matched or not.
        """
        F = np.array([[1, 0, dt, 0],
                      [0, 1, 0,  dt],
                      [0, 0, 1,  0],
                      [0, 0, 0,  1]], dtype=np.float64)

        Q = np.diag([self.Q_pos_var, self.Q_pos_var,
                     self.Q_vel_var, self.Q_vel_var])

        self.x = F @ self.x
        self.P = F @ self.P @ F.T + Q
        self.age += 1

    # ── Kalman update ─────────────────────────────────────────────────────────

    def update(self, z: np.ndarray):
        """
        Correct state with measurement z = [px, py].
        Call only when this track is matched to a detection.
        """
        R = np.eye(2) * self.R_var
        S = self.H @ self.P @ self.H.T + R          # innovation covariance
        K = self.P @ self.H.T @ np.linalg.inv(S)    # Kalman gain

        innovation = z - self.H @ self.x
        self.x = self.x + K @ innovation
        self.P = (np.eye(4) - K @ self.H) @ self.P

        self.hits   += 1
        self.misses  = 0

    # ── Convenience properties ────────────────────────────────────────────────

    @property
    def position(self) -> np.ndarray:
        """Estimated [px, py] in metres."""
        return self.x[:2].copy()

    @property
    def velocity(self) -> np.ndarray:
        """Estimated [vx, vy] in m/s."""
        return self.x[2:].copy()

    @property
    def speed(self) -> float:
        return float(np.linalg.norm(self.velocity))

    def mahalanobis_dist(self, z: np.ndarray) -> float:
        """
        Statistical distance between measurement z and this track's prediction.
        Used as the cost in the assignment matrix — more principled than
        plain Euclidean because it accounts for prediction uncertainty.
        """
        R    = np.eye(2) * self.R_var
        S    = self.H @ self.P @ self.H.T + R
        diff = z - self.H @ self.x
        return float(np.sqrt(diff @ np.linalg.inv(S) @ diff))

    def __repr__(self):
        p = self.position
        return (f"Track(id={self.track_id}, hits={self.hits}, "
                f"misses={self.misses}, pos=[{p[0]:.2f},{p[1]:.2f}], "
                f"spd={self.speed:.2f}m/s)")


# ── Track manager ─────────────────────────────────────────────────────────────

class TrackManager:
    """
    Manages a set of Tracks across frames.

    Lifecycle
    ---------
    Tentative  → hits < MIN_HITS         (not yet published upstream)
    Confirmed  → hits >= MIN_HITS        (published to /tracked_human)
    Dead       → misses >= MAX_MISSES    (deleted)

    Association
    -----------
    Hungarian algorithm on Mahalanobis distance matrix.
    A match is only accepted if cost < GATE_MAHAL (gates out ambiguous
    matches at the edges of a crowd).
    """

    MIN_HITS    : int   = 3      # frames needed to confirm a new track
    MAX_MISSES  : int   = 8      # consecutive misses before deletion
    GATE_MAHAL  : float = 4.0    # max Mahalanobis distance for valid match
    GATE_EUCL   : float = 2.0    # hard Euclidean gate (m) as a second guard

    def __init__(self):
        self._tracks   : List[Track] = []
        self._next_id  : int         = 0
        self.target_id : Optional[int] = None

    # ── Main update — call once per detection frame ───────────────────────────

    def update(self, detections: List[np.ndarray],
               dt: float) -> List[Track]:
        """
        Parameters
        ----------
        detections : list of np.ndarray([px, py])
        dt         : seconds since last call

        Returns
        -------
        List of confirmed tracks (hits >= MIN_HITS).
        """
        dt = np.clip(dt, 1e-3, 1.0)   # guard against zero or huge dt

        # 1. Predict every track forward
        for t in self._tracks:
            t.predict(dt)

        # 2. Associate detections → tracks
        unmatched_dets = set(range(len(detections)))
        if self._tracks and detections:
            unmatched_dets = self._associate(detections)

        # 3. Spawn new tracks for unmatched detections
        for i in unmatched_dets:
            t = Track(track_id=self._next_id)
            t.x[:2] = detections[i]
            self._next_id += 1
            self._tracks.append(t)

        # 4. Increment misses for unmatched tracks
        #    (already done inside _associate for matched ones)

        # 5. Prune dead tracks
        before = {t.track_id for t in self._tracks}
        self._tracks = [t for t in self._tracks
                        if t.misses < self.MAX_MISSES]
        removed = before - {t.track_id for t in self._tracks}
        if self.target_id in removed:
            self.target_id = None   # target was lost — re-acquire

        # 6. Select / maintain target
        self._update_target()

        confirmed = [t for t in self._tracks if t.hits >= self.MIN_HITS]
        return confirmed

    # ── Hungarian association ─────────────────────────────────────────────────

    def _associate(self, detections: List[np.ndarray]) -> set:
        """
        Returns set of detection indices that were NOT matched to any track.
        Side-effect: calls track.update() for matched pairs,
                     increments track.misses for unmatched tracks.
        """
        n_trk = len(self._tracks)
        n_det = len(detections)

        # Build cost matrix (n_tracks × n_detections)
        cost = np.full((n_trk, n_det), fill_value=1e6)
        for r, trk in enumerate(self._tracks):
            for c, det in enumerate(detections):
                eucl = np.linalg.norm(trk.position - det)
                if eucl < self.GATE_EUCL:             # skip obviously far pairs
                    cost[r, c] = trk.mahalanobis_dist(det)

        row_ind, col_ind = linear_sum_assignment(cost)

        matched_trks = set()
        matched_dets = set()

        for r, c in zip(row_ind, col_ind):
            if cost[r, c] < self.GATE_MAHAL:
                self._tracks[r].update(detections[c])
                matched_trks.add(r)
                matched_dets.add(c)

        # Unmatched tracks get a miss
        for r, trk in enumerate(self._tracks):
            if r not in matched_trks:
                trk.misses += 1

        return set(range(n_det)) - matched_dets

    # ── Target selection ──────────────────────────────────────────────────────

    def _update_target(self):
        confirmed = [t for t in self._tracks if t.hits >= self.MIN_HITS]
        if not confirmed:
            self.target_id = None
            return

        # Keep existing target if it is still alive
        if self.target_id is not None:
            if any(t.track_id == self.target_id for t in confirmed):
                return    # target still alive — keep it

        # Re-acquire: pick closest confirmed track to the robot (origin)
        self.target_id = min(
            confirmed,
            key=lambda t: np.linalg.norm(t.position)
        ).track_id

    # ── Public helpers ────────────────────────────────────────────────────────

    def get_target(self) -> Optional[Track]:
        return next(
            (t for t in self._tracks
             if t.track_id == self.target_id and t.hits >= self.MIN_HITS),
            None)

    def force_target(self, track_id: int):
        """Let an operator manually lock onto a specific track ID."""
        if any(t.track_id == track_id for t in self._tracks):
            self.target_id = track_id

    @property
    def all_tracks(self) -> List[Track]:
        return list(self._tracks)

    @property
    def confirmed_tracks(self) -> List[Track]:
        return [t for t in self._tracks if t.hits >= self.MIN_HITS]