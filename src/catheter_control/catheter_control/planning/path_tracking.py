"""Pure geometry and progress logic for continuous catheter tip paths."""
from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np


def _finite_vector(name, value, size):
    result = np.asarray(value, dtype=np.float64)
    if result.shape != (size,) or not np.all(np.isfinite(result)):
        raise ValueError(f"{name} must contain {size} finite values")
    return result


def _pchip_derivatives(x: np.ndarray, y: np.ndarray) -> np.ndarray:
    """Fritsch-Carlson derivatives for one scalar coordinate."""
    h = np.diff(x)
    delta = np.diff(y)/h
    count = len(x)
    result = np.zeros(count, dtype=np.float64)
    if count == 2:
        result[:] = delta[0]
        return result
    for index in range(1, count-1):
        left, right = delta[index-1], delta[index]
        if left == 0.0 or right == 0.0 or np.sign(left) != np.sign(right):
            result[index] = 0.0
        else:
            left_weight = 2.0*h[index]+h[index-1]
            right_weight = h[index]+2.0*h[index-1]
            result[index] = (left_weight+right_weight)/(
                left_weight/left+right_weight/right)

    def endpoint(first_h, second_h, first_delta, second_delta):
        slope = ((2.0*first_h+second_h)*first_delta
                 - first_h*second_delta)/(first_h+second_h)
        if np.sign(slope) != np.sign(first_delta):
            return 0.0
        if (np.sign(first_delta) != np.sign(second_delta)
                and abs(slope) > 3.0*abs(first_delta)):
            return 3.0*first_delta
        return slope

    result[0] = endpoint(h[0], h[1], delta[0], delta[1])
    result[-1] = endpoint(h[-1], h[-2], delta[-1], delta[-2])
    return result


class ArcLengthPath:
    """C1 coordinate-wise PCHIP parameterized by knot chord length."""

    def __init__(self, knots_m, *, closest_samples_per_segment: int = 24):
        knots = np.asarray(knots_m, dtype=np.float64)
        if (knots.ndim != 2 or knots.shape[1:] != (3,)
                or len(knots) < 2 or not np.all(np.isfinite(knots))):
            raise ValueError("path knots must be a finite Nx3 array, N >= 2")
        chord = np.linalg.norm(np.diff(knots, axis=0), axis=1)
        if np.any(chord <= 1e-9):
            raise ValueError("consecutive path knots must be distinct")
        if closest_samples_per_segment < 4:
            raise ValueError("closest_samples_per_segment must be at least 4")
        self.knots = knots.copy()
        self.arc_knots = np.concatenate(([0.0], np.cumsum(chord)))
        self.length_m = float(self.arc_knots[-1])
        self.derivatives = np.column_stack([
            _pchip_derivatives(self.arc_knots, knots[:, axis])
            for axis in range(3)
        ])
        dense_s = []
        for index in range(len(knots)-1):
            values = np.linspace(
                self.arc_knots[index], self.arc_knots[index+1],
                closest_samples_per_segment, endpoint=False)
            dense_s.extend(values.tolist())
        dense_s.append(self.length_m)
        self._closest_s = np.asarray(dense_s, dtype=np.float64)
        self._closest_points, _ = self.evaluate(self._closest_s)

    def evaluate(self, arc_m):
        values = np.asarray(arc_m, dtype=np.float64)
        if not np.all(np.isfinite(values)):
            raise ValueError("arc coordinates must be finite")
        clipped = np.clip(values, 0.0, self.length_m)
        flat = clipped.reshape(-1)
        indices = np.searchsorted(
            self.arc_knots, flat, side="right")-1
        indices = np.clip(indices, 0, len(self.knots)-2)
        start = self.arc_knots[indices]
        span = self.arc_knots[indices+1]-start
        t = (flat-start)/span
        t2, t3 = t*t, t*t*t
        h00 = 2*t3-3*t2+1
        h10 = t3-2*t2+t
        h01 = -2*t3+3*t2
        h11 = t3-t2
        points = (
            h00[:, None]*self.knots[indices]
            + h10[:, None]*span[:, None]*self.derivatives[indices]
            + h01[:, None]*self.knots[indices+1]
            + h11[:, None]*span[:, None]*self.derivatives[indices+1])
        dh00 = (6*t2-6*t)/span
        dh10 = 3*t2-4*t+1
        dh01 = (-6*t2+6*t)/span
        dh11 = 3*t2-2*t
        derivatives = (
            dh00[:, None]*self.knots[indices]
            + dh10[:, None]*self.derivatives[indices]
            + dh01[:, None]*self.knots[indices+1]
            + dh11[:, None]*self.derivatives[indices+1])
        norms = np.linalg.norm(derivatives, axis=1)
        fallback = self.knots[indices+1]-self.knots[indices]
        fallback /= np.linalg.norm(fallback, axis=1)[:, None]
        tangents = np.divide(
            derivatives, norms[:, None], out=fallback.copy(),
            where=norms[:, None] > 1e-12)
        output_shape = values.shape+(3,)
        return points.reshape(output_shape), tangents.reshape(output_shape)

    def _closest_in_arc_interval(self, point_m, lower_m, upper_m):
        point = _finite_vector("point", point_m, 3)
        lower = float(lower_m)
        upper = float(upper_m)
        if (not math.isfinite(lower) or not math.isfinite(upper)
                or lower < 0.0 or upper > self.length_m or lower > upper):
            raise ValueError("closest arc interval is invalid")
        if upper-lower <= 1e-12:
            exact, tangent = self.evaluate(lower)
            error = point-exact
            along = float(np.dot(error, tangent))
            cross = error-along*tangent
            return PathProjection(
                arc_m=lower, point_m=exact, tangent=tangent,
                distance_mm=1000.0*float(np.linalg.norm(error)),
                along_error_mm=1000.0*along,
                cross_error_mm=1000.0*float(np.linalg.norm(cross)))
        segment_start_s = np.maximum(self._closest_s[:-1], lower)
        segment_end_s = np.minimum(self._closest_s[1:], upper)
        valid = segment_end_s > segment_start_s
        segment_start_s = segment_start_s[valid]
        segment_end_s = segment_end_s[valid]
        starts, _ = self.evaluate(segment_start_s)
        ends, _ = self.evaluate(segment_end_s)
        segments = ends-starts
        denominator = np.square(segments).sum(axis=1)
        # Clipping a sampling interval near a zero-derivative turnaround can
        # collapse its geometric chord to one floating-point position. Treat
        # it as a point projection instead of dividing by zero.
        fractions = np.clip(np.divide(
            ((point-starts)*segments).sum(axis=1), denominator,
            out=np.zeros_like(denominator), where=denominator > 0.0),
            0.0, 1.0)
        projected = starts+fractions[:, None]*segments
        squared = np.square(projected-point).sum(axis=1)
        index = int(np.argmin(squared))
        arc = segment_start_s[index]+fractions[index]*(
            segment_end_s[index]-segment_start_s[index])
        exact, tangent = self.evaluate(arc)
        error = point-exact
        along = float(np.dot(error, tangent))
        cross = error-along*tangent
        return PathProjection(
            arc_m=float(arc), point_m=exact, tangent=tangent,
            distance_mm=1000.0*float(np.linalg.norm(error)),
            along_error_mm=1000.0*along,
            cross_error_mm=1000.0*float(np.linalg.norm(cross)))

    def closest(self, point_m):
        """Return the globally closest geometric path projection."""
        return self._closest_in_arc_interval(point_m, 0.0, self.length_m)

    def closest_near(self, point_m, arc_hint_m, half_window_m):
        """Return the closest projection on the branch near ``arc_hint_m``.

        Geometrically closed or self-intersecting paths can have multiple
        equally close projections.  Control phase must remain on the branch
        continuous with the monotonic governed arc rather than jumping to a
        coincident point elsewhere on the path.
        """
        hint = float(arc_hint_m)
        window = float(half_window_m)
        if (not math.isfinite(hint) or not 0.0 <= hint <= self.length_m
                or not math.isfinite(window) or window <= 0.0):
            raise ValueError("closest-near hint/window is invalid")
        return self._closest_in_arc_interval(
            point_m, max(0.0, hint-window),
            min(self.length_m, hint+window))


@dataclass(frozen=True)
class PathProjection:
    arc_m: float
    point_m: np.ndarray
    tangent: np.ndarray
    distance_mm: float
    along_error_mm: float
    cross_error_mm: float


class OrderedPathCoverage:
    """Observation-backed traversal, independent of the reference clock.

    Visit dense stations in order, inside the endpoint-tolerance tube and
    across each station's forward tangent plane. A stationary tip cannot
    accumulate credit merely by lying within several station radii. Only
    consecutive fresh observations are joined; gaps and duplicate samples
    never manufacture a traversed segment. This is a tolerance-based sampled
    traversal metric, not a claim about the unobserved continuous trajectory.
    """

    def __init__(self, path, tolerance_mm, maximum_gap_s=.5):
        self.path = path
        self.tolerance_m = 1e-3*float(tolerance_mm)
        self.maximum_gap_s = float(maximum_gap_s)
        if (not math.isfinite(self.tolerance_m) or self.tolerance_m <= 0
                or not math.isfinite(self.maximum_gap_s)
                or self.maximum_gap_s <= 0):
            raise ValueError("coverage tolerance/gap must be finite and positive")
        spacing = min(self.tolerance_m/2, .0005)
        self.arcs = np.linspace(0, path.length_m,
                                int(np.ceil(path.length_m/spacing))+1)
        self.points, self.tangents = path.evaluate(self.arcs)
        self.next_station = 0
        self.covered_m = 0.0
        self.previous_tip = None
        self.previous_time = None

    @property
    def complete(self):
        return self.next_station == len(self.arcs)

    def update(self, tip_m, observation_time):
        tip = _finite_vector("coverage tip", tip_m, 3)
        stamp = float(observation_time)
        if not math.isfinite(stamp):
            raise ValueError("coverage observation time must be finite")
        if self.previous_time is not None and stamp <= self.previous_time:
            return False
        continuous = (self.previous_time is not None
                      and stamp-self.previous_time <= self.maximum_gap_s)
        start = self.previous_tip if continuous else tip
        moved = np.linalg.norm(tip-start) > 1e-10
        # Resolve the observed chord finely enough to detect shortcuts that
        # leave the tube. No extrapolation or bridging over a marker gap.
        count = max(1, int(np.ceil(np.linalg.norm(tip-start)
                                  / (self.tolerance_m/4))))
        for point in np.linspace(start, tip, count+1):
            while self.next_station < len(self.arcs):
                index = self.next_station
                if index and not moved:
                    break
                error = point-self.points[index]
                if np.linalg.norm(error) > self.tolerance_m:
                    break
                if index and np.dot(error, self.tangents[index]) < -1e-10:
                    # Permit endpoint settling within tolerance only after
                    # the preceding stations were actually traversed.
                    if index != len(self.arcs)-1:
                        break
                self.covered_m = float(self.arcs[index])
                self.next_station += 1
        self.previous_tip = tip.copy()
        self.previous_time = stamp
        return True


@dataclass(frozen=True)
class GovernorConfig:
    nominal_speed_m_s: float
    total_timeout_s: float
    final_tolerance_mm: float
    final_settle_time_s: float
    soft_error_mm: float
    pause_error_mm: float
    resume_error_mm: float
    hard_error_mm: float
    recovery_speed_scale: float = 0.20

    def __post_init__(self):
        values = (
            self.nominal_speed_m_s, self.total_timeout_s,
            self.final_tolerance_mm, self.soft_error_mm,
            self.pause_error_mm, self.hard_error_mm)
        if any(not math.isfinite(value) or value <= 0.0 for value in values):
            raise ValueError("path governor positive values are invalid")
        if (not math.isfinite(self.final_settle_time_s)
                or self.final_settle_time_s < 0.0):
            raise ValueError(
                "final settle time must be finite and nonnegative")
        if not (0.0 < self.resume_error_mm <= self.soft_error_mm
                < self.pause_error_mm < self.hard_error_mm):
            raise ValueError(
                "require 0 < resume <= soft < pause < hard error")
        if (not math.isfinite(self.recovery_speed_scale)
                or not 0.0 < self.recovery_speed_scale <= 1.0):
            raise ValueError("recovery speed scale must be in (0,1]")


@dataclass(frozen=True)
class GovernorUpdate:
    progress_m: float
    progress_fraction: float
    speed_scale: float
    state: str
    elapsed_s: float
    reference_point_m: np.ndarray
    tangent: np.ndarray
    reference_error_mm: float
    closest: PathProjection
    complete: bool
    timed_out: bool
    hard_error: bool
    covered_arc_m: float
    coverage_fraction: float
    traversal_complete: bool
    endpoint_error_mm: float


class PathProgressGovernor:
    """Measurement-led phase and latched spatial lookahead.

    Time never earns path progress. Fresh local projection is constrained by
    ordered observed coverage so a shortcut cannot skip unvisited stations.
    A small forward position target supplies motion intent, but remains fixed
    until the tip captures its forward arc neighborhood. Rolling ROS timestamps therefore
    cannot move the objective while the robot is stationary or lagging.
    """

    def __init__(self, path: ArcLengthPath, config: GovernorConfig,
                 *, maximum_observation_gap_s=.5, lookahead_s=.6,
                 x_only=False, xz_only=False):
        if not isinstance(xz_only, bool) or (x_only and xz_only):
            raise ValueError("xz_only must be boolean and mutually exclusive with x_only")
        if not isinstance(x_only, bool):
            raise ValueError("x_only must be boolean")
        if (x_only or xz_only) and not np.allclose(path.knots[:, 1:], path.knots[0, 1:],
                                      rtol=0, atol=1e-12):
            raise ValueError("x_only/xz_only require constant Y/Z path knots")
        self.x_only = x_only
        self.xz_only = xz_only
        self.path = path
        self.config = config
        self.progress_m = 0.0
        self.started_at = None
        self.last_update_at = None
        self.paused = False
        self.final_within_since = None
        self.coverage = OrderedPathCoverage(
            path, config.final_tolerance_mm, maximum_observation_gap_s)
        if not math.isfinite(lookahead_s) or lookahead_s <= 0:
            raise ValueError("position lookahead duration must be positive")
        self.lookahead_m = min(
            config.nominal_speed_m_s*lookahead_s,
            1e-3*config.soft_error_mm)
        self.target_arc_m = min(path.length_m, self.lookahead_m)

    def start(self, now: float):
        if not math.isfinite(now):
            raise ValueError("start time must be finite")
        self.started_at = self.last_update_at = float(now)

    def _tracking_error_mm(self, closest, reference_error_mm):
        """Return path-tube error while guarding closed-path ambiguity."""
        forward_gap_m = closest.arc_m-self.progress_m
        catchup_limit_m = 1e-3*self.config.pause_error_mm
        if forward_gap_m > catchup_limit_m:
            # On a closed path the global closest point can be the coincident
            # far endpoint.  Do not treat that as zero lag or jump a lap.
            return float(reference_error_mm)
        lag_mm = 1000.0*max(0.0, self.progress_m-closest.arc_m)
        return float(np.hypot(closest.distance_mm, lag_mm))

    def update(self, tip_m, now: float, *, external_hold=False,
               observation_time=None) -> GovernorUpdate:
        if self.started_at is None:
            raise RuntimeError("path governor has not started")
        if not math.isfinite(now) or now < self.last_update_at:
            raise ValueError("update time must be finite and monotonic")
        tip = _finite_vector("tip", tip_m, 3)
        if self.x_only:
            tip = tip.copy()
            tip[1:] = self.path.knots[0, 1:]
        elif self.xz_only:
            tip = tip.copy()
            tip[1] = self.path.knots[0, 1]
        now = float(now)
        stamp = now if observation_time is None else float(observation_time)
        if stamp > now:
            raise ValueError("coverage observation cannot be from the future")
        prior_stamp = self.coverage.previous_time
        fresh_observation = self.coverage.update(tip, stamp)
        observation_gap = (prior_stamp is not None
                           and stamp-prior_stamp > self.coverage.maximum_gap_s)
        if observation_gap:
            self.final_within_since = None
        elapsed = now-self.started_at
        # Keep the unrestricted projection for geometric path-error reporting
        # and hard safety limits.  Phase decisions use an arc-local branch so
        # a closed path cannot jump between coincident start/end points.
        closest = self.path.closest(tip)
        phase_window_m = 1e-3*self.config.pause_error_mm
        next_arc = (self.path.length_m if self.coverage.complete else
                    self.coverage.arcs[self.coverage.next_station])
        # Only project onto the next unvisited branch. A symmetric search
        # around phase includes the coincident outbound leg after a reversal,
        # where a geometric distance tie can otherwise freeze phase forever.
        phase_closest = self.path._closest_in_arc_interval(
            tip, self.progress_m, max(self.progress_m, min(
                self.path.length_m, self.progress_m+phase_window_m, next_arc)))
        if not external_hold and fresh_observation and not observation_gap:
            forward_gap_m = phase_closest.arc_m-self.progress_m
            catchup_limit_m = 1e-3*self.config.pause_error_mm
            if (0.0 < forward_gap_m <= catchup_limit_m
                    and phase_closest.distance_mm
                    <= self.config.final_tolerance_mm):
                # Permit interpolation up to the next dense coverage station,
                # not an arbitrary jump to a geometrically nearby branch.
                self.progress_m = max(self.progress_m, min(
                    phase_closest.arc_m, next_arc))
            if self.coverage.complete:
                # Ordered fresh observations earned traversal, independent
                # of any lag in the latched lookahead. Capture the endpoint
                # with the existing point objective rather than a corridor.
                self.progress_m = self.path.length_m
                self.target_arc_m = self.path.length_m
        reference, tangent = self.path.evaluate(self.progress_m)
        reference_error_mm = 1000.0*float(np.linalg.norm(tip-reference))
        # If an early station was crossed outside the path tube, a forward
        # target can strand us beyond it forever. Recover that station rather
        # than crediting a shortcut or advancing the reference with time.
        coverage_recovery = False
        if (not external_hold and fresh_observation and not observation_gap
                and not self.coverage.complete):
            station = self.coverage.next_station
            error = tip-self.coverage.points[station]
            coverage_recovery = bool(
                np.dot(error, self.coverage.tangents[station]) > 1e-10
                and np.linalg.norm(error) > self.coverage.tolerance_m)
            if coverage_recovery:
                self.target_arc_m = next_arc
        tracking_error_mm = self._tracking_error_mm(
            phase_closest, reference_error_mm)
        if external_hold:
            scale = 0.0
        elif tracking_error_mm <= self.config.soft_error_mm:
            scale = 1.0
        elif tracking_error_mm < self.config.pause_error_mm:
            scale = ((self.config.pause_error_mm-tracking_error_mm)
                     / (self.config.pause_error_mm
                        - self.config.soft_error_mm))
            scale = self.config.recovery_speed_scale+(
                1.0-self.config.recovery_speed_scale)*scale
        elif tracking_error_mm < self.config.hard_error_mm:
            scale = self.config.recovery_speed_scale*(
                self.config.hard_error_mm-tracking_error_mm)/(
                    self.config.hard_error_mm-self.config.pause_error_mm)
        else:
            scale = 0.0
        scale = float(np.clip(scale, 0.0, 1.0))
        self.last_update_at = now
        if not external_hold and fresh_observation and not observation_gap:
            # A quarter-lookahead capture margin avoids requiring an exact
            # noisy projection equality. This is spatial, never time-earned.
            if (not coverage_recovery and self.target_arc_m < self.path.length_m
                    and self.progress_m >= self.target_arc_m-.25*self.lookahead_m):
                self.target_arc_m = min(
                    self.path.length_m, self.progress_m+self.lookahead_m)
        reference, tangent = self.path.evaluate(
            self.progress_m if external_hold else self.target_arc_m)
        reference_error_mm = 1000.0*float(np.linalg.norm(tip-reference))
        at_end = self.progress_m >= self.path.length_m-1e-12
        self.paused = bool(
            not external_hold and scale <= 1e-6 and not at_end)
        within_final = (at_end and self.coverage.complete
                        and not external_hold and reference_error_mm <=
                        self.config.final_tolerance_mm)
        if within_final:
            if self.final_within_since is None:
                self.final_within_since = stamp
        else:
            self.final_within_since = None
        complete = bool(
            elapsed < self.config.total_timeout_s
            and within_final and fresh_observation and stamp-self.final_within_since
            >= self.config.final_settle_time_s)
        return GovernorUpdate(
            progress_m=self.progress_m,
            progress_fraction=self.progress_m/self.path.length_m,
            speed_scale=scale,
            state=("TRANSMISSION_HOLD" if external_hold else
                   "FINAL_HOLD" if at_end else
                   "PAUSED" if self.paused else
                   "RECOVERY_HOLD" if coverage_recovery else
                   "RECOVERY_HOLD" if tracking_error_mm
                   >= self.config.pause_error_mm else
                   "SLOWED" if scale < 1.0 else "RUNNING"),
            elapsed_s=elapsed,
            reference_point_m=reference,
            tangent=tangent,
            reference_error_mm=reference_error_mm,
            closest=closest,
            complete=complete,
            timed_out=elapsed >= self.config.total_timeout_s,
            hard_error=closest.distance_mm > self.config.hard_error_mm,
            covered_arc_m=self.coverage.covered_m,
            coverage_fraction=self.coverage.covered_m/self.path.length_m,
            traversal_complete=self.coverage.complete,
            endpoint_error_mm=1000.0*float(np.linalg.norm(
                tip-self.path.knots[-1])))

    def preview(self, speed_scale: float, offsets_s):
        """Publish a spatial target, constant over the entire time horizon.

        ``speed_scale`` is retained for caller compatibility, not integrated.
        Transmission hold offers the measured phase; otherwise keep the
        latched target. Final capture still uses the existing endpoint cost.
        """
        offsets = np.asarray(offsets_s, dtype=np.float64)
        if offsets.ndim != 1 or not np.all(np.isfinite(offsets)):
            raise ValueError("preview offsets must be a finite vector")
        arc = self.progress_m if speed_scale == 0.0 else self.target_arc_m
        arcs = np.full(offsets.shape, arc)
        return self.path.evaluate(arcs)


@dataclass(frozen=True)
class ReferenceHorizon:
    path_id: str
    sequence: int
    source_timestamp_ns: int
    received_at_s: float
    sample_period_s: float
    positions_m: np.ndarray
    expiry_s: float
    progress_m: float
    total_length_m: float
    final_hold: bool
    progress_paused: bool
    tangents: np.ndarray | None = None
    x_only: bool = False
    xz_only: bool = False

    def __post_init__(self):
        if not isinstance(self.xz_only, bool) or (self.x_only and self.xz_only):
            raise ValueError("xz_only must be boolean and mutually exclusive with x_only")
        if not isinstance(self.x_only, bool):
            raise ValueError("x_only must be boolean")
        positions = np.asarray(self.positions_m, dtype=np.float64)
        tangents = (None if self.tangents is None else
                    np.asarray(self.tangents, dtype=np.float64))
        if (not self.path_id or self.sequence < 0
                or self.source_timestamp_ns <= 0
                or not math.isfinite(self.received_at_s)
                or not math.isfinite(self.sample_period_s)
                or self.sample_period_s <= 0.0
                or positions.ndim != 2 or positions.shape[1:] != (3,)
                or len(positions) < 2 or not np.all(np.isfinite(positions))
                or (tangents is not None and (
                    tangents.shape != positions.shape
                    or not np.all(np.isfinite(tangents))))
                or not math.isfinite(self.expiry_s) or self.expiry_s <= 0.0):
            raise ValueError("invalid reference horizon")
        object.__setattr__(self, "positions_m", positions.copy())
        if tangents is not None:
            norms = np.linalg.norm(tangents, axis=1)
            if np.any(norms <= 1e-12):
                raise ValueError("reference tangents must be nonzero")
            object.__setattr__(
                self, "tangents", tangents/norms[:, None])

    def targets(self, root_timestamp_ns: int, horizon_steps: int,
                rollout_step_s: float, now_s: float) -> np.ndarray:
        return self.sample(
            root_timestamp_ns, horizon_steps, rollout_step_s, now_s)[0]

    def compatible_with(self, current, root_timestamp_ns: int,
                        horizon_steps: int, rollout_step_s: float,
                        now_s: float, *, bounded_refresh: bool = False,
                        bounded_tangent_tolerance: float = 1e-3) -> bool:
        """Allow rolling refreshes only when they preserve the solved objective.

        Compare at identical absolute rollout times, not array indices: previews
        move their timestamp origin. Sampling retains expiry/coverage checks.
        Micrometre position and 1e-4 unit-tangent tolerances allow interpolation
        roundoff, not millimetre-scale task changes. Final holds do not score
        tangents, so only their position objective must match.

        The shared controller commit uses the qualified bounded-refresh policy.
        Moving objectives allow 10 um coordinate and 1e-3 tangent-component
        changes, with at most 0.1 mm forward phase advance and 1 um rewind.
        Phase bounds reject closed-path aliases even if sampled positions match.
        Final holds retain the strict endpoint tolerance. The strict default is
        retained for historical offline comparisons; the controller selects
        a 2e-3 moving tangent-component bound. This does not relax identity,
        policy, expiry, coverage, cancellation or deadline checks.
        """
        if (not math.isfinite(bounded_tangent_tolerance)
                or bounded_tangent_tolerance <= 0.0):
            raise ValueError("invalid bounded tangent tolerance")
        if (current is None or self.path_id != current.path_id
                or current.sequence < self.sequence
                or self.final_hold != current.final_hold
                or self.progress_paused != current.progress_paused
                or self.total_length_m != current.total_length_m
                or self.x_only != current.x_only
                or self.xz_only != current.xz_only):
            return False
        moving_refresh = bounded_refresh and not self.final_hold
        if moving_refresh:
            phase_change = current.progress_m-self.progress_m
            if not math.isfinite(phase_change) or not -1e-6 <= phase_change <= 1e-4:
                return False
        positions, tangents = self.sample(
            root_timestamp_ns, horizon_steps, rollout_step_s, now_s)
        current_positions, current_tangents = current.sample(
            root_timestamp_ns, horizon_steps, rollout_step_s, now_s)
        if not np.allclose(positions, current_positions, rtol=0.0,
                           atol=1e-5 if moving_refresh else 1e-6):
            return False
        if self.final_hold:
            return True
        if tangents is None or current_tangents is None:
            return tangents is current_tangents
        return bool(np.allclose(
            tangents, current_tangents, rtol=0.0,
            atol=bounded_tangent_tolerance if moving_refresh else 1e-4))

    def sample(self, root_timestamp_ns: int, horizon_steps: int,
               rollout_step_s: float, now_s: float):
        if now_s-self.received_at_s > self.expiry_s:
            raise ValueError("path_reference_stale")
        query = root_timestamp_ns/1e9+rollout_step_s*np.arange(
            1, horizon_steps+1, dtype=np.float64)
        source = self.source_timestamp_ns/1e9+self.sample_period_s*np.arange(
            len(self.positions_m), dtype=np.float64)
        tolerance = 1e-6
        if query[0] < source[0]-tolerance:
            raise ValueError("path_reference_starts_too_late")
        if query[-1] > source[-1]+tolerance:
            raise ValueError("path_reference_too_short")
        positions = np.column_stack([
            np.interp(query, source, self.positions_m[:, axis])
            for axis in range(3)
        ])
        if self.tangents is None:
            return positions, None
        tangents = np.column_stack([
            np.interp(query, source, self.tangents[:, axis])
            for axis in range(3)
        ])
        norms = np.linalg.norm(tangents, axis=1)
        if np.any(norms <= 1e-12):
            raise ValueError("path_reference_tangent_degenerate")
        return positions, tangents/norms[:, None]
