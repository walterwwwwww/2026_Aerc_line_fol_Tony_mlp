#!/usr/bin/env python3
"""
shapes.py -- geometry for the 10 physical tuning-test shapes.

All shapes use a LINE THICKNESS OF 2 cm (20 mm) of black tape/paint on a light
background, matching the firmware's expected line width.

COORDINATE SYSTEM
    Units are MILLIMETRES throughout (1 unit = 1 mm), matching the SVG output
    (width="..." height="..." viewBox="0 0 W H") so the plots print/machine at
    true 1:1 scale.
    x -> right, y -> down (SVG convention).

GEOMETRY MODEL
    Each shape is a list of "runs". A run is a straight segment traversed by the
    robot.  We model the robot as driving the CENTRE LINE of the run and then
    being laterally displaced by `offset` and rotated by `theta`.

    This is what lets us generate ground-truth training data: for any
    (run, progress, offset, theta) we know exactly where the 14 sensors sit in
    world space, hence exactly which ones are over the 2 cm black line.

    Shapes whose runs are not axis-aligned (the 45 deg chamfer, the arcs) are
    handled by the same segment model -- arcs are approximated by a polyline of
    short chords, which is exactly how the robot sees them anyway.

Each shape carries metadata about WHICH tuning parameter it is designed to
exercise, so the bench session in README.md has a purpose for every piece.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import List, Tuple

# --------------------------------------------------------------------------
# Physical constants of the robot / track
# --------------------------------------------------------------------------
LINE_WIDTH_MM = 20.0        # 2 cm black line, per requirement
LINE_HALF_MM = LINE_WIDTH_MM / 2.0

SENSOR_COUNT = 14
SENSOR_PITCH_MM = 10.0      # 1 cm between adjacent sensor centres
# 14 sensors at 10 mm pitch span 130 mm centre-to-centre.
ARRAY_SPAN_MM = (SENSOR_COUNT - 1) * SENSOR_PITCH_MM


@dataclass
class Run:
    """One straight traversed segment, given by its centre-line endpoints."""

    x0: float
    y0: float
    x1: float
    y1: float
    note: str = ""

    @property
    def length(self) -> float:
        return math.hypot(self.x1 - self.x0, self.y1 - self.y0)

    @property
    def heading(self) -> float:
        """Angle of travel, radians, measured from +x (screen right)."""
        return math.atan2(self.y1 - self.y0, self.x1 - self.x0)

    def point_at(self, s: float) -> Tuple[float, float]:
        """Point at arc-length s along the run, clamped to [0, length]."""
        L = self.length
        if L <= 0:
            return self.x0, self.y0
        t = max(0.0, min(1.0, s / L)) if L else 0.0
        return self.x0 + (self.x1 - self.x0) * t, self.y0 + (self.y1 - self.y0) * t

    def normal(self) -> Tuple[float, float]:
        """Unit normal (left of travel direction)."""
        h = self.heading
        return -math.sin(h), math.cos(h)


@dataclass
class Shape:
    name: str
    slug: str
    width: float
    height: float
    runs: List[Run]
    # which tuning knob this shape is for
    tunes: str = ""
    how: str = ""
    difficulty: str = "easy"
    # pen-up moves (for drawing the real track, so the robot path is continuous
    # where it should be and lifted where it must not be)
    notes: str = ""
    tags: List[str] = field(default_factory=list)

    @property
    def total_path_mm(self) -> float:
        return sum(r.length for r in self.runs)


# ==========================================================================
#  Helper builders
# ==========================================================================
def _rect_loop(x0, y0, w, h, start="bottom-left", direction="ccw") -> List[Run]:
    """Closed rectangle, traversed corner to corner. Runs join end-to-start."""
    x1, y1 = x0 + w, y0 + h
    # corners in clockwise screen order: BL -> BR -> TR -> TL
    bl = (x0, y1)
    br = (x1, y1)
    tr = (x1, y0)
    tl = (x0, y0)
    order = [bl, br, tr, tl, bl]
    if direction == "ccw":
        order = list(reversed(order))
    # rotate so we start where asked
    if start == "bottom-left":
        pass
    return [Run(*order[i], *order[i + 1]) for i in range(4)]


def _polyline_runs(points, close=False) -> List[Run]:
    pts = list(points)
    if close:
        pts = pts + [pts[0]]
    return [Run(*pts[i], *pts[i + 1]) for i in range(len(pts) - 1)]


def _arc_runs(cx, cy, r, a0_deg, a1_deg, n=28) -> List[Run]:
    """Approximate a circular arc by n chords."""
    pts = []
    a0 = math.radians(a0_deg)
    a1 = math.radians(a1_deg)
    for i in range(n + 1):
        a = a0 + (a1 - a0) * i / n
        pts.append((cx + r * math.cos(a), cy + r * math.sin(a)))
    return _polyline_runs(pts)


# ==========================================================================
#  The 10 shapes
# ==========================================================================
def build_shapes() -> List[Shape]:
    shapes: List[Shape] = []

    # ---------------------------------------------------------------- 1
    # 100 cm dead-straight. The single most important piece: everything that is
    # not a turn gets tuned here, because a straight line shows you drift,
    # steady-state offset, and whether the robot oscillates.
    shapes.append(
        Shape(
            name="S1 - Straight 1000 mm",
            slug="s01_straight_1000",
            width=1000 + 2 * 60,
            height=200,
            runs=[Run(60, 100, 1060, 100, "dead straight")],
            tunes="Sensor weight symmetry, base_speed, steering PID (KP_L/KI_L/KD_L) straight-line damping",
            how=(
                "Place the robot centred on the line and let it run. Watch for (a) it "
                "veering consistently to one side -> weight asymmetry, (b) slow "
                "weaving -> KP_L too high or KD_L too low, (c) sluggish correction -> "
                "KP_L too low."
            ),
            notes="Longest single run in the set; use it to set base_speed before any turn work.",
            tags=["straight", "baseline"],
        )
    )

    # ---------------------------------------------------------------- 2
    # 200 mm square loop, four 90 deg corners. The classic "square of side 20 cm".
    shapes.append(
        Shape(
            name="S2 - Square loop 200 mm (4 x 90 deg)",
            slug="s02_square_200",
            width=200 + 2 * 60,
            height=200 + 2 * 60,
            runs=_rect_loop(60, 60, 200, 200),
            tunes="90 deg turn rate (current_exit_ms / turn table), corner detection threshold",
            how=(
                "Run the loop repeatedly in one direction. If it cuts the corner, "
                "increase the turn duration; if it overshoots and has to come back, "
                "decrease it. Both left and right corners must behave the same -> "
                "that is a symmetry check on weights 1..14."
            ),
            notes="Corners are 90 deg with 200 mm legs; a 130 mm sensor array fits the leg comfortably.",
            tags=["square", "90deg", "loop"],
        )
    )

    # ---------------------------------------------------------------- 3
    # 20 cm square, sharp 90 deg, but traversed as an OPEN path (3 turns) so you
    # can watch one corner at a time without the loop closing on itself.
    shapes.append(
        Shape(
            name="S3 - Open 3-turn Z (3 x 90 deg, 200 mm legs)",
            slug="s03_open_z_200",
            width=200 * 3 + 2 * 60,
            height=200 + 2 * 60,
            runs=_polyline_runs(
                [(60, 160), (260, 160), (260, 60), (460, 60), (460, 160), (660, 160)]
            ),
            tunes="Turn direction reversal, turn-table slots, acceleration limit into/out of turns",
            how=(
                "Each corner alternates left/right. If only one direction misbehaves, "
                "the fault is in the turn table symmetry, not in speed. Watch the "
                "transition: the robot should slow, rotate, then reacquire without "
                "hunting."
            ),
            notes="Best shape for isolating a single bad corner because the straights between are identical.",
            tags=["90deg", "alternating"],
        )
    )

    # ---------------------------------------------------------------- 4
    # Wide 20 cm square: 90 deg corners with long 300 mm approach so the robot
    # is at full cruise speed when it hits the corner. This is the acceleration
    # / deceleration limit test.
    shapes.append(
        Shape(
            name="S4 - Square 300 mm legs (high-speed corner entry)",
            slug="s04_square_300",
            width=300 + 2 * 60,
            height=300 + 2 * 60,
            runs=_rect_loop(60, 60, 300, 300, direction="cw"),
            tunes="Acceleration limit / deceleration distance before a turn",
            how=(
                "Same 90 deg corner as S2 but with 300 mm to build speed. If the robot "
                "runs wide here but is fine on S2, the acceleration limit is too high "
                "or braking starts too late. Set the accel limit on this shape, then "
                "re-check S2."
            ),
            notes="Compare against S2: identical corner angle, longer run-up. Difference isolates speed effects.",
            tags=["90deg", "accel", "loop"],
        )
    )

    # ---------------------------------------------------------------- 5
    # Gentle 45 deg chamfered square. 45 deg is the hardest angle for a
    # discrete-sensor line follower because both the outer and an inner sensor
    # see black at once -- exactly the regime the paired-weight maths handles.
    shapes.append(
        Shape(
            name="S5 - Chamfered square (4 x 45 deg)",
            slug="s05_chamfer_45",
            width=280 + 2 * 60,
            height=280 + 2 * 60,
            runs=_polyline_runs(
                [
                    (60, 280),
                    (200, 280),
                    (280, 200),
                    (280, 60),
                    (140, 60),
                    (60, 140),
                    (60, 280),
                ]
            ),
            tunes="Outer sensor weights (w1, w14), FORCE_ERROR_MAG, 45 deg handling",
            how=(
                "At 45 deg two sensors go black almost simultaneously. If the robot "
                "stutters or loses the line here but is fine on S2, the outer weights "
                "are wrong. This is the shape that justifies ML-tuned weights."
            ),
            notes="45 deg is the worst case for the hand-tuned linear weight ramp.",
            tags=["45deg", "diagonal", "hard"],
        )
    )

    # ---------------------------------------------------------------- 6
    # 90 deg arc. Continuous curvature rather than discrete corners, so the
    # steering loop must track a slowly varying error instead of a step.
    shapes.append(
        Shape(
            name="S6 - Quarter arc R=200 mm (90 deg continuous)",
            slug="s06_arc_r200",
            width=200 + 2 * 60 + 130,
            height=200 + 2 * 60 + 130,
            runs=_arc_runs(260, 260, 200, 180, 270),
            tunes="Steering integral (KI_L) on a sustained error, continuous tracking",
            how=(
                "On a constant-radius arc the error is non-zero for a long time. Too "
                "little KI_L leaves a steady inward bias; too much winds up and the "
                "robot spirals off the end."
            ),
            tags=["arc", "continuous"],
        )
    )

    # ---------------------------------------------------------------- 7
    # S-curve. Forces the steering integrator to reverse; the classic test for
    # integral windup.
    shapes.append(
        Shape(
            name="S7 - S-curve (2 x 200 mm arcs, reversed)",
            slug="s07_scurve",
            width=2 * 160 + 2 * 60,
            height=160 + 2 * 60,
            runs=_arc_runs(60 + 80, 60 + 80, 80, 90, 270)
            + _arc_runs(60 + 240, 60 + 80, 80, 270, 90),
            tunes="Integral windup (KI_L), reversal symmetry",
            how=(
                "Watch the second half. If the robot swings wide coming out of the "
                "first arc and takes time to settle, KI_L is winding up. Reduce it or "
                "clear the integral when the error sign flips."
            ),
            notes="Two equal opposite arcs; any asymmetry is a bug, not a tuning choice.",
            tags=["scurve", "windup"],
        )
    )

    # ---------------------------------------------------------------- 8
    # Zigzag of 90 deg corners on short 120 mm legs -- deliberately shorter than
    # the 130 mm sensor array span so the array straddles two corners at once.
    shapes.append(
        Shape(
            name="S8 - Zigzag 120 mm legs (array straddles corners)",
            slug="s08_zigzag_120",
            width=120 * 6 + 2 * 60,
            height=120 + 2 * 60,
            runs=_polyline_runs(
                [
                    (60, 180),
                    (180, 180),
                    (180, 60),
                    (300, 60),
                    (300, 180),
                    (420, 180),
                    (420, 60),
                    (540, 60),
                    (540, 180),
                    (660, 180),
                ]
            ),
            tunes="Behaviour when the array cannot see a single clean line (line_found fallback)",
            how=(
                "120 mm legs are shorter than the 130 mm array, so the array sees two "
                "line segments at once. If the robot loses the line only here, the "
                "problem is the fallback path, not the weights."
            ),
            notes="Stress test. Expect it to be the hardest piece in the set.",
            tags=["zigzag", "hard", "straddle"],
        )
    )

    # ---------------------------------------------------------------- 9
    # Long straight, a single 90 deg corner, then long straight. Isolates ONE
    # corner with full-length straights either side, so the whole run is
    # repeatable in one direction -- ideal for measuring turn duration in ms.
    shapes.append(
        Shape(
            name="S9 - L-bend, 400 mm legs (single 90 deg, measurable)",
            slug="s09_lbend_400",
            width=400 + 2 * 60,
            height=400 + 2 * 60,
            runs=_polyline_runs([(60, 460), (460, 460), (460, 60)]),
            tunes="Exact turn duration / turn-table slot for 90 deg (measure in ms)",
            how=(
                "One corner only, with 400 mm of straight either side. Time the "
                "rotation and check the exit: it should leave the corner parallel to "
                "the new line, not angled."
            ),
            notes="Best shape for turning the 20-row turn table into numbers, because there is exactly one corner.",
            tags=["90deg", "single", "measure"],
        )
    )

    # ---------------------------------------------------------------- 10
    # Rectangle 400 x 200. Long straights for speed, four 90 deg corners for the
    # turn table, and a 2:1 aspect ratio so the robot never completes a full
    # straight-line settle before the next corner.
    shapes.append(
        Shape(
            name="S10 - Rectangle 400 x 200 mm (race-style loop)",
            slug="s10_rect_400x200",
            width=400 + 2 * 60,
            height=200 + 2 * 60,
            runs=_rect_loop(60, 60, 400, 200),
            tunes="End-to-end integration: all parameters together, lap timing",
            how=(
                "Run five laps and time them. This is the acceptance test: it should "
                "be clean and consistent. Use it to confirm whatever you changed on "
                "S1..S9 did not regress the whole system."
            ),
            notes="Acceptance / regression shape. Do not tune on this one; verify on it.",
            tags=["loop", "acceptance", "timing"],
        )
    )

    # ---------------------------------------------------------------- 11
    # Stadium loop: two 400 mm straights joined by two 180 deg arcs of R=150 mm.
    # This is THE continuous-training loop for the MLP: the car can run laps
    # endlessly, and every lap contributes fresh labelled frames. The long
    # straights give clean offset labels; the two constant-radius arcs give
    # sustained non-zero error for the network to learn from. No sharp corners,
    # so the car almost never loses the line and never needs a human to reset it.
    shapes.append(
        Shape(
            name="S11 - Stadium loop (continuous ML training, R=150 mm)",
            slug="s11_stadium_loop",
            width=400 + 2 * 60 + 2 * 150,
            height=300 + 2 * 60,
            runs=_polyline_runs([(60 + 150, 60), (60 + 150 + 400, 60)])
            + _arc_runs(60 + 150 + 400, 60 + 150, 150, -90, 90)
            + _polyline_runs([(60 + 150 + 400, 60 + 300), (60 + 150, 60 + 300)])
            + _arc_runs(60 + 150, 60 + 150, 150, 90, 270),
            tunes="Continuous lap-based ML training: endless labelled frames per loop",
            how=(
                "Run laps continuously with the 上位机 in labelling mode. Each lap "
                "streams frames through both straights and both arcs, so the MLP sees "
                "a full range of offsets and curvatures every lap. Vary the lateral "
                "offset slightly between laps to cover more of the input space."
            ),
            notes="Recommended day-to-day training loop. Runs forever; no reset needed.",
            tags=["loop", "continuous", "training", "stadium"],
        )
    )

    return shapes


# --------------------------------------------------------------------------
#  Sampling: turn a shape into ground-truth samples
# --------------------------------------------------------------------------
def sample_runs(
    shape: Shape,
    step_mm: float = 2.0,
    offsets_mm=None,
    thetas_deg=None,
):
    """
    Yield (x_center, y_center, heading, offset, theta) sample points along a
    shape.

    offset  : lateral displacement of the SENSOR ARRAY CENTRE from the line
              centre, in mm. Positive = array centre to the left of travel.
    theta   : heading error of the robot relative to the line, degrees.
              Positive = robot pointing left of the line direction.
    """
    if offsets_mm is None:
        offsets_mm = [0.0]
    if thetas_deg is None:
        thetas_deg = [0.0]

    for run in shape.runs:
        L = run.length
        if L <= 0:
            continue
        n = max(2, int(math.ceil(L / step_mm)))
        for i in range(n + 1):
            s = L * i / n
            cx, cy = run.point_at(s)
            h = run.heading
            for off in offsets_mm:
                for th in thetas_deg:
                    yield cx, cy, h, off, th


if __name__ == "__main__":
    total = 0.0
    for sh in build_shapes():
        print(
            f"{sh.slug:22s} path={sh.total_path_mm:7.1f} mm  runs={len(sh.runs):3d}  "
            f"canvas={sh.width:.0f}x{sh.height:.0f}  tunes={sh.tunes[:44]}"
        )
        total += sh.total_path_mm
    print(f"\nTotal traversable path across all 10 shapes: {total/1000:.2f} m")
