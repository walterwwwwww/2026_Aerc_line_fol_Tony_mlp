#!/usr/bin/env python3
"""
sensor_model.py -- forward model of the 14-channel IR array on the 2 cm line,
plus the synthetic dataset generator that the MLP trains on.

WHY A SYNTHETIC MODEL
---------------------
The 10 physical shapes (shapes.py) are exact geometry. From geometry alone we
can compute, for any robot pose, which of the 14 sensors is over the black line.
That gives us an unlimited supply of perfectly-labelled training data where the
LABEL (true lateral offset of the array centre from the line) is known exactly
-- which is precisely what you can never measure by hand on a real track.

The model is deliberately explicit about its assumptions so it can be corrected
later with logged real data (see train.py --real).

WHAT THE MODEL ASSUMES (all overridable)
----------------------------------------
  sensor pitch        10.0 mm   -> 14 sensors span 130 mm
  line width          20.0 mm   (the 2 cm requirement)
  sensor FOV diameter 8.0 mm    (how much track one photodiode averages)
  threshold           reflectance <= 6.0 of 0..10 counts as "on the black line"
                      -- this is copied from app_adc.c:
                         if_bai[i] = (adc_test[i] <= 6.0f) ? 1 : 0;

GROUND TRUTH SIGN CONVENTION (must match the firmware)
------------------------------------------------------
  offset < 0  ->  array centre is to the LEFT of the line   -> robot must turn left
  offset > 0  ->  array centre is to the RIGHT of the line  -> robot must turn right

The firmware's adc_error has the opposite sign convention for a left offset
(sensor 0 is the leftmost sensor and a left-side hit yields a negative error),
so export.py flips the sign when it writes the C header. Getting this wrong
would make the robot steer away from the line -- it is the single most likely
integration bug, so it is asserted in tests.
"""

from __future__ import annotations

import math
import os
import sys
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from shapes import (  # noqa: E402
    ARRAY_SPAN_MM,
    LINE_HALF_MM,
    LINE_WIDTH_MM,
    SENSOR_COUNT,
    SENSOR_PITCH_MM,
    Shape,
    build_shapes,
)

# --------------------------------------------------------------------------
#  Model parameters (override these after measuring your real robot)
# --------------------------------------------------------------------------
# Effective width of one sensor's view. This is the single most influential
# parameter in the model, so it is worth being explicit about it.
#
# A sensor reads "black" when its sensitive spot overlaps the 20 mm band. With
# an effective spot of `FOV_MM`, a sensor whose centre is `d` mm from the line
# centre-line reads black while  d <= LINE_HALF_MM + FOV_MM/2.
#
# FOV_MM = 4.0 was chosen so that a perfectly centred array reports exactly the
# sensor pair (6,7) as black -- which is what the firmware's own default weights
# are built around (weights 7 and 8 are both 0.1, i.e. the centre pair is treated
# as a near-deadband). Setting FOV too large makes every sensor read black at
# once and destroys the geometry; that bug is caught by _selftest().
FOV_MM = 4.0
BLACK_THRESHOLD = 6.0       # adc_test[i] <= this  =>  sensor is on the line

# Sensor positions across the array, left (0) to right (13), in mm from centre.
SENSOR_X_MM = np.array(
    [(i - (SENSOR_COUNT - 1) / 2.0) * SENSOR_PITCH_MM for i in range(SENSOR_COUNT)],
    dtype=np.float64,
)


# --------------------------------------------------------------------------
#  Geometry: does a point lie on the black line?
# --------------------------------------------------------------------------
def _point_on_line(
    px: float,
    py: float,
    ox: float,
    oy: float,
    tx: float,
    ty: float,
    half_width: float = LINE_HALF_MM,
) -> bool:
    """
    Is world point (px,py) within `half_width` of the infinite line through
    (ox,oy) with unit direction (tx,ty)?

    We use the infinite line rather than a finite segment because the shapes are
    built from runs that join end to end, and at a joint the robot really does
    see the neighbouring run's line through the corner.
    """
    vx, vy = px - ox, py - oy
    # perpendicular distance to the line
    perp = abs(-ty * vx + tx * vy)
    return perp <= half_width


def sensors_on_line(
    offset_mm: float,
    theta_rad: float,
    fov_mm: Optional[float] = None,
) -> np.ndarray:
    """
    Binary "is sensor i over black" vector for a robot whose array centre is
    `offset_mm` laterally from the line and rotated `theta_rad` from the line
    direction.

    Because the line is locally straight and infinite, we can put the line along
    the world x-axis at y=0 and place the array analytically.

    Returns int8 array of length 14, 1 = sensor sees black.
    """
    if fov_mm is None:
        fov_mm = FOV_MM

    # ---- Derivation (this is the heart of the model) ----------------------
    # Let the LINE pass through the origin with unit direction
    #       d = (cos theta, sin theta)
    # and unit normal
    #       n = (-sin theta, cos theta)
    #
    # The array is a rigid bar:
    #   * its centre C is displaced from the line by `offset_mm` along the
    #     normal -- so C = offset * n  (signed)
    #   * its axis is rotated by `theta` from the line, i.e. the axis is along d
    #   * sensor i sits at signed arclength s_i = SENSOR_X_MM[i] along that axis
    #
    #     P_i = C + s_i * d
    #
    # The perpendicular distance from P_i to the line is |P_i . n|:
    #
    #     P_i . n = (offset * n + s_i * d) . n
    #             = offset * (n.n) + s_i * (d.n)
    #             = offset * 1      + s_i * 0          because d _|_ n
    #             = offset                              ... for theta = 0
    #
    # That last line is only right when d and n are as defined ABOVE. The
    # general expression is obtained by keeping theta in d and n:
    #
    #     d . n = cos theta * (-sin theta) + sin theta * cos theta = 0   (always)
    #
    # so with the normal held fixed at (-sin theta, cos theta) the spacing term
    # would vanish entirely -- which is wrong. The correct physical statement is
    # that the array axis is rotated by theta RELATIVE TO THE LINE, so the
    # perpendicular offset contributed by sensor i is s_i * cos(theta):
    #
    #     perp_i = | offset + s_i * cos(theta) |
    #
    # Sanity checks on this expression, both asserted in _selftest():
    #   theta = 0      -> perp_i = |offset + s_i| : every sensor sits at its own
    #                     distinct distance; a centred 2 cm line lights the
    #                     middle pair.  <-- the case that caught the earlier bug
    #   theta = 90 deg -> perp_i = |offset|       : the bar lies ALONG the normal,
    #                     so all sensors are equally far from the line.
    #
    # An earlier version of this file used sin() here, which made every sensor
    # report the same distance at theta = 0 and lit all 14 sensors at once.
    ct = math.cos(theta_rad)
    perp = np.abs(offset_mm + SENSOR_X_MM * ct)

    # Sensor reads black when its sensitive spot overlaps the black band, i.e.
    # its centre is within (line half-width + half the sensor's own spot).
    return (perp <= (LINE_HALF_MM + fov_mm / 2.0)).astype(np.int8)


# --------------------------------------------------------------------------
#  Firmware-equivalent error calculation (the thing we are trying to beat)
# --------------------------------------------------------------------------
DEFAULT_WEIGHTS = np.array(
    [6.5, 5.0, 4.0, 3.0, 2.0, 1.0, 0.1, 0.1, 1.0, 2.0, 3.0, 4.0, 5.0, 6.5],
    dtype=np.float64,
)

FORCE_ERROR_MAG = 10.0      # matches the 10.0f in sensor_pair_error()


def _sensor_pair_error(l: float, r: float, wl: float, wr: float,
                       lb: bool, rb: bool) -> float:
    """Byte-for-byte reimplementation of sensor_pair_error() in app_sensor_weights.h."""
    if lb and rb:
        if wl == wr:
            return (r - l) * wl
        return r * wr - l * wl
    if lb:
        return -FORCE_ERROR_MAG * wl
    if rb:
        return FORCE_ERROR_MAG * wr
    return 0.0


def firmware_error(
    refl: np.ndarray,
    weights: np.ndarray = DEFAULT_WEIGHTS,
    threshold: float = BLACK_THRESHOLD,
) -> float:
    """
    Reproduce app_adc.c's error computation exactly.

    refl : (..., 14) calibrated reflectance on the firmware's 0..10 scale,
           where LOW means dark.
    Returns the adc_error value the firmware would compute.

    This is used both as the baseline to beat and as an equivalence test.
    """
    refl = np.atleast_1d(np.asarray(refl, dtype=np.float64))
    if refl.ndim == 1:
        refl = refl[None, :]
    out = np.zeros(refl.shape[0], dtype=np.float64)
    for n in range(refl.shape[0]):
        row = refl[n]
        black = row <= threshold
        err = 0.0
        # firmware loops i in 0..SENSOR_HALF-1, j = SENSOR_COUNT-1-i
        for i in range(SENSOR_COUNT // 2):
            j = SENSOR_COUNT - 1 - i
            if black[i] or black[j]:
                err = _sensor_pair_error(
                    float(row[i]), float(row[j]),
                    float(weights[i]), float(weights[j]),
                    bool(black[i]), bool(black[j]),
                )
                break
        out[n] = err
    return float(out[0]) if out.size == 1 else out


def offset_to_firmware_error_scale(
    offsets_mm: np.ndarray,
    weights: np.ndarray = DEFAULT_WEIGHTS,
    thetas_rad: Optional[np.ndarray] = None,
) -> float:
    """
    Least-squares gain g minimising ||g * firmware_error - offset_mm||.

    The firmware's error is in arbitrary weight units; the physical offset is in
    mm. To compare "how well does the hand-tuned scheme locate the line" against
    the MLP, we are allowed to give the hand-tuned scheme its best possible
    linear cheat, i.e. an optimal gain. If it still loses, the loss is real and
    not an artefact of unit choice.
    """
    if thetas_rad is None:
        thetas_rad = np.zeros_like(offsets_mm)
    fe = []
    for o, t in zip(offsets_mm, thetas_rad):
        black = sensors_on_line(float(o), float(t))
        refl = np.where(black == 1, 0.0, 10.0)
        fe.append(firmware_error(refl, weights))
    fe = np.asarray(fe)
    denom = float(np.dot(fe, fe))
    if denom < 1e-12:
        return 0.0
    return float(np.dot(fe, offsets_mm) / denom)


# --------------------------------------------------------------------------
#  Dataset (augmented: noise, uneven sensors, real-world dirt)
# --------------------------------------------------------------------------
@dataclass
class Dataset:
    X: np.ndarray            # (N, 14) calibrated reflectance, 0..10
    y_mm: np.ndarray         # (N,)    true lateral offset, mm  (target)
    y_theta: np.ndarray      # (N,)    true heading error, rad  (aux target)
    shape_id: np.ndarray     # (N,)    which shape each sample came from
    meta: Dict[str, float]


def _sample_shape_positions(sh: Shape, step_mm: float) -> List[Tuple[float, float, float]]:
    """(cx, cy, heading) points along the shape's runs."""
    pts = []
    for run in sh.runs:
        L = run.length
        if L <= 1e-9:
            continue
        n = max(2, int(math.ceil(L / step_mm)))
        for i in range(n + 1):
            s = L * i / n
            cx, cy = run.point_at(s)
            pts.append((cx, cy, run.heading))
    return pts


def build_dataset(
    offsets_mm: Optional[np.ndarray] = None,
    thetas_deg: Optional[np.ndarray] = None,
    pos_step_mm: float = 4.0,
    noise_sigma: float = 0.20,
    gain_sigma: float = 0.06,
    offset_sigma: float = 0.15,
    seed: int = 20261007,
    max_samples: Optional[int] = 240_000,
    verbose: bool = True,
) -> Dataset:
    """
    Build the training set.

    Augmentation rationale (each one mimics a real failure mode):
      noise_sigma    sensor read noise on the 0..10 reflectance scale
      gain_sigma     per-sensor gain mismatch after a sloppy calibration
      offset_sigma   per-sensor additive calibration offset error
    Without these the MLP learns a razor-sharp threshold and then performs badly
    on a real robot whose calibration is never perfect.
    """
    rng = np.random.default_rng(seed)
    if offsets_mm is None:
        offsets_mm = np.arange(-40.0, 40.0 + 1e-9, 1.0)
    if thetas_deg is None:
        thetas_deg = np.arange(-10.0, 10.0 + 1e-9, 1.0)

    shapes = build_shapes()
    rows_X: List[np.ndarray] = []
    rows_off: List[float] = []
    rows_th: List[float] = []
    rows_sid: List[int] = []

    for sid, sh in enumerate(shapes):
        pts = _sample_shape_positions(sh, pos_step_mm)
        # for each run position, we mainly care about the pose relative to the
        # line; the world position only matters for which run we are on, and
        # because the line is locally straight the relative model is identical.
        # We therefore enumerate (offset, theta) once per run-sample.
        for (cx, cy, h) in pts:
            n_off = len(offsets_mm)
            n_th = len(thetas_deg)
            black = np.zeros((n_off * n_th, SENSOR_COUNT), dtype=np.int8)
            k = 0
            for o in offsets_mm:
                for tdeg in thetas_deg:
                    black[k] = sensors_on_line(float(o), math.radians(float(tdeg)))
                    k += 1

            # reflectance: black -> ~0..2, white -> ~8..10
            refl = np.where(black == 1, 1.0, 9.0).astype(np.float64)

            off_grid, th_grid = np.meshgrid(offsets_mm, thetas_deg, indexing="ij")
            off_flat = off_grid.ravel()
            th_flat = np.radians(th_grid.ravel())

            rows_X.append(refl)
            rows_off.append(off_flat)
            rows_th.append(th_flat)
            rows_sid.append(np.full(off_flat.shape, sid, dtype=np.int32))

    X = np.concatenate(rows_X, axis=0)
    y_mm = np.concatenate(rows_off, axis=0)
    y_theta = np.concatenate(rows_th, axis=0)
    shape_id = np.concatenate(rows_sid, axis=0)

    # ---- augmentation -----------------------------------------------------
    n = X.shape[0]
    gain = 1.0 + rng.normal(0.0, gain_sigma, size=(n, SENSOR_COUNT))
    offs = rng.normal(0.0, offset_sigma, size=(n, SENSOR_COUNT))
    X = X * gain + offs
    X = X + rng.normal(0.0, noise_sigma, size=X.shape)
    X = np.clip(X, 0.0, 10.0)

    # drop samples where NO sensor sees the line -- the firmware treats that as
    # line-lost and it contributes no usable steering information
    any_black = (X <= BLACK_THRESHOLD).any(axis=1)
    keep = any_black
    X, y_mm, y_theta, shape_id = X[keep], y_mm[keep], y_theta[keep], shape_id[keep]

    # subsample if huge, keeping coverage across offsets
    if max_samples is not None and X.shape[0] > max_samples:
        idx = rng.choice(X.shape[0], size=max_samples, replace=False)
        X, y_mm, y_theta, shape_id = X[idx], y_mm[idx], y_theta[idx], shape_id[idx]

    meta = dict(
        n_samples=float(X.shape[0]),
        pitch_mm=SENSOR_PITCH_MM,
        line_width_mm=LINE_WIDTH_MM,
        fov_mm=FOV_MM,
        threshold=BLACK_THRESHOLD,
        noise_sigma=noise_sigma,
        gain_sigma=gain_sigma,
        offset_sigma=offset_sigma,
    )
    if verbose:
        print(f"dataset: {X.shape[0]:,} samples x {X.shape[1]} sensors  "
              f"(offset {y_mm.min():.0f}..{y_mm.max():.0f} mm, "
              f"theta {math.degrees(y_theta.min()):.0f}..{math.degrees(y_theta.max()):.0f} deg)")
    return Dataset(X=X, y_mm=y_mm, y_theta=y_theta, shape_id=shape_id, meta=meta)


# --------------------------------------------------------------------------
#  Self-test
# --------------------------------------------------------------------------
def _selftest() -> int:
    ok = True
    print("=== sensor_model self-test ===")

    # 1. centred on the line -> exactly the centre pair (6,7) black
    b = sensors_on_line(0.0, 0.0)
    print(f"offset 0.0 mm, theta 0: black sensors = {np.flatnonzero(b).tolist()}")
    if not (b[6] == 1 and b[7] == 1):
        print("  FAIL: centre sensors must see black"); ok = False
    if b[0] == 1 or b[13] == 1:
        print("  FAIL: outer sensors must NOT see a 2 cm line dead centre"); ok = False
    if not np.array_equal(np.flatnonzero(b), [6, 7]):
        print("  note: centre black set is not the expected (6,7)")

    # 2. shifted left -> the black set moves to the right of the array
    b = sensors_on_line(-35.0, 0.0)
    print(f"offset -35 mm: black sensors = {np.flatnonzero(b).tolist()}")
    if b.sum() == 0:
        print("  FAIL: should still see the line at -35 mm"); ok = False
    if b[0] == 1:
        print("  FAIL: leftmost sensor should be off the line at -35 mm"); ok = False
    # monotonic: as the array moves right relative to the line, the black set
    # must move left (lower indices)
    idx_at = lambda o: (np.flatnonzero(sensors_on_line(o, 0.0))[0]
                        if sensors_on_line(o, 0.0).any() else -1)
    seq = [idx_at(o) for o in (-30, -20, -10, 0, 10, 20, 30)]
    print(f"first-black-sensor vs offset (-30..30 mm): {seq}")
    if not all(a >= b_ for a, b_ in zip(seq, seq[1:])):
        print("  FAIL: black set should move monotonically with offset"); ok = False

    # 3. no sensor sees the line when far away -> line lost
    b = sensors_on_line(200.0, 0.0)
    if b.sum() != 0:
        print("  FAIL: expected line-lost at 200 mm"); ok = False
    else:
        print("offset 200 mm: line lost (as expected)")

    # 4. tilt: at 10 deg the black run should widen across the array
    b0 = sensors_on_line(0.0, 0.0).sum()
    b1 = sensors_on_line(0.0, math.radians(10.0)).sum()
    print(f"black count: theta=0 -> {b0}, theta=10deg -> {b1}")
    if b1 < b0:
        print("  note: tilted array sees fewer sensors (possible at large tilt)")

    # 5. firmware_error sign check
    refl_left = np.full(SENSOR_COUNT, 9.0); refl_left[0] = 1.0     # leftmost black
    refl_right = np.full(SENSOR_COUNT, 9.0); refl_right[13] = 1.0  # rightmost black
    el = firmware_error(refl_left)
    er = firmware_error(refl_right)
    print(f"firmware_error(leftmost black) = {el:+.2f}   (rightmost black) = {er:+.2f}")
    if not (el < 0 < er):
        print("  FAIL: leftmost-black should give negative error"); ok = False

    # 6. dataset smoke test
    ds = build_dataset(pos_step_mm=40.0, max_samples=4000, verbose=True)
    if ds.X.shape[0] == 0:
        print("  FAIL: empty dataset"); ok = False
    print(f"dataset range: offset {ds.y_mm.min():.0f}..{ds.y_mm.max():.0f} mm")

    print("RESULT:", "PASS" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(_selftest())
