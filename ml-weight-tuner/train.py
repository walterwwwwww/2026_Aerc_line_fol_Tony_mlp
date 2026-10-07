#!/usr/bin/env python3
"""
train.py -- fit the MLP to the 10 shapes, optimise the firmware's 14 sensor
weights, and report the comparison. Writes model.json for export.py.

    python train.py            # full run
    python train.py --quick    # fast settings, for iterating

WHAT IS BEING COMPARED
----------------------
Two ways for the firmware to turn 14 reflectance readings into one steering
number:

  1. WEIGHTS MODE (what the robot does today)
     app_sensor_weights.h's sensor_pair_error() picks the first black pair
     (i, 14-1-i) and emits a value linear in the 14 weights. It is the baseline.

  2. MLP MODE (what this adds)
     A 14->16->1 network that regresses the lateral offset in millimetres.

The comparison is made fair deliberately:

  * The weights get their BEST POSSIBLE linear gain fitted (offset_to_firmware_
    error_scale). Without that, comparing a weight-unit number against a
    millimetre number would be meaningless and would flatter the MLP.

  * Three metrics are reported because MAE alone is misleading here.
      MAE (mm)      average positioning error -- what the steering loop sees
      R^2           how much of the offset variance is actually explained
      SIGN ACCURACY fraction of poses where the correction points the right way.
                    This is the one that decides whether the robot stays on the
                    line: magnitude errors are absorbed by the PID, but a wrong
                    sign steers INTO the error.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from mlp import (  # noqa: E402
    TrainedModel,
    mae_mm,
    r2,
    sign_accuracy,
    rmse_mm,
    train,
)
from sensor_model import (  # noqa: E402
    BLACK_THRESHOLD,
    DEFAULT_WEIGHTS,
    SENSOR_COUNT,
    build_dataset,
    firmware_error,
    offset_to_firmware_error_scale,
)
from shapes import build_shapes  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
MODEL_JSON = os.path.join(HERE, "model.json")


# ==========================================================================
#  Optimising the 14 firmware weights
# ==========================================================================
def firmware_error_matrix(X: np.ndarray, weights: np.ndarray) -> np.ndarray:
    """Vectorised firmware_error over a batch."""
    return firmware_error(X, weights)


def precompute_active_pairs(X: np.ndarray) -> np.ndarray:
    """
    For every row, which pair index (0..6) the firmware would select.

    The firmware breaks at the FIRST pair with a black sensor, scanning
    i = 0..6 with j = 13-i. Knowing the active pair turns the error into a
    piecewise-linear function of the weights, which is what makes an exact
    fit possible.
    """
    black = X <= BLACK_THRESHOLD
    n = X.shape[0]
    pair = np.full(n, -1, dtype=np.int32)
    for i in range(SENSOR_COUNT // 2):
        j = SENSOR_COUNT - 1 - i
        hit = (black[:, i] | black[:, j]) & (pair < 0)
        pair[hit] = i
    # rows where nothing is black never run the error body; mark as 7
    pair[pair < 0] = SENSOR_COUNT // 2
    return pair


def optimise_weights(
    X: np.ndarray,
    y_mm: np.ndarray,
    iters: int = 400,
    lr: float = 0.05,
    w_max: float = 100.0,
    seed: int = 0,
    verbose: bool = True,
) -> tuple:
    """
    Find the 14 sensor weights that make the firmware's error signal most
    proportional to the TRUE offset in millimetres.

    Method: Adam on the MSE between (gain * firmware_error) and the true offset,
    with the gain re-fitted every few steps. Gradient-free fallback (coordinate
    descent) is not needed because the objective is piecewise linear in the
    weights and therefore has usable sub-gradients almost everywhere; the
    finite-difference gradient used here is robust to the kinks.

    Constraint: weights are clamped to [0, w_max] -- SENSOR_WEIGHT_MAX is 100 in
    app_sensor_weights.h and negative weights make no physical sense (a sensor
    on the left must never push the error the other way).
    """
    rng = np.random.default_rng(seed)
    w = DEFAULT_WEIGHTS.copy()

    def loss_and_grad(wv: np.ndarray) -> tuple:
        e = firmware_error(X, wv)
        denom = float(np.dot(e, e)) + 1e-12
        g = float(np.dot(e, y_mm) / denom)          # best linear gain
        resid = g * e - y_mm
        L = float(np.mean(resid ** 2))
        # dL/dw = 2 * mean(resid * g * de/dw); use finite differences on the
        # 14 weights (cheap: 14 extra passes over a precomputed black matrix)
        grad = np.zeros_like(wv)
        eps = 1e-3
        for k in range(len(wv)):
            wp = wv.copy(); wp[k] += eps
            ep = firmware_error(X, wp)
            gp = float(np.dot(ep, y_mm) / (float(np.dot(ep, ep)) + 1e-12))
            Lp = float(np.mean((gp * ep - y_mm) ** 2))
            grad[k] = (Lp - L) / eps
        return L, grad, g

    m = np.zeros_like(w)
    v = np.zeros_like(w)
    b1, b2, eps = 0.9, 0.999, 1e-8
    best = (float("inf"), w.copy(), 0.0)

    for t in range(1, iters + 1):
        L, grad, gain = loss_and_grad(w)
        if L < best[0]:
            best = (L, w.copy(), gain)
        m = b1 * m + (1 - b1) * grad
        v = b2 * v + (1 - b2) * grad * grad
        mh = m / (1 - b1 ** t)
        vh = v / (1 - b2 ** t)
        w = w - lr * mh / (np.sqrt(vh) + eps)
        w = np.clip(w, 0.0, w_max)
        if verbose and t % 50 == 0:
            print(f"    weight-opt iter {t:4d}  loss {L:10.4f}  gain {gain:7.4f}")

    L, w, gain = best
    return w, gain, L


# ==========================================================================
#  Main
# ==========================================================================
def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--quick", action="store_true",
                    help="fast settings for iterating")
    ap.add_argument("--epochs", type=int, default=None)
    ap.add_argument("--hid", type=int, default=16)
    ap.add_argument("--seed", type=int, default=20261007)
    ap.add_argument("--no-weight-opt", action="store_true")
    args = ap.parse_args()

    t0 = time.time()
    print("=" * 74)
    print(" LineFollower ML weight tuner -- training")
    print("=" * 74)

    epochs = args.epochs if args.epochs is not None else (120 if args.quick else 700)
    pos_step = 12.0 if args.quick else 5.0
    max_samples = 20_000 if args.quick else 400_000

    # ---------------------------------------------------------------- data
    print(f"\n[1/5] generating dataset from the {len(build_shapes())} shapes ...")
    ds = build_dataset(pos_step_mm=pos_step, max_samples=max_samples,
                       seed=args.seed, verbose=True)
    n = ds.X.shape[0]
    rng = np.random.default_rng(args.seed)

    # split by SHAPE for the test set, so we measure generalisation to shapes the
    # net has never seen rather than memorisation of the training poses
    test_shapes = {1, 4, 9}          # S2 square, S5 chamfer, S10 rectangle
    is_test = np.isin(ds.shape_id, list(test_shapes))
    if is_test.sum() < 100:          # fall back to a random split
        is_test = rng.random(n) < 0.15
    Xtr, ytr = ds.X[~is_test], ds.y_mm[~is_test]
    Xte, yte = ds.X[is_test], ds.y_mm[is_test]
    print(f"      train {Xtr.shape[0]:,}   test {Xte.shape[0]:,} "
          f"(test shapes: {sorted(test_shapes)})")

    # ---------------------------------------------------------------- train
    print(f"\n[2/5] training MLP 14->{args.hid}->1 "
          f"({epochs} epochs) ...")
    model, hist = train(
        Xtr, ytr, Xte, yte,
        n_hid=args.hid, epochs=epochs, lr=8e-3, batch_size=1024,
        seed=args.seed, verbose=True, log_every=max(1, epochs // 8),
    )

    # --------------------------------------------------------------- metrics
    print("\n[3/5] evaluating ...")
    p_mlp_te = model.predict(Xte)
    mae_mlp = mae_mm(p_mlp_te, yte)
    rmse_mlp = rmse_mm(p_mlp_te, yte)
    r2_mlp = r2(p_mlp_te, yte)
    sa_mlp = sign_accuracy(p_mlp_te, yte)

    # --- baseline: existing hand-tuned weights, with their best linear gain
    fe_te = firmware_error(Xte, DEFAULT_WEIGHTS)
    gain0 = float(np.dot(fe_te, yte) / (float(np.dot(fe_te, fe_te)) + 1e-12))
    p_w0_te = gain0 * fe_te
    mae_w0 = mae_mm(p_w0_te, yte)
    r2_w0 = r2(p_w0_te, yte)
    sa_w0 = sign_accuracy(p_w0_te, yte)

    print(f"      baseline weights (as shipped), best gain {gain0:.4f}")
    print(f"        MAE {mae_w0:7.3f} mm   R2 {r2_w0:7.4f}   sign {sa_w0*100:5.1f}%")
    print(f"      MLP 14->{args.hid}->1")
    print(f"        MAE {mae_mlp:7.3f} mm   R2 {r2_mlp:7.4f}   sign {sa_mlp*100:5.1f}%")

    # ------------------------------------------------------- weight optimisation
    w_opt, gain_opt, _ = DEFAULT_WEIGHTS, gain0, 0.0
    p_wo_te = p_w0_te
    mae_wo = mae_w0
    r2_wo = r2_w0
    sa_wo = sa_w0
    if not args.no_weight_opt:
        print(f"\n[4/5] optimising the 14 firmware weights "
              f"({'300' if args.quick else '400'} iters) ...")
        w_opt, gain_opt, _ = optimise_weights(
            Xtr, ytr, iters=300 if args.quick else 400,
            verbose=True, seed=args.seed,
        )
        p_wo_te = gain_opt * firmware_error(Xte, w_opt)
        mae_wo = mae_mm(p_wo_te, yte)
        r2_wo = r2(p_wo_te, yte)
        sa_wo = sign_accuracy(p_wo_te, yte)
        print(f"      optimised weights, gain {gain_opt:.4f}")
        print(f"        MAE {mae_wo:7.3f} mm   R2 {r2_wo:7.4f}   sign {sa_wo*100:5.1f}%")
        print("      weights: " + ", ".join(f"{v:.2f}" for v in w_opt))

    # ------------------------------------------------------------- improvement
    imp_mae = (mae_w0 - mae_mlp) / max(mae_w0, 1e-9) * 100.0
    print(f"\n      MLP vs baseline: MAE {imp_mae:+.1f}%  "
          f"sign accuracy {(sa_mlp-sa_w0)*100:+.1f} points")

    # ---------------------------------------------------------------- persist
    print(f"\n[5/5] writing {os.path.basename(MODEL_JSON)} ...")
    out = {
        "format": "linfol-ml-model-v1",
        "created": time.strftime("%Y-%m-%dT%H:%M:%S"),
        "architecture": {"n_in": 14, "n_hid": args.hid, "n_out": 1,
                         "activation": "relu", "output": "linear"},
        "scaling": {"x_mean": model.x_mean, "x_std": model.x_std,
                    "y_mean": model.y_mean, "y_std": model.y_std},
        "W1": model.net.W1.tolist(),
        "b1": model.net.b1.tolist(),
        "W2": model.net.W2.ravel().tolist(),
        "b2": model.net.b2.ravel().tolist(),
        "weights_baseline": DEFAULT_WEIGHTS.tolist(),
        "weights_optimised": w_opt.tolist(),
        "weights_optimised_gain": gain_opt,
        "metrics": {
            "test_shapes": sorted(int(s) for s in test_shapes),
            "n_train": int(Xtr.shape[0]),
            "n_test": int(Xte.shape[0]),
            "baseline": {"mae_mm": mae_w0, "r2": r2_w0,
                         "sign_acc": sa_w0, "gain": gain0},
            "weights_optimised": {"mae_mm": mae_wo, "r2": r2_wo,
                                  "sign_acc": sa_wo, "gain": gain_opt},
            "mlp": {"mae_mm": mae_mlp, "rmse_mm": rmse_mlp, "r2": r2_mlp,
                    "sign_acc": sa_mlp, "n_params": model.n_params},
        },
        "dataset_meta": ds.meta,
        "train_meta": model.meta,
    }
    with open(MODEL_JSON, "w", encoding="utf-8") as f:
        json.dump(out, f, indent=2)

    print(f"      wrote {MODEL_JSON}")
    print(f"\ndone in {time.time()-t0:.1f}s")
    print("\nNext:  python export.py      (emit C headers + rebuild firmware)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
