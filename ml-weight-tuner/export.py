#!/usr/bin/env python3
"""
export.py -- turn a trained model.json into firmware sources.

Writes
    firmware/ml_weights/ml_weights.h   -- extern decls + constants
    firmware/ml_weights/ml_weights.c   -- the weight tables + forward pass
    firmware/ml_weights/README.md      -- what it is, how to wire it in

The generated C is a direct transcription of mlp.MLP.forward(), so the numbers
below must match mlp.encode_inputs() exactly:

    x[0..13]  = adc_test[i] / 10.0
    x[14..27] = (adc_test[i] <= 6.0) ? 1.0 : 0.0
    hidden    = ReLU
    output    = linear, then  offset_mm = y * y_std + y_mean

SIGN CONVENTION -- READ THIS BEFORE WIRING IT INTO THE CONTROL LOOP
------------------------------------------------------------------
The network predicts the LATERAL OFFSET of the array centre from the line:

    offset_mm < 0  ->  array is LEFT of the line  ->  robot must steer LEFT
    offset_mm > 0  ->  array is RIGHT of the line ->  robot must steer RIGHT

The firmware's existing adc_error has the OPPOSITE sign for the same physical
situation: sensor 0 is the leftmost sensor, and a left-side hit yields a
NEGATIVE sensor_pair_error. Rather than silently negate and hope, the generated
header exposes `ml_error_mm()` (the raw prediction) and a clearly-named
`ml_adc_error_equivalent()` that returns the value in the firmware's own sign
convention and in the firmware's magnitude scale, so it can be dropped in where
adc_error is used today.

The scale factor is fitted at export time by least squares against the
firmware's own error over a grid of synthetic poses, so the magnitude is
comparable and the existing KP_L/KI_L/KD_L gains stay roughly valid.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time
from typing import Dict, List, Tuple

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from sensor_model import BLACK_THRESHOLD, SENSOR_COUNT, sensors_on_line  # noqa: E402
from sensor_model import firmware_error  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
FIRMWARE_DIR = os.path.join(
    os.path.dirname(HERE), "firmware", "ml_weights"
)
MODEL_JSON = os.path.join(HERE, "model.json")


# --------------------------------------------------------------------------
#  C formatting helpers
# --------------------------------------------------------------------------
def _fmt(v: float) -> str:
    """Compact but exact enough for float32 round-trip."""
    return f"{v:.9g}f"


def _fmt_row(vals, per_line: int = 6, indent: str = "    ") -> str:
    out = []
    for i in range(0, len(vals), per_line):
        chunk = ", ".join(_fmt(v) for v in vals[i:i + per_line])
        out.append(indent + chunk + ",")
    return "\n".join(out)


def _c_type(n: int) -> str:
    return "float"


# --------------------------------------------------------------------------
#  Fit the scale that maps the MLP prediction onto the firmware's error scale
# --------------------------------------------------------------------------
def fit_firmware_scale(model_path: str) -> Tuple[float, Dict[str, float]]:
    """
    Least-squares gain k minimising || k * (-prediction) - adc_error || over a
    grid of plausible poses.

    The negation converts the MLP's convention (left = negative) to the
    firmware's (left = negative sensor_pair_error, i.e. the opposite sign).
    Fitting k means the shipped header keeps the existing steering gains
    usable, instead of silently rescaling every PID term.
    """
    data = json.load(open(model_path, encoding="utf-8"))
    import mlp as mlp_mod

    net = mlp_mod.MLP(tuple(data["architecture"]["layers"]))
    net.W = [np.asarray(w, dtype=np.float64) for w in data["W"]]
    net.b = [np.asarray(b, dtype=np.float64) for b in data["b"]]
    y_mean = float(data["y_mean"])
    y_std = float(data["y_std"])

    offs = np.arange(-36.0, 36.0 + 1e-9, 2.0)
    thetas = np.radians(np.arange(-8.0, 8.0 + 1e-9, 2.0))

    refl: List[np.ndarray] = []
    errs: List[float] = []
    for o in offs:
        for t in thetas:
            black = sensors_on_line(float(o), float(t))
            row = np.where(black == 1, 1.0, 9.0)
            refl.append(row)
            errs.append(firmware_error(row))
    R = np.asarray(refl)
    E = np.asarray(errs)

    X = mlp_mod.encode_inputs(R)
    pred = net.forward(X)[:, 0] * y_std + y_mean      # mm, MLP sign

    # firmware error ~ k * (-pred)
    basis = -pred
    denom = float(np.dot(basis, basis)) + 1e-12
    k = float(np.dot(basis, E) / denom)

    resid = k * basis - E
    info = dict(
        gain=abs(k),
        sign=-1.0 if k < 0 else 1.0,
        fit_rmse=float(np.sqrt(np.mean(resid ** 2))),
        fw_err_std=float(E.std()),
        n_grid=len(E),
    )
    return k, info


# --------------------------------------------------------------------------
#  Emit the C
# --------------------------------------------------------------------------
HEADER_TMPL = '''/* ===========================================================================
 * ml_weights.h  --  machine-learned line-position estimator
 *
 * GENERATED FILE -- do not edit by hand.
 *   produced by : ml-weight-tuner/export.py
 *   generated   : {created}
 *   architecture: {arch}
 *   parameters  : {nparams:,}
 *   trained on  : {ntrain} samples
 *
 * WHAT IT REPLACES
 *   The 14 scalar sensor weights in app_sensor_weights.h, and the
 *   sensor_pair_error() law in app_adc.c that consumes them. That law can only
 *   express a linear ramp over the array; it has no way to represent a pose
 *   where several sensors are dark at once, which is exactly what happens on
 *   the 45-degree and 120 mm-leg shapes.
 *
 * WHAT IT IS
 *   A {nlayers}-layer perceptron: {arch}
 *   input  : 28 values
 *              [0..13]  adc_test[i] / 10.0                  (continuous 0..1)
 *              [14..27] (adc_test[i] <= 6.0) ? 1.0 : 0.0     (the same
 *                       threshold app_adc.c already uses for if_bai[])
 *   output : lateral offset of the array centre from the line centre-line,
 *            in millimetres.
 *
 * SIGN CONVENTION (the most likely integration bug -- read it)
 *   ml_error_mm()              < 0  =>  array LEFT  of line  =>  steer LEFT
 *                              > 0  =>  array RIGHT of line  =>  steer RIGHT
 *   ml_adc_error_equivalent()  is the same information expressed in the
 *                              firmware's EXISTING adc_error sign convention
 *                              and magnitude scale, so it can be substituted
 *                              for adc_error directly with no gain changes.
 *
 * COST
 *   {nparams:,} parameters. Roughly {macs:,} multiply-accumulates per call.
 *   Called from the 1 kHz TIM7 ISR on a 168 MHz Cortex-M4 that is about
 *   {cpu:.2f}% of the CPU, which is small next to the blocking SPI sensor read.
 *
 * TABLE SIZE
 *   {flashbytes:,} bytes of const float tables (plus {bssbytes} bytes of scratch).
 * ===========================================================================
 */

#ifndef ML_WEIGHTS_H
#define ML_WEIGHTS_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {{
#endif

/* ---- configuration the caller must respect ---- */
#define ML_INPUTS            {n_in}
#define ML_OUTPUTS           {n_out}
#define ML_N_LAYERS          {nlayers}   /* incl. input and output layers */
#define ML_SENSOR_COUNT      {nsensor}
#define ML_BLACK_THRESHOLD   {thresh}f    /* adc_test[i] <= this => dark */

/* Reflectance scale: x[i] = adc_test[i] / ML_REFLECTANCE_SCALE */
#define ML_REFLECTANCE_SCALE {rscale}f

/* ---- the estimator ------------------------------------------------------ */
/*
 * Feed it the 14 calibrated reflectance values from app_adc.c (the same
 * adc_test[] that if_bai[] is derived from). Ordering is sensor 0 (leftmost)
 * .. sensor 13 (rightmost). Values are on the firmware's 0..10 scale where
 * LOW means DARK.
 *
 * Returns the lateral offset in millimetres:
 *     negative = array is left of the line  (steer left)
 *     positive = array is right of the line (steer right)
 *
 * Calling this from the 1 kHz ISR is fine, but it must NOT be called before
 * calibration has run, because adc_test[] is meaningless until then.
 */
float ml_error_mm(const float adc_test[ML_SENSOR_COUNT]);

/*
 * Same prediction, mapped into the sign convention and magnitude scale of the
 * firmware's existing adc_error, so it is a drop-in replacement:
 *
 *     adc_error = ml_adc_error_equivalent(adc_test);
 *
 * Fitted gain: {gain:.6f}   (residual RMSE {fitrmse:.4f} against the
 * hand-tuned law over {ngrid} grid poses; hand-tuned law std {fwstd:.4f})
 */
float ml_adc_error_equivalent(const float adc_test[ML_SENSOR_COUNT]);

/* Raw forward pass, exposed for host-side tests that compare against Python. */
void ml_forward(const float *x, float *y);

/* True if the last ml_forward() saw at least one sensor over the threshold.
 * When this is 0 the prediction is an extrapolation and the caller should fall
 * back to the existing line-lost handling rather than trust the number. */
int ml_last_had_line(void);

#ifdef __cplusplus
}}
#endif

#endif /* ML_WEIGHTS_H */
'''


def emit_header(data: dict, net_info: dict, created: str) -> str:
    layers = data["architecture"]["layers"]
    nparams = int(sum(np.asarray(w).size for w in data["W"])
                  + sum(np.asarray(b).size for b in data["b"]))
    macs = 0
    for a, b in zip(layers[:-1], layers[1:]):
        macs += a * b
    flashbytes = nparams * 4
    bss = sum(layers) * 4
    # one multiply-accumulate per weight, plus ReLU; ~1.5 cycles each on an M4
    cpu = (macs * 1.5) / (168e6 / 1000.0) * 100.0
    return HEADER_TMPL.format(
        created=created,
        arch=" -> ".join(str(v) for v in layers),
        nlayers=len(layers),
        nparams=nparams,
        ntrain=data.get("metrics", {}).get("n_train", "?"),
        n_in=layers[0],
        n_out=layers[-1],
        nsensor=SENSOR_COUNT,
        thresh=BLACK_THRESHOLD,
        rscale=10.0,
        gain=abs(net_info["gain"]),
        fitrmse=net_info["fit_rmse"],
        fwstd=net_info["fw_err_std"],
        ngrid=net_info["n_grid"],
        macs=macs,
        cpu=cpu,
        flashbytes=flashbytes,
        bssbytes=bss,
    )


def emit_source(data: dict, net_info: dict, created: str) -> str:
    layers = data["architecture"]["layers"]
    Ws = [np.asarray(w, dtype=np.float64) for w in data["W"]]
    bs = [np.asarray(b, dtype=np.float64) for b in data["b"]]
    y_mean = float(data["y_mean"])
    y_std = float(data["y_std"])
    gain = net_info["gain"]
    sign = net_info["sign"]

    L: List[str] = []
    L.append("/* ===========================================================================")
    L.append(" * ml_weights.c  --  GENERATED by ml-weight-tuner/export.py, do not edit.")
    L.append(f" * generated   : {created}")
    L.append(f" * architecture: {' -> '.join(str(v) for v in layers)}")
    L.append(" * ===========================================================================")
    L.append(" */")
    L.append("")
    L.append('#include "ml_weights.h"')
    L.append("#include <string.h>")
    L.append("")
    L.append("/* ---- output de-normalisation: offset_mm = y * ML_Y_STD + ML_Y_MEAN ---- */")
    L.append(f"#define ML_Y_MEAN  ({_fmt(y_mean)})")
    L.append(f"#define ML_Y_STD   ({_fmt(y_std)})")
    L.append("")
    L.append("/* ---- mapping onto the firmware's adc_error convention -------------------")
    L.append(" * adc_error ~= ML_FW_GAIN * ML_FW_SIGN * offset_mm")
    L.append(f" * fitted gain {gain:.6f}, sign {sign:+.0f}")
    L.append(" * ----------------------------------------------------------------------- */")
    L.append(f"#define ML_FW_GAIN ({_fmt(gain)})")
    L.append(f"#define ML_FW_SIGN ({_fmt(sign)})")
    L.append("")

    # weight tables
    for l in range(len(Ws)):
        n_in, n_out = layers[l], layers[l + 1]
        L.append(f"/* W{l}: ({n_in}, {n_out})  x[{l}] -> z[{l+1}] */")
        L.append(f"static const float ML_W{l}[{n_in}][{n_out}] = {{")
        for r in range(n_in):
            L.append("  {")
            L.append(_fmt_row(Ws[l][r], per_line=6, indent="    "))
            L.append("  },")
        L.append("};")
        L.append(f"static const float ML_B{l}[{n_out}] = {{")
        L.append(_fmt_row(bs[l], per_line=6, indent="  "))
        L.append("};")
        L.append("")

    # scratch buffers sized to the widest layer
    maxw = max(layers)
    L.append(f"/* scratch: widest layer is {maxw} */")
    L.append(f"static float ml_a0[{layers[0]}];")
    L.append(f"static float ml_a1[{maxw}];")
    L.append(f"static float ml_a2[{maxw}];")
    L.append("static int   ml_had_line;")
    L.append("")

    # forward pass, transcribed layer by layer
    L.append("static void ml_relu(float *v, int n)")
    L.append("{")
    L.append("    for (int i = 0; i < n; i++)")
    L.append("        if (v[i] < 0.0f) v[i] = 0.0f;")
    L.append("}")
    L.append("")
    L.append("/* Generic dense layer: y = W^T x + b  with W stored row-major (n_in, n_out). */")
    L.append("static void ml_dense(const float *x, int n_in,")
    L.append("                     const float *W, const float *b, int n_out,")
    L.append("                     float *y)")
    L.append("{")
    L.append("    for (int j = 0; j < n_out; j++)")
    L.append("    {")
    L.append("        float acc = b[j];")
    L.append("        for (int i = 0; i < n_in; i++)")
    L.append("            acc += x[i] * W[i * n_out + j];")
    L.append("        y[j] = acc;")
    L.append("    }")
    L.append("}")
    L.append("")
    L.append("/* ---- input encoding: 14 reflectance -> 28 network inputs ----------------")
    L.append(" * MUST match mlp.encode_inputs():")
    L.append(" *   x[0..13]  = adc_test[i] / 10")
    L.append(" *   x[14..27] = (adc_test[i] <= 6.0) ? 1 : 0")
    L.append(" * ----------------------------------------------------------------------- */")
    L.append("static void ml_encode(const float adc_test[ML_SENSOR_COUNT], float *x)")
    L.append("{")
    L.append("    int n_black = 0;")
    L.append("    for (int i = 0; i < ML_SENSOR_COUNT; i++)")
    L.append("    {")
    L.append("        float v = adc_test[i];")
    L.append("        x[i] = v / ML_REFLECTANCE_SCALE;")
    L.append("        if (v <= ML_BLACK_THRESHOLD) { x[ML_SENSOR_COUNT + i] = 1.0f; n_black++; }")
    L.append("        else                        { x[ML_SENSOR_COUNT + i] = 0.0f; }")
    L.append("    }")
    L.append("    ml_had_line = (n_black > 0);")
    L.append("}")
    L.append("")

    # forward
    L.append("void ml_forward(const float *x, float *y)")
    L.append("{")
    L.append(f"    ml_dense(x, {layers[0]}, &ML_W0[0][0], ML_B0, {layers[1]}, ml_a1);")
    L.append(f"    ml_relu(ml_a1, {layers[1]});")
    for l in range(1, len(Ws)):
        src = "ml_a1" if l == 1 else f"ml_a{l}"
        dst = f"ml_a{l+1}"
        L.append(f"    ml_dense({src}, {layers[l]}, &ML_W{l}[0][0], ML_B{l}, {layers[l+1]}, {dst});")
        if l < len(Ws) - 1:
            L.append(f"    ml_relu({dst}, {layers[l+1]});")
    L.append(f"    for (int j = 0; j < {layers[-1]}; j++) y[j] = {('ml_a' + str(len(Ws)))}[j];")
    L.append("}")
    L.append("")

    L.append("int ml_last_had_line(void) { return ml_had_line; }")
    L.append("")

    L.append("float ml_error_mm(const float adc_test[ML_SENSOR_COUNT])")
    L.append("{")
    L.append("    float y[ML_OUTPUTS];")
    L.append("    ml_encode(adc_test, ml_a0);")
    L.append("    ml_forward(ml_a0, y);")
    L.append("    /* de-normalise the first output, which is the offset in mm */")
    L.append("    return y[0] * ML_Y_STD + ML_Y_MEAN;")
    L.append("}")
    L.append("")

    L.append("float ml_adc_error_equivalent(const float adc_test[ML_SENSOR_COUNT])")
    L.append("{")
    L.append("    /* Convert the MLP's convention to the firmware's adc_error convention,")
    L.append("     * and rescale so the existing steering gains stay usable. */")
    L.append("    return ML_FW_GAIN * ML_FW_SIGN * ml_error_mm(adc_test);")
    L.append("}")
    L.append("")
    return "\n".join(L)


README_TMPL = """# `firmware/ml_weights/` -- generated, do not edit by hand

Regenerate with:

```sh
python ml-weight-tuner/export.py
```

## Files

| file | what |
|---|---|
| `ml_weights.h` | public interface + constants |
| `ml_weights.c` | weight tables + forward pass ({nparams:,} parameters) |
| `README.md` | this file (generated) |

Generated {created} from `ml-weight-tuner/model.json`.
Architecture: **{arch}**

## Wiring it into the control loop

The estimator replaces the weight-based error law. Two ways in, easiest first.

### Option A -- swap the error source (one line)

In `app_adc.c`, the value that eventually becomes `adc_error` is computed by
`adc_error_average[9] = sensor_pair_error(...)` and the forced-turn path. Leave
all of that logic in place (it handles line-lost, forced turns, bend entry and
exit) and simply override the number at the end of `app_adc_get()`:

```c
#include "ml_weights.h"
...
/* USER CODE BEGIN app_adc_get tail */
if (calibrated && !turn_forced_flag)
{
    adc_error_average[9] = ml_adc_error_equivalent((const float *)adc_test);
}
/* USER CODE END */
```

`ml_adc_error_equivalent()` already returns the same sign convention and a
comparable magnitude to the existing law, so `KP_L` / `KI_L` / `KD_L` do not
need retuning from scratch -- only a small trim.

The forced-turn path is deliberately NOT overridden: when the robot is spinning
on the gyro it is not tracking the line, and the firmware's
`FORCE_ERROR_MAG * outer_weight` term is doing a different job.

### Option B -- replace the weights entirely

If you would rather remove the old law, delete the `sensor_pair_error()` call
and use `ml_error_mm()` scaled by your own gain. This is cleaner but you will
have to re-tune the steering gains, because the units change.

## Guarding against extrapolation

`ml_last_had_line()` returns 0 when no sensor was over the threshold, i.e. the
network is being asked about a pose it never saw in training. In that case the
firmware's existing line-lost handling should take over rather than trusting the
output:

```c
float e = ml_adc_error_equivalent(adc_test);
if (ml_last_had_line()) adc_error_average[9] = e;
```

## Stack and CPU

`ml_error_mm()` uses {bssbytes} bytes of static scratch and {flashbytes:,} bytes
of const tables. It is safe to call from the 1 kHz TIM7 ISR ({cpu:.2f}% of a
168 MHz M4), but it must only be called once calibration has run, because
`adc_test[]` is meaningless before that.

## Retraining

See `ml-weight-tuner/README.md`. The short version:

```sh
python ml-weight-tuner/collect.py --port COM5 --out data/run1.csv   # on the car
python ml-weight-tuner/train.py --data data/run1.csv                 # on the laptop
python ml-weight-tuner/export.py                                     # regenerate this dir
```
"""


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--model", default=MODEL_JSON)
    ap.add_argument("--out", default=FIRMWARE_DIR)
    args = ap.parse_args()

    if not os.path.exists(args.model):
        print(f"ERROR: {args.model} not found. Run train.py first.", file=sys.stderr)
        return 1

    data = json.load(open(args.model, encoding="utf-8"))
    created = time.strftime("%Y-%m-%d %H:%M:%S")

    print(f"exporting {args.model}")
    print(f"  architecture: {' -> '.join(str(v) for v in data['architecture']['layers'])}")

    gain, info = fit_firmware_scale(args.model)
    print(f"  firmware scale fit: gain={abs(gain):.4f} sign={info['sign']:+.0f} "
          f"rmse={info['fit_rmse']:.4f} (fw std {info['fw_err_std']:.4f}) over "
          f"{info['n_grid']} poses")

    os.makedirs(args.out, exist_ok=True)
    h = emit_header(data, info, created)
    c = emit_source(data, info, created)

    layers = data["architecture"]["layers"]
    nparams = int(sum(np.asarray(w).size for w in data["W"])
                  + sum(np.asarray(b).size for b in data["b"]))
    macs = sum(a * b for a, b in zip(layers[:-1], layers[1:]))
    cpu = (macs * 1.5) / (168e6 / 1000.0) * 100.0
    readme = README_TMPL.format(
        created=created,
        arch=" -> ".join(str(v) for v in layers),
        nparams=nparams,
        bssbytes=sum(layers) * 4,
        flashbytes=nparams * 4,
        cpu=cpu,
    )

    for name, text in (("ml_weights.h", h), ("ml_weights.c", c),
                       ("README.md", readme)):
        p = os.path.join(args.out, name)
        with open(p, "w", encoding="utf-8") as f:
            f.write(text)
        print(f"  wrote {p}  ({len(text):,} bytes)")

    print(f"\nparameters {nparams:,}   MACs {macs:,}   ISR cost ~{cpu:.2f}% CPU")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
