"""ml_trainer.py -- supervised online weight tuning for the 上位机.

The MLP lives INSIDE the pid-tuner server. It keeps the 上位机's zero-magic
dependency rule: stdlib + pyserial + numpy only (no torch/sklearn).

THE SUPERVISED DESIGN
---------------------
An MLP that tunes sensor weights cannot learn from sensor data alone, because
the car cannot report its own lateral offset -- there is no sensor for it. So
the ground truth comes from the human, through the 10 physical test shapes:

    every recorded run is labelled with (shape, lateral_offset_mm).

The shapes' geometry is exact (see ml-weight-tuner/shapes.py), so when the car
is placed a known distance from the line, the true lateral offset of the sensor
array centre is known exactly. That is the supervision.

NETWORK (as specified: 5 layers, sigmoid)
-----------------------------------------
    28 -> 45 -> 22 -> 5 -> 1

    28 inputs  = 14 reflectance (0..1) + 14 binary "on the line" flags
    1 output   = lateral offset of the array centre from the line, in mm

        offset < 0  ->  array LEFT  of the line
        offset > 0  ->  array RIGHT of the line

FROM OFFSET BACK TO 14 WEIGHTS
------------------------------
The firmware does not run the network; it still runs its own sensor_pair_error()
law, whose 14 weights are what we are tuning. So after the network predicts the
offset for a frame, we invert that law to obtain the weight that WOULD produce
that offset, and that becomes the frame's contribution to the fitted weights.

For a centred/clean pose the firmware error is

    err = (refl_right - refl_left) * w        (both sensors dark)

so a predicted offset `e` maps to the weight `w = e / (refl_right - refl_left)`.
Across the labelled batch we regress the 14 weights against these implied
values, with the mirror-symmetry constraint (w[i] == w[13-i]) enforced by
averaging mirrored sensors. The result is a symmetric 14-vector that reproduces
the network's offset estimates on average and cannot introduce a fixed veer.

This keeps the car's control loop and its sensor_pair_error() law untouched --
only the 14 weight numbers change.
"""

from __future__ import annotations

import math
import threading
import time
from collections import deque
from dataclasses import dataclass
from typing import Deque, List, Optional, Tuple

import numpy as np

SENSOR_COUNT = 14
BLACK_THRESHOLD = 6.0
REFLECTANCE_SCALE = 10.0

LAYERS = (28, 45, 22, 5, 1)
ACTIVATION = "sigmoid"

# The stock symmetric geometry is kept as a SANITY REFERENCE / FALLBACK only;
# the fitted weights below are derived from data, not from this template.
STOCK_WEIGHTS = np.array(
    [6.5, 5.0, 4.0, 3.0, 2.0, 1.0, 0.1, 0.1, 1.0, 2.0, 3.0, 4.0, 5.0, 6.5],
    dtype=np.float64,
)

WEIGHT_MAX = 100.0


def sigmoid(z: np.ndarray) -> np.ndarray:
    out = np.empty_like(z, dtype=np.float64)
    pos = z >= 0.0
    out[pos] = 1.0 / (1.0 + np.exp(-z[pos]))
    ez = np.exp(z[~pos])
    out[~pos] = ez / (1.0 + ez)
    return out


def encode(reflectance: np.ndarray) -> np.ndarray:
    """(N,14) reflectance 0..10  ->  (N,28) network input."""
    r = np.atleast_2d(np.asarray(reflectance, dtype=np.float64))
    cont = r / REFLECTANCE_SCALE
    binary = (r <= BLACK_THRESHOLD).astype(np.float64)
    return np.concatenate([cont, binary], axis=1)


def pair_error(refl: np.ndarray, weights: np.ndarray) -> np.ndarray:
    """
    Vectorised transcription of sensor_pair_error() + first-black-pair scan.
    refl: (N,14); weights: (14,). Returns (N,) in the firmware's sign convention.
    """
    black = refl <= BLACK_THRESHOLD
    n = refl.shape[0]
    err = np.zeros(n)
    active = np.zeros(n, dtype=bool)
    for i in range(SENSOR_COUNT // 2):
        j = SENSOR_COUNT - 1 - i
        hit = (black[:, i] | black[:, j]) & ~active
        both = hit & black[:, i] & black[:, j]
        left_only = hit & black[:, i] & ~black[:, j]
        right_only = hit & black[:, j] & ~black[:, i]
        err[both] = (refl[both, j] - refl[both, i]) * weights[i]
        err[left_only] = -10.0 * weights[i]
        err[right_only] = 10.0 * weights[j]
        active |= hit
    return err


def offsets_to_weights(refl: np.ndarray, offsets: np.ndarray) -> np.ndarray:
    """
    Invert the firmware error law over a batch to recover the 14 weights that
    best reproduce the given lateral offsets, with mirror symmetry enforced.

    refl    : (N,14) reflectance
    offsets : (N,) true lateral offsets in mm, same sign convention as pair_error
              (left = negative).
    Returns (14,) symmetric weights.

    Each frame contributes an implied weight for the FIRST black pair (i, 13-i):
        both sensors dark : w = offset / (refl_right - refl_left)
        only left dark    : w = -offset / 10
        only right dark   : w = +offset / 10
    These are then averaged across frames per sensor index, mirror-averaged with
    sensor 13-i, clipped, and -- if a sensor never fired in any frame -- filled
    from its mirror or from the stock value.
    """
    black = refl <= BLACK_THRESHOLD
    n = refl.shape[0]
    acc = np.zeros(SENSOR_COUNT)
    cnt = np.zeros(SENSOR_COUNT)
    active = np.zeros(n, dtype=bool)

    for i in range(SENSOR_COUNT // 2):
        j = SENSOR_COUNT - 1 - i
        hit = (black[:, i] | black[:, j]) & ~active
        both = hit & black[:, i] & black[:, j]
        left_only = hit & black[:, i] & ~black[:, j]
        right_only = hit & black[:, j] & ~black[:, i]

        if both.any():
            denom = refl[both, j] - refl[both, i]
            # guard against near-zero reflectance difference (both fully dark)
            ok = np.abs(denom) > 1e-3
            acc[i] += np.sum(np.where(ok, offsets[both] / np.where(ok, denom, 1.0), 0.0))
            acc[j] += acc[i]
            cnt[i] += int(ok.sum())
            cnt[j] += int(ok.sum())
        if left_only.any():
            acc[i] += np.sum(-offsets[left_only] / 10.0)
            acc[j] += acc[i]
            cnt[i] += int(left_only.sum())
            cnt[j] += int(left_only.sum())
        if right_only.any():
            acc[j] += np.sum(offsets[right_only] / 10.0)
            acc[i] += acc[j]
            cnt[j] += int(right_only.sum())
            cnt[i] += int(right_only.sum())
        active |= hit

    w = np.where(cnt > 0, acc / np.maximum(cnt, 1.0), STOCK_WEIGHTS)
    # mirror symmetry hard-constraint
    for i in range(SENSOR_COUNT // 2):
        j = SENSOR_COUNT - 1 - i
        if cnt[i] > 0 and cnt[j] > 0:
            w[i] = w[j] = (w[i] + w[j]) / 2.0
        elif cnt[i] > 0:
            w[j] = w[i]
        elif cnt[j] > 0:
            w[i] = w[j]
    return np.clip(w, 0.0, WEIGHT_MAX)


class MLP:
    """5-layer sigmoid network: (28,45,22,5,1)."""

    def __init__(self, layers: Tuple[int, ...] = LAYERS, activation: str = ACTIVATION):
        self.layers = tuple(int(v) for v in layers)
        self.activation = activation
        rng = np.random.default_rng(0)
        self.W: List[np.ndarray] = []
        self.b: List[np.ndarray] = []
        for l in range(len(self.layers) - 1):
            n_in, n_out = self.layers[l], self.layers[l + 1]
            std = math.sqrt(2.0 / n_in) if activation == "relu" else math.sqrt(1.0 / n_in)
            self.W.append(rng.normal(0.0, std, size=(n_in, n_out)))
            self.b.append(np.zeros(n_out))
        self.W[-1] *= 0.5

    @property
    def n_params(self) -> int:
        return int(sum(p.size for p in self.W) + sum(p.size for p in self.b))

    def forward(self, X: np.ndarray, cache: bool = False):
        a = X
        acts = [a]
        zs: List[np.ndarray] = []
        L = len(self.W)
        for l in range(L):
            z = a @ self.W[l] + self.b[l]
            zs.append(z)
            a = sigmoid(z) if l < L - 1 else z
            acts.append(a)
        if cache:
            return a, (acts, zs)
        return a

    def backward(self, cache, dy: np.ndarray) -> List[np.ndarray]:
        """dy: summed-loss gradient wrt output; returns mean-loss grads."""
        acts, zs = cache
        N = acts[0].shape[0]
        L = len(self.W)
        gW: List[Optional[np.ndarray]] = [None] * L
        gb: List[Optional[np.ndarray]] = [None] * L
        delta = dy
        for l in reversed(range(L)):
            gW[l] = acts[l].T @ delta / N
            gb[l] = delta.sum(axis=0) / N
            if l > 0:
                da = delta @ self.W[l].T
                if self.activation == "sigmoid":
                    s = acts[l]
                    delta = da * s * (1.0 - s)
                else:
                    delta = da * (zs[l - 1] > 0.0)
        out: List[np.ndarray] = []
        for l in range(L):
            out.append(gW[l])  # type: ignore[arg-type]
            out.append(gb[l])  # type: ignore[arg-type]
        return out

    def params(self) -> List[np.ndarray]:
        out: List[np.ndarray] = []
        for l in range(len(self.W)):
            out.append(self.W[l])
            out.append(self.b[l])
        return out

    def set_params(self, values: List[np.ndarray]) -> None:
        for l in range(len(self.W)):
            self.W[l] = values[2 * l]
            self.b[l] = values[2 * l + 1]


@dataclass
class LabeledSample:
    reflectance: np.ndarray          # (14,)
    offset_mm: float                 # true lateral offset, left = negative


class OnlineWeightTrainer:
    """
    Holds labelled samples, trains the MLP to predict lateral offset, and
    derives symmetric 14 weights from the network's predictions.
    """

    def __init__(self, buffer_size: int = 8192, batch_size: int = 256,
                 lr: float = 3e-3, step_interval_s: float = 1.0,
                 min_buffer: int = 64):
        self.net = MLP()
        self.buffer_size = buffer_size
        self.batch_size = batch_size
        self.lr = lr
        self.step_interval_s = step_interval_s
        self.min_buffer = min_buffer

        self.lock = threading.RLock()
        self.buffer: Deque[LabeledSample] = deque(maxlen=buffer_size)
        self.running = False
        self.thread: Optional[threading.Thread] = None
        self.samples_seen = 0
        self.steps = 0
        self.loss = float("nan")
        self.offset_mae = float("nan")
        self.weights = STOCK_WEIGHTS.copy()
        self.error = ""
        self.current_label: Optional[float] = None
        self.current_shape: str = ""

        self.m: List[np.ndarray] = [np.zeros_like(p) for p in self.net.params()]
        self.v: List[np.ndarray] = [np.zeros_like(p) for p in self.net.params()]
        self.t = 0

    # ------------------------------------------------------------------ feed
    def set_label(self, offset_mm: float, shape: str = "") -> None:
        """Set the ground-truth label applied to subsequently fed frames."""
        with self.lock:
            self.current_label = float(offset_mm)
            self.current_shape = shape

    def clear_label(self) -> None:
        with self.lock:
            self.current_label = None
            self.current_shape = ""

    def feed(self, reflectance) -> bool:
        """Feed one frame; only accepted while a label is set. Returns accepted."""
        r = np.asarray(reflectance, dtype=np.float64).reshape(-1)
        if r.shape[0] != SENSOR_COUNT or not np.isfinite(r).all():
            return False
        with self.lock:
            if self.current_label is None:
                return False
            self.buffer.append(LabeledSample(r.copy(), self.current_label))
            self.samples_seen += 1
            return True

    # ------------------------------------------------------------------ life
    def start(self) -> None:
        with self.lock:
            if self.running:
                return
            self.running = True
        self.thread = threading.Thread(target=self._loop, daemon=True,
                                       name="ml-weight-trainer")
        self.thread.start()

    def stop(self) -> None:
        with self.lock:
            self.running = False
        if self.thread:
            self.thread.join(timeout=2.0)
            self.thread = None

    def reset(self) -> None:
        self.stop()
        with self.lock:
            self.buffer.clear()
            self.samples_seen = 0
            self.steps = 0
            self.loss = float("nan")
            self.offset_mae = float("nan")
            self.weights = STOCK_WEIGHTS.copy()
            self.net = MLP()
            self.m = [np.zeros_like(p) for p in self.net.params()]
            self.v = [np.zeros_like(p) for p in self.net.params()]
            self.t = 0

    # ------------------------------------------------------------------ loop
    def _loop(self) -> None:
        last = time.monotonic()
        while True:
            with self.lock:
                if not self.running:
                    return
                ready = len(self.buffer) >= self.min_buffer
            if not ready:
                time.sleep(0.2)
                continue
            now = time.monotonic()
            if now - last >= self.step_interval_s:
                try:
                    self._train_step()
                except Exception as e:
                    with self.lock:
                        self.error = str(e)
                last = now
            time.sleep(0.05)

    # --------------------------------------------------------------- training
    def _train_step(self) -> None:
        with self.lock:
            if len(self.buffer) < self.min_buffer:
                return
            samples = list(self.buffer)
        rng = np.random.default_rng()
        idx = rng.integers(0, len(samples), size=self.batch_size)
        refl = np.stack([samples[i].reflectance for i in idx])
        offs = np.array([samples[i].offset_mm for i in idx], dtype=np.float64)

        X = encode(refl)
        # normalise the offset target to zero-mean, unit-variance for stable
        # training, then un-normalise predictions later
        offs_norm = (offs - offs.mean()) / (offs.std() + 1e-9)
        pred, cache = self.net.forward(X, cache=True)   # (B,1)

        resid = pred.ravel() - offs_norm
        loss = float(np.mean(resid ** 2))
        dy = (2.0 * resid[:, None])                       # summed-loss grad
        grads = self.net.backward(cache, dy)

        b1, b2, eps = 0.9, 0.999, 1e-8
        self.t += 1
        params = self.net.params()
        for i in range(len(params)):
            self.m[i] = b1 * self.m[i] + (1 - b1) * grads[i]
            self.v[i] = b2 * self.v[i] + (1 - b2) * (grads[i] ** 2)
            mh = self.m[i] / (1 - b1 ** self.t)
            vh = self.v[i] / (1 - b2 ** self.t)
            params[i] -= self.lr * mh / (np.sqrt(vh) + eps)

        # un-normalise predictions -> mm, and derive weights from them
        pred_mm = pred.ravel() * (offs.std() + 1e-9) + offs.mean()
        mae = float(np.mean(np.abs(pred_mm - offs)))
        w = offsets_to_weights(refl, pred_mm)

        with self.lock:
            self.weights = w
            self.loss = loss
            self.offset_mae = mae
            self.steps += 1

    # ------------------------------------------------------------------ state
    def status(self) -> dict:
        with self.lock:
            return dict(
                running=self.running,
                buffer=len(self.buffer),
                samples_seen=self.samples_seen,
                steps=self.steps,
                loss=self.loss,
                offset_mae_mm=self.offset_mae,
                label=self.current_label,
                shape=self.current_shape,
                weights=[round(float(w), 4) for w in self.weights],
                error=self.error,
            )
