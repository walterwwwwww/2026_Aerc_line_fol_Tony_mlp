#!/usr/bin/env python3
"""
mlp.py -- a general multilayer perceptron in pure NumPy (manual forward and
backward passes) with Adam, plus the training loop.

WHY NOT PYTORCH / SCIKIT-LEARN
------------------------------
 1. No dependency. The bundled Python here has NumPy but not torch or sklearn.
 2. The forward pass has to be transcribed into C for the STM32 anyway
    (export.py writes it out). Writing it by hand here means the C version is a
    direct transcription rather than a re-derivation from framework docs, which
    removes the most likely place for a silent train/inference mismatch.

ARCHITECTURE (as specified)
---------------------------
    28 -> 45 -> 22 -> 5 -> n

That is five layers in the colloquial sense: an input layer of 28, three hidden
layers (45, 22, 5) and an output layer of n. Internally that is four weight
matrices.

INPUT VECTOR (28) -- why 28 and not 14
--------------------------------------
The array gives 14 calibrated reflectance values on a 0..10 scale, where LOW
means dark. The network is fed

    x[0..13]  = reflectance_i / 10          (continuous, 0..1)
    x[14..27] = 1 if reflectance_i <= 6.0 else 0   (the firmware's own threshold)

The binary half matters: the firmware's whole error law is driven by WHICH
sensors crossed the threshold, and a threshold is a hard nonlinearity that a
small network approximates badly if it only ever sees the continuous value. The
continuous half supplies sub-threshold magnitude that the firmware throws away.
Together they are the natural 28-wide descriptor of the array state.

OUTPUT (n)
----------
n is a parameter (`--n-out`). The default, n = 1, is the one that matters:
the lateral offset of the array centre from the line centre-line, in
millimetres. That single number is what the steering loop needs, and it is what
the 14 firmware weights are a crude approximation of.

n > 1 is supported for auxiliary targets (offset, heading error, confidence,
...) and for experimenting; only output 0 is used by the firmware.

SIGN CONVENTION
---------------
    offset < 0  ->  array centre is to the LEFT of the line  -> steer left
    offset > 0  ->  array centre is to the RIGHT of the line -> steer right
export.py is responsible for mapping this onto the firmware's adc_error sign.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

# The architecture specified for this project.
DEFAULT_LAYERS: Tuple[int, ...] = (28, 45, 22, 5)

# Hidden-layer activation. "sigmoid" or "relu".
#
# Why this is a knob rather than hard-coded:
#   sigmoid  -- bounded 0..1, smooth, every unit contributes everywhere. Needs
#               small (Xavier) initial weights or it saturates immediately and
#               stops learning. Slower to train but very predictable, and it
#               cannot produce the dead-unit failure mode of ReLU.
#   relu     -- trains faster and is cheaper on the MCU, but units can die.
# On a Cortex-M4 without an FPU-accelerated exp(), sigmoid also costs a
# libm expf() per unit: 45+22+5 = 72 expf calls per inference. At the 1 kHz
# control rate that is worth measuring rather than assuming (see
# export.py's cost report, which prints both variants).
DEFAULT_ACTIVATION = "sigmoid"

# Input preprocessing constants -- must match ml_weights.c exactly.
REFLECTANCE_SCALE = 10.0     # reflectance is 0..10, so divide to get 0..1
BLACK_THRESHOLD = 6.0        # firmware: adc_test[i] <= 6.0  =>  sensor on line


# --------------------------------------------------------------------------
#  Activations
# --------------------------------------------------------------------------
def sigmoid(z: np.ndarray) -> np.ndarray:
    """
    Numerically stable logistic sigmoid.

    The textbook 1/(1+exp(-z)) overflows for large negative z because exp(-z)
    blows up. Splitting on the sign keeps the exponent argument <= 0 always, so
    no overflow is possible. This matters here because early in training, with
    un-normalised inputs, z routinely reaches +/-40.
    """
    out = np.empty_like(z, dtype=np.float64)
    pos = z >= 0
    out[pos] = 1.0 / (1.0 + np.exp(-z[pos]))
    ez = np.exp(z[~pos])
    out[~pos] = ez / (1.0 + ez)
    return out


def apply_activation(name: str, z: np.ndarray) -> np.ndarray:
    if name == "sigmoid":
        return sigmoid(z)
    if name == "relu":
        return np.maximum(z, 0.0)
    raise ValueError(f"unknown activation {name!r}")


# --------------------------------------------------------------------------
#  Loss gradient
# --------------------------------------------------------------------------
def mse_grad(pred: np.ndarray,
             target: np.ndarray,
             sample_weights: Optional[np.ndarray] = None) -> np.ndarray:
    """
    d(summed weighted squared error)/d(pred), i.e. WITHOUT the 1/N factor.

    This exists because MLP.backward() divides its accumulated gradients by N
    itself, so it expects a gradient of the SUMMED loss. Supplying the gradient
    of the MEAN loss instead silently scales every gradient by 1/N, which makes
    training look like it is merely slow rather than broken.

    Use this helper rather than writing dy by hand.
    """
    err = np.asarray(pred, dtype=np.float64) - np.asarray(target, dtype=np.float64)
    if sample_weights is None:
        return 2.0 * err
    return 2.0 * np.asarray(sample_weights, dtype=np.float64)[:, None] * err


# ==========================================================================
#  The network
# ==========================================================================
@dataclass
class MLP:
    """
    Fully-connected network, ReLU on every hidden layer, linear output.

    layers: (n_in, h1, h2, ..., hk, n_out). For the project default that is
            (28, 45, 22, 5, 1) -> four weight matrices.

    Weights are held in lists W[0..L-1], b[0..L-1] where
        W[l] : (layers[l], layers[l+1])
    """

    layers: Sequence[int] = DEFAULT_LAYERS + (1,)
    activation: str = DEFAULT_ACTIVATION
    W: List[np.ndarray] = field(default_factory=list)
    b: List[np.ndarray] = field(default_factory=list)

    def __post_init__(self) -> None:
        self.layers = tuple(int(v) for v in self.layers)
        if len(self.layers) < 2:
            raise ValueError("need at least input and output layers")
        if self.activation not in ("sigmoid", "relu"):
            raise ValueError(f"unknown activation {self.activation!r}")
        if not self.W:
            rng = np.random.default_rng(0)
            for l in range(len(self.layers) - 1):
                n_in, n_out = self.layers[l], self.layers[l + 1]
                if self.activation == "relu":
                    # He: correct for ReLU, which zeroes half the distribution
                    std = math.sqrt(2.0 / n_in)
                else:
                    # Xavier/Glorot: correct for sigmoid/tanh, which squash the
                    # distribution. Using He instead would push most sigmoid
                    # units straight into saturation at initialisation and the
                    # network would barely train.
                    std = math.sqrt(1.0 / n_in)
                self.W.append(rng.normal(0.0, std, size=(n_in, n_out)))
                self.b.append(np.zeros(n_out))
            # scale the last layer down so initial predictions are not wild
            self.W[-1] *= 0.5

    # ------------------------------------------------------------- properties
    @property
    def n_layers(self) -> int:
        return len(self.layers)

    @property
    def n_in(self) -> int:
        return self.layers[0]

    @property
    def n_out(self) -> int:
        return self.layers[-1]

    @property
    def n_hidden(self) -> Tuple[int, ...]:
        return tuple(self.layers[1:-1])

    def n_params(self) -> int:
        return int(sum(p.size for p in self.W) + sum(p.size for p in self.b))

    # ---------------------------------------------------------------- forward
    def forward(self, X: np.ndarray, cache: bool = False):
        """
        X: (N, n_in)  ->  (N, n_out)

        The activation is applied to every layer EXCEPT the last, which stays
        linear. A bounded output activation would be wrong here: the target is a
        signed offset in millimetres, and clamping it to 0..1 (sigmoid) or -1..1
        (tanh) would make it impossible to represent the outer poses where the
        robot is nearly off the line.

        cache=True additionally returns (activations, pre_activations).
        """
        a = X
        acts = [a]
        zs: List[np.ndarray] = []
        L = len(self.W)
        for l in range(L):
            z = a @ self.W[l] + self.b[l]
            zs.append(z)
            if l < L - 1:
                a = apply_activation(self.activation, z)
            else:
                a = z                       # linear output
            acts.append(a)
        if cache:
            return a, (acts, zs)
        return a

    # --------------------------------------------------------------- backward
    def backward(self, cache, dy: np.ndarray) -> List[np.ndarray]:
        """
        dy: (N, n_out) gradient of the SUMMED loss wrt the network output.

        NOTE the "summed": this method divides by N internally, at the gW/gb
        accumulation step, which is what makes the returned gradients those of
        the MEAN loss. Passing the mean-loss gradient here double-divides by N
        and scales every gradient by 1/N. Use mse_grad() to build dy.
        Returns flat list [gW0, gb0, gW1, gb1, ...], already averaged over N.

        Index bookkeeping, which is where this kind of code usually goes wrong:
            acts[0]   = network input
            zs[l]     = acts[l] @ W[l] + b[l]        (pre-activation, layer l)
            acts[l+1] = f(zs[l])                     (its activation)

        The output layer is LINEAR, so dL/dz[L-1] == dy exactly, with no
        activation derivative. Then for each layer l, delta holds dL/dz[l] and
        we need dL/dz[l-1]:

            dL/da[l]   = delta @ W[l]^T      <-- W[l] is the matrix that PRODUCED
                                                 z[l] from a[l], so the transpose
                                                 of the same index takes us back
            dL/dz[l-1] = dL/da[l] * f'(z[l-1])

        Applying f' to z[l] instead of z[l-1] is the classic off-by-one here. It
        is invisible for ReLU (a diagonal 0/1 mask commutes with the linear map)
        but plainly wrong for sigmoid -- which is exactly why the numerical
        gradient check in __main__ exists.
        """
        acts, zs = cache
        N = acts[0].shape[0]
        L = len(self.W)
        gW: List[Optional[np.ndarray]] = [None] * L
        gb: List[Optional[np.ndarray]] = [None] * L

        # dL/dz for the last (linear) layer
        delta = dy
        for l in reversed(range(L)):
            gW[l] = acts[l].T @ delta / N
            gb[l] = delta.sum(axis=0) / N
            if l > 0:
                # back through layer l's weights into layer l's input activation
                da = delta @ self.W[l].T
                if self.activation == "sigmoid":
                    # f'(z[l-1]) = a[l] (1 - a[l]) because a[l] = f(z[l-1]);
                    # reuse the cached activation instead of another exp().
                    s = acts[l]
                    delta = da * s * (1.0 - s)
                else:
                    # ReLU: derivative is 1 where the pre-activation was > 0
                    delta = da * (zs[l - 1] > 0.0)
        out: List[np.ndarray] = []
        for l in range(L):
            out.append(gW[l])
            out.append(gb[l])
        return out

    # ------------------------------------------------------------- parameter IO
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

    def clone(self) -> "MLP":
        m = MLP(self.layers, self.activation)
        m.set_params([p.copy() for p in self.params()])
        return m

    def as_dict(self) -> Dict[str, list]:
        return {
            "layers": list(self.layers),
            "activation": self.activation,
            "W": [w.tolist() for w in self.W],
            "b": [b.tolist() for b in self.b],
        }


# ==========================================================================
#  Input encoding (must be mirrored exactly in ml_weights.c)
# ==========================================================================
def encode_inputs(reflectance: np.ndarray,
                  threshold: float = BLACK_THRESHOLD,
                  scale: float = REFLECTANCE_SCALE) -> np.ndarray:
    """
    (N, 14) reflectance on the 0..10 scale  ->  (N, 28) network input.

    columns 0..13 : reflectance / scale        (continuous)
    columns 14..27: 1.0 where reflectance <= threshold else 0.0
    """
    r = np.atleast_2d(np.asarray(reflectance, dtype=np.float64))
    if r.shape[1] != 14:
        raise ValueError(f"expected 14 sensor columns, got {r.shape[1]}")
    cont = r / scale
    binary = (r <= threshold).astype(np.float64)
    return np.concatenate([cont, binary], axis=1)


# ==========================================================================
#  Adam
# ==========================================================================
class Adam:
    """Adam, written out explicitly (no dependency)."""

    def __init__(self, params: List[np.ndarray], lr: float = 3e-3,
                 beta1: float = 0.9, beta2: float = 0.999, eps: float = 1e-8):
        self.lr, self.b1, self.b2, self.eps = lr, beta1, beta2, eps
        self.m = [np.zeros_like(p) for p in params]
        self.v = [np.zeros_like(p) for p in params]
        self.t = 0

    def step(self, params: List[np.ndarray], grads: List[np.ndarray]) -> None:
        self.t += 1
        for i, (p, g) in enumerate(zip(params, grads)):
            self.m[i] = self.b1 * self.m[i] + (1 - self.b1) * g
            self.v[i] = self.b2 * self.v[i] + (1 - self.b2) * (g * g)
            mh = self.m[i] / (1 - self.b1 ** self.t)
            vh = self.v[i] / (1 - self.b2 ** self.t)
            p -= self.lr * mh / (np.sqrt(vh) + self.eps)


# ==========================================================================
#  Trained wrapper: network + the output scaling it was fitted under
# ==========================================================================
@dataclass
class TrainedModel:
    """
    Bundles the network with the target normalisation fitted during training.

    A bare MLP.predict() on raw millimetre targets is wrong whenever the target
    was z-scored during training, which it is. Keeping the scaler welded to the
    network makes that class of bug impossible, and it is exactly what export.py
    needs to emit a self-contained C function.
    """

    net: MLP
    y_mean: float = 0.0
    y_std: float = 1.0
    meta: Dict[str, object] = field(default_factory=dict)

    @property
    def layers(self) -> Tuple[int, ...]:
        return tuple(self.net.layers)

    @property
    def n_params(self) -> int:
        return self.net.n_params()

    def predict(self, reflectance: np.ndarray, raw: bool = True) -> np.ndarray:
        """(N,14) reflectance  ->  (N,) predicted offset in millimetres."""
        X = encode_inputs(reflectance)
        yn = self.net.forward(X)
        if raw:
            yn = yn * self.y_std + self.y_mean
        return yn[:, 0] if yn.shape[1] == 1 else yn


@dataclass
class TrainHistory:
    train_loss: List[float] = field(default_factory=list)
    val_loss: List[float] = field(default_factory=list)
    val_mae: List[float] = field(default_factory=list)


# ==========================================================================
#  Training
# ==========================================================================
def train(
    X_reflect: np.ndarray,
    y: np.ndarray,
    X_val: Optional[np.ndarray] = None,
    y_val: Optional[np.ndarray] = None,
    layers: Sequence[int] = DEFAULT_LAYERS,
    n_out: int = 1,
    activation: str = DEFAULT_ACTIVATION,
    epochs: int = 400,
    batch_size: int = 256,
    lr: float = 3e-3,
    l2: float = 1e-6,
    outlier_weight: float = 3.0,
    seed: int = 0,
    verbose: bool = True,
    log_every: int = 50,
    patience: int = 60,
) -> Tuple[TrainedModel, TrainHistory]:
    """
    Train to regress the lateral offset in millimetres.

    X_reflect : (N, 14) raw reflectance on the 0..10 scale.  Encoding to the
                28-wide network input happens here so callers never have to
                know about it.
    y         : (N,) or (N, n_out) targets. Column 0 must be the offset in mm,
                because that is the column the firmware uses and the column the
                outlier weighting is computed from.

    outlier_weight: extra loss weight for samples with |offset| in the outer
        third of the range. Those are the near-line-loss poses; getting them
        right matters far more than shaving error off the easy centred cases,
        and they are under-represented in any natural dataset.
    """
    rng = np.random.default_rng(seed)
    X = encode_inputs(X_reflect)
    y = np.asarray(y, dtype=np.float64)
    if y.ndim == 1:
        y = y[:, None]
    if y.shape[1] != n_out:
        raise ValueError(f"n_out={n_out} but y has {y.shape[1]} columns")

    y_mean = float(y[:, 0].mean())
    y_std = float(y[:, 0].std() + 1e-9)
    yn = (y - y_mean) / y_std

    n = X.shape[0]
    span = float(np.abs(y[:, 0]).max()) + 1e-9
    w = np.ones(n)
    w[np.abs(y[:, 0]) > span * 0.66] = outlier_weight
    w = w / w.mean()

    net = MLP(tuple(layers) + (int(n_out),), activation)
    opt = Adam(net.params(), lr=lr)
    hist = TrainHistory()

    best_val = float("inf")
    best = [p.copy() for p in net.params()]
    since_improve = 0

    for ep in range(1, epochs + 1):
        perm = rng.permutation(n)
        ep_loss, nb = 0.0, 0
        for s in range(0, n, batch_size):
            idx = perm[s:s + batch_size]
            xb, yb, wb = X[idx], yn[idx], w[idx]
            pred, cache = net.forward(xb, cache=True)
            dy = mse_grad(pred, yb, wb)
            grads = net.backward(cache, dy)
            # L2 on weights only, never on biases
            for l in range(len(net.W)):
                grads[2 * l] = grads[2 * l] + l2 * net.W[l]
            opt.step(net.params(), grads)
            ep_loss += float(np.mean(wb * (pred - yb) ** 2))
            nb += 1
        hist.train_loss.append(ep_loss / max(nb, 1))

        if X_val is not None and y_val is not None:
            vp = net.forward(encode_inputs(X_val)) * y_std + y_mean
            yv = np.asarray(y_val, dtype=np.float64)
            if yv.ndim == 1:
                yv = yv[:, None]
            hist.val_loss.append(float(np.mean((vp - yv) ** 2)))
            hist.val_mae.append(float(np.mean(np.abs(vp[:, 0] - yv[:, 0]))))
            if hist.val_loss[-1] < best_val - 1e-9:
                best_val = hist.val_loss[-1]
                best = [p.copy() for p in net.params()]
                since_improve = 0
            else:
                since_improve += 1

        if verbose and (ep % log_every == 0 or ep == epochs):
            msg = f"  epoch {ep:4d}  train_loss {hist.train_loss[-1]:.6f}"
            if hist.val_mae:
                msg += f"  val_MAE {hist.val_mae[-1]:.4f}"
            print(msg)

        if patience and since_improve >= patience:
            if verbose:
                print(f"  early stop at epoch {ep} (no improvement for "
                      f"{patience} epochs)")
            break

    if X_val is not None and best_val < float("inf"):
        net.set_params(best)

    model = TrainedModel(
        net=net, y_mean=y_mean, y_std=y_std,
        meta=dict(layers=list(net.layers), activation=net.activation,
                  epochs=epochs, lr=lr, l2=l2,
                  batch_size=batch_size, outlier_weight=outlier_weight,
                  n_train=int(n), seed=seed),
    )
    return model, hist


# ==========================================================================
#  Metrics
# ==========================================================================
def mae(pred: np.ndarray, true: np.ndarray) -> float:
    return float(np.mean(np.abs(np.asarray(pred) - np.asarray(true))))


def rmse(pred: np.ndarray, true: np.ndarray) -> float:
    return float(math.sqrt(np.mean((np.asarray(pred) - np.asarray(true)) ** 2)))


def r2(pred: np.ndarray, true: np.ndarray) -> float:
    pred = np.asarray(pred, dtype=np.float64)
    true = np.asarray(true, dtype=np.float64)
    ss_res = float(np.sum((true - pred) ** 2))
    ss_tot = float(np.sum((true - true.mean()) ** 2)) + 1e-12
    return 1.0 - ss_res / ss_tot


def sign_accuracy(pred: np.ndarray, true: np.ndarray,
                  deadband: float = 2.0) -> float:
    """
    Fraction of poses where the correction points the RIGHT WAY.

    This is the metric that decides whether the robot stays on the line.
    Magnitude errors are absorbed by the steering PID; a wrong sign actively
    steers further off. Poses within `deadband` of centre are ignored because
    either sign is acceptable there.
    """
    pred = np.asarray(pred, dtype=np.float64)
    true = np.asarray(true, dtype=np.float64)
    m = np.abs(true) > deadband
    if not np.any(m):
        return 1.0
    return float(np.mean(np.sign(pred[m]) == np.sign(true[m])))


if __name__ == "__main__":
    # Smoke test: the network must fit a nonlinear function of the array, under
    # BOTH activations, and the sigmoid path must not collapse to a constant.
    rng = np.random.default_rng(0)
    R = rng.uniform(0.0, 10.0, size=(6000, 14))
    idx = np.arange(14) - 6.5
    depth = (10.0 - R)
    y = (depth * idx).sum(axis=1) / (depth.sum(axis=1) + 1e-6)
    y = y * 10.0 + 0.2 * rng.normal(size=6000)

    # --- numerical gradient check on the sigmoid path -----------------------
    # A hand-written backward pass with a smooth activation is exactly where a
    # transpose, a missing derivative term, or a lost 1/N hides silently, so
    # check every layer against finite differences.
    small = MLP((28, 6, 4, 1), "sigmoid")
    Xs = encode_inputs(rng.uniform(0, 10, size=(5, 14)))
    ys = rng.normal(size=(5, 1))

    def _loss() -> float:
        return float(np.mean((small.forward(Xs) - ys) ** 2))

    _, cache = small.forward(Xs, cache=True)
    dy = mse_grad(small.forward(Xs), ys)      # summed-loss gradient
    grads = small.backward(cache, dy)

    eps = 1e-6
    worst = 0.0
    for l in range(len(small.W)):
        for _ in range(8):
            i = int(rng.integers(0, small.W[l].shape[0]))
            j = int(rng.integers(0, small.W[l].shape[1]))
            orig = small.W[l][i, j]
            small.W[l][i, j] = orig + eps
            lp = _loss()
            small.W[l][i, j] = orig - eps
            lm = _loss()
            small.W[l][i, j] = orig
            num = (lp - lm) / (2 * eps)
            ana = float(grads[2 * l][i, j])
            denom = max(abs(num), abs(ana), 1e-12)
            worst = max(worst, abs(num - ana) / denom)
    print(f"gradient check (sigmoid): worst relative error {worst:.3e}")
    if worst > 1e-5:
        raise SystemExit("FAIL: backprop does not match numerical gradient")
    print("gradient check PASS")

    # --- saturation demo: Xavier vs He -------------------------------------
    # Shows WHY the init had to change: with He init the sigmoid units saturate
    # at step 0 and the mean activation sits at the flat part of the curve.
    for act, std_rule in (("sigmoid", "xavier"), ("relu", "he")):
        n = MLP((28, 45, 22, 5, 1), act)
        a = n.forward(encode_inputs(rng.uniform(0, 10, size=(500, 14))),
                      cache=True)[1][0]
        h1 = a[1]
        print(f"  {act:8s} ({std_rule:7s} init): layer-1 activation "
              f"mean {h1.mean():.3f} std {h1.std():.3f} "
              f"min {h1.min():.3f} max {h1.max():.3f}")

    # --- fit under both activations ----------------------------------------
    for act in ("sigmoid", "relu"):
        net = MLP(tuple(DEFAULT_LAYERS) + (1,), act)
        print(f"\narchitecture : {net.layers}  activation {act}")
        print(f"parameters   : {net.n_params():,}")
        m, h = train(R, y, R, y, layers=DEFAULT_LAYERS, activation=act,
                     epochs=150, lr=5e-3, verbose=False, patience=0)
        p = m.predict(R)
        print(f"  MAE={mae(p, y):.4f}  R2={r2(p, y):.4f}  "
              f"sign={sign_accuracy(p, y, 0.0)*100:.1f}%  "
              f"pred_std={p.std():.3f} (target std {y.std():.3f})")
        if r2(p, y) < 0.8:
            raise SystemExit(f"FAIL: {act} net did not fit the smoke problem")
    print("\nPASS")
