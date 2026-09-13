#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Traction (zero-bias loop) design sweep, answered from the recorded raw gyro stream
(every line = one frame, 125 us, logger timestamps ignored).

  Q1  bias frozen (no traction) for T seconds -> how far does the MEDIAN of the
      corrected zero move?   (median = robust level of the corrected output)
  Q2  how does that value scale with the freeze duration T?
  Q3  sweet spot of the traction speed (1/tau): faster traction pulls rate noise
      into the zero estimate, slower traction fails to follow the real drift.
      Both sides are measured by replaying the firmware loop on the recorded raw.

Usage: python bias_traction_sweep.py <export.txt> [-o outdir]
"""

import argparse
import math
import os
import sys

import numpy as np

from bias_uncertainty import load_frames, detrended, moving_average

TS = 125e-6                 # assumed frame interval [s]
FS = 1.0 / TS
AXES = ("x", "y", "z")
GAPS = (1, 2, 5, 10, 20, 30, 60, 120)
TAUS = (0.5, 1, 2, 3, 4, 6, 8, 12, 16, 24, 32, 48, 64, 128)


def lpf_alpha(x, alpha, b0=0.0, block=8192):
    """b[k] = (1-alpha)*b[k-1] + alpha*x[k], block-vectorised.

    With alpha = Ts/tau this is exactly the firmware's per-frame traction, so the
    replay reproduces the loop that ran on the target (up to float rounding).
    """
    beta = 1.0 - alpha
    n = len(x)
    out = np.empty(n, np.float64)
    ar = np.arange(block)
    b = float(b0)
    for s in range(0, n, block):
        e = min(n, s + block)
        m = e - s
        aw = ar[:m]
        pw = beta ** aw                      # beta^i
        inv = beta ** (-aw)                  # beta^-i  (block capped -> no overflow)
        c = np.cumsum(x[s:e] * inv)
        out[s:e] = pw * (beta * b + alpha * c)
        b = out[e - 1]
    return out


def median_drift(raw, bias, w, stride):
    """median(raw[t:t+w]) - bias[t] for every start t."""
    n = len(raw)
    return np.array([float(np.median(raw[t0:t0 + w])) - float(bias[t0])
                     for t0 in range(0, n - w, stride)])


def mean_drift(raw, bias, w):
    """mean(raw[t:t+w]) - bias[t] for every start t (cheap, via moving average)."""
    ma = moving_average(raw, w)                  # ma[j] = mean(raw[j:j+w])
    return ma - bias[:len(ma)]


def main(argv):
    ap = argparse.ArgumentParser()
    ap.add_argument("export")
    ap.add_argument("-o", "--outdir", default="bias_out")
    a = ap.parse_args(argv)

    data, st = load_frames(a.export)
    n = len(data)
    raw = data[:, 0:3].astype(np.float64)
    bias = data[:, 6:9].astype(np.float64)
    print("frames=%d  span=%.2f s  (%.0f us/frame)" % (n, n * TS, TS * 1e6))
    os.makedirs(a.outdir, exist_ok=True)

    lines = []
    p = lines.append

    # ------------------------------------------------------------------ Q1 + Q2
    p("=" * 92)
    p("Q1/Q2  no traction for T seconds -> MEDIAN level drift of the corrected zero")
    p("       = median(raw over the gap) - bias at the start of the gap.")
    p("       Measured on the recorded bias (the tau = 8 s loop that actually ran).")
    p("")
    p("   T [s]   windows    sd[dps]    sd[deg/h]   median[dps]  p95|d|[dps]  max|d|[dps]")
    tab = {}
    for T in GAPS:
        w = int(round(T * FS))
        if w >= n - FS:
            continue
        d = np.concatenate([median_drift(raw[:, ax], bias[:, ax], w, int(FS))
                            for ax in range(3)])
        tab[T] = d
        p("  %5d   %7d    %.3e    %.3e    %+.3e     %.3e     %.3e"
          % (T, d.size, d.std(), d.std() * 3600, float(np.median(d)),
             np.percentile(np.abs(d), 95), np.abs(d).max()))
    if 10 in tab:
        p("")
        p("  sd(T)/sd(10 s):  " + "   ".join("%ds:%.2f" % (T, tab[T].std() / tab[10].std())
                                             for T in tab))
        p("  sqrt(T) would be: " + "   ".join(
            "%ds:%.2f" % (T, math.sqrt(T / 10.0)) for T in tab))

    # ------------------------------------------------------------------- Q3
    p("")
    p("=" * 92)
    p("Q3  traction sweep: replay b[k]=(1-a)b[k-1]+a*raw[k] with a = Ts/tau on the")
    p("    recorded raw (initialised at the recorded bias, first 10 s skipped)")
    p("      noise_mix : sd of the zero estimate after removing its >5 s drift")
    p("                  = rate noise the traction pulls into the estimate")
    p("      lag       : mean(bias - 60 s reference), the systematic following error")
    p("      err_total : rms(bias - 60 s reference)")
    p("      gap30     : same cost metric as Q1 but with the 30 s window MEAN (cheap)")
    p("")
    ref = np.empty_like(raw)
    for ax in range(3):
        ma = moving_average(raw[:, ax], int(round(60 * FS)))
        ref[:, ax] = np.concatenate([ma, np.full(n - len(ma), ma[-1])])   # pad the tail
    skip = int(10 * FS)
    sx_mean = float(np.mean([np.std(raw[:, ax] - bias[:, ax]) for ax in range(3)]))
    p("   tau[s]  alpha      noise_mix    theory      lag       err_total    gap30       gap30_p95")
    res = []
    for tau in TAUS:
        alpha = TS / tau
        nm, lg, et, g30, g30p = [], [], [], [], []
        for ax in range(3):
            bs = lpf_alpha(raw[:, ax], alpha, b0=bias[0, ax])
            b = bs[skip:]
            rf = ref[skip:, ax]
            nm.append(detrended(b, int(round(5 * FS))).std())
            lg.append(float(np.mean(b - rf)))
            et.append(float(np.sqrt(np.mean((b - rf) ** 2))))
            d = mean_drift(raw[:, ax], bs, int(round(30 * FS)))[skip:]
            g30.append(d.std())
            g30p.append(float(np.percentile(np.abs(d), 95)))
        nm, lg, et, g30, g30p = (float(np.mean(v)) for v in (nm, lg, et, g30, g30p))
        theory = sx_mean * math.sqrt(alpha / 2.0)
        res.append((tau, alpha, nm, theory, lg, et, g30, g30p))
        p("  %6.1f  %.3e  %.3e   %.3e  %+.3e  %.3e  %.3e  %.3e"
          % (tau, alpha, nm, theory, lg, et, g30, g30p))

    res = np.array(res)
    ib = int(np.argmin(res[:, 6]))
    ok = res[:, 6] <= res[ib, 6] * 1.10
    it2 = int(np.argmin(res[:, 2] + np.abs(res[:, 4])))       # noise+lag tradeoff
    p("")
    p("  small-gap optimum (min gap30)  : tau = %.1f s -> gap30 = %.3e dps"
      % (res[ib, 0], res[ib, 6]))
    p("  within 10%% of that minimum    : tau = %.1f .. %.1f s"
      % (res[ok, 0].min(), res[ok, 0].max()))
    p("  noise+lag optimum              : tau = %.1f s" % res[it2, 0])
    p("  at tau = 8 s (current firmware): noise_mix=%.3e  lag=%+.3e  gap30=%.3e dps"
      % (res[np.argmin(np.abs(res[:, 0] - 8)), 2],
         res[np.argmin(np.abs(res[:, 0] - 8)), 4],
         res[np.argmin(np.abs(res[:, 0] - 8)), 6]))

    # ----------------------------------------------------------------- outputs
    np.savetxt(os.path.join(a.outdir, "traction_sweep.csv"), res, delimiter=",",
               header="tau_s,alpha,noise_mix,noise_theory,lag,err_total,gap30_sd,gap30_p95",
               comments="", fmt="%.6e")
    Ts = np.array([T for T in GAPS if T in tab], float)
    np.savetxt(os.path.join(a.outdir, "freeze_median_drift.csv"),
               np.column_stack([Ts, [tab[T].std() for T in GAPS if T in tab],
                                [np.abs(tab[T]).max() for T in GAPS if T in tab]]),
               delimiter=",", header="T_s,sd_dps,max_abs_dps", comments="", fmt="%.6e")
    with open(os.path.join(a.outdir, "traction_sweep.txt"), "w", encoding="utf-8") as fh:
        fh.write("\n".join(lines) + "\n")

    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        fig, ax = plt.subplots(1, 2, figsize=(13, 5))
        sd = np.array([tab[T].std() for T in GAPS if T in tab])
        mx = np.array([np.abs(tab[T]).max() for T in GAPS if T in tab])
        ax[0].loglog(Ts, sd, "o-", label="sd of median drift")
        ax[0].loglog(Ts, mx, "s-", label="max |median drift|")
        ax[0].loglog(Ts, sd[0] * np.sqrt(Ts / Ts[0]), "k--", label="sqrt(T) reference")
        ax[0].set_xlabel("no-traction duration T [s]")
        ax[0].set_ylabel("corrected-zero median drift [dps]")
        ax[0].set_title("cost of a traction-free gap")
        ax[0].grid(True, which="both", alpha=0.3)
        ax[0].legend(fontsize=8)

        t = res[:, 0]
        ax[1].loglog(t, np.maximum(res[:, 6], 1e-12), "o-", label="gap30 (30 s freeze cost)")
        ax[1].loglog(t, res[:, 2], "s-", label="noise_mix (rate noise pulled in)")
        ax[1].loglog(t, np.abs(res[:, 4]) + 1e-12, "^-", label="|lag| (following error)")
        ax[1].loglog(t, res[:, 3], "k--", lw=1, label="white-noise theory")
        ax[1].axvline(res[ib, 0], color="r", ls=":", lw=1,
                      label="optimum tau=%.1f s" % res[ib, 0])
        ax[1].set_xlabel("traction time constant tau [s]")
        ax[1].set_ylabel("dps")
        ax[1].set_title("traction speed sweet spot")
        ax[1].grid(True, which="both", alpha=0.3)
        ax[1].legend(fontsize=8)
        fig.tight_layout()
        fig.savefig(os.path.join(a.outdir, "traction_sweep.png"), dpi=110)
        plt.close(fig)
        p("")
        p("plot: %s" % os.path.join(a.outdir, "traction_sweep.png"))
    except Exception as exc:
        p("plotting skipped: %r" % (exc,))

    txt = "\n".join(lines)
    print(txt)
    with open(os.path.join(a.outdir, "traction_sweep.txt"), "w", encoding="utf-8") as fh:
        fh.write(txt + "\n")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
