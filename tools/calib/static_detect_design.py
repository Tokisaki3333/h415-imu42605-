#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Static/motion detector design from the recorded static stream.

The detector (second processing step) low-pass filters the *bias-corrected* gyro
(previous step's result) and compares max_axis |mean over W| against a threshold.
This script derives the threshold from measured data:

  1) worst-case median / mean level offset of the corrected zero after a
     no-traction gap of T seconds  (what the threshold has to survive)
  2) distribution of the detector statistic during static  -> false-alarm rate
  3) required threshold = worst static drift + k * windowed white noise
  4) detection latency for a step motion of amplitude A (threshold crossing)

Usage: python static_detect_design.py <export.txt> [-o outdir]
"""

import argparse
import math
import os
import sys

import numpy as np

from bias_uncertainty import load_frames, moving_average

TS = 125e-6
FS = 1.0 / TS
AXES = ("x", "y", "z")


def worst_case_drift(raw1, bias1, T, stride_s=0.05):
    """|median(raw1[t:t+w]) - bias1[t]| over all starts, and the same with the mean."""
    w = int(round(T * FS))
    stride = max(1, int(round(stride_s * FS)))
    med = np.array([float(np.median(raw1[t0:t0 + w])) - float(bias1[t0])
                    for t0 in range(0, len(raw1) - w, stride)])
    ma = moving_average(raw1, w)
    mean = ma - bias1[:len(ma)]
    return med, mean


def main(argv):
    ap = argparse.ArgumentParser()
    ap.add_argument("export")
    ap.add_argument("-o", "--outdir", default="bias_out")
    a = ap.parse_args(argv)

    data, st = load_frames(a.export)
    n = len(data)
    raw = data[:, 0:3].astype(np.float64)
    cor = data[:, 3:6].astype(np.float64)        # corrected = what the detector sees
    bias = data[:, 6:9].astype(np.float64)
    p = print
    p("frames=%d  span=%.2f s" % (n, n * TS))
    os.makedirs(a.outdir, exist_ok=True)
    out = []

    # ---------------------------------------------------- 1) worst-case level offset
    p("")
    p("=" * 96)
    p("1) worst-case level offset of the corrected zero over a no-traction gap")
    p("   (fine 50 ms stride; median-based = robust readout, mean-based = dither-averaged)")
    p("    T[s]  windows   median:|d|max   median:|d|p99.9   mean:|d|max   mean:|d|p99.9   mean sd")
    tab = {}
    for T in (10, 20, 30, 60, 90):
        ms, ns = [], []
        for ax in range(3):
            med, mean = worst_case_drift(raw[:, ax], bias[:, ax], T)
            ms.append(med)
            ns.append(mean)
        med = np.concatenate(ms)
        mean = np.concatenate(ns)
        if len(mean) == 0:
            continue
        tab[T] = (med, mean)
        p("   %5d  %7d   %12.3e   %12.3e   %11.3e   %12.3e   %.3e"
          % (T, len(med), np.abs(med).max(), np.percentile(np.abs(med), 99.9),
             np.abs(mean).max(), np.percentile(np.abs(mean), 99.9), mean.std()))
        out.append((T, float(np.abs(med).max()), float(np.abs(mean).max()), float(mean.std())))
    p("")
    p("   -> the median readout is locked by the 1 LSB = %.4f dps quantum:" % (1 / 16.4))
    p("      worst case grows only in whole-LSB-ish steps, so 30 s vs 60 s can come out")
    p("      nearly equal; the mean readout grows monotonically with T.")
    np.savetxt(os.path.join(a.outdir, "gap_worstcase.csv"), np.array(out), delimiter=",",
               header="T_s,median_absmax,mean_absmax,mean_sd", comments="", fmt="%.6e")

    # ------------------------------------------------- 2/3) detector statistic + thr
    p("")
    p("=" * 96)
    p("2/3) detector statistic s = max_axis |mean(cor over W)|  (static record)")
    p("     white noise floor of the statistic = sigma_cor/sqrt(W)")
    p("     sigma_cor (per sample, averaged over axes) = %.4f dps" % np.mean(cor.std(axis=0)))
    p("")
    p("     W        W[ms]   s:p99.9     s:max      FA@0.01  FA@0.02  FA@0.05  FA@0.1   wn_floor")
    sig = float(np.mean(cor.std(axis=0)))
    det = []
    for W in (64, 128, 256, 512, 1024, 2048, 4096):
        ma = np.column_stack([moving_average(cor[:, ax], W) for ax in range(3)])
        s = np.abs(ma).max(axis=1)
        p999, smax = np.percentile(s, 99.9), s.max()
        fa = [float((s > thr).mean()) for thr in (0.01, 0.02, 0.05, 0.1)]
        wn = sig / math.sqrt(W)
        p("   %5d  %7.1f   %.3e  %.3e  %7.1e %8.1e %8.1e %8.1e  %.3e"
          % (W, W * TS * 1e3, p999, smax, fa[0], fa[1], fa[2], fa[3], wn))
        det.append((W, W * TS, p999, smax, fa[0], fa[1], fa[2], fa[3], wn))
    np.savetxt(os.path.join(a.outdir, "detector_stat.csv"), np.array(det), delimiter=",",
               header="W_samples,W_s,s_p999,s_max,fa_0.01,fa_0.02,fa_0.05,fa_0.1,wn_floor",
               comments="", fmt="%.6e")

    p("")
    p("     threshold budget (= what the user asked for):")
    for W in (128, 1024):
        wn = sig / math.sqrt(W)
        gap30_med = np.abs(tab[30][0]).max()
        gap30_mean = np.abs(tab[30][1]).max()
        p("       W=%4d (%5.1f ms): wn=%.3e ; + worst 30 s drift (median %.3e / mean %.3e)"
          % (W, W * TS * 1e3, wn, gap30_med, gap30_mean))
        p("                        -> median-based thr >= %.3e ; mean-based thr >= %.3e dps"
          % (gap30_med + 4 * wn, gap30_mean + 4 * wn))
        p("                        (= %.2f LSB / %.2f LSB)"
          % ((gap30_med + 4 * wn) * 16.4, (gap30_mean + 4 * wn) * 16.4))

    # ------------------------------------------------------- 4) detection latency
    p("")
    p("=" * 96)
    p("4) detection latency for a step motion (W = 128 samples = 16 ms, threshold 0.05 dps)")
    p("   statistic rises linearly, crossing thr at t = W*thr/A")
    for A in (0.05, 0.1, 0.2, 0.5, 1.0, 2.0, 5.0):
        t_cross = 128 * TS * min(1.0, 0.05 / A)
        p("     A = %5.2f dps -> %6.1f ms (%3.0f frames)" % (A, t_cross * 1e3, t_cross * FS))
    p("")
    p("   note: drag of the bias while the detector is still saying 'static' is")
    p("         omega/tau per second -> at tau=12 s, 1 dps for 20 ms costs 1.7e-3 dps")

    txt = "\n".join([]) if False else None
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
