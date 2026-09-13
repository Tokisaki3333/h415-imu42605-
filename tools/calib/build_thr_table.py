#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Threshold lookup table for the static detector.

Rigorous criterion.  The detector statistic s = max_axis |W-frame mean of the
corrected rate| has two uncertainty sources:

    sigma_s   = sigma_raw / sqrt(W)                  windowed rate noise (measured)
    e(T)      = e0 * exp(-T/tau)                     residual initial-bias error,
                                                     T = accumulated static time

so the statistic is distributed with scale

    sigma_eff(T) = sqrt(sigma_s^2 + e(T)^2)

Keeping the *same* false-alarm budget (K sigma) therefore requires a threshold

    thr(T) = clamp( K * sigma_eff(T),  base,  2*base ),   base = K*sigma_s

i.e. raise the threshold while the accumulated static evidence is small, and let it
fall back to base once e(T) has decayed into the noise.  Threshold, decay time and
ceiling are all pre-computed here -> the firmware only does a table lookup.

Usage: python build_thr_table.py <coldstart.txt> <longstatic.txt>
"""

import argparse
import math
import sys

import numpy as np

from bias_uncertainty import load_frames, moving_average

TS = 125e-6
FS = 1.0 / TS
W = 128
SIG_RAW = 0.1224
SIG_S = SIG_RAW / math.sqrt(W)
K_SIG = 10.0
BASE = K_SIG * SIG_S             # 0.1082 dps
CEIL = 3.0 * BASE                # 0.3246 dps = 30 sigma (启动段)
TAU = 12.0
E0 = 0.083                       # design value: makes the 3x ceiling hold exactly 12 s
                                 # (measured 0.02 dps on this unit -> 2.5x margin)
BUCKET_GRAN = 128                # 128 granules of 20 ms = 2.56 s per entry
NBUCK = 20                       # covers 51 s, then base


def main(argv):
    ap = argparse.ArgumentParser()
    ap.add_argument("cold")
    ap.add_argument("long")
    a = ap.parse_args(argv)

    print("sigma_s = %.5f dps   base = %.4f (%.0f sigma)   ceiling = %.4f (2x)"
          % (SIG_S, BASE, K_SIG, CEIL))
    print("criterion: thr(T) = clamp(%.0f*sqrt(sigma_s^2 + e0^2*exp(-2T/tau)), base, 2*base)"
          % K_SIG)
    print("design e0 = %.3f dps, tau = %.0f s  -> ceiling holds until T = %.1f s"
          % (E0, TAU, TAU * math.log(E0 / math.sqrt((CEIL / K_SIG) ** 2 - SIG_S ** 2)) / 2))
    print("")

    tab = []
    for i in range(NBUCK):
        T = i * BUCKET_GRAN * 0.020
        e = E0 * math.exp(-T / TAU)
        thr = K_SIG * math.sqrt(SIG_S ** 2 + e ** 2)
        thr = min(CEIL, max(BASE, thr))
        tab.append(round(thr, 4))

    print("   i   T[s]     e(T)[dps]   thr[dps]   thr/base")
    for i, v in enumerate(tab):
        T = i * BUCKET_GRAN * 0.020
        print("  %2d  %6.2f    %9.5f   %8.4f   %6.2f"
              % (i, T, E0 * math.exp(-T / TAU), v, v / BASE))
    print("")
    print("C array (%.2f s per entry, %d entries):" % (BUCKET_GRAN * 0.020, NBUCK))
    print("static const float s_thr_tab[%d] = {" % NBUCK)
    for i in range(0, NBUCK, 8):
        print("    " + ", ".join("%.4ff" % v for v in tab[i:i + 8]) + ",")
    print("};")
    print("")

    # ---------------------------------------------------------------- validation
    d, st = load_frames(a.cold)
    cor = d[:, 3:6].astype(np.float64)
    s = np.abs(np.column_stack([moving_average(cor[:, ax], W) for ax in range(3)])).max(axis=1)
    idx = np.minimum((np.arange(len(s)) * TS / (BUCKET_GRAN * 0.020)).astype(int), NBUCK - 1)
    thr = np.array(tab)[idx]
    marg = thr / np.maximum(s, 1e-12)
    print("validation on the cold-start record (%d frames, %.1f s):" % (len(s), len(s) * TS))
    print("   crossings = %d   min margin = %.2fx at t=%.2f s"
          % (int((s > thr).sum()), marg.min(), float(np.argmin(marg)) * TS))
    print("   statistic max: %.4f (boot, t<12.8s)  %.4f (rest)"
          % (s[:int(12.8 * FS)].max(), s[int(12.8 * FS):].max()))
    print("   would a FLAT base table suffice here? crossings = %d"
          % int((s > BASE).sum()))
    print("   negative control, flat 6 sigma = %.3f -> crossings = %d"
          % (6 * SIG_S, int((s > 6 * SIG_S).sum())))

    d2, _ = load_frames(a.long)
    cor2 = d2[:, 3:6].astype(np.float64)
    s2 = np.abs(np.column_stack([moving_average(cor2[:, ax], W) for ax in range(3)])).max(axis=1)
    print("   steady-state record: max=%.4f -> margin vs base = %.2fx, vs ceiling = %.2fx"
          % (s2.max(), BASE / s2.max(), CEIL / s2.max()))
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
