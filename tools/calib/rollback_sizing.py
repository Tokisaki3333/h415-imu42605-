#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Rollback sizing seen as: for a given detection latency L, how many 20 ms granules
must the rollback span?  Uses the one typical rotation in the record.

  drag(L)      = (1/tau) * |int omega dt over [onset, onset+L]|      what must go
  residual(N)  = (1/tau) * |int omega dt over [onset+L-N*20ms, onset+L]|  what stays

onset is defined threshold-free: the time when the swept angle reaches 1% of the
total swept angle of the rotation.

Usage: python rollback_sizing.py <export.txt> [--tau 12] [--target 1e-4]
"""

import argparse
import math
import sys

import numpy as np

from bias_uncertainty import load_frames, moving_average

TS = 125e-6
FS = 1.0 / TS
AXES = ("x", "y", "z")
GRAN = 0.020


def main(argv):
    ap = argparse.ArgumentParser()
    ap.add_argument("export")
    ap.add_argument("--tau", type=float, default=12.0)
    ap.add_argument("--target", type=float, default=1e-4, help="allowed residual [dps]")
    a = ap.parse_args(argv)

    data, _ = load_frames(a.export)
    n = len(data)
    raw = data[:, 0:3].astype(np.float64)
    tail = raw[int(n - 0.3 * FS):, :].mean(axis=0)
    om = raw - tail                                    # [dps], ~0 when static

    # rotation = the window that holds the swept angle; use a 20 ms smoothed rate
    w = int(round(GRAN * FS))
    om20 = np.column_stack([moving_average(np.abs(om[:, ax]), w) for ax in range(3)])
    ang = om20.sum(axis=1) * TS                        # |omega| integral, per sample
    tot = ang.sum()
    t = np.arange(len(ang)) * TS
    i1 = int(np.argmax(np.cumsum(ang) >= 0.01 * tot))  # onset: first 1% of the angle
    i2 = int(np.argmax(np.cumsum(ang) >= 0.99 * tot))
    print("record %.3f s, swept |omega|dt = %.2f dps*s" % (n * TS, tot))
    print("rotation body: t=%.3f .. %.3f s, peak 20 ms rate %.1f dps, tau=%.0f s"
          % (t[i1], t[i2], om20.max(), a.tau))
    print("")

    # ---------------------------------------------------------- drag vs latency
    print("=" * 88)
    print("drag acquired during the detection latency L (tau=%.0f s)" % a.tau)
    print("   L[ms]   swept angle[dps*s]   drag[dps]   drag[LSB]   N=ceil(L/20ms)")
    for L in (5, 10, 20, 40, 60, 100, 150, 200, 300, 500):
        k = int(round(L * 1e-3 * FS))
        if i1 + k >= n:
            break
        ang_l = np.trapezoid(om[i1:i1 + k + 1, :], dx=TS, axis=0)
        drag = np.abs(ang_l) / a.tau
        print("  %6d   %16.3e   %10.3e   %10.3f   %12d"
              % (L, np.abs(ang_l).max(), drag.max(), drag.max() * 16.4,
                 math.ceil(L / (GRAN * 1e3))))
    print("")

    # -------------------------------------------------- residual vs N, per latency
    print("=" * 88)
    print("residual left after rolling back N granules, for a given latency L")
    print("(target %.1e dps = %.2f LSB; '-' = already clean, '.' = worse than target)"
          % (a.target, a.target * 16.4))
    print("")
    hdr = "   N  window"
    print(hdr + "".join("   L=%3dms" % L for L in (10, 20, 50, 100, 200)))
    for N in (1, 2, 3, 4, 5, 8, 12, 16, 24, 32, 64):
        gw = int(round(N * GRAN * FS))
        row = "%4d  %5.0fms" % (N, N * GRAN * 1e3)
        for L in (10, 20, 50, 100, 200):
            k = int(round(L * 1e-3 * FS))
            j0, j1 = i1 + k - gw, i1 + k
            if j0 < 0 or j1 >= n:
                row += "        -"
                continue
            seg = np.trapezoid(om[j0:j1 + 1, :], dx=TS, axis=0)
            res = np.abs(seg).max() / a.tau
            row += "  %8.1e" % res if res <= a.target else "  %8.1f." % res
        print(row)

    # ------------------------------- worst credible onset: slope of this rotation
    slope = om20[:, 2].max() / max(t[i2] - t[i1], 1e-6)
    print("")
    print("worst credible onset of this record: ramp slope ~ %.1f dps/s (z, body peak/body time)"
          % slope)
    print("  latency for the statistic (16 ms mean, thr 0.05 dps) to cross on such a ramp:")
    print("    L ~= thr/slope + W/2 = %.1f + 8 = %.1f ms  ->  N = %d granule(s)"
          % (0.05 / slope * 1e3, (0.05 / slope + 0.008) * 1e3,
             math.ceil((0.05 / slope + 0.008) / GRAN)))
    print("  a *slow* onset (0.5 dps/s) needs L ~ 100 ms but then only sweeps")
    print("    %.1e dps*s -> drag %.1e dps, i.e. negligible even with a modest N"
          % (0.5 * 0.1 ** 2 / 2, 0.5 * 0.1 ** 2 / 2 / a.tau))
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
