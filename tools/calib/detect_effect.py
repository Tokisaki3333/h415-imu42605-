#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Judge the static/motion detection on a cold-start record.

Only 9 channels are reported (raw / corrected / bias), so the detector is judged by
  (a) the gate signature in the bias channel: traction keeps moving it while static
      (no freeze), a rollback would show as a step, a false 'motion' freezes it;
  (b) replaying the exact detector statistic/thresholds offline on the corrected
      channel and looking at the margin to the thresholds.

Usage: python detect_effect.py <export.txt> [--tau 12]
"""

import argparse
import math
import sys

import numpy as np

from bias_uncertainty import load_frames, moving_average

TS = 125e-6
FS = 1.0 / TS
AXES = ("x", "y", "z")
W = 128
THR_BOOT, THR_ON, THR_OFF = 0.216, 0.108, 0.065
BOOT = 96000
DEB_ON, DEB_OFF = 8, 800


def main(argv):
    ap = argparse.ArgumentParser()
    ap.add_argument("export")
    ap.add_argument("--tau", type=float, default=12.0)
    a = ap.parse_args(argv)

    d, st = load_frames(a.export)
    n = len(d)
    raw = d[:, 0:3].astype(np.float64)
    cor = d[:, 3:6].astype(np.float64)
    bia = d[:, 6:9].astype(np.float64)
    print("frames=%d span=%.2f s  malformed=%d badtail=%d" %
          (n, n * TS, st["n_bad"], st["n_tail_bad"]))

    # ---------------------------------------------------------------- cold start
    print("")
    print("=" * 92)
    print("cold start: bias at t=0 vs the calibrated constants (LSB)")
    print("   frame0 bias [LSB] = %.4f %.4f %.4f   (constants 1.0334 0.8494 12.0910)"
          % tuple(bia[0] * 16.4))
    print("   corrected level over the first 0.5 s : mean %+.4f %+.4f %+.4f dps"
          % tuple(cor[:int(0.5 * FS)].mean(axis=0)))
    print("   corrected level over the whole record: mean %+.4f %+.4f %+.4f dps"
          % tuple(cor.mean(axis=0)))
    print("   bias drift over the record           : %+.4f %+.4f %+.4f dps"
          % tuple(bia[-1] - bia[0]))

    # ------------------------------------------------------- loop gain / tau check
    print("")
    print("=" * 92)
    print("loop check (alpha must read back as Ts/tau = %.4e)" % (TS / a.tau))
    for ax in range(3):
        bp = bia[:-1, ax]
        db = bia[1:, ax] - bp
        xx = raw[1:, ax] - bp
        sel = np.abs(xx) > 0.1
        al = float(np.median(db[sel] / xx[sel]))
        print("   axis %s: alpha=%.6e -> tau=%.2f s   (n=%d)"
              % (AXES[ax], al, TS / al, int(sel.sum())))

    # ------------------------------------------------- gate signature in the bias
    print("")
    print("=" * 92)
    print("gate signature in the bias channel")
    db = np.abs(np.diff(bia, axis=0))
    dmax = db.max(axis=1)
    step_typ = float(np.median(dmax[dmax > 0]))
    print("   typical |dbias| per frame = %.3e dps ; max = %.3e dps at frame %d (t=%.2f s)"
          % (step_typ, dmax.max(), int(np.argmax(dmax)), int(np.argmax(dmax)) * TS))
    for k in (1e-5, 3e-5, 1e-4, 1e-3):
        print("   frames with |dbias| > %.0e : %d" % (k, int((dmax > k).sum())))
    frozen = dmax < step_typ * 0.05
    # longest run of frozen frames
    best = cur = 0
    for f in frozen:
        cur = cur + 1 if f else 0
        best = max(best, cur)
    print("   longest frozen run (|dbias| < 5%% of typical): %.3f s  <-- a false 'motion'"
          " would freeze the bias" % (best * TS))
    roll = np.diff(bia, axis=0)
    neg = np.sort(roll.min(axis=1))[:5]
    print("   five most negative single-frame dbias: %s  (a rollback would show here)"
          % " ".join("%.3e" % v for v in neg))

    # ------------------------------------------- detector statistic replayed offline
    print("")
    print("=" * 92)
    print("detector statistic replayed on the corrected channel")
    print("   W=%d (%.0f ms); thr_boot(12 s)=%.3f  20 sigma, thr_on=%.3f  10 sigma, thr_off=%.3f"
          % (W, W * TS * 1e3, THR_BOOT, THR_ON, THR_OFF))
    ma = np.column_stack([moving_average(cor[:, ax], W) for ax in range(3)])
    s = np.abs(ma).max(axis=1)
    sb = s[:BOOT]
    sa = s[BOOT:]
    print("   boot phase (0..12 s) : max=%.4f dps  p99.99=%.4f  margin to %.3f = %.2fx"
          % (sb.max(), np.percentile(sb, 99.99), THR_BOOT, THR_BOOT / sb.max()))
    print("   normal phase (12..end): max=%.4f dps  p99.99=%.4f  margin to %.3f = %.2fx"
          % (sa.max(), np.percentile(sa, 99.99), THR_ON, THR_ON / sa.max()))
    print("   frames over thr_on  = %d ; over thr_boot = %d  (0 = no false trigger)"
          % (int((sa > THR_ON).sum()), int((sb > THR_BOOT).sum())))
    print("   sigma_s of the statistic = %.4f dps  (raw sigma/sqrt(W) = %.4f)"
          % (s.std(), float(np.mean([cor[:, ax].std() for ax in range(3)])) / math.sqrt(W)))
    print("")
    print("   smallest sustained motion the normal threshold would catch: %.3f dps"
          % THR_ON)
    print("   (previous record's real rotation peaked at 100 dps -> %d x margin)"
          % int(100.0 / THR_ON))
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
