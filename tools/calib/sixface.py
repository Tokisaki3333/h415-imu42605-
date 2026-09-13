#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Six-face rotation test -> static-threshold and time-constant selection.

Splits the record into rotation bursts and static holds, then reports per hold
  * the converged bias (the zero for that face)      -> face-to-face zero shift
  * the corrected level right after the rotation     -> what thr_off must cover
  * how long the detector needs to release back to static (replayed firmware model)
  * the loop gain alpha -> the effective tau

Usage: python sixface.py <export.txt> [--tau 12]
"""

import argparse
import math
import sys

import numpy as np

from bias_uncertainty import load_frames, moving_average

TS = 125e-6
FS = 1.0 / TS
W = 128
BASE = 0.108
THR_OFF = 0.065
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
    print("frames=%d span=%.2f s  malformed=%d badtail=%d"
          % (n, n * TS, st["n_bad"], st["n_tail_bad"]))

    s = np.abs(np.column_stack([moving_average(cor[:, ax], W) for ax in range(3)])).max(axis=1)
    # 粗分类：转动 = 统计量 > 0.5 dps
    moving = s > 0.5
    # 找静止保持段：连续 >= 0.5 s 的 non-moving
    holds = []
    i = 0
    minlen = int(0.5 * FS)
    while i < len(moving):
        if not moving[i]:
            j = i
            while j < len(moving) and not moving[j]:
                j += 1
            if j - i >= minlen:
                holds.append((i, j))
            i = j
        else:
            i += 1
    print("detected %d static holds (>=0.5 s):" % len(holds))
    print("")
    print("=" * 104)
    print("   #  t_start   dur[s]   conv bias x/y/z [dps]            | after-rotation offset [dps]"
          "  |  max|offset|  cor level")
    rows = []
    for k, (i0, i1) in enumerate(holds):
        dur = (i1 - i0) * TS
        h = int(min(i1 - i0, int(1.0 * FS)))            # 用保持段最后 1 s 当收敛值
        conv = bia[i1 - h:i1].mean(axis=0)
        off = np.abs(cor[i0:i0 + int(0.3 * FS)].mean(axis=0))
        lvl = cor[i0:i1].mean(axis=0)
        rows.append((i0, i1, dur, conv, off, lvl))
        print("  %2d  %8.3f  %7.2f   %+.5f %+.5f %+.5f   |  %6.4f %6.4f %6.4f  |  %.4f      %+.5f %+.5f %+.5f"
              % (k, i0 * TS, dur, conv[0], conv[1], conv[2], off[0], off[1], off[2],
                 off.max(), lvl[0], lvl[1], lvl[2]))

    print("")
    print("=" * 104)
    print("face-to-face zero shift (converged bias difference between neighbouring holds):")
    tot = []
    for k in range(1, len(rows)):
        db = rows[k][3] - rows[k - 1][3]
        tot.append(np.abs(db))
        print("   hold %d -> %d : %+.5f %+.5f %+.5f dps   max %.5f" %
              (k - 1, k, db[0], db[1], db[2], np.abs(db).max()))
    if tot:
        tot = np.array(tot)
        print("   -> |shift| per rotation: max=%.5f  median=%.5f dps"
              % (tot.max(), np.median(tot)))
        print("   -> 这是转动导致的零偏漂移上界（含 g 敏感/安装变化），逐面看")
    print("")
    off_all = np.array([r[4].max() for r in rows])
    print("after-rotation offset (first 0.3 s of each hold), max over axes:")
    print("   max=%.5f  median=%.5f dps   -> thr_off 至少要到这个值之上" %
          (off_all.max(), np.median(off_all)))

    # ---------------------------------------------------------- tau from loop gain
    print("")
    print("=" * 104)
    print("loop gain measured on the static holds (alpha = dbias/(raw-bias_prev)):")
    al = []
    for (i0, i1, dur, conv, off, lvl) in rows:
        bp = bia[i0:i1 - 1]
        db = bia[i0 + 1:i1] - bp
        xx = raw[i0 + 1:i1] - bp
        sel = np.abs(xx) > 0.05
        if sel.sum() > 100:
            al.append(np.median(db[sel] / xx[sel]))
    if al:
        al = float(np.median(al))
        print("   alpha = %.6e  ->  tau_eff = %.2f s  (design %.0f s)"
              % (al, TS / al, a.tau))
        print("   含义：每次转完后偏置残差按 tau_eff 衰减，保持段至少需要 ~3*tau 才收敛")

    # --------------------------------------------- release replay on this record
    print("")
    print("=" * 104)
    print("release replay (firmware model, W=128, debounce %d/%d, thr_off=%.3f):"
          % (DEB_ON, DEB_OFF, THR_OFF))
    thr = np.where(s > BASE, 1e9, THR_OFF)      # 只用 off 判据，估计"能否释放"
    for k, (i0, i1) in enumerate(holds):
        seg = s[i0:i1]
        over = np.flatnonzero(seg > THR_OFF)
        first_ok = -1
        c = 0
        for j in range(len(seg)):
            c = c + 1 if seg[j] <= THR_OFF else 0
            if c >= DEB_OFF:
                first_ok = j - DEB_OFF + 1
                break
        print("   hold %2d: 起始偏移=%.4f dps  保持段内持续超 thr_off 的帧数=%d(%.2f s)"
              "  释放时刻(hard) = %s"
              % (k, rows[k][4].max(), len(over), len(over) * TS,
                 ("%.3f s" % ((i0 + first_ok) * TS)) if first_ok >= 0 else "未释放(锁死)"))
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
