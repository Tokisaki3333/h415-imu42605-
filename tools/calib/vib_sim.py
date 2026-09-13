#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Vibration stress test for the static detector.

Synthetic stream = the REAL static record (so the zero wander, the white rate noise
and the 1 LSB = 0.061 dps quantisation are all the measured ones) + a synthetic
vibration added on top and re-quantised:

    raw[k] = round((raw_real[k] + vib[k]) * 16.4) / 16.4

vib = sum of M tones, log-spaced in [f_lo, f_hi], random phases, scaled to a target
RMS per axis.  Two profiles: continuous and bursty (1 s on / 1 s off), the bursty one
being what produces repeated static->motion edges (i.e. repeated rollbacks).

Replayed model = the firmware: bias loop tau=12 s, 20 ms snapshot ring, rollback N on
each static->motion edge clamped by the current static run, evidence counter reduced by
a rollback, detector with W=128, hysteresis, debounce, and threshold from either
  flat   : thr = base = 0.108 dps
  table  : thr = s_thr_tab[evidence >> 7]  (the dynamic table, ceiling 2x)

Usage: python vib_sim.py <longstatic.txt> [--tau 12]
"""

import argparse
import math
import sys

import numpy as np

from bias_uncertainty import load_frames, moving_average

TS = 125e-6
FS = 1.0 / TS
TAU = 12.0
W = 128
GRAN = 160
DEPTH = 64
N_ROLLBACK = 24
DEB_ON, DEB_OFF = 8, 800
BASE = 0.108
SHIFT = 7
NBUCK = 16
THR_TAB = np.array([0.2164, 0.2164, 0.2164, 0.2164, 0.2164, 0.2033, 0.1762, 0.1559,
                    0.1412, 0.1307, 0.1233, 0.1183, 0.1149, 0.1126, 0.1111, 0.1101])


def ma2(x, w):
    c = np.concatenate([np.zeros((1, x.shape[1])), np.cumsum(x, axis=0)], axis=0)
    return (c[w:] - c[:-w]) / w


def lpf2(x, alpha, b0, block=8192):
    beta = 1.0 - alpha
    n = x.shape[0]
    out = np.empty_like(x)
    b = np.array(b0, float)
    ar = np.arange(block)
    for s in range(0, n, block):
        e = min(n, s + block)
        m = e - s
        pw = (beta ** ar[:m])[:, None]
        inv = (beta ** (-ar[:m]))[:, None]
        c = np.cumsum(x[s:e] * inv, axis=0)
        out[s:e] = pw * (beta * b + alpha * c)
        b = out[e - 1]
    return out


def first_run(flag, deb):
    c = 0
    for i in range(len(flag)):
        c = c + 1 if flag[i] else 0
        if c >= deb:
            return i - deb + 1
    return -1


def run(stream, mode, tau=TAU, n_rollback=N_ROLLBACK):
    """Replay the firmware. mode: 'none' | 'flat' | 'table'."""
    n = stream.shape[0]
    alpha = TS / tau
    bias = np.empty((n, 3))
    m0 = int(0.5 * FS)
    b = stream[:m0].mean(axis=0)          # calibrated initial value
    ring = np.zeros((3, DEPTH))
    head = -1
    gran_t = []                            # frame index of each snapshot
    evidence = 0
    static_gran = 0
    state = 1
    t = 0
    decl = rolls = 0
    motion_frames = 0
    while t < n:
        if state == 1:
            seg = lpf2(stream[t:], alpha, b)
            cor = stream[t:] - seg
            s = np.abs(ma2(cor, W)).max(axis=1)
            m = len(seg)
            ms = len(s)
            # per-granule evidence / threshold
            gidx = (np.arange(ms) + (t % GRAN)) // GRAN
            if mode == 'none':
                thr = np.full(ms, 1e9)
            elif mode == 'flat':
                thr = np.full(ms, BASE)
            else:
                idx = np.minimum((evidence + gidx) >> SHIFT, NBUCK - 1)
                thr = np.maximum(THR_TAB[idx], BASE)
            k = first_run(s > thr, DEB_ON)
            mm = m if k < 0 else k + 1
            bias[t:t + mm] = seg[:mm]
            # snapshots
            g = (GRAN - (t % GRAN)) % GRAN
            while g < mm:
                head = (head + 1) % DEPTH
                ring[:, head] = seg[g]
                gran_t.append(t + g)
                if len(gran_t) > 4 * DEPTH:
                    gran_t.pop(0)
                static_gran = min(static_gran + 1, DEPTH)
                evidence = min(evidence + 1, NBUCK << SHIFT)
                g += GRAN
            if k < 0:
                break
            decl += 1
            b = seg[k].copy()
            tt = t + k + 1
            if n_rollback > 0:
                back = min(n_rollback, static_gran, max(0, len(gran_t) - 1))
                if back > 0:
                    src = gran_t[-1 - back]
                    b = (bias[src] if src < tt else seg[src - t]).copy()
                    rolls += 1
                    evidence = max(0, evidence - back)
            t, state, static_gran = tt, 0, 0
        else:
            bias[t:] = b
            cor = stream[t:] - b
            s = np.abs(ma2(cor, W)).max(axis=1)
            if mode == 'table':
                idx = min(evidence >> SHIFT, NBUCK - 1)
                thr_on = max(THR_TAB[idx], BASE)
            else:
                thr_on = BASE if mode == 'flat' else 1e9
            k = first_run(s < thr_on * 0.6, DEB_OFF)
            if k < 0:
                motion_frames += n - t
                break
            motion_frames += k
            t, state = t + k + 1, 1
    return bias, decl, rolls, motion_frames


def vib_tones(n, rms, f_lo, f_hi, seed, n_tone=12):
    rng = np.random.default_rng(seed)
    t = np.arange(n) * TS
    fs = np.geomspace(f_lo, f_hi, n_tone)
    ph = rng.uniform(0, 2 * np.pi, size=(3, n_tone))
    out = np.zeros((n, 3))
    for ax in range(3):
        for i, f in enumerate(fs):
            out[:, ax] += np.sin(2 * np.pi * f * t + ph[ax, i])
    out *= rms / np.sqrt(out.var(axis=0).mean())
    return out


def main(argv):
    ap = argparse.ArgumentParser()
    ap.add_argument("long")
    ap.add_argument("--f-lo", type=float, default=8.0)
    ap.add_argument("--f-hi", type=float, default=120.0)
    a = ap.parse_args(argv)

    d, _ = load_frames(a.long)
    rawf = d[:, 0:3].astype(np.float64)
    n = min(len(rawf), 480000)                     # 60 s
    raw = rawf[:n]
    w60 = int(60 * FS)
    truth_full = np.empty_like(rawf)
    for ax in range(3):
        m = moving_average(rawf[:, ax], w60)
        truth_full[:, ax] = np.concatenate([np.full(w60 // 2, m[0]), m,
                                            np.full(len(rawf) - len(m) - w60 // 2, m[-1])])
    truth = truth_full[:n]
    print("base = %d frames (%.1f s) of the real static record; truth = 60 s centred mean"
          % (n, n * TS))
    print("vibration = %d tones log-spaced %.0f..%.0f Hz, re-quantised to 1/16.4 dps"
          % (12, a.f_lo, a.f_hi))
    print("")
    print("=" * 104)
    print("continuous vibration")
    print("  rms[dps] peak[dps]  mode    decl  rolls  motion%   bias rms err   bias max err"
          "   s:max / p99.99")
    for rms in (0.0, 0.05, 0.1, 0.2, 0.5, 1.0, 2.0):
        vib = vib_tones(n, rms, a.f_lo, a.f_hi, 7) if rms > 0 else np.zeros((n, 3))
        stream = np.round((raw + vib) * 16.4) / 16.4
        s = np.abs(ma2(stream - truth, W)).max(axis=1) if rms > 0 else np.zeros(n)
        for mode in ('none', 'flat', 'table'):
            bias, decl, rolls, mf = run(stream, mode)
            err = bias - truth
            print("  %7.3f %9.3f  %-6s  %4d  %5d  %6.1f%%   %.3e      %.3e      %.4f / %.4f"
                  % (rms, np.abs(vib).max(), mode, decl, rolls, 100.0 * mf / n,
                     float(np.sqrt((err ** 2).mean())), float(np.abs(err).max()),
                     s.max(), np.percentile(s, 99.99)))
        print("")

    print("=" * 104)
    print("bursty vibration (1 s on / 1 s off, rms = 0.5 dps)")
    vib = vib_tones(n, 0.5, a.f_lo, a.f_hi, 11)
    gate = (np.arange(n) // int(1.0 * FS)) % 2 == 0
    vib[~gate] = 0.0
    stream = np.round((raw + vib) * 16.4) / 16.4
    for mode in ('none', 'flat', 'table'):
        bias, decl, rolls, mf = run(stream, mode)
        err = bias - truth
        print("  %-6s decl=%4d rolls=%4d motion=%5.1f%%  bias rms err=%.3e max=%.3e"
              % (mode, decl, rolls, 100.0 * mf / n,
                 float(np.sqrt((err ** 2).mean())), float(np.abs(err).max())))
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
