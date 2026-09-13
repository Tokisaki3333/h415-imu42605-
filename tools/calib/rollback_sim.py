#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Rollback sweep by direct simulation.

Builds one stream:  [ 60 s of the long static record ] + [ the real rotation ] +
[ 90 s of the long static record ]   (z axis; the rotation in the new record is on z)

then replays the firmware loop frame by frame:

    static : bias += (raw - bias) * Ts/tau          (tau = 12 s)
    motion : bias frozen
    20 ms  : push a bias snapshot into a 64-deep ring
    edge   : on static->motion, bias = ring[head - N]

The detector is the second processing step:  s = |mean(raw - bias over W)|,
on if s > thr_on for deb_on frames, off if s < thr_off for deb_off frames.
Ground truth is the *un-injected* static record itself, so the bias error is
exactly measurable.

Usage: python rollback_sim.py <static.txt> <rotation.txt> [--tau 12]
"""

import argparse
import math
import sys

import numpy as np

from bias_uncertainty import load_frames, moving_average, detrended

TS = 125e-6
FS = 1.0 / TS
GRAN = int(round(0.020 * FS))          # 160 samples
DEPTH = 64


def lpf(x, alpha, b0=0.0, block=8192):
    beta = 1.0 - alpha
    out = np.empty(len(x), np.float64)
    ar = np.arange(block)
    b = float(b0)
    for s in range(0, len(x), block):
        e = min(len(x), s + block)
        m = e - s
        aw = ar[:m]
        c = np.cumsum(x[s:e] * beta ** (-aw))
        out[s:e] = (beta ** aw) * (beta * b + alpha * c)
        b = out[e - 1]
    return out


def first_run(flag, deb, start=0):
    """index where `deb` consecutive True start (firmware declares the state there)."""
    c = 0
    for i in range(start, len(flag)):
        c = c + 1 if flag[i] else 0
        if c >= deb:
            return i - deb + 1
    return -1


def simulate(stream, tau, W, thr_on, deb_on, thr_off, deb_off, N, init_len):
    """Replay the firmware.

    Step 0 (startup): the first `init_len` frames are averaged and used as the initial
    bias, so the loop starts already converged (no startup transient, no false
    'motion' from an offset corrected output).  The detector is armed after that.
    """
    n = len(stream)
    alpha = TS / tau
    bias = np.empty(n)
    ring = np.zeros(DEPTH)
    head = 0
    state = 1
    b = float(stream[:init_len].mean())
    t = 0
    triggers = []
    steps = []
    gran_idx = []
    while t < n:
        if state == 1:
            if t < init_len:
                m = init_len - t                      # init window only, then re-enter
                seg = lpf(stream[t:t + m], alpha, b0=b)
                bias[t:t + m] = seg
                b = seg[-1]
                g = (GRAN - (t % GRAN)) % GRAN
                while g < m:
                    head = (head + 1) % DEPTH
                    ring[head] = seg[g]
                    gran_idx.append(t + g)
                    if len(gran_idx) > 4 * DEPTH:
                        gran_idx.pop(0)
                    g += GRAN
                t += m
                continue
            seg = lpf(stream[t:], alpha, b0=b)
            cor = stream[t:] - seg
            s = np.abs(moving_average(cor, W))
            k = first_run(s > thr_on, deb_on)
            m = n - t if k < 0 else k + 1
            bias[t:t + m] = seg[:m]
            g = (GRAN - (t % GRAN)) % GRAN
            while g < m:
                head = (head + 1) % DEPTH
                ring[head] = seg[g]
                gran_idx.append(t + g)
                if len(gran_idx) > 4 * DEPTH:
                    gran_idx.pop(0)
                g += GRAN
            if k < 0:
                break
            b = seg[k]
            tt = t + k + 1
            if N > 0 and len(gran_idx) > N:
                src = gran_idx[-1 - N]
                b0 = bias[src] if src < tt else seg[src - t]
                steps.append((tt, b0 - float(bias[tt - 1])))
                b = b0
            triggers.append((tt, "motion"))
            t, state = tt, 0
        else:
            bias[t:] = b
            cor = stream[t:] - b
            s = np.abs(moving_average(cor, W))
            k = first_run(s < thr_off, deb_off)
            if k < 0:
                break
            triggers.append((t + k + 1, "static"))
            t, state = t + k + 1, 1
    return bias, triggers, steps


def main(argv):
    ap = argparse.ArgumentParser()
    ap.add_argument("static_rec")
    ap.add_argument("rotation_rec")
    ap.add_argument("--tau", type=float, default=12.0)
    ap.add_argument("--pre", type=float, default=60.0)
    ap.add_argument("--post", type=float, default=90.0)
    a = ap.parse_args(argv)

    ds, _ = load_frames(a.static_rec)
    dr, _ = load_frames(a.rotation_rec)
    raw_s = ds[:, 2].astype(np.float64)                       # z axis, static
    raw_r = dr[:, 2].astype(np.float64)                       # z axis carries the rotation
    om_r = raw_r - raw_r[int(len(raw_r) - 0.3 * FS):].mean()  # true rotation rate
    np_ = int(round(a.pre * FS))
    n_rot = len(om_r)
    n_post = min(int(round(a.post * FS)), len(raw_s) - np_ - n_rot)
    stream = np.concatenate([raw_s[:np_], om_r + raw_s[np_], raw_s[np_:np_ + n_post]])
    t_rot = np_
    print("stream = %.1f s static + %.3f s rotation (peak %.1f dps, |w|dt=%.2f dps*s)"
          % (a.pre, n_rot * TS, np.abs(om_r).max(), np.abs(om_r).sum() * TS))

    # ------------------------------------------------------------------ step 0: init
    print("")
    print("=" * 100)
    print("step 0: initial bias = mean of the first K frames (stabilises the startup)")
    true0 = float(om_r[:1].size and raw_s[np_:np_ + int(0.5 * FS)].mean())  # settled level
    for K in (1000, 2000, 4000, 8000, 16000):
        b0 = float(raw_s[:K].mean())
        sd = float(raw_s[:K].std()) / math.sqrt(K)
        print("   K=%6d (%4.0f ms): bias0=%.5f dps (err %+.2e +- %.1e)  <- 1 sigma of the mean"
              % (K, K * TS * 1e3, b0, b0 - true0, sd))
    init_len = 4000                                  # 0.5 s
    print("   -> use K=4000 (0.5 s): startup transient removed, detector armed afterwards")

    # ------------------------------------------- step 1: threshold from white noise
    print("")
    print("=" * 100)
    print("step 1: static-detection threshold picked from the white rate noise")
    sig = float(np.mean([ds[:, ax].std() for ax in range(3)]))
    print("   raw rate noise sigma = %.4f dps (avg over axes)" % sig)
    print("      W[ms]  sigma of W-mean   6 sigma   10 sigma   false alarms in 151 s @10 sigma")
    for W in (64, 128, 256, 512, 1024):
        sw = sig / math.sqrt(W)
        ma = np.column_stack([moving_average(ds[:, ax].astype(np.float64) - ds[:, ax].mean(), W)
                              for ax in range(3)])
        s = np.abs(ma).max(axis=1)
        fa = int((s > 10 * sw).sum())
        print("      %5.0f   %12.4f   %8.4f   %8.4f   %d"
              % (W * TS * 1e3, sw, 6 * sw, 10 * sw, fa))
    W_DET, K_SIG = 128, 10.0
    thr_on = K_SIG * sig / math.sqrt(W_DET)
    thr_off = 0.6 * thr_on
    print("   -> use W=%d (%.0f ms), thr_on=%.3f dps (%.0f sigma, %.2f LSB), thr_off=%.3f"
          % (W_DET, W_DET * TS * 1e3, thr_on, K_SIG, thr_on * 16.4, thr_off))
    print("      (a 100 dps/s ramp reaches thr_on after %.0f ms -> rollback must cover that)"
          % (thr_on / 100.0 * 1e3))

    # ----------------------------------------------- ground truth + N sweep
    w60 = int(round(60 * FS))
    truth = moving_average(raw_s, w60)
    truth = np.concatenate([np.full(w60 // 2, truth[0]), truth,
                            np.full(len(raw_s) - len(truth) - w60 // 2, truth[-1])])

    def stats(N):
        bias, trig, steps = simulate(stream, a.tau, W_DET, thr_on, 8, thr_off, 800,
                                     N, init_len)
        L = min(len(bias), len(truth))
        e = bias[:L] - truth[:L]
        mo = slice(t_rot, t_rot + n_rot)
        af = slice(t_rot + n_rot, L)
        pk = float(np.abs(e[mo]).max())
        end = float(abs(e[t_rot + n_rot - 1]))
        ap = float(np.abs(e[af]).max())
        idx = np.flatnonzero(np.abs(e[af]) > 1e-4)
        rec = (idx[-1] + 1) * TS if idx.size else 0.0
        pre = float(np.abs(e[t_rot - int(10 * FS):t_rot]).max())
        st = steps[0][1] if steps else 0.0
        return (trig[0][0] * TS if trig else -1.0), st, pre, pk, end, ap, rec, len(trig)

    print("")
    print("=" * 100)
    print("N sweep with the above (init 0.5 s mean, thr 10 sigma from white noise)")
    print("    N  trigger[s]  rollback[dps]  |err| pre   peak in motion  end of motion"
          "   peak after  recovery>1e-4  edges")
    for N in (0, 1, 2, 3, 4, 5, 6, 8, 10, 12, 16, 24, 32, 48, 64):
        tg, st, pre, pk, end, ap, rec, ne = stats(N)
        print("  %3d  %9.3f  %12.3e  %9.3e  %13.3e  %13.3e  %10.3e  %10.1f s  %5d"
              % (N, tg, st, pre, pk, end, ap, rec, ne))
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
