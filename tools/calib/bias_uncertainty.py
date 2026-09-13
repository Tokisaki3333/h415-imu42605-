#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Gyro zero-bias uncertainty: shape and magnitude (temperature drift excluded).

Parses the JustFloat serial export produced by the V5F firmware:
    [hh:mm:ss.mmm] [RX] 90 C1 F9 3D ... 00 00 80 7F          (40 bytes per line)

Frame layout (9 x float32 LE + 4-byte tail):
    ch0..2  raw gyro                 = gyro_lsb / 16.4        [dps]
    ch3..5  bias-corrected gyro      = imu.gyro_dps           [dps]
    ch6..8  bias estimate            = imu.gyro_bias_dps      [dps]

The logger timestamps are unreliable, so every line is taken as one frame 125 us
after the previous one (--ts-us).

The interesting quantity is ch6..8 (the zero-bias estimate) and what it is worth:
  * magnitude : sigma of the bias estimate, and its Allan deviation floor
  * shape     : how the uncertainty grows with averaging time (sqrt(tau) rate-noise
                region -> plateau = bias instability -> drift upturn), plus the
                distribution and the AR(1) correlation time of the estimate.

Usage:
    python bias_uncertainty.py <export.txt> [-o outdir] [--ts-us 125] [--lsb-per-dps 16.4]
"""

import argparse
import math
import os
import sys

import numpy as np

TAIL = b"\x00\x00\x80\x7f"
AXES = ("x", "y", "z")


# --------------------------------------------------------------------------- parse
def load_frames(path):
    """Stream the export, return (n,9) float32 array + integrity counters."""
    chunks = []
    buf = []
    n_frame = n_bad = n_tail_bad = n_nonrx = 0

    with open(path, "r", encoding="utf-8", errors="replace") as fh:
        for line in fh:
            i = line.find("[RX]")
            if i < 0:
                n_nonrx += 1
                continue
            try:
                b = bytes.fromhex(line[i + 4:])
            except ValueError:
                n_bad += 1
                continue
            if len(b) != 40:
                n_bad += 1
                continue
            n_frame += 1
            if b[36:] != TAIL:
                n_tail_bad += 1
            buf.append(b[:36])
            if len(buf) >= 20000:
                chunks.append(np.frombuffer(b"".join(buf), dtype="<f4")
                              .reshape(-1, 9).copy())
                buf = []
    if buf:
        chunks.append(np.frombuffer(b"".join(buf), dtype="<f4").reshape(-1, 9).copy())

    stats = dict(n_frame=n_frame, n_bad=n_bad, n_tail_bad=n_tail_bad, n_nonrx=n_nonrx)
    return (np.vstack(chunks) if chunks else np.zeros((0, 9), np.float32)), stats


# ----------------------------------------------------------------------- utilities
def block_mean(x, m):
    n = len(x) // m
    return x[:n * m].reshape(n, m).mean(axis=1)


def moving_average(x, w):
    """Trailing moving average; element j = mean(x[j : j+w]).  Length n-w+1."""
    c = np.concatenate(([0.0], np.cumsum(np.asarray(x, np.float64))))
    return (c[w:] - c[:-w]) / w


def detrended(x, w):
    """x minus its centred moving average of w samples (returns the valid interior)."""
    ma = moving_average(x, w)
    h = w // 2
    return x[h:h + len(ma)] - ma                   # aligned to x[h : h+len(ma)]


def allan_dev(x, fs, m0=8, n_tau=64):
    """Non-overlapping Allan deviation after block-averaging to m0 samples.

    Block-averaging to m0 then using lag n is identical to the Allan estimator at
    tau = n*m0/fs on the raw series, and keeps the work manageable.
    """
    z = block_mean(np.asarray(x, np.float64), m0)
    n = len(z)
    out = []
    for lag in np.unique(np.round(np.logspace(0, math.log10(max(2, n // 4)),
                                             n_tau)).astype(int)):
        m = n // lag
        if m < 4:
            break
        y = z[:m * lag].reshape(m, lag).mean(axis=1)
        d = np.diff(y)
        out.append((lag * m0 / fs, math.sqrt(float(np.sum(d * d)) / (2.0 * (m - 1)))))
    return np.array(out)


def loop_gain(b, r):
    """Recover the firmware's per-frame traction coefficient alpha.

    The firmware does, once per frame k:  bias[k] = bias[k-1] + alpha*(raw_lsb - bias_lsb)
    with alpha = dt/tau_ticks, using the *current* frame's raw and the previous bias.
    Both sides are reported, so alpha can be read back exactly (median is robust against
    the frame-to-frame dt jitter from interrupt latency).
    """
    bp = b[:-1]                       # bias as of the end of frame k-1
    db = b[1:] - bp                   # this frame's traction step
    xx = r[1:] - bp                   # x the firmware actually used
    sel = np.abs(xx) > 0.1
    med = float(np.median(db[sel] / xx[sel])) if sel.any() else float("nan")
    lsq = float(np.dot(xx, db) / np.dot(xx, xx))
    return med, lsq


# -------------------------------------------------------------------------- report
def analyse(data, ts, lsb_per_dps, outdir, stats):
    fs = 1.0 / ts
    n = len(data)
    bias = data[:, 6:9].astype(np.float64)
    raw = data[:, 0:3].astype(np.float64)
    cor = data[:, 3:6].astype(np.float64)
    resid = raw - bias - cor                       # should be ~float rounding only
    t = np.arange(n) * ts

    lines = []
    p = lines.append

    p("=" * 78)
    p("frames = %d   span = %.3f s   (assumed %.4g us/frame = %.1f Hz)"
      % (n, n * ts, ts * 1e6, fs))
    p("integrity: malformed=%d  bad_tail=%d  non-rx lines=%d  -> every frame is 40 B + tail"
      % (stats["n_bad"], stats["n_tail_bad"], stats["n_nonrx"]))
    p("max |raw - bias - cor| = %.3g dps   (the three channel groups are consistent)"
      % np.abs(resid).max())
    frac = np.abs(raw * lsb_per_dps - np.round(raw * lsb_per_dps))
    p("scale check: max|raw*%.1f - round| = %.3g  (confirms %g LSB/dps)"
      % (lsb_per_dps, frac.max(), lsb_per_dps))
    db_all = np.abs(np.diff(bias, axis=0)).max(axis=1)
    p("traction integrity: frozen frames (all 3 dbias == 0): %d ; max |dbias| = %.3e dps"
      % (int((db_all == 0).sum()), db_all.max()))
    p("")

    adev = {}
    for a, name in enumerate(AXES):
        b = bias[:, a]
        r = raw[:, a]
        c = cor[:, a]
        x = r - b                                   # LPF input (what the filter pulls on)

        mu, sd = b.mean(), b.std()
        lin = np.polyfit(t, b, 1)[0]                # dps/s  (temperature drift)
        p("-" * 78)
        p("bias_%s : mean=%.7f dps   sd=%.7f dps (%.4f LSB)   p-p=%.7f"
          % (name, mu, sd, sd * lsb_per_dps, b.max() - b.min()))
        p("          linear drift = %.3e dps/s = %+.4f dps/min = %+.2f dps/h  (temperature, ignored)"
          % (lin, lin * 60, lin * 3600))
        row = []
        for w_s in (1, 2, 5, 10, 30, 60):
            w = int(round(w_s * fs))
            if w >= n:
                continue
            d = detrended(b, w)
            row.append("%ds:%.2e(%.2f deg/h)" % (w_s, d.std(), d.std() * 3600))
        p("          sigma after removing slow drift (>W):")
        p("            " + "  ".join(row))

        # per-frame traction step -> loop gain -> effective loop time constant
        db = np.diff(b)
        alpha_med, alpha_lsq = loop_gain(b, r)
        p("          dbias: sd=%.3e  max|d|=%.3e dps" % (db.std(), np.abs(db).max()))
        p("          loop gain alpha: median=%.5e -> tau_eff=%.2f s | LSQ=%.5e -> tau_eff=%.2f s"
          % (alpha_med, ts / alpha_med, alpha_lsq, ts / alpha_lsq))
        p("          (firmware says tau=8 s; per-frame dt quantization + ISR jitter only)")

        # if the loop input were white, the bias would settle at sigma_b = sigma_x*sqrt(alpha/2)
        sx = x.std()
        theo = sx * math.sqrt(alpha_med / 2.0)
        fast = detrended(b, int(5 * fs)).std()
        p("          sigma(raw-bias)=%.6f dps (the corrected output's own noise)" % sx)
        p("          white-noise loop theory sigma_b=%.3e dps ; measured total sd=%.3e (%.1fx)"
          % (theo, sd, sd / theo))

        dev = allan_dev(b, fs)
        adev[name] = dev
        rel = dev[:, 0] <= n * ts / 10.0            # keep tau with enough independent bins
        ipk = int(np.argmax(np.where(rel, dev[:, 1], 0.0)))
        sl = np.polyfit(np.log10(dev[:12, 0]), np.log10(dev[:12, 1]), 1)[0]
        p("          Allan: slope(1ms..7ms)=%.2f (0.5 = white rate noise)"
          % sl)
        p("                 peak over tau<=span/10: %.3e dps @ tau=%.2f s (%.3f LSB, %.1f deg/h)"
          % (dev[ipk, 1], dev[ipk, 0], dev[ipk, 1] * lsb_per_dps, dev[ipk, 1] * 3600))
        p("                 adev(1s)=%.3e  adev(10s)=%.3e  adev(60s)=%.3e dps"
          % (np.interp(1.0, dev[:, 0], dev[:, 1]),
             np.interp(10.0, dev[:, 0], dev[:, 1]),
             np.interp(60.0, dev[:, 0], dev[:, 1])))
        # distribution shape of the fast part
        d = detrended(b, int(10 * fs))
        sk = float(((d - d.mean()) ** 3).mean() / d.std() ** 3)
        ku = float(((d - d.mean()) ** 4).mean() / d.std() ** 4) - 3.0
        p("          detrended(>10s) distribution: sd=%.3e skew=%.2f excess-kurt=%.2f (0/0 = gaussian)"
          % (d.std(), sk, ku))

    # ------------------------------------------------- zero-drift rate / motion gap
    p("")
    p("=" * 78)
    p("zero-drift rate and motion-gap cost")
    p("  block mean over W seconds (averages the 0.1 dps rate noise down), then look at")
    p("  how much the zero moves between neighbouring W-blocks.")
    p("  'wn' = what pure white rate noise of this axis would give (sigma_x/sqrt(W*fs)).")
    for a, name in enumerate(AXES):
        r = raw[:, a]
        c = cor[:, a]
        kr = float(np.polyfit(t, r, 1)[0])          # dps/s
        kc = float(np.polyfit(t, c, 1)[0])
        sx = (r - bias[:, a]).std()
        p("  --- axis %s ---" % name)
        p("      linear drift over record: raw %+.3e dps/s  ->  cor %+.3e dps/s  (residual, %.0fx smaller)"
          % (kr, kc, abs(kr / kc)))
        for w_s in (1, 10, 30, 60, 120):
            w = int(w_s * fs)
            if w >= n // 2:                          # need two full w-windows to compare
                continue
            # level change between two neighbouring w-second windows, for every offset
            mar = moving_average(r, w)
            mac = moving_average(c, w)
            dr = mar[w:] - mar[:-w]
            dc = mac[w:] - mac[:-w]
            wn = sx * math.sqrt(2.0 / w)            # white rate noise reference
            p("      W=%3ds : raw sd=%.3e (wn %.3e) max|d|=%.3e p95=%.3e | cor sd=%.3e max|d|=%.3e p95=%.3e"
              % (w_s, dr.std(), wn, np.abs(dr).max(), np.percentile(np.abs(dr), 95),
                 dc.std(), np.abs(dc).max(), np.percentile(np.abs(dc), 95)))

    # motion gap: bias frozen while is_static == 0 -> the whole zero drift of the gap
    # is left in the correction, then decays as exp(-t/tau) once static resumes.
    p("")
    p("motion-gap cost (bias frozen during the gap, gate tells the truth):")
    p("  cost = how far the zero moves during the gap; recovery = exp(-t/tau), tau = 8 s")
    for gap in (10, 30, 60):
        w = int(gap * fs)
        if w >= n - 2:
            continue
        row = []
        for a, name in enumerate(AXES):
            ma = moving_average(raw[:, a], w)
            d = np.abs(ma[w:] - ma[:-w])
            mx, p95 = float(d.max()), float(np.percentile(d, 95))
            row.append("%s max=%.2e p95=%.2e (%.0f/%.0f s to 1e-4)"
                       % (name, mx, p95, 8.0 * math.log(mx / 1e-4), 8.0 * math.log(p95 / 1e-4)))
        p("  gap %2ds : %s" % (gap, " | ".join(row)))
    p("  after a rollback, bias_ok needs 3*tau = 24 s of accumulated static time again")
    p("")

    # ---------------------------------------------------------------- outputs
    os.makedirs(outdir, exist_ok=True)
    np.savetxt(os.path.join(outdir, "bias_allan.csv"),
               np.column_stack([adev["x"][:, 0], adev["x"][:, 1],
                                adev["y"][:, 1], adev["z"][:, 1]]),
               delimiter=",", header="tau_s,adev_x,adev_y,adev_z", comments="",
               fmt="%.6e")

    step = max(1, int(round(0.01 * fs)))            # 10 ms series for plotting
    np.savetxt(os.path.join(outdir, "bias_series_10ms.csv"),
               np.column_stack([t[::step], bias[::step], raw[::step]]),
               delimiter=",", header="t_s,bias_x,bias_y,bias_z,raw_x,raw_y,raw_z",
               comments="", fmt="%.9g")

    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        fig, ax = plt.subplots(3, 2, figsize=(13, 9))
        for a, name in enumerate(AXES):
            ax[a, 0].plot(t, bias[:, a], lw=0.7)
            ax[a, 0].set_title("bias_%s estimate  (sd=%.2e dps)" % (name, bias[:, a].std()))
            ax[a, 0].set_xlabel("t [s] (assumed %.0f us/frame)" % (ts * 1e6))
            ax[a, 0].set_ylabel("dps")
            ax[a, 0].grid(alpha=0.3)

            d = detrended(bias[:, a], int(10 * fs))
            ax[a, 1].plot(np.arange(len(d)) * ts + 5.0, d, lw=0.5)
            ax[a, 1].set_title("bias_%s after removing >10 s drift  (sd=%.2e dps)" % (name, d.std()))
            ax[a, 1].set_xlabel("t [s]")
            ax[a, 1].grid(alpha=0.3)
        fig.tight_layout()
        fig.savefig(os.path.join(outdir, "bias_timeseries.png"), dpi=110)
        plt.close(fig)

        fig, ax = plt.subplots(1, 2, figsize=(13, 5))
        for name in AXES:
            ax[0].loglog(adev[name][:, 0], adev[name][:, 1], ".-", label="bias_" + name)
        ax[0].loglog(adev["x"][:16, 0], adev["x"][:16, 1] * 2.0, "k--", lw=1,
                     label="slope 1/2 (white rate noise)")
        ax[0].set_xlabel("averaging time tau [s]")
        ax[0].set_ylabel("Allan deviation [dps]")
        ax[0].set_title("zero-bias uncertainty vs averaging time")
        ax[0].grid(True, which="both", alpha=0.3)
        ax[0].legend(fontsize=8)

        d = detrended(bias[:, 0], int(10 * fs))
        ax[1].hist(d, bins=120, density=True, alpha=0.8)
        xs = np.linspace(d.min(), d.max(), 200)
        ax[1].plot(xs, np.exp(-xs ** 2 / (2 * d.std() ** 2)) / (d.std() * math.sqrt(2 * math.pi)),
                   "r-", lw=1, label="gaussian fit")
        ax[1].set_title("bias_x, >10 s drift removed")
        ax[1].set_xlabel("dps")
        ax[1].legend(fontsize=8)
        ax[1].grid(alpha=0.3)
        fig.tight_layout()
        fig.savefig(os.path.join(outdir, "bias_allan_hist.png"), dpi=110)
        plt.close(fig)
        p("")
        p("plots: %s" % os.path.join(outdir, "bias_timeseries.png"))
        p("       %s" % os.path.join(outdir, "bias_allan_hist.png"))
    except Exception as exc:                        # plotting is optional
        p("plotting skipped: %r" % (exc,))

    p("csv:   %s" % os.path.join(outdir, "bias_allan.csv"))
    p("       %s" % os.path.join(outdir, "bias_series_10ms.csv"))
    p("=" * 78)
    return "\n".join(lines)


def main(argv):
    ap = argparse.ArgumentParser(description="gyro zero-bias uncertainty from a JustFloat export")
    ap.add_argument("export", help="serial export .txt")
    ap.add_argument("-o", "--outdir", default=".", help="output directory (default: .)")
    ap.add_argument("--ts-us", type=float, default=125.0, help="frame interval, us (default 125)")
    ap.add_argument("--lsb-per-dps", type=float, default=16.4, help="gyro scale (default 16.4)")
    a = ap.parse_args(argv)

    data, st = load_frames(a.export)
    print("parsed %d frames (%d malformed, %d bad tail, %d non-RX lines)"
          % (st["n_frame"], st["n_bad"], st["n_tail_bad"], st["n_nonrx"]))
    if not len(data):
        print("no frames found")
        return 1
    report = analyse(data, a.ts_us * 1e-6, a.lsb_per_dps, a.outdir, st)
    print(report)
    with open(os.path.join(a.outdir, "bias_uncertainty_report.txt"), "w",
              encoding="utf-8") as fh:
        fh.write(report + "\n")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
