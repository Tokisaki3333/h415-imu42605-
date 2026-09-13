# -*- coding: utf-8 -*-
"""从 6 条整转记录解 3x3 标度+轴间耦合矩阵（鲁棒版：整段积分 + 首尾静止段去偏置）。"""
import re, os, sys
import numpy as np

FPS = 8029.0
DT = 1.0 / FPS
SCALE = np.array([16.3182, 16.5382, 16.4849])
KFIX = 16.4 / SCALE

RX = re.compile(r'\[RX\]\s*((?:[0-9A-Fa-f]{2}\s*){40})')

def load(path):
    fr = []
    with open(path, 'r', errors='ignore') as f:
        for line in f:
            m = RX.search(line)
            if not m:
                continue
            b = bytes.fromhex(m.group(1).replace(' ', ''))
            if b[-4:] != b'\x00\x00\x80\x7f':
                continue
            fr.append(np.frombuffer(b[:36], dtype='<f4'))
    return np.array(fr)

def integral(a):
    """整段积分（校正通道），用首尾各 0.5 s 的均值去掉偏置。单位 deg。"""
    w = a[:, 3:6] * KFIX
    n = max(50, int(0.5 * FPS))
    off = 0.5 * (w[:n].mean(axis=0) + w[-n:].mean(axis=0))
    return (w - off).sum(axis=0) * DT

CASES = [
    ('serial_runtime_20260913_181436_858_export.txt', 2, 360.0, 'z +360'),
    ('serial_runtime_20260913_181548_799_export.txt', 2, -360.0, 'z -360'),
    ('serial_runtime_20260913_182119_546_export.txt', 1, 180.0, 'y +180'),
    ('serial_runtime_20260913_182228_199_export.txt', 1, -180.0, 'y -180'),
    ('serial_runtime_20260913_182626_777_export.txt', 0, 180.0, 'x +180'),
    ('serial_runtime_20260913_182645_181_export.txt', 0, -180.0, 'x -180'),
]

cols = {0: [], 1: [], 2: []}
print("rec      dx(deg)    dy(deg)    dz(deg)    |d|    main   T(s)  peak(dps)")
for fn, ax, phi, tag in CASES:
    if not os.path.exists(fn):
        print("MISSING %s" % fn); continue
    a = load(fn)
    d = integral(a)
    sgn = 1.0 if d[ax] >= 0 else -1.0
    d = d * sgn
    wmax = np.abs(a[:, 3:6] * KFIX).max()
    cols[ax].append(d / abs(phi))
    print("%-7s %9.3f %10.3f %10.3f %8.3f %8.3f %6.2f %9.1f"
          % (tag, d[0], d[1], d[2], np.linalg.norm(d), abs(d[ax]), len(a) * DT, wmax))

S = np.eye(3)
for j in range(3):
    S[:, j] = np.mean(cols[j], axis=0) if cols[j] else np.eye(3)[:, j]

np.set_printoptions(precision=6, suppress=True, linewidth=140)
print("\nS (col = true rotation axis, row = gyro channel):")
print(S)
M = np.linalg.inv(S)
print("\nM = inv(S)  (w_true = M @ w_meas), C literals:")
for i in range(3):
    print("    " + ", ".join("%.7ff" % v for v in M[i]))

print("\noff-diagonal coupling (%):")
for i in range(3):
    for j in range(3):
        if i != j:
            print("  w%d <- w%d : %+7.4f %%" % (i, j, S[i, j] * 100.0))
print("\ndiagonal residual scale (%): " +
      "  ".join("w%d %+.4f%%" % (i, (S[i, i] - 1) * 100) for i in range(3)))
for j in range(3):
    for k in range(j + 1, 3):
        c = np.dot(S[:, j], S[:, k]) / (np.linalg.norm(S[:, j]) * np.linalg.norm(S[:, k]))
        print("  angle(col%d,col%d) = %.4f deg" % (j, k, np.degrees(np.arccos(np.clip(c, -1, 1)))))
