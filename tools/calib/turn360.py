# -*- coding: utf-8 -*-
"""
单周 360° 整圈（四元数输出）→ 直接量各轴角标度误差。
方法只用姿态本身 + 固件自己的时间基，不含任何主机帧率假设。

单周整圈后末态相对起始姿态的偏差 = 该圈的总角度误差：
    净体轴转角 phi_meas = phi_true * (1 - eps)
    末态姿态偏差 e      = -phi_meas  (mod 360)  =>  |e| = |phi_true| * eps
双向两圈相减消掉零偏：eps = (e_pos - e_neg)/720
"""
import re, sys, os
import numpy as np

np.set_printoptions(precision=4, suppress=True, linewidth=160)

def load(fn):
    raw = re.compile(r'\[RX\]\s*((?:[0-9A-Fa-f]{2}\s*){20})').findall(
        open(fn, 'r', errors='ignore').read())
    b = np.frombuffer(b''.join(bytes.fromhex(s.replace(' ', '')) for s in raw), dtype='<f4')
    q = b.reshape(-1, 5)[:, :4].astype(np.float64)
    q /= np.linalg.norm(q, axis=1, keepdims=True)
    return q

def mul(a, b):
    aw, ax, ay, az = a; bw, bx, by, bz = b
    return np.array([aw*bw-ax*bx-ay*by-az*bz, aw*bx+ax*bw+ay*bz-az*by,
                     aw*by-ax*bz+ay*bw+az*bx, aw*bz+ax*by-ay*bx+az*bw])

def rv(q):
    s = 1.0 if q[0] >= 0 else -1.0
    v = q[1:] * s
    n = np.linalg.norm(v)
    return np.zeros(3) if n < 1e-14 else np.degrees(2*np.arctan2(n, abs(q[0]))) * v / n

def incr(q):
    q0, q1 = q[:-1], q[1:]
    w0, x0, y0, z0 = q0.T; w1, x1, y1, z1 = q1.T
    dw = w0*w1+x0*x1+y0*y1+z0*z1; dx = w0*x1-x0*w1-y0*z1+z0*y1
    dy = w0*y1+x0*z1-y0*w1-z0*x1; dz = w0*z1-x0*y1+y0*x1-z0*w1
    ng = dw < 0; dw[ng], dx[ng], dy[ng], dz[ng] = -dw[ng], -dx[ng], -dy[ng], -dz[ng]
    nv = np.sqrt(dx*dx+dy*dy+dz*dz)
    k = np.where(nv > 1e-15, 2*np.arctan2(nv, np.clip(dw, -1, 1))/np.maximum(nv, 1e-300), 0)
    return np.stack([dx*k, dy*k, dz*k], 1) * 180.0/np.pi

FILES = sys.argv[1:] or [
    'serial_runtime_20260913_190158_640_export.txt',
    'serial_runtime_20260913_190359_575_export.txt',
    'serial_runtime_20260913_190713_772_export.txt',
    'serial_runtime_20260913_190745_795_export.txt',
]

rows = []
for fn in FILES:
    if not os.path.exists(fn):
        print("MISSING %s" % fn); continue
    q = load(fn); d = incr(q)
    mag = np.linalg.norm(d, axis=1) * 8030.0
    idx = np.flatnonzero(mag > 1.0)
    seg = (idx[0], idx[-1]) if len(idx) else (0, len(d)-1)
    net = d[seg[0]:seg[1]+1].sum(axis=0)
    ax = int(np.argmax(np.abs(net)))
    qi = np.array([q[0][0], -q[0][1], -q[0][2], -q[0][3]])
    e = rv(mul(qi, q[-1]))
    print("%s  N=%6d %.1fs  主=%s 净转角 %+9.4f  体轴余量 [%+6.2f %+6.2f]  末态偏差 [%+7.4f %+7.4f %+7.4f]  |e|=%.4f"
          % (fn[-25:-11], len(q), len(q)/8030, 'xyz'[ax], net[ax],
             net[(ax+1) % 3], net[(ax+2) % 3], e[0], e[1], e[2], np.linalg.norm(e)))
    rows.append((ax, net, e))

print("\n================ 按轴 / 方向分组 ================")
for ax in range(3):
    g = [r for r in rows if r[0] == ax]
    if len(g) < 2:
        continue
    pos = [r for r in g if r[1][ax] > 0]
    neg = [r for r in g if r[1][ax] < 0]
    print("\n--- %s 轴 ---" % 'xyz'[ax])
    for r in g:
        # 末态偏差沿主轴的分量，符号按转动方向归一
        sg = 1.0 if r[1][ax] > 0 else -1.0
        print("   净 %+9.4f deg -> 少转 %7.4f deg (%.4f%%)   末态偏差主分量 %+8.4f deg (归一后 %+8.4f)"
              % (r[1][ax], 360 - abs(r[1][ax]), 100*(360-abs(r[1][ax]))/360,
                 r[2][ax], -sg * r[2][ax]))
    if pos and neg:
        ep = pos[0][2]; em = neg[0][2]
        eps = (ep - em)/720.0
        bias = (ep + em)/2.0
        print("   双向: 零偏项 %s deg (可忽略)" % np.array2string(bias, precision=4))
        print("   双向: eps = (e+ - e-)/720 = %s  ->  %s %%"
              % (np.array2string(eps, precision=7), np.array2string(eps*100, precision=4)))
        SC = np.array([16.3182, 16.5382, 16.4849])
        print("   该轴隐含正确标度 = %.4f  (当前 %.4f)" % (SC[ax]/(1+eps[ax]), SC[ax]))
    else:
        e_all = np.array([r[2] for r in g])
        sg = np.array([1.0 if r[1][ax] > 0 else -1.0 for r in g])
        print("   单向重复 %d 次: eps = %s %%  (均值 %+.4f%%, 散布 %.4f%%)"
              % (len(g), np.array2string(np.array([100*(360-abs(r[1][ax]))/360 for r in g]), precision=4),
                 np.mean([100*(360-abs(r[1][ax]))/360 for r in g]),
                 np.ptp([100*(360-abs(r[1][ax]))/360 for r in g])))
