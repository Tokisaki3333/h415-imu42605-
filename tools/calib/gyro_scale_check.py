# -*- coding: utf-8 -*-
"""
决定性实验：在留出的翻滚校验记录上扫陀螺标度修正，看末态误差（起末姿态差）在哪最小。

两套结论打架：
  闭合法         y 标度误差 ≈ 0.011%（末态误差 2.261 deg 就是它做到的）
  重力椭圆联立   y 标度误差 ≈ -0.32%（两条独立单轴记录 + 两种参数化都一致）
谁对，就看末态误差在哪一点最小。

性能：四元数连乘用**倍增归约**（vectorized pairwise reduction），不要逐样本 Python 循环
      —— 28.5 万样本 x 160 个扫描点的逐样本循环会直接卡死。
"""
import sys, os
import numpy as np
sys.path.insert(0, r'C:\Users\33\Documents\v2\h415-imu42605-\tools\calib')
from jf_load import load_jf


def qmul_arr(a, b):
    aw, ax, ay, az = a.T
    bw, bx, by, bz = b.T
    return np.stack([aw*bw-ax*bx-ay*by-az*bz,
                     aw*bx+ax*bw+ay*bz-az*by,
                     aw*by-ax*bz+ay*bw+az*bx,
                     aw*bz+ax*by-ay*bx+az*bw], 1)


def qprod_scan(dq):
    """按顺序把所有增量四元数乘起来，倍增归约"""
    cur = dq
    while len(cur) > 2:
        n = len(cur)//2
        prod = qmul_arr(cur[:2*n:2], cur[1:2*n:2])
        if len(cur) % 2:
            prod = np.vstack([prod, cur[-1:]])
        cur = prod
    return cur[0] if len(cur) == 1 else qmul_arr(cur[:1], cur[1:])[0]


def expq_arr(v):
    t = np.linalg.norm(v, axis=1)
    out = np.zeros((len(v), 4))
    out[:, 0] = 1.0
    nz = t > 1e-13
    half = t[nz]*0.5
    out[nz, 0] = np.cos(half)
    out[nz, 1:] = np.sin(half)[:, None]*v[nz]/t[nz][:, None]
    return out


def increments(q):
    """逐样本机体系增量旋转矢量：dq = conj(q[k]) ⊗ q[k+1] -> 旋转矢量"""
    qc = q.copy(); qc[:, 1:] *= -1.0
    aw, ax, ay, az = qc[:-1].T
    bw, bx, by, bz = q[1:].T
    v = np.stack([aw*bx+ax*bw+ay*bz-az*by,
                  aw*by-ax*bz+ay*bw+az*bx,
                  aw*bz+ax*by-ay*bx+az*bw], 1)
    ww = aw*bw-ax*bx-ay*by-az*bz
    n = np.linalg.norm(v, axis=1)
    ang = 2.0*np.arctan2(n, ww)
    return v/np.maximum(n, 1e-15)[:, None]*ang[:, None]


fn = sys.argv[1]
p = fn if os.path.exists(fn) else os.path.join('..', fn)
A = np.asarray(load_jf(p), dtype=np.float64)
q = A[:, :4].copy(); q /= np.linalg.norm(q, axis=1, keepdims=True)
RV = increments(q)
print("记录 %s  %d 帧 %.1f s   增量已算好" % (os.path.basename(fn), len(A), len(A)/8027.0))


def end_err(eps_pct):
    """eps_pct: 逐轴标度误差百分数；除以 (1+eps) 即 S_old/S_new"""
    sc = 1.0/(1.0+np.asarray(eps_pct)/100.0)
    dq = expq_arr(RV*sc[None, :])
    tot = qprod_scan(dq)
    qe = qmul_arr(q[:1], tot[None, :])[0]
    qe /= np.linalg.norm(qe)
    return np.degrees(2*np.arccos(np.clip(abs(np.dot(qe, q[0])), -1, 1)))


e0 = end_err([0.0, 0.0, 0.0])
print("自检：修正=0 时末态误差 %.4f deg（应与从原记录直接算的一致）" % e0)

print("\n扫 y 轴（x/z 不动）:")
print("   %12s %14s" % ("eps_y[%]", "末态误差[deg]"))
rows = []
for ey in [-1.2, -1.0, -0.8, -0.6, -0.5, -0.4, -0.375, -0.32, -0.265, -0.2, -0.1, 0.0, 0.1, 0.2, 0.4, 0.6]:
    e = end_err([0.0, ey, 0.0])
    rows.append((e, ey))
    print("   %12.3f %14.4f" % (ey, e))
rows.sort()
print("   最小点: eps_y = %+.3f%%  末态误差 %.4f deg" % (rows[0][1], rows[0][0]))

print("\n三维粗扫:")
best = (1e9, None)
for ex in [-0.6, -0.3, 0.0, 0.3, 0.6]:
    for ey in [-1.0, -0.7, -0.4, -0.2, 0.0, 0.2]:
        for ez in [-0.6, -0.3, 0.0, 0.3, 0.6]:
            e = end_err([ex, ey, ez])
            if e < best[0]:
                best = (e, (ex, ey, ez))
print("   最优 eps = (x %+.2f%%, y %+.2f%%, z %+.2f%%)  末态误差 %.4f deg"
      % (*best[1], best[0]))
print("   对照: 全 0 -> %.4f deg   （重力法给的 x/z 修正 %+.2f%% / %+.2f%% 代进去 -> %.4f deg）"
      % (e0, 0.01, -0.05, end_err([0.01, -0.32, -0.05])))
