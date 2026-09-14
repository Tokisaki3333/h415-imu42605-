# -*- coding: utf-8 -*-
"""
向心加速度整流的幅度实验判定。

机理：a_c = w^2 * r 恒 >= 0、机体系方向固定 -> 陀螺若有加速度敏感度 S_g，
      产生单向偏差 S_g * a_c，累积角误差 = S_g * ∫a_c dt（正比于 ∫w^2 dt，不是转角）。

判定：同一时长、两个摇晃幅度。
    整流   -> 误差正比于 ∫a_c dt（幅度翻倍误差约翻 4 倍）
    真实转动/零偏 -> 不随幅度变

注意：末态姿态"反了 18 度"要用**最短旋转**算，即 theta = 2*acos(|q_a·q_b|) ∈ [0,180]。
     直接取 log(q_a^-1 q_b) 的模会得到 342 deg —— 那是同一个姿态，不是 18 倍误差。

用法：python centrifugal_check.py <记录> [<记录> ...]
"""
import sys, os
import numpy as np
sys.path.insert(0, r'C:\Users\33\Documents\v2\h415-imu42605-\tools\calib')
from jf_load import load_jf

GB = np.array([1.0334, 0.8494, 12.0910])
GS = np.array([16.2753, 16.4793, 16.4235])       # 第 3/4 版混合（x/z 回退，y 用第 4 版）
FPS = 8027.0
W_A = 4000
ACC_B0 = np.array([-10.10, -15.42, 43.72])
ACC_S0 = np.array([2028.48, 2040.78, 2016.07])


def Rm(qq):
    w_, x_, y_, z_ = qq
    return np.array([[1-2*(y_*y_+z_*z_), 2*(x_*y_-w_*z_), 2*(x_*z_+w_*y_)],
                     [2*(x_*y_+w_*z_), 1-2*(x_*x_+z_*z_), 2*(y_*z_-w_*x_)],
                     [2*(x_*z_-w_*y_), 2*(y_*z_+w_*x_), 1-2*(x_*x_+y_*y_)]])


rows = []
for fn in sys.argv[1:]:
    p = fn if os.path.exists(fn) else os.path.join('..', fn)
    if not os.path.exists(p):
        print("缺 %s" % fn); continue
    A = np.asarray(load_jf(p), dtype=np.float64)
    N = len(A)
    q = A[:, :4].copy(); q /= np.linalg.norm(q, axis=1, keepdims=True)
    ab = (A[:, 4:7] - ACC_B0)/ACC_S0
    amag = np.linalg.norm(A[:, 4:7], axis=1)/2048.0
    dps = (A[:, 7:10]-GB)/GS
    M = np.cumsum(np.vstack([np.zeros((1, 3)), ab]), 0)
    kk = np.arange(N)
    sa = np.abs((M[np.minimum(kk+1, N)]-M[np.maximum(kk+1-W_A, 0)]) /
                np.maximum(np.minimum(kk+1, N)-np.maximum(kk+1-W_A, 0), 1)[:, None]
                - (M[np.maximum(kk+1-W_A, 0)]-M[np.maximum(kk+1-2*W_A, 0)]) /
                np.maximum(np.maximum(kk+1-W_A, 0)-np.maximum(kk+1-2*W_A, 0), 1)[:, None]
                ).max(1)*1000
    rate = np.linalg.norm(dps, axis=1)
    still = (sa < 1.15) & (rate < 1.0)
    segs, i = [], 0
    while i < N:
        if still[i]:
            j = i
            while j < N and still[j]:
                j += 1
            if (j-i)/FPS > 0.3:
                segs.append((i, j))
            i = j
        else:
            i += 1
    if len(segs) < 2:
        print("%s 静止段不足" % os.path.basename(fn)); continue
    (s0, e0), (s1, e1) = segs[0], segs[-1]
    qa = q[s0:e0].mean(0); qa /= np.linalg.norm(qa)
    qb = q[s1:e1].mean(0); qb /= np.linalg.norm(qb)
    # 最短旋转角
    e_short = np.degrees(2*np.arccos(np.clip(abs(np.dot(qa, qb)), -1, 1)))
    # 摆幅/积分：只统计运动部分
    i0, i1 = e0, s1
    mv = rate[i0:i1] > 5.0
    w2i = (np.sum(np.radians(dps[i0:i1])**2, axis=1)[mv]).sum()/FPS
    aci = (amag[i0:i1][mv]-1.0).clip(0).sum()/FPS
    wmed = np.median(np.linalg.norm(dps[i0:i1][mv], axis=1)) if mv.sum() else 0
    wmax = np.linalg.norm(dps[i0:i1][mv], axis=1).max() if mv.sum() else 0
    amed = np.median(amag[i0:i1][mv]) if mv.sum() else 0
    # 加速度计给出的一致性（首末静止点的重力方向是否相同）
    ua = ab[s0+int(0.1*FPS):e0-int(0.1*FPS)].mean(0); ua /= np.linalg.norm(ua)
    ub = ab[s1+int(0.1*FPS):e1-int(0.1*FPS)].mean(0); ub /= np.linalg.norm(ub)
    dg = np.degrees(np.arccos(np.clip(np.dot(ua, ub), -1, 1)))
    T = (i1-i0)/FPS
    print("\n== %s" % os.path.basename(fn))
    print("   运动段 %.2f~%.2f s  (%.1f s)，静止段 %d 个" % (i0/FPS, i1/FPS, T, len(segs)))
    print("   |w| 中位 %.0f  最大 %.0f dps    |a| 中位 %.2f g" % (wmed, wmax, amed))
    print("   ∫w²dt = %.1f rad²/s     ∫a_c dt = %.1f g·s" % (w2i, aci))
    print("   首末姿态差（最短旋转）= %.2f deg" % e_short)
    print("   首末重力方向差（加速度计）= %.3f deg  -> 倾角是否复原" % dg)
    rows.append((os.path.basename(fn), T, w2i, aci, e_short, dg))

print("\n" + "=" * 78)
print("幅度对比（整流应正比于 ∫a_c dt / ∫w²dt）")
print("=" * 78)
print("%-34s %8s %12s %10s %10s %10s" % ("记录", "时长s", "∫w²dt", "∫a_c dt", "末态角差", "S_g dps/g"))
for (nm, T, w2, ac, e, dg) in rows:
    sg = e/ac if ac > 1e-9 else float('nan')
    print("%-34s %8.1f %12.1f %10.1f %10.2f %10.2f" % (nm, T, w2, ac, e, sg))
if len(rows) == 2:
    r0, r1 = rows
    print("\n判定：")
    print("   幅度比(∫a_c) = %.2f 倍  ->  误差比 = %.2f 倍" % (r1[3]/r0[3], r1[4]/r0[4]))
    print("   若纯整流，误差比应 = 幅度比 = %.2f" % (r1[3]/r0[3]))
    print("   S_g（两条）: %.2f 与 %.2f dps/g  ->  一致性 %.0f%%"
          % (r0[4]/r0[3], r1[4]/r1[3], 100*(1-abs(r0[4]/r0[3]-r1[4]/r1[3])/max(r0[4]/r0[3], r1[4]/r1[3]))))
