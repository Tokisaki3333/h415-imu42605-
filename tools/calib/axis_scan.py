# -*- coding: utf-8 -*-
"""快速诊断：每条记录绕哪根机体轴转、静止点数、总转角"""
import os, sys
import numpy as np

_here = os.path.dirname(os.path.abspath(__file__))
for _p in (_here, os.path.join(_here, 'h415-imu42605-', 'tools', 'calib')):
    if os.path.isfile(os.path.join(_p, 'jf_load.py')):
        sys.path.insert(0, _p)
        break
from jf_load import load_jf

FPS = 8027.0
W_A = 4000
ACC_B0 = np.array([-10.10, -15.42, 43.72])
ACC_S0 = np.array([2028.48, 2040.78, 2016.07])
AX = 'xyz'

for fn in sys.argv[1:]:
    p = fn if os.path.exists(fn) else os.path.join('..', fn)
    A = np.asarray(load_jf(p), dtype=np.float64)
    N = len(A)
    q = A[:, :4].copy(); q /= np.linalg.norm(q, axis=1, keepdims=True)
    acc = A[:, 4:7]
    a_b = (acc - ACC_B0)/ACC_S0
    M = np.cumsum(np.vstack([np.zeros((1, 3)), a_b]), 0)
    kk = np.arange(N)
    sa = np.abs((M[np.minimum(kk+1, N)]-M[np.maximum(kk+1-W_A, 0)]) /
                np.maximum(np.minimum(kk+1, N)-np.maximum(kk+1-W_A, 0), 1)[:, None]
                - (M[np.maximum(kk+1-W_A, 0)]-M[np.maximum(kk+1-2*W_A, 0)]) /
                np.maximum(np.maximum(kk+1-W_A, 0)-np.maximum(kk+1-2*W_A, 0), 1)[:, None]
                ).max(1)*1000
    if A.shape[1] >= 10:
        GB = np.array([1.0334, 0.8494, 12.0910])      # 不减这一项，rate 会被 z 零偏顶到门槛上
        rate = np.linalg.norm((A[:, 7:10]-GB)/16.4, axis=1)
    else:
        r0 = np.degrees(2*np.arccos(np.clip(np.abs((q[:-1]*q[1:]).sum(1)), -1, 1)))*FPS
        rate = np.concatenate([[r0[0]], r0])
    still = (sa < 1.15) & (rate < 1.0)
    segs, i, pts = [], 0, []
    while i < N:
        if still[i]:
            j = i
            while j < N and still[j]:
                j += 1
            if (j-i)/FPS > 0.4:
                segs.append((i, j))
                sl = slice(i+int(0.1*FPS), max(i+int(0.1*FPS)+1, j-int(0.1*FPS)))
                pts.append(acc[sl].mean(0))
            i = j
        else:
            i += 1
    P = np.array(pts)
    print("\n== %s  %d 帧 %.1f s  通道 %d" % (os.path.basename(fn), N, N/FPS, A.shape[1]))
    print("   静止段 %d 个，共 %.1f s (%.0f%%)" % (len(segs), still.sum()/FPS, 100.0*still.sum()/N))
    if len(P) >= 4:
        c = P.mean(0)
        _, S, Vt = np.linalg.svd(P-c, full_matrices=False)
        n = Vt[2]
        k = int(np.argmax(np.abs(n)))
        print("   静止点云 std = (%6.1f %6.1f %6.1f) LSB   半径 %.0f" % (*P.std(0), np.linalg.norm(P-c, axis=1).mean()))
        print("   平面法向 = (%+.4f %+.4f %+.4f) -> 绕机体 %s 轴（失准 %.2f deg）"
              % (*n, AX[k], np.degrees(np.arccos(min(1.0, abs(n[k]))))))
        # 累计转角（四元数）
        tot = 0.0
        qs = []
        for (s, e) in segs:
            sl = slice(s+int(0.1*FPS), max(s+int(0.1*FPS)+1, e-int(0.1*FPS)))
            qm = q[sl].mean(0); qm /= np.linalg.norm(qm)
            qs.append(qm)
        for a in range(1, len(qs)):
            tot += np.degrees(2*np.arccos(np.clip(abs(np.dot(qs[a-1], qs[a])), -1, 1)))
        print("   静止点之间累计转角（四元数）%.1f deg" % tot)
