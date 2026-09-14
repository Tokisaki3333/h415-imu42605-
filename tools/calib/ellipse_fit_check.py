# -*- coding: utf-8 -*-
"""
椭圆中心的拟合方式校验 —— 确认 b_y 的 1.03 mg 分歧不是拟合伪影

动机：085 记录的 11 个静止点在 180 deg 附近挤了三个，角度分布不均。而**代数圆锥直接
最小二乘对角度分布不均是有偏的**，所以必须先排除"椭圆中心偏了"这个可能，才能说
椭圆法与六面法在 b_y 上真的不一致。

三种拟合对比：
  1) 代数圆锥直接最小二乘（accel_ellipse_cal.py 现用）
  2) Fitzgibbon 直接椭圆拟合（带 4ac-b^2=1 约束，公认偏差小得多）
  3) 几何圆拟合（Gauss-Newton 最小化正交距离）—— 故意用一个**更弱的模型**作对照：
     数据其实是 0.6% 椭圆度的椭圆，圆拟合会把椭圆度吸收成中心偏移，所以它偏多少
     不代表真值，反而用来确认"谁在吸收模型误差"。

实测（两条记录）：
  圆锥拟合与 Fitzgibbon 中心完全相同（<0.01 LSB）=> 角度分布不均没有造成偏差
  几何圆拟合中心偏 1.9 / 2.2 LSB，但其正交距离 rms 2.52 LSB，而椭圆只有 0.2 LSB
  => 偏移来自圆模型本身，b_y = -7.703 mg 是可信的，与六面法 -6.67 mg 的差是真实系统差

从仓库根目录运行：python tools/calib/ellipse_fit_check.py [记录...]
"""
import os, sys
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from jf_load import load_jf

AX = 'xyz'
fps = 8027.0
WIN = int(0.20 * fps)
FS = 2048.0
DEF = ['serial_runtime_20260913_233731_085_export.txt',
       'serial_runtime_20260913_234829_229_export.txt']


def smooth_rate(q):
    q0, q1 = q[:-1], q[1:]
    w_, x_, y_, z_ = q0.T
    w2, x2, y2, z2 = q1.T
    V = np.stack([w_*x2-x_*w2-y_*z2+z_*y2, w_*y2+x_*z2-y_*w2-z_*x2, w_*z2-x_*y2+y_*x2-z_*w2], 1)
    cs = np.vstack([np.zeros(3), np.cumsum(V, 0)])
    k = np.arange(len(q))
    lo = np.clip(k-WIN, 0, None); hi = np.clip(k+1, None, len(V))
    mv = (cs[hi]-cs[lo])/np.maximum(hi-lo, 1)[:, None]
    return np.degrees(2.0*np.linalg.norm(mv, axis=1)*fps)


def static_pts(acc, rate, thr, minlen=0.4):
    N = len(acc); out, still, i = [], rate < thr, 0
    while i < N:
        if still[i]:
            j = i
            while j < N and still[j]:
                j += 1
            if (j-i)/fps >= minlen:
                g_ = int(0.05*fps); sl = slice(i+g_, j-g_)
                if sl.stop-sl.start > int(0.1*fps):
                    out.append(acc[sl].mean(0))
            i = j
        else:
            i += 1
    return out


def conic_fit(P):
    """代数圆锥直接最小二乘"""
    c0 = P.mean(0)
    _, _, Vt = np.linalg.svd(P-c0, full_matrices=False)
    nrm = Vt[2]
    k = int(np.argmax(np.abs(nrm)))
    ij = [i for i in range(3) if i != k]
    u, v = P[:, ij[0]], P[:, ij[1]]
    A = np.stack([u*u, u*v, v*v, u, v, np.ones_like(u)], 1)
    _, _, Vt2 = np.linalg.svd(A, full_matrices=False)
    a_, b_, c_, d_, e_, f_ = Vt2[-1]
    den = b_*b_ - 4*a_*c_
    return np.array([(2*c_*d_-b_*e_)/den, (2*a_*e_-b_*d_)/den]), ij


def fitzgibbon(u, v):
    """Fitzgibbon 直接最小二乘椭圆拟合"""
    D1 = np.stack([u*u, u*v, v*v], 1)
    D2 = np.stack([u, v, np.ones_like(u)], 1)
    S1 = D1.T @ D1; S2 = D1.T @ D2; S3 = D2.T @ D2
    T = -np.linalg.solve(S3, S2.T)
    M = S1 + S2 @ T
    C = np.array([[0, 0, 2.], [0, -1., 0], [2., 0, 0]])
    M = np.linalg.solve(C, M)
    _, vec = np.linalg.eig(M)
    vec = np.real(vec)                       # M 非对称，eig 会给复特征向量
    cond = 4*vec[0]*vec[2] - vec[1]**2
    idx = np.where(cond > 0)[0]
    if len(idx) == 0:
        return None
    a_ = np.concatenate([vec[:, idx[0]], T @ vec[:, idx[0]]])
    A, B, Cc, D, E, F = np.real(a_)
    den = B*B - 4*A*Cc
    return np.array([(2*Cc*D - B*E)/den, (2*A*E - B*D)/den])


def circle_geom(u, v):
    """几何圆拟合：Gauss-Newton 最小化 sum (|p-c| - r)^2"""
    cx, cy = u.mean(), v.mean()
    r = np.linalg.norm(np.stack([u-cx, v-cy], 1), axis=1).mean()
    for _ in range(80):
        dx, dy = u-cx, v-cy
        d = np.hypot(dx, dy)
        J = np.stack([-dx/d, -dy/d, -np.ones_like(d)], 1)
        dp, *_ = np.linalg.lstsq(J, -(d-r), rcond=None)
        cx += dp[0]; cy += dp[1]; r += dp[2]
        if np.abs(dp).max() < 1e-12:
            break
    return np.array([cx, cy]), r


def main():
    for fn in (sys.argv[1:] or DEF):
        a = load_jf(fn)
        q = np.asarray(a[:, :4], dtype=np.float64).copy()
        q /= np.linalg.norm(q, axis=1, keepdims=True)
        acc = np.asarray(a[:, 4:7], dtype=np.float64)
        rate = smooth_rate(q)
        P = np.array(static_pts(acc, rate, 1.0))
        c_con, ij = conic_fit(P)
        u, v = P[:, ij[0]], P[:, ij[1]]
        c_fitz = fitzgibbon(u, v)
        c_cir, r = circle_geom(u, v)
        ang = np.sort(np.degrees(np.arctan2(v-c_cir[1], u-c_cir[0])) % 360)
        gaps = np.diff(np.concatenate([ang, [ang[0]+360]]))
        print("\n== %s   (平面内轴 %s,%s, %d 点)"
              % (os.path.basename(fn), AX[ij[0]], AX[ij[1]], len(P)))
        print("   角度覆盖: 最大间隙 %.1f deg  最小间隙 %.1f deg  (均匀时 360/n = %.1f)"
              % (gaps.max(), gaps.min(), 360.0/len(P)))
        print("   几何圆拟合: 半径 %.2f LSB, 正交距离 rms %.3f LSB"
              % (r, np.abs(np.hypot(u-c_cir[0], v-c_cir[1]) - r).std()))
        for nm, c in (('代数圆锥(现用)', c_con), ('Fitzgibbon   ', c_fitz), ('几何圆拟合   ', c_cir)):
            if c is None:
                print("   %s: 失败" % nm); continue
            print("   %s 中心 = (%8.2f, %8.2f) LSB = (%+7.3f, %+7.3f) mg"
                  % (nm, np.real(c[0]), np.real(c[1]), np.real(c[0])/FS*1000, np.real(c[1])/FS*1000))
        d1 = np.real(c_con) - np.real(c_fitz)
        print("   圆锥 vs Fitzgibbon 差 = (%.4f, %.4f) LSB  => 角度分布不均没有造成偏差"
              % (d1[0], d1[1]))
        d2 = np.real(c_con) - np.real(c_cir)
        print("   圆锥 vs 几何圆   差 = (%.2f, %.2f) LSB  => 圆是更弱的模型，偏移算在它头上"
              % (d2[0], d2[1]))


if __name__ == '__main__':
    main()
