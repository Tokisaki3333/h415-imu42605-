# -*- coding: utf-8 -*-
"""分离"旧链参考误差"与"地磁侧误差"。

思路（不需要第二个角度基准）：
  地磁标定好之后，世界系磁场 u_i = R(q_i)·(A·m_i + C) 应当恒定。任何散布 δu_i 只能来自
  参考姿态误差或地磁误差。而**陀螺标度误差 ε 的作用是确定的**：它让参考姿态在 t 时刻
  相对起点多转 ε·θ_vec(t)（θ_vec = 参考姿态相对起点的旋转矢量），于是
        δu_i ≈ ε · θ_vec_i        （世界系）
  所以：
    * 用最小二乘估 ε̂（ppm）与回归 R²：**这是用数据自身测出来的参考标度误差**；
    * 300 ppm 假设下的残差贡献 = 3e-4 · rms(|θ_vec|)；
    * 剔除该成分后剩下的 rms = 地磁/环境侧误差。

用法: python tools/calib/mag360_refcheck.py <bin 或 hex 文本> [--wmax 50] [--ppm 300]
"""
import argparse
import os
import sys

import numpy as np

try:                      # GBK 控制台编不出组合符(m-hat)等字符
    sys.stdout.reconfigure(encoding='utf-8', errors='replace')
except Exception:
    pass

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import cols_162 as C      # noqa: E402
import mag360_cal as M    # noqa: E402


def log_rotvec(q):
    """q=(w,x,y,z) -> 旋转矢量(rad)。"""
    w = np.clip(q[..., 0], -1.0, 1.0)
    v = q[..., 1:4]
    n = np.linalg.norm(v, axis=-1)
    ang = 2.0 * np.arctan2(n, w)
    k = np.where(n > 1e-12, ang / np.maximum(n, 1e-12), 2.0)
    return v * k[..., None]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('log')
    ap.add_argument('--wmax', type=float, default=50.0)
    ap.add_argument('--ppm', type=float, default=300.0, help='假定的旧链标度误差')
    ap.add_argument('--amag', type=float, default=0.02)
    a = ap.parse_args()

    C.selfcheck(verbose=False)
    arr, info = C.load_frames(a.log)
    rep = C.frame_report(arr)
    A, Cv = M.read_current_AC()
    print('数据 %s  帧 %d  fw_tag %.0f (VER=%d)  校验和 %.0f%%'
          % (os.path.basename(a.log), rep['frames'], rep['fw_tag'], rep['ver'], 100 * rep['checksum_ok']))

    sel, q, gyro, w, am, m = M.select(arr, a.wmax, a.amag, False)
    idx = np.flatnonzero(sel)
    c = C.CH_162
    dt = arr[:, c['dt_us']].astype(np.float64) * 1e-6

    # 世界系磁场（用固件当前 A/C）
    u = np.empty((len(idx), 3))
    for t, i in enumerate(idx):
        v = A @ m[i] + Cv
        u[t] = M.quat_to_R(q[i]) @ v
    u /= np.linalg.norm(u, axis=1, keepdims=True)
    ubar = u.mean(0)
    ubar /= np.linalg.norm(ubar)

    # δu_i：把 u_i 旋到 ū 的小旋转矢量
    dv = np.cross(u, ubar)
    dn = np.linalg.norm(dv, axis=1)
    du = dv * np.where(dn > 1e-12, np.arcsin(np.clip(dn, -1, 1)) / np.maximum(dn, 1e-12), 1.0)[:, None]

    # θ_vec_i：参考姿态相对起点的旋转矢量（世界系）
    q0 = q[idx[0]]
    th = np.empty((len(idx), 3))
    for t, i in enumerate(idx):
        qi = q[i]
        w0_, x0, y0, z0 = q0
        # q_rel = q_i ⊗ q_0^{-1}
        wr = qi[0] * w0_ + qi[1] * x0 + qi[2] * y0 + qi[3] * z0
        xr = -qi[0] * x0 + qi[1] * w0_ - qi[2] * z0 + qi[3] * y0
        yr = -qi[0] * y0 + qi[1] * z0 + qi[2] * w0_ - qi[3] * x0
        zr = -qi[0] * z0 - qi[1] * y0 + qi[2] * x0 + qi[3] * w0_
        th[t] = log_rotvec(np.array([wr, xr, yr, zr]))
    thn = np.linalg.norm(th, axis=1)

    # 标量回归 δu ≈ eps·θ  =>  eps = Σ(du·θ̂)|θ| / Σ|θ|²  （投影到单位阵）
    proj = np.einsum('ij,ij->i', du, th) / np.maximum(thn ** 2, 1e-12)
    eps = float(np.sum(proj * thn ** 2) / np.sum(thn ** 2))
    pred = eps * th
    r2 = 1.0 - np.sum((du - pred) ** 2) / np.sum(du ** 2)
    resid = du - pred

    deg = 180.0 / np.pi
    print()
    print('参考姿态在该段里的行程: |θ_vec| max %.1f deg, rms %.1f deg' % (thn.max() * deg, np.sqrt((thn ** 2).mean()) * deg))
    print('残差 δu: rms %.3f deg, p90 %.3f deg' % (np.sqrt((np.linalg.norm(du, axis=1) ** 2).mean()) * deg,
                                                   np.percentile(np.linalg.norm(du, axis=1), 90) * deg))
    print()
    print('== 用数据测出的参考标度误差 ==')
    print('  ε̂ = %+.1f ppm   (回归 R² = %.3f)' % (eps * 1e6, r2))
    print('  按假设 %+.0f ppm 预测的残差贡献 rms = %.4f deg' % (a.ppm, a.ppm * 1e-6 * np.sqrt((thn ** 2).mean()) * deg))
    print('  剔除该成分后剩余 rms = %.3f deg  ⇒ 归地磁/环境' % (np.sqrt((np.linalg.norm(resid, axis=1) ** 2).mean()) * deg))
    print()
    print('== 对拟合参数的影响（解析界）==')
    print('  标度误差 ε 等价于把参考旋转角整体缩放，A 只能被 ε 量级扰动:')
    print('    |ΔA|/|A| ≈ ε = %.1e  (300ppm = 3.0e-4)，而实测两段 A 的复现性 ≈ 1.8e-2' % max(abs(eps), a.ppm * 1e-6))
    print('    零漂 2 deg/h × 90 s = %.3f deg（近似常数，被 w0 方向的规范自由度吸收）' % (2.0 / 3600.0 * 90.0))


if __name__ == '__main__':
    main()
