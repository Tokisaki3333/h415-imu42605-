# -*- coding: utf-8 -*-
r"""磁标定：**椭球拟合（不需要姿态参考）+ 重力对齐**，只依赖重力与 WMM 倾角。

为什么不用旧链 att.q 当参考：它是无磁陀螺链，几秒到几十秒就会累积几十度偏航漂移
（本录像 89 s 内 yaw 累计 1015°、陀螺零偏 0.36 dps ⇒ ~30° 漂移），
拿它当真值会把标定精度锁在 ~4°（实测 p50 4.12°/p90 9.35°，而同一标定在它拟合的台面数据上是 0.48°）。

两步：
  1) **椭球**：|A·raw + C| = const（线性最小二乘，10 参数 -> 9 未知），不需要任何姿态参考，
     定出软磁/标度/硬铁（即 A^T A 与 C）。
  2) **取向**：椭球只给出 A0 = chol(A^T A)，还差一个 SO(3) 自由度 U（A = U·A0）。
     用**重力**钉住它：要求标定后磁场与实测重力的夹角 = 90° + WMM 倾角 I
     （准静止帧，多姿态 => 超定，U 唯一）。这一步只用重力，不用 WMM 的偏航/方位。

判据（才是真正说明"模型对不对"的）：
  * 对齐后**倾角的姿态一致性**（p10~p90 展宽）：椭球+取向对了就该 <1°；
  * 原始 |B|（0.3 µT/LSB）与 WMM F 的差；
  * 与现用 A/C 在同一数据上的世界系一致性对比（次要，受参考精度限制）。

用法:
  python tools/calib/mag_ell_cal.py R:\imu_20260921_040450.bin
  python tools/calib/mag_ell_cal.py <bin> --lat 36.23 --lon 120.44 --wmax 1000 --ws 20
"""
import argparse
import datetime
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
if HERE not in sys.path:
    sys.path.insert(0, HERE)
import cols_162 as C          # noqa: E402
import mag360_cal as M        # noqa: E402

try:
    sys.stdout.reconfigure(encoding='utf-8', errors='replace')
except Exception:
    pass

c = C.CH_162
LSB2UT = 0.3


def wmm_i(lat, lon, year):
    from pygeomag import GeoMag
    r = GeoMag().calculate(glat=lat, glon=lon, alt=0.0, time=year)
    return r.d, r.i, r.h, r.f


def ellipsoid_fit(u):
    """|A u + C| = const 的线性最小二乘。返回 A(上三角, det>0)、C，标度未定（|y|≈1）。

    先中心化+归一化再拟合（加速度计原始 LSB 可达 ±32768 且球心远离原点，
    直接用 "u^T M u + b.u = 1" 参数化会病态到 NaN）。
    """
    mu = u.mean(0)
    su = u.std(0) + 1e-12
    v = (u - mu) / su
    x, y, z = v[:, 0], v[:, 1], v[:, 2]
    D = np.stack([x * x, y * y, z * z, 2 * x * y, 2 * x * z, 2 * y * z, x, y, z], 1)
    sol, *_ = np.linalg.lstsq(D, np.ones(len(v)), rcond=None)
    Mx = np.array([[sol[0], sol[3], sol[4]],
                   [sol[3], sol[1], sol[5]],
                   [sol[4], sol[5], sol[2]]])
    b = sol[6:9]
    Ci = -0.5 * np.linalg.solve(Mx, b)
    R = np.sqrt(max(1.0 + Ci @ Mx @ Ci, 1e-30))
    try:
        Av = np.linalg.cholesky(Mx / R ** 2).T        # Av^T Av = Mx/R^2, det>0
    except np.linalg.LinAlgError:
        w, V = np.linalg.eigh(Mx)
        Av = np.diag(np.sqrt(np.maximum(w, 1e-12))) @ V.T
    Cv = Ci / R
    # 回到原尺度：y = Av·((u-mu)/su) + Cv  =>  A = Av/su（按列）, C = Cv - A·mu
    A = Av / su
    return A, Cv - A @ mu


def dip_angle(y, gh):
    yn = y / np.maximum(np.linalg.norm(y, axis=1, keepdims=True), 1e-12)
    return np.degrees(np.arccos(np.clip(np.einsum('ni,ni->n', yn, gh), -1, 1)))


def rotvec_to_R(v):
    th = np.linalg.norm(v)
    if th < 1e-15:
        return np.eye(3)
    k = v / th
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + np.sin(th) * K + (1 - np.cos(th)) * (K @ K)


def align_by_gravity(A0, C0, u, gh, target_dip, ntry=60, rng=None):
    """求 U 使 ∠(U(A0 u + C0), g) = 90 + target_dip。多起点 Gauss-Newton。"""
    rng = rng or np.random.default_rng(0)
    best = (1e9, np.eye(3))

    def cost(U):
        y = (U @ (A0 @ u.T + C0[:, None])).T
        return dip_angle(y, gh) - (90.0 + target_dip)

    for k in range(ntry):
        U = np.eye(3) if k == 0 else rotvec_to_R(rng.normal(0, 1.2, 3))
        for _ in range(40):
            f = cost(U)
            rmse = float(np.sqrt(np.mean(f ** 2)))
            if rmse < best[0]:
                best = (rmse, U.copy())
            J = np.zeros((len(u), 3))
            for j in range(3):
                d = np.zeros(3); d[j] = 1e-6
                J[:, j] = (cost(rotvec_to_R(d) @ U) - f) / 1e-6
            try:
                dx = -np.linalg.solve(J.T @ J + 1e-9 * np.eye(3), J.T @ f)
            except np.linalg.LinAlgError:
                break
            U = rotvec_to_R(dx) @ U
            if np.linalg.norm(dx) < 1e-9:
                break
        f = cost(U)
        r = float(np.sqrt(np.mean(f ** 2)))
        if r < best[0]:
            best = (r, U.copy())
    return best[1], best[0]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('log')
    ap.add_argument('--lat', type=float, default=36.23)
    ap.add_argument('--lon', type=float, default=120.44)
    ap.add_argument('--year', type=float, default=None)
    ap.add_argument('--wmax', type=float, default=1000.0, help='椭球拟合用的 |w| 上限 dps')
    ap.add_argument('--ws', type=float, default=20.0, help='重力对齐用的准静止 |w| 上限 dps')
    ap.add_argument('--amag', type=float, default=0.05, help='重力对齐用的 ||a|-1| 上限 g')
    a = ap.parse_args()
    d0 = datetime.date.today()
    y = a.year if a.year is not None else d0.year + (d0.timetuple().tm_yday - 1) / 365.25
    D, I, H, F = wmm_i(a.lat, a.lon, y)
    print('WMM2025 %.2fN %.2fE 历元 %.2f: D=%+.3f° I=%+.3f° H=%.0f F=%.0f nT' % (a.lat, a.lon, y, D, I, H, F))
    print('固件现值: D=-7.53°  tanI=1.70 -> I=%.2f°（与真值差 %+.2f°）' % (np.degrees(np.arctan(1.70)),
                                                                np.degrees(np.arctan(1.70)) - I))

    C.selfcheck(verbose=False)
    fr, _ = C.load_frames(a.log)
    rep = C.frame_report(fr)
    print('\n录像 %s VER=%s 帧 %d 校验和 %.1f%%' % (os.path.basename(a.log), rep.get('ver'),
                                                  rep.get('n', len(fr)), 100.0 * rep.get('checksum_ok', 0)))
    raw = fr[:, c['mag_lsb0']:c['mag_lsb0'] + 3].astype(float)
    g = fr[:, c['accel_g0']:c['accel_g0'] + 3].astype(float)
    gn = np.linalg.norm(g, axis=1)
    gh_all = g / np.maximum(gn[:, None], 1e-9)
    w = np.linalg.norm(fr[:, c['gyro_dps0']:c['gyro_dps0'] + 3].astype(float), axis=1)

    m_ell = w < a.wmax
    m_st = (w < a.ws) & (np.abs(gn - 1.0) < a.amag)
    print('椭球用帧 %d，重力对齐用准静止帧 %d' % (m_ell.sum(), m_st.sum()))
    A0, C0 = ellipsoid_fit(raw[m_ell])
    print('椭球形状: A0^T A0 特征值 %s（各向异性 %.1f%%）'
          % (np.round(np.linalg.eigvalsh(A0.T @ A0), 6),
             100 * (np.sqrt(np.linalg.eigvalsh(A0.T @ A0).max() / np.linalg.eigvalsh(A0.T @ A0).min()) - 1)))

    Ub, rmse = align_by_gravity(A0, C0, raw[m_st], gh_all[m_st], I)
    A, Cv = Ub @ A0, Ub @ C0
    print('重力对齐: 多起点解残差 RMS %.3f°（目标倾角 %.3f°）' % (rmse, I))

    # 标度：与固件一致 median|y| = 1（倾角与标度无关）
    yy = (A @ raw.T + Cv[:, None]).T
    s = np.median(np.linalg.norm(yy, axis=1))
    A, Cv = A / s, Cv / s
    yy = (A @ raw.T + Cv[:, None]).T

    def dip_of(Ax, Cx):
        yv = (Ax @ raw.T + Cx[:, None]).T
        return dip_angle(yv, gh_all) - 90.0

    cur_A, cur_C = M.read_current_AC()
    for tag, Ax, Cx in (('现用 A/C', cur_A, cur_C), ('原始 LSB', np.diag([1.0, 1.0, 1.0]), np.zeros(3)),
                        ('本次(椭球+重力)', A, Cv)):
        d = dip_of(Ax, Cx)[m_st]
        p = np.percentile(d, [10, 50, 90])
        print('  %-16s 倾角 p10/p50/p90 = %6.2f /%6.2f /%6.2f   ΔI(p50) %+6.2f°   展宽 %.2f°'
              % (tag, p[0], p[1], p[2], p[1] - I, p[2] - p[0]))
    Bm = np.linalg.norm(raw[m_st], axis=1) * LSB2UT
    print('  原始 |B| p50 = %.1f µT  vs WMM %.1f µT（%+.1f%%）'
          % (np.median(Bm), F / 1000.0, 100 * (np.median(Bm) / (F / 1000.0) - 1)))

    # 世界系一致性（用旧链姿态，仅作参考：受参考漂移限制）
    q = fr[:, c['att_q0']:c['att_q0'] + 4].astype(float)
    ok = np.linalg.norm(q, axis=1) > 0.5
    R = np.stack([M.quat_to_R(r) for r in q])
    for tag, Ax, Cx in (('现用 A/C', cur_A, cur_C), ('本次(椭球+重力)', A, Cv)):
        v = np.einsum('nij,nj->ni', R, (Ax @ raw.T + Cx[:, None]).T)
        v /= np.maximum(np.linalg.norm(v, axis=1, keepdims=True), 1e-12)
        vm = v[ok].mean(0); vm /= np.linalg.norm(vm)
        ang = np.degrees(np.arccos(np.clip(v[ok] @ vm, -1, 1)))
        print('  世界系方向残差(att.q 参考，受参考限制) %-16s p50 %.2f p90 %.2f' %
              (tag, *np.percentile(ang, [50, 90])))

    print('\n写回固件：')
    for i, nm in enumerate(('A 行0', 'A 行1', 'A 行2')):
        print('  %s { %+.8ef, %+.8ef, %+.8ef }' % (nm, A[i, 0], A[i, 1], A[i, 2]))
    print('  C    { %+.8ef, %+.8ef, %+.8ef }' % (Cv[0], Cv[1], Cv[2]))
    return 0


if __name__ == '__main__':
    sys.exit(main())
