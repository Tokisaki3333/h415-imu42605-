# -*- coding: utf-8 -*-
r"""**绝对项**验收：把实测的磁倾角 / 场强与 WMM2025 比（不需要指北、不需要转台，静止即可）。

为什么必须有这一条：项目里其它所有指标（世界系磁场方向残差、磁创新 th、nis4、dq、回原位复现）
都是**内部一致性**指标 —— 一个恒定的测量畸变（软磁/轴向不对齐/环境）会被姿态原样吸收，
它们**天生看不见**（见 docs/mag_observability_math.md §7.10）。

原理：磁倾角 I = angle(m̂, ĝ) − 90° 是**姿态无关量**（m̂ = 标定后磁场机体系单位矢量，ĝ = 加速度单位矢量），
所以只要器件静止、|a|≈1 g，就能直接和 WMM 的 I 比。

用法:
  python tools/calib/mag_site_check.py R:\imu_20260921_040450.bin
  python tools/calib/mag_site_check.py <bin> --lat 36.23 --lon 120.44 --year 2026.72
  python tools/calib/mag_site_check.py <bin> --tol-i 0.5 --tol-f 2.0      # 判据
"""
import argparse
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
if HERE not in sys.path:
    sys.path.insert(0, HERE)
import cols_162 as C
import mag360_cal as M

c = C.CH_162
LSB2UT = 0.3          # IST8310 灵敏度 0.3 uT/LSB（±1200 uT 档）


def wmm(lat, lon, year, alt):
    from pygeomag import GeoMag
    r = GeoMag().calculate(glat=lat, glon=lon, alt=alt, time=year)
    return r.d, r.i, r.h, r.f


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('log')
    ap.add_argument('--lat', type=float, default=36.23)
    ap.add_argument('--lon', type=float, default=120.44)
    ap.add_argument('--year', type=float, default=None)
    ap.add_argument('--alt', type=float, default=0.0, help='海拔 km')
    ap.add_argument('--wmove', type=float, default=5.0, help='静止判据 |w| 上限 dps')
    ap.add_argument('--amag', type=float, default=0.03, help='静止判据 ||a|-1g| 上限 g')
    ap.add_argument('--tol-i', type=float, default=0.5, help='倾角容差 deg')
    ap.add_argument('--tol-f', type=float, default=2.0, help='场强容差 %%')
    a = ap.parse_args()

    import datetime
    y = a.year if a.year is not None else (datetime.date.today().year
                                           + (datetime.date.today().timetuple().tm_yday - 1) / 365.25)
    D, I, H, F = wmm(a.lat, a.lon, y, a.alt)
    print('WMM2025 点位 %.2fN %.2fE alt %.1f km 历元 %.2f:  D=%+.3f°  I=%+.3f°  H=%.0f nT  F=%.0f nT (%.1f uT)'
          % (a.lat, a.lon, a.alt, y, D, I, H, F, F / 1000.0))
    print('  固件常量: D=%+.2f°  tanI=1.70 -> I=%+.2f°   =>  D 差 %+.2f°,  I 差 %+.2f°'
          % (-7.53, np.degrees(np.arctan(1.70)), -7.53 - D, np.degrees(np.arctan(1.70)) - I))

    C.selfcheck(verbose=False)
    fr, _ = C.load_frames(a.log)
    rep = C.frame_report(fr)
    print('\n录像 %s  VER=%s  帧 %d  校验和 %.1f%%'
          % (os.path.basename(a.log), rep.get('ver'), rep.get('n', len(fr)),
             100.0 * rep.get('checksum_ok', 0)))
    A, Cv = M.read_current_AC()
    dt = fr[:, c['dt_us']].astype(float) * 1e-6
    t = np.cumsum(dt) - dt[0]
    w = np.linalg.norm(fr[:, c['gyro_dps0']:c['gyro_dps0'] + 3].astype(float), axis=1)
    raw = fr[:, c['mag_lsb0']:c['mag_lsb0'] + 3].astype(float)
    cal = (A @ raw.T + Cv[:, None]).T
    g = fr[:, c['accel_g0']:c['accel_g0'] + 3].astype(float)
    gn = np.linalg.norm(g, axis=1)
    gh = g / np.maximum(gn[:, None], 1e-9)
    st = (w < a.wmove) & (np.abs(gn - 1.0) < a.amag)
    qs = (w < 60.0) & (np.abs(gn - 1.0) < a.amag)
    print('  静止帧 %d / %d（|w|<%.0f dps 且 ||a|-1|<%.2f g）；宽静置 %d 帧'
          % (st.sum(), len(t), a.wmove, a.amag, qs.sum()))
    if st.sum() < 50:
        print('  !! 静止帧太少，判据不可靠')
        return 2

    print('\n  量                       实测 p10 / p50 / p90                    WMM          差(p50)')
    out = {}
    for tag, v, ref in (('原始 LSB 倾角 [deg]', raw, I), ('标定后倾角 [deg]', cal, I)):
        vn = v / np.linalg.norm(v, axis=1, keepdims=True)
        dip = np.degrees(np.arccos(np.clip(np.einsum('ni,ni->n', vn, gh), -1, 1))) - 90.0
        d = dip[st]
        p = np.percentile(d, [10, 50, 90])
        print('  %-22s %7.2f /%7.2f /%7.2f %21.3f %+9.2f'
              % (tag, p[0], p[1], p[2], ref, p[1] - ref))
        out[tag] = p[1]
    Bm = np.linalg.norm(raw[st], axis=1) * LSB2UT
    pB = np.percentile(Bm, [10, 50, 90])
    print('  %-22s %7.1f /%7.1f /%7.1f uT %18.1f uT %+8.1f%%'
          % ('原始 |B| [uT]', pB[0], pB[1], pB[2], F / 1000.0, 100 * (pB[1] / (F / 1000.0) - 1)))
    hv = np.linalg.norm(raw[st][:, :2], axis=1)
    vv = raw[st][:, 2]
    print('  原始 V/H = %.3f（p50）                              真值 tanI = %.3f'
          % (np.median(np.abs(vv)) / np.median(hv), np.tan(np.radians(I))))

    dI = out['原始 LSB 倾角 [deg]'] - I
    dF = 100 * (pB[1] / (F / 1000.0) - 1)
    ok = (abs(dI) <= a.tol_i) and (abs(dF) <= a.tol_f)
    print('\n判据: |ΔI| ≤ %.2f°（现 %+.2f°）   |Δ|B|| ≤ %.1f%%（现 %+.1f%%）  -> %s'
          % (a.tol_i, dI, a.tol_f, dF, 'PASS' if ok else '**FAIL**'))
    if not ok:
        print('  => 测量链在场方向上被拧歪了：先按 §7.10 的顺序查（干净场地三维翻滚标定 -> 仍不对则查磁/加速度计轴向）')
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
