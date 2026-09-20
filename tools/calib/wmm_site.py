# -*- coding: utf-8 -*-
r"""WMM2025 站点/极点实算（不联网也能复现）：磁偏角 D、磁倾角 I、H、F，以及磁北极位置。

用途：
  1) 核对固件常量 V5F_MAG_DECL_DEG / V5F_EKF_DIP_TAN 是否对得上你的点位；
  2) 反查：给定 (D, I) 找出中国境内最接近的经纬度（判"常量到底对应哪个城市"）；
  3) 实算磁北极（dip pole, H→0）当前位置。

依赖: pip install pygeomag      （自带 WMM_2025.COF / WMMHR_2025.COF）

用法:
  python tools/calib/wmm_site.py                          # 默认 40.00N 116.25E（北京）
  python tools/calib/wmm_site.py --lat 36.23 --lon 120.44 # 青岛
  python tools/calib/wmm_site.py --match -7.53 59.53      # 反查常量对应的点位
  python tools/calib/wmm_site.py --pole                   # 只算磁北极
  python tools/calib/wmm_site.py --year 2026.72           # 历元（默认按当天）
"""
import argparse
import datetime
import sys

import numpy as np
from pygeomag import GeoMag

GM = GeoMag()


def field(lat, lon, year, alt=0.0):
    r = GM.calculate(glat=lat, glon=lon, alt=alt, time=year)
    return r.d, r.i, r.h, r.f


def year_now():
    d = datetime.date.today()
    return d.year + (d.timetuple().tm_yday - 1) / 365.25


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--lat', type=float, default=40.00)
    ap.add_argument('--lon', type=float, default=116.25)
    ap.add_argument('--alt', type=float, default=0.0, help='海拔 km（WMM 用 km）')
    ap.add_argument('--year', type=float, default=None)
    ap.add_argument('--match', nargs=2, type=float, default=None, metavar=('D', 'I'),
                    help='反查：给定磁偏角/磁倾角，找中国境内最接近的点')
    ap.add_argument('--pole', action='store_true', help='只算磁北极（dip pole）')
    a = ap.parse_args()
    y = a.year if a.year is not None else year_now()

    if not a.pole:
        d, i, h, f = field(a.lat, a.lon, y, a.alt)
        print('点位 %.3fN %.3fE  alt %.1f km  历元 %.2f' % (a.lat, a.lon, a.alt, y))
        print('  D  磁偏角 = %+8.3f°   (东正西负)' % d)
        print('  I  磁倾角 = %+8.3f°   tan I = %.4f' % (i, np.tan(np.radians(i))))
        print('  H  水平   = %8.1f nT' % h)
        print('  F  总强度 = %8.1f nT' % f)
        d2, i2, _, _ = field(a.lat, a.lon, y + 1.0, a.alt)
        print('  一年后(%.2f): D=%+.3f (Δ%+.3f°/yr)  I=%+.3f (Δ%+.3f°/yr)'
              % (y + 1, d2, d2 - d, i2, i2 - i))
        print('  固件常量: V5F_MAG_DECL_DEG=-7.53°(tan I=1.70 -> I=%.2f°)  '
              '-> D 差 %+.3f°, I 差 %+.3f°'
              % (np.degrees(np.arctan(1.70)), -7.53 - d, np.degrees(np.arctan(1.70)) - i))

    if a.match:
        td, ti = a.match
        best = []
        for la in np.arange(30.0, 50.01, 0.25):
            for lo in np.arange(100.0, 135.01, 0.25):
                d, i, _, _ = field(la, lo, y)
                best.append(((d - td) ** 2 * 4.0 + (i - ti) ** 2, la, lo, d, i))
        best.sort()
        print('\n反查 (D=%.3f°, I=%.3f°) 最接近的 5 点:' % (td, ti))
        for e, la, lo, d, i in best[:5]:
            print('  %.2fN %.2fE   D=%+7.3f° (Δ%+.2f)  I=%6.3f° (Δ%+.2f)  tanI=%.3f'
                  % (la, lo, d, d - td, i, i - ti, np.tan(np.radians(i))))

    # 磁北极（dip pole）：H 最小处
    bb = None
    for la in np.arange(80.0, 90.001, 0.05):
        for lo in np.arange(-180.0, 180.0, 0.5):
            d, i, h, f = field(la, lo, y)
            if bb is None or h < bb[0]:
                bb = (h, la, lo, i, f)
    print('\n磁北极(dip pole, H→0) 历元 %.2f: %.2fN %.2fE   H=%.1f nT  I=%.3f°  F=%.0f nT'
          % (y, bb[1], bb[2], bb[0], bb[3], bb[4]))
    return 0


if __name__ == '__main__':
    sys.exit(main())
