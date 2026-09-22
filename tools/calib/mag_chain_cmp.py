# -*- coding: utf-8 -*-
"""分清哪条链在跟磁：直算磁航向 / att.q / ekf_q 三者的机头航向误差对比 + 磁修正量。
用法: python _who6.py <cap.bin> <anchor_deg>
"""
import os
import sys

import numpy as np

sys.path.insert(0, r'C:\Users\33\Documents\v2\h415-imu42605-\tools\calib')
import cols_162 as C          # noqa: E402
import mag360_cal as M        # noqa: E402

TUNE = r'C:\Users\33\Documents\v2\h415-imu42605-\V5F\User\inc\v5f_tune.h'
c = C.CH_162
D_TRUE = -7.639
DT = 0.002989


def wrap(a):
    return (a + 180.0) % 360.0 - 180.0


def Rq(q):
    w, x, y, z = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


cap = sys.argv[1]
anchor = float(sys.argv[2])
A, Cc = M.read_current_AC(TUNE)
fr, _ = C.load_frames(cap)
g = np.asarray(fr[:, c['accel_g0']:c['accel_g0'] + 3], float)
gn = np.linalg.norm(g, axis=1)
gy = np.asarray(fr[:, c['gyro_dps0']:c['gyro_dps0'] + 3], float)
raw = np.asarray(fr[:, c['mag_lsb0']:c['mag_lsb0'] + 3], float)
qa = np.asarray(fr[:, 0:4], float)
qe = np.asarray(fr[:, c['ekf_q0']:c['ekf_q0'] + 4], float)
print('%s VER=%s 锚点%.0f  det(A)=%.3e' % (os.path.basename(cap), C.frame_report(fr).get('ver'),
                                           anchor, np.linalg.det(A)))
st = (np.linalg.norm(np.radians(gy), axis=1) < np.radians(3.0)) & (np.abs(gn - 1.0) < 0.03)
segs, i = [], 0
while i < len(st):
    if st[i]:
        j = i
        while j + 1 < len(st) and st[j + 1]:
            j += 1
        if (j - i + 1) * DT >= 5.0:
            segs.append((i, j))
        i = j + 1
    else:
        i += 1
mid = [(a + b) // 2 for a, b in segs]
def yaw_of(qq):
    w, x, y, z = qq / max(np.linalg.norm(qq), 1e-9)
    return np.degrees(np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)))


psi = np.array([yaw_of(qe[m]) for m in mid])
hdg = [anchor]
for k in range(len(segs) - 1):
    hdg.append(wrap(hdg[-1] - 90.0 * round(wrap(psi[k + 1] - psi[k]) / 90.0)))
hdg = np.array(hdg)
print('已知航向 %s' % np.round(hdg, 0))

axes = {'+x': np.array([1.0, 0, 0]), '-x': np.array([-1.0, 0, 0]),
        '+y': np.array([0, 1.0, 0]), '-y': np.array([0, -1.0, 0])}
print('\n 段 |  直算磁航向(用A/C)        |  att.q 航向               |  ekf_q 航向              | dqz均值  nis4  rej')
for k, (a, b) in enumerate(segs):
    mm = raw[a:b + 1].mean(0) @ A.T + Cc
    mm = mm / np.linalg.norm(mm)
    gg = g[mid[k]] / gn[mid[k]]
    u = gg   # 加速度静止=比力, +g_hat 即世界 up(见 dip 列用 +g 得 +54)
    mh = mm - (mm @ u) * u
    mh = mh / np.linalg.norm(mh)
    out = []
    for tag, qq in (('direct', None), ('att.q', qa[mid[k]]), ('ekf.q', qe[mid[k]])):
        if qq is None:
            vals = []
            for ax, av in axes.items():
                ah = av - (av @ u) * u
                ah = ah / np.linalg.norm(ah)
                th = np.degrees(np.arctan2(u @ np.cross(mh, ah), mh @ ah))
                vals.append('%+7.1f' % wrap((D_TRUE - th) - hdg[k]))
            out.append('/'.join(vals))
        else:
            qq = qq / max(np.linalg.norm(qq), 1e-9)
            R = Rq(qq)
            vals = []
            for ax, av in axes.items():
                v = R @ av
                hs = np.degrees(np.arctan2(v[0], v[1]))
                vals.append('%+7.1f' % wrap(hs - hdg[k]))
            out.append('/'.join(vals))
    dqz = np.asarray(fr[a:b + 1, c['ekf_mag_dqz']], float).mean()
    nis4 = np.asarray(fr[a:b + 1, c['ekf_nis4']], float)
    rej = np.asarray(fr[a:b + 1, c['ekf_mag_rej']], float)
    print('  %d | %-22s | %-22s | %-22s | %+7.4f %6.2f %6.0f'
          % (k + 1, out[0], out[1], out[2], dqz, nis4.mean(), rej.sum()))
print(' (每格 = +x/-x/+y/-y 的航向误差 deg)')
