# -*- coding: utf-8 -*-
"""独立定责：完全不用 EKF 四元数，只用 (m_cal, 加速度) + 已知正方向，算每个候选机头轴的罗盘航向。
  H(a) = D_true - atan2( u.(m_h x a_h), m_h.a_h ),  u = -g_hat (世界up在体系), D_true=-7.639
  段序(已知)：锚点 + 由【陀螺积分】得到的物理转向(惯性, 免疫镜像/软磁)
用法: python _who5.py <cap.bin> <anchor_deg> <tune1.h> [tune2.h ...]
"""
import os
import re
import sys

import numpy as np

sys.path.insert(0, r'C:\Users\33\Documents\v2\h415-imu42605-\tools\calib')
import cols_162 as C          # noqa: E402
import mag360_cal as M        # noqa: E402

c = C.CH_162
D_TRUE = -7.639
DT = 0.002989


def wrap(a):
    return (a + 180.0) % 360.0 - 180.0


def ver_of(path):
    try:
        t = open(path, 'rb').read().decode('gbk', errors='replace')
        m = re.search(r'#define\s+V5F_FW_VER\s+(\d+)u', t)
        return m.group(1) if m else '?'
    except Exception:
        return '?'


cap = sys.argv[1]
anchor = float(sys.argv[2])
tunes = sys.argv[3:]
fr, _ = C.load_frames(cap)
g = np.asarray(fr[:, c['accel_g0']:c['accel_g0'] + 3], float)
gn = np.linalg.norm(g, axis=1)
gy = np.asarray(fr[:, c['gyro_dps0']:c['gyro_dps0'] + 3], float)
gyn = np.linalg.norm(np.radians(gy), axis=1)
raw = np.asarray(fr[:, c['mag_lsb0']:c['mag_lsb0'] + 3], float)
q = np.asarray(fr[:, c['ekf_q0']:c['ekf_q0'] + 4], float)
print('%s VER=%s 帧%d  锚点=%.0f deg' % (os.path.basename(cap), C.frame_report(fr).get('ver'),
                                        len(fr), anchor))

st = (gyn < np.radians(3.0)) & (np.abs(gn - 1.0) < 0.03)
segs, i = [], 0
while i < len(st):
    if st[i]:
        j = i
        while j + 1 < len(st) and st[j + 1]:
            j += 1
        if (j - i + 1) * DT >= 2.0:
            segs.append((i, j))
        i = j + 1
    else:
        i += 1
print('静止段: %s' % [(round(a * DT, 1), round((b - a + 1) * DT, 1)) for a, b in segs])

mid = [(a + b) // 2 for a, b in segs]
psi, gyroz = [], []
for k, (a, b) in enumerate(segs):
    qq = q[a:b + 1].mean(0)
    qq = qq / np.linalg.norm(qq)
    psi.append(np.degrees(np.arctan2(2 * (qq[0] * qq[3] + qq[1] * qq[2]),
                                     1 - 2 * (qq[2] ** 2 + qq[3] ** 2))))
for k in range(len(segs) - 1):
    gyroz.append(float(np.sum(gy[mid[k]:mid[k + 1] + 1, 2]) * DT))
psi = np.array(psi)
gyroz = np.array(gyroz)
dpsi = np.array([wrap(psi[k + 1] - psi[k]) for k in range(len(psi) - 1)])
print('\n EKF yaw       %s' % np.round(psi, 2))
print(' EKF yaw 步进  %s' % np.round(dpsi, 2))
print(' 陀螺z积分步进 %s  (与上面同号则体系z=上, 符号可信)' % np.round(gyroz, 2))
snap = [90.0 * round(s / 90.0) for s in dpsi]
hdg = [anchor]
for s in snap:
    hdg.append(wrap(hdg[-1] - s))
hdg = np.array(hdg)
print(' 已知航向(锚点+惯性转向, 取整90) %s' % np.round(hdg, 0))

axes = {'+x': np.array([1.0, 0, 0]), '-x': np.array([-1.0, 0, 0]),
        '+y': np.array([0, 1.0, 0]), '-y': np.array([0, -1.0, 0])}

for tn in tunes:
    A, Cc = M.read_current_AC(tn)
    if A is None:
        print('\n!! %s 无 A/C' % tn)
        continue
    print('\n===== A/C 源: %s (VER=%s)  det=%.3e' % (os.path.basename(tn), ver_of(tn),
                                                    np.linalg.det(A)))
    print('  段  dip-90  ' + '  '.join('%14s' % ('%s H / err' % ax) for ax in axes))
    for k, (a, b) in enumerate(segs):
        mm = raw[a:b + 1].mean(0) @ A.T + Cc
        mm = mm / np.linalg.norm(mm)
        gg = g[mid[k]] / gn[mid[k]]
        u = gg   # 加速度静止=比力, +g_hat 即世界 up(见 dip 列用 +g 得 +54)
        mh = mm - (mm @ u) * u
        mh = mh / np.linalg.norm(mh)
        row = []
        for ax, av in axes.items():
            ah = av - (av @ u) * u
            ah = ah / np.linalg.norm(ah)
            th = np.degrees(np.arctan2(u @ np.cross(mh, ah), mh @ ah))
            H = (D_TRUE - th) % 360.0
            row.append('%6.1f /%+6.1f' % (H, wrap(H - hdg[k])))
        dip = np.degrees(np.arccos(np.clip(mm @ gg, -1, 1))) - 90.0
        print('  %d  %+6.2f  ' % (k + 1, dip) + '  '.join('%14s' % s for s in row))
