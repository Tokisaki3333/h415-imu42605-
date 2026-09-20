# -*- coding: utf-8 -*-
r"""盲区诊断：把姿态误差拆成"地磁看得见的 2 维"和"地磁看不见的 1 维（绕 b^ 的转动）"。

  theta_mag  = angle(m^_meas, b^_b(R))       地磁残差（可见部分）
  theta_grav = angle(g^_b, R^T z^_w)         重力残差（含盲区）
  盲区角 psi ~ theta_grav / cos(dip)          绕磁场轴 b^ 的转角（I=59.5° -> 系数 0.51）
再打印加速度模值 |a| 与重力 NIS / 门，判断"动态中重力能否软使用"。
"""
import os
import sys
import numpy as np

sys.path.insert(0, r'C:\Users\33\Documents\v2\h415-imu42605-\tools\calib')
import cols_162 as C
import mag360_cal as M

c = C.CH_162
DIP = np.degrees(np.arctan(1.70))
LOG = sys.argv[1] if len(sys.argv) > 1 else r'R:\imu_20260921_030649.bin'
fr, _ = C.load_frames(LOG)
rep = C.frame_report(fr)
print('%s  VER=%s  帧 %d' % (os.path.basename(LOG), rep.get('ver'), rep.get('n', len(fr))))
print('   frame_report:', {k: v for k, v in rep.items() if k != 'ver'})
dt = fr[:, c['dt_us']].astype(float) * 1e-6
t = np.cumsum(dt) - dt[0]
q = fr[:, c['ekf_q0']:c['ekf_q0'] + 4].astype(float)
qn = np.linalg.norm(q, axis=1, keepdims=True)
ok = qn[:, 0] > 0.5
q = np.where(ok[:, None], q / np.maximum(qn, 1e-9), np.array([1.0, 0, 0, 0]))
R = np.stack([M.quat_to_R(r) for r in q])
A, Cv = M.read_current_AC()
mb = (A @ fr[:, c['mag_lsb0']:c['mag_lsb0'] + 3].astype(float).T + Cv[:, None]).T
mhat = mb / np.linalg.norm(mb, axis=1, keepdims=True)
g = fr[:, c['accel_g0']:c['accel_g0'] + 3].astype(float)
agn = np.linalg.norm(g, axis=1)
ghat = g / np.maximum(agn[:, None], 1e-9)
ci = 1.0 / np.sqrt(1 + 1.70 ** 2)
bn = np.array([ci * np.sin(np.radians(-7.53)), ci * np.cos(np.radians(-7.53)), -1.70 * ci])
bb = np.einsum('nji,j->ni', R, bn)
bb /= np.linalg.norm(bb, axis=1, keepdims=True)
th_mag = np.degrees(np.arccos(np.clip(np.einsum('ni,ni->n', mhat, bb), -1, 1)))
gw = np.einsum('nji,j->ni', R, np.array([0.0, 0.0, 1.0]))       # R^T z_w (模型重力机体系)
th_grav = np.degrees(np.arccos(np.clip(np.einsum('ni,ni->n', gw, ghat), -1, 1)))
bg = np.linalg.norm(fr[:, 100:103].astype(float), axis=1)
dqn = np.linalg.norm(fr[:, c['ekf_mag_dqx']:c['ekf_mag_dqz'] + 1].astype(float), axis=1)
w = np.linalg.norm(fr[:, c['gyro_dps0']:c['gyro_dps0'] + 3].astype(float), axis=1)
clip = ((fr[:, c['flags']].astype(int) >> 12) & 1).astype(bool)
print('I_model=%.2f deg   cos(I)=%.3f   盲区角 psi ~ theta_grav/cos(I)' % (DIP, np.cos(np.radians(DIP))))
print('\n   t     w_p50 clip%  |a| p50  |a|min |a|max grav_ok% gnis_p50 |'
      ' th_mag_p50 p90 | th_grav_p50 p90 | psi~p50 |  |bg|dps  used%')
for s in np.arange(0.0, float(t[-1]) + 0.25, 0.5):
    m = (t >= s) & (t < s + 0.5)
    if m.sum() < 5:
        continue
    print('%5.1f %6.0f %5.0f %7.3f %6.3f %6.3f %8.0f %8.0f | %10.2f %7.2f |'
          ' %11.2f %7.2f | %7.1f | %8.2f %5.0f'
          % (s, np.percentile(w[m], 50), 100 * clip[m].mean(), np.median(agn[m]),
             agn[m].min(), agn[m].max(), 100 * (fr[m, 152] > 0.5).mean(),
             np.percentile(fr[m, 153], 50),
             np.percentile(th_mag[m], 50), np.percentile(th_mag[m], 90),
             np.percentile(th_grav[m], 50), np.percentile(th_grav[m], 90),
             np.percentile(th_grav[m], 50) / np.cos(np.radians(DIP)),
             np.median(bg[m]), 100 * (fr[m, 120] > 0.5).mean()))

# ---- 首尾对比：若首尾是同一物理姿态（"放回原位"），则姿态估计的首尾差 = 残余漂移 ----
k0 = np.flatnonzero((t >= 0.2) & (t <= 1.0))
k1 = np.flatnonzero(t >= t[-1] - 2.5)
if k0.size > 20 and k1.size > 20:
    def unit(v):
        return v / np.linalg.norm(v)
    m0, m1 = unit(mhat[k0].mean(0)), unit(mhat[k1].mean(0))
    g0, g1 = unit(ghat[k0].mean(0)), unit(ghat[k1].mean(0))
    print('\n  首尾**物理姿态**自检（地磁/重力机体系，若"放回原位"应 ~0）:')
    print('    磁场机体系夹角 %.2f°   重力机体系夹角 %.2f°'
          % (np.degrees(np.arccos(np.clip(m0 @ m1, -1, 1))),
             np.degrees(np.arccos(np.clip(g0 @ g1, -1, 1)))))
    R0 = M.quat_to_R(unit(q[k0].mean(0)))
    R1 = M.quat_to_R(unit(q[k1].mean(0)))
    Rr = R1 @ R0.T
    ang = np.degrees(np.arccos(np.clip((np.trace(Rr) - 1) / 2, -1, 1)))
    ax = np.array([Rr[2, 1] - Rr[1, 2], Rr[0, 2] - Rr[2, 0], Rr[1, 0] - Rr[0, 1]])
    ax = ax / max(np.linalg.norm(ax), 1e-12)
    rv = np.radians(ang) * ax
    print('    姿态估计首尾差: 总 %.2f°  (竖直/yaw %.2f°  水平 %.2f°)  转轴 %s'
          % (ang, rv[2], np.linalg.norm(rv[:2]), np.round(ax, 2)))
    print('    首尾静置: th_mag %.2f° -> %.2f°   th_grav %.2f° -> %.2f°   |a| %.3f -> %.3f g'
          % (np.median(th_mag[k0]), np.median(th_mag[k1]),
             np.median(th_grav[k0]), np.median(th_grav[k1]),
             np.median(agn[k0]), np.median(agn[k1])))
