# -*- coding: utf-8 -*-
r"""静止时 EKF 倾角 vs 加速度计真值：确认那 3.25 deg 是真实静差还是 pr 列不新鲜。

(1) pr(col141..143) 与 由发布四元数算出的 up_body = R(q)^T*[0,0,1] 是否一致；
(2) 加速度计方向 与 两者 的夹角；
(3) 倾角更新的 NIS(col111) 与重力门、sigma_tilt。
"""
import sys, os, math
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'calib'))
from cols_162 import load_frames, CH_162

C = CH_162
RAD2DEG = 180.0 / math.pi


def q_to_R(q):
    w, x, y, z = q / max(np.linalg.norm(q), 1e-12)
    return np.array([[1-2*(y*y+z*z), 2*(x*y-w*z), 2*(x*z+w*y)],
                     [2*(x*y+w*z), 1-2*(x*x+z*z), 2*(y*z-w*x)],
                     [2*(x*z-w*y), 2*(y*z+w*x), 1-2*(x*x+y*y)]])


path = sys.argv[1] if len(sys.argv) > 1 else r'R:\imu_20260922_211657.bin'
fr = load_frames(path)
a = fr[0] if isinstance(fr, tuple) else fr
n = len(a)
acc = np.asarray(a[:, C['accel_g0']:C['accel_g0'] + 3], float)
an = np.linalg.norm(acc, axis=1)
accn = acc / an[:, None]
pr = np.asarray(a[:, C['ekf_tilt_prx']:C['ekf_tilt_prx'] + 3], float)
q = np.asarray(a[:, C['ekf_q0']:C['ekf_q0'] + 4], float)
nis3 = np.asarray(a[:, C['ekf_nis3']], float)
sg = np.asarray(a[:, C['ekf_sigma_tilt_deg']], float)
gt = np.asarray(a[:, C['gate_ekf_tilt']], float)
upq = np.array([q_to_R(q[k]).T @ np.array([0.0, 0.0, 1.0]) for k in range(n)])
ang_pr_acc = np.degrees(np.arccos(np.clip(np.sum(pr * accn, axis=1), -1, 1)))
ang_q_acc = np.degrees(np.arccos(np.clip(np.sum(upq * accn, axis=1), -1, 1)))
ang_pr_q = np.degrees(np.arccos(np.clip(np.sum(pr * upq, axis=1), -1, 1)))
print('%s  n=%d' % (os.path.basename(path), n))
for nm, v in (('pr vs q-up', ang_pr_q), ('accel vs q-up', ang_q_acc),
              ('accel vs pr', ang_pr_acc)):
    print('  %-14s p50 %.3f  p95 %.3f  max %.3f deg' % (nm, np.percentile(v, 50),
          np.percentile(v, 95), v.max()))
print('  NIS3(col111) p50 %.2f p95 %.2f max %.2f  (阈值 V5F_EKF_NIS_MAX_3=16.27)'
      % (np.percentile(nis3, 50), np.percentile(nis3, 95), nis3.max()))
print('  sigma_tilt p50 %.3f deg   gateT %.0f%%' % (np.median(sg), 100 * np.mean(gt > 0.5)))
# 每 30 s 看一次，便于观察趋势
dt = np.asarray(a[:, C['dt_us']], float) * 1e-6
dt = np.where((dt > 1e-4) & (dt < 0.05), dt, 3.0e-3)
t = np.cumsum(dt)
print('  t(s)  accel-vs-qup  accel-vs-pr  pr-vs-qup  NIS3  sigma_tilt')
for s in range(0, int(t[-1]), 30):
    m = (t >= s) & (t < s + 30)
    if not m.any():
        continue
    print('  %5d %11.3f %12.3f %11.4f %6.2f %11.3f'
          % (s, np.median(ang_q_acc[m]), np.median(ang_pr_acc[m]), np.median(ang_pr_q[m]),
             np.median(nis3[m]), np.median(sg[m])))
