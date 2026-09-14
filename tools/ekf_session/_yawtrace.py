# -*- coding: utf-8 -*-
"""把偏航时间段摊开看：每 0.25 s 一行 —— 旧链偏航 / EKF 偏航 / 磁残差 / 转动速率 / 门。
阈值不再靠"每帧变化 >0.05 度"这种拍脑袋判据（转动慢时每帧只有 0.01 度，永远检不出来）。"""
import numpy as np

a = np.fromfile(r'R:\raw_v9.bin', dtype='<f4').reshape(-1, 112)
N = a.shape[0]
dt = a[:, 25].astype(np.float64) * 1e-6
t = np.cumsum(dt)
D = -7.53


def rot(q, v):
    w, x, y, z = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
    R = np.empty((len(q), 3, 3))
    R[:, 0, 0] = 1-2*(y*y+z*z); R[:, 0, 1] = 2*(x*y-w*z); R[:, 0, 2] = 2*(x*z+w*y)
    R[:, 1, 0] = 2*(x*y+w*z);   R[:, 1, 1] = 1-2*(x*x+z*z); R[:, 1, 2] = 2*(y*z-w*x)
    R[:, 2, 0] = 2*(x*z-w*y);   R[:, 2, 1] = 2*(y*z+w*x);  R[:, 2, 2] = 1-2*(x*x+y*y)
    return np.einsum('nij,nj->ni', R, v)


def yaw(q):
    w, x, y, z = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
    return np.degrees(np.arctan2(2*(w*z+x*y), 1-2*(y*y+z*z)))


def wrap(d):
    return (d + 180.0) % 360.0 - 180.0


qe, ql = a[:, 89:93], a[:, 0:4]
f = a[:, 42:45]
B = rot(qe, f)
h = np.degrees(np.arctan2(B[:, 0], B[:, 1]))
r = wrap(D - h)
ye, yl = yaw(qe), yaw(ql)
# 展开（去 ±180 跳变），看真实连续转角
yeu = np.degrees(np.unwrap(np.radians(ye)))
ylu = np.degrees(np.unwrap(np.radians(yl)))
g = a[:, 26:29]                      # 旧链校正后角速度 dps
gz = g[:, 2]
gb = a[:, 103].astype(np.int32)

print('总转角（展开后）: 旧链 %+.2f 度   EKF %+.2f 度' % (ylu[-1]-ylu[0], yeu[-1]-yeu[0]))
print('|w| 分位: p50 %.2f p90 %.2f p99 %.2f max %.2f dps' % tuple(
    np.percentile(np.linalg.norm(g, axis=1), [50, 90, 99, 100])))
qn = np.linalg.norm(qe, axis=1)
bad = np.where(qn < 0.9)[0]
print('|ekf_q|<0.9 的帧: %d 个, 下标 %s' % (len(bad), bad[:12]))
print()
print('    t     旧链偏航   EKF偏航   磁残差r   |w|dps   tilt mag  chi2')
step = max(int(0.25/dt.mean()), 1)
for i in range(0, N, step):
    j = min(i+step, N-1)
    print('%6.2f  %9.2f  %9.2f  %+8.2f  %8.2f    %d    %d    %d'
          % (t[j], ylu[j], yeu[j], r[j], np.linalg.norm(g[j]), 
             (gb[j] & 0x20) != 0, (gb[j] & 0x40) != 0, (gb[j] & 0x200) != 0))
