# -*- coding: utf-8 -*-
"""判据：地磁水平轴是"镜像"还是"绕 Z 转 180 度"。

原理：机体系矢量在刚体绕竖轴转 +dpsi 时，其机体系方位角变化 -dpsi。
      若地磁 X 轴被镜像（极性反），则方位角变化变为 +dpsi -> 斜率变号。
      "绕 Z 转 180 度"是**真旋转**，不改变方位角变化的方向 -> 斜率不变号。
所以：斜率符号 = 极性是否正确；180 度偏置 = 框架约定，可以用牵引拉掉。

同时给出：把 r = wrap(D - az) 拉掉所需的偏置量，及其随时间的稳定性。
"""
import numpy as np

b = np.fromfile(r'R:\raw_v9.bin', dtype='<f4').reshape(-1, 122).astype(np.float64)
D = -7.53
dt = b[:, 25] * 1e-6
w = b[:, 26:29]                       # gyro_dps, body
f = b[:, 42:45]                       # body mag (unit)
Bw = b[:, 46:49]                      # legacy nav-frame mag
g = np.linalg.norm(w, axis=1)


def wrap(d):
    return (d + 180.0) % 360.0 - 180.0


# 机体系地磁方位角（在垂直于重力的平面内）：用重力把 f 投影到水平面
# 简化：用旧链的导航系地磁 Bw 反推不方便，直接看 f 的方位角变化与 w 的关系
az_b = np.degrees(np.arctan2(f[:, 0], f[:, 1]))
d_az = wrap(np.diff(az_b))            # 每帧变化（有跳变，做鲁棒拟合）

m = (g[:-1] > 100.0) & (np.abs(w[:-1, 2]) > 100.0) & (np.abs(d_az) < 30.0)
lhs = d_az[m]
rhs = -w[:-1, 2][m] * dt[:-1][m]      # 期望: d_az = -w_z*dt
A = np.vstack([rhs, np.ones_like(rhs)]).T
sol, *_ = np.linalg.lstsq(A, lhs, rcond=None)
print('样本 %d 帧（|gyro|>100dps 且 |d_az|<30deg）' % m.sum())
print('  拟合 d_az = k * (-w_z*dt) + b :  k = %+.4f   b = %+.2f deg' % (sol[0], sol[1]))
print('  -> k > 0 : 极性/手性正确（180 度只是框架偏置，可牵引拉掉）')
print('  -> k < 0 : 水平轴被镜像（必须改源头）')
print('  相关系数 r = %+.3f' % np.corrcoef(lhs, rhs)[0, 1])

# 需要的偏置量随时间是否稳定（稳定 = 固定框架偏置；乱跳 = 别的问题）
az_nav = np.degrees(np.arctan2(Bw[:, 0], Bw[:, 1]))
off = wrap(D - az_nav)
print('\n  r = wrap(D - az_nav) 的稳定性：')
print('    全场 p10 %.1f  p50 %.1f  p90 %.1f  标准差 %.1f 度'
      % (np.percentile(off, 10), np.median(off), np.percentile(off, 90), off.std()))
rest = g < 5.0
print('    静止段 p50 %.1f 度   运动段 p50 %.1f 度' % (np.median(off[rest]), np.median(off[~rest])))
# 分段看是否恒定
for k in range(6):
    s, e = k * len(b) // 6, (k + 1) * len(b) // 6
    print('    %5.1f-%5.1fs  off p50 %7.1f  |gyro|p50 %6.1f' % (
        np.cumsum(dt)[s], np.cumsum(dt)[e - 1], np.median(off[s:e]), np.median(g[s:e])))
