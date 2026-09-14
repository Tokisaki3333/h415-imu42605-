# -*- coding: utf-8 -*-
"""气压计符号与特性：
 ① 桌面上等高的那一段（抬高之前）应当**平**；用它的 std 衡量真实噪声
 ② 抬高的那一下压差符号：真抬升 20 cm -> p 应**下降** 2.4 Pa
 ③ 看 EKF 的 p_z / h_baro 在这两处的走向
"""
import re

import numpy as np

SRC = r'C:\Users\33\Documents\v2\h415-imu42605-\V5F\User\src\SPI_rx.c'
NCH = int(re.search(r'#define JF_CH_NUM\s+(\d+)u',
                    open(SRC, 'rb').read().decode('gbk')).group(1))
a = np.fromfile(r'R:\raw_v9.bin', dtype='<f4').reshape(-1, NCH)
N = len(a)
dt = a[:, 25].astype(np.float64) * 1e-6
t = np.cumsum(dt)
p_avg = a[:, 80].astype(np.float64)      # press_avg（软件 OSR）
p_single = a[:, 51].astype(np.float64)   # 单样本
pz, bb = a[:, 85].astype(np.float64), a[:, 102].astype(np.float64)
hb = (101325.0 - p_avg) * 0.08326
print('fw_tag %.0f  时长 %.2f s' % (a[0, 76], t[-1]))
print()
print('press_avg 全程: min %.2f max %.2f Pa  峰-峰 %.2f Pa (= %.3f m)'
      % (p_avg.min(), p_avg.max(), p_avg.max()-p_avg.min(),
         (p_avg.max()-p_avg.min())*0.08326))
nz = p_avg[p_avg > 1000]
print('press_avg 去掉 0 值后: 唯一值 %d, p50 %.2f, std %.3f Pa (=%.1f mm)'
      % (len(np.unique(nz)), np.median(nz), nz.std(), nz.std()*83.26))
print('单样本 press_pascal: p50 %.2f std %.3f Pa' % (
    np.median(p_single[p_single > 1000]), p_single[p_single > 1000].std()))

print()
print('=== ① 桌面等高段（抬高之前）逐 1 s：应当平 ===')
for x0, x1 in [(0.2, 3.0), (3.0, 7.0), (7.0, 11.0), (11.0, 15.0)]:
    i0, i1 = int(np.searchsorted(t, x0)), int(np.searchsorted(t, x1))
    q = p_avg[i0:i1]
    q = q[q > 1000]
    if len(q) < 10:
        continue
    print('  t=%5.1f~%5.1f  唯一值 %5d  p50 %.3f  段内 std %.4f Pa (=%.2f mm)  '
          '首末差 %+7.3f Pa (=%+6.2f cm)'
          % (x0, x1, len(np.unique(q)), np.median(q), q.std(), q.std()*83.26,
             q[-1]-q[0], (q[0]-q[-1])*8.326))

print()
print('=== ② 全时间轴（每 1 s）：press_avg / h_baro / p_z / b_baro / zupt / baro门 ===')
gb = a[:, 103].astype(np.int32)
kk = max(int(1.0/dt.mean()), 1)
for i in range(0, N, kk):
    print('  t=%5.1f  p=%9.3f Pa  h=%+8.3f m  p_z=%+8.3f  b_baro=%+8.3f  zupt=%d baro=%d'
          % (t[i], p_avg[i], hb[i], pz[i], bb[i],
             (gb[i] & 0x10) != 0, (gb[i] & 0x04) != 0))
