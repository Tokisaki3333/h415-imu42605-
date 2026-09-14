# -*- coding: utf-8 -*-
"""看开头的"慢速平移"段：门控 + 速度。判据是**持续性**（走动有几秒的周期波形），
而不是峰值 —— 峰值分不开"走动"和"摇晃"。"""
import re

import numpy as np

SRC = r'C:\Users\33\Documents\v2\h415-imu42605-\V5F\User\src\SPI_rx.c'
NCH = int(re.search(r'#define JF_CH_NUM\s+(\d+)u',
                    open(SRC, 'rb').read().decode('gbk')).group(1))
a = np.fromfile(r'R:\raw_v9.bin', dtype='<f4').reshape(-1, NCH)
dt = a[:, 25].astype(np.float64) * 1e-6
t = np.cumsum(dt)
gb = a[:, 103].astype(np.int32)
aln = np.linalg.norm(a[:, 10:13], axis=1)
w3 = np.linalg.norm(a[:, 26:29], axis=1)
eac = a[:, 81]
vn = a[:, 86:89]
vz = a[:, 88]
svel = a[:, 106]

print('t=0~9 s，每 40 ms 一行（走动要有几秒的持续波形，摇晃是孤立爆发）')
print('   t     |a_lin|   |w|     e_ac   is_st  zupt tilt mag | v_nav(x,y,z)          |v|    sig_vel')
k = max(int(0.04/dt.mean()), 1)
for i in range(0, int(9.0/dt.mean()), k):
    j = min(i+k, len(a)-1)
    print('%5.2f  %7.3f %7.1f %8.2f    %d     %d    %d   %d  | %6.3f %6.3f %6.3f  %6.3f  %6.3f'
          % (t[j], aln[j], w3[j], eac[j], a[j, 8] > 0.5,
             (gb[j] & 0x10) != 0, (gb[j] & 0x20) != 0, (gb[j] & 0x40) != 0,
             vn[j, 0], vn[j, 1], vn[j, 2], np.linalg.norm(vn[j]), svel[j]))

print()
print('=== 前 9 秒汇总 ===')
n0, n1 = 0, int(9.0/dt.mean())
print('  |a_lin| 分位: ' + '  '.join('p%d %.3f' % (q, np.percentile(aln[n0:n1], q))
                                     for q in (50, 90, 99, 100)) + ' g')
print('  |w|     分位: ' + '  '.join('p%d %.1f' % (q, np.percentile(w3[n0:n1], q))
                                     for q in (50, 90, 99, 100)) + ' dps')
print('  e_ac    分位: ' + '  '.join('p%d %.2f' % (q, np.percentile(eac[n0:n1], q))
                                     for q in (50, 90, 99, 100)))
print('  is_static %.1f%%   zupt %.1f%%   tilt %.1f%%   mag %.1f%%' % (
    100*np.mean(a[n0:n1, 8] > 0.5), 100*np.mean((gb[n0:n1] & 0x10) != 0),
    100*np.mean((gb[n0:n1] & 0x20) != 0), 100*np.mean((gb[n0:n1] & 0x40) != 0)))
print('  |v| max %.3f m/s   sigma_vel_h max %.3f   v_z max|.| %.3f' % (
    np.linalg.norm(vn[n0:n1], axis=1).max(), svel[n0:n1].max(), np.abs(vz[n0:n1]).max()))
# 周期性：|a_lin| 的自相关峰值（走动应有 ~2 Hz 的步频）
x = aln[n0:n1] - aln[n0:n1].mean()
ac = np.correlate(x, x, 'full')[len(x)-1:]
ac = ac / ac[0]
lag = np.argmax(ac[int(0.2/dt.mean()):int(1.0/dt.mean())]) + int(0.2/dt.mean())
print('  |a_lin| 自相关在 %.3f s 处有峰 (值 %.3f)  —— 2 Hz 步频约在 0.5 s'
      % (lag*dt.mean(), ac[lag]))
print()
print('  对照：17.5~19 s（剧烈摇晃段）')
m0, m1 = int(17.5/dt.mean()), int(19.0/dt.mean())
x2 = aln[m0:m1] - aln[m0:m1].mean()
ac2 = np.correlate(x2, x2, 'full')[len(x2)-1:]
ac2 = ac2 / ac2[0]
lag2 = np.argmax(ac2[int(0.2/dt.mean()):int(1.0/dt.mean())]) + int(0.2/dt.mean())
print('  |a_lin| p50 %.3f  p99 %.3f g   e_ac p50 %.1f   自相关峰在 %.3f s (值 %.3f)' % (
    np.median(aln[m0:m1]), np.percentile(aln[m0:m1], 99), np.median(eac[m0:m1]),
    lag2*dt.mean(), ac2[lag2]))
