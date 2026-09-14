# -*- coding: utf-8 -*-
"""静止段 ZUPT 为什么不开 + mag 基线自举死锁的确认。"""
import re

import numpy as np

SRC = r'C:\Users\33\Documents\v2\h415-imu42605-\V5F\User\src\SPI_rx.c'
NCH = int(re.search(r'#define JF_CH_NUM\s+(\d+)u',
                    open(SRC, 'rb').read().decode('gbk')).group(1))
a = np.fromfile(r'R:\raw_v9.bin', dtype='<f4').reshape(-1, NCH)
N = len(a)
dt = a[:, 25].astype(np.float64) * 1e-6
t = np.cumsum(dt)
gb = a[:, 103].astype(np.int32)

b0 = np.array([-10.10, -15.42, 43.72])
sc = np.array([2028.48, 2040.78, 2016.07])
g_off = (a[:, 21:24] - b0) / sc
amag = np.linalg.norm(g_off, axis=1)
isst = a[:, 8] > 0.5
eac = a[:, 81]
zupt = (gb & 0x10) != 0
mag = (gb & 0x40) != 0
mn = a[:, 45]

print('=== mag 基线自举死锁确认 ===')
print('  mag_norm 头 5 帧: %s' % np.round(mn[:5], 4))
print('  mag_norm 前 1000 帧 min %.4f max %.4f' % (mn[:1000].min(), mn[:1000].max()))
print('  mag_norm 全程 p50 %.4f   min %.4f max %.4f' % (np.median(mn), mn.min(), mn.max()))
print('  mag 门开启 %.2f%%   （若基线锁在开机值上，则永远 0%%）' % (100*mag.mean()))

print()
print('=== 静止段 ZUPT 四条子判据 ===')
for (x0, x1, nm) in [(7.83, 9.25, '出问题的静止段'), (10.74, 11.71, '另一静止段'),
                     (14.74, 16.29, '再一段'), (45.10, 53.53, '最长静止 8.4 s')]:
    i0, i1 = int(np.searchsorted(t, x0)), int(np.searchsorted(t, x1))
    sl = slice(i0, i1)
    print('  %s (%.2f~%.2f s):' % (nm, x0, x1))
    print('     is_static      %6.1f%%' % (100*np.mean(isst[sl])))
    print('     e_ac<0.20      %6.1f%%   (e_ac p50 %.4f)' % (
        100*np.mean(eac[sl] < 0.20), np.median(eac[sl])))
    print('     ||a_off|-1|<.03 %5.1f%%  (p50 %.5f  max %.5f)' % (
        100*np.mean(np.abs(amag[sl]-1) < 0.03), np.median(np.abs(amag[sl]-1)),
        np.abs(amag[sl]-1).max()))
    print('     zupt 实开      %6.1f%%' % (100*np.mean(zupt[sl])))
    print('     mag 实开       %6.1f%%   mag_norm p50 %.4f' % (
        100*np.mean(mag[sl]), np.median(mn[sl])))

print()
print('=== 加速度量级（判断 3.61 g 是真运动还是姿态误差）===')
for (x0, x1, nm) in [(5.15, 7.69, '位移段2'), (7.83, 9.25, '其后静止')]:
    i0, i1 = int(np.searchsorted(t, x0)), int(np.searchsorted(t, x1))
    sl = slice(i0, i1)
    print('  %s: |accel_g| p50 %.4f p99 %.4f max %.4f    |a_lin| p50 %.4f max %.4f'
          % (nm, np.median(np.linalg.norm(a[sl, 32:35], axis=1)),
             np.percentile(np.linalg.norm(a[sl, 32:35], axis=1), 99),
             np.linalg.norm(a[sl, 32:35], axis=1).max(),
             np.median(np.linalg.norm(a[sl, 10:13], axis=1)),
             np.linalg.norm(a[sl, 10:13], axis=1).max()))
    print('        |a_nav| p50 %.4f max %.4f m/s^2   |v| max %.3f m/s'
          % (np.median(np.linalg.norm(a[sl, 93:96], axis=1)),
             np.linalg.norm(a[sl, 93:96], axis=1).max(),
             np.linalg.norm(a[sl, 86:89], axis=1).max()))
