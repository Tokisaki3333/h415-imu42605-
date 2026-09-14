# -*- coding: utf-8 -*-
"""按 113 列的正确列号读：sigma_tilt=107, nis=[108..112]。
并查 mag.ok 那个 |y| 模长门到底是不是在乱关。"""
import re

import numpy as np

SRC = r'C:\Users\33\Documents\v2\h415-imu42605-\V5F\User\src\SPI_rx.c'
NCH = int(re.search(r'#define JF_CH_NUM\s+(\d+)u',
                    open(SRC, 'rb').read().decode('gbk')).group(1))
a = np.fromfile(r'R:\raw_v9.bin', dtype='<f4').reshape(-1, NCH)
N = len(a)
dt = a[:, 25].astype(np.float64) * 1e-6
t = np.cumsum(dt)
print('NCH=%d  帧数 %d  时长 %.1f s' % (NCH, N, t[-1]))

# 113 列：107 sigma_tilt, 108..112 nis
NIS0 = 108
print()
print('=== NIS（正确列号）===')
for i, nm, dim in [(0, '位置', 2), (1, '速度', 2), (2, '气压', 1), (3, '重力', 3), (4, '磁偏航', 1)]:
    v = a[:, NIS0 + i]
    nz = v[v > 0]
    print('  %-4s 维数 %d  非零 %6d 帧  p50 %9.4f  p90 %9.4f  max %10.1f'
          % (nm, dim, len(nz), np.median(nz) if len(nz) else float('nan'),
             np.percentile(nz, 90) if len(nz) else float('nan'), v.max()))
print('  sigma_tilt_deg p50 %.4f  p90 %.4f  max %.4f 度' % (
    np.median(a[:, 107]), np.percentile(a[:, 107], 90), a[:, 107].max()))
print('  sigma_yaw_deg  p50 %.4f  max %.4f 度' % (np.median(a[:, 104]), a[:, 104].max()))

print()
print('=== 气压环：nis_baro 随时间 + p_z/b_baro ===')
nb = a[:, NIS0 + 2]
hd = (101325.0 - a[:, 80]) * 0.08326
pz, bb = a[:, 85], a[:, 102]
gb = a[:, 103].astype(np.int32)
baro_ok = (gb & 0x04) != 0
print('  nis_baro: p50 %.3f  >10.83 的帧占比 %.1f%%   （硬门限 10.83）'
      % (np.median(nb), 100*np.mean(nb > 10.83)))
print('  baro 门成功 %d 帧 (%.2f%%)   理论尝试上限 %.0f 次' % (
    baro_ok.sum(), 100*baro_ok.mean(), 6.1*t[-1]))
print('  p_z      min %8.2f max %8.2f' % (pz.min(), pz.max()))
print('  b_baro   min %8.2f max %8.2f' % (bb.min(), bb.max()))
print('  h_baro-(p_z+b_baro)  p50 %8.3f  p90 %8.3f  max|.| %10.1f'
      % (np.median(hd - (pz + bb)), np.percentile(np.abs(hd - (pz + bb)), 90),
         np.abs(hd - (pz + bb)).max()))

print()
print('=== mag.ok 那个 |y| 模长门 ===')
mn = a[:, 45]                      # mag_norm（归一化前的模长，理论 1.0）
mag_ok = (gb & 0x40) != 0
print('  mag_norm p50 %.4f  p10 %.4f  p90 %.4f  min %.4f  max %.4f'
      % (np.median(mn), np.percentile(mn, 10), np.percentile(mn, 90), mn.min(), mn.max()))
print('  |mag_norm-1| p50 %.4f  p90 %.4f   （门限 V5F_MAG_ERR_LIM=0.10）'
      % (np.median(np.abs(mn-1)), np.percentile(np.abs(mn-1), 90)))
print('  mag 门开启 %.1f%%   mag.trust(col50) 开启 %.1f%%' % (
    100*mag_ok.mean(), 100*np.mean(a[:, 50] > 0.5)))
# 门关的帧里模长偏离多少
off = np.abs(mn - 1)
print('  mag门关的帧: |mag_norm-1| p50 %.4f  max %.4f' % (
    np.median(off[~mag_ok]) if (~mag_ok).any() else float('nan'),
    off[~mag_ok].max() if (~mag_ok).any() else float('nan')))
print('  mag门开的帧: |mag_norm-1| p50 %.4f  max %.4f' % (
    np.median(off[mag_ok]) if mag_ok.any() else float('nan'),
    off[mag_ok].max() if mag_ok.any() else float('nan')))
