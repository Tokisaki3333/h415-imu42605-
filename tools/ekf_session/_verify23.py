# -*- coding: utf-8 -*-
"""分段位移验收：
  桌面横向 4×20cm（东→北→西→南，闭环应回原点）+ 原地抬高 20cm + 高位重复 4×20cm
  + 放下 + 静止到底。
关键真值：① 每段 |Δp_h| ≈ 0.20 m  ② 4 段闭环残差 ≈ 0  ③ 抬高 Δp_z ≈ +0.20 m
        ④ 气压高度 h_baro = p_z + b_baro 应独立看到那次抬高
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
gb = a[:, 103].astype(np.int32)
p, v = a[:, 83:86], a[:, 86:89]
h_baro = (101325.0 - a[:, 80]) * 0.08326          # 气压高度（绝对项被 b_baro 吸收）
print('fw_tag %.0f  帧数 %d  时长 %.2f s' % (a[0, 76], N, t[-1]))

aln = np.linalg.norm(a[:, 10:13], axis=1)
w3 = np.linalg.norm(a[:, 26:29], axis=1)
eac = a[:, 81]
k = max(int(0.25/dt.mean()), 1)
mv = ((np.convolve(aln, np.ones(k)/k, mode='same') > 0.04)
      | (np.convolve(w3, np.ones(k)/k, mode='same') > 2.0) | (eac > 0.2))
idx = np.where(np.diff(mv.astype(np.int8)) != 0)[0]
segs = []
for i in range(len(idx)-1):
    s0, s1 = idx[i], idx[i+1]
    if s1-s0 > int(0.10/dt.mean()):
        segs.append((s0, s1, bool(mv[min(s0+1, N-1)])))
# 把连续的运动段合并（真值是一次 20 cm 移动，但可能被细分）
merged = []
for s0, s1, ismv in segs:
    if merged and merged[-1][2] == ismv and t[s0]-t[merged[-1][1]-1] < 0.35:
        merged[-1] = (merged[-1][0], s1, ismv)
    else:
        merged.append((s0, s1, ismv))
print()
print('  # 类型  t起    t止   时长 |a|峰 |w|峰 | Δp_x    Δp_y    Δp_z   |Δp_h|  |v|末  zupt mag baro')
print('  ' + '-'*104)
cum_h = []
for n, (s0, s1, ismv) in enumerate(merged):
    d = p[s1-1] - p[s0]
    print('  %2d %-4s %6.2f %6.2f %5.2f %5.1f %5.0f | %+7.4f %+7.4f %+7.4f  %6.4f  %6.4f %4.0f %4.0f %4.0f'
          % (n, '动' if ismv else '静', t[s0], t[s1-1], t[s1-1]-t[s0],
             aln[s0:s1].max(), w3[s0:s1].max(),
             d[0], d[1], d[2], float(np.linalg.norm(d[:2])), float(np.linalg.norm(v[s1-1])),
             100*np.mean((gb[s0:s1] & 0x10) != 0), 100*np.mean((gb[s0:s1] & 0x40) != 0),
             100*np.mean((gb[s0:s1] & 0x04) != 0)))

print()
print('=== 累计位置（动能段累加的水平位移向量，看闭环）===')
run = np.zeros(3)
for n, (s0, s1, ismv) in enumerate(merged):
    if not ismv:
        continue
    d = p[s1-1] - p[s0]
    run = run + d
    print('  段 %2d 后: 累计 Δp = (%+7.4f, %+7.4f, %+7.4f)   |Δp_h| = %6.4f m'
          % (n, run[0], run[1], run[2], float(np.linalg.norm(run[:2]))))

print()
print('=== 垂直通道：抬高的独立参照 ===')
print('  p_z     首 %.4f 末 %.4f  min %.4f max %.4f' % (p[0, 2], p[-1, 2], p[:, 2].min(), p[:, 2].max()))
print('  b_baro  首 %.4f 末 %.4f' % (a[0, 102], a[-1, 102]))
print('  h_baro  首 %.4f 末 %.4f  min %.4f max %.4f  （峰-峰 %.4f m）'
      % (h_baro[0], h_baro[-1], h_baro.min(), h_baro.max(), h_baro.max()-h_baro.min()))
print('  p_z+b_baro 峰-峰 %.4f m' % ((p[:, 2]+a[:, 102]).max() - (p[:, 2]+a[:, 102]).min()))

print()
print('=== 末尾静止段（应从某时刻起完全静止）===')
last_static = [s for s in merged if not s[2]][-1]
s0, s1, _ = last_static
print('  最后静置段 t=%.2f~%.2f (%.1f s)' % (t[s0], t[s1-1], t[s1-1]-t[s0]))
print('  |v| 均值 %.5f max %.5f m/s    Δp %.5f m    zupt %.0f%%   mag %.0f%%'
      % (np.linalg.norm(v[s0:s1], axis=1).mean(), np.linalg.norm(v[s0:s1], axis=1).max(),
         float(np.linalg.norm(p[s1-1]-p[s0])), 100*np.mean((gb[s0:s1] & 0x10) != 0),
         100*np.mean((gb[s0:s1] & 0x40) != 0)))
print('  sigma_vel_h 首 %.4f 末 %.4f   sigma_pos_h 首 %.4f 末 %.4f'
      % (a[s0, 106], a[s1-1, 106], a[s0, 105], a[s1-1, 105]))
print('  sigma_tilt 末 %.4f 度   sigma_yaw 末 %.4f 度' % (a[s1-1, 107], a[s1-1, 104]))
