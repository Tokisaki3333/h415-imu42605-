# -*- coding: utf-8 -*-
"""VER=25 验收：抬高 40cm 再放回。
 ① 气压差值观测能否干净给出 ±40cm 台阶（旧版被 ±33cm 慢漂淹掉）
 ② p_z 是否跟着台阶走、b_baro 是否不再被绝对观测拉扯
 ③ P_zz / P_bb 分配、ba 是否吸走泄漏、baro 门是否真的在跑
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
print('fw_tag %.0f  列数 %d  时长 %.2f s' % (a[0, 76], NCH, t[-1]))

p_avg = a[:, 80].astype(np.float64)
hb = (101325.0 - p_avg) * 0.08326
pz, bb = a[:, 85].astype(np.float64), a[:, 102].astype(np.float64)
ba = a[:, 96:99]
pzz, pbb = a[:, 113], a[:, 114]
nis_b = a[:, 110]
nz = p_avg > 1000
print('press_avg(去0): p50 %.2f  峰-峰 %.2f Pa (= %.2f m)' % (
    np.median(p_avg[nz]), p_avg[nz].max()-p_avg[nz].min(),
    (p_avg[nz].max()-p_avg[nz].min())*0.08326))
print('baro 门开启 %.2f%%   nis_baro p50 %.3f' % (
    100*np.mean((gb & 0x04) != 0), np.median(nis_b)))
print('P_zz  首 %.3f 末 %.3f   P_bb 首 %.3f 末 %.3f' % (pzz[0], pzz[-1], pbb[0], pbb[-1]))
print('b_baro 首 %.3f 末 %.3f   p_z 首 %.3f 末 %.3f' % (bb[0], bb[-1], pz[0], pz[-1]))
print('ba 末 %s m/s^2   |ba| max %.4f' % (np.round(ba[-1], 5), np.linalg.norm(ba, axis=1).max()))
print()
print('   t    press_avg   h_baro    p_z     b_baro   P_zz   P_bb   nis_b  baro zupt')
kk = max(int(1.0/dt.mean()), 1)
for i in range(0, N, kk):
    print('  %5.1f %10.2f %+9.3f %+8.3f %+8.3f %6.2f %7.2f %7.3f   %d   %d'
          % (t[i], p_avg[i], hb[i], pz[i], bb[i], pzz[i], pbb[i], nis_b[i],
             (gb[i] & 0x04) != 0, (gb[i] & 0x10) != 0))
print()
# 台阶检测：h_baro 的 5 s 差分
k5 = max(int(5.0/dt.mean()), 1)
d5 = hb[k5:] - hb[:-k5]
print('h_baro 的 5 s 差分: min %+.3f max %+.3f m  (抬高 40cm 应看到 ~+0.4 再 ~-0.4)'
      % (d5.min(), d5.max()))
i1 = int(np.argmax(d5)); i2 = int(np.argmin(d5))
print('  最大上升在 t=%.1f s (%+.3f m)   最大下降在 t=%.1f s (%+.3f m)'
      % (t[i1+k5], d5[i1], t[i2+k5], d5[i2]))
dp5 = pz[k5:] - pz[:-k5]
print('p_z   的 5 s 差分: min %+.3f max %+.3f m' % (dp5.min(), dp5.max()))
print('  p_z 最大上升在 t=%.1f s (%+.3f m)  最大下降 t=%.1f s (%+.3f m)'
      % (t[int(np.argmax(dp5))+k5], dp5.max(), t[int(np.argmin(dp5))+k5], dp5.min()))
print('  p_z 全程峰-峰 %.3f m；h_baro 全程峰-峰 %.3f m（去0值）'
      % (pz.max()-pz.min(), hb[nz].max()-hb[nz].min()))
