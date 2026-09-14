# -*- coding: utf-8 -*-
"""VER=26 验收：20cm 台阶 -> 40cm 台阶 -> 转一圈放回。
读：baro 门是否恢复到 ~1.75% / p_z 台阶 / P0 自检四列 / b_baro 来源 / ba / 整圈偏航闭环。"""
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
p_avg = a[:, 80].astype(np.float64)
hb = (101325.0 - p_avg) * 0.08326
pz, bb = a[:, 85].astype(np.float64), a[:, 102].astype(np.float64)
print('fw_tag %.0f  列数 %d  时长 %.2f s' % (a[0, 76], NCH, t[-1]))
print('baro 门开启 %.2f%%  （设计 6.1 Hz -> 约 1.75%%；VER=25 只有 0.34%%）'
      % (100*np.mean((gb & 0x04) != 0)))
print('nis_baro p50 %.3f   max %.1f' % (np.median(a[:, 110]), a[:, 110].max()))
print()
print('=== P0 自检四列（列 113/114 = s_Pn；115/116 = s_P）===')
for nm, c in [('pzz (s_Pn[2][2])', 113), ('pbb (s_Pn[15][15])', 114),
              ('p_pzz (s_P[2][2])', 115), ('p_pbb (s_P[15][15])', 116)]:
    v = a[:, c]
    print('  %-20s 首 %9.4f  末 %9.4f  max %10.4f' % (nm, v[0], v[-1], v.max()))
print('  b_baro 首 %+.4f 末 %+.4f   min %+.4f  max %+.4f' % (bb[0], bb[-1], bb.min(), bb.max()))
print('  ba 末 %s   |ba| max %.5f m/s^2' % (np.round(a[-1, 96:99], 5),
                                            np.linalg.norm(a[:, 96:99], axis=1).max()))
print()
print('=== h_baro 的台阶（差分找 20/40cm）===')
nz = p_avg > 1000
k5 = max(int(3.0/dt.mean()), 1)
d5 = hb[k5:] - hb[:-k5]
print('  h_baro 全程峰-峰 %.3f m（去0值）  p_z 峰-峰 %.3f m'
      % (hb[nz].max()-hb[nz].min(), pz.max()-pz.min()))
for nm, v in (('h_baro', hb), ('p_z', pz)):
    dd = v[k5:] - v[:-k5]
    print('  %s 3 s 差分: max %+.3f (t=%.1f)  min %+.3f (t=%.1f)'
          % (nm, dd.max(), t[int(np.argmax(dd))+k5], dd.min(), t[int(np.argmin(dd))+k5]))
print()
print('=== 整圈偏航闭环（转一圈应回到原朝向）===')
def yaw(q):
    w, x, y, z = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
    return np.degrees(np.arctan2(2*(w*z+x*y), 1-2*(y*y+z*z)))
for nm, q in (('EKF', a[:, 89:93]), ('旧链', a[:, 0:4])):
    yy = yaw(q)
    yu = np.degrees(np.unwrap(np.radians(yy)))
    print('  %-4s 总转角 %+9.2f 度   首末差 %+8.3f 度' % (nm, yu[-1]-yu[0], yy[-1]-yy[0]))
w3 = np.linalg.norm(a[:, 26:29], axis=1)
print('  |w| p50 %.2f  p99 %.1f  max %.0f dps' % (
    np.median(w3), np.percentile(w3, 99), w3.max()))
print('  sigma_yaw p50 %.4f max %.4f 度   sigma_tilt p50 %.4f max %.4f 度'
      % (np.median(a[:, 104]), a[:, 104].max(), np.median(a[:, 107]), a[:, 107].max()))
print()
print('=== 时间轴（每 1 s）===')
print('   t    press_avg   h_baro    p_z    b_baro  P_zz  P_bb   pzz  pbb   p_pzz p_pbb  baro')
kk = max(int(1.0/dt.mean()), 1)
for i in range(0, N, kk):
    print('  %5.1f %10.2f %+9.3f %+7.3f %+8.3f %5.2f %5.2f  %5.2f %5.2f  %5.2f %5.2f   %d'
          % (t[i], p_avg[i], hb[i], pz[i], bb[i], a[i, 115], a[i, 116],
             a[i, 113], a[i, 114], a[i, 115], a[i, 116], (gb[i] & 0x04) != 0))
