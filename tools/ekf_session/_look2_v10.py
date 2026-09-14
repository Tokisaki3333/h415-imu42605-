# -*- coding: utf-8 -*-
import numpy as np
a = np.fromfile(r'R:\raw_v9.bin', dtype='<f4').reshape(-1, 112)
N = a.shape[0]
d = a[:, 17]
print('ISR(us) 分位: ', {q: round(float(np.percentile(d, q)), 1)
                       for q in (50, 75, 90, 93, 94, 95, 96, 97, 99, 100)})
big = np.where(d > 60)[0]
print('>60us 的帧数 %d (%.3f%%)  前 12 个下标 %s' % (len(big), 100*len(big)/N, big[:12]))
if len(big) > 2:
    dd = np.diff(big)
    print('  尖峰间隔 p50 %s  min %s  max %s' % (np.median(dd), dd.min(), dd.max()))
print('  >60us 帧的 ISR 值 p50 %.0f max %.0f' % (np.median(d[big]), d[big].max()))

qn = np.linalg.norm(a[:, 89:93], axis=1)
bad = np.where(qn < 0.9)[0]
print()
print('|ekf_q| < 0.9 的帧 %d 个, 下标 %s' % (len(bad), bad[:12]))
print('|ekf_q| 分位', {q: round(float(np.percentile(qn, q)), 5) for q in (0, 1, 50, 100)})

sy = a[:, 104]
print()
print('sigma_yaw_deg 分位', {q: round(float(np.percentile(sy, q)), 4) for q in (0, 50, 90, 99, 100)})
w = np.where(sy > 1.0)[0]
print('sigma_yaw > 1 度的帧 %d 个, 下标 %s' % (len(w), w[:12]))

gb = a[:, 103].astype(np.int32)
print()
print('gate_bits 出现的全部不同取值 (前 12):', np.unique(gb)[:12])
print('  bit2(baro) 帧数 %d' % int(np.sum((gb & 4) != 0)))
print('  bit0/1/3(GPS) 帧数 %d' % int(np.sum((gb & 0x0B) != 0)))
print('  bit10(origin) 帧数 %d' % int(np.sum((gb & 0x400) != 0)))

print()
print('=== 气压链 ===')
print('  baro_temp  p50 %.1f  范围 %.1f~%.1f' % (np.median(a[:, 52]), a[:, 52].min(), a[:, 52].max()))
p = a[:, 80]
print('  press_avg 首 %.1f 末 %.1f Pa  (变化 %.2f m)' % (p[0], p[-1], (p[0]-p[-1])*0.08326))
z = a[:, 83+2]
bb = a[:, 102]
print('  ekf p_z 首 %.3f 末 %.3f' % (z[0], z[-1]))
print('  b_baro 首 %.3f 末 %.3f  (h_baro - p_z 应等于它)' % (bb[0], bb[-1]))
h = (101325.0 - p) * 0.08326
print('  h_baro 首 %.3f 末 %.3f   p_z+b_baro 末 %.3f  差 %.3f m' % (
    h[0], h[-1], z[-1] + bb[-1], h[-1] - (z[-1] + bb[-1])))
print('  nis_baro p50 %.1f  非零帧 %d' % (np.median(a[:, 109]), int(np.sum(a[:, 109] > 0))))
vz = a[:, 86+2]
print('  ekf v_z p50 %.5f  max|v_z| %.5f' % (np.median(vz), np.abs(vz).max()))
print('  ZUPT 的 nis p50 %.4f  (速度 3 维应 ~3)' % np.median(a[:, 108]))
print('  tilt 的 nis p50 %.4f  (3 维应 ~3)' % np.median(a[:, 110]))
print('  mag  的 nis p50 %.4f  (1 维应 ~1)' % np.median(a[:, 111]))
