# -*- coding: utf-8 -*-
"""VER=12 检查：bg 是否停止发散、姿态/合加速度是否合理、栈与气压是否仍稳。"""
import numpy as np
a = np.fromfile(r'R:\raw_v9.bin', dtype='<f4').reshape(-1, 112)
N = a.shape[0]
t = np.cumsum(a[:, 25].astype(np.float64)) * 1e-6
print('帧数 %d  时长 %.1f s  fw_tag %.0f' % (N, t[-1], a[0, 76]))

print()
print('=== 这次改动的正主：陀螺零偏残差 bg（VER=11 末值 -803 dps）===')
for nm, c in [('bg_x', 99), ('bg_y', 100), ('bg_z', 101)]:
    v = a[:, c]
    print('  %-5s 首 %10.4f  末 %10.4f  min %10.4f  max %10.4f dps'
          % (nm, v[0], v[-1], v.min(), v.max()))
print('  |bg| 分位:', {q: round(float(np.percentile(np.linalg.norm(a[:, 99:102], axis=1), q)), 4)
                       for q in (50, 90, 99, 100)})

print()
print('=== 加计零偏残差 ba ===')
for nm, c in [('ba_x', 96), ('ba_y', 97), ('ba_z', 98)]:
    v = a[:, c]
    print('  %-5s 末 %10.6f  max|.| %10.6f m/s^2' % (nm, v[-1], np.abs(v).max()))

print()
print('=== 姿态 / 合加速度 ===')
q = a[:, 89:93]
qn = np.linalg.norm(q, axis=1)
print('  |ekf_q| min %.6f  (<0.9 帧数 %d)' % (qn.min(), int(np.sum(qn < 0.9))))
an = np.linalg.norm(a[:, 93:96], axis=1)
print('  |a_nav| p50 %.5f  p99 %.5f  max %.5f m/s^2' % (
    np.median(an), np.percentile(an, 99), an.max()))
print('  ekf_q 末 %s' % np.round(q[-1], 5))
print('  旧链 q 末 %s' % np.round(a[-1, 0:4], 5))
# 两者只应差一个绕竖直轴的固定旋转：算一下差角里"倾角"那一部分
def tilt_of(qq):
    w, x, y, z = qq
    return np.degrees(np.arccos(np.clip(1 - 2*(x*x + y*y), -1, 1)))
print('  EKF 倾角 %.4f 度   旧链倾角 %.4f 度   差 %.4f 度'
      % (tilt_of(q[-1]), tilt_of(a[-1, 0:4]), abs(tilt_of(q[-1]) - tilt_of(a[-1, 0:4]))))
print('  sigma_yaw p50 %.4f  max %.4f 度' % (np.median(a[:, 104]), a[:, 104].max()))
print('  sigma_pos_h p50 %.4f   sigma_vel_h p50 %.4f' % (
    np.median(a[:, 105]), np.median(a[:, 106])))

print()
print('=== ISR / 坏帧 ===')
d = a[:, 17]
print('  ISR 分位:', {q: round(float(np.percentile(d, q)), 1) for q in (50, 90, 99, 100)})
print('  >60us 帧数 %d' % int(np.sum(d > 60)))

print()
gb = a[:, 103].astype(np.int32)
print('=== 门 (%) ===')
for nm, m in [('gps_pos',1),('gps_alt',2),('baro',4),('gps_vel',8),('zupt',0x10),
              ('tilt',0x20),('mag_yaw',0x40),('aligned',0x80),('step',0x100),
              ('chi2',0x200),('origin',0x400)]:
    print('  %-9s %6.2f%%' % (nm, 100.0*np.mean((gb & m) != 0)))

print()
print('=== NIS（应趋近 2/2/1/3/1）+ 气压链 ===')
for i, nm in enumerate(['位置', '速度', '气压', '重力', '磁偏航']):
    v = a[:, 107 + i]
    print('  %-4s p50 %9.4f  max %9.2f' % (nm, np.median(v), v.max()))
p = a[:, 80]; h = (101325.0 - p) * 0.08326
print('  h_baro p50 %.3f   p_z 末 %.3f   b_baro 首 %.3f 末 %.3f' % (
    np.median(h), a[-1, 85], a[0, 102], a[-1, 102]))
print('  h_baro-(p_z+b_baro) 末 %.4f m' % (h[-1] - (a[-1, 85] + a[-1, 102])))

print()
print('=== 末 5 秒（应静止）===')
m = t > t[-1] - 5.0
print('  |a_nav| 均值 %.5f m/s^2   sigma_yaw 均值 %.4f 度' % (
    an[m].mean(), a[m, 104].mean()))
print('  v_nav 末 %s   p 末 %s' % (np.round(a[-1, 86:89], 4), np.round(a[-1, 83:86], 4)))
print('  zupt 门 %.1f%%   tilt 门 %.1f%%' % (
    100.0*np.mean((gb[m] & 0x10) != 0), 100.0*np.mean((gb[m] & 0x20) != 0)))
