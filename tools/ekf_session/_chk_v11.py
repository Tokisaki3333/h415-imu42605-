# -*- coding: utf-8 -*-
"""VER=11 检查：大栈是否有效（踩栈症状是否消失）+ ISR 尖峰 + 气压环是否解开。"""
import numpy as np
a = np.fromfile(r'R:\raw_v9.bin', dtype='<f4').reshape(-1, 112)
N = a.shape[0]
print('帧数 %d  时长 %.1f s   fw_tag %.0f' % (N, a[:, 25].sum() * 1e-6, a[0, 76]))

print()
print('=== 踩栈症状（VER=10 时各有 10~17 帧）===')
qn = np.linalg.norm(a[:, 89:93], axis=1)
print('  |ekf_q|   min %.6f  (<0.9 的帧数 %d, VER=10 是 10)' % (qn.min(), int(np.sum(qn < 0.9))))
sy = a[:, 104]
print('  sigma_yaw max %.4f 度  (>1 度的帧数 %d, VER=10 是 17)' % (sy.max(), int(np.sum(sy > 1.0))))
print('  press_avg == 0 的帧数 %d' % int(np.sum(a[:, 80] == 0)))
print('  b_baro == 0 的帧数 %d' % int(np.sum(a[:, 102] == 0)))

print()
print('=== ISR 耗时 (us) ===')
d = a[:, 17]
print('  分位:', {q: round(float(np.percentile(d, q)), 1) for q in (50, 75, 90, 95, 97, 99, 100)})
big = np.where(d > 60)[0]
print('  >60us 帧数 %d (%.3f%%)   VER=10 是 22569 (6.251%%)' % (len(big), 100.0*len(big)/N))
print('  预算 124.55 us/帧')

print()
gb = a[:, 103].astype(np.int32)
print('=== 门 / 状态位 (%) 和 VER=10 对照 ===')
for nm, m, old in [('gps_pos',1,0.0), ('gps_alt',2,0.0), ('baro',4,0.0), ('gps_vel',8,0.0),
                   ('zupt',0x10,100.0), ('tilt',0x20,100.0), ('mag_yaw',0x40,100.0),
                   ('aligned',0x80,100.0), ('step',0x100,100.0), ('chi2',0x200,0.0),
                   ('origin',0x400,0.0)]:
    print('  %-9s %6.2f%%   (VER=10: %.2f%%)' % (nm, 100.0*np.mean((gb & m) != 0), old))
print('  gate_bits 取值:', np.unique(gb))

print()
print('=== 气压环（VER=10 时 NIS 恒 180、b_baro 锁在 -25.05 不动）===')
p = a[:, 80]
h = (101325.0 - p) * 0.08326
z = a[:, 85]; bb = a[:, 102]; nis = a[:, 109]
print('  h_baro p50 %.3f m    p_z 末 %.3f    b_baro 首 %.3f 末 %.3f' % (
    np.median(h), z[-1], bb[0], bb[-1]))
print('  h_baro-(p_z+b_baro) 末值 %.3f m   (VER=10 末值是 13.46 m)' % (h[-1] - (z[-1] + bb[-1])))
print('  nis_baro p50 %.3f  max %.1f   (应趋近 1)' % (np.median(nis), nis.max()))

print()
print('=== 其它 NIS（应趋近 2/2/1/3/1）===')
for i, nm in enumerate(['位置', '速度', '气压', '重力', '磁偏航']):
    v = a[:, 107 + i]
    print('  %-4s p50 %9.4f  max %9.2f' % (nm, np.median(v), v.max()))

print()
print('=== 状态量 ===')
print('  ekf_q 末 %s   旧链 q 末 %s' % (np.round(a[-1, 89:93], 5), np.round(a[-1, 0:4], 5)))
print('  sigma_yaw p50 %.4f  sigma_pos_h p50 %.4f  sigma_vel_h p50 %.4f'
      % (np.median(sy), np.median(a[:, 105]), np.median(a[:, 106])))
print('  ekf_bg 末 %s dps     ekf_ba 末 %s m/s^2' % (
    np.round(a[-1, 99:102], 4), np.round(a[-1, 96:99], 6)))
print('  |a_nav| p50 %.5f  max %.5f m/s^2' % (
    np.median(np.linalg.norm(a[:, 93:96], axis=1)), np.linalg.norm(a[:, 93:96], axis=1).max()))
