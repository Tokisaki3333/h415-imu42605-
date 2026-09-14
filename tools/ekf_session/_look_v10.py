# -*- coding: utf-8 -*-
"""VER=10 首份记录速览：只回答四个问题
   1) EKF 有没有对齐/在步进；2) ISR 尖峰多大（EKF 步那一帧）；3) 各门开了几成；
   4) 状态量与旧链是否自洽（ekf.q vs q、NIS、sigma）。"""
import numpy as np

a = np.fromfile(r'R:\raw_v9.bin', dtype='<f4').reshape(-1, 112)
N = a.shape[0]
print('帧数 %d  时长 %.1f s' % (N, a[:, 25].sum() * 1e-6))

gb = a[:, 103].astype(np.int32)
bits = [('gps_pos', 0x01), ('gps_alt', 0x02), ('baro', 0x04), ('gps_vel', 0x08),
        ('zupt', 0x10), ('tilt', 0x20), ('mag_yaw', 0x40), ('aligned', 0x80),
        ('step', 0x100), ('chi2', 0x200), ('origin', 0x400)]
print()
print('=== 门 / 状态位（占全部帧的百分比）===')
for nm, m in bits:
    print('  %-9s %6.2f%%' % (nm, 100.0 * np.mean((gb & m) != 0)))

step = (gb & 0x100) != 0
print()
print('=== ISR 耗时 col17 (us) ===')
d = a[:, 17]
print('  全部帧    p50 %.0f  p99 %.0f  max %.0f' % (np.percentile(d, 50), np.percentile(d, 99), d.max()))
if step.any():
    ds, dn = d[step], d[~step]
    print('  EKF步那帧 p50 %.0f  p99 %.0f  max %.0f   (n=%d)' % (
        np.percentile(ds, 50), np.percentile(ds, 99), ds.max(), step.sum()))
    print('  其他帧    p50 %.0f  p99 %.0f  max %.0f' % (
        np.percentile(dn, 50), np.percentile(dn, 99), dn.max()))
print('  预算 124.56 us/帧')

print()
print('=== EKF 输出 ===')
for nm, c, k in [('sigma_yaw_deg', 104, 1), ('sigma_pos_h', 105, 1), ('sigma_vel_h', 106, 1),
                 ('b_baro', 102, 1)]:
    v = a[:, c]
    print('  %-14s min %9.4f  p50 %9.4f  max %9.4f' % (nm, v.min(), np.median(v), v.max()))
print('  ekf_a_nav |a| p50 %.4f m/s^2  max %.4f' % (
    np.median(np.linalg.norm(a[:, 93:96], axis=1)), np.linalg.norm(a[:, 93:96], axis=1).max()))
print('  ekf_p  末值 %s' % np.round(a[-1, 83:86], 3))
print('  ekf_v  末值 %s' % np.round(a[-1, 86:89], 3))
print('  ekf_bg 末值 %s dps' % np.round(a[-1, 99:102], 4))
print('  ekf_ba 末值 %s m/s^2' % np.round(a[-1, 96:99], 5))
qn = a[:, 89:93]
print('  ekf_q 模长 min %.6f max %.6f' % (np.linalg.norm(qn, axis=1).min(),
                                          np.linalg.norm(qn, axis=1).max()))
print('  ekf_q 末值 %s' % np.round(a[-1, 89:93], 5))
print('  旧链 q 末值 %s' % np.round(a[-1, 0:4], 5))

print()
print('=== NIS（应趋近 2/2/1/3/1）===')
for i, nm in enumerate(['位置', '速度', '气压', '重力', '磁偏航']):
    v = a[:, 107 + i]
    nz = v[v > 0]
    print('  %-4s 有值时 p50 %8.3f  (有值帧 %d / %d)' % (
        nm, np.median(nz) if len(nz) else float('nan'), len(nz), N))

print()
print('=== 传感器/门源 ===')
for nm, c in [('is_static', 8), ('att_tilt', 36), ('vel_soft', 37), ('mag_trust', 50),
              ('gps_fixq', 58), ('gps_sv', 59), ('gps_hdop', 60), ('gps_vdop', 63),
              ('gps_sats_view', 70), ('snr_avg', 71), ('snr_n', 73), ('talkers', 75)]:
    v = a[:, c]
    print('  %-14s min %8.2f  p50 %8.2f  max %8.2f  唯一值 %d' % (
        nm, v.min(), np.median(v), v.max(), len(np.unique(v))))
print('  baro_press_avg  p50 %.1f Pa  首末差 %.1f Pa' % (
    np.median(a[:, 80]), a[-1, 80] - a[0, 80]))
print('  e_ac p50 %.4f  ac_bypass %.1f%%' % (np.median(a[:, 81]), 100 * np.mean(a[:, 82] > 0.5)))
print('  flags 唯一值 %s' % np.unique(a[:, 16].astype(np.int32))[:6])
