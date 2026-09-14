# -*- coding: utf-8 -*-
"""为什么 M3(气压) 一次都没成功：把 r、NIS、S=r^2/NIS、b_baro、p_z 按周期打出来。"""
import numpy as np
a = np.fromfile(r'R:\raw_v9.bin', dtype='<f4').reshape(-1, 112)
h = (101325.0 - a[:, 80]) * 0.08326
z = a[:, 85]
bb = a[:, 102]
nis = a[:, 109]

# 每个 EKF 周期只发布一次，取发布帧（gate_bits 变化的帧）
gb = a[:, 103].astype(np.int32)
pub = np.where(np.diff(gb, prepend=gb[0] - 1) != 0)[0]
print('发布帧数 %d' % len(pub))
print()
print('  idx    h_baro    p_z     b_baro     r        NIS      S=r^2/NIS   sig_pos_h')
for k in list(range(0, 12)) + list(range(len(pub) - 4, len(pub))):
    i = pub[k]
    r = h[i] - (z[i] + bb[i])
    S = (r * r) / nis[i] if nis[i] > 0 else float('nan')
    print('%6d %9.3f %8.3f %9.3f %8.3f %9.2f %9.4f %9.4f'
          % (i, h[i], z[i], bb[i], r, nis[i], S, a[i, 105]))

print()
print('b_baro 变化范围 %.4f ~ %.4f (跨 %d 个发布帧)' % (bb[pub].min(), bb[pub].max(), len(pub)))
print('p_z    变化范围 %.4f ~ %.4f' % (z[pub].min(), z[pub].max()))
print('r      变化范围 %.3f ~ %.3f' % ((h - z - bb)[pub].min(), (h - z - bb)[pub].max()))
print('nis2   变化范围 %.2f ~ %.2f   中位 %.2f' % (nis[pub].min(), nis[pub].max(), np.median(nis[pub])))
Sv = (h - z - bb)[pub] ** 2 / np.maximum(nis[pub], 1e-9)
print('S      中位 %.4f  最小 %.4f  最大 %.4f' % (np.median(Sv), Sv.min(), Sv.max()))
print()
print('sigma_pos_h 中位 %.4f  (=sqrt(P00+P11))' % np.median(a[pub, 105]))
print('sigma_vel_h 中位 %.4f' % np.median(a[pub, 106]))
print('b_baro 首 5 个发布帧: %s' % np.round(bb[pub[:5]], 4))
print('p_z    首 5 个发布帧: %s' % np.round(z[pub[:5]], 4))
print('h_baro 首 5 个发布帧: %s' % np.round(h[pub[:5]], 4))
print('press_avg 首 5 帧原始: %s' % np.round(a[:5, 80], 1))
print('press_avg 帧0~2000 的 min/max: %.1f / %.1f' % (a[:2000, 80].min(), a[:2000, 80].max()))
