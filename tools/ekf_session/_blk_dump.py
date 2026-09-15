# -*- coding: utf-8 -*-
"""把跳变帧的整个 EKF 块（83..121）打出来：只有 bg 坏 = 状态问题；整块乱 = 撕裂/错位。"""
import glob

import numpy as np

P = max([c for c in glob.glob(r'R:\*.bin') if len(c) and __import__('os').path.getsize(c) > 100000],
        key=__import__('os').path.getmtime)
b = np.fromfile(P, dtype='<f4').reshape(-1, 133).astype(np.float64)
N = b.shape[0]
NAME = (['ekf_p%d' % i for i in range(3)] + ['ekf_v%d' % i for i in range(3)] +
        ['ekf_q%d' % i for i in range(4)] + ['a_nav%d' % i for i in range(3)] +
        ['ekf_ba%d' % i for i in range(3)] + ['ekf_bg%d' % i for i in range(3)] +
        ['b_baro', 'gate_bits', 'sigma_yaw', 'sigma_pos_h', 'sigma_vel_h', 'sigma_tilt',
         'nis0', 'nis1', 'nis2', 'nis3', 'nis4', 'pzz', 'pbb', 'p_pzz', 'p_pbb',
         'mag_gate', 'mag_bh', 'mag_r', 'mag_used', 'p_yy', 'prop_ok', 'f_ok',
         'prop_row', 'stage', 'mag_rej', 'mag_fhb', 'mag_rx', 'mag_ry',
         'mag_dqx', 'mag_dqy', 'mag_dqz'])
assert len(NAME) == 133 - 83, len(NAME)


def yaw(q):
    w, x, y, z = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
    n = np.sqrt(w*w+x*x+y*y+z*z)
    n = np.where((~np.isfinite(n)) | (n < 1e-6), 1.0, n)
    return np.degrees(np.arctan2(2*(w*z+x*y), 1-2*(y*y+z*z)))


def wrap(d):
    return (d + 180.0) % 360.0 - 180.0


ye = yaw(b[:, 89:93])
d = np.abs(wrap(np.diff(ye)))
J = np.flatnonzero(d > 10.0)
print('跳变帧 %d 个；四元数非有限帧 %.4f%%' % (len(J), 100*(~np.isfinite(b[:, 89:93]).all(axis=1)).mean()))
bad = np.flatnonzero(~np.isfinite(b[:, 99:102]).all(axis=1))
print('bg 含非有限值的帧 %d 个 (%.3f%%)' % (len(bad), 100*len(bad)/N))
if len(bad):
    print('  bg 非有限的帧里，ekf_q 非有限占 %.1f%%' % (100*(~np.isfinite(b[bad, 89:93]).all(axis=1)).mean()))

# 找一个"bg 巨大"的帧
mag = np.nanmax(np.abs(b[:, 99:102]), axis=1)
k = int(np.argmax(mag))
print('\nbg 最大的一帧 idx %d (t=%.3f s):' % (k, np.sum(b[:k, 25])*1e-6))
for lab, idx in (('前3帧', k-3), ('前1帧', k-1), ('该帧', k), ('后1帧', k+1), ('后3帧', k+3)):
    if 0 <= idx < N:
        print('  %-6s idx %6d  p=%s  q=%s' % (lab, idx,
              np.array2string(b[idx, 83:86], precision=3),
              np.array2string(b[idx, 89:93], precision=5)))
        print('         ba=%s' % np.array2string(b[idx, 96:99], precision=4))
        print('         bg=%s   b_baro=%.4g  gate=%d  sig_yaw=%.2f  p_yy=%.4g'
              % (np.array2string(b[idx, 99:102], precision=6), b[idx, 102], int(b[idx, 103]),
                 b[idx, 104], b[idx, 121]))
        print('         mag: used=%d bh=%.4f r=%.2f dq=%s'
              % (int(b[idx, 120]), b[idx, 118], b[idx, 119],
                 np.array2string(b[idx, 130:133], precision=4)))

# 超大 bg 帧的分布
big = mag > 100.0
print('\n|bg| > 100 dps 的帧: %d (%.3f%%)   |bg|>1e6: %d' % (big.sum(), 100*big.mean(), (mag > 1e6).sum()))
if big.any():
    t = np.cumsum(b[:, 25]*1e-6)
    tt = t[big]
    print('  出现时刻: %s ...' % np.array2string(tt[:10], precision=2))
    print('  相邻间隔中位 %.3f s' % (np.median(np.diff(tt)) if len(tt) > 1 else -1))
    print('  这些帧的 gate_bits 取值: %s' % np.unique(b[big, 103].astype(int))[:8])
    print('  这些帧的 ekf_q 非有限占 %.1f%%' % (100*(~np.isfinite(b[big, 89:93]).all(axis=1)).mean()))
