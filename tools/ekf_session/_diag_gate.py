# -*- coding: utf-8 -*-
"""门位时间线：aligned(0x80) / mag_yaw(0x40) / origin(0x400) / chi2_rej(0x200)。

假设：VER=44 里 EKF 从头到尾没有完成对齐 -> 偏航从未被地磁锚定 -> r 恒 ~170 度、
mag 门恒关、mag_used 恒 0。若成立，则"地磁没有牵引"的真因是**对齐没成功**，
而不是我上一轮认定的"陈旧补偿相加/复合"。
"""
import numpy as np

b = np.fromfile(r'R:\raw_v9.bin', dtype='<f4').reshape(-1, 122).astype(np.float64)
N = len(b)
gb = b[:, 103].astype(int)
dt = np.cumsum(b[:, 25] * 1e-6)
g = np.linalg.norm(b[:, 26:29], axis=1)

BITS = [(0x80, 'aligned'), (0x40, 'mag_yaw'), (0x10, 'zupt'), (0x20, 'tilt'),
        (0x04, 'baro'), (0x400, 'origin'), (0x200, 'chi2_rej'), (0x800, 'acc_sat')]

print('总时长 %.2f s   帧 %d' % (dt[-1], N))
print('\n各门位全场占比与**首次置位时刻**：')
for bit, nm in BITS:
    on = (gb & bit) != 0
    first = dt[on][0] if on.any() else float('nan')
    print('  %-9s 0x%03X  占比 %6.2f%%   首次置位 %7.2f s' % (nm, bit, 100.0 * on.mean(), first))
    if nm == 'mag_yaw':
        tr = np.flatnonzero(np.diff(on.astype(np.int8)) == 1)
        print('             mag_yaw 由关->开 %d 次' % len(tr))

print('\n分段（5 段）门位时间线：')
for k in range(5):
    s, e = k * N // 5, (k + 1) * N // 5
    row = '  %5.1f-%5.1fs ' % (dt[s], dt[e - 1])
    for bit, nm in BITS:
        row += '%s%3.0f%% ' % (nm[:4], 100.0 * ((gb[s:e] & bit) != 0).mean())
    print(row + ' |g|p50 %6.1f' % np.median(g[s:e]))

print('\n关键判定：')
al = (gb & 0x80) != 0
print('  aligned 帧数 %d / %d  (%.2f%%)' % (al.sum(), N, 100.0 * al.mean()))
if al.any():
    print('  首次 aligned @ %.2f s   其后 mag_yaw 开启占比 %.2f%%'
          % (dt[al][0], 100.0 * ((gb[al] & 0x40) != 0).mean()))
    print('  aligned 之前 mag_yaw 占比 %.2f%%' % (100.0 * ((gb[~al] & 0x40) != 0).mean()))
print('  gate_bits 出现的全部取值：', np.unique(gb.astype(np.int32))[:20])
