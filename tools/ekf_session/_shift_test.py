# -*- coding: utf-8 -*-
import numpy as np
P = r'R:\raw_v9.bin'; NCH = 144
r = np.fromfile(P, dtype='<f4'); N = r.size // NCH
b = r[:N*NCH].reshape(-1, NCH).astype(np.float64); t = np.cumsum(b[:,25]*1e-6)

bad = np.abs(b[:, 121] + 0.056781) < 1e-6          # p_yy == b0x
print('p_yy(121) == -0.056781 的帧 %d (%.2f%%)' % (bad.sum(), 100*bad.mean()))
i = np.where(bad)[0]
k = int(i[0])
print()
print('用相邻干净帧做参照, 扫平移量:')
sc = []
for sh in range(-30, 31):
    x = np.roll(b[k], sh)
    sc.append((np.median(np.abs(x - b[k-1])), sh))
sc.sort()
for v, s in sc[:8]:
    print('   平移 %+4d  中位差 %.6f' % (s, v))
print('   平移    0  中位差 %.6f' % np.median(np.abs(b[k] - b[k-1])))

print()
prev_same = (b[1:] == b[:-1]).mean(0)
order = np.argsort(prev_same)[:16]
print('逐列 与上一帧完全相同 的比例(最低的16列):')
for c in sorted(order):
    print('   col %3d : %.1f%%' % (c, 100*prev_same[c]))

print()
print('坏帧的成簇性: 相邻帧同为坏的比例 %.1f%%' % (100*(bad[1:] & bad[:-1]).mean()))

# 坏帧里哪些列等于同帧的其它列(找重复块)
print()
print('坏帧内 mag_* 段原值 (idx %d):' % k)
lbl = {117:'mag_gate',118:'mag_bh',119:'mag_r',120:'mag_used',121:'p_yy',134:'yawpre',
       130:'vx',131:'vy',132:'v0x',133:'v0y',137:'dqz',104:'sig_yaw',113:'pzz'}
for c in sorted(lbl):
    print('   col %3d %-9s = %14.6f' % (c, lbl[c], b[k, c]))
print()
print('同一帧里等于 -0.056781 的列: %s' % [c for c in range(NCH) if abs(b[k,c]+0.056781) < 1e-6])
print('同一帧里等于 mag_used 值的列: %s' % [c for c in range(NCH) if abs(b[k,c]-b[k,120]) < 1e-9])
