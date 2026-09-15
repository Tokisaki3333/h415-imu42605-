# -*- coding: utf-8 -*-
# 损坏帧的异常值来自哪一帧的哪一列？
import numpy as np
P = r'R:\raw_v9.bin'; NCH = 144
r = np.fromfile(P, dtype='<f4'); N = r.size // NCH
b = r[:N*NCH].reshape(-1, NCH).astype(np.float64)

bad = np.abs(b[:, 121] + 0.056781) < 1e-6
i = np.where(bad)[0]
k = int(i[0])
print('损坏帧 idx %d' % k)
lbl = {117:'mag_gate',118:'mag_bh',119:'mag_r',120:'mag_used',121:'p_yy',
       130:'vx',131:'vy',132:'v0x',133:'v0y',134:'yawpre',137:'dqz',
       104:'sig_yaw',113:'pzz',45:'mag_norm'}
print()
print('在 k-1 / k+1 / k-2 帧中查找该帧异常值的来源列:')
for c in sorted(lbl):
    v = b[k, c]
    hits = []
    for kk in (k-1, k+1, k-2, k+2):
        for cc in range(NCH):
            if abs(b[kk, cc] - v) < 1e-6:
                hits.append((kk-k, cc))
    print('  col %3d %-9s = %12.6f  -> 出现在 %s' % (c, lbl[c], v, hits if hits else '本帧独有'))

print()
print('字节级测试: 损坏帧是否为好帧平移 1~3 字节 (丢/多字节)')
ref = b[k-1]
raw_ok = np.fromfile(P, dtype='<f4', count=NCH, offset=(k)*NCH*4)
for sh in (0, 1, 2, 3, -1, -2, -3):
    raw_k = np.fromfile(P, dtype='<u1', count=NCH*4, offset=k*NCH*4)
    raw_r = np.fromfile(P, dtype='<u1', count=NCH*4, offset=(k-1)*NCH*4)
    a = raw_k if sh >= 0 else raw_r
    m = min(len(a)-abs(sh), NCH*4)
    s = a[abs(sh):abs(sh)+m]
    # 逐字节比较
    r2 = raw_r[sh:] if sh >= 0 else raw_r[:sh]
    n = min(len(s), len(r2))
    same = (s[:n] == r2[:n]).mean()
    print('  字节平移 %+d : 与上一帧逐字节相同比例 %.3f' % (sh, same))
print()
print('同帧内: 与上一帧逐字节相同的比例 %.4f ; 逐 float 相同比例 %.4f'
      % ((b[k].view(np.uint8) if False else 0) if False else
         (np.fromfile(P,dtype='<u1',count=NCH*4,offset=k*NCH*4) ==
          np.fromfile(P,dtype='<u1',count=NCH*4,offset=(k-1)*NCH*4)).mean(),
         (b[k] == b[k-1]).mean()))
