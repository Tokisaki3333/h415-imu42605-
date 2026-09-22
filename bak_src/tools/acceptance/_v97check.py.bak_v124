# -*- coding: utf-8 -*-
"""VER=97 校验：① 帧 tag/校验和 ② 削顶标志 bit12~15 与原始 LSB 逐帧互检
   ③ 削顶段统计（磕碰） ④ 磁样本跳变帧的真实年龄 ⑤ 启动基线"""
import numpy as np, os
np.seterr(all='ignore')
P = r'R:\raw_v9.bin'
sz = os.path.getsize(P)
NCH = 162 if (sz//4) % 162 == 0 else (159 if (sz//4) % 159 == 0 else 154)
raw = np.fromfile(P, dtype='<f4'); N = raw.size//NCH
b = raw[:N*NCH].reshape(N, NCH).astype(np.float64)
u8 = raw[:N*NCH].reshape(N, NCH).view(np.uint8)
dt = b[:, 25].copy(); bad = ~np.isfinite(dt) | (dt < 10) | (dt > 50000); dt[bad] = 124.58
t = np.cumsum(dt)*1e-6
tag = int(np.median(b[np.isfinite(b).all(1), 76]))
print('%s  %d B  NCH=%d  帧 %d  %.2f s' % (os.path.basename(P), sz, NCH, N, t[-1]))
print('  帧 tag = 0x%06X -> VER %d CH %d flags %d' % (tag, tag >> 16, (tag >> 8) & 0xFF, tag & 0xFF))
if NCH == 162:
    xk = np.bitwise_xor.reduce(u8[:, :161*4], axis=1).astype(np.uint16) ^ np.uint16(0x5A5A)
    ok = (np.nan_to_num(b[:, 161]) == xk) & np.isfinite(b).all(1)
    print('  ① XOR 校验: 通过 %d/%d (坏 %.3f%%)' % (ok.sum(), N, 100*(~ok).mean()))
    L = b[:, 18:21]
    cx = (np.abs(L[:, 0]) >= 32700); cy = (np.abs(L[:, 1]) >= 32700); cz = (np.abs(L[:, 2]) >= 32700)
    f16 = np.nan_to_num(b[:, 16]).astype(np.int64)
    bx = (f16 >> 13) & 1; by = (f16 >> 14) & 1; bz = (f16 >> 15) & 1; ba = (f16 >> 12) & 1
    print('\n  ② 削顶标志互检 (只用校验通过的帧):')
    for nm, bit, rawm in (('X', bx, cx), ('Y', by, cy), ('Z', bz, cz)):
        m = ok
        print('     %s: 标志置位 %6d, 原始判据 %6d, 不一致 %d' % (nm, (bit[m] == 1).sum(), rawm[m].sum(), (bit[m] != rawm[m]).sum()))
    anyr = cx | cy | cz
    print('     any: 标志 %6d, 原始 %6d, 不一致 %d' % ((ba[ok] == 1).sum(), anyr[ok].sum(), (ba[ok] != anyr[ok]).sum()))
    print('  ③ 削顶统计: 帧 %d (%.3f%%), 轴计数 X%d Y%d Z%d' %
          (anyr.sum(), 100*anyr.mean(), cx.sum(), cy.sum(), cz.sum()))
    idx = np.where(anyr)[0]
    if len(idx):
        seg = np.split(idx, np.where(np.diff(idx) > 1)[0]+1)
        ln = np.array([len(s) for s in seg])
        print('     连续段 %d 个, 长度 中位 %d p90 %d max %d (=%.1f ms), 时间 %.2f~%.2f s'
              % (len(seg), np.median(ln), np.percentile(ln, 90), ln.max(), ln.max()*np.median(dt)*1e-3,
                 t[idx.min()], t[idx.max()]))
        big = [(round(t[s[0]], 2), len(s), int(np.abs(L[s]).max())) for s in seg if len(s) >= 20]
        print('     >=20 帧的段 (时刻 s, 帧数, 最大|LSB|):', big[:20])
    ag = ((b[:, 159] - b[:, 160]) % (1 << 24))
    f = b[:, 42:45]
    newmag = np.zeros(N, bool); newmag[1:] = (np.abs(np.diff(f, axis=0)).sum(1) > 1e-9)
    sm = newmag & ok
    print('\n  ④ 磁样本跳变帧真实年龄: 中位 %.2f us  p90 %.2f us  (样本数 %d, 间隔中位 %.2f ms)'
          % (np.median(ag[sm]), np.percentile(ag[sm], 90), sm.sum(),
             np.median(np.diff(np.where(sm)[0]))*np.median(dt)*1e-3))
    q = b[:, 0:4]; q = q/np.linalg.norm(q, axis=1, keepdims=True)
    w_, x_, y_, z_ = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
    UP = np.stack([2*(x_*z_-w_*y_), 2*(y_*z_+w_*x_), 1-2*(x_*x_+y_*y_)], 1)
    fn = np.linalg.norm(f, axis=1)
    dipU = np.degrees(np.arcsin(np.clip(np.einsum('ni,ni->n', f, UP)/np.maximum(fn, 1e-9), -1, 1)))
    acc = b[:, 32:35]; an = np.linalg.norm(acc, axis=1)
    av = acc/np.maximum(an, 1e-9)[:, None]
    dipA = np.degrees(np.arcsin(np.clip(np.einsum('ni,ni->n', f, av)/np.maximum(fn, 1e-9), -1, 1)))
    gy = np.linalg.norm(b[:, 26:29], axis=1)
    S0 = ok & (t < 3.0) & (gy < 1.0)
    print('\n  ⑤ 启动 0-3 s: dip(旧链UP) %+.2f  dip(加速度) %+.2f  |f| %.3f  n %.3f  |acc| %.4f  |gy| %.2f'
          % (np.median(dipU[S0]), np.median(dipA[S0]), np.median(fn[S0]), np.median(b[S0, 45]),
             np.median(an[S0]), np.median(gy[S0])))
    print('     UP^acc 中位 %.2f deg ; dip 四分位(加速度) [%+.2f, %+.2f]'
          % (np.median(np.degrees(np.arccos(np.clip(np.einsum('ni,ni->n', UP, av), -1, 1)))[S0]),
             np.percentile(dipA[S0], 25), np.percentile(dipA[S0], 75)))
