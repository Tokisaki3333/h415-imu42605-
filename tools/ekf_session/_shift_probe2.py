# -*- coding: utf-8 -*-
# 定出上报错位的精确范围与来源
import numpy as np
P = r'R:\raw_v9.bin'; NCH = 144
r = np.fromfile(P, dtype='<f4'); N = r.size // NCH
b = r[:N*NCH].reshape(-1, NCH).astype(np.float64); t = np.cumsum(b[:,25]*1e-6)
qn = np.linalg.norm(b[:, 89:93], axis=1)
badq = np.abs(qn - 1.0) > 0.01
print('|q|≠1 的帧 %d (%.2f%%) ; |q| 分位 p50 %.4f p90 %.2f p99 %.2f max %.0f'
      % (badq.sum(), 100*badq.mean(), np.median(qn), np.percentile(qn,90),
         np.percentile(qn,99), qn.max()))
idx = np.where(badq)[0]
print('样例 |q|: %s' % np.round(qn[idx[:10]], 3))

# 用坏帧与前一帧逐列比对，找出被位移的列段
print()
print('=== 逐列判定: 本帧是否等于上一帧的 col+11 (或其它平移) ===')
for k in idx[:3]:
    if k == 0: continue
    out = []
    for c in range(NCH):
        hit = None
        for sh in range(1, 20):
            if c + sh < NCH and abs(b[k, c] - b[k-1, c+sh]) < 1e-9:
                hit = sh; break
        out.append(hit)
    seg = [(c, h) for c, h in enumerate(out) if h]
    if seg:
        print(' 帧 %d: 被位移的列 %d~%d, 平移量 %s' % (k, seg[0][0], seg[-1][0],
              sorted(set(h for _, h in seg))))
        print('        含四元数列 89~92 ? %s' % [c for c in (89,90,91,92) if out[c]])
    else:
        print(' 帧 %d: 无列匹配 col+sh (不是简单平移)' % k)

# 统计: 位移帧里各列被位移的比例
print()
print('=== 统计 400 个坏帧, 每列"等于上一帧 col+11"的比例 ===')
cnt = np.zeros(NCH); tot = 0
for k in idx[:400]:
    if k == 0: continue
    tot += 1
    for c in range(NCH-11):
        if abs(b[k, c] - b[k-1, c+11]) < 1e-9:
            cnt[c] += 1
for c in range(0, NCH, 8):
    row = ' '.join('%3d:%3.0f%%' % (cc, 100*cnt[cc]/tot) for cc in range(c, min(c+8, NCH)))
    print('  ' + row)
print()
qcols = [89,90,91,92]
print('四元数列被 col+11 替换的比例: %s' % ['%d:%.0f%%' % (c, 100*cnt[c]/tot) for c in qcols])
print('总坏帧 %d ; 4.06%% 位移帧里有 %d 个四元数坏' % (badq.sum(), (badq & (np.abs(b[:,132]+0.0567804)<5e-3)).sum()))
