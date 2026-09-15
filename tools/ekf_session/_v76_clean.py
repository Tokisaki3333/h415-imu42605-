# -*- coding: utf-8 -*-
# 只在"加计=重力"的干净帧上看收敛性与新息
import numpy as np
P = r'R:\raw_v9.bin'; NCH = 144
r = np.fromfile(P, dtype='<f4'); N = r.size // NCH
b = r[:N*NCH].reshape(-1, NCH).astype(np.float64); t = np.cumsum(b[:,25]*1e-6)
B0X, B0Y = -0.0567804, 0.4296474
good = ((np.abs(b[:,132]-B0X) < 5e-3) & (np.abs(b[:,133]-B0Y) < 5e-3)
        & (b[:,118] > 0.0) & (b[:,119] >= 0) & ((b[:,120] == 0) | (b[:,120] == 1)))
acc = b[:,32:35]; an = np.linalg.norm(acc, axis=1)
dev = np.abs(an - 1.0)
for tol in (0.005, 0.01, 0.02, 0.03):
    q = good & (dev < tol)
    print('加计门 |a|-1 < %.3f g : 可用帧 %6.2f%%   |新息| p50 %6.2f p90 %7.2f 度'
          % (tol, 100*q.mean(), np.median(b[q,119]), np.percentile(b[q,119],90)))
print()

# 收敛性：只在新息更新处，且两端加计都干净
ch = np.where(np.diff(b[:,134]) != 0)[0] + 1
keep = good & (dev < 0.01)
ch = ch[keep[ch] & keep[np.maximum(ch-1,0)]]
i0, i1 = ch[:-1], ch[1:]
dt = t[i1]-t[i0]; k = (dt > 1e-4) & (dt < 2e-2)
i0, i1 = i0[k], i1[k]
dr = b[i1,119] - b[i0,119]
print('=== 干净帧上的收敛性（相邻磁更新间 |新息| 变化）===')
print('  n=%d  d|新息| p50 %+.4f 度 ; 缩小占 %.1f%% ; |新息| p50 %.3f'
      % (len(dr), np.median(dr), 100*(dr < 0).mean(), np.median(b[i0,119])))
print('  施加的 dqz 与 |新息| 的关系: |dqz| p50 %.4f 度' % np.median(np.abs(b[i0,137])))
print()
print('  时段          |新息|p50    d|新息|p50    缩小%%   |dqz|p50   used%%')
for lo in range(0, int(t[-1]), 4):
    q = (t[i1] >= lo) & (t[i1] < lo+4)
    if q.sum() < 100: continue
    print('  t %2d~%2d %12.3f %13.4f %8.1f %10.4f %8.0f'
          % (lo, lo+4, np.median(b[i0[q],119]), np.median(dr[q]), 100*(dr[q] < 0).mean(),
             np.median(np.abs(b[i0[q],137])), 100*q.mean()))
print()
print('=== 全程 静止段(|w|<1) 的航向精度 ===')
st = good & (dev < 0.01) & (np.linalg.norm(b[:,26:29],axis=1) < 1.0)
print('  静止且加计干净: %d 帧 (%.1f%%)  |新息| p50 %.3f p90 %.3f max %.3f 度'
      % (st.sum(), 100*st.mean(), np.median(b[st,119]), np.percentile(b[st,119],90), b[st,119].max()))
