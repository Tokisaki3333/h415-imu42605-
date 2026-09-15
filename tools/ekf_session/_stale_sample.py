# -*- coding: utf-8 -*-
# 磁实测航向 thm 在两次采样之间是否"完全重复"(陈旧/重复样本)，以及此时姿态已经转了多少
import numpy as np
P = r'R:\raw_v9.bin'; NCH = 148
raw = np.fromfile(P, dtype='<f4'); N = raw.size // NCH
b = raw[:N*NCH].reshape(-1, NCH).astype(np.float64)
dt = b[:, 25]*1e-6; t = np.cumsum(dt)
D = 180/np.pi
yawE = np.unwrap(np.arctan2(2*(b[:, 89]*b[:, 92]+b[:, 90]*b[:, 91]),
                            1-2*(b[:, 91]**2+b[:, 92]**2)))*D
yawL = np.unwrap(np.arctan2(2*(b[:, 0]*b[:, 3]+b[:, 1]*b[:, 2]),
                            1-2*(b[:, 2]**2+b[:, 3]**2)))*D
inn = (b[:, 144]-b[:, 145]+180) % 360-180
thm = b[:, 144]
ic = b[:, 41]
idx = np.where(np.concatenate([[True], ic[1:] != ic[:-1]]))[0]

same = thm[idx[1:]] == thm[idx[:-1]]
dpsi = np.abs(yawE[idx[1:]]-yawE[idx[:-1]])
print('磁采样点 %d 个 ; thm 与上一采样点逐位相同的比例 %.2f%%' % (len(idx), 100*same.mean()))
print('  其中姿态在同一间隔内转过的角度: p50 %.4f p90 %.4f p99 %.3f max %.2f 度'
      % (np.percentile(dpsi[same], 50), np.percentile(dpsi[same], 90),
         np.percentile(dpsi[same], 99), dpsi[same].max()))
print('  重复样本里姿态转过 >1 度的有 %d 个 (%.2f%%)'
      % ((dpsi[same] > 1).sum(), 100*(dpsi[same] > 1).mean()))
print()
bad = same & (dpsi > 1)
print('=== 静止 vs 运动：重复率 ===')
st = (b[:, 8] == 1)[idx[:-1]]
for nm, msk in (('静止', st), ('运动', ~st)):
    print('  %s段: 重复率 %.2f%% ; 重复且姿态转过>1度的占该段采样 %.2f%%'
          % (nm, 100*same[msk].mean(), 100*bad[msk].mean()))
print()
print('=== 大新息(>10度)的采样点里，thm 是重复样本的比例 ===')
big = np.abs(inn[idx[:-1]]) > 10
print('  |r|>10 度的采样 %d 个 ; 其中 thm 重复的 %.1f%% ; 上一样本->本样本姿态转角 p50 %.2f 度'
      % (big.sum(), 100*same[big].mean(), np.percentile(dpsi[big], 50) if big.sum() else 0))
print()
print('=== 逐段：重复率与最大新息 ===')
edges = [0]+list(np.where(np.diff(st.astype(np.int8)) != 0)[0]+1)+[N]
seg = [(edges[i], edges[i+1]-1, bool(st[edges[i]])) for i in range(len(edges)-1)]
print('  #  起(s)  止(s) 性质  样本数  thm重复率  重复且姿态转角>1度 样本占比  |r|max')
for k, (a, z, s) in enumerate(seg):
    m = (idx[:-1] >= a) & (idx[:-1] <= z)
    if m.sum() < 5:
        continue
    print('  %2d %6.2f %6.2f %s %6d %9.2f%% %20.2f%% %9.2f'
          % (k, t[a], t[z], '静' if s else '动', m.sum(), 100*same[m].mean(),
             100*bad[m].mean(), np.abs(b[a:z+1, 144]-b[a:z+1, 145]).max()))
