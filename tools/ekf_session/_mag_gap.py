# -*- coding: utf-8 -*-
# 磁环把偏航往回拽(而不是往前) -> 说明"磁场看起来跟着机体转"(陈旧量补偿过度)。
# 直接量测磁采样计数 col41(=ist.hdr.cnt) 的间隔，看运动段是否出现长空档。
import numpy as np
P = r'R:\raw_v9.bin'; NCH = 148
raw = np.fromfile(P, dtype='<f4'); N = raw.size // NCH
b = raw[:N*NCH].reshape(-1, NCH).astype(np.float64)
dt = b[:, 25]*1e-6; t = np.cumsum(dt)

q = b[:, 0:4].copy(); q /= np.linalg.norm(q, axis=1)[:, None]
qc = q.copy(); qc[:, 1:] *= -1
def qmul(a, c):
    w1, x1, y1, z1 = a[:, 0], a[:, 1], a[:, 2], a[:, 3]
    w2, x2, y2, z2 = c[:, 0], c[:, 1], c[:, 2], c[:, 3]
    return np.stack([w1*w2-x1*x2-y1*y2-z1*z2, w1*x2+x1*w2+y1*z2-z1*y2,
                     w1*y2-x1*z2+y1*w2+z1*x2, w1*z2+x1*y2-y1*x2+z1*w2], 1)
dqq = qmul(qc[:-1], q[1:])
om = np.concatenate([[0.0], 2*np.degrees(np.linalg.norm(dqq[:, 1:], axis=1))/dt[1:]])
# 绕竖直轴的累计旋转（度），用旧姿态的偏航差
D = 180/np.pi
yawL = np.unwrap(np.arctan2(
    2*(b[:, 0]*b[:, 3]+b[:, 1]*b[:, 2]), 1-2*(b[:, 2]**2+b[:, 3]**2)))*D

ic = b[:, 41]
chg = np.concatenate([[True], ic[1:] != ic[:-1]])
print('col41 唯一值 %d ; 计到 %.1f Hz ; 帧率 %.0f Hz -> 每次采样约 %.1f 帧'
      % (len(np.unique(ic)), chg.sum()/t[-1], N/t[-1], N/max(1, chg.sum())))
idx = np.where(chg)[0]
gap = np.diff(idx)
print('采样间隔(帧): p50 %.0f p90 %.0f p99 %.0f max %d  (=%.1f ms max)'
      % (np.percentile(gap, 50), np.percentile(gap, 90), np.percentile(gap, 99),
         gap.max(), gap.max()*np.median(dt)*1e3))
print('间隔 > 3*p50 的次数 %d ; > 10*p50 的次数 %d' % ((gap > 3*np.percentile(gap, 50)).sum(),
                                                      (gap > 10*np.percentile(gap, 50)).sum()))
print()
print('=== 每个采样间隔内旧姿态绕竖直轴的转角 (度) ===')
mid = idx[:-1]+gap//2
ang = np.abs(np.diff(yawL[idx]))
print('  一次采样间隔内的偏航转角: p50 %.3f p90 %.3f p99 %.3f max %.2f 度'
      % (np.percentile(ang, 50), np.percentile(ang, 90), np.percentile(ang, 99), ang.max()))
print('  间隔内转角 > 5 度的次数 %d (%d%% 的采样)'
      % ((ang > 5).sum(), round(100*(ang > 5).mean())))
print()
print('=== 逐段：采样间隔与采样间隔内的转角 ===')
st = (b[:, 8] == 1)
edges = [0]+list(np.where(np.diff(st.astype(np.int8)) != 0)[0]+1)+[N]
seg = [(edges[i], edges[i+1]-1, bool(st[edges[i]])) for i in range(len(edges)-1)]
print('  #  起(s)  止(s) 性质  采样率  间隔p50 间隔max 间隔内转角p50 p95 max')
for k, (a, z, s) in enumerate(seg):
    m = (idx >= a) & (idx <= z)
    if m.sum() < 5:
        continue
    ii = idx[m]
    gp = np.diff(ii) if len(ii) > 1 else np.array([1])
    an = np.abs(np.diff(yawL[ii])) if len(ii) > 1 else np.array([0.0])
    print('  %2d %6.2f %6.2f %s %7.1f %8.0f %7d %11.3f %6.3f %6.2f'
          % (k, t[a], t[z], '静' if s else '动', len(ii)/(t[z]-t[a]+1e-9),
             np.median(gp), gp.max(), np.median(an), np.percentile(an, 95), an.max()))
print()
print('=== 最大 10 个采样空档 ===')
o = np.argsort(gap)[::-1][:10]
for j in sorted(o):
    print('   t=%7.3f s  空档 %5d 帧 (%6.2f ms)  空档内旧姿态偏航转了 %8.3f 度'
          % (t[idx[j]], gap[j], gap[j]*np.median(dt)*1e3, abs(yawL[idx[j+1]]-yawL[idx[j]])))
