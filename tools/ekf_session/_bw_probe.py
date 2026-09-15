# -*- coding: utf-8 -*-
# 决定性证据：cols46-48 = 导航系磁场 Bw = R(旧姿态q) * mag.f，在**磁采样时刻**算出来。
# 若磁样本与所用姿态在时间上对齐，Bw 应恒等于 b0(常数)。任何偏离都是"磁样本 vs 姿态"的错位。
#   dev(t) = angle(Bw(t), Bw_ref)      <- 与 EKF 完全无关的独立量
#   等效年龄 age ~ dev / |w|           <- 直接给出"旧了多少"
import numpy as np
P = r'R:\raw_v9.bin'; NCH = 148
raw = np.fromfile(P, dtype='<f4'); N = raw.size // NCH
b = raw[:N*NCH].reshape(-1, NCH).astype(np.float64)
dt = b[:, 25]*1e-6; t = np.cumsum(dt)
Bw = b[:, 46:49].copy()
n = np.linalg.norm(Bw, axis=1)
print('cols46-48: |Bw| p50 %.6f min %.6f max %.6f  (=1 说明是单位矢量)' % (np.median(n), n.min(), n.max()))
Bw = Bw/np.maximum(n, 1e-9)[:, None]
ref = np.median(Bw, axis=0); ref /= np.linalg.norm(ref)
dot = np.clip(Bw@ref, -1, 1)
dev = np.degrees(np.arccos(dot))
# 旧姿态角速率
q = b[:, 0:4].copy(); q /= np.linalg.norm(q, axis=1)[:, None]
qc = q.copy(); qc[:, 1:] *= -1
def qmul(a, c):
    w1, x1, y1, z1 = a[:, 0], a[:, 1], a[:, 2], a[:, 3]
    w2, x2, y2, z2 = c[:, 0], c[:, 1], c[:, 2], c[:, 3]
    return np.stack([w1*w2-x1*x2-y1*y2-z1*z2, w1*x2+x1*w2+y1*z2-z1*y2,
                     w1*y2-x1*z2+y1*w2+z1*x2, w1*z2+x1*y2-y1*x2+z1*w2], 1)
dqq = qmul(qc[:-1], q[1:])
om = np.concatenate([[0.0], 2*np.degrees(np.linalg.norm(dqq[:, 1:], axis=1))/dt[1:]])
st = (b[:, 8] == 1)
print()
print('=== Bw 相对自身中位数的偏离角 dev (度) —— 与 EKF 无关 ===')
print('  静止帧: p50 %.3f  p95 %.3f  p99 %.3f  max %.2f'
      % (np.median(dev[st]), np.percentile(dev[st], 95), np.percentile(dev[st], 99), dev[st].max()))
print('  运动帧: p50 %.3f  p95 %.3f  p99 %.3f  max %.2f'
      % (np.median(dev[~st]), np.percentile(dev[~st], 95), np.percentile(dev[~st], 99), dev[~st].max()))
print()
print('=== 等效年龄 = dev / |w| （只在高转速时才有意义）===')
hi = (~st) & (om > 50)
ok = hi & (dev > 0.2)
age = dev[ok]/om[ok]
print('  |w|>50dps 的帧 %d 个 ; 其中 dev>0.2 度的 %d 个' % (hi.sum(), ok.sum()))
if ok.sum():
    print('  等效年龄(ms): p10 %.2f p50 %.2f p90 %.2f p99 %.2f max %.2f'
          % tuple(np.percentile(age, [10, 50, 90, 99, 100])*1000))
print()
print('=== 逐段 ===')
edges = [0]+list(np.where(np.diff(st.astype(np.int8)) != 0)[0]+1)+[N]
print('  #  起(s)  止(s) 性质  |w|p50  dev_p50  dev_p95  dev_max  等效年龄p50(ms)')
for k in range(len(edges)-1):
    a, z = edges[k], edges[k+1]-1
    if z-a < 60:
        continue
    s = bool(st[a]); m = slice(a, z+1)
    mm = (~st[a:z+1]) & (om[a:z+1] > 50) & (dev[a:z+1] > 0.2)
    ag = np.median(dev[a:z+1][mm]/om[a:z+1][mm])*1000 if mm.sum() > 5 else float('nan')
    print('  %2d %6.2f %6.2f %s %7.1f %8.3f %8.3f %8.2f %14.2f'
          % (k, t[a], t[z], '静' if s else '动', np.median(om[m]), np.median(dev[m]),
             np.percentile(dev[m], 95), dev[m].max(), ag))
print()
print('=== 段11 (12.02~13.00) 逐 0.05s：dev 与转速 ===')
a, z = edges[11], edges[12]-1
for tt in np.arange(0, t[z]-t[a], 0.05):
    i = a+int(np.searchsorted(t[a:z+1]-t[a], tt))
    if i >= z: break
    print('   t=%6.2f  |w| %8.1f dps   dev %8.3f 度   等效年龄 %7.2f ms'
          % (t[i], om[i], dev[i], 1000*dev[i]/max(om[i], 1e-6)))
