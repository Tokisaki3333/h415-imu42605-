# -*- coding: utf-8 -*-
# 运动后偏航漂移归因（列号全部经数据自检确认）
#   0..3   旧路径姿态四元数   (|q|==1 已验)
#   8      is_static          ({0,1} 已验)
#   25     dt (us)            (已验)
#   76     fw_tag = 5411847   (常量已验)
#   89..92 EKF 姿态四元数     (|q|==1 已验)
#   103    gate_bits
#   118    mag_bh             (=147 已验)
#   119    mag_r_deg          (==|144-145| 已验)
#   120    mag_used           ({0,1})
#   126    mag_rej            (恒0 已验)
#   137    mag_dqz (度/更新)  (max==0.015*max|r| 已验)
#   138..140 tilt_dq
#   144/145 thm/thp (度)      (==ac 已验)
#   146    amn (度)  147 mhn
import numpy as np

P = r'R:\raw_v9.bin'; NCH = 148
raw = np.fromfile(P, dtype='<f4'); N = raw.size // NCH
b = raw[:N*NCH].reshape(-1, NCH).astype(np.float64)
dt = b[:, 25]*1e-6; t = np.cumsum(dt)


def rq(q):
    w, x, y, z = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
    n = np.sqrt(w*w+x*x+y*y+z*z); n[n < 1e-9] = 1.0
    w, x, y, z = w/n, x/n, y/n, z/n
    R = np.empty((len(q), 3, 3))
    R[:, 0, 0] = 1-2*(y*y+z*z); R[:, 0, 1] = 2*(x*y-w*z); R[:, 0, 2] = 2*(x*z+w*y)
    R[:, 1, 0] = 2*(x*y+w*z); R[:, 1, 1] = 1-2*(x*x+z*z); R[:, 1, 2] = 2*(y*z-w*x)
    R[:, 2, 0] = 2*(x*z-w*y); R[:, 2, 1] = 2*(y*z+w*x); R[:, 2, 2] = 1-2*(x*x+y*y)
    return R


Rl = rq(b[:, 0:4]); Re = rq(b[:, 89:93])
Rrel = np.einsum('nji,njk->nik', Rl, Re)
D = 180.0/np.pi
yawL = np.unwrap(np.arctan2(Rl[:, 1, 0], Rl[:, 0, 0]))*D
yawE = np.unwrap(np.arctan2(Re[:, 1, 0], Re[:, 0, 0]))*D
off = np.unwrap(np.arctan2(Rrel[:, 1, 0], Rrel[:, 0, 0]))*D
off2 = yawE-yawL
tilt = np.degrees(np.arccos(np.clip(Rrel[:, 2, 2], -1, 1)))

# 机体角速率：由旧姿态差分得到（不依赖 col26..28 的身份）
q = b[:, 0:4].copy(); q /= np.linalg.norm(q, axis=1)[:, None]
qc = q.copy(); qc[:, 1:] *= -1
def qmul(a, c):
    w1, x1, y1, z1 = a[:, 0], a[:, 1], a[:, 2], a[:, 3]
    w2, x2, y2, z2 = c[:, 0], c[:, 1], c[:, 2], c[:, 3]
    return np.stack([w1*w2-x1*x2-y1*y2-z1*z2, w1*x2+x1*w2+y1*z2-z1*y2,
                     w1*y2-x1*z2+y1*w2+z1*x2, w1*z2+x1*y2-y1*x2+z1*w2], 1)
dq = qmul(qc[:-1], q[1:])
om = 2*np.degrees(np.linalg.norm(dq[:, 1:], axis=1))/dt[1:]
om = np.concatenate([[om[0]], om])

inn = (b[:, 144]-b[:, 145]+180) % 360-180
r_abs = b[:, 119]
dqz = b[:, 137]
used = (b[:, 120] == 1)
stat = (b[:, 8] == 1)
TAU = 1.0/(0.015*196.6)

print('fw_tag %d  VER=%d  %.2f s  %d 帧 (%.0f Hz)' % (b[0, 76], int(b[0, 76]) >> 16, t[-1], N, N/t[-1]))
print('mag_dqz 锁存检验: 与上一帧相同的比例 %.2f%%  -> 每次更新重复约 %.1f 帧'
      % (100*np.mean(dqz[1:] == dqz[:-1]), 1/max(1e-9, 1-np.mean(dqz[1:] == dqz[:-1]))))
nz = dqz != 0.0
print('used=1 帧 %.1f%% ; dqz!=0 帧 %.2f%% (=%.1f Hz) ; used&nz %.2f%%'%(100*used.mean(),100*nz.mean(),nz.sum()/t[-1],100*(used&nz).mean()))
print('dqz 非零帧上 used=1 的比例 %.1f%%'%(100*(used[nz].mean() if nz.sum() else 0)))
chg = np.concatenate([[True], dqz[1:] != dqz[:-1]])
print('实际更新次数 %d = %.1f Hz ; 模型 tau=%.3f s' % (chg.sum(), chg.sum()/t[-1], TAU))
print('-> 之前用"逐帧求和"把 dqz 放大了 %.1f 倍，那个 241 度是假的' % (1/max(1e-9, 1-np.mean(dqz[1:] == dqz[:-1]))))


def osum(x, mask=None):
    m = chg & ((used) if mask is None else mask)
    return np.nansum(x[m])


edges = [0]+list(np.where(np.diff(stat.astype(np.int8)) != 0)[0]+1)+[N]
seg = [(edges[i], edges[i+1]-1, bool(stat[edges[i]])) for i in range(len(edges)-1)]
seg = [s for s in seg if s[1]-s[0] >= 30]
print()
print('=== 逐段：偏航归属（度） ===')
print('  #   起(s)   止(s) 性质  |w|p50 |w|max | dYaw_旧  dYaw_EKF   dOffset | 磁注入Sdqz  磁模型S(r/tau)dt | used% |r|p50 |r|max')
for k, (a, z, s) in enumerate(seg):
    m = slice(a, z+1)
    dL = yawL[z]-yawL[a]; dE = yawE[z]-yawE[a]; dO = off[z]-off[a]
    magsum = osum(dqz, np.zeros(N, bool)) if False else np.nansum(dqz[chg & (np.arange(N) >= a) & (np.arange(N) <= z)])
    pred = np.trapezoid(np.radians(inn[m]), t[m])*D/TAU
    print('  %2d %6.2f %6.2f %s %7.1f %7.1f | %8.2f %8.2f %8.2f | %8.3f %10.3f | %5.0f %7.3f %8.3f'
          % (k, t[a], t[z], '静' if s else '动', np.median(om[m]), om[m].max(),
             dL, dE, dO, magsum, pred, 100*used[m].mean(),
             np.median(np.abs(inn[m])), np.abs(inn[m]).max()))
print()
print('  说明: dYaw_旧/dYaw_EKF = 各自姿态流在同一段内的偏航变化; dOffset = EKF-旧(应=dE-dL)')
print('        磁注入Sdqz = 环路自己记账的偏航修正总量(只在真更新处累加, 度)')

print()
print('=== 运动段：磁环当时在干什么 ===')
for k, (a, z, s) in enumerate(seg):
    if s or z-a < 100:
        continue
    m = slice(a, z+1)
    ai = np.abs(inn[m])
    pred = np.trapezoid(np.radians(inn[m]), t[m])*D/TAU
    print('  运动#%-2d t=%5.2f~%5.2f (%4.1fs) |w|p50 %6.1f p95 %7.1f max %8.0f dps' %
          (k, t[a], t[z], t[z]-t[a], np.median(om[m]), np.percentile(om[m], 95), om[m].max()))
    print('        |r| p50 %7.3f p95 %7.2f max %8.2f 度 | 磁模型注入 %+9.3f 度 | used %3.0f%% | 拒绝 %d 帧'
          % (np.median(ai), np.percentile(ai, 95), ai.max(), pred, 100*used[m].mean(),
             int((b[m, 126] > 0).sum())))
    print('        amn p50 %6.2f p95 %7.2f 度 | corr(|r|,amn)=%.3f  corr(|r|,|w|)=%.3f'
          % (np.median(b[m, 146]), np.percentile(b[m, 146], 95),
             np.corrcoef(ai, b[m, 146])[0, 1], np.corrcoef(ai, om[m])[0, 1]))
    print('        gate_bits p50 %d ; mag_bh p50 %.4f ; mhn p50 %.4f'
          % (np.median(b[m, 103]), np.median(b[m, 118]), np.median(b[m, 147])))

print()
print('=== 运动后静止段：settle 轨迹 (0.1 s 分辨率) ===')
for k in range(1, len(seg)-1):
    a, z, s = seg[k]
    if not s or z-a < 300:
        continue
    pa, pz, ps = seg[k-1]
    if ps:
        continue
    print('  运动#%d (%.2f~%.2f s, |w|p50 %.1f) -> 静止#%d' % (k-1, t[pa], t[pz], np.median(om[pa:pz+1]), k))
    seg_t = t[a:z+1]-t[a]
    take = np.arange(0, min(3.0, seg_t[-1]), 0.1)
    line = []
    for tt in take:
        i = a+int(np.searchsorted(seg_t, tt))
        i = min(i, z)
        line.append('%5.2fs off%+8.3f r%7.3f' % (tt, off[i]-off[a], inn[i]))
    for j in range(0, len(line), 3):
        print('     ' + '  |  '.join(line[j:j+3]))
