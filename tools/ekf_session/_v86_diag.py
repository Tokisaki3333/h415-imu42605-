# -*- coding: utf-8 -*-
# VER=86 采集分析：平动漂移 / 旋转抖动 / 180 度后误差；旧姿态为参考
import numpy as np
P = r'R:\raw_v9.bin'; NCH = 154
raw = np.fromfile(P, dtype='<f4'); N = raw.size // NCH
b = raw[:N*NCH].reshape(-1, NCH).astype(np.float64)
t = np.cumsum(b[:, 25]*1e-6); D = 180/np.pi
print('fw_tag %d (VER=%d)  NCH=%d  帧 %d  %.2f s' % (b[0, 76], int(b[0, 76]) >> 16, NCH, N, t[-1]))


def qn(q):
    n = np.linalg.norm(q, axis=-1, keepdims=True); n[n < 1e-9] = 1
    return q/n


def qc(a): return np.stack([a[..., 0], -a[..., 1], -a[..., 2], -a[..., 3]], -1)


def qm(a, c):
    w1, x1, y1, z1 = a[..., 0], a[..., 1], a[..., 2], a[..., 3]
    w2, x2, y2, z2 = c[..., 0], c[..., 1], c[..., 2], c[..., 3]
    return np.stack([w1*w2-x1*x2-y1*y2-z1*z2, w1*x2+x1*w2+y1*z2-z1*y2,
                     w1*y2-x1*z2+y1*w2+z1*x2, w1*z2+x1*y2-y1*x2+z1*w2], -1)


def rv(q):
    q = qn(q)
    if q[0] < 0: q = -q
    s = np.linalg.norm(q[1:4])
    if s < 1e-12: return np.zeros(3)
    return q[1:4]*(2*np.arctan2(s, min(1.0, q[0]))/s)*D


qE = qn(b[:, 89:93]); qL = qn(b[:, 0:4])
inn = (b[:, 144]-b[:, 145]+180) % 360-180
stat = (b[:, 8] == 1)
dqq = qm(qc(qL[:-1]), qL[1:])
om = np.concatenate([[0.0], 2*np.degrees(np.linalg.norm(dqq[:, 1:], axis=1))/(b[1:, 25]*1e-6)])
yawE = np.unwrap(np.arctan2(2*(qE[:, 0]*qE[:, 3]+qE[:, 1]*qE[:, 2]), 1-2*(qE[:, 2]**2+qE[:, 3]**2)))*D
yawL = np.unwrap(np.arctan2(2*(qL[:, 0]*qL[:, 3]+qL[:, 1]*qL[:, 2]), 1-2*(qL[:, 2]**2+qL[:, 3]**2)))*D

edges = [0]+list(np.where(np.diff(stat.astype(np.int8)) != 0)[0]+1)+[N]
seg = [(edges[i], edges[i+1]-1, bool(stat[edges[i]])) for i in range(len(edges)-1)]
seg = [s for s in seg if s[1]-s[0] >= 40]
print()
print('  #   起~止       时长  |w|p50 |w|max  dYaw旧  dYawEKF  sig_yaw sig_tilt  |新息|p50  Σ|dqz|  mag_hold%')
for k, (a, z, s) in enumerate(seg):
    m = slice(a, z+1)
    print('  %2d %6.2f~%6.2f %5.2f %7.1f %7.0f %8.2f %8.2f %8.2f %8.2f %9.3f %7.2f %8.1f'
          % (k, t[a], t[z], t[z]-t[a], np.median(om[m]), om[m].max(),
             yawL[z]-yawL[a], yawE[z]-yawE[a], np.median(b[m, 104]), np.median(b[m, 107]),
             np.median(np.abs(inn[m])), np.nansum(np.abs(b[m, 137])), 100*np.mean(b[m, 151] > 0)))

# 抖动：静止段与运动段的偏航二阶差分
print()
print('=== 抖动（偏航二阶差分 std, 度/帧^2）===')
for k, (a, z, s) in enumerate(seg):
    if z-a < 400: continue
    m = slice(a+10, z-10)
    d2E = np.diff(yawE[m], 2); d2L = np.diff(yawL[m], 2)
    print('  seg#%-2d %-4s |w|p50 %7.1f : EKF %.6f   旧姿态 %.6f   比 %.1f'
          % (k, '静' if s else '动', np.median(om[m]), np.std(d2E), np.std(d2L),
             np.std(d2E)/max(1e-12, np.std(d2L))))

# 锚点：所有静止段相对第一个
print()
print('=== 锚点（物理上一组相同朝向的静止段）===')
anch = [(k, a, z) for k, (a, z, s) in enumerate(seg) if s and (z-a) > 800]
if len(anch) >= 2:
    def core(a, z):
        c = (a+z)//2; h = max(50, (z-a)//4); return slice(c-h, c+h)
    a0, z0 = anch[0][1], anch[0][2]
    qE0 = np.median(qE[core(a0, z0)], axis=0); qL0 = np.median(qL[core(a0, z0)], axis=0)
    print('  seg#  时刻   | EKF偏航 EKF倾斜 | 旧姿态偏航 旧姿态倾斜 | sig_yaw sig_tilt')
    for k, a, z in anch:
        c = core(a, z)
        rE = rv(qm(np.median(qE[c], axis=0), qc(qE0)))
        rL = rv(qm(np.median(qL[c], axis=0), qc(qL0)))
        print('  #%-3d %6.2f | %8.3f %8.3f | %10.3f %10.3f | %7.2f %8.2f'
              % (k, t[a], rE[2], np.hypot(rE[0], rE[1]), rL[2], np.hypot(rL[0], rL[1]),
                 np.median(b[core(a, z), 104]), np.median(b[core(a, z), 107])))
print()
print('=== 旧姿态 vs EKF 的偏航差（dOffset）时间轨迹（每 1s）===')
off = yawE - yawL
for lo in range(0, int(t[-1]), 1):
    m = (t >= lo) & (t < lo+1)
    if m.sum() < 100: continue
    print('  t=%4d  dOffset %+8.3f 度  |新息|p50 %7.3f  sig_yaw %7.2f  Σ|dqz| %7.3f'
          % (lo, np.median(off[m])-np.median(off[t < 0.5]), np.median(np.abs(inn[m])),
             np.median(b[m, 104]), np.nansum(np.abs(b[m, 137]))))
