# -*- coding: utf-8 -*-
# 判断 63~92 度的大新息: 真实的航向误差, 还是 â(加计) 在剧烈运动时不是重力导致测量被污染
import numpy as np
P = r'R:\raw_v9.bin'; NCH = 144
r = np.fromfile(P, dtype='<f4'); N = r.size // NCH
b = r[:N*NCH].reshape(-1, NCH).astype(np.float64); t = np.cumsum(b[:,25]*1e-6)
B0X, B0Y = -0.0567804, 0.4296474
good = ((np.abs(b[:,132]-B0X) < 5e-3) & (np.abs(b[:,133]-B0Y) < 5e-3)
        & (b[:,118] > 0.0) & (b[:,119] >= 0) & ((b[:,120] == 0) | (b[:,120] == 1)))
acc = b[:,32:35]; magf = b[:,42:45]
an = np.linalg.norm(acc, axis=1); fn = np.linalg.norm(magf, axis=1)
ab = acc/np.maximum(an,1e-9)[:,None]; mf = magf/np.maximum(fn,1e-9)[:,None]

# 1) 新息 vs |a|-1（加计偏离 1g 的程度）
D = -7.53
azi0 = np.degrees(np.arctan2(np.cos(np.radians(D)), np.sin(np.radians(D))))
innov = ((b[:,119]) )                       # 固件上报的 |新息|
print('|a|  : p50 %.4f p90 %.4f p99 %.4f max %.2f g' % (
    np.median(an), np.percentile(an,90), np.percentile(an,99), an.max()))
print('||a|-1|: p50 %.4f p90 %.4f p99 %.3f' % (
    np.median(abs(an-1)), np.percentile(abs(an-1),90), np.percentile(abs(an-1),99)))
m = good & (b[:,119] > 0.01)
print()
print('=== 新息大小 vs 加计偏离 1g ===')
for lo, hi in [(0,0.01),(0.01,0.03),(0.03,0.10),(0.10,0.30),(0.30,1.0),(1.0,99)]:
    q = m & (abs(an-1) >= lo) & (abs(an-1) < hi)
    if q.sum() < 100: continue
    print('  ||a|-1| %5.2f~%-5.2f g : n=%6d   |新息| p50 %7.2f p90 %7.2f 度   |w|p50 %7.1f'
          % (lo, hi, q.sum(), np.median(b[q,119]), np.percentile(b[q,119],90),
             np.median(np.linalg.norm(b[q,26:29],axis=1))))
print('  相关系数 corr(||a|-1|, |新息|) = %+.3f'
      % np.corrcoef(abs(an[m]-1), b[m,119])[0,1])

# 2) 用**估计姿态的"上"** 代替 â 复算航向, 与大新息比较
def yaw_of(q):
    w,x,y,z = q[:,0],q[:,1],q[:,2],q[:,3]
    n = np.sqrt(w*w+x*x+y*y+z*z); n[n<1e-9]=1e-9
    return np.arctan2(2*(w/n)*(z/n)+2*(x/n)*(y/n), 1-2*((y/n)**2+(z/n)**2))
qu = b[:,89:93]; qn = np.linalg.norm(qu,axis=1); qu = qu/np.maximum(qn,1e-9)[:,None]
w_,x_,y_,z_ = qu[:,0],qu[:,1],qu[:,2],qu[:,3]
# 估计的机体系"上" = R(q)^T (0,0,1)
up_est = np.stack([2*(x_*z_-w_*y_), 2*(y_*z_+w_*x_), 1-2*(x_*x_+y_*y_)],1)
def heading_about(axis, mfv, xaxis):
    ax = axis/np.maximum(np.linalg.norm(axis,axis=1),1e-9)[:,None]
    mh = mfv - np.sum(mfv*ax,1)[:,None]*ax
    xv = xaxis - np.sum(xaxis*ax,1)[:,None]*ax
    cs = np.cross(ax, mh)
    return np.degrees(np.arctan2(np.sum(cs*xv,1), np.sum(mh*xv,1)))
xax = np.tile([1.0,0,0],(len(b),1))
thm_acc = heading_about(ab, mf, xax)        # 用加计 â（固件做法）
thm_est = heading_about(up_est, mf, xax)    # 用估计姿态的"上"
psih = b[:,134]
r_acc = np.abs((thm_acc - (psih - azi0) + 180) % 360 - 180)
r_est = np.abs((thm_est - (psih - azi0) + 180) % 360 - 180)
print()
print('=== 两种投影轴的对比（|新息| 中位）===')
for lo in range(0, int(t[-1]), 4):
    q = (t >= lo) & (t < lo+4) & good
    if q.sum() < 200: continue
    print('  t %2d~%2d: |w|p50 %7.1f  ||a|-1|p50 %6.3f   用加计â %7.2f 度   用估计上 %7.2f 度'
          % (lo, lo+4, np.median(np.linalg.norm(b[q,26:29],axis=1)),
             np.median(abs(an[q]-1)), np.median(r_acc[q]), np.median(r_est[q])))

# 3) 直接测收敛: 相邻磁更新之间 |新息| 是否在缩小
ch = np.where(np.diff(b[:,134]) != 0)[0] + 1
ch = ch[good[ch]]
i0, i1 = ch[:-1], ch[1:]
dt = t[i1]-t[i0]; k = (dt > 1e-4) & (dt < 2e-2)
i0, i1 = i0[k], i1[k]
dr = b[i1,119] - b[i0,119]
print()
print('=== 收敛性: 相邻磁更新间 |新息| 的变化 ===')
print('  d|新息| p50 %+.3f 度 ; 缩小(<0)占 %.1f%%' % (np.median(dr), 100*(dr < 0).mean()))
for lo in range(0, int(t[-1]), 6):
    q = (t[i1] >= lo) & (t[i1] < lo+6)
    if q.sum() < 200: continue
    print('   t %2d~%2d: d|新息| p50 %+8.3f 度  缩小占 %5.1f%%  |新息|p50 %7.2f'
          % (lo, lo+6, np.median(dr[q]), 100*(dr[q] < 0).mean(), np.median(b[i0[q],119])))
