# -*- coding: utf-8 -*-
# VER=76 验收：核对新观测是否按设计在跑，以及偏航是否重新跟陀螺
import numpy as np
P = r'R:\raw_v9.bin'; NCH = 144; R2D = 57.2957795
r = np.fromfile(P, dtype='<f4'); N = r.size // NCH
b = r[:N*NCH].reshape(-1, NCH).astype(np.float64); t = np.cumsum(b[:,25]*1e-6)
TAG = int(round(b[0,76]))
print('fw_tag %d -> VER=%d   帧 %d  时长 %.2f s   (VER=76 应为 5017607)' % (TAG, TAG>>16, N, t[-1]))

B0X, B0Y = -0.0567804, 0.4296474
good = ((np.abs(b[:,132]-B0X) < 5e-3) & (np.abs(b[:,133]-B0Y) < 5e-3)
        & (b[:,118] > 0.0) & (b[:,119] >= 0) & ((b[:,120] == 0) | (b[:,120] == 1)))
print('好帧 %.2f%% (上报污染 %.2f%%)' % (100*good.mean(), 100*(~good).mean()))

def yaw_of(q):
    w,x,y,z = q[:,0],q[:,1],q[:,2],q[:,3]
    n = np.sqrt(w*w+x*x+y*y+z*z); n[n<1e-9] = 1e-9
    return np.degrees(np.arctan2(2*(w/n)*(z/n)+2*(x/n)*(y/n), 1-2*((y/n)**2+(z/n)**2)))
ye = yaw_of(b[:,89:93]); yl = yaw_of(b[:,0:4])

# ---- 1) 用日志复算 M7 的实测航向 thm，验证固件实现与设计一致 ----
acc = b[:,32:35]; magf = b[:,42:45]
an = np.linalg.norm(acc, axis=1); fn = np.linalg.norm(magf, axis=1)
ab = acc/np.maximum(an,1e-9)[:,None]
mf = magf/np.maximum(fn,1e-9)[:,None]
dpar = np.sum(mf*ab, axis=1)
mh = mf - dpar[:,None]*ab
xv = np.stack([1-ab[:,0]*ab[:,0], -ab[:,0]*ab[:,1], -ab[:,0]*ab[:,2]], 1)
mhn = np.linalg.norm(mh, axis=1); xvn = np.linalg.norm(xv, axis=1)
crs = np.cross(ab, mh)
thm = np.degrees(np.arctan2(np.sum(crs*xv,1)/(mhn*xvn), np.sum(mh*xv,1)/(mhn*xvn)))
D = -7.53
azi0 = np.degrees(np.arctan2(np.cos(np.radians(D)), np.sin(np.radians(D))))   # atan2(b0y,b0x)
psih = b[:,134]                       # 上报的修正前偏航(度)
innov = (thm - (psih - azi0) + 180.0) % 360.0 - 180.0
ok = good & (mhn > 1e-3) & (xvn > 1e-3)
rec = b[:,119]                        # 固件上报的 mag_r (= |r| 度)
d = np.abs(np.abs(innov[ok]) - rec[ok])
print()
print('=== 1) 固件实现 vs 设计复算 ===')
print('  复算 |新息| p50 %.3f 度 ; 固件 mag_r p50 %.3f 度 ; 中位差 %.4f 度, p90 差 %.4f 度'
      % (np.median(np.abs(innov[ok])), np.median(rec[ok]), np.median(d), np.percentile(d,90)))
print('  -> 差应为 0（accel_g 与 raw_f 的在线零偏差会带来零点几度）')

# ---- 2) 偏航是否重新跟陀螺 ----
K = 23
gyr = np.linalg.norm(b[:,26:29], axis=1)
d23g = np.array([(b[i:i+K,28]*b[i:i+K,25]*1e-6).sum() for i in range(0, len(b)-K, K)])
d23e = ((ye[K:]-ye[:-K]+180) % 360-180)[::K]
d23l = ((yl[K:]-yl[:-K]+180) % 360-180)[::K]
n = min(len(d23g), len(d23e), len(d23l))
d23g, d23e, d23l = d23g[:n], d23e[:n], d23l[:n]
m = np.abs(d23g) > 0.05
print()
print('=== 2) 偏航跟随性（每 23 帧；同一把尺子量两条链）===')
print('  corr(陀螺积分, 旧链偏航变化) = %+.3f' % np.corrcoef(d23g[m], d23l[m])[0,1])
print('  corr(陀螺积分, EKF 偏航变化) = %+.3f   <-- VER=74 时只有 +0.176' % np.corrcoef(d23g[m], d23e[m])[0,1])
print('  比值中位 旧链 %.3f ; EKF %.3f  (1.0 = 完全跟随)'
      % (np.median(d23l[m]/d23g[m]), np.median(d23e[m]/d23g[m])))

# ---- 3) 新息符号 vs 实际施加的修正 ----
u = (b[:,120] == 1) & ok
print()
print('=== 3) 收敛符号（施加 dqz 应与新息反号）===')
for lo in range(0, int(t[-1]), 3):
    q = u & (t >= lo) & (t < lo+3)
    if q.sum() < 30: continue
    print('  t %2d~%2d: n=%5d  corr(新息, dqz) %+.3f  同号(发散) %3.0f%%  |新息|p50 %6.2f  |dqz|p50 %7.4f'
          % (lo, lo+3, q.sum(), np.corrcoef(innov[q], b[q,137])[0,1],
             100*(np.sign(innov[q]) == np.sign(b[q,137])).mean(),
             np.median(np.abs(innov[q])), np.median(np.abs(b[q,137]))))

# ---- 4) 总览 ----
print()
print('  t(s)  |w|p50  EKF偏航  旧链偏航  两者差   mag_r  mag_bh  used%%  p_yy   sig_yaw')
for lo in range(0, int(t[-1]), 2):
    q = (t >= lo) & (t < lo+2) & good
    if q.sum() < 100: continue
    print('%5d %7.1f %9.2f %9.2f %8.2f %7.2f %7.3f %6.0f %7.3f %8.2f'
          % (lo, np.median(gyr[q]), np.median(ye[q]), np.median(yl[q]),
             (np.median(ye[q])-np.median(yl[q])+180) % 360-180,
             np.median(b[q,119]), np.median(b[q,118]), 100*(b[q,120]==1).mean(),
             np.median(b[q,121]), np.median(b[q,104])))
