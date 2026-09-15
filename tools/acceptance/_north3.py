# -*- coding: utf-8 -*-
"""修正单位 bug(dt 是 us, 之前误用 1e-8 = 积分被冻住 100 倍)后重算:
   陀螺/att.q/EKF 的台阶偏航 = 可信锚点; 磁场只作为待评对象。
   两张表: ① 各链条的台阶与转角(含 4x90 闭合) ② 以陀螺为真值, 地磁每步错多少"""
import numpy as np, os, sys, math, glob
np.seterr(all='ignore'); sys.stdout.reconfigure(encoding='utf-8', errors='replace')
P = sorted(glob.glob(r'R:\raw*.bin'), key=os.path.getmtime, reverse=True)[0]
sz = os.path.getsize(P); NCH = 162 if (sz//4) % 162 == 0 else 159
raw = np.fromfile(P, dtype='<f4').reshape(-1, NCH); N = raw.shape[0]
c = lambda i: raw[:, i].astype(np.float64)
dt = c(25).copy(); bd = ~np.isfinite(dt) | (dt < 10) | (dt > 50000); dt[bd] = 124.58
t = np.cumsum(dt)*1e-6
tag = c(76); ok = (tag == np.median(tag))
w = raw[:, 26:29].astype(np.float64); gy = np.linalg.norm(w, axis=1)
acc = raw[:, 32:35].astype(np.float64); an = np.linalg.norm(acc, axis=1)
f = raw[:, 42:45].astype(np.float64); n45 = c(45)
still = ok & (gy < 3.0) & (np.abs(an-1.0) < 0.015)
idx = np.where(still)[0]
segs = [s for s in np.split(idx, np.where(np.diff(idx) > 400)[0]+1) if len(s) >= 3200]
print('%s 帧 %d %.2f s; 台阶 %d' % (os.path.basename(P), N, t[-1], len(segs)))
def yaw(q):
    q = q/np.linalg.norm(q); w_, x_, y_, z_ = q
    return math.degrees(math.atan2(2*(w_*z_+x_*y_), 1-2*(y_*y_+z_*z_)))
# 逐帧积分(正确单位: dt[us]*1e-6 s)
qg = np.zeros((N, 4)); qw, qx, qy, qz = 1.0, 0.0, 0.0, 0.0
qg[0] = (1, 0, 0, 0)
for k in range(1, N):
    h = 0.5*(dt[k]*1e-6)*(math.pi/180.0)
    dx, dy, dz = w[k, 0]*h, w[k, 1]*h, w[k, 2]*h
    nw = qw-qx*dx-qy*dy-qz*dz; nx = qw*dx+qx+qy*dz-qz*dy
    ny = qw*dy-qx*dz+qy+qz*dx; nz = qw*dz+qx*dy-qy*dx+qz
    s = 1.0/math.sqrt(nw*nw+nx*nx+ny*ny+nz*nz)
    qw, qx, qy, qz = nw*s, nx*s, ny*s, nz*s
    qg[k] = (qw, qx, qy, qz)
pl = []
for s in segs:
    i = s[len(s)//2]
    u = acc[i]/an[i]; B = f[i]; Bh = B-(B@u)*u; Bh /= np.linalg.norm(Bh)
    pl.append(dict(t=t[i], n=n45[i], dip=math.degrees(math.asin(np.clip(B@u/np.linalg.norm(B), -1, 1))),
                   azi=math.degrees(math.atan2(Bh[1], Bh[0])), i=i,
                   g=yaw(qg[i]), att=yaw(raw[i, 0:4]), ekf=yaw(raw[i, 89:93])))
print('\n台阶  t(s)     n       dip     陀螺yaw    att.q_yaw  EKF_yaw   场方位')
for k, p in enumerate(pl):
    print('#%d %6.2f  %6.4f  %+7.2f  %8.2f  %8.2f  %8.2f  %8.2f' %
          (k+1, p['t'], p['n'], p['dip'], p['g'], p['att'], p['ekf'], p['azi']))
print('\n台阶间转角 (磁场 Δyaw = -Δ方位, 因世界矢量在机体系反向转):')
print('  区间     磁场Δyaw   陀螺Δyaw   att.qΔ     EKFΔ   |  磁场-陀螺(地磁误差)')
sm = sg = 0.0
for k in range(len(pl)-1):
    dm = -(((pl[k+1]['azi']-pl[k]['azi']+180) % 360)-180)
    dg = ((pl[k+1]['g']-pl[k]['g']+180) % 360)-180
    da = ((pl[k+1]['att']-pl[k]['att']+180) % 360)-180
    de = ((pl[k+1]['ekf']-pl[k]['ekf']+180) % 360)-180
    sm += dm; sg += dg
    print('  #%d->#%d  %+8.2f   %+8.2f  %+7.2f  %+7.2f  |  %+7.2f' % (k+1, k+2, dm, dg, da, de, dm-dg))
print('\n累计(4 次 90°): 磁场 %+.2f°  陀螺 %+.2f°  att.q %+.2f°  EKF %+.2f°' %
      (sm, sg, ((pl[-1]['att']-pl[0]['att']+180) % 360)-180, ((pl[-1]['ekf']-pl[0]['ekf']+180) % 360)-180))
print('陀螺 4x90 闭合偏离 -360°: %+.2f°  (机电量程/手转误差都在内)' % (((sg+180) % 360)-180+360))
print('地磁相对陀螺的每步误差: 均值 %+.2f°  rms %.2f°' %
      (np.mean([-(((pl[k+1]['azi']-pl[k]['azi']+180) % 360)-180) - (((pl[k+1]['g']-pl[k]['g']+180) % 360)-180) for k in range(len(pl)-1)]),
       np.sqrt(np.mean([(-(((pl[k+1]['azi']-pl[k]['azi']+180) % 360)-180) - (((pl[k+1]['g']-pl[k]['g']+180) % 360)-180))**2 for k in range(len(pl)-1)]))))
