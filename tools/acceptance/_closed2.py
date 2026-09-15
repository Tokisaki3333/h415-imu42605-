# -*- coding: utf-8 -*-
"""当前文件的闭环闭合(att.q/EKF) + 削顶细节, 一致口径"""
import numpy as np, math, glob, os, sys
np.seterr(all='ignore'); sys.stdout.reconfigure(encoding='utf-8', errors='replace')
P = sorted(glob.glob(r'R:\raw*.bin'), key=os.path.getmtime, reverse=True)[0]
raw = np.fromfile(P, dtype='<f4').reshape(-1, 162); N = raw.shape[0]
c = lambda i: raw[:, i].astype(np.float64)
dt = c(25).copy(); bd = ~np.isfinite(dt) | (dt < 10) | (dt > 50000); dt[bd] = 124.58
t = np.cumsum(dt)*1e-6
tag = c(76); ok = (tag == np.median(tag))
gy = np.linalg.norm(raw[:, 26:29].astype(np.float64), axis=1)
an = np.linalg.norm(raw[:, 32:35].astype(np.float64), axis=1)
clip = (np.abs(raw[:, 18:21]).max(axis=1) >= 32700) & ok
print('%s VER=%d 帧 %d %.2f s 削顶 %d' % (os.path.basename(P), int(np.median(tag)) >> 16, N, t[-1], clip.sum()))
still = ok & (gy < 0.5) & (np.abs(an-1.0) < 0.01)
idx = np.where(still)[0]
segs = [s for s in np.split(idx, np.where(np.diff(idx) > 800)[0]+1) if len(s) >= 3200]
for k, s in enumerate(segs):
    print('  静置#%d %6.2f~%6.2f (%4.1fs) |acc| %.4f n %.4f' % (k+1, t[s[0]], t[s[-1]], t[s[-1]]-t[s[0]], np.median(an[s]), np.median(c(45)[s])))
A, B, C = int(segs[0][-1]), int(segs[-1][0]), int(segs[-1][-1])
m = np.zeros(N, bool); m[A+1:B+1] = True
path = (gy*dt*1e-6)[m & ok].sum()
print('运动段 %.2f->%.2f s (%.2f s) 路径 %.0f° |ω|中位 %.0f |acc|中位 %.2f g 段内削顶 %d' %
      (t[A], t[B], t[B]-t[A], path, np.median(gy[m]), np.median(an[m]), clip[m].sum()))
qa = raw[:, 0:4].astype(np.float64); qe = raw[:, 89:93].astype(np.float64)
def yaw(q):
    q = q/np.linalg.norm(q); a, b, d, e = q
    return math.degrees(math.atan2(2*(a*e+b*d), 1-2*(d*d+e*e)))
def UP(q):
    q = q/np.linalg.norm(q); a, b, d, e = q
    return np.array([2*(b*e-a*d), 2*(d*e+a*b), 1-2*(b*b+d*d)])
def rel(q, i, j): 
    a = q[j]/np.linalg.norm(q[j]); b = q[i]/np.linalg.norm(q[i])
    return 2*math.degrees(math.acos(min(1.0, abs(float(a@b)))))
print('\n闭环误差(真值=0)         总角     ppm   倾角   偏航')
for nm, i in (('刚停', B), ('停稳', C)):
    for tagq, q in (('att.q', qa), ('EKF  ', qe)):
        dp = math.degrees(math.acos(max(-1, min(1, float(UP(q[i])@UP(q[A]))))))
        print('  %-5s -> %-4s        %6.2f° %6.0f %6.2f %+7.2f' %
              (tagq, nm, rel(q, A, i), rel(q, A, i)/path*1e6, dp,
               ((yaw(q[i])-yaw(q[A])+180) % 360)-180))
