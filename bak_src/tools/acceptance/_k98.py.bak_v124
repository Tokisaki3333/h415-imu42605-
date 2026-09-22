# -*- coding: utf-8 -*-
"""VER=98 收益的直接检验(不需要重录 VER=97):
   ① 磁有效增益 K_eff = 施加|dqz| / |新息|, 分 静置/剧烈 两段。
      VER=97 在 R≈120° 时 K=0.0063; VER=98 若把 R 压到 21.1° 则 K 应回到 MAG_K_MAX=0.015 钳位。
   ② 剧烈运动结束后的静止段: EKF 倾角误差(vs 加速度) 与 att.q 倾角误差对照。"""
import numpy as np, os, sys, math, glob
np.seterr(all='ignore'); sys.stdout.reconfigure(encoding='utf-8', errors='replace')
P = sorted(glob.glob(r'R:\raw*.bin'), key=os.path.getmtime, reverse=True)[0]
sz = os.path.getsize(P); NCH = 162 if (sz//4) % 162 == 0 else 159
raw = np.fromfile(P, dtype='<f4').reshape(-1, NCH); N = raw.shape[0]
c = lambda i: raw[:, i].astype(np.float64)
dt = c(25).copy(); bd = ~np.isfinite(dt) | (dt < 10) | (dt > 50000); dt[bd] = 124.58
t = np.cumsum(dt)*1e-6
tag = c(76); ok = (tag == np.median(tag))
gy = np.linalg.norm(raw[:, 26:29].astype(np.float64), axis=1)
acc = raw[:, 32:35].astype(np.float64); an = np.linalg.norm(acc, axis=1); av = acc/an[:, None]
clip = (np.abs(raw[:, 18:21]).max(axis=1) >= 32700)
used = c(120); dqz = c(137); thm = c(144); thp = c(145); rs = c(149); sg = c(107)
inn = (thm-thp+180.0) % 360.0-180.0
print('%s VER=%d 帧 %d %.2f s 削顶 %d' % (os.path.basename(P), int(np.median(tag)) >> 16, N, t[-1], clip.sum()))
def UP(q):
    q = q/np.linalg.norm(q, axis=1, keepdims=True); a, b, d, e = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
    return np.stack([2*(b*e-a*d), 2*(d*e+a*b), 1-2*(b*b+d*d)], 1)
upA, upE = UP(raw[:, 0:4].astype(np.float64)), UP(raw[:, 89:93].astype(np.float64))
eA = np.degrees(np.arccos(np.clip((upA*av).sum(1), -1, 1)))
eE = np.degrees(np.arccos(np.clip((upE*av).sum(1), -1, 1)))
cls = [('静置', ok & (gy < 3.0) & (np.abs(an-1.0) < 0.015)),
       ('剧烈运动', ok & (gy > 300.0)),
       ('温和运动', ok & (gy >= 3.0) & (gy <= 300.0))]
print('\n类别      | 新息中位°  施加|dq|中位°   K_eff中位   p10     p90    用满0.015比例  R中位°  σtilt中位°')
for nm, m in cls:
    u = m & (used > 0.5) & (np.abs(inn) > 1e-6)
    if u.sum() < 200: print('%-9s 帧数不足 %d' % (nm, u.sum())); continue
    K = np.abs(dqz[u]/inn[u])
    print('%-9s | %8.2f %12.4f %11.4f %7.4f %7.4f %12.1f%% %8.1f %10.2f' %
          (nm, np.median(np.abs(inn[u])), np.median(np.abs(dqz[u])), np.median(K),
           np.percentile(K, 10), np.percentile(K, 90), 100*np.mean(K > 0.0149),
           np.median((3.5*np.sqrt(np.maximum(rs[u], 0)))), np.median(sg[u])))
print('\n对照: 上一份 VER=97 剧烈段 K_eff = 0.0063 (R≈120°, 未到钳位); VER=98 期望 K_eff = 0.0150 (钳位)')
still = ok & (gy < 0.5) & (np.abs(an-1.0) < 0.01)
idx = np.where(still)[0]
segs = [s for s in np.split(idx, np.where(np.diff(idx) > 800)[0]+1) if len(s) >= 6400]
print('\n静止段倾角误差 (att.q / EKF):')
for s in segs:
    print('  %6.2f~%6.2f s (%4.1fs)  att.q %6.3f°   EKF %6.3f°   (EKF-att.q 偏航差 %+7.2f°)' %
          (t[s[0]], t[s[-1]], t[s[-1]]-t[s[0]], np.median(eA[s]), np.median(eE[s]),
           ((np.median(np.degrees(np.arctan2(2*(raw[s, 89]*raw[s, 93]+raw[s, 90]*raw[s, 91]),
                                            1-2*(raw[s, 90]**2+raw[s, 91]**2)))) -
             np.median(np.degrees(np.arctan2(2*(raw[s, 0]*raw[s, 3]+raw[s, 1]*raw[s, 2]),
                                             1-2*(raw[s, 1]**2+raw[s, 2]**2))))+180) % 360)-180))
