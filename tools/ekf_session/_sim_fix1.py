# -*- coding: utf-8 -*-
"""① 去掉/限幅 χ² 软加权后仿真对比  ② 定位 v 与 v0 的 116° 方向差。"""
import glob, os, re
import numpy as np

P = max([c for c in glob.glob(r'R:\*.bin') if os.path.getsize(c) > 100000], key=os.path.getmtime)
r0 = np.fromfile(P, dtype='<f4'); n = 139
b = r0[:r0.size//n*n].reshape(-1, n).astype(np.float64)
T = open(r'C:\Users\33\Documents\v2\h415-imu42605-\V5F\User\inc\v5f_tune.h', 'rb').read().decode('gbk')
def G(nm, d):
    m = re.search(r'#define\s+%s\s+([-\d.]+)f' % nm, T); return float(m.group(1)) if m else d
DIP, DECL = G('V5F_EKF_DIP_TAN', 2.08), G('V5F_MAG_DECL_DEG', -7.53)
KMX, BHMIN, RMAX, NISMAX = G('V5F_EKF_MAG_K_MAX', .05), G('V5F_EKF_MAG_BH_MIN', .12), G('V5F_EKF_MAG_R_MAX_DEG', 150.), 10.
SG = G('V5F_EKF_MAG_SIG_RAD', 0.00873)
ci = 1/np.sqrt(1+DIP*DIP); si = DIP*ci; Dr = np.radians(DECL)
b0x, b0y, b0z = ci*np.sin(Dr), ci*np.cos(Dr), -si; n2 = b0x*b0x+b0y*b0y


def qn(q): return q/np.linalg.norm(q)
def Rq(q):
    w, x, y, z = q
    return np.array([[1-2*(y*y+z*z), 2*(x*y-w*z), 2*(x*z+w*y)],
                     [2*(x*y+w*z), 1-2*(x*x+z*z), 2*(y*z-w*x)],
                     [2*(x*z-w*y), 2*(y*z+w*x), 1-2*(x*x+y*y)]])
def yaw(q):
    q = qn(q); w, x, y, z = q; return np.arctan2(2*(w*z+x*y), 1-2*(y*y+z*z))
def rp(q):
    q = qn(q); w, x, y, z = q
    return (np.arctan2(2*(w*x+y*z), 1-2*(x*x+y*y)), np.arcsin(np.clip(2*(w*y-z*x), -1, 1)))
def eul2q(yw, pt, rl):
    cy, sy = np.cos(yw/2), np.sin(yw/2); cp, sp = np.cos(pt/2), np.sin(pt/2); cr, sr = np.cos(rl/2), np.sin(rl/2)
    return np.array([cy*cp*cr+sy*sp*sr, cy*cp*sr-sy*sp*cr, cy*sp*cr+sy*cp*sr, sy*cp*cr-cy*sp*sr])

mf = b[:, 42:45].copy(); nn = np.linalg.norm(mf, axis=1); mf = mf/np.where(nn < 1e-9, 1, nn)[:, None]
ql = b[:, 0:4].copy()
Bw = np.einsum('nij,nj->ni', np.stack([Rq(qn(q)) for q in ql[::40]]), mf[::40])
az = np.degrees(np.arctan2(Bw[:, 0], Bw[:, 1]))
dip = np.degrees(np.arcsin(np.clip(Bw[:, 2]/np.linalg.norm(Bw, axis=1), -1, 1)))
print('【②】由 旧链姿态 + 实测机体系磁场 反推的世界磁场：')
print('   |Bw| p50 %.4f' % np.median(np.linalg.norm(Bw, axis=1)))
print('   方位角 p50 %+8.2f 度   (模型 D = %+.2f)' % (np.median(az), DECL))
print('   方位角 p10/p90 %+8.2f / %+8.2f' % (np.percentile(az, 10), np.percentile(az, 90)))
print('   磁倾角 p50 %+8.2f 度   (模型 %.2f)' % (np.median(dip), np.degrees(np.arctan(DIP))))
print('   水平占比 p50 %.4f      (模型 |v0| %.4f)' % (np.median(np.hypot(Bw[:, 0], Bw[:, 1])), np.sqrt(n2)))
daz = np.median(az) - DECL
print('   => 方位角差 %+8.2f 度  <== 这就是 mag_r = 116 度的来源' % daz)
print('   两个水平轴的原始均值: Bx %+.4f  By %+.4f  Bz %+.4f (机体系 f 用旧链转到导航系)'
      % tuple(np.median(Bw, axis=0)))

dte = 23*np.median(b[:, 25])*1e-6
cyc = np.arange(0, len(b), 23)


def run(mode, e0=0.0):
    err = np.radians(e0); Pyy = np.radians(3.)**2; Rr = SG*SG; hs = []
    for c, k in enumerate(cyc):
        rl, pt = rp(b[k, 89:93]); qh = eul2q(yaw(ql[k])+err, pt, rl)
        Bv = Rq(qn(qh)) @ mf[k]; v = Bv[:2]; bh = np.hypot(v[0], v[1])
        r = v - np.array([b0x, b0y]); mrr = np.degrees(np.linalg.norm(r)/max(bh, 1e-6))
        if bh < BHMIN or mrr > RMAX:
            hs.append(err); continue
        Hz = np.array([-b0y, b0x]); rz = float(Hz @ r); S = float(Hz@Hz)*Pyy + Rr
        nis = rz*rz/S; K = Pyy*float(Hz[0])/S
        if mode == 'soft' and nis > NISMAX: K /= (nis/NISMAX)
        if mode == 'clamp3' and nis > NISMAX: K /= min(nis/NISMAX, 3.0)
        if mode == 'nosw': pass
        if mode == 'snap' and (c*dte) < 3.0:
            crs = b0x*r[1]-b0y*r[0]; dt2 = b0x*r[0]+b0y*r[1]+n2; K = 0.0
            err = np.arctan2(np.sin(err+np.arctan2(crs, dt2)), np.cos(err+np.arctan2(crs, dt2)))
            hs.append(err); continue
        K = float(np.clip(K, -KMX, KMX))
        err = np.arctan2(np.sin(err+K*rz), np.cos(err+K*rz))
        Ky = float(np.clip(Pyy*float(Hz[0])**2/S, 0, 1)); Pyy = (1-Ky)*Pyy + 1e-8
        hs.append(err)
    return np.degrees(np.array(hs))


print('\n【①】软加权方案对比（真实倾角，起始 err=0）')
for md, nm in (('soft', '现状：Si/=nis/nis_max'), ('nosw', '完全去掉软加权'),
               ('clamp3', '软加权限幅 sc<=3'), ('snap', '现状+开机3s精确角')):
    h = run(md)
    print('   %-22s 末 err %+8.2f   |err|p50 %7.2f   max %8.2f' % (nm, h[-1], np.median(np.abs(h)), np.abs(h).max()))
print('\n   从大误差出发（完全去掉软加权）：')
for e in (30, 90, 120, 179):
    h = run('nosw', e); print('     起始 %+4d -> 末 %+8.2f' % (e, h[-1]))
