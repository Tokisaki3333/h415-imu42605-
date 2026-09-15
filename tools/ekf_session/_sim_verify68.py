# -*- coding: utf-8 -*-
"""PC 验证 VER=68 的轴修正：用旧链姿态当真值，按固件那套环路跑一遍。
   对照三组：① 原样(0,1,2)  ② VER=66 的 (-x,-y,z)  ③ VER=68 的 (y,x,-z)
"""
import glob, os, re
import numpy as np

P = max([c for c in glob.glob(r'R:\*.bin') if os.path.getsize(c) > 100000], key=os.path.getmtime)
r = np.fromfile(P, dtype='<f4'); n = 144
b = r[:r.size//n*n].reshape(-1, n).astype(np.float64)
T = open(r'C:\Users\33\Documents\v2\h415-imu42605-\V5F\User\inc\v5f_tune.h', 'rb').read().decode('gbk')
def gg(nm, d):
    m = re.search(r'#define\s+' + nm + r'\s+\(?\s*([-\d.]+)', T); return float(m.group(1)) if m else d
DIP, DECL = gg('V5F_EKF_DIP_TAN', 2.08), gg('V5F_MAG_DECL_DEG', -7.53)
KMX, BHMIN, RMAX, NISMAX = gg('V5F_EKF_MAG_K_MAX', .05), gg('V5F_EKF_MAG_BH_MIN', .12), gg('V5F_EKF_MAG_R_MAX_DEG', 150.), 10.
DEAD = gg('V5F_EKF_MAG_DEAD_DEG', 12.0); SG = gg('V5F_EKF_MAG_SIG_RAD', 0.00873)
ci = 1/np.sqrt(1+DIP*DIP); si = DIP*ci; Dr = np.radians(DECL)
b0x, b0y, b0z = ci*np.sin(Dr), ci*np.cos(Dr), -si; n2 = b0x*b0x+b0y*b0y


def qn(q): return q/np.linalg.norm(q)
def Rq(q):
    w, x, y, z = qn(q)
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

f0 = b[:, 42:45].copy(); nn = np.linalg.norm(f0, axis=1); f0 = f0/np.where(nn < 1e-9, 1, nn)[:, None]
ql = b[:, 0:4].copy()
dte = 23*np.median(b[:, 25])*1e-6
cyc = np.arange(0, len(b), 23)
AX = {'原样 (x,y,z)': lambda f: f,
      'VER66 (-x,-y,z)': lambda f: np.column_stack([-f[:, 0], -f[:, 1], f[:, 2]]),
      'VER68 (y,x,-z)': lambda f: np.column_stack([f[:, 1], f[:, 0], -f[:, 2]])}


def run(mapf, e0=0.0):
    F = mapf(f0)
    err = np.radians(e0); Pyy = np.radians(3.)**2; Rr = SG*SG; hs = []; rs = []
    for c, k in enumerate(cyc):
        rl, pt = rp(b[k, 89:93]); qh = eul2q(yaw(ql[k])+err, pt, rl)
        Bv = Rq(qn(qh)) @ F[k]; v = Bv[:2]; bh = np.hypot(v[0], v[1])
        r = v - np.array([b0x, b0y]); mrr = np.degrees(np.linalg.norm(r)/max(bh, 1e-6))
        rs.append(mrr)
        if bh < BHMIN or mrr > RMAX:
            hs.append(err); continue
        if mrr < DEAD:
            hs.append(err); continue
        Hz = np.array([-b0y, b0x]); rz = float(Hz @ r); S = float(Hz@Hz)*Pyy + Rr
        nis = rz*rz/S; K = Pyy*float(Hz[0])/S
        if nis > NISMAX: K /= (nis/NISMAX)
        K = float(np.clip(K, -KMX, KMX))
        if (c*dte) < 3.0:                     # 开机时间窗：精确角一步
            crs = b0x*r[1]-b0y*r[0]; dt2 = b0x*r[0]+b0y*r[1]+n2
            err = np.arctan2(np.sin(err+np.arctan2(crs, dt2)), np.cos(err+np.arctan2(crs, dt2)))
            Pyy = np.radians(3.)**2; hs.append(err); continue
        err = np.arctan2(np.sin(err+K*rz), np.cos(err+K*rz))
        Ky = float(np.clip(Pyy*float(Hz[0])**2/S, 0, 1)); Pyy = (1-Ky)*Pyy + 1e-8
        hs.append(err)
    return np.degrees(np.array(hs)), np.array(rs)


print('轴映射对照（复现固件环路，含死区12/软加权/k_cap/开机3s精确角）：')
for nm, mf in AX.items():
    h, rr = run(mf)
    w = int(3.0/dte)
    print('  %-16s 末err %+8.2f | err p50 %7.2f | mag_r: 窗内p50 %6.2f  出窗p50 %7.2f p90 %7.2f  <死区占比 %5.1f%%'
          % (nm, h[-1], np.median(np.abs(h)), np.median(rr[:w]), np.median(rr[w:]), np.percentile(rr[w:], 90),
             100*np.mean(rr[w:] < DEAD)))
print()
print('VER=68 轴映射下，从大误差出发：')
for e in (30, 90, 120, 179):
    h, rr = run(AX['VER68 (y,x,-z)'], e)
    print('   起始 %+4d -> 末 %+8.2f  出窗 mag_r p50 %6.2f' % (e, h[-1], np.median(rr[int(3.0/dte):])))
