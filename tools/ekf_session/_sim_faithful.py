# -*- coding: utf-8 -*-
"""PC 复现（与固件逐项一致）：k_cap + chi2 软加权 + 死点 + 新息门 + 真实 ekf_q 倾角。

与固件的对应：
  观测   v = (R(q_hat) f)[0:2]，r = v - v0，偏航列 H = (-b0y, b0x)
  更新   S = H Pyy H^T + R; K = Pyy H^T / S; |K| <= MAG_K_MAX(0.05)
         nis = rz^2/S; nis>NIS_MAX -> Si /= nis/NIS_MAX（软加权）
         dx = K rz;  Pyy -= K S K^T;  Pyy += Qyaw*dt_e
  门     死点 |v|<BH_MIN(0.12) 跳过；隐含偏航误差 >MAG_R_MAX(150) 度 跳过
  姿态   倾角用日志里真实 ekf_q 的 roll/pitch（环路只注入偏航，不动倾角）
         偏航 = 旧链偏航 + err（旧链在本实验中足够精确）
"""
import glob, os, re
import numpy as np

P = max([c for c in glob.glob(r'R:\*.bin') if os.path.getsize(c) > 100000], key=os.path.getmtime)
r0 = np.fromfile(P, dtype='<f4'); n = 139
b = r0[:r0.size//n*n].reshape(-1, n).astype(np.float64)
T = open(r'C:\Users\33\Documents\v2\h415-imu42605-\V5F\User\inc\v5f_tune.h', 'rb').read().decode('gbk')
def G(nm, d):
    m = re.search(r'#define\s+%s\s+([-\d.]+)f' % nm, T)
    return float(m.group(1)) if m else d
DIP, DECL = G('V5F_EKF_DIP_TAN', 2.08), G('V5F_MAG_DECL_DEG', -7.53)
KMX, BHMIN = G('V5F_EKF_MAG_K_MAX', 0.05), G('V5F_EKF_MAG_BH_MIN', 0.12)
RMAX, NISMAX = G('V5F_EKF_MAG_R_MAX_DEG', 150.0), 10.0
SG = G('V5F_EKF_MAG_SIG_RAD', 0.00873)
ci = 1.0/np.sqrt(1.0+DIP*DIP); si = DIP*ci; Dr = np.radians(DECL)
b0x, b0y, b0z = ci*np.sin(Dr), ci*np.cos(Dr), -si
n2 = b0x*b0x+b0y*b0y
print('%s 帧%d  DIP=%.2f DECL=%.2f K_MAX=%.2f BH_MIN=%.2f R_MAX=%.0f |v0|=%.4f'
      % (os.path.basename(P), len(b), np.degrees(np.arctan(DIP)), DECL, KMX, BHMIN, RMAX, np.sqrt(n2)))


def qn(q):
    return q/np.linalg.norm(q)


def Rq(q):
    w, x, y, z = q
    return np.array([[1-2*(y*y+z*z), 2*(x*y-w*z), 2*(x*z+w*y)],
                     [2*(x*y+w*z), 1-2*(x*x+z*z), 2*(y*z-w*x)],
                     [2*(x*z-w*y), 2*(y*z+w*x), 1-2*(x*x+y*y)]])


def yaw(q):
    q = qn(q); w, x, y, z = q
    return np.arctan2(2*(w*z+x*y), 1-2*(y*y+z*z))


def rp(q):
    q = qn(q); w, x, y, z = q
    return (np.arctan2(2*(w*x+y*z), 1-2*(x*x+y*y)), np.arcsin(np.clip(2*(w*y-z*x), -1, 1)))


def eul2q(yw, pt, rl):
    cy, sy = np.cos(yw/2), np.sin(yw/2)
    cp, sp = np.cos(pt/2), np.sin(pt/2)
    cr, sr = np.cos(rl/2), np.sin(rl/2)
    return np.array([cy*cp*cr+sy*sp*sr, cy*cp*sr-sy*sp*cr, cy*sp*cr+sy*cp*sr, sy*cp*cr-cy*sp*sr])


mf = b[:, 42:45].copy(); nn = np.linalg.norm(mf, axis=1); mf = mf/np.where(nn < 1e-9, 1, nn)[:, None]
dte = 23*np.median(b[:, 25])*1e-6
STEP = 23
cyc = np.arange(0, len(b), STEP)


def run(k_cap=True, softw=True, real_tilt=True, boot_snap_s=0.0, e0=0.0, verbose=False):
    err = np.radians(e0); Pyy = np.radians(3.0)**2; Rr = SG*SG
    hist = []
    for c, k in enumerate(cyc):
        qe = b[k, 89:93]
        if real_tilt:
            rl, pt = rp(qe); yw = yaw(b[k, 0:4]) + err; qh = eul2q(yw, pt, rl)
        else:
            h = 0.5*err
            dq = np.array([np.cos(h), 0, 0, np.sin(h)])
            ql = qn(b[k, 0:4]); w1, x1, y1, z1 = dq; w0, x0, y0, z0 = ql
            qh = np.array([w1*w0-x1*x0-y1*y0-z1*z0, w1*x0+x1*w0+y1*z0-z1*y0,
                           w1*y0-x1*z0+y1*w0+z1*x0, w1*z0+x1*y0-y1*x0+z1*w0])
        Bn = Rq(qn(qh)) @ mf[k]
        v = Bn[:2]
        bh = np.hypot(v[0], v[1])
        r = v - np.array([b0x, b0y])
        mrr = np.degrees(np.linalg.norm(r)/max(bh, 1e-6))
        if bh < BHMIN or mrr > RMAX:
            Pyy += 1e-6
            hist.append(err); continue
        Hz = np.array([-b0y, b0x])
        rz = float(Hz @ r)
        S = float(Hz @ Hz)*Pyy + Rr
        nis = rz*rz/S
        K = Pyy*float(Hz[0])/S
        if softw and nis > NISMAX:
            K /= (nis/NISMAX)
        if k_cap:
            K = np.clip(K, -KMX, KMX)
        dx = K*rz
        # 开机窗：改用精确角一步
        if boot_snap_s > 0 and (c*dte) < boot_snap_s:
            crs = b0x*r[1]-b0y*r[0]; dt2 = b0x*r[0]+b0y*r[1]+n2
            dx = np.arctan2(crs, dt2)
            Pyy = np.radians(3.0)**2
        err = np.arctan2(np.sin(err+dx), np.cos(err+dx))
        Ky = np.clip(Pyy*float(Hz[0])*float(Hz[0])/S, 0, 1)
        Pyy = (1-Ky)*Pyy + 1e-8
        hist.append(err)
        if verbose and c % 300 == 0:
            print('    c=%4d t=%5.2fs err=%+8.2f  mag_r=%6.2f  K=%.4f nis=%.2f bh=%.3f'
                  % (c, c*dte, np.degrees(err), mrr, K, nis, bh))
    return np.degrees(np.array(hist))


print('\n【A】复现：真实倾角 + k_cap + 软加权（=固件 VER=64）')
h = run(True, True, True, 0.0, 0.0, verbose=True)
print('   末 err %+7.2f 度   |err|p50 %6.2f  max %7.2f' % (h[-1], np.median(np.abs(h)), np.abs(h).max()))
print('\n【B】对照：关掉 k_cap（K 不限）')
h2 = run(False, True, True, 0.0, 0.0)
print('   末 err %+7.2f 度   |err|p50 %6.2f' % (h2[-1], np.median(np.abs(h2))))
print('\n【C】对照：完美倾角（用旧链姿态）+ k_cap')
h3 = run(True, True, False, 0.0, 0.0)
print('   末 err %+7.2f 度   |err|p50 %6.2f' % (h3[-1], np.median(np.abs(h3))))
print('\n【D】对照：真实倾角 + k_cap，但重力法向投影保留（当前） vs 用完整三维矢量')
print('\n【E】修法候选：真实倾角 + k_cap + 开机 3s 精确角 big-step')
h5 = run(True, True, True, 3.0, 0.0, verbose=True)
print('   末 err %+7.2f 度   |err|p50 %6.2f  max %7.2f' % (h5[-1], np.median(np.abs(h5)), np.abs(h5).max()))
print('\n【F】从大误差出发（真实倾角 + k_cap）：')
for e in (30, 90, 120, 179):
    hh = run(True, True, True, 0.0, e)
    print('   起始 %+4d -> 末 %+8.2f 度' % (e, hh[-1]))
