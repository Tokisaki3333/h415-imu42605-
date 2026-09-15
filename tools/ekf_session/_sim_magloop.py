# -*- coding: utf-8 -*-
"""PC 离线复现 M7 地磁偏航环：用旧链姿态当真值，喂真实记录的陀螺/地磁，
   看环路把偏航误差推向 0（收敛）还是推离（发散），并测"假点"位置。

模型（与固件 M7 一致的那套数学）：
  B0  = (cosI*sinD, cosI*cosD, -sinI)   tanI=V5F_EKF_DIP_TAN, D=V5F_MAG_DECL_RAD
  q_hat = Exp((0,0,err)) (x) q_legacy    真值姿态上只改偏航
  Bn  = R(q_hat) * mag_f
  v   = (Bn[0], Bn[1])                  投影到重力法向平面
  r   = v - v0                          v0=(b0x,b0y)
  H   = [-b0y, b0x]                     偏航列
  dx  = sgn * k * (H . r)               sgn=+1 原版, -1 取反
"""
import glob, os, re
import numpy as np

P = max([c for c in glob.glob(r'R:\*.bin') if os.path.getsize(c) > 100000],
        key=os.path.getmtime)
b = np.fromfile(P, dtype='<f4')
nch = None
for n in (139, 133, 128, 122, 117, 115, 113, 112):
    if b.size % n == 0 and 1e6 < b[76] < 4e6 and b[76] == b[n+76]:
        nch = n
        break
if nch is None:          # 兜底：取余数最小的并截断
    cands = [(b.size % n, n) for n in (139, 133, 128, 122) if b.size >= n*3]
    nch = min(cands)[1]
    print('!! 未能严格判定列数，兜底取 %d (余 %d)' % (nch, b.size % nch))
    b = b[:b.size//nch*nch]
b = b.reshape(-1, nch).astype(np.float64)

# tune 常量
T = open(r'C:\Users\33\Documents\v2\h415-imu42605-\V5F\User\inc\v5f_tune.h', 'rb').read().decode('gbk')
def g(nm, dflt):
    m = re.search(r'#define\s+%s\s+([-\d.]+)f' % nm, T)
    return float(m.group(1)) if m else dflt
DIP = g('V5F_EKF_DIP_TAN', 2.08)
DECL = g('V5F_MAG_DECL_DEG', -7.53)
KYAW = g('V5F_EKF_MAG_K_MAX', 0.05)
print('数据 %s  列%d 帧%d' % (os.path.basename(P), nch, len(b)))
print('DIP_TAN=%.3f  DECL=%.2f deg  K_MAX=%.3f' % (DIP, DECL, KYAW))

ci = 1.0/np.sqrt(1.0+DIP*DIP); si = DIP*ci
Dr = np.radians(DECL)
b0x, b0y, b0z = ci*np.sin(Dr), ci*np.cos(Dr), -si
n2 = b0x*b0x + b0y*b0y
print('B0 = (%.4f, %.4f, %.4f)   |v0| = %.4f' % (b0x, b0y, b0z, np.sqrt(n2)))

q = b[:, 0:4].copy()
nn = np.sqrt((q*q).sum(axis=1)); nn = np.where(nn < 1e-9, 1.0, nn); q = q/nn[:, None]
mf = b[:, 42:45].copy()
nmm = np.sqrt((mf*mf).sum(axis=1)); nmm = np.where(nmm < 1e-9, 1.0, nmm); mf = mf/nmm[:, None]
gdt = b[:, 25]*1e-6


def q2R(qq):
    w, x, y, z = qq[..., 0], qq[..., 1], qq[..., 2], qq[..., 3]
    R = np.empty(qq.shape[:-1]+(3, 3))
    R[..., 0, 0] = 1-2*(y*y+z*z); R[..., 0, 1] = 2*(x*y-w*z); R[..., 0, 2] = 2*(x*z+w*y)
    R[..., 1, 0] = 2*(x*y+w*z); R[..., 1, 1] = 1-2*(x*x+z*z); R[..., 1, 2] = 2*(y*z-w*x)
    R[..., 2, 0] = 2*(x*z-w*y); R[..., 2, 1] = 2*(y*z+w*x); R[..., 2, 2] = 1-2*(x*x+y*y)
    return R


# 每 23 帧取一次（≈EKF 周期），模拟偏航误差的演化
STEP = 23
idx = np.arange(0, len(b), STEP)
err = 0.0
Pyy = np.radians(3.0)**2      # P0_YAW 3 度
Rr = np.radians(0.5)**2
print('\n   sgn   起始err   末err     |err|p50   是否收敛')
res = {}
for sgn in (+1.0, -1.0):
    err = 0.0
    Pyy = np.radians(3.0)**2
    hist = []
    for k in idx:
        # q_hat = Exp((0,0,err)) (x) q_legacy
        h = 0.5*err
        dq = np.array([np.cos(h), 0.0, 0.0, np.sin(h)])
        w0, x0, y0, z0 = q[k]
        w1, x1, y1, z1 = dq
        qh = np.array([w1*w0-x1*x0-y1*y0-z1*z0, w1*x0+x1*w0+y1*z0-z1*y0,
                       w1*y0-x1*z0+y1*w0+z1*x0, w1*z0+x1*y0-y1*x0+z1*w0])
        qh = qh/np.linalg.norm(qh)
        R = q2R(qh)
        Bn = R @ mf[k]
        v = Bn[:2]
        r = v - np.array([b0x, b0y])
        Hz = np.array([-b0y, b0x])
        rz = float(Hz @ r)
        K = Pyy/(Pyy + Rr)
        dx = sgn*K*rz
        err = np.arctan2(np.sin(err+dx), np.cos(err+dx))
        Pyy = (1-K)*Pyy + 1e-8
        hist.append(err)
    hist = np.array(hist)
    conv = abs(hist[-1]) < abs(hist[0]) + 1e-9
    print('   %+d   %7.2f  %7.2f   %8.2f   %s' %
          (int(sgn), np.degrees(0.0), np.degrees(hist[-1]), np.degrees(np.median(np.abs(hist))),
           '收敛' if abs(hist[-1]) < np.radians(5) else '不收敛/发散'))
    res[sgn] = hist

print('\n=== 关键：从不同初始误差出发，环路最终停在哪（sgn=+1 与 -1 各测一遍）===')
for sgn in (+1.0, -1.0):
    print(' sgn=%+d' % int(sgn))
    for e0 in (0, 30, 60, 90, 120, 150, 179, -90):
        err = np.radians(e0); Pyy = np.radians(3.0)**2
        for k in idx[:400]:
            h = 0.5*err
            dq = np.array([np.cos(h), 0.0, 0.0, np.sin(h)])
            w0, x0, y0, z0 = q[k]; w1, x1, y1, z1 = dq
            qh = np.array([w1*w0-x1*x0-y1*y0-z1*z0, w1*x0+x1*w0+y1*z0-z1*y0,
                           w1*y0-x1*z0+y1*w0+z1*x0, w1*z0+x1*y0-y1*x0+z1*w0])
            qh = qh/np.linalg.norm(qh)
            Bn = q2R(qh) @ mf[k]
            r = Bn[:2] - np.array([b0x, b0y])
            rz = float(np.array([-b0y, b0x]) @ r)
            K = Pyy/(Pyy+Rr)
            err = np.arctan2(np.sin(err+sgn*K*rz), np.cos(err+sgn*K*rz))
            Pyy = (1-K)*Pyy + 1e-8
        print('   起始 %+4d 度  ->  末 %+7.2f 度' % (e0, np.degrees(err)))
