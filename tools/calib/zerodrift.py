# -*- coding: utf-8 -*-
"""全程静止记录：残余零漂速率、漂移率、Allan 方差（ARW / 零偏不稳定性）。"""
import re, sys, os
import numpy as np

FN = sys.argv[1]
CACHE = '_static_q.npy'

if os.path.exists(CACHE):
    q = np.load(CACHE)
    t0, t1 = np.load('_static_t.npy')
else:
    rx = re.compile(r'\[RX\]\s*((?:[0-9A-Fa-f]{2}\s*){28})')
    ts = re.compile(r'\[(\d\d):(\d\d):(\d\d)\.(\d\d\d)\]')
    buf = []; t0 = t1 = None
    with open(FN, 'r', errors='ignore') as f:
        for line in f:
            m = rx.search(line)
            if not m:
                continue
            b = bytes.fromhex(m.group(1).replace(' ', ''))
            if b[24:28] != b'\x00\x00\x80\x7f':     # VER=132: 6 float + 4 B 尾 = 28 B
                continue
            buf.append(b[:16])
            g = ts.search(line)
            if g:
                s = int(g.group(1))*3600+int(g.group(2))*60+int(g.group(3))+int(g.group(4))/1000.0
                if t0 is None: t0 = s
                t1 = s
    q = np.frombuffer(b''.join(buf), dtype='<f4').reshape(-1, 4).astype(np.float32)
    np.save(CACHE, q); np.save('_static_t.npy', np.array([t0, t1]))
q = q.astype(np.float64)
q /= np.linalg.norm(q, axis=1, keepdims=True)
N = len(q); T = float(t1-t0); dt = T/(N-1); fps = 1.0/dt
print("N=%d  T=%.3f s  fps=%.2f" % (N, T, fps))

q0, q1 = q[:-1], q[1:]
w0, x0, y0, z0 = q0.T; w1, x1, y1, z1 = q1.T
dw = w0*w1+x0*x1+y0*y1+z0*z1; dx = w0*x1-x0*w1-y0*z1+z0*y1
dy = w0*y1+x0*z1-y0*w1-z0*x1; dz = w0*z1-x0*y1+y0*x1-z0*w1
ng = dw < 0; dw[ng], dx[ng], dy[ng], dz[ng] = -dw[ng], -dx[ng], -dy[ng], -dz[ng]
nv = np.sqrt(dx*dx+dy*dy+dz*dz)
k = np.where(nv > 1e-15, 2*np.arctan2(nv, np.clip(dw, -1, 1))/np.maximum(nv, 1e-300), 0)
inc = np.stack([dx*k, dy*k, dz*k], 1)                     # rad, 逐帧机体增量
rate_dph = np.degrees(inc)/dt*3600.0                      # deg/h

def mul(a, b):
    aw, ax, ay, az = a; bw, bx, by, bz = b
    return np.array([aw*bw-ax*bx-ay*by-az*bz, aw*bx+ax*bw+ay*bz-az*by,
                     aw*by-ax*bz+ay*bw+az*bx, aw*bz+ax*by-ay*bx+az*bw])
r = mul(np.array([q[0, 0], -q[0, 1], -q[0, 2], -q[0, 3]]), q[-1])
sgn = 1.0 if r[0] >= 0 else -1.0
ang = 2*np.arctan2(np.linalg.norm(r[1:]), abs(r[0]))
e = np.degrees(ang)*r[1:]*sgn/max(np.linalg.norm(r[1:]), 1e-30)

print("\n===== 零漂（全程 %.0f s 纯静止）=====" % T)
print("末态相对起始姿态漂移      : %s deg   |%.4f| deg" % (np.array2string(e, precision=4), np.linalg.norm(e)))
print("                            %.4f deg/h   %.2f deg/天" % (np.linalg.norm(e)/(T/3600), np.linalg.norm(e)/(T/86400)))
print("残余速率均值             : %s deg/h" % np.array2string(rate_dph.mean(0), precision=4))
print("（对照）逐帧速率噪声 1sig : %s deg/h  <- 陀螺白噪 0.12 dps，不是漂移"
      % np.array2string(rate_dph.std(0), precision=1))

# ---- 漂移随时间的走向 ----
th = np.cumsum(inc, axis=0)*180.0/np.pi                   # 累积角 deg（N-1 点）
print("\n分段漂移(deg)  [每 %.0f s 一段]" % (T/8))
for i in range(8):
    a, b = int(i*(N-1)/8), int((i+1)*(N-1)/8)-1
    print("  %5.0f~%5.0f s : %s" % (a*dt, b*dt, np.array2string(th[b]-th[a], precision=4)))

# ---- 重叠 Allan 方差（对速率，deg/h）----
M = len(th)
taus = np.unique(np.round(np.logspace(np.log10(2), np.log10(M//8), 60)).astype(int))
tab = []
for m in taus:
    d = th[2*m:] - 2*th[m:-m] + th[:-2*m]
    var = (d*d).sum(0)/(2.0*(M-2*m))                      # deg^2
    tau = m*dt
    sr = np.sqrt(var)/tau*3600.0                          # deg/h，逐轴
    comp = np.sqrt((sr*sr).sum())                         # 合成
    Narw = 60.0*np.sqrt(var.sum())/np.sqrt(tau)           # deg/rt-h
    tab.append((tau, sr, comp, Narw))
print("\n===== Allan（对速率）=====")
print("  tau(s)     sx(deg/h)  sy(deg/h)  sz(deg/h)   合成(deg/h)   N(deg/rt-h)")
for t, s, c, n in tab:
    if any(abs(np.log10(t)-x) < 0.06 for x in (-4, -3, -2, -1, 0, 1, 2, 2.7)) or t > 200:
        print("  %8.4f  %9.3f %9.3f %9.3f  %10.3f  %11.3f"
              % (t, s[0], s[1], s[2], c, n))
sig_all = np.array([x[2] for x in tab]); tau_all = np.array([x[0] for x in tab])
imin = int(np.argmin(sig_all))
print("\n零偏不稳定性 B = %.4f deg/h   (Allan 最小 %.4f deg/h @ tau=%.1f s, /0.664)"
      % (sig_all[imin]/0.664, sig_all[imin], tau_all[imin]))
sel = (tau_all >= 1.0) & (tau_all <= 10.0)
print("角度随机游走 ARW = %.4f deg/rt-h   (tau=1~10 s 段, sigma(tau)*sqrt(tau))"
      % np.mean(np.array([x[3] for x in tab])[sel]))
print("\n=== 换算到姿态 ===")
print("纯静止 1 小时漂移 ~ %.2f deg" % (np.linalg.norm(e)/(T/3600)))
for hrs in (1, 8, 24):
    print("  静止 %2d h -> 漂移 %.2f deg" % (hrs, np.linalg.norm(e)/(T/3600)*hrs))
print("对照：53 ppm 比例项，转 36000 deg(100 圈) -> %.2f deg" % (53e-6*36000))
