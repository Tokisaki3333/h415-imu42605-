# -*- coding: utf-8 -*-
# 泄漏测试：多轴同时复杂旋转为基本运动, 注入大的磁航向误差, 分解姿态误差为
#           偏航(绕竖直) + 倾斜(水平两轴), 看激励是否泄漏到倾斜。
# 两次运行用同一噪声种子, 唯一差别是磁注入 -> 差值即"泄露"。
import numpy as np

DT = 125e-6; T = 6.0; NS = int(T/DT)
G = np.array([0.0, 0.0, -9.80665])
BG_TRUE = np.array([0.5, -0.3, 0.8])*np.pi/180
BA_TRUE = np.array([0.030, -0.020, 0.015]); BB_TRUE = 2.0
SIG_MAG = np.radians(0.42); SIG_BARO = 0.30; SIG_GPS = 0.10
SIG_G = 0.006; SIG_A = 0.06
MAG_HZ, BARO_HZ, GPS_HZ = 197.0, 7.0, 5.0
BA_RW, BG_RW, BB_RW = 1e-4, 1e-6, 1e-3
SIG_TILT0 = np.radians(2.0)
INJ = np.radians(30.0)          # 磁注入幅度 30 度
T_INJ0, T_INJ1 = 2.0, 4.0
AX = 8000                       # 分段边界


def skew(v):
    return np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])


def qmul(a, b):
    w1, x1, y1, z1 = a; w2, x2, y2, z2 = b
    return np.array([w1*w2-x1*x2-y1*y2-z1*z2, w1*x2+x1*w2+y1*z2-z1*y2,
                     w1*y2-x1*z2+y1*w2+z1*x2, w1*z2+x1*y2-y1*x2+z1*w2])


def qconj(a):
    return np.array([a[0], -a[1], -a[2], -a[3]])


def q2R(q):
    w, x, y, z = q
    return np.array([[1-2*(y*y+z*z), 2*(x*y-w*z), 2*(x*z+w*y)],
                     [2*(x*y+w*z), 1-2*(x*x+z*z), 2*(y*z-w*x)],
                     [2*(x*z-w*y), 2*(y*z+w*x), 1-2*(x*x+y*y)]])


# ---------------- 真值: 多轴同时复杂旋转 ----------------
rng0 = np.random.default_rng(11)
seed_state = rng0.integers(0, 2**31)
ng = np.random.default_rng(seed_state)
na = np.random.default_rng(seed_state+1)
nm = np.random.default_rng(seed_state+2)
nb = np.random.default_rng(seed_state+3)
nx = np.random.default_rng(seed_state+4)
true_w = np.zeros((NS, 3)); true_f = np.zeros((NS, 3))
q = np.array([1.0, 0, 0, 0]); v = np.zeros(3); p = np.zeros(3)
qlog = np.zeros((NS, 4)); vlog = np.zeros((NS, 3)); plog = np.zeros((NS, 3))
A_dps = 200.0; F = (0.37, 0.53, 0.71); PH = (0.0, 1.1, 2.3)
for k in range(NS):
    tt = k*DT
    wb = np.zeros(3); anav = np.zeros(3)
    if 1.0 < tt < 5.0:
        wb = np.radians(A_dps)*np.array([np.sin(2*np.pi*F[i]*tt+PH[i]) for i in range(3)])
        anav = 0.4*np.array([np.sin(2*np.pi*0.23*tt), np.cos(2*np.pi*0.31*tt),
                             np.sin(2*np.pi*0.17*tt)])
    fb = q2R(q).T @ (anav - G)
    wm = wb + BG_TRUE + ng.normal(0, SIG_G, 3)
    fm = fb + BA_TRUE + na.normal(0, SIG_A, 3)
    true_w[k] = wm; true_f[k] = fm
    dq = np.array([1.0, 0.5*DT*wm[0], 0.5*DT*wm[1], 0.5*DT*wm[2]])
    q = qmul(dq, q); q /= np.linalg.norm(q)
    v = v + DT*(q2R(q) @ (fm - BA_TRUE) + G)
    p = p + DT*v
    qlog[k] = q; vlog[k] = v; plog[k] = p

mag_a = nm.random(NS) < MAG_HZ*DT
mag_base = np.array([np.arctan2(2*(qlog[k, 0]*qlog[k, 3]+qlog[k, 1]*qlog[k, 2]),
                                1-2*(qlog[k, 2]**2+qlog[k, 3]**2)) + nm.normal(0, SIG_MAG)
                     for k in range(NS)])
baro_a = nb.random(NS) < BARO_HZ*DT
baro_z = plog[:, 2] + BB_TRUE + nb.normal(0, SIG_BARO, NS)
gps_a = nx.random(NS) < GPS_HZ*DT
gps_z = vlog[:, :2] + nx.normal(0, SIG_GPS, (NS, 2))
inj = np.array([INJ if T_INJ0 < k*DT < T_INJ1 else 0.0 for k in range(NS)])


class EKF:
    def __init__(self, use_accel=True, layout='fixed'):
        self.use_accel, self.layout = use_accel, layout
        self.x = np.zeros(17); self.x[6] = 1.0
        self.P = np.diag(np.r_[np.full(3, 0.1**2), np.full(3, 0.05**2),
                               np.full(3, np.radians(3.0)**2), np.full(3, 0.05**2),
                               np.full(3, np.radians(0.3)**2), [0.5**2]])
        self.err = []; self.corr = []; self.served = 0; self.inj = []

    def _upd(self, H, r, R, src=None):
        PHt = self.P @ H.T
        K = PHt @ np.linalg.inv(H @ PHt + R)
        dx = K @ r
        if src == 'mag':
            self.inj.append((np.degrees(np.hypot(dx[6], dx[7])), np.degrees(dx[8]),
                             np.degrees(r[0])))
        self.x[0:3] += dx[0:3]; self.x[3:6] += dx[3:6]
        dq = np.array([1.0, 0.5*dx[6], 0.5*dx[7], 0.5*dx[8]]); dq /= np.linalg.norm(dq)
        self.x[6:10] = qmul(dq, self.x[6:10]); self.x[6:10] /= np.linalg.norm(self.x[6:10])
        if self.layout == 'fixed':
            self.x[10:13] += dx[9:12]; self.x[13:16] += dx[12:15]; self.x[16] += dx[15]
        else:
            self.x[10:13] += dx[10:13]; self.x[13:16] += dx[13:16]; self.x[16] += dx[15]
        self.P = (np.eye(16) - K @ H) @ self.P
        self.P = 0.5*(self.P + self.P.T)

    def u_accel(self, fm):
        n = np.linalg.norm(fm)
        if n < 1e-3: return
        Rd = q2R(self.x[6:10])
        H = np.zeros((3, 16)); H[0:3, 6:9] = Rd.T @ skew([0.0, 0.0, 1.0])
        sig = SIG_TILT0 + 1.0*abs(n/9.80665 - 1.0)
        self._upd(H, fm/n - Rd.T @ np.array([0.0, 0.0, 1.0]), np.eye(3)*sig*sig)

    def u_mag(self, z):
        H = np.zeros((1, 16)); H[0, 8] = 1.0
        psi = np.arctan2(2*(self.x[6]*self.x[9]+self.x[7]*self.x[8]),
                         1-2*(self.x[8]**2+self.x[9]**2))
        self.corr.append((self.P[6, 8]/(np.sqrt(self.P[6, 6]*self.P[8, 8])+1e-30),
                          self.P[7, 8]/(np.sqrt(self.P[7, 7]*self.P[8, 8])+1e-30)))
        self._upd(H, np.array([(z - psi + np.pi) % (2*np.pi) - np.pi]), np.array([[SIG_MAG**2]]), src='mag')

    def u_baro(self, z):
        H = np.zeros((1, 16)); H[0, 2] = 1.0; H[0, 15] = 1.0
        self._upd(H, np.array([z - (self.x[2] + self.x[16])]), np.array([[SIG_BARO**2]]))

    def u_gps(self, z):
        H = np.zeros((2, 16)); H[0, 3] = 1.0; H[1, 4] = 1.0
        self._upd(H, z - self.x[3:5], np.eye(2)*SIG_GPS**2)

    def step(self, k, wm, fm):
        dt = DT
        Rd = q2R(self.x[6:10]); fb = fm - self.x[10:13]
        P = self.P; Gn = np.empty((16, 16))
        Gn[0:3, :] = P[0:3, :] + dt*P[3:6, :]
        if self.layout == 'fixed':
            Gn[3:6, :] = P[3:6, :] - dt*(skew(Rd @ fb) @ P[6:9, :] + Rd @ P[9:12, :])
            Gn[6:9, :] = P[6:9, :] - dt*(Rd @ P[12:15, :])
        else:
            Gn[3:6, :] = P[3:6, :] - dt*(skew(Rd @ fb) @ P[6:9, :] + Rd @ P[10:13, :])
            Gn[6:9, :] = P[6:9, :] - dt*(Rd @ P[13:16, :])
        Gn[9:16, :] = P[9:16, :]
        Pn = Gn.copy(); Pn[:, 0:3] += dt*Gn[:, 3:6]
        if self.layout == 'fixed':
            Pn[:, 3:6] -= dt*(Gn[:, 6:9] @ skew(Rd @ fb).T + Gn[:, 9:12] @ Rd.T)
            Pn[:, 6:9] -= dt*(Gn[:, 12:15] @ Rd.T)
        else:
            Pn[:, 3:6] -= dt*(Gn[:, 6:9] @ skew(Rd @ fb).T + Gn[:, 10:13] @ Rd.T)
            Pn[:, 6:9] -= dt*(Gn[:, 13:16] @ Rd.T)
        a2 = SIG_A**2 + (9.80665*np.sqrt(P[6, 6]+P[7, 7]))**2
        Q = np.zeros((16, 16))
        Q[3:6, 3:6] = np.eye(3)*a2*dt; Q[6:9, 6:9] = np.eye(3)*SIG_G**2*dt
        Q[9:12, 9:12] = np.eye(3)*BA_RW**2*dt; Q[12:15, 12:15] = np.eye(3)*BG_RW**2*dt
        Q[15, 15] = BB_RW**2*dt
        self.P = Pn + Q
        wc = wm - self.x[13:16]
        dq = np.array([1.0, 0.5*dt*wc[0], 0.5*dt*wc[1], 0.5*dt*wc[2]])
        self.x[6:10] = qmul(dq, self.x[6:10]); self.x[6:10] /= np.linalg.norm(self.x[6:10])
        Rn = q2R(self.x[6:10]); an = Rn @ (fm - self.x[10:13]) + G
        self.x[0:3] += self.x[3:6]*dt + 0.5*an*dt*dt; self.x[3:6] += an*dt
        if self.use_accel:
            self.u_accel(fm)
        # 依次轮询, 服务一个就停: 低频 -> 高频
        if gps_a[k]:
            self.u_gps(gps_z[k]); self.served += 1
        elif baro_a[k]:
            self.u_baro(baro_z[k]); self.served += 1
        elif mag_a[k]:
            self.u_mag(mag_z[k]); self.served += 1
        # 误差分解
        qe = qmul(self.x[6:10], qconj(qlog[k]))
        if qe[0] < 0: qe = -qe
        dth = 2*qe[1:4]
        self.err.append(dth.copy())


def run(inject, **kw):
    global mag_z
    mag_z = mag_base + (inj if inject else 0.0)
    f = EKF(**kw)
    for k in range(NS):
        f.step(k, true_w[k], true_f[k])
    return f, np.array(f.err)


def phase(E, a, b):
    e = E[a:b]
    yaw = np.sqrt(np.mean(e[:, 2]**2))*180/np.pi
    tilt = np.sqrt(np.mean(e[:, 0]**2 + e[:, 1]**2))*180/np.pi
    return yaw, tilt


SEG = [('静止 0-1s', 0, 8000), ('旋转 1-2s', 8000, 16000), ('旋转+注入 2-4s', 16000, 32000),
       ('旋转 4-5s', 32000, 40000), ('静止 5-6s', 40000, 48000)]

for tag, kw in [('重力观测开', dict(use_accel=True)),
                ('重力观测关', dict(use_accel=False))]:
    fb_, Eb = run(False, **kw)
    fi_, Ei = run(True, **kw)
    print('=== %s ===' % tag)
    print('  %-16s | %8s %8s | %8s %8s | %10s %10s' %
          ('段', '偏航base', '倾斜base', '偏航inj', '倾斜inj', 'Δ偏航', 'Δ倾斜(泄漏)'))
    for nm, a, b in SEG:
        yb, tb = phase(Eb, a, b); yi, ti = phase(Ei, a, b)
        print('  %-16s | %7.3f° %7.3f° | %7.3f° %7.3f° | %9.3f° %10.3f°'
              % (nm, yb, tb, yi, ti, yi-yb, ti-tb))
    d = Ei - Eb
    pk = np.abs(d[:, 0:2]).max()*180/np.pi
    print('  注入期间倾斜差峰值 %.3f°  (偏航差峰值 %.3f°)' % (pk, np.abs(d[:, 2]).max()*180/np.pi))
    J = np.array(fi_.inj)
    if len(J):
        print('  【磁通道本身】每次磁更新的注入量（30 度注入期间）:')
        w = J[J[:,2] > 5.0]      # 只看大新息的那些更新
        print('     大新息(|r|>5度) 次数 %d ; 单次注入 倾斜 p50 %.4f° p99 %.4f° max %.4f° | 偏航 p50 %.3f° max %.3f°'
              % (len(w), np.percentile(w[:,0],50), np.percentile(w[:,0],99), w[:,0].max(),
                 np.percentile(w[:,1],50), w[:,1].max()))
        print('     倾斜/偏航 注入比 p50 %.5f  (=直接泄漏比)' % np.median(w[:,0]/np.abs(w[:,1])))
    if fi_.corr:
        c = np.array(fi_.corr)
        w = (np.arange(len(c))*0 + 1)
        print('  磁更新时 P[tilt,yaw] 相关系数: |rho_x| p50 %.3f max %.3f ; |rho_y| p50 %.3f max %.3f'
              % (np.median(np.abs(c[:, 0])), np.abs(c[:, 0]).max(),
                 np.median(np.abs(c[:, 1])), np.abs(c[:, 1]).max()))
    print()
