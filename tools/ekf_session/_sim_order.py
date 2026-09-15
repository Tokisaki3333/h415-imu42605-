# -*- coding: utf-8 -*-
# 检验"每帧一次更新 + 从低频向高频轮询"的优先级顺序
#   对比: 同帧批处理 / 低频优先(GPS,气压,磁) / 高频优先(磁,气压,GPS) / 先到先服务
#   关注: 各源被延后的帧数、姿态与速度 RMSE、以及"空槽帧"占比
import numpy as np

DT = 125e-6; T = 3.0; NS = int(T/DT)
G = np.array([0.0, 0.0, -9.80665])
BG_TRUE = np.array([0.5, -0.3, 0.8])*np.pi/180
BA_TRUE = np.array([0.030, -0.020, 0.015]); BB_TRUE = 2.0
SIG_MAG = np.radians(0.42); SIG_BARO = 0.30; SIG_GPS = 0.10
SIG_G = 0.006; SIG_A = 0.06
MAG_HZ, BARO_HZ, GPS_HZ = 197.0, 7.0, 5.0
BA_RW, BG_RW, BB_RW = 1e-4, 1e-6, 1e-3
SIG_TILT0 = np.radians(2.0)
NAMES = ['mag', 'baro', 'gps']; RATES = [MAG_HZ, BARO_HZ, GPS_HZ]


def skew(v):
    return np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])


def qmul(a, b):
    w1, x1, y1, z1 = a; w2, x2, y2, z2 = b
    return np.array([w1*w2-x1*x2-y1*y2-z1*z2, w1*x2+x1*w2+y1*z2-z1*y2,
                     w1*y2-x1*z2+y1*w2+z1*x2, w1*z2+x1*y2-y1*x2+z1*w2])


def q2R(q):
    w, x, y, z = q
    return np.array([[1-2*(y*y+z*z), 2*(x*y-w*z), 2*(x*z+w*y)],
                     [2*(x*y+w*z), 1-2*(x*x+z*z), 2*(y*z-w*x)],
                     [2*(x*z-w*y), 2*(y*z+w*x), 1-2*(x*x+y*y)]])


rng = np.random.default_rng(7)
true_w = np.zeros((NS, 3)); true_f = np.zeros((NS, 3))
q = np.array([1.0, 0, 0, 0]); v = np.zeros(3); p = np.zeros(3)
qlog = np.zeros((NS, 4)); vlog = np.zeros((NS, 3)); plog = np.zeros((NS, 3))
for k in range(NS):
    tt = k*DT
    wb = np.zeros(3); anav = np.zeros(3)
    if 1.0 < tt < 2.0:
        wb = np.radians(200.0)*np.array([0.45, 0.35, 0.82])
        anav = 0.5*np.array([0.6, -0.5, 0.2])
    fb = q2R(q).T @ (anav - G)
    wm = wb + BG_TRUE + rng.normal(0, SIG_G, 3)
    fm = fb + BA_TRUE + rng.normal(0, SIG_A, 3)
    true_w[k] = wm; true_f[k] = fm
    dq = np.array([1.0, 0.5*DT*wm[0], 0.5*DT*wm[1], 0.5*DT*wm[2]])
    q = qmul(dq, q); q /= np.linalg.norm(q)
    v = v + DT*(q2R(q) @ (fm - BA_TRUE) + G)
    p = p + DT*v
    qlog[k] = q; vlog[k] = v; plog[k] = p

mag_a = rng.random(NS) < MAG_HZ*DT
mag_z = np.array([np.arctan2(2*(qlog[k, 0]*qlog[k, 3]+qlog[k, 1]*qlog[k, 2]),
                             1-2*(qlog[k, 2]**2+qlog[k, 3]**2)) + rng.normal(0, SIG_MAG)
                  for k in range(NS)])
baro_a = rng.random(NS) < BARO_HZ*DT
baro_z = plog[:, 2] + BB_TRUE + rng.normal(0, SIG_BARO, NS)
gps_a = rng.random(NS) < GPS_HZ*DT
gps_z = vlog[:, :2] + rng.normal(0, SIG_GPS, (NS, 2))


class EKF:
    def __init__(self, policy='low2high', order=None):
        self.policy = policy
        self.order = order if order else [2, 1, 0]        # 默认 低频->高频
        self.x = np.zeros(17); self.x[6] = 1.0
        self.P = np.diag(np.r_[np.full(3, 0.1**2), np.full(3, 0.05**2),
                               np.full(3, np.radians(3.0)**2), np.full(3, 0.05**2),
                               np.full(3, np.radians(0.3)**2), [0.5**2]])
        self.k = 0; self.pend = []; self.lat = {0: [], 1: [], 2: [], -1: []}
        self.mac = {"pred": 0, "upd": 0, "upd_cheap": 0}
        self.err_q = []; self.err_v = []; self.served = 0; self.empty = 0
        self.ba = []; self.bg = []; self.bb = []

    def _upd(self, H, r, R):
        m = H.shape[0]
        self.mac['upd'] += 16*16*m + m*m*16 + 16*m*m + 16*m*16 + 16*16*16
        self.mac['upd_cheap'] += 16*16*m + m*m*16 + 16*m*m + m*16*16 + 16*m*16
        PHt = self.P @ H.T
        K = PHt @ np.linalg.inv(H @ PHt + R)
        dx = K @ r
        self.x[0:3] += dx[0:3]; self.x[3:6] += dx[3:6]
        dq = np.array([1.0, 0.5*dx[6], 0.5*dx[7], 0.5*dx[8]]); dq /= np.linalg.norm(dq)
        self.x[6:10] = qmul(dq, self.x[6:10]); self.x[6:10] /= np.linalg.norm(self.x[6:10])
        self.x[10:13] += dx[9:12]; self.x[13:16] += dx[12:15]; self.x[16] += dx[15]
        self.P = (np.eye(16) - K @ H) @ self.P
        self.P = 0.5*(self.P + self.P.T)

    def u_accel(self, fm):
        n = np.linalg.norm(fm)
        if n < 1e-3: return
        Rd = q2R(self.x[6:10])
        H = np.zeros((3, 16)); H[0:3, 6:9] = Rd.T @ skew([0.0, 0.0, 1.0])
        sig = SIG_TILT0 + 1.0*abs(n/9.80665 - 1.0)
        self._upd(H, fm/n - Rd.T @ np.array([0.0, 0.0, 1.0]), np.eye(3)*sig*sig)

    def serve(self, n, z):
        if n == -1:
            self.u_accel(z)
        elif n == 0:
            H = np.zeros((1, 16)); H[0, 8] = 1.0
            psi = np.arctan2(2*(self.x[6]*self.x[9]+self.x[7]*self.x[8]),
                             1-2*(self.x[8]**2+self.x[9]**2))
            self._upd(H, np.array([(z - psi + np.pi) % (2*np.pi) - np.pi]), np.array([[SIG_MAG**2]]))
        elif n == 1:
            H = np.zeros((1, 16)); H[0, 2] = 1.0; H[0, 15] = 1.0
            self._upd(H, np.array([z - (self.x[2] + self.x[16])]), np.array([[SIG_BARO**2]]))
        else:
            H = np.zeros((2, 16)); H[0, 3] = 1.0; H[1, 4] = 1.0
            self._upd(H, z - self.x[3:5], np.eye(2)*SIG_GPS**2)

    def step(self, k, wm, fm):
        dt = DT
        Rd = q2R(self.x[6:10]); fb = fm - self.x[10:13]
        F = np.eye(16); F[0:3, 3:6] = np.eye(3)*dt
        F[3:6, 6:9] = -skew(Rd @ fb)*dt
        F[3:6, 9:12] = -Rd*dt; F[6:9, 12:15] = -Rd*dt
        a2 = SIG_A**2 + (9.80665*np.sqrt(self.P[6, 6]+self.P[7, 7]))**2
        Q = np.zeros((16, 16))
        Q[3:6, 3:6] = np.eye(3)*a2*dt; Q[6:9, 6:9] = np.eye(3)*SIG_G**2*dt
        Q[9:12, 9:12] = np.eye(3)*BA_RW**2*dt; Q[12:15, 12:15] = np.eye(3)*BG_RW**2*dt
        Q[15, 15] = BB_RW**2*dt
        P = self.P; Gn = np.empty((16, 16))
        Gn[0:3, :] = P[0:3, :] + dt*P[3:6, :]
        Gn[3:6, :] = P[3:6, :] - dt*(skew(Rd @ fb) @ P[6:9, :] + Rd @ P[9:12, :])
        Gn[6:9, :] = P[6:9, :] - dt*(Rd @ P[12:15, :])
        Gn[9:16, :] = P[9:16, :]
        Pn = Gn.copy(); Pn[:, 0:3] += dt*Gn[:, 3:6]
        Pn[:, 3:6] -= dt*(Gn[:, 6:9] @ skew(Rd @ fb).T + Gn[:, 9:12] @ Rd.T)
        Pn[:, 6:9] -= dt*(Gn[:, 12:15] @ Rd.T)
        self.P = Pn + Q
        self.mac["pred"] += 672
        wc = wm - self.x[13:16]
        dq = np.array([1.0, 0.5*dt*wc[0], 0.5*dt*wc[1], 0.5*dt*wc[2]])
        self.x[6:10] = qmul(dq, self.x[6:10]); self.x[6:10] /= np.linalg.norm(self.x[6:10])
        Rn = q2R(self.x[6:10]); an = Rn @ (fm - self.x[10:13]) + G
        self.x[0:3] += self.x[3:6]*dt + 0.5*an*dt*dt; self.x[3:6] += an*dt
        self.k += 1
        if self.policy == 'imu_fill':
            self.pend.append((k, (-1, fm)))
        else:
            self.u_accel(fm)
        arr = []
        if mag_a[k]:  arr.append((0, mag_z[k]))
        if baro_a[k]: arr.append((1, baro_z[k]))
        if gps_a[k]:  arr.append((2, gps_z[k]))
        for it in arr:
            self.pend.append((k, it))
        if self.policy == 'batch':
            for _, (n, z) in self.pend: self.serve(n, z)
            self.served += len(self.pend); self.pend = []
        elif self.policy == 'fifo':
            if self.pend:
                k0, (n, z) = self.pend.pop(0)
                self.lat[n].append(k-k0); self.serve(n, z); self.served += 1
            else:
                self.empty += 1
        else:                                             # 按优先级取一个
            pick = -1
            for n in self.order:
                for idx, (k0, (nn, z)) in enumerate(self.pend):
                    if nn == n: pick = idx; break
                if pick >= 0: break
            if pick >= 0:
                k0, (n, z) = self.pend.pop(pick)
                self.lat[n].append(k-k0); self.serve(n, z); self.served += 1
            else:
                self.empty += 1
        self.err_q.append(np.degrees(2*np.arccos(min(1.0, abs(np.dot(self.x[6:10], qlog[k]))))))
        self.err_v.append(np.linalg.norm(self.x[3:6]-vlog[k]))
        self.ba.append(self.x[10:13].copy()); self.bg.append(self.x[13:16].copy())
        self.bb.append(self.x[16])


def run(**kw):
    f = EKF(**kw)
    for k in range(NS):
        f.step(k, true_w[k], true_f[k])
    return f


print('各源到达率: 磁 %.0f Hz  气压 %.0f Hz  GPS %.0f Hz  (帧率 8000 Hz)'
      % (MAG_HZ, BARO_HZ, GPS_HZ))
print('到达事件数: 磁 %d  气压 %d  GPS %d  = 共 %d ; 一帧一个 -> 服务上限 %d 次'
      % (mag_a.sum(), baro_a.sum(), gps_a.sum(), mag_a.sum()+baro_a.sum()+gps_a.sum(), NS))
print()
print('%-26s %9s %9s %9s | %9s %9s %8s %8s | %s'
      % ('策略', '姿态末段', '速度末段', 'bb误差', '姿态旋转段', '速度旋转段', '姿态max', '速度max', '各源延后(帧)'))
res = {}
for tag, kw in [('同帧批处理', dict(policy='batch')),
                ('低频->高频 (GPS,气压,磁)', dict(policy='prio', order=[2, 1, 0])),
                ('高频->低频 (磁,气压,GPS)', dict(policy='prio', order=[0, 1, 2])),
                ('先到先服务', dict(policy='fifo')),
                ('低频->高频+重力兜底', dict(policy='imu_fill', order=[2, 1, 0, -1]))]:
    f = run(**kw)
    n = 4000
    qe = np.mean(f.err_q[-n:]); ve = np.mean(f.err_v[-n:])
    eq = np.array(f.err_q); ev = np.array(f.err_v)
    q_rot = np.mean(eq[8000:16000]); v_rot = np.mean(ev[8000:16000])
    q_max = eq.max(); v_max = ev.max()
    bbe = abs(np.mean(f.bb[-n:]) - BB_TRUE)
    s = []
    for i in [0, 1, 2]:
        L = np.array(f.lat[i]) if f.lat[i] else np.array([0])
        s.append('%s %.0f/%.0f/%d' % (NAMES[i], np.percentile(L, 50), np.percentile(L, 99), L.max()))
    print('%-26s %8.3f° %8.4f %9.3f | %8.3f° %8.4f %7.3f° %7.3f | %s'
          % (tag, qe, ve, bbe, q_rot, v_rot, q_max, v_max, '  '.join(s)))
    res[tag] = (qe, ve, f.empty, f.served)
print()
print('空槽帧占比: ' + '  '.join('%s %.1f%%' % (t, 100*v[2]/NS) for t, v in res.items()))
print('实际服务次数: ' + '  '.join('%s %d' % (t, v[3]) for t, v in res.items()))
print('到达总数 = %d' % (mag_a.sum()+baro_a.sum()+gps_a.sum()))

f = run(policy='imu_fill', order=[2, 1, 0, -1])
print()
print('=== 重力每帧兜底：每帧更新次数 ===')
print('   帧数 %d ; 服务次数 %d -> 每帧 %.3f 次' % (NS, f.served, f.served/NS))
print('   外部源 %d 次 + 重力 %d 次' % (f.served-NS, NS))
for i in [-1, 0, 1, 2]:
    L = np.array(f.lat[i]) if f.lat[i] else np.array([0])
    print('   %-5s 延后(帧) p50 %d p99 %d max %d' % ('imu' if i == -1 else NAMES[i],
          np.percentile(L, 50), np.percentile(L, 99), L.max()))
print()
print('=== 每帧 MAC 分解（16 维误差态）===')
pr = f.mac['pred']//NS; up = f.mac['upd_cheap']//NS
print('   稀疏预测              : %5d' % pr)
print('   重力更新 3 维(P-=K(HP)): %5d   (若用 (I-KH)P 写法: %d)' % (up, f.mac['upd']//NS))
print('   合计/帧               : %5d MAC  @144MHz ~ %.1f us  (占 125us 的 %.0f%%)'
      % (pr+up, (pr+up)/144.0, 100*(pr+up)/144.0/125))
