# -*- coding: utf-8 -*-
# 端到端验证：多轴同时强旋转 + 磁注入 30 度
#   变量 A: 重力观测是否按 PX4 口径门控 (低通|a| in [0.9,1.1]g)
#   变量 B: 磁观测 R 里是否含 (tan(dip)*sigma_tilt)^2   <- 倾角误差 2.08 倍灌进偏航的补偿
#   磁观测按固件口径: 用姿态自己的重力法向量平面投影, 取平面内方位角之差, H=[0..0,1]
import numpy as np

DT = 125e-6; T = 4.0; NS = int(T/DT)
G = np.array([0.0, 0.0, -9.80665])
DIP = np.radians(64.32); DECL = np.radians(-7.53)
B0 = np.array([np.cos(DIP)*np.cos(DECL), np.cos(DIP)*np.sin(DECL), -np.sin(DIP)])
TANDIP = abs(B0[2])/np.hypot(B0[0], B0[1])
BG_TRUE = np.array([0.5, -0.3, 0.8])*np.pi/180
BA_TRUE = np.array([0.030, -0.020, 0.015]); BB_TRUE = 2.0
SIG_MAG = np.radians(0.42); SIG_BARO = 0.30; SIG_GPS = 0.10
SIG_G = 0.006; SIG_A = 0.06
SIG_GRAV = 0.01                      # rad, PX4 的 floor = max(grav_noise,0.01)
BA_RW, BG_RW, BB_RW = 1e-4, 1e-6, 1e-3
MAG_HZ, BARO_HZ, GPS_HZ = 197.0, 7.0, 0.0   # 无 GPS，与真实设备一致
INJ = np.radians(30.0); T_INJ0, T_INJ1 = 1.5, 3.0


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


def rotz(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]])


# ---------------- 真值: 多轴同时强旋转 + 大动态加速度 ----------------
sd = 20240915
ng = np.random.default_rng(sd); na = np.random.default_rng(sd+1)
nm1 = np.random.default_rng(sd+2); nm2 = np.random.default_rng(sd+3)
nb = np.random.default_rng(sd+4); nx = np.random.default_rng(sd+5)
true_w = np.zeros((NS, 3)); true_f = np.zeros((NS, 3))
q = np.array([1.0, 0, 0, 0]); v = np.zeros(3); p = np.zeros(3)
qlog = np.zeros((NS, 4)); vlog = np.zeros((NS, 3)); plog = np.zeros((NS, 3))
A_dps = 200.0; F = (0.37, 0.53, 0.71); PH = (0.0, 1.1, 2.3)
for k in range(NS):
    tt = k*DT
    wb = np.zeros(3); anav = np.zeros(3)
    if 0.8 < tt < 3.2:
        wb = np.radians(A_dps)*np.array([np.sin(2*np.pi*F[i]*tt+PH[i]) for i in range(3)])
        anav = 0.25*9.80665*np.array([np.sin(2*np.pi*0.41*tt), np.cos(2*np.pi*0.29*tt),
                                     np.sin(2*np.pi*0.53*tt)])
    fb = q2R(q).T @ (anav - G)
    wm = wb + BG_TRUE + ng.normal(0, SIG_G, 3)
    fm = fb + BA_TRUE + na.normal(0, SIG_A, 3)
    true_w[k] = wm; true_f[k] = fm
    dq = np.array([1.0, 0.5*DT*wm[0], 0.5*DT*wm[1], 0.5*DT*wm[2]])
    q = qmul(dq, q); q /= np.linalg.norm(q)
    v = v + DT*(q2R(q) @ (fm - BA_TRUE) + G)
    p = p + DT*v
    qlog[k] = q; vlog[k] = v; plog[k] = p

# 磁实测: 真场 + 切向噪声; 注入 = 绕导航系竖直轴转 30 度
e1 = np.cross(B0, np.array([0.0, 0.0, 1.0])); e1 /= np.linalg.norm(e1)
e2 = np.cross(B0, e1)
mag_a = nm1.random(NS) < MAG_HZ*DT
mag_nav = np.zeros((NS, 3))
for k in range(NS):
    inj = INJ if T_INJ0 < k*DT < T_INJ1 else 0.0
    bn = rotz(inj) @ B0
    m = bn + nm2.normal(0, SIG_MAG)*e1 + nm2.normal(0, SIG_MAG)*e2
    mag_nav[k] = m/np.linalg.norm(m)
baro_a = nb.random(NS) < BARO_HZ*DT
baro_z = plog[:, 2] + BB_TRUE + nb.normal(0, SIG_BARO, NS)
gps_a = nx.random(NS) < GPS_HZ*DT
gps_z = vlog[:, :2] + nx.normal(0, SIG_GPS, (NS, 2))


class EKF:
    def __init__(self, gate='fw', r_tilt=True, pause=False):
        self.gate, self.rt, self.pause = gate, r_tilt, pause
        self.t_ng = 0.0; self.pn = 0; self.mn = 0
        self.x = np.zeros(17); self.x[6] = 1.0
        self.P = np.diag(np.r_[np.full(3, 0.1**2), np.full(3, 0.05**2),
                               np.full(3, np.radians(3.0)**2), np.full(3, 0.05**2),
                               np.full(3, np.radians(0.3)**2), [0.5**2]])
        self.lpf = 9.80665; self.err = []; self.gon = 0; self.n = 0; self.rfac = []

    def _upd(self, H, r, R):
        PHt = self.P @ H.T
        K = PHt @ np.linalg.inv(H @ PHt + R)
        dx = K @ r
        self.x[0:3] += dx[0:3]; self.x[3:6] += dx[3:6]
        dq = np.array([1.0, 0.5*dx[6], 0.5*dx[7], 0.5*dx[8]]); dq /= np.linalg.norm(dq)
        self.x[6:10] = qmul(dq, self.x[6:10]); self.x[6:10] /= np.linalg.norm(self.x[6:10])
        self.x[10:13] += dx[9:12]; self.x[13:16] += dx[12:15]; self.x[16] += dx[15]
        self.P = (np.eye(16) - K @ H) @ self.P
        self.P = 0.5*(self.P + self.P.T)

    def u_grav(self, fm, dt, wdps):
        n = np.linalg.norm(fm)
        if n < 1e-3: return
        self.lpf += (n - self.lpf)*(dt/0.5)
        self.n += 1
        if self.gate == 'px4':
            good = (0.9*9.80665 < self.lpf < 1.1*9.80665)          # 低通 |a| 带宽
        elif self.gate == 'fw':
            good = (abs(n/9.80665 - 1.0) < 0.06) and (wdps < 2.0)  # 固件: ±6% 且低角速率
        else:
            good = True
        if not good:
            self.t_ng += dt
            return
        self.gon += 1; self.t_ng = 0.0
        Rd = q2R(self.x[6:10])
        H = np.zeros((3, 16)); H[0:3, 6:9] = Rd.T @ skew([0.0, 0.0, 1.0])
        self._upd(H, fm/n - Rd.T @ np.array([0.0, 0.0, 1.0]), np.eye(3)*SIG_GRAV**2)

    def u_mag(self, m_nav_meas):
        """按固件口径: 投影到姿态自己的重力法向量平面, 平面内方位角之差, H=[0..0,1]"""
        Rh = q2R(self.x[6:10])
        ab = Rh.T @ np.array([0.0, 0.0, 1.0])
        m_body = Rh.T @ m_nav_meas                      # 实测场(机体系)
        fp = Rh.T @ B0                                  # 模型场(机体系)
        mh = m_body - np.dot(m_body, ab)*ab
        mh2 = fp - np.dot(fp, ab)*ab
        xb = np.array([1.0, 0.0, 0.0]); xb = xb - np.dot(xb, ab)*ab
        crs = np.cross(ab, xb)
        thm = np.arctan2(np.dot(crs, mh), np.dot(mh, xb))
        thp = np.arctan2(np.dot(crs, mh2), np.dot(mh2, xb))
        r = np.array([(thm - thp + np.pi) % (2*np.pi) - np.pi])
        st = np.sqrt(self.P[6, 6] + self.P[7, 7])       # EKF 自己的倾角不确定度
        self.mn += 1
        if self.pause and self.t_ng > 0.30:             # 倾角参考断了太久 -> 暂停磁
            self.pn += 1
            return
        var = SIG_MAG**2
        if self.rt:
            var = var + (TANDIP*st)**2
            self.rfac.append((TANDIP*st)**2/SIG_MAG**2)
        H = np.zeros((1, 16)); H[0, 8] = 1.0
        self._upd(H, r, np.array([[var]]))

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
        Gn[3:6, :] = P[3:6, :] - dt*(skew(Rd @ fb) @ P[6:9, :] + Rd @ P[9:12, :])
        Gn[6:9, :] = P[6:9, :] - dt*(Rd @ P[12:15, :])
        Gn[9:16, :] = P[9:16, :]
        Pn = Gn.copy(); Pn[:, 0:3] += dt*Gn[:, 3:6]
        Pn[:, 3:6] -= dt*(Gn[:, 6:9] @ skew(Rd @ fb).T + Gn[:, 9:12] @ Rd.T)
        Pn[:, 6:9] -= dt*(Gn[:, 12:15] @ Rd.T)
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
        self.u_grav(fm, dt, np.degrees(np.linalg.norm(wm - self.x[13:16])))
                                                         # ③ 每帧固定拉上
        if gps_a[k]:
            self.u_gps(gps_z[k])
        elif baro_a[k]:
            self.u_baro(baro_z[k])
        elif mag_a[k]:
            self.u_mag(mag_nav[k])                       # ④ 一帧一个
        qe = qmul(self.x[6:10], qconj(qlog[k]))
        if qe[0] < 0: qe = -qe
        self.err.append(2*qe[1:4])


def run(**kw):
    f = EKF(**kw)
    for k in range(NS):
        f.step(k, true_w[k], true_f[k])
    return f, np.array(f.err)


SEG = [('静止 0-0.8s', 0, int(0.8/DT)), ('旋转 0.8-1.5s', int(0.8/DT), int(1.5/DT)),
       ('旋转+注入 1.5-3.0s', int(1.5/DT), int(3.0/DT)), ('静止 3.0-4.0s', int(3.0/DT), NS)]
print('tan(dip) = b0z/b0h = %.4f' % TANDIP)
print('重力门控: PX4 口径 低通|a| in [0.9,1.1]g ; 磁 R 附加项 (tan(dip)*sigma_tilt)^2')
print()
print('%-30s | %s' % ('组合', ' | '.join('%-16s' % s[0] for s in SEG)))
print('%-30s | %s' % ('', ' | '.join('%-16s' % '倾角/偏航(度)' for s in SEG)))
RES = []
COMBOS = [('无门 + 无R项', dict(gate='none', r_tilt=False)),
          ('PX4带(0.9-1.1g) + 无R项', dict(gate='px4', r_tilt=False)),
          ('固件门(6%%,2dps) + 无R项', dict(gate='fw', r_tilt=False)),
          ('无门 + 有R项', dict(gate='none', r_tilt=True)),
          ('PX4带 + 有R项', dict(gate='px4', r_tilt=True)),
          ('固件门 + 有R项', dict(gate='fw', r_tilt=True)),
          ('固件门 + R项 + 磁暂停(0.3s)', dict(gate='fw', r_tilt=True, pause=True))]
for tag, kw in COMBOS:
    f, E = run(**kw)
    RES.append((tag, f, E))
    cells = []
    for nm, a, b in SEG:
        e = E[a:b]
        tilt = np.sqrt(np.mean(e[:, 0]**2+e[:, 1]**2))*180/np.pi
        yaw = np.sqrt(np.mean(e[:, 2]**2))*180/np.pi
        cells.append('%5.2f /%6.2f ' % (tilt, yaw))
    print('%-30s | %s' % (tag, ' | '.join(cells)))
print()
print('%-30s | %8s %10s %12s' % ('组合', '重力融合率', 'R放大p50', 'R放大max'))
for tag, f, E in RES:
    rf = np.array(f.rfac) if f.rfac else np.array([0.0])
    print('%-30s | %7.1f%% %10.2f %12.1f'
          % (tag, 100*f.gon/max(1, f.n), np.median(rf), rf.max()))
