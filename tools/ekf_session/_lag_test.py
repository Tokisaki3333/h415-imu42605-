# -*- coding: utf-8 -*-
# 检验"用旧磁数据修正新陀螺仪"：若磁样本滞后 dt_lag，则
#   r(t) = thm(t) - thp(t) ~= psi(t-dt_lag) - psi(t) = -[psi(t)-psi(t-dt_lag)]
# 用**旧姿态偏航**(纯陀螺, 非循环)当 psi 的真值，在 dt_lag 网格上回归 r ~ -dpsi(dt_lag)。
# 若某个 dt_lag>0 处 R^2 明显最高且斜率≈1，则滞后成立，且幅度=该 dt_lag。
import numpy as np
P = r'R:\raw_v9.bin'; NCH = 148
raw = np.fromfile(P, dtype='<f4'); N = raw.size // NCH
b = raw[:N*NCH].reshape(-1, NCH).astype(np.float64)
dt = b[:, 25]*1e-6; t = np.cumsum(dt)
D = 180/np.pi
yawL = np.unwrap(np.arctan2(2*(b[:, 0]*b[:, 3]+b[:, 1]*b[:, 2]),
                            1-2*(b[:, 2]**2+b[:, 3]**2)))*D
yawE = np.unwrap(np.arctan2(2*(b[:, 89]*b[:, 92]+b[:, 90]*b[:, 91]),
                            1-2*(b[:, 91]**2+b[:, 92]**2)))*D
r = (b[:, 144]-b[:, 145]+180) % 360-180          # 带符号新息(度)
# 机体角速率(由旧姿态差分)
q = b[:, 0:4].copy(); q /= np.linalg.norm(q, axis=1)[:, None]
qc = q.copy(); qc[:, 1:] *= -1
def qmul(a, c):
    w1, x1, y1, z1 = a[:, 0], a[:, 1], a[:, 2], a[:, 3]
    w2, x2, y2, z2 = c[:, 0], c[:, 1], c[:, 2], c[:, 3]
    return np.stack([w1*w2-x1*x2-y1*y2-z1*z2, w1*x2+x1*w2+y1*z2-z1*y2,
                     w1*y2-x1*z2+y1*w2+z1*x2, w1*z2+x1*y2-y1*x2+z1*w2], 1)
dqq = qmul(qc[:-1], q[1:])
om = np.concatenate([[0.0], 2*np.degrees(np.linalg.norm(dqq[:, 1:], axis=1))/dt[1:]])
# 只在 EKF 周期帧上取(上报按周期锁存)；用 yawE 的变化点
cyc = np.concatenate([[True], (np.abs(np.diff(b[:, 89]))+np.abs(np.diff(b[:, 90]))
                               + np.abs(np.diff(b[:, 91]))+np.abs(np.diff(b[:, 92]))) > 0])
ci = np.where(cyc)[0]
print('EKF 周期数 %d (%.1f Hz)' % (len(ci), len(ci)/t[-1]))

motion = om[ci] > 5.0
static = om[ci] <= 1.0
print('周期中 运动(|w|>5dps) %d 个 ; 静止(|w|<=1dps) %d 个' % (motion.sum(), static.sum()))
print()
print('=== 回归  r  ~  -[yawL(t) - yawL(t-dt_lag)]  (旧姿态偏航为真值) ===')
print('  dt_lag(ms)   运动R2    运动斜率   静止R2   静止斜率   |  用EKF偏航(循环)R2')
for lag in [0, 2, 5, 10, 15, 20, 30, 40, 60, 80, 100, 150, 200, 300]:
    j = np.searchsorted(t, t[ci]-lag*1e-3)
    j = np.clip(j, 0, N-1)
    pL = -(yawL[ci]-yawL[j]); pE = -(yawE[ci]-yawE[j])
    def fit(m, p):
        if m.sum() < 50: return float('nan'), float('nan')
        x, y = p[m], r[ci][m]
        A = np.vstack([x, np.ones_like(x)]).T
        sol, *_ = np.linalg.lstsq(A, y, rcond=None)
        pred = A@sol
        ss = 1-np.sum((y-pred)**2)/max(1e-9, np.sum((y-y.mean())**2))
        return ss, sol[0]
    s2m, k2m = fit(motion, pL)
    s2s, k2s = fit(static, pL)
    s2e, _ = fit(motion, pE)
    print('  %8.0f  %8.4f  %9.3f  %8.4f  %9.3f   |  %8.4f' % (lag, s2m, k2m, s2s, k2s, s2e))
print()
print('=== 不滞后(0ms)时 r 与实际偏航变化的关系(运动帧) ===')
print('  同时给: r 与"本周期偏航变化"的相关(应最强若滞后=1个周期)')
for n in [1, 2, 3, 5, 10, 20]:
    dy = yawL[ci]-yawL[np.clip(ci-n, 0, N-1)]
    m = motion
    print('  过去 %2d 个周期(=%.1f ms): corr(r,-dy)=%+.3f  斜率 %+.3f'
          % (n, n*np.median(np.diff(t[ci]))*1e3,
             np.corrcoef(-dy[m], r[ci][m])[0, 1],
             np.linalg.lstsq(np.vstack([-dy[m], np.ones(m.sum())]).T, r[ci][m], rcond=None)[0][0]))
