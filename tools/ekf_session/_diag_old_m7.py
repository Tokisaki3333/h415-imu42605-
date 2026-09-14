# -*- coding: utf-8 -*-
"""离线重算旧公式：Bn_old = R(q_now) * Exp(-dth) * f_s，dth = 边沿起累积的 w*dt。

目的：判定旧代码为什么能给出小残差（VER=43 mag_used 69.89%），
      而新代码（数学上应当等价的 R(q_s) f_s）给出 170 度。
如果 r_old ~= 0 而 dth 很大 -> 旧的"去陈旧"实际在抵消一个大系统偏差，
说明 **磁样本与姿态之间存在固定的时间/轴向错配**，不是"相加/复合"那种二阶问题。
"""
import numpy as np

b = np.fromfile(r'R:\raw_v9.bin', dtype='<f4').reshape(-1, 122).astype(np.float64)
D = -7.53
dt = b[:, 25] * 1e-6
t = np.cumsum(dt)
w = b[:, 26:29] * np.pi / 180.0          # gyro_dps -> rad/s
f = b[:, 42:45].copy()
ic = b[:, 41].astype(np.int64)
q = b[:, 89:93]


def q2R(qq):
    ww, xx, yy, zz = qq[:, 0], qq[:, 1], qq[:, 2], qq[:, 3]
    n = np.sqrt(ww**2 + xx**2 + yy**2 + zz**2)
    ww, xx, yy, zz = ww/n, xx/n, yy/n, zz/n
    R = np.empty((len(qq), 3, 3))
    R[:, 0, 0] = 1-2*(yy*yy+zz*zz); R[:, 0, 1] = 2*(xx*yy-ww*zz); R[:, 0, 2] = 2*(xx*zz+ww*yy)
    R[:, 1, 0] = 2*(xx*yy+ww*zz); R[:, 1, 1] = 1-2*(xx*xx+zz*zz); R[:, 1, 2] = 2*(yy*zz-ww*xx)
    R[:, 2, 0] = 2*(xx*zz-ww*yy); R[:, 2, 1] = 2*(yy*zz+ww*xx); R[:, 2, 2] = 1-2*(xx*xx+yy*yy)
    return R


def wrap(d):
    return (d + 180.0) % 360.0 - 180.0


# ---- 边沿检测 ----
edge = np.flatnonzero(np.diff(ic) != 0) + 1
print('IST 边沿数 %d   平均间隔 %.4f s (%.1f Hz)   总时长 %.2f s'
      % (len(edge), np.mean(np.diff(edge)) * np.median(dt), 1.0 / (np.mean(np.diff(edge)) * np.median(dt)), t[-1]))

# ---- 逐步累积 dth（每个边沿清零），完全按固件写法 ----
dth = np.zeros((len(b), 3))
acc = np.zeros(3)
e = set(edge.tolist())
for i in range(len(b)):
    if i in e:
        acc = np.zeros(3)
    acc = acc + w[i] * dt[i]
    dth[i] = acc
am = np.linalg.norm(dth, axis=1)
print('dth 模（度）: p50 %.3f  p90 %.3f  max %.3f' % tuple(np.percentile(np.degrees(am), [50, 90, 100])))

# ---- Exp(-dth) 按固件写法（用"相加的轴角"构造四元数） ----
half = 0.5 * am
safe = np.where(am > 1e-4, am, 1.0)
s = np.where(am > 1e-4, np.sin(half) / safe, 0.0)
dq = np.stack([np.cos(half), -s*dth[:, 0], -s*dth[:, 1], -s*dth[:, 2]], axis=1)

Rn = q2R(q)


def rot(qq, v):
    Rq = q2R(qq)
    return np.einsum('nij,nj->ni', Rq, v)


Rq = q2R(dq)
f_old = np.einsum('nij,nj->ni', Rq, f)
Bn_old = np.einsum('nij,nj->ni', Rn, f_old)
Bn_new = np.einsum('nij,nj->ni', Rn, f)

r_old = wrap(D - np.degrees(np.arctan2(Bn_old[:, 0], Bn_old[:, 1])))
r_new = wrap(D - np.degrees(np.arctan2(Bn_new[:, 0], Bn_new[:, 1])))
r_fw = b[:, 119]

g = np.linalg.norm(b[:, 26:29], axis=1)
rest = g < 5.0

print('\n  |r| 静止p50  全场p50   p90    max')
for nm, r in [('旧公式 Exp(-dth)', r_old), ('新公式 R(q_s)f_s', r_new), ('固件 119 列', r_fw)]:
    print('  %-16s %8.3f %8.3f %7.2f %7.2f'
          % (nm, np.median(np.abs(r[rest])), np.median(np.abs(r)), np.percentile(np.abs(r), 90), np.abs(r).max()))

print('\n  校验：新公式 vs 固件 119 之差 p50 %.3f 度' % np.median(np.abs(wrap(r_new - r_fw))))
print('  旧公式 |r|<45 占比 %.2f%%   （VER=43 实测 mag_used 69.89%%，chi2_rej 从未置位）'
      % (100.0 * np.mean(np.abs(r_old) < 45.0)))
print('  新公式 |r|<45 占比 %.2f%%' % (100.0 * np.mean(np.abs(r_new) < 45.0)))

# dth 是否与真实转动量一致？静止段应当很小
print('\n  dth 模在静止段 p50 %.4f 度（应接近 0）' % np.degrees(np.median(am[rest])))
