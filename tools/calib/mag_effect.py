# -*- coding: utf-8 -*-
r"""地磁修正作用分析：是否**均匀分布在整个运动过程**、是否**起正面作用**。

没有绝对参考时的判据（只用"世界系磁场方向应恒定"这一物理不变量）：
  * 一致性：u = R(q)·(A·m_lsb + C) 的方向散布。用 EKF 姿态算 => 磁+EKF 的一致性；
    用旧链 att.q 算 => 旧链自身误差（对照组）。两者对比 = 地磁到底做了多少功。
  * 均匀性：把上述量按时间（每秒）与按角速率分档统计，看有没有"地磁没管"的时段。
  * 方向分解：施加的 mag_dq（体坐标旋转矢量，度）投影到**重力方向**（yaw 分量）
    与其正交方向（tilt 分量）——置信度加权下 tilt 应被压小。
  * 符号平衡：Σyaw vs Σ|yaw|。≈0 纠随机误差；→1 单方向纠偏（打架）。
  * 模长/削顶：mag_norm 与 clip 位随 |w| 的分布（与姿态无关的量测健康度）。

用法: python tools/calib/mag_effect.py R:\imu_xxx.bin [每秒行数上限]
"""
import sys
import numpy as np

sys.path.insert(0, r'C:\Users\33\Documents\v2\h415-imu42605-\tools\calib')
import cols_162 as C
import mag360_cal as M
import mag360_refcheck as RC

c = C.CH_162
path = sys.argv[1] if len(sys.argv) > 1 else r'R:\imu_20260921_030649.bin'
nmax = int(sys.argv[2]) if len(sys.argv) > 2 else 40

a, info = C.load_frames(path)
rep = C.frame_report(a)
A, Cv = M.read_current_AC()
dt = a[:, c['dt_us']].astype(float) * 1e-6
t = np.cumsum(dt)
w = np.linalg.norm(a[:, c['gyro_dps0']:c['gyro_dps0'] + 3].astype(float), axis=1)
clip = ((a[:, c['flags']].astype(int) >> 12) & 1).astype(bool)
deg = 180.0 / np.pi


def worldfield(qcol):
    q = a[:, c[qcol]:c[qcol] + 4].astype(float)
    m = a[:, c['mag_lsb0']:c['mag_lsb0'] + 3].astype(float)
    y = (A @ m.T + Cv[:, None]).T
    R = np.stack([M.quat_to_R(r) for r in q])
    u = np.einsum('nij,nj->ni', R, y)
    return u / np.linalg.norm(u, axis=1, keepdims=True)


u_ekf, u_leg = worldfield('ekf_q0'), worldfield('att_q0')
mb_e = u_ekf.mean(0); mb_e /= np.linalg.norm(mb_e)
mb_l = u_leg.mean(0); mb_l /= np.linalg.norm(mb_l)
d_e = np.degrees(np.arccos(np.clip(u_ekf @ mb_e, -1, 1)))
d_l = np.degrees(np.arccos(np.clip(u_leg @ mb_l, -1, 1)))

g = a[:, c['accel_g0']:c['accel_g0'] + 3].astype(float)
gn = g / np.linalg.norm(g, axis=1, keepdims=True)
dqv = a[:, c['ekf_mag_dqx']:c['ekf_mag_dqz'] + 1].astype(float)
yawc = np.einsum('ni,ni->n', dqv, gn)
tilt = np.sqrt(np.maximum(np.einsum('ni,ni->n', dqv, dqv) - yawc * yawc, 0.0))
used = a[:, c['ekf_mag_used']] > 0.5

print('==== %s ====' % path)
print('  帧 %d  %.1f s (%.0f Hz)  fw_tag %.0f -> VER=%d  校验和 %.0f%%'
      % (len(a), t[-1], len(a) / t[-1], rep['fw_tag'], rep['ver'], 100 * rep['checksum_ok']))
print('  |w| p50 %.0f p90 %.0f p99 %.0f max %.0f dps   行程 %.0f deg   削顶 %d 帧 (%.1f%%)   坏浮点 %d'
      % (np.percentile(w, 50), np.percentile(w, 90), np.percentile(w, 99), w.max(),
         float(np.sum(w * dt)), int(clip.sum()), 100 * clip.mean(),
         int(((a[:, c['flags']].astype(int) >> 11) & 1).sum())))
print('  ★ 世界系场方向散布（物理不变量）: EKF p50 %.2f p90 %.2f max %.2f | 旧链 p50 %.2f p90 %.2f max %.2f'
      % (np.percentile(d_e, 50), np.percentile(d_e, 90), d_e.max(),
         np.percentile(d_l, 50), np.percentile(d_l, 90), d_l.max()))
print('  ★ 修正（仅执行更新的 %d/%d 帧）: |dq| 均值 %.4f 中位 %.4f max %.4f 度/次'
      % (used.sum(), len(a), np.linalg.norm(dqv[used], axis=1).mean(),
         np.median(np.linalg.norm(dqv[used], axis=1)), np.linalg.norm(dqv[used], axis=1).max()))
print('     |yaw 分量| 均值 %.4f  Σ|yaw| %.1f 度 | |tilt 分量| 均值 %.4f  Σ|tilt| %.1f 度 (能量比 %.3f)'
      % (np.abs(yawc[used]).mean(), np.abs(yawc[used]).sum(), tilt[used].mean(), tilt[used].sum(),
         (tilt[used] ** 2).sum() / max((yawc[used] ** 2).sum(), 1e-12)))
print('     符号平衡 Σyaw %+.1f / Σ|yaw| %.1f -> 相干比 %.3f（≈0 纠随机；→1 单调偏）'
      % (yawc.sum(), np.abs(yawc).sum(), abs(yawc.sum()) / max(np.abs(yawc).sum(), 1e-9)))
print()
print('---- 每秒：活动度/一致性 ----')
print('  t[s] |w|max 行程 used%   θ中位  NIS中位  |dq|均  EKF散布p90 旧链散布p90  mag_rs中位')
sec = np.floor(t).astype(int)
for s in range(0, min(sec[-1] + 1, nmax)):
    k = sec == s
    if k.sum() < 10:
        continue
    print('  %4d %6.0f %5.0f %5.0f  %6.2f %7.2f %6.3f    %7.2f     %8.2f     %8.2f'
          % (s, w[k].max(), np.sum(w[k] * dt[k]), 100 * used[k].mean(),
             np.median(a[k, c['ekf_mag_r_deg']]), np.median(a[k, c['ekf_nis0'] + 4]),
             np.linalg.norm(dqv[k], axis=1)[used[k]].mean() if used[k].any() else 0.0,
             np.percentile(d_e[k], 90), np.percentile(d_l[k], 90),
             np.median(a[k, c['ekf_mag_rs']])))
print()
print('---- 按角速率分档 ----')
print('  |w| 档         n  used%   θ中位  |dq|均  EKF散布p90 旧链散布p90  mag_norm中位  mag_rs中位')
qs = [0, 50, 100, 200, 400, 800, 1600, 4000]
for i in range(len(qs) - 1):
    k = (w >= qs[i]) & (w < qs[i + 1])
    if k.sum() < 200:
        continue
    print('  %4.0f-%4.0f %6d  %5.0f  %6.2f %6.3f     %7.2f     %8.2f      %.4f     %8.2f'
          % (qs[i], qs[i + 1], k.sum(), 100 * used[k].mean(),
             np.median(a[k, c['ekf_mag_r_deg']]),
             np.linalg.norm(dqv[k], axis=1)[used[k]].mean() if used[k].any() else 0.0,
             np.percentile(d_e[k], 90), np.percentile(d_l[k], 90),
             np.median(a[k, c['mag_norm']]), np.median(a[k, c['ekf_mag_rs']])))
