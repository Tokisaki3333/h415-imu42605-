# -*- coding: utf-8 -*-
"""地磁在运动中"拉正姿态"的占空比与力度。
列号(=解析器-2, 已用 fw_tag/mag.f/EKF q/mag_dqz 锚点校验):
  117 ekf.mag_gate  119 mag_r_deg  120 mag_used  127 mag_rej
  135-137 mag_dq{x,y,z}  144 mag_cmp_thm  145 mag_cmp_thp  146 mag_cmp_amn  147 mag_cmp_mhn
  148 mag_age_ms  151 mag_hold  152 grav_ok  154 mag.ok  155 gate.ekf_mag_yaw
  156 gate.ekf_tilt  157 ekf.mag_gate  158 ekf.mag_used  16 flags  89-92 EKF q
EKF 区列是 ~196Hz 零阶保持 -> 全部按"跳变帧"去重后再求和。
"""
import numpy as np, os, glob, math
np.seterr(all='ignore')
P = sorted(glob.glob(r'R:\raw*.bin'), key=os.path.getmtime, reverse=True)[0]
sz = os.path.getsize(P); NCH = 162 if (sz//4) % 162 == 0 else 159
raw = np.fromfile(P, dtype='<f4').reshape(-1, NCH)
N = raw.shape[0]
c = lambda i: raw[:, i].astype(np.float64)
c3 = lambda a, b: raw[:, a:b].astype(np.float64)
print('列校验: tag76=%.0f (VER %d CH %d)  dt25 中位 %.2f  |f42-44| %.3f  col45 %.3f  |qEKF89-92| %.4f' %
      (np.median(c(76)), int(np.median(c(76))) >> 16, (int(np.median(c(76))) >> 8) & 0xFF,
       np.median(c(25)), np.median(np.linalg.norm(c3(42, 45), axis=1)), np.median(c(45)),
       np.median(np.linalg.norm(c3(89, 93), axis=1))))
dt = c(25).copy(); bad = ~np.isfinite(dt) | (dt < 10) | (dt > 50000); dt[bad] = 124.58
t = np.cumsum(dt)*1e-6
tag = c(76); okr = (tag == np.median(tag))
gy = np.linalg.norm(c3(26, 29), axis=1); an = np.linalg.norm(c3(32, 35), axis=1)
clip = (np.abs(c3(18, 21)).max(axis=1) >= 32700)
gate_yaw = c(155); gate_tilt = c(156); ekf_gate = c(157); ekf_used = c(158)
magok = c(154); magused120 = c(120); grav = c(152); hold = c(151); reg = c(127)
dq = c3(135, 138)
thm = c(144); thp = c(145); amn = c(146); mhn = c(147); age = c(148); rdeg = c(119)

seg = [('启动静置', 0.0, 2.9), ('第一段运动(含削顶)', 2.9, 27.4), ('中间静置', 26.4, 29.2),
       ('第二段运动(无削顶)', 29.2, 52.7), ('尾部静置', 53.5, 57.3)]
print('\n%-22s %7s %6s | 外门ekf_mag_yaw  内门ekf_mag_gate  实际执行mag_used | 上报mag_used  mag.ok  tilt门 grav_ok' % ('段落', '时长s', '帧数'))
tot = {}
for nm, t0, t1 in seg:
    m = (t >= t0) & (t < t1)
    n = m.sum()
    if n < 100: continue
    f = lambda v: 100.0*np.mean(v[m] > 0.5)
    print('%-22s %7.2f %6d | %14.1f%% %16.1f%% %16.1f%% | %11.1f%% %6.1f%% %6.1f%% %6.1f%%' %
          (nm, t1-t0, n, f(gate_yaw), f(ekf_gate), f(ekf_used), f(magused120), f(magok), f(gate_tilt), f(grav)))
    tot[nm] = n

# ---- 去重后的 EKF 段 (磁更新只在跳变帧发生) ----
new = np.zeros(N, bool); new[1:] = (np.abs(np.diff(dq, axis=0)).sum(1) > 1e-12) | (np.abs(np.diff(thm)) > 1e-12)
print('\n%-22s %8s %10s %12s %14s %12s %12s' % ('段落', 'M7更新数', '更新率/s', 'Σ|dq|全部°', 'Σ|dqz|°/s', '|创新|中位°', '创新p90°'))
for nm, t0, t1 in seg:
    m = (t >= t0) & (t < t1) & new
    n = m.sum()
    if n < 20: continue
    span = t1-t0
    d = np.linalg.norm(dq[m], axis=1)
    inn = (thm[m]-thp[m]+180.0) % 360.0-180.0
    print('%-22s %8d %10.1f %12.1f %14.1f %12.3f %12.3f' %
          (nm, n, n/span, np.degrees(d.sum()), np.degrees(np.abs(dq[m, 2]).sum())/span,
           np.median(np.abs(inn)), np.percentile(np.abs(inn), 90)))

# ---- 权重: 磁偏航权限花在哪里 ----
mot = ((t >= 2.9) & (t < 27.4)) | ((t >= 29.2) & (t < 52.7))
sta = ~mot
nm_ = new & mot; ns_ = new & sta
A_m = np.degrees(np.abs(dq[nm_, 2]).sum()); A_s = np.degrees(np.abs(dq[ns_, 2]).sum())
print('\n偏航修正总量: 运动期 %.1f° (%.1f%%), 静止/停歇期 %.1f° (%.1f%%) ; 运动时长占 %.1f%%' %
      (A_m, 100*A_m/(A_m+A_s), A_s, 100*A_s/(A_m+A_s), 100*mot.mean()))
print('运动期偏航修正速率 %.2f °/s ; 静止期 %.2f °/s' %
      (A_m/(t[mot][-1]-t[mot][0]), A_s/(t[sta][-1]-t[sta][0])))
print('\n=== 修正是"系统拉正"还是"追噪声"? (签名和 vs 绝对值和) ===')
print('%-22s %10s %10s %10s %10s %10s' % ('段落', 'Σ|dqz|°', '|Σdqz|°', '相干性', 'Σ|dqxy|°', '|Σdqxy|°'))
for nm, t0, t1 in seg:
    m = (t >= t0) & (t < t1) & new
    if m.sum() < 20: continue
    az = np.degrees(np.abs(dq[m, 2]).sum()); sz = np.degrees(abs(dq[m, 2].sum()))
    axy = np.degrees(np.linalg.norm(dq[m, :2], axis=1).sum())
    sxy = np.degrees(np.linalg.norm(dq[m, :2].sum(0)))
    print('%-22s %10.1f %10.1f %9.3f %10.1f %10.1f' % (nm, az, sz, sz/max(az, 1e-9), axy, sxy))
# ---- 创新是否在被拉下去: 看 |创新| 的时间趋势 (分段 5s) ----
print('\n|创新|(实测航向-预测航向) 的 5s 中位, 运动段:')
for t0 in np.arange(2.9, 52.7, 5.0):
    m = (t >= t0) & (t < t0+5) & new
    if m.sum() < 20: continue
    inn = (thm[m]-thp[m]+180.0) % 360.0-180.0
    md = (t >= t0) & (t < t0+5)
    print('  %5.1f-%5.1f s  |w|中位%6.0f  |创新|中位 %6.2f°  p90 %6.2f°  执行率 %5.1f%%  门开率 %5.1f%%  R %5.1f°  剔磁 %d' %
          (t0, t0+5, np.median(gy[md]), np.median(np.abs(inn)), np.percentile(np.abs(inn), 90),
           100*np.mean(ekf_used[md] > 0.5), 100*np.mean(gate_yaw[md] > 0.5), np.median(rdeg[md]), reg[md].max()))
