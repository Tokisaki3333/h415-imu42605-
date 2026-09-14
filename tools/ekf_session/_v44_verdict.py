# -*- coding: utf-8 -*-
"""VER=44 判决：地磁观测改用 q_s 后，mag_bh / mag_r 是否回到物理值。

对照 VER=43 实测：mag_bh p50 0.4957(min 0.3008)、mag_r p90 59.9 max 72.0、
EKF 磁残差 +0.001 -> +54.451 度。
预测（VER=44）：mag_bh 收紧到 cos(dip) 附近且不随运动波动；mag_r 长尾大幅收缩。
"""
import numpy as np

P = r'R:\raw_v9.bin'
NCH = 122
b = np.fromfile(P, dtype='<f4').reshape(-1, NCH).astype(np.float64)
N = b.shape[0]
dt = b[:, 25] * 1e-6
t = np.cumsum(dt)

MAG_GATE, MAG_BH, MAG_R, MAG_USED, P_YY = 117, 118, 119, 120, 121
EKF_Q, EKF_SYAW, EKF_GB = 89, 104, 103
GYRO = slice(26, 29)
PSI_MAG, PSI_TRUE = 57, 58

print('=' * 78)
print('VER=44 判决   fw_tag %.0f   帧 %d   时长 %.2f s' % (b[0, 76], N, t[-1]))
print('=' * 78)

g = np.linalg.norm(b[:, GYRO], axis=1)
gs = np.median(np.lib.stride_tricks.sliding_window_view(
    np.pad(g, 50, mode='edge'), 101), axis=1)

seg = np.where(gs < 5.0, 0, np.where(gs < 200.0, 1, 2))
names = ['静止  ', '慢转  ', '快转  ']
tot = len(seg)


def q2yaw(q):
    w, x, y, z = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
    return np.degrees(np.arctan2(2.0 * (w * z + x * y),
                                 1.0 - 2.0 * (y * y + z * z)))


def wrap(d):
    return (d + 180.0) % 360.0 - 180.0


print('\n【一】分段统计（按陀螺模值分段：<5 / 5~200 / >200 dps）')
print('  段     占比   陀螺p50  bh_p10  bh_p50  bh_p90 | r_p50   r_p90   r_max  |r|>10 |r|>45'
      '  gate  used  p_yy_p50  syaw_p50')
for s, nm in enumerate(names):
    m = seg == s
    if m.sum() < 50:
        print('  %s  段内帧数不足(%d)' % (nm, m.sum()))
        continue
    r = b[m, MAG_R]
    gb = b[m, EKF_GB].astype(int)
    print('  %s %5.1f%%  %6.1f  %6.4f  %6.4f  %6.4f | %6.3f  %7.3f  %7.2f  %5.1f%%  %5.1f%%'
          '  %4.0f%%  %4.0f%%  %.3e  %6.2f' % (
              nm, 100.0 * m.mean(), np.median(g[m]),
              np.percentile(b[m, MAG_BH], 10), np.median(b[m, MAG_BH]),
              np.percentile(b[m, MAG_BH], 90),
              np.median(np.abs(r)), np.percentile(np.abs(r), 90), np.abs(r).max(),
              100.0 * np.mean(np.abs(r) > 10.0), 100.0 * np.mean(np.abs(r) > 45.0),
              100.0 * np.mean((gb & 0x40) != 0), 100.0 * b[m, MAG_USED].mean(),
              np.median(b[m, P_YY]), np.median(b[m, EKF_SYAW])))

print('\n【二】全场对照 VER=43')
r = b[:, MAG_R]
print('  mag_bh  p10 %.4f  p50 %.4f  p90 %.4f  min %.4f      (43: p50 0.4957 min 0.3008)'
      % (np.percentile(b[:, MAG_BH], 10), np.median(b[:, MAG_BH]),
         np.percentile(b[:, MAG_BH], 90), b[:, MAG_BH].min()))
print('  mag_r   p50 %.3f  p90 %.3f  max %.3f        (43: p50 0.768 p90 59.94 max 71.998)'
      % (np.median(np.abs(r)), np.percentile(np.abs(r), 90), np.abs(r).max()))
print('  |r|>45  %.2f%%   |r|>10  %.2f%%   |r|==0  %.2f%%'
      % (100.0 * np.mean(np.abs(r) > 45), 100.0 * np.mean(np.abs(r) > 10),
         100.0 * np.mean(r == 0)))
print('  mag_gate %.2f%%   mag_used %.2f%%   p_yy p50 %.3e (43: 5.79e-02)'
      % (100.0 * b[:, MAG_GATE].mean(), 100.0 * b[:, MAG_USED].mean(),
         np.median(b[:, P_YY])))

print('\n【三】偏航牵引：EKF 偏航 vs 地磁观测（|r| 即 EKF 的偏航误差）')
yq = q2yaw(b[:, EKF_Q:EKF_Q + 4])
for s, nm in enumerate(names):
    m = seg == s
    if m.sum() < 50:
        continue
    rr = np.abs(b[m, MAG_R])
    print('  %s  偏航净变化 %8.2f 度   |r| p50 %6.3f  p90 %7.3f  <2度占比 %5.1f%%'
          % (nm, wrap(yq[m][-1] - yq[m][0]), np.median(rr),
             np.percentile(rr, 90), 100.0 * np.mean(rr < 2.0)))

print('\n【四】总转角（陀螺跟踪能力，与磁无关）')
d = wrap(np.diff(yq))
print('  EKF 偏航总转角 %9.2f 度    净变化 %8.2f 度' % (np.abs(d).sum(), wrap(yq[-1] - yq[0])))
print('  旧链 psi_mag 净变化 %8.2f 度   psi_true 净变化 %8.2f 度'
      % (wrap(b[-1, PSI_MAG] - b[0, PSI_MAG]), wrap(b[-1, PSI_TRUE] - b[0, PSI_TRUE])))

print('\n【五】失效可观测：门关时 sigma_yaw 是否变大')
gb = b[:, EKF_GB].astype(int)
for bit, nm in [(0x40, 'mag_yaw'), (0x200, 'chi2_rej'), (0x800, 'acc_sat')]:
    on = (gb & bit) != 0
    if on.sum() > 20 and (~on).sum() > 20:
        print('  %-9s 置位 %5.1f%%   sigma_yaw 置位时 p50 %6.2f / 未置位 %6.2f'
              % (nm, 100.0 * on.mean(), np.median(b[on, EKF_SYAW]),
                 np.median(b[~on, EKF_SYAW])))
