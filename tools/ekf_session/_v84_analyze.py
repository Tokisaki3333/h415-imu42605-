# -*- coding: utf-8 -*-
# VER=84 运行效果分析：磁门控 / R 放大 / 注入分解（泄漏）/ 重力观测门与牵引
# 列号（0基，NCH=154 起）:
#   8 is_static | 25 dt_us | 76 fw_tag | 89-92 EKF q | 0-3 旧姿态 q
#   119 mag_r(度) | 135/136/137 mag_dq x/y/z(度) | 138/139/140 tilt_dq x/y/z(度)
#   144 thm | 145 thp | 146 amn | 147 mhn | 148 mag_age_ms
#   149 mag_rs | 150 tilt_inv_ms | 151 mag_hold | 152 grav_ok | 153 grav_nis
#   106 sigma_tilt_deg（既有列）
import sys
import numpy as np

P = sys.argv[1] if len(sys.argv) > 1 else r'R:\raw_v9.bin'
raw = np.fromfile(P, dtype='<f4')
NCH = 154
N = raw.size // NCH
# 自适应：若帧数不是整数，按 148/149/154 试
for nch in (154, 149, 148):
    if raw.size % nch == 0 and float(raw[76]) in (5544455.0, 5544711.0, 5477639.0, 5411847.0):
        NCH = nch; N = raw.size // nch; break
b = raw[:N*NCH].reshape(-1, NCH).astype(np.float64)
t = np.cumsum(b[:, 25]*1e-6)
D = 180/np.pi
print('文件 %s   NCH=%d   帧 %d   %.2f s   fw_tag %d (VER=%d)'
      % (P, NCH, N, t[-1], b[0, 76], int(b[0, 76]) >> 16))

mag_dq = b[:, 135:138]; tilt_dq = b[:, 138:141]
mag_tilt = np.hypot(mag_dq[:, 0], mag_dq[:, 1])
mag_yaw = np.abs(mag_dq[:, 2])
til_tilt = np.hypot(tilt_dq[:, 0], tilt_dq[:, 1])

print()
print('=== 1) 磁更新分解（只在真更新的帧上: |mag_dqz|>0 或 mag_tilt>0）===')
up = (mag_tilt > 0) | (mag_yaw > 0)
print('  磁更新帧 %d (%.1f%%)' % (up.sum(), 100*up.mean()))
if up.sum():
    print('  单次注入: 倾斜 |dq_xy| p50 %.4f p99 %.4f max %.4f 度'
          % (np.percentile(mag_tilt[up], 50), np.percentile(mag_tilt[up], 99), mag_tilt[up].max()))
    print('            偏航 |dq_z|  p50 %.4f p99 %.4f max %.4f 度'
          % (np.percentile(mag_yaw[up], 50), np.percentile(mag_yaw[up], 99), mag_yaw[up].max()))
    r = mag_tilt[up]/np.maximum(mag_yaw[up], 1e-9)
    print('  倾斜/偏航 注入比 p50 %.5f p99 %.5f max %.4f  <- 直接泄漏比'
          % (np.percentile(r, 50), np.percentile(r, 99), r.max()))
print('  累计: 倾斜 Σ|dq_xy| %.3f 度   偏航 Σ|dq_z| %.3f 度   比值 %.5f'
      % (mag_tilt.sum(), mag_yaw.sum(), mag_tilt.sum()/max(1e-9, mag_yaw.sum())))

print()
print('=== 2) 重力观测（M6 牵引）===')
gt = (til_tilt > 0)
print('  重力更新帧 %d (%.1f%%)' % (gt.sum(), 100*gt.mean()))
if gt.sum():
    print('  单次注入 倾斜 |dq_xy| p50 %.4f p99 %.4f max %.4f 度'
          % (np.percentile(til_tilt[gt], 50), np.percentile(til_tilt[gt], 99), til_tilt[gt].max()))
    print('  累计 Σ|dq_xy| %.3f 度' % til_tilt.sum())

if NCH >= 149:
    print()
    print('=== 3) VER=84 门控量 ===')
    rs = b[:, 149]; ti = b[:, 150]; mh = b[:, 151]; go = b[:, 152]; gn = b[:, 153]
    print('  mag_rs (R放大)     : p50 %.3f p95 %.3f p99 %.3f max %.2f' %
          tuple(np.percentile(rs, [50, 95, 99, 100])))
    print('  tilt_inv_ms (失效) : p50 %.2f p95 %.2f p99 %.2f max %.1f'
          % tuple(np.percentile(ti, [50, 95, 99, 100])))
    print('  mag_hold 占比      : %.1f%%' % (100*np.mean(mh > 0)))
    print('  grav_ok 占比       : %.1f%%' % (100*np.mean(go > 0)))
    print('  grav_nis           : p50 %.3f p95 %.3f p99 %.3f'
          % tuple(np.percentile(gn, [50, 95, 99])))
    print('  停磁时 tilt_inv_ms p50 %.1f 度 ; 不停磁时 p50 %.1f'
          % (np.median(ti[mh > 0]) if (mh > 0).any() else -1, np.median(ti[mh == 0])))
    if NCH >= 107:
        st = b[:, 106]
        k = np.hypot(b[:, 6], b[:, 7])*0  # 占位
        pred = (2.08*st*D/0.5)**2
        m = rs > 0
        if m.any():
            print('  mag_rs vs 预测 (2.08*sigma_tilt/0.5度)^2 相关 %.3f'
                  % np.corrcoef(rs[m], pred[m])[0, 1])
    # 运动/静止分段
    stat = (b[:, 8] == 1)
    print()
    print('=== 4) 运动 vs 静止 ===')
    for nm, m in (('静止', stat), ('运动', ~stat)):
        if m.sum() < 100:
            continue
        print('  %s: mag_rs p50 %.2f | tilt_inv_ms p50 %.1f | mag_hold %.1f%% | grav_ok %.1f%% | '
              '磁倾斜注入 p50 %.5f 度 | 重力注入 p50 %.5f 度'
              % (nm, np.median(rs[m]), np.median(ti[m]), 100*np.mean(mh[m] > 0),
                 100*np.mean(go[m] > 0), np.median(mag_tilt[m]), np.median(til_tilt[m])))
