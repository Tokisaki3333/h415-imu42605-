# -*- coding: utf-8 -*-
"""VER=47（133 列）效果判决。
关键：122 prop_ok / 124 prop_row 是否=1/17；二维观测是否在跑；
      130~132 mag_dq* 修正落在哪个轴；121 p_yy 是否被压低；死点是否触发。
"""
import glob
import os
import time

import numpy as np

P = max([c for c in glob.glob(r'R:\*.bin') if os.path.getsize(c) > 100000],
        key=os.path.getmtime)
b = np.fromfile(P, dtype='<f4')
NCH = b.size // (b.size // 133) if b.size % 133 == 0 else 122
b = b.reshape(-1, NCH).astype(np.float64)
N = b.shape[0]
dt = b[:, 25] * 1e-6
t = np.cumsum(dt)
print('文件 %s   改动 %s' % (P, time.strftime('%H:%M:%S', time.localtime(os.path.getmtime(P)))))
print('列数 %d   帧 %d   时长 %.2f s   fw_tag %d' % (NCH, N, t[-1], b[0, 76]))
assert NCH == 133 and int(b[0, 76]) == 3114247, '不是 VER=47/133 列的数据'

C = dict(gyro=(26, 29), q=(0, 4), ekf_q=(89, 93), mag_used=120, mag_bh=118, mag_r=119,
         p_yy=121, prop_ok=122, f_ok=123, prop_row=124, stage=125, mag_rej=126,
         mag_fhb=127, mag_rx=128, mag_ry=129, mag_dqx=130, mag_dqy=131, mag_dqz=132,
         gate=117, syaw=104, gb=103)


def yaw(q):
    w, x, y, z = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
    n = np.sqrt(w*w + x*x + y*y + z*z)
    w, x, y, z = w/n, x/n, y/n, z/n
    return np.degrees(np.arctan2(2*(w*z + x*y), 1 - 2*(y*y + z*z)))


def wrap(d):
    return (d + 180.0) % 360.0 - 180.0


g = np.linalg.norm(b[:, 26:29], axis=1)

# ---------- 0) 关键开关量 ----------
print('\n【0】开关量（决定"到底跑没跑"）')
for k in ('prop_ok', 'f_ok', 'prop_row', 'stage'):
    v = b[:, C[k]]
    u, c = np.unique(v.astype(np.int32), return_counts=True)
    top = sorted(zip(c, u), reverse=True)[:5]
    print('  %-9s 取值分布(top): %s' % (k, ' '.join('%d:%.1f%%' % (int(x), 100.0*nn/N) for nn, x in top)))
print('  阶段机推进: stage 覆盖 %d 个不同值（应含 0..22），prop_row 最大 %d（应 17）'
      % (len(np.unique(b[:, C['stage']])), int(b[:, C['prop_row']].max())))

# ---------- 1) 二维观测是否在跑 ----------
print('\n【1】二维地磁观测是否在跑（VER=46/47 核心）')
mu = b[:, C['mag_used']]
mrx, mry = b[:, C['mag_rx']], b[:, C['mag_ry']]
mdq = b[:, C['mag_dqx']:C['mag_dqz']+1]
print('  mag_gate 置位 %.1f%%   mag_used=1 %.1f%%' % (100*b[:, C['gate']].mean(), 100*mu.mean()))
nz = np.abs(mrx) + np.abs(mry) > 0
print('  二维新息 (rx,ry) 非零占比 %.1f%%   |rx| p50 %.3e  |ry| p50 %.3e' %
      (100*nz.mean(), np.median(np.abs(mrx)), np.median(np.abs(mry))))
print('  隐含偏航误差 mag_r: p50 %.3f  p90 %.3f  max %.2f 度' %
      (np.median(np.abs(b[:, C['mag_r']])), np.percentile(np.abs(b[:, C['mag_r']]), 90),
       np.abs(b[:, C['mag_r']]).max()))
d = np.abs(mdq).max(axis=1)
nzd = d > 0
print('  实际注入 mag_dq* 非零占比 %.1f%%' % (100*nzd.mean()))
if nzd.any():
    for i, nm in enumerate(('mag_dqx', 'mag_dqy', 'mag_dqz')):
        v = mdq[nzd, i]
        print('    %-8s p50 %+8.4f  |p50|占比 %.1f%%  max|.| %7.3f 度'
              % (nm, np.median(v), 100*np.median(np.abs(v))/max(np.median(np.abs(mdq[nzd])).sum(), 1e-12),
                 np.abs(v).max()))
    a = np.abs(mdq[nzd])
    frac = a.mean(axis=0) / max(a.mean(), 1e-12)
    print('    三轴平均占比 x/y/z = %.1f%% / %.1f%% / %.1f%%   <- z 应主导' %
          (100*frac[0], 100*frac[1], 100*frac[2]))

# ---------- 2) p_yy / sigma_yaw 是否被压低 ----------
print('\n【2】偏航不确定度（VER=46 前它会一路涨到饱和）')
pyy = b[:, C['p_yy']]
print('  p_yy: p10 %.4g  p50 %.4g  p90 %.4g  max %.4g  min %.4g' %
      (np.percentile(pyy, 10), np.median(pyy), np.percentile(pyy, 90), pyy.max(), pyy.min()))
print('  sigma_yaw: p10 %.2f  p50 %.2f  p90 %.2f  max %.2f 度' %
      tuple(np.percentile(b[:, C['syaw']], [10, 50, 90, 100])))

# ---------- 3) 死点 ----------
print('\n【3】死点（mag_bh < V5F_EKF_MAG_BH_MIN=0.30 时强制归零）')
bh = b[:, C['mag_bh']]
dead = bh < 0.30
print('  mag_bh: p1 %.4f  p10 %.4f  p50 %.4f  p90 %.4f' %
      tuple(np.percentile(bh, [1, 10, 50, 90])))
print('  mag_bh < 0.30 占比 %.3f%%   （死点帧要求 mag_used=0 且 6 个诊断全 0：%s）'
      % (100*dead.mean(),
         bool(np.all(mu[dead] == 0) and np.all(np.abs(mdq[dead]).max() == 0)) if dead.any() else 'n/a'))

# ---------- 4) 分段：静/动（用 |gyro|，并把证据一起打出来） ----------
print('\n【4】分段（按 |gyro| 中位判定，同时给出总转角自证）')
yl, ye = yaw(b[:, 0:4]), yaw(b[:, 89:93])
win = 4
for k in range(win):
    s, e = k*N//win, (k+1)*N//win
    tot = np.abs(wrap(np.diff(ye[s:e]))).sum()
    print('  %5.1f-%5.1fs  |gyro|p50 %7.1f dps  EKF总转角 %9.1f 度  mag_used %4.0f%%  |mag_r|p50 %6.2f'
          % (t[s], t[e-1], np.median(g[s:e]), tot, 100*mu[s:e].mean(), np.median(np.abs(b[s:e, C['mag_r']]))))

# ---------- 5) 地磁牵引：EKF 与旧链偏航差 ----------
print('\n【5】地磁牵引效果（EKF 相对旧链的偏航差 dyaw）')
dy = wrap(ye - yl)
print('  dyaw: p10 %+8.2f  p50 %+8.2f  p90 %+8.2f 度   首末 %+.2f -> %+.2f' %
      (np.percentile(dy, 10), np.median(dy), np.percentile(dy, 90), dy[0], dy[-1]))
print('  净转角: 旧链 %+.2f  EKF %+.2f 度   偏置 %+.2f 度' %
      (wrap(yl[-1]-yl[0]), wrap(ye[-1]-ye[0]), wrap(wrap(ye[-1]-ye[0])-wrap(yl[-1]-yl[0]))))
tot = np.abs(wrap(np.diff(ye))).sum()
print('  EKF 总转角 %.1f 度（%.1f 度/秒 平均）' % (tot, tot/t[-1]))
gb = b[:, C['gb']].astype(int)
for v, nm in [(0x20, 'tilt'), (0x40, 'mag'), (0x80, 'aligned'), (0x200, 'chi2')]:
    print('  gate bit %-8s %5.1f%%' % (nm, 100*((gb & v) != 0).mean()))
