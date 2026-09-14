# -*- coding: utf-8 -*-
"""死区到底吃掉了什么：逐运动段比较
     |a_lin| = 旧链（**不含死区**）的残余比力 -> 真实线性加速度的独立度量
     |a_nav| = EKF 输出（**含死区**）
     td = K*g*sigma_tilt -> 死区门限
   若 |a_nav| << |a_lin| 且 |a_lin| 在 td 量级，就是死区把真实运动吃了。"""
import re

import numpy as np

SRC = r'C:\Users\33\Documents\v2\h415-imu42605-\V5F\User\src\SPI_rx.c'
NCH = int(re.search(r'#define JF_CH_NUM\s+(\d+)u',
                    open(SRC, 'rb').read().decode('gbk')).group(1))
a = np.fromfile(r'R:\raw_v9.bin', dtype='<f4').reshape(-1, NCH)
N = len(a)
dt = a[:, 25].astype(np.float64) * 1e-6
t = np.cumsum(dt)
gb = a[:, 103].astype(np.int32)
G = 9.7985
aln = np.linalg.norm(a[:, 10:13], axis=1) * G        # 旧链残余比力 -> m/s^2（不含死区）
anv = np.linalg.norm(a[:, 93:96], axis=1)            # EKF 导航系线性加速度（含死区）
stilt = np.radians(a[:, 107])
td = 1.0 * G * stilt                                  # 死区门限 m/s^2
p, v = a[:, 83:86], a[:, 86:89]
h_baro = (101325.0 - a[:, 80]) * 0.08326

alnsm = np.convolve(aln, np.ones(max(int(0.2/dt.mean()),1))/max(int(0.2/dt.mean()),1), mode='same')
w3 = np.linalg.norm(a[:, 26:29], axis=1)
wsm = np.convolve(w3, np.ones(max(int(0.2/dt.mean()),1))/max(int(0.2/dt.mean()),1), mode='same')
mv = (alnsm > 0.3) | (wsm > 2.0) | (a[:, 81] > 0.2)
idx = np.where(np.diff(mv.astype(np.int8)) != 0)[0]
segs = [(idx[i], idx[i+1]) for i in range(len(idx)-1)
        if mv[min(idx[i]+1, N-1)] and idx[i+1]-idx[i] > int(0.4/dt.mean())]

print('fw_tag %.0f' % a[0, 76])
print()
print(' #  t起    t止   时长 |a_lin|中位 |a_nav|中位  sigma_tilt  死区td | 被吃掉比例 | Δp_h   Δp_z')
print(' ' + '-'*100)
for n, (s0, s1) in enumerate(segs):
    al, an, sT, T = (np.median(aln[s0:s1]), np.median(anv[s0:s1]),
                     np.degrees(np.median(stilt[s0:s1])), np.median(td[s0:s1]))
    eat = 100.0 * (1.0 - an / al) if al > 1e-6 else float('nan')
    d = p[s1-1] - p[s0]
    print('%2d %6.2f %6.2f %5.2f  %9.4f %9.4f  %8.3f 度 %7.4f | %8.1f%% | %6.4f %+7.4f'
          % (n, t[s0], t[s1-1], t[s1-1]-t[s0], al, an, sT, T, eat,
             float(np.linalg.norm(d[:2])), d[2]))

print()
print('=== 垂直：抬高 20cm 的那一下 ===')
h_mid = h_baro.copy()
h_mid[np.abs(h_mid) > 1000] = np.nan          # 丢掉 press_avg=0 的坏帧
pzb = p[:, 2] + a[:, 102]
print('  h_baro(去坏帧) min %.3f max %.3f 峰-峰 %.3f m' % (
    np.nanmin(h_mid), np.nanmax(h_mid), np.nanmax(h_mid)-np.nanmin(h_mid)))
print('  p_z+b_baro   min %.3f max %.3f 峰-峰 %.3f m' % (
    pzb.min(), pzb.max(), pzb.max()-pzb.min()))
print('  p_z          min %.3f max %.3f' % (p[:, 2].min(), p[:, 2].max()))
print('  b_baro       min %.3f max %.3f' % (a[:, 102].min(), a[:, 102].max()))
print('  v_z          min %.3f max %.3f' % (v[:, 2].min(), v[:, 2].max()))
# baro 门开着时的 p_z 轨迹（每 0.5 s）
print()
print('  时间轴（每 0.5 s）：t / p_z / b_baro / h_baro / 死区td / zupt')
kk = max(int(0.5/dt.mean()), 1)
for i in range(0, N, kk):
    print('   %5.1f  %8.3f %8.3f %9.3f  %7.4f   %d' % (
        t[i], p[i, 2], a[i, 102], h_mid[i] if np.isfinite(h_mid[i]) else float('nan'),
        td[i], (gb[i] & 0x10) != 0))
