# -*- coding: utf-8 -*-
# 用 v0（编译期常量）当帧校验字，剔掉上报污染帧后重做全部分析
import numpy as np

P = r'R:\raw_v9.bin'; NCH = 144
r = np.fromfile(P, dtype='<f4'); N = r.size // NCH
b = r[:N*NCH].reshape(-1, NCH).astype(np.float64); t = np.cumsum(b[:,25]*1e-6)
B0X, B0Y = -0.0567804, 0.4296474          # ci*sin(-7.53), ci*cos(-7.53)

good = ((np.abs(b[:,132] - B0X) < 5e-3) & (np.abs(b[:,133] - B0Y) < 5e-3)
        & (b[:,118] > 0.05) & (b[:,119] >= 0.0)
        & ((b[:,120] == 0) | (b[:,120] == 1)))
print('帧 %d   好帧 %d (%.2f%%)   坏帧 %d (%.2f%%)'
      % (N, good.sum(), 100*good.mean(), (~good).sum(), 100*(~good).mean()))
print('坏帧成簇: 相邻两帧同为坏的比例 %.2f%%' % (100*(~good[1:] & ~good[:-1]).mean()))

vx, vy = b[:,130], b[:,131]
psi = np.degrees(np.arctan2(B0X*vy - B0Y*vx, B0X*vx + B0Y*vy))
dqz = b[:,137]; used = (b[:,120] == 1)
fn = np.linalg.norm(b[:,42:45], axis=1); fok = (fn > 0.9) & (fn < 1.1)

ch = np.where(np.diff(psi) != 0)[0] + 1
ch = ch[good[ch] & good[ch-1] & fok[ch]]
i0, i1 = ch[:-1], ch[1:]
dtc = t[i1] - t[i0]
ok = (dtc > 1e-4) & (dtc < 2e-2)
i0, i1, dtc = i0[ok], i1[ok], dtc[ok]
dpsi = (psi[i1] - psi[i0] + 180.0) % 360.0 - 180.0
applied = np.where(used[i0], dqz[i0], 0.0)
dist = dpsi - applied
tt = t[i1]
w = np.linalg.norm(b[i1, 26:29], axis=1)
print('干净周期对 %d' % len(dpsi))

print()
print('================ 修正方向（收敛性）================')
m = np.abs(applied) > 1e-3
print('|施加|>0.001 的周期 %d:  corr(psi,施加) = %+.4f  同号(发散)占 %.1f%%'
      % (m.sum(), np.corrcoef(psi[i0][m], applied[m])[0, 1],
         100*(np.sign(psi[i0][m]) == np.sign(applied[m])).mean()))
for lo in range(0, 44, 4):
    mm = m & (tt >= lo) & (tt < lo+4)
    if mm.sum() < 20: continue
    print('  t %2d~%2d: n=%5d  corr %+.3f  同号 %3.0f%%   |psi|p50 %6.2f  更新率 %3.0f%%'
          % (lo, lo+4, mm.sum(), np.corrcoef(psi[i0][mm], applied[mm])[0, 1],
             100*(np.sign(psi[i0][mm]) == np.sign(applied[mm])).mean(),
             np.median(np.abs(psi[i0][mm])), 100*used[i0][mm].mean()))

print()
print('================ 修正 vs 扰动（总和可加）================')
for lo, hi, nm in [(0,10,'静止 0-10s'), (10,28,'强运动 10-28s'), (28,32,'过渡 28-32s'),
                   (32,44,'静止 32-44s')]:
    mm = (tt >= lo) & (tt < hi)
    if mm.sum() < 20: continue
    a = applied[mm].sum(); d = dist[mm].sum()
    print(' %-14s 施加修正 %+9.2f 度   扰动 %+10.2f 度   净变化 %+9.2f 度   修正/扰动 %6.3f'
          % (nm, a, d, a+d, abs(a)/max(1e-9, abs(d))))

print()
print('================ 修正幅度/时间分布 ================')
ad = np.abs(applied[np.abs(applied) > 1e-9])
print('|施加| 分位: p50 %.4f p90 %.4f p99 %.4f max %.4f 度'
      % tuple(np.percentile(ad, [50, 90, 99, 100])))
print('更新率(整体) %.1f%%  死区内占 %.1f%%' % (100*used.mean(), 100*(b[:,119] < 12).mean()))
print()
print('  t(s)  |w|p50   更新率   施加修正(度/秒)  扰动(度/秒)   |psi|p50  p_yy')
for lo in range(0, 44, 2):
    mm = (tt >= lo) & (tt < lo+2)
    if mm.sum() < 20: continue
    print('%6d %7.1f %8.1f%% %14.3f %12.3f %11.2f %8.4f'
          % (lo, np.median(w[mm]), 100*used[i0][mm].mean(),
             (applied[mm]/dtc[mm]).mean(), (dist[mm]/dtc[mm]).mean(),
             np.median(np.abs(psi[i0][mm])), np.median(b[i1[mm], 121])))

print()
print('================ 航向分布（系统性?）只在静止段 ================')
mm = ((tt >= 0) & (tt < 10)) | ((tt >= 32) & (tt < 44))
yaw = b[i0, 134]
for lo in range(-180, 180, 30):
    q = mm & (yaw >= lo) & (yaw < lo+30)
    if q.sum() < 20: continue
    print(' 偏航 %+4d~%+4d: n=%5d  psi p50 %+7.2f  |psi| p50 %6.2f  |psi| p90 %6.2f'
          % (lo, lo+30, q.sum(), np.median(psi[i0][q]),
             np.median(np.abs(psi[i0][q])), np.percentile(np.abs(psi[i0][q]), 90)))
