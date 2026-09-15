# -*- coding: utf-8 -*-
# 把残差变化拆开:  dpsi = dqz(环路施加的修正) + 扰动(陀螺/倾角耦合/样本陈旧/运动)
# 注入约定已实测: psi_new = psi + dqz (v 是世界系磁场, 方位角只被施加的修正改变)
import numpy as np

P = r'R:\raw_v9.bin'; NCH = 144
r = np.fromfile(P, dtype='<f4'); N = r.size // NCH
b = r[:N*NCH].reshape(-1, NCH).astype(np.float64); t = np.cumsum(b[:,25]*1e-6)
vx, vy, v0x, v0y = b[:,130], b[:,131], b[:,132], b[:,133]
psi = np.degrees(np.arctan2(v0x*vy - v0y*vx, v0x*vx + v0y*vy))
dqz = b[:,137]
fn = np.linalg.norm(b[:,42:45], axis=1); clean = (fn > 0.9) & (fn < 1.1)

# mag 周期边界: psi 变化的帧
ch = np.where(np.diff(psi) != 0)[0] + 1
ch = ch[clean[ch]]
i0, i1 = ch[:-1], ch[1:]
dtc = (t[i1] - t[i0])
ok = (dtc > 1e-4) & (dtc < 0.02)          # 只要相邻/近邻周期
i0, i1, dtc = i0[ok], i1[ok], dtc[ok]
dpsi = (psi[i1] - psi[i0] + 180.0) % 360.0 - 180.0
corr = dqz[i0]                            # 上一周期施加的修正
dist = dpsi - corr                        # 扰动（陀螺/倾角/陈旧/运动）
tt = t[i1]

print('周期对 %d' % len(dpsi))
print()
print(' t(s)   |w|p50   |dpsi|p50  dqz p50  扰动p50   扰动/dpsi   dpsi/dt  dqz/dt  扰动/dt')
print('                                                          (度/秒)')
for lo in np.arange(0, 44, 2.0):
    m = (tt >= lo) & (tt < lo + 2.0)
    if m.sum() < 30: continue
    w = np.linalg.norm(b[i1[m], 26:29], axis=1)
    print('%5.0f %8.1f %9.3f %+9.4f %+8.3f %9.3f %9.2f %8.2f %9.2f'
          % (lo, np.median(w), np.median(np.abs(dpsi[m])), np.median(corr[m]),
             np.median(dist[m]), np.median(dist[m]) / max(1e-9, np.median(np.abs(dpsi[m]))),
             np.median(dpsi[m]/dtc[m]), np.median(corr[m]/dtc[m]), np.median(dist[m]/dtc[m])))

print()
print('全局: 扰动/|dpsi| 中位 %.3f ; 扰动与 dpsi 同号占比 %.1f%%'
      % (np.median(dist/np.maximum(np.abs(dpsi), 1e-9)), 100*(np.sign(dist) == np.sign(dpsi)).mean()))
print()
print('=== 扰动 vs 运动强度 ===')
for lo, hi in [(0, 5), (5, 50), (50, 300), (300, 800), (800, 1200), (1200, 2000)]:
    w = np.linalg.norm(b[i1, 26:29], axis=1)
    m = (w >= lo) & (w < hi)
    if m.sum() < 50: continue
    print(' |w| %4d~%4d dps: n=%6d  |扰动| p50 %7.3f  |dqz| p50 %6.4f  |dpsi| p50 %7.3f  扰动/修正 %6.1f'
          % (lo, hi, m.sum(), np.median(np.abs(dist[m])), np.median(np.abs(corr[m])),
             np.median(np.abs(dpsi[m])),
             np.median(np.abs(dist[m]))/max(1e-9, np.median(np.abs(corr[m])))))
print()
print('=== 修正与残差的符号（收敛性）===')
m = np.abs(corr) > 1e-3
print('|dqz|>0.001 的周期 %d:  corr(psi, dqz) = %+.4f   同号占 %.1f%%'
      % (m.sum(), np.corrcoef(psi[i0][m], corr[m])[0,1],
         100*(np.sign(psi[i0][m]) == np.sign(corr[m])).mean()))
for lo in np.arange(0, 44, 4.0):
    mm = m & (tt >= lo) & (tt < lo+4.0)
    if mm.sum() < 30: continue
    print('   t %2.0f~%2.0f: n=%5d  corr(psi,dqz) %+.3f  同号 %.0f%%'
          % (lo, lo+4, mm.sum(), np.corrcoef(psi[i0][mm], corr[mm])[0,1],
             100*(np.sign(psi[i0][mm]) == np.sign(corr[mm])).mean()))
