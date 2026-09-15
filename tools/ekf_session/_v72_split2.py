# -*- coding: utf-8 -*-
# 正确拆分: 每个 EKF 周期 dpsi = 施加的修正 + 扰动
#   施加量 = dqz (仅当该周期 mag_used==1)  否则 = 0 (死区/拒绝, 什么都不施加)
#   psi_new = psi + dqz  (注入约定, 已在 VER=70 数据上实测)
import numpy as np

P = r'R:\raw_v9.bin'; NCH = 144
r = np.fromfile(P, dtype='<f4'); N = r.size // NCH
b = r[:N*NCH].reshape(-1, NCH).astype(np.float64); t = np.cumsum(b[:,25]*1e-6)
vx, vy, v0x, v0y = b[:,130], b[:,131], b[:,132], b[:,133]
psi = np.degrees(np.arctan2(v0x*vy - v0y*vx, v0x*vx+v0y*vy))
dqz = b[:,137]; used = (b[:,120] == 1)
fn = np.linalg.norm(b[:,42:45], axis=1); clean = (fn > 0.9) & (fn < 1.1)

ch = np.where(np.diff(psi) != 0)[0] + 1
ch = ch[clean[ch]]
i0, i1 = ch[:-1], ch[1:]
dtc = t[i1] - t[i0]
ok = (dtc > 1e-4) & (dtc < 0.02)
i0, i1, dtc = i0[ok], i1[ok], dtc[ok]
dpsi = (psi[i1] - psi[i0] + 180.0) % 360.0 - 180.0
applied = np.where(used[i0], dqz[i0], 0.0)     # ★ 只有真的更新才施加
dist = dpsi - applied
tt = t[i1]
print('周期对 %d   (每个 EKF 周期一对)' % len(dpsi))
print()
print('  t(s)   |w|p50   修正率  扰动率   净变化率   |修正|p50 |扰动|p50  更新率  同号%%')
print('                 (度/秒)  (度/秒)  (度/秒)                          (收敛=0)')
for lo in np.arange(0, 44, 2.0):
    m = (tt >= lo) & (tt < lo + 2.0)
    if m.sum() < 30: continue
    w = np.linalg.norm(b[i1[m], 26:29], axis=1)
    ar = applied[m]/dtc[m]; dr = dist[m]/dtc[m]; nr = dpsi[m]/dtc[m]
    sg = np.sign(psi[i0][m]) == np.sign(applied[m])
    print('%6.0f %8.1f %8.2f %8.2f %9.2f %9.4f %9.4f %7.1f %6.0f'
          % (lo, np.median(w), ar.mean(), dr.mean(), nr.mean(),
             np.median(np.abs(applied[m])), np.median(np.abs(dist[m])),
             100*used[i0][m].mean(), 100*sg.mean() if np.abs(applied[m]).max() > 1e-6 else 0))
print()
print('=== 总账（求和，可加） ===')
for lo, hi, nm in [(0,10,'静止 0-10s'), (10,28,'强运动 10-28s'), (28,32,'过渡 28-32s'),
                   (32,36,'静止 32-36s'), (36,44,'静止 36-44s')]:
    m = (tt >= lo) & (tt < hi)
    if m.sum() < 30: continue
    d = dist[m].sum(); a = applied[m].sum()
    print(' %-14s 施加修正总 %+9.2f 度   扰动总 %+10.2f 度   净变化 %+9.2f 度   修正/扰动 %6.2f'
          % (nm, a, d, a+d, abs(a)/max(1e-9, abs(d))))
print()
print('=== 扰动 vs 运动强度（只用真的更新过的周期）===')
w = np.linalg.norm(b[i1, 26:29], axis=1)
for lo, hi in [(0,5),(5,50),(50,300),(300,800),(800,1200),(1200,2000)]:
    m = (w >= lo) & (w < hi) & used[i0]
    if m.sum() < 50: continue
    print(' |w| %4d~%4d dps: n=%6d  扰动率均值 %+9.2f 度/秒   修正率均值 %+9.2f 度/秒   |修正|p50 %.4f'
          % (lo, hi, m.sum(), (dist[m]/dtc[m]).mean(), (applied[m]/dtc[m]).mean(),
             np.median(np.abs(applied[m]))))
