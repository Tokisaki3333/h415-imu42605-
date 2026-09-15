# -*- coding: utf-8 -*-
# VER=72 复核：地磁修正到底是"均匀分布"还是"错误集中"
#   (a) 时间分布: 修正是否集中在少数爆发, 还是均匀铺开
#   (b) 航向分布: |psi| 是否集中在特定航向 = 系统性(标定/安装)误差
#   (c) 符号:     dqz 是否与 psi 反号 = 收敛
import numpy as np, os

P = r'R:\raw_v9.bin'
NCH = 144
r = np.fromfile(P, dtype='<f4')
N = r.size // NCH
b = r[:N*NCH].reshape(-1, NCH).astype(np.float64)
t = np.cumsum(b[:, 25] * 1e-6)
R2D = 57.2957795

TAG = int(round(b[0, 76]))
VER = TAG >> 16
print('文件 %s  %.1f MB  帧 %d  时长 %.2f s' % (P, os.path.getsize(P)/1048576, N, t[-1]))
print('fw_tag %d -> VER=%d CH=%d EKF=%d MAGCAL=%d DETAC=%d   (VER=72 应为 4755459)'
      % (TAG, VER, (TAG >> 8) & 0xFF, TAG & 1, (TAG >> 1) & 1, (TAG >> 2) & 1))

fn = np.linalg.norm(b[:, 42:45], axis=1)
clean = (fn > 0.9) & (fn < 1.1)
print('|mag.f| 干净帧 %.3f%%   坏帧 %d' % (100*clean.mean(), (~clean).sum()))
print('col120(mag_used) 取值: %s' % np.unique(b[:, 120])[:6])

# 有符号残差 psi: v0 -> v
vx, vy = b[:, 130], b[:, 131]
v0x, v0y = b[:, 132], b[:, 133]
psi = np.degrees(np.arctan2(v0x*vy - v0y*vx, v0x*vx + v0y*vy))
magr = b[:, 119]
dqz = b[:, 137]
used = (b[:, 120] == 1)

# ---------- (a) 时间分布 ----------
print()
print('================ (a) 时间分布 ================')
print(' 段(s)      mag_r p50  p90   max   <12度%%   used%%    dqz p50   dqz p90   sigma_yaw')
edges = np.arange(0, t[-1], 2.0)
for lo in edges:
    m = (t >= lo) & (t < lo + 2.0)
    if m.sum() < 100: continue
    print('%4.0f-%4.0f %10.2f %6.2f %6.2f %7.1f %7.1f %+9.4f %+9.4f %9.2f'
          % (lo, lo+2, np.median(magr[m]), np.percentile(magr[m], 90), magr[m].max(),
             100*(magr[m] < 12).mean(), 100*used[m].mean(),
             np.median(dqz[m]), np.percentile(dqz[m], 90), np.median(b[m, 104])))

# ---------- (b)(c) 只在真正更新的周期上 ----------
sel = used & clean
print()
print('================ (c) 符号（收敛性） ================')
print('真正更新周期 %d' % sel.sum())
if sel.sum():
    print('corr(psi, dqz) = %+.4f   (>0 发散, <0 收敛)'
          % np.corrcoef(psi[sel], dqz[sel])[0, 1])
    sg = np.sign(psi[sel]) == np.sign(dqz[sel])
    print('psi 与 dqz 同号占比 %.1f%%   (收敛时应接近 0%%)' % (100*sg.mean()))
    print('psi   p10 %+.2f p50 %+.2f p90 %+.2f' % tuple(np.percentile(psi[sel], [10, 50, 90])))
    print('dqz   p10 %+.4f p50 %+.4f p90 %+.4f' % tuple(np.percentile(dqz[sel], [10, 50, 90])))

print()
print('================ (b) 航向分布（系统性?） ================')
yaw = b[:, 134]          # mag_yawpre: 修正前 EKF 偏航(度)
for lo in range(-180, 180, 30):
    m = sel & (yaw >= lo) & (yaw < lo + 30)
    if m.sum() < 50: continue
    print(' 偏航 %+4d~%+4d: n=%6d   psi p50 %+7.2f  |psi| p90 %6.2f   dqz p50 %+8.4f'
          % (lo, lo+30, m.sum(), np.median(psi[m]), np.percentile(np.abs(psi[m]), 90),
             np.median(dqz[m])))

print()
print('================ 修正幅度分布（均匀? 集中?） ================')
ad = np.abs(dqz[sel])
print('|dqz| 分位: p10 %.4f p25 %.4f p50 %.4f p75 %.4f p90 %.4f p99 %.4f max %.4f'
      % tuple(np.percentile(ad, [10, 25, 50, 75, 90, 99, 100])))
print('|dqz| > 0.5 度的周期占 %.2f%%   > 1 度占 %.2f%%   > 2 度占 %.2f%%'
      % (100*(ad > 0.5).mean(), 100*(ad > 1).mean(), 100*(ad > 2).mean()))
print('mag_rej(126) p50 %d max %d   mag_gate(117) 平均 %.3f'
      % (np.median(b[:, 126]), b[:, 126].max(), b[:, 117].mean()))
# 死区占用
print('mag_r < 12 度(死区) 的帧占 %.1f%% ; < 20 度 %.1f%%'
      % (100*(magr < 12).mean(), 100*(magr < 20).mean()))
