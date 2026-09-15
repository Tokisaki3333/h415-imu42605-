# -*- coding: utf-8 -*-
# VER=73 验收：v0 当帧校验字过滤污染帧
import numpy as np
P = r'R:\raw_v9.bin'; NCH = 144
r = np.fromfile(P, dtype='<f4'); N = r.size // NCH
b = r[:N*NCH].reshape(-1, NCH).astype(np.float64); t = np.cumsum(b[:,25]*1e-6)
B0X, B0Y = -0.0567804, 0.4296474
good = ((np.abs(b[:,132]-B0X) < 5e-3) & (np.abs(b[:,133]-B0Y) < 5e-3)
        & (b[:,118] > 0.05) & (b[:,119] >= 0) & ((b[:,120] == 0) | (b[:,120] == 1)))
vx, vy = b[:,130], b[:,131]
psi = np.degrees(np.arctan2(B0X*vy - B0Y*vx, B0X*vx + B0Y*vy))
dqz = b[:,137]; used = b[:,120] == 1
print('好帧 %.2f%%' % (100*good.mean()))
print()
print('  t(s)  |w|p50   mag_r  |psi|p50 |psi|p90  used%%  更新率Hz  p_yy    sig_yaw  |f|限制')
for lo in np.arange(0, 44, 2.0):
    m = (t >= lo) & (t < lo+2.0) & good
    if m.sum() < 100: continue
    w = np.linalg.norm(b[m, 26:29], axis=1)
    print('%6.0f %7.1f %8.2f %8.2f %8.2f %6.1f %9.1f %8.4f %8.2f'
          % (lo, np.median(w), np.median(b[m,119]), np.median(np.abs(psi[m])),
             np.percentile(np.abs(psi[m]),90), 100*used[m].mean(), used[m].mean()/2.0*349.8,
             np.median(b[m,121]), np.median(b[m,104])))

print()
print('=== 全局 ===')
for nm, v in (('mag_r', b[good,119]), ('|psi|', np.abs(psi[good])), ('p_yy', b[good,121]),
              ('sigma_yaw', b[good,104])):
    print(' %-10s p10 %8.3f p50 %8.3f p90 %8.3f max %9.3f' % (nm, *np.percentile(v,[10,50,90]), v.max()))
print()
print('死区内(|mag_r|<12)占 %.1f%% ; 可更新区(12~150)占 %.1f%% ; 超上限(>150)占 %.1f%%'
      % (100*(b[good,119]<12).mean(), 100*((b[good,119]>=12)&(b[good,119]<=150)).mean(),
         100*(b[good,119]>150).mean()))
print('p_yy < 0.001 的占 %.2f%%  (VER=72 是 5.40%%+塌陷)' % (100*(b[good,121]<1e-3).mean()))
print()
print('=== 收敛性：施加量与 psi 反号 = 收敛 ===')
m = used & good
print('mag_used==1 的帧 %d ; corr(psi, dqz) = %+.4f ; 同号(发散)占 %.1f%%'
      % (m.sum(), np.corrcoef(psi[m], dqz[m])[0,1], 100*(np.sign(psi[m])==np.sign(dqz[m])).mean()))
for lo in range(0, 44, 4):
    q = m & (t >= lo) & (t < lo+4)
    if q.sum() < 20: continue
    print('  t %2d~%2d: n=%5d  corr %+.3f  同号 %3.0f%%   |psi|p50 %6.2f'
          % (lo, lo+4, q.sum(), np.corrcoef(psi[q], dqz[q])[0,1],
             100*(np.sign(psi[q])==np.sign(dqz[q])).mean(), np.median(np.abs(psi[q]))))
