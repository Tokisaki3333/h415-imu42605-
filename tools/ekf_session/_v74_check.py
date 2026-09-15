# -*- coding: utf-8 -*-
# VER=74 验收
import numpy as np
P = r'R:\raw_v9.bin'; NCH = 144
r = np.fromfile(P, dtype='<f4'); N = r.size // NCH
b = r[:N*NCH].reshape(-1, NCH).astype(np.float64); t = np.cumsum(b[:,25]*1e-6)
B0X, B0Y = -0.0567804, 0.4296474
TAG = int(round(b[0,76]))
print('fw_tag %d -> VER=%d  CH=%d  帧 %d  时长 %.2f s' % (TAG, TAG>>16, (TAG>>8)&0xFF, N, t[-1]))
good = ((np.abs(b[:,132]-B0X) < 5e-3) & (np.abs(b[:,133]-B0Y) < 5e-3)
        & (b[:,118] > 0.02) & (b[:,119] >= 0) & ((b[:,120] == 0) | (b[:,120] == 1)))
print('好帧 %.2f%%  坏帧(上报污染) %.2f%%' % (100*good.mean(), 100*(~good).mean()))
vx, vy = b[:,130], b[:,131]
psi = np.degrees(np.arctan2(B0X*vy - B0Y*vx, B0X*vx + B0Y*vy))
used = b[:,120] == 1
bh = b[:,118]
print('磁样本率 %.1f Hz ; EKF 周期率 %.1f Hz'
      % ((np.diff(b[:,41]) != 0).sum()/t[-1], (np.diff(b[:,130]) != 0).sum()/t[-1]))
print()
print('  t(s)  |w|p50  mag_r  mag_bh  |psi|p50 |psi|p90  used%%  BH_MIN挡掉%%  R_MAX挡掉%%  p_yy   sig_yaw  dqz p50')
for lo in np.arange(0, t[-1], 2.0):
    m = (t >= lo) & (t < lo+2.0) & good
    if m.sum() < 100: continue
    w = np.linalg.norm(b[m,26:29], axis=1)
    print('%6.0f %7.1f %7.2f %7.3f %8.2f %8.2f %6.1f %11.1f %11.1f %7.3f %8.2f %+9.4f'
          % (lo, np.median(w), np.median(b[m,119]), np.median(bh[m]),
             np.median(np.abs(psi[m])), np.percentile(np.abs(psi[m]),90),
             100*used[m].mean(), 100*(bh[m] < 0.28).mean(),
             100*((b[m,119] > 150) & (bh[m] >= 0.28)).mean(),
             np.median(b[m,121]), np.median(b[m,104]), np.median(b[m,137])))

print()
print('=== 全局 ===')
for nm, v in (('mag_r', b[good,119]), ('|psi|', np.abs(psi[good])), ('mag_bh', bh[good]),
              ('p_yy', b[good,121]), ('sigma_yaw', b[good,104]), ('|dqz|', np.abs(b[good,137]))):
    print(' %-10s p10 %8.3f p50 %8.3f p90 %8.3f p99 %9.3f' % (nm, *np.percentile(v,[10,50,90,99])))
print()
print('mag_used==1 占 %.1f%%' % (100*used[good].mean()))
print('BH_MIN(<0.28) 挡掉 %.2f%% ; mag_r>150 挡掉 %.2f%% ; 真正融合占 %.1f%%'
      % (100*(bh[good] < 0.28).mean(), 100*(b[good,119] > 150).mean(), 100*used[good].mean()))
print()
print('=== 收敛性: 施加 dqz 与 psi 反号 = 收敛 ===')
m = used & good & (np.abs(b[:,137]) > 1e-4)
print('有效周期 %d: corr(psi,dqz) = %+.4f  同号(发散)占 %.1f%%'
      % (m.sum(), np.corrcoef(psi[m], b[m,137])[0,1],
         100*(np.sign(psi[m]) == np.sign(b[m,137])).mean()))
for lo in range(0, int(t[-1]), 4):
    q = m & (t >= lo) & (t < lo+4)
    if q.sum() < 30: continue
    print('  t %2d~%2d: n=%5d corr %+.3f 同号 %3.0f%%  |psi|p50 %6.2f  |w|p50 %7.1f'
          % (lo, lo+4, q.sum(), np.corrcoef(psi[q], b[q,137])[0,1],
             100*(np.sign(psi[q]) == np.sign(b[q,137])).mean(),
             np.median(np.abs(psi[q])), np.median(np.linalg.norm(b[q,26:29],axis=1))))
print()
print('=== 静止段 |psi| (|w|<1 dps) ===')
st = good & (np.linalg.norm(b[:,26:29],axis=1) < 1.0)
for lo in range(0, int(t[-1]), 5):
    q = st & (t >= lo) & (t < lo+5)
    if q.sum() < 200: continue
    print('  t %2d~%2d: n=%6d  |psi| p50 %6.3f p90 %6.3f max %6.3f   mag_r p50 %6.2f  used %5.1f%%'
          % (lo, lo+5, q.sum(), np.median(np.abs(psi[q])), np.percentile(np.abs(psi[q]),90),
             np.abs(psi[q]).max(), np.median(b[q,119]), 100*used[q].mean()))
