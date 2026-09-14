# -*- coding: utf-8 -*-
"""静置段里 ZUPT 只开 53%，查是哪个子判据在关它（我新加了 e_ac 条件，怕是加过头了）。
同时查运动段 mag 门那 8~17% 的关闭是 mag.ok=0（真磁干扰）还是别的原因。"""
import numpy as np

a = np.fromfile(r'R:\raw_v9.bin', dtype='<f4').reshape(-1, 112)
N = len(a)
dt = a[:, 25].astype(np.float64) * 1e-6
t = np.cumsum(dt)
gb = a[:, 103].astype(np.int32)

is_static = a[:, 8] > 0.5
eac = a[:, 81]
alin2 = (a[:, 10]**2 + a[:, 11]**2 + a[:, 12]**2)
spac = a[:, 54]
rmc_speed_flag = (a[:, 55].astype(np.int64) & 0x04) != 0
zupt = (gb & 0x10) != 0

s0, s1 = int(np.searchsorted(t, 17.23)), int(np.searchsorted(t, 21.93))
print('静置段 t=17.23~21.93 s  (%d 帧)' % (s1-s0))
print('  is_static      %.1f%%' % (100*np.mean(is_static[s0:s1])))
print('  e_ac<0.20      %.1f%%   (e_ac p50 %.4f max %.4f)'
      % (100*np.mean(eac[s0:s1] < 0.20), np.median(eac[s0:s1]), eac[s0:s1].max()))
print('  |a_lin|<0.05g  %.1f%%   (|a_lin| p50 %.4f max %.4f g)'
      % (100*np.mean(alin2[s0:s1] < 0.0025), np.median(np.sqrt(alin2[s0:s1])),
         np.sqrt(alin2[s0:s1]).max()))
print('  speed<0.5或无比 %.1f%%  (speed p50 %.2f, 有SPEED位 %.0f%%)'
      % (100*np.mean((spac[s0:s1] < 0.5) | (~rmc_speed_flag[s0:s1])),
         np.median(spac[s0:s1]), 100*np.mean(rmc_speed_flag[s0:s1])))
cond = (is_static & (eac < 0.20) & (alin2 < 0.0025)
        & ((spac < 0.5) | (~rmc_speed_flag)))
print('  四条同时成立  %.1f%%   （实测 zupt 门 %.1f%%）'
      % (100*np.mean(cond[s0:s1]), 100*np.mean(zupt[s0:s1])))
print()
print('  各条单独不成立的帧占比（静置段内）：')
print('    !is_static      %.1f%%' % (100*np.mean(~is_static[s0:s1])))
print('    e_ac>=0.20      %.1f%%' % (100*np.mean(eac[s0:s1] >= 0.20)))
print('    |a_lin|>=0.05g  %.1f%%' % (100*np.mean(alin2[s0:s1] >= 0.0025)))
print('    speed>=0.5      %.1f%%' % (100*np.mean((spac[s0:s1] >= 0.5) & rmc_speed_flag[s0:s1])))

print()
print('=== 运动段 mag 门关闭的原因 ===')
for (x0, x1, nm) in [(4.32, 19.43, '段1 动'), (21.92, 60.0, '段3 动')]:
    i0, i1 = int(np.searchsorted(t, x0)), int(np.searchsorted(t, x1))
    mag = (gb[i0:i1] & 0x40) != 0
    ok = a[i0:i1, 50] > 0.5           # 第 50 列 = mag.trust（旧口径，供对照）
    print('  %s: mag门 %.0f%%   mag.trust(col50>0.5) %.0f%%   chi2剔 %.0f%%'
          % (nm, 100*mag.mean(), 100*ok.mean(), 100*np.mean((gb[i0:i1] & 0x200) != 0)))
    print('       => 关门的帧里 mag.trust=0 的占 %.0f%%（说明剩下的关闭来自内层 chi2 或未更新）'
          % (100*np.mean(~ok[~mag]) if (~mag).any() else float('nan')))
print()
print('注：第 50 列是 mag.trust（含 |a_lin|<0.05g 的旧条件），只作对照；M7 现在用 mag.ok。')
