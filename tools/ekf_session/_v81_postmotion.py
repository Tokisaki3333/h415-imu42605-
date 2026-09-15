# -*- coding: utf-8 -*-
# VER=81：强激励下 EKF 是否有效校正
#   方法：用旧路径判静(col 8 is_static)切"运动/静止"段；
#   运动后静止段的磁修正量 ∝ 运动中累积的偏航误差：
#     · 该段一开始的 |新息| = 残余误差
#     · 该段累计 Σ|dqz| = 环路实际补掉的量
#     · 收敛耗时 = 环路把它拉回死区/噪声底所需时间
import numpy as np
P = r'R:\raw_v9.bin'; NCH = 148
r = np.fromfile(P, dtype='<f4'); N = r.size // NCH
b = r[:N*NCH].reshape(-1, NCH).astype(np.float64); t = np.cumsum(b[:,25]*1e-6)
B0X, B0Y = -0.0567804, 0.4296474
good = ((np.abs(b[:,132]-B0X) < 5e-3) & (np.abs(b[:,133]-B0Y) < 5e-3)
        & (b[:,118] > 0) & (b[:,119] >= 0) & ((b[:,120] == 0) | (b[:,120] == 1)))
inn = (b[:,144] - b[:,145] + 180) % 360 - 180     # 带符号新息(固件上报)
gn = np.linalg.norm(b[:,26:29], axis=1)
gcl = gn.copy(); gcl[gcl > 1e4] = np.nan          # 剔除 gyro_dps 垃圾段
stat = (b[:,8] == 1) & good                        # 旧路径判静
print('fw_tag %d VER=%d  时长 %.2f s' % (int(round(b[0,76])), int(round(b[0,76]))>>16, t[-1]))
print('旧路径判静占 %.1f%% ; |w|<1dps 占 %.1f%%' % (100*stat.mean(), 100*(np.nan_to_num(gcl,nan=1e9) < 1).mean()))

# 段切分：用 stat 的边沿
lab = np.zeros(len(b), dtype=int); cur = 0
prev = stat[0]
seg = []
start = 0
for i in range(1, len(b)):
    if stat[i] != prev:
        seg.append((start, i-1, prev)); start = i; prev = stat[i]
seg.append((start, len(b)-1, prev))
print()
print('=== 时段划分（旧路径判静）===')
print('  #  起(s)   止(s)   时长   性质   |w|p50   |新息|p50  |新息|max')
for k, (a, z, s) in enumerate(seg):
    if z - a < 50: continue
    print('  %2d %6.2f %7.2f %6.2f  %s %8.2f %10.3f %10.3f'
          % (k, t[a], t[z], t[z]-t[a], '静止' if s else '运动',
             np.nanmedian(gcl[a:z+1]), np.median(np.abs(inn[a:z+1])), np.abs(inn[a:z+1]).max()))

print()
print('=== 运动后静止段：磁修正量 ===')
for k in range(1, len(seg)-1):
    a, z, s = seg[k]
    if not s or z - a < 100:      # 只看静止段
        continue
    pa, pz, ps = seg[k-1]
    if ps or (pz - pa) < 100:     # 前一段必须是运动
        continue
    # 该静止段内的修正累计与收敛
    m = slice(a, z+1)
    dq = b[m, 137]
    used = (b[m, 120] == 1)
    ai = np.abs(inn[m])
    # 收敛：|新息| 首次降到 0.5 度以下的位置
    idx = np.where(ai < 0.5)[0]
    settle = t[a + idx[0]] - t[a] if len(idx) else float('nan')
    print('  运动段 t=%.2f~%.2f (%.2f s, |w|p50 %.0f) -> 静止段 t=%.2f~%.2f'
          % (t[pa], t[pz], t[pz]-t[pa], np.nanmedian(gcl[pa:pz+1]), t[a], t[z]))
    print('     静止段起始 |新息| = %.3f 度   (即运动中累积的偏航误差)'
          % ai[0] if len(ai) else '')
    print('     该段 |新息| p50 %.3f 末值 %.3f 度 ; 累计 Σ|dqz| = %.3f 度 ; used %.0f%% ; 收敛到0.5度用时 %.2f s'
          % (np.median(ai), ai[-1], np.nansum(np.abs(dq)), 100*used.mean(), settle))
    print('     |dqz| 峰值 %.4f 度/更新 (=k_cap*|r|: %.4f)'
          % (np.abs(dq).max(), 0.05*np.abs(np.radians(ai)).max()*57.2957795))

print()
print('=== 全程修正量分布 ===')
u = (b[:,120] == 1) & good
print('  |dqz| p50 %.4f p90 %.4f p99 %.4f max %.3f 度' %
      tuple(np.percentile(np.abs(b[u,137]), [50,90,99,100])))
print('  |dqz| > 0.5 度的帧占 %.2f%% ; > 2 度 %.2f%%' %
      (100*(np.abs(b[:,137]) > 0.5).mean(), 100*(np.abs(b[:,137]) > 2).mean()))
print()
print('  t(s)  is_static |w|p50  |新息|p50  Σ|dqz|(1s窗)  dqz_p50')
for lo in range(0, int(t[-1]), 1):
    m = (t >= lo) & (t < lo+1) & good
    if m.sum() < 100: continue
    print('%5d %10.0f %7.2f %10.3f %14.4f %10.4f'
          % (lo, 100*stat[m].mean(), np.nanmedian(gcl[m]), np.median(np.abs(inn[m])),
             np.nansum(np.abs(b[m,137])), np.median(b[m,137])))
