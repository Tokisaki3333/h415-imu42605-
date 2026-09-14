# -*- coding: utf-8 -*-
"""追问三件事：① 静置段为什么这么短（用户说运动间有静置）② 垂直通道被打散多严重
③ 气压观测到底尝试了多少次、成功多少。"""
import numpy as np

a = np.fromfile(r'R:\raw_v9.bin', dtype='<f4').reshape(-1, 112)
N = len(a)
dt = a[:, 25].astype(np.float64) * 1e-6
t = np.cumsum(dt)
k = max(int(0.2/dt.mean()), 1)
w3 = np.linalg.norm(a[:, 26:29], axis=1)
aln = np.linalg.norm(a[:, 10:13], axis=1)
eac = a[:, 81]
wsm = np.convolve(w3, np.ones(k)/k, mode='same')
asm = np.convolve(aln, np.ones(k)/k, mode='same')

print('=== ① 用更松的判据找"静置"（|w|<3 dps 平滑, |a|<0.08g, e_ac<0.5）===')
still = (wsm < 3.0) & (asm < 0.08) & (eac < 0.5)
idx = np.where(np.diff(still.astype(np.int8)) != 0)[0]
segs = []
for i in range(len(idx)-1):
    s0, s1 = idx[i], idx[i+1]
    if still[min(s0+1, N-1)] and (s1-s0) > int(0.15/dt.mean()):
        segs.append((s0, s1))
print('  找到 %d 段静置（>0.15 s）：' % len(segs))
gb = a[:, 103].astype(np.int32)
r_yaw_ok = True
D = -7.53
def q2R(q):
    w, x, y, z = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
    R = np.empty((len(q), 3, 3))
    R[:, 0, 0] = 1-2*(y*y+z*z); R[:, 0, 1] = 2*(x*y-w*z); R[:, 0, 2] = 2*(x*z+w*y)
    R[:, 1, 0] = 2*(x*y+w*z);   R[:, 1, 1] = 1-2*(x*x+z*z); R[:, 1, 2] = 2*(y*z-w*x)
    R[:, 2, 0] = 2*(x*z-w*y);   R[:, 2, 1] = 2*(y*z+w*x);  R[:, 2, 2] = 1-2*(x*x+y*y)
    return R
def bn(R, v): return np.einsum('nij,nj->ni', R, v)
def wrap(d): return (d + 180.0) % 360.0 - 180.0
R = q2R(a[:, 89:93])
Bq = bn(R, a[:, 42:45])
r_yaw = wrap(D - np.degrees(np.arctan2(Bq[:, 0], Bq[:, 1])))
print('    #   t起    t止   时长   zupt  baro  tilt  mag   yaw误   p_z      b_baro')
for n, (s0, s1) in enumerate(segs):
    m = gb[s0:s1]
    print('   %2d %6.2f %6.2f %5.2f s %5.0f%% %5.0f%% %5.0f%% %5.0f%%  %+7.3f  %8.2f  %8.2f'
          % (n, t[s0], t[s1-1], t[s1-1]-t[s0],
             100*np.mean((m & 0x10) != 0), 100*np.mean((m & 0x04) != 0),
             100*np.mean((m & 0x20) != 0), 100*np.mean((m & 0x40) != 0),
             np.median(r_yaw[s0:s1]), np.median(a[s0:s1, 85]), np.median(a[s0:s1, 102])))

print()
print('=== ② 垂直通道 ===')
pz, bb, ha = a[:, 85], a[:, 102], a[:, 51]
hlin = (101325.0 - a[:, 80]) * 0.08326
print('  p_z     min %9.2f  max %9.2f  -> 摆动 %8.2f m' % (pz.min(), pz.max(), pz.max()-pz.min()))
print('  b_baro  min %9.2f  max %9.2f  -> 摆动 %8.2f m' % (bb.min(), bb.max(), bb.max()-bb.min()))
print('  h_baro-(p_z+b_baro)  p50 %8.3f  max|.| %8.3f m' % (
    np.median(hlin-(pz+bb)), np.abs(hlin-(pz+bb)).max()))
print('  v_z 末 %8.3f  max|.| %8.3f m/s' % (a[-1, 88], np.abs(a[:, 88]).max()))
print('  sigma_vel_h p50 %.4f   sigma_pos_h p50 %.4f' % (np.median(a[:, 106]), np.median(a[:, 105])))

print()
print('=== ③ 气压观测尝试/成功次数（M3 每 0.164 s 最多一次）===')
baro_bit = (gb & 0x04) != 0
cyc = np.where(gb[1:] != gb[:-1])[0] + 1          # 每周期只发布一次，取变化沿
print('  周期数 %d  (%.1f /s)' % (len(cyc), len(cyc)/t[-1]))
print('  其中 baro 成功 %d 次 (%.2f%%)' % (baro_bit[cyc].sum(), 100*baro_bit[cyc].mean()))
print('  理论上限 = 6.1 Hz * 60 s = %d 次 -> 实际成功率 %.0f%%'
      % (int(6.1*t[-1]), 100*baro_bit[cyc].sum()/max(int(6.1*t[-1]), 1)))
nis_b = a[:, 109]
nb = nis_b[np.where(baro_bit[cyc])[0]] if baro_bit[cyc].any() else np.array([])
print('  成功那些周期的 nis_baro: %s' % (np.round(nb[:10], 3) if len(nb) else '无'))
print('  nis_baro 全帧 p50 %.3f  p90 %.3f  max %.1f' % (
    np.median(nis_b), np.percentile(nis_b, 90), nis_b.max()))
