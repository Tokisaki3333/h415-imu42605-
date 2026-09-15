# -*- coding: utf-8 -*-
# VER=78 验收（148 列）+ 检查"重力方向"判断是否正确
import numpy as np
P = r'R:\raw_v9.bin'; NCH = 148; R2D = 57.2957795
r = np.fromfile(P, dtype='<f4'); N = r.size // NCH
b = r[:N*NCH].reshape(-1, NCH).astype(np.float64); t = np.cumsum(b[:,25]*1e-6)
TAG = int(round(b[0,76]))
print('fw_tag %d -> VER=%d  帧 %d  列 %d  时长 %.2f s' % (TAG, TAG>>16, N, NCH, t[-1]))
print('  (VER=78 应为 5083143)')
B0X, B0Y = -0.0567804, 0.4296474
good = ((np.abs(b[:,132]-B0X) < 5e-3) & (np.abs(b[:,133]-B0Y) < 5e-3)
        & (b[:,118] > 0.0) & (b[:,119] >= 0) & ((b[:,120] == 0) | (b[:,120] == 1)))
print('好帧 %.2f%%' % (100*good.mean()))

amn = b[:,146]; mhn = b[:,147]; thm_rep = b[:,144]; thp_rep = b[:,145]
print()
print('=== 新增罗盘列 ===')
for nm, v in (('cmp_amn 比力模长(g)', amn), ('cmp_mhn 法平面磁场模长', mhn),
              ('cmp_thm 实测航向(度)', thm_rep), ('cmp_thp 预测航向(度)', thp_rep)):
    print('  %-22s p10 %8.3f p50 %8.3f p90 %8.3f  min %8.2f max %8.2f'
          % (nm, np.percentile(v,10), np.median(v), np.percentile(v,90), v.min(), v.max()))
print('  cmp_amn 落在 [0.97,1.03] 的比例 %.1f%%  (1.0=加计此刻即重力)'
      % (100*((amn > 0.97) & (amn < 1.03)).mean()))

# ---- 复算：用日志里的 accel_g(已处理) 与 mag.f 重算航向，与固件上报对比 ----
acc = b[:,32:35]; magf = b[:,42:45]
an = np.linalg.norm(acc, axis=1); fn = np.linalg.norm(magf, axis=1)
ab = acc/np.maximum(an,1e-9)[:,None]; mf = magf/np.maximum(fn,1e-9)[:,None]
dpar = np.sum(mf*ab,1); mh = mf - dpar[:,None]*ab
xv = np.stack([1-ab[:,0]*ab[:,0], -ab[:,0]*ab[:,1], -ab[:,0]*ab[:,2]],1)
mhn_c = np.linalg.norm(mh,1 and 1, ) if False else np.linalg.norm(mh,axis=1)
xvn = np.linalg.norm(xv,axis=1)
crs = np.cross(ab,mh)
thm_c = np.degrees(np.arctan2(np.sum(crs*xv,1)/(mhn_c*xvn), np.sum(mh*xv,1)/(mhn_c*xvn)))
print()
print('=== 固件实现 vs 用日志复算 ===')
ok = good & (mhn_c > 1e-3) & (xvn > 1e-3) & (amn > 0.9) & (amn < 1.1)
d1 = np.abs(((thm_c - thm_rep + 180) % 360 - 180)[ok])
print('  cmp_thm 复算 vs 上报: 中位差 %.4f 度, p90 %.4f 度' % (np.median(d1), np.percentile(d1,90)))
print('  cmp_mhn 复算 vs 上报: 中位差 %.5f' % np.median(np.abs(mhn_c-mhn)[ok]))
print('  cmp_amn 复算(用 accel_g) vs 上报(raw_f): 中位差 %.4f g' % np.median(np.abs(an-amn)[ok]))
print('   -> cmp_amn 用 raw_f_mps2(离线常量), accel_g 是用在线零偏重算的比力; 两者差 = 在线零偏/g')

# ---- 重力方向判断：raw_f 推出的 a_up 与 accel_g 推出的 a_up 夹角 ----
dot = np.clip(np.sum(ab*(acc/np.maximum(an,1e-9)[:,None]),1),-1,1)
print()
print('=== 重力方向 ===')
st = good & (np.linalg.norm(b[:,26:29],axis=1) < 1.0) & (amn > 0.9) & (amn < 1.1)
print('  静止且 |a|~1g 的帧: %d (%.1f%%)' % (st.sum(), 100*st.mean()))
acc_med = acc[st]
print('  静止时 accel_g 三分量中位 = %s   (机体 z 应 ~ +1.0 表示"上")' % np.round(np.median(acc_med,0),4))
print('  静止时 |accel_g| 中位 %.4f ; cmp_amn 中位 %.4f' % (np.median(an[st]), np.median(amn[st])))

# ---- 新息与收敛 ----
innov = (thm_rep - thp_rep + 180) % 360 - 180
print()
print('=== 新息（带符号 = cmp_thm - cmp_thp）与收敛 ===')
print('  |新息| p50 %.3f p90 %.3f max %.3f 度' % (np.median(np.abs(innov)), np.percentile(np.abs(innov),90), np.abs(innov).max()))
print('  mag_r(119) 与 |新息| 中位差 %.4f 度 (应~0)' % np.median(np.abs(b[:,119]-np.abs(innov))))
u = (b[:,120]==1) & good
print('  mag_used==1 占 %.1f%%' % (100*u.mean()))
m2 = u & (np.abs(b[:,137]) > 1e-4)
sg = 100*(np.sign(innov[m2]) == np.sign(b[m2,137])).mean()
print('  同号(=收敛) 占 %.1f%% ; corr(新息,dqz) = %+.3f' % (sg, np.corrcoef(innov[m2], b[m2,137])[0,1]))

# ---- 静止精度与按 |a| 分档 ----
print()
print('=== 静止段(|w|<1) 航向精度，按 cmp_amn 分档 ===')
for lo, hi in [(0.97,1.03),(0.90,0.97),(1.03,1.10),(1.10,99)]:
    q = good & st & (amn >= lo) & (amn < hi)
    if q.sum() < 200: continue
    print('  amn %.2f~%.2f : n=%6d  |新息| p50 %7.3f p90 %8.3f 度' %
          (lo, hi, q.sum(), np.median(np.abs(innov[q])), np.percentile(np.abs(innov[q]),90)))
print()
print('  t(s)  |w|p50  amn p50  mhn p50   |新息|p50  used%%   EKF偏航  旧链偏航')
for lo in range(0, int(t[-1]), 3):
    q = (t>=lo)&(t<lo+3)&good
    if q.sum()<100: continue
    print('%5d %7.1f %8.3f %8.3f %10.3f %6.0f %9.2f %9.2f'
          % (lo, np.median(np.linalg.norm(b[q,26:29],axis=1)), np.median(amn[q]),
             np.median(mhn[q]), np.median(np.abs(innov[q])), 100*(b[q,120]==1).mean(),
             np.median(b[q,89]), np.median(b[q,0])))
