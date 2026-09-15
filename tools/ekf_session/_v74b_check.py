# -*- coding: utf-8 -*-
import numpy as np
P = r'R:\raw_v9.bin'; NCH = 144
r = np.fromfile(P, dtype='<f4'); N = r.size // NCH
b = r[:N*NCH].reshape(-1, NCH).astype(np.float64); t = np.cumsum(b[:,25]*1e-6)
q = b[:, 89:93].copy(); qn = np.linalg.norm(q, axis=1)
print('|q| p50 %.6f  p99 %.2f  max %.0f ; |q|!=1 的帧 %.2f%%' %
      (np.median(qn), np.percentile(qn,99), qn.max(), 100*(abs(qn-1) > 0.01).mean()))

# ★ 正确算法: 先归一化再算夹角
qu = q / np.maximum(qn, 1e-9)[:, None]
dot = np.clip(np.abs(np.sum(qu[1:]*qu[:-1], axis=1)), -1, 1)
d = np.degrees(2*np.arccos(dot))
gyr = np.linalg.norm(b[:,26:29], axis=1)
gth = gyr[:-1]*b[:-1,25]*1e-6
print()
print('归一化后的相邻帧姿态夹角:  p50 %.4f  p90 %.4f  p99 %.2f  max %.1f 度' %
      (np.median(d), np.percentile(d,90), np.percentile(d,99), d.max()))
print('陀螺对应应有转角        :  p50 %.4f  p90 %.4f  p99 %.2f  max %.2f 度' %
      (np.median(gth), np.percentile(gth,90), np.percentile(gth,99), gth.max()))
ex = d - gth
print('超出量: p50 %+.4f p99 %.2f max %.1f 度 ; 超 1 度 %.2f%% ; 超 5 度 %.2f%% ; 超 30 度 %.2f%%' %
      (np.median(ex), np.percentile(ex,99), ex.max(),
       100*(ex > 1).mean(), 100*(ex > 5).mean(), 100*(ex > 30).mean()))
print()
v0ok = (np.abs(b[:,132]+0.0567804) < 5e-3) & (np.abs(b[:,133]-0.4296474) < 5e-3)
print('上报污染帧占 %.2f%%' % (100*(~v0ok).mean()))
print('  污染帧里 姿态跳>30度 的占 %.1f%% ; 正常帧里 %.2f%%' %
      (100*(ex > 30)[~v0ok[1:]].mean() if (~v0ok[1:]).sum() else -1,
       100*(ex > 30)[v0ok[1:]].mean()))
print()
print('=== 污染帧的四元数与相邻好帧的关系 ===')
k = int(np.where(~v0ok)[0][0])
for kk in range(max(1,k-1), min(N, k+3)):
    print('  idx %6d t=%7.3f  风=%s |q|=%8.3f  与上一帧夹角 %6.2f度  v0=(%.4f,%.4f) %s' %
          (kk, t[kk], np.array2string(q[kk], precision=4), qn[kk],
           d[kk-1] if kk > 0 else 0, b[kk,132], b[kk,133], 'BAD' if not v0ok[kk] else ''))
print()
print('=== 各物理量的极值（找根本性问题）===')
lbl = {0:'q0',1:'q1',2:'q2',3:'q3',26:'wx',28:'wz',32:'ax',34:'az',45:'mag_norm',
       49:'psi_true',93:'an_x',95:'an_z',103:'gate_bits',118:'mag_bh',119:'mag_r',
       121:'p_yy',127:'mag_fhb',134:'yawpre'}
for c in sorted(lbl):
    v = b[:, c]
    print('  col %3d %-9s min %12.4f  p50 %10.4f  max %12.4f' % (c, lbl[c], v.min(), np.median(v), v.max()))
an = np.linalg.norm(b[:,32:35], axis=1)
print('  |accel| max %.2f g ; >3g 的帧 %.3f%%' % (an.max(), 100*(an > 3).mean()))
print('  |a_nav| p50 %.3f max %.2f m/s^2' % (np.median(np.linalg.norm(b[:,93:96],axis=1)), np.linalg.norm(b[:,93:96],axis=1).max()))
