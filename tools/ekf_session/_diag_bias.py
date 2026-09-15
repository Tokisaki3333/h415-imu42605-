# -*- coding: utf-8 -*-
# 检验用户的诊断: 磁环把机体系磁场当成"机头永远朝北",
# 而真正的转动被倾角观测(M6, mask 含 bg)吸收成陀螺零偏
import numpy as np
P = r'R:\raw_v9.bin'; NCH = 144
r = np.fromfile(P, dtype='<f4'); N = r.size // NCH
b = r[:N*NCH].reshape(-1, NCH).astype(np.float64); t = np.cumsum(b[:,25]*1e-6)
print('fw_tag %d VER=%d  时长 %.2f s' % (int(round(b[0,76])), int(round(b[0,76]))>>16, t[-1]))

# ---- 1) 机体系磁场 mag.f 是否跟着机体转 ----
f = b[:, 42:45]; fn = np.linalg.norm(f, axis=1)
fu = f / np.maximum(fn, 1e-9)[:, None]
K = 23
df = np.degrees(np.arccos(np.clip(np.sum(fu[K:]*fu[:-K], axis=1), -1, 1)))
gth = np.linalg.norm(b[:-K, 26:29], axis=1)*K*b[:-K, 25]*1e-6
print()
print('=== 1) 机体系磁场 mag.f 的转动 vs 陀螺转动 (每 23 帧) ===')
print('  |Δf| p50 %.3f p90 %.3f 度 ; 陀螺转动 p50 %.3f p90 %.3f 度' %
      (np.median(df), np.percentile(df,90), np.median(gth), np.percentile(gth,90)))
m = gth > 1.0
print('  陀螺转>1度的样本: n=%d  corr(|Δf|, 陀螺) = %+.3f  比值中位 %.3f  (1.0=跟着转)' %
      (m.sum(), np.corrcoef(df[m], gth[m])[0,1], np.median(df[m]/gth[m])))

# ---- 2) EKF 陀螺零偏 bg (99,100,101) ----
bg = b[:, 99:102]
print()
print('=== 2) EKF 陀螺零偏 bg (dps) ===')
for i, ax in enumerate('xyz'):
    v = bg[:, i]
    print('  bg_%s: p50 %+8.4f  p90 %+8.4f  p99 %+8.4f  min %+8.3f  max %+8.3f' %
          (ax, np.median(v), np.percentile(v,90), np.percentile(v,99), v.min(), v.max()))
print('  (固件钳位 ±%.1f dps)' % 10.0)
print()
print('   t(s)   |w|p50   bg_x     bg_y     bg_z    |w_z-bg_z|  EKF偏航   旧链偏航')
def yaw_of(q):
    w,x,y,z = q[:,0],q[:,1],q[:,2],q[:,3]
    n = np.sqrt(w*w+x*x+y*y+z*z); n[n<1e-9]=1e-9
    return np.degrees(np.arctan2(2*(w/n)*(z/n)+2*(x/n)*(y/n), 1-2*((y/n)**2+(z/n)**2)))
ye = yaw_of(b[:,89:93]); yl = yaw_of(b[:,0:4])
for lo in np.arange(0, t[-1], 1.0):
    m = (t >= lo) & (t < lo+1.0)
    if m.sum() < 200: continue
    print('%6.1f %8.1f %+8.4f %+8.4f %+8.4f %10.2f %10.2f %10.2f' % (
        lo, np.median(np.linalg.norm(b[m,26:29],axis=1)),
        np.median(bg[m,0]), np.median(bg[m,1]), np.median(bg[m,2]),
        np.median(b[m,28]-b[m,101]), np.median(ye[m]), np.median(yl[m])))

# ---- 3) 正北牵引: 偏航误差 psi 与 EKF 偏航的关系 ----
B0X,B0Y = -0.0567804, 0.4296474
psi = np.degrees(np.arctan2(B0X*b[:,131]-B0Y*b[:,130], B0X*b[:,130]+B0Y*b[:,131]))
print()
print('=== 3) 磁环目标: 它把 (Bn_x,Bn_y) 拉向 v0=(%.4f,%.4f) ===' % (B0X,B0Y))
print('  mag_r p50 %.2f 度 ; mag_bh p50 %.3f' % (np.median(b[:,119]), np.median(b[:,118])))
print('  EKF 偏航 p50 %.1f 度 ; 旧链偏航 p50 %.1f 度 ; 差 %.1f 度' % (
    np.median(ye), np.median(yl), (np.median(ye)-np.median(yl)+180)%360-180))
