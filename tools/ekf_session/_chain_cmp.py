# -*- coding: utf-8 -*-
# 旧通路(att.q, 0~3) vs EKF通路(ekf_q, 89~92) 的偏航对比
import numpy as np
P = r'R:\raw_v9.bin'; NCH = 144
r = np.fromfile(P, dtype='<f4'); N = r.size // NCH
b = r[:N*NCH].reshape(-1, NCH).astype(np.float64); t = np.cumsum(b[:,25]*1e-6)
R2D = 57.2957795

def yaw_of(q):
    w, x, y, z = q[:,0], q[:,1], q[:,2], q[:,3]
    n = np.sqrt(w*w+x*x+y*y+z*z); n[n < 1e-9] = 1e-9
    w, x, y, z = w/n, x/n, y/n, z/n
    return np.degrees(np.arctan2(2.0*(w*z + x*y), 1.0 - 2.0*(y*y + z*z)))

yl = yaw_of(b[:, 0:4])      # 旧链偏航
ye = yaw_of(b[:, 89:93])    # EKF 偏航
gyro_z = b[:, 28]
gn = np.linalg.norm(b[:, 26:29], axis=1)
print('时长 %.2f s' % t[-1])
print()
print('   t(s)   |w|p50  旧链偏航    EKF偏航    两者差   旧链转动率  EKF转动率  |>90度阀?|')
prev = None
for lo in np.arange(0, t[-1], 1.0):
    m = (t >= lo) & (t < lo+1.0)
    if m.sum() < 200: continue
    idx = np.where(m)[0]
    dl = (yl[idx[-1]] - yl[idx[0]] + 180) % 360 - 180
    de = (ye[idx[-1]] - ye[idx[0]] + 180) % 360 - 180
    print('%6.1f %8.1f %9.2f %10.2f %9.2f %11.2f %10.2f' % (
        lo, np.median(gn[m]), np.median(yl[m]), np.median(ye[m]),
        (np.median(ye[m]) - np.median(yl[m]) + 180) % 360 - 180, dl, de))
print()
print('=== 旧链与 EKF 偏航角速度对比（谁在跟着板子转）===')
wz = b[:, 28] - b[:, 101]
print('  陀螺 z 积分 vs 旧链 yaw vs EKF yaw 的累计变化')
cum_g = np.cumsum(wz) * (b[:,25]*1e-6) * 0          # 占位
# 分段累计: 用差分法
dyl = np.diff(np.unwrap(np.radians(yl)))*R2D
dye = np.diff(np.unwrap(np.radians(ye)))*R2D
wzr = wz[:-1]*b[:-1,25]*1e-6
for lo in np.arange(0, t[-1], 2.0):
    m = (t[:-1] >= lo) & (t[:-1] < lo+2.0)
    if m.sum() < 500: continue
    print('  t %4.1f~%4.1f: 陀螺积分 %+9.1f 度   旧链 %+9.1f 度   EKF %+9.1f 度   |w|p50 %7.1f' % (
        lo, lo+2, wzr[m].sum(), dyl[m].sum(), dye[m].sum(), np.median(gn[:-1][m])))
print()
print('=== EKF 偏航是否被钉住: 相邻 EKF 周期(23帧)的偏航变化 ===')
K = 23
d23e = (ye[K:] - ye[:-K] + 180) % 360 - 180
d23g = np.array([ (b[i:i+K,28]*b[i:i+K,25]*1e-6).sum() for i in range(0, len(b)-K, K) ])
print('  陀螺应有转角 p50 %.4f p90 %.3f 度' % (np.median(abs(d23g)), np.percentile(abs(d23g),90)))
print('  EKF 偏航实际变化 p50 %.4f p90 %.3f 度' % (np.median(abs(d23e[::K])), np.percentile(abs(d23e[::K]),90)))
dd = d23e[::K] - d23g
print('  差(实际-陀螺) p50 %+.4f p90 %+.3f p99 %+.3f 度' % (np.median(dd), np.percentile(dd,90), np.percentile(dd,99)))
print()
print('=== 关键: EKF 偏航是否跟着板子转 ===')
print('  相关性 corr(陀螺积分, EKF偏航变化) = %+.3f' % np.corrcoef(d23g, d23e[::K])[0,1])
print('  相关性 corr(陀螺积分, 旧链偏航变化) = %+.3f' % np.corrcoef(d23g, ((yl[K:]-yl[:-K]+180)%360-180)[::K])[0,1])
print('  EKF偏航变化 / 陀螺积分 中位比值 = %+.3f  (1.0=完全跟随, 0=被钉住)' %
      np.median(d23e[::K]/np.where(abs(d23g)<1e-6, np.nan, d23g)))
print('  旧链偏航变化 / 陀螺积分 中位比值 = %+.3f' %
      np.median(((yl[K:]-yl[:-K]+180)%360-180)[::K]/np.where(abs(d23g)<1e-6, np.nan, d23g)))
