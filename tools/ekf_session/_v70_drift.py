# -*- coding: utf-8 -*-
"""VER=70 判决 + 定位"偶尔大漂"：
  ekf_yaw 的变化 = 陀螺竖直投影（真实转动） + 修正驱动（观测造成）
  把两者分开，找出"陀螺没动、修正却把偏航推走"的时段，并记录当时的磁量。
"""
import glob, os, time
import numpy as np

P = max([c for c in glob.glob(r'R:\*.bin') if os.path.getsize(c) > 100000], key=os.path.getmtime)
r = np.fromfile(P, dtype='<f4'); n = 144
b = r[:r.size//n*n].reshape(-1, n).astype(np.float64); N = len(b)
dt = b[:, 25]*1e-6; t = np.cumsum(dt)
print('%s %s 帧%d 时长%.2fs fw_tag %d'
      % (os.path.basename(P), time.strftime('%H:%M:%S', time.localtime(os.path.getmtime(P))),
         N, t[-1], b[0, 76]))


def qn(q):
    nn = np.sqrt((q*q).sum(axis=1)); nn = np.where((~np.isfinite(nn)) | (nn < 1e-9), 1, nn)
    return q/nn[:, None]
def Rq(q):
    q = qn(q); w, x, y, z = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
    R = np.empty((len(q), 3, 3))
    R[:, 0, 0] = 1-2*(y*y+z*z); R[:, 0, 1] = 2*(x*y-w*z); R[:, 0, 2] = 2*(x*z+w*y)
    R[:, 1, 0] = 2*(x*y+w*z); R[:, 1, 1] = 1-2*(x*x+z*z); R[:, 1, 2] = 2*(y*z-w*x)
    R[:, 2, 0] = 2*(x*z-w*y); R[:, 2, 1] = 2*(y*z+w*x); R[:, 2, 2] = 1-2*(x*x+y*y)
    return R
def yawf(q):
    q = qn(q); w, x, y, z = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
    return np.arctan2(2*(w*z+x*y), 1-2*(y*y+z*z))
def wr(d): return (d+180.0) % 360.0-180.0

ye, yl = yawf(b[:, 89:93]), yawf(b[:, 0:4])
w = b[:, 26:29]*np.pi/180.0                       # body gyro rad/s
Rl = Rq(b[:, 0:4])
up = np.zeros((N, 3)); up[:, 2] = 1.0
w_nav = np.einsum('nij,nj->ni', Rl, w)            # 陀螺转到导航系
gyaw = np.degrees(w_nav[:, 2])                    # 竖直分量 = 真实偏航角速度
ekf_rate = np.degrees(wr(np.diff(ye)))/dt[:-1]    # EKF 偏航角速度
gy_rate = gyaw[:-1]
corr = ekf_rate - gy_rate                         # 修正驱动部分
print('EKF偏航速率 p50 %.3f p90 %.3f max %.1f 度/秒' % (np.median(np.abs(ekf_rate)), np.percentile(np.abs(ekf_rate), 90), np.abs(ekf_rate).max()))
print('陀螺竖直    p50 %.3f p90 %.3f max %.1f 度/秒' % (np.median(np.abs(gy_rate)), np.percentile(np.abs(gy_rate), 90), np.abs(gy_rate).max()))
print('修正驱动    p50 %.3f p90 %.3f max %.1f 度/秒' % (np.median(np.abs(corr)), np.percentile(np.abs(corr), 90), np.abs(corr).max()))
print('修正驱动占 EKF 总变化的比例 p50 %.1f%%' % (100*np.median(np.abs(corr))/max(np.median(np.abs(ekf_rate)), 1e-9)))

# 积分"修正驱动"得到修正造成的累计偏航位移（每 0.5s 一段）
tgt = 5.0/1000.0
seg = []
acc = 0.0; t0 = t[0]
for i in range(len(corr)):
    acc += corr[i]*dt[i]
    if t[i] - t0 >= 0.25:
        seg.append((t0, t[i], acc)); acc = 0.0; t0 = t[i]
seg = np.array(seg)
print('\n每 0.25 秒内"修正造成的偏航位移"最大的 10 段：')
idx = np.argsort(-np.abs(seg[:, 2]))[:10]
mr, mbh, mu, mdq = b[:, 119], b[:, 118], b[:, 120], b[:, 137]
gb = b[:, 103].astype(int)
print('   时段(s)         修正位移(度)   期间 mag_r p50  mag_bh p50  used%%  mag_dqz均值  gate')
for k in sorted(idx):
    s, e, v = seg[k]
    m = (t >= s) & (t < e)
    if m.sum() < 3: continue
    print('  %5.2f-%5.2f      %+8.2f      %8.2f     %7.3f    %4.0f%%   %+8.4f   %d'
          % (s, e, v, np.median(mr[m]), np.median(mbh[m]), 100*mu[m].mean(), np.mean(mdq[m]), np.median(gb[m])))
print('\n全程 mag_r: p50 %.2f p90 %.2f max %.2f | <12度(死区)占 %.1f%%'
      % (np.median(abs(mr)), np.percentile(abs(mr), 90), abs(mr).max(), 100*np.mean(abs(mr) < 12)))
print('mag_used=1 占 %.1f%%  p_yy p50 %.4g 末 %.4g' % (100*mu.mean(), np.median(b[:, 121]), b[-1, 121]))
print('mag_dqz |.| p50 %.4f p90 %.4f max %.3f' % (np.median(abs(mdq)), np.percentile(abs(mdq), 90), abs(mdq).max()))
