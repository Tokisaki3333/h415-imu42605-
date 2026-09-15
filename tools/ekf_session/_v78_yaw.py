# -*- coding: utf-8 -*-
# 关键: EKF 偏航到底跟不跟板子转; 以及"测量用加计重力 / 预测用姿态"这对不对称
import numpy as np
P = r'R:\raw_v9.bin'; NCH = 148
r = np.fromfile(P, dtype='<f4'); N = r.size // NCH
b = r[:N*NCH].reshape(-1, NCH).astype(np.float64); t = np.cumsum(b[:,25]*1e-6)
R2D = 57.2957795

def yaw_of(q):
    w,x,y,z = q[:,0],q[:,1],q[:,2],q[:,3]
    n = np.sqrt(w*w+x*x+y*y+z*z); n[n<1e-9]=1e-9
    return np.degrees(np.arctan2(2*(w/n)*(z/n)+2*(x/n)*(y/n), 1-2*((y/n)**2+(z/n)**2)))
ye = yaw_of(b[:,89:93]); yl = yaw_of(b[:,0:4])
print('  t(s)   EKF偏航   旧链偏航   两者差    |w|p50   |新息|p50  amn')
for lo in range(0, int(t[-1]), 2):
    q = (t>=lo)&(t<lo+2)
    if q.sum()<100: continue
    print('%5d %9.2f %10.2f %9.2f %9.1f %10.3f %6.3f'
          % (lo, np.median(ye[q]), np.median(yl[q]),
             (np.median(ye[q])-np.median(yl[q])+180)%360-180,
             np.median(np.linalg.norm(b[q,26:29],axis=1)),
             np.median(np.abs(b[q,119])), np.median(b[q,146])))

# 偏航变化 vs 旧链偏航变化（同一把尺子）
K = 100
dye = ((ye[K:]-ye[:-K]+180)%360-180)
dyl = ((yl[K:]-yl[:-K]+180)%360-180)
m = np.abs(dyl) > 5
print()
print('=== 每 %d 帧的偏航变化，两条链对比 ===' % K)
print('  旧链转动 >5度的样本 n=%d' % m.sum())
print('  corr(EKF偏航变化, 旧链偏航变化) = %+.3f' % np.corrcoef(dye[m], dyl[m])[0,1])
print('  EKF 变化/旧链变化 中位比值 = %+.3f   (1.0=完全跟随, 0=被钉住)'
      % np.median(dye[m]/dyl[m]))
print('  |EKF偏航变化| p50 %.2f 度 ; |旧链偏航变化| p50 %.2f 度'
      % (np.median(np.abs(dye[m])), np.median(np.abs(dyl[m]))))

# 测量轴(加计)与姿态倾角是否一致: 用姿态推出的"上" 与 加计方向 的夹角
acc = b[:,32:35]; an = np.linalg.norm(acc,axis=1); ab = acc/np.maximum(an,1e-9)[:,None]
q = b[:,89:93]; qn = np.linalg.norm(q,axis=1); qu = q/np.maximum(qn,1e-9)[:,None]
w_,x_,y_,z_ = qu[:,0],qu[:,1],qu[:,2],qu[:,3]
up_est = np.stack([2*(x_*z_-w_*y_), 2*(y_*z_+w_*x_), 1-2*(x_*x_+y_*y_)],1)
mis = np.degrees(np.arccos(np.clip(np.sum(up_est*ab,1),-1,1)))
print()
print('=== 投影轴不对称检查 ===')
print('  姿态推出的"上" 与 加计方向 的夹角: p50 %.2f p90 %.2f max %.2f 度'
      % (np.median(mis), np.percentile(mis,90), mis.max()))
print('  (测量 thm 用加计轴, 预测 thp 只用姿态偏航 -> 两者相差这个夹角时新息被污染)')
for lo,hi in [(0,1),(1,3),(3,10),(10,30),(30,90)]:
    k = (mis>=lo)&(mis<hi)&(b[:,146]>0.9)&(b[:,146]<1.1)
    if k.sum()<200: continue
    print('   倾角不一致 %2d~%-2d 度: n=%6d  |新息| p50 %7.3f p90 %8.3f  |w|p50 %7.1f'
          % (lo,hi,k.sum(),np.median(np.abs(b[k,119])),np.percentile(np.abs(b[k,119]),90),
             np.median(np.linalg.norm(b[k,26:29],axis=1))))
