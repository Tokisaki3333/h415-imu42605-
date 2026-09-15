# -*- coding: utf-8 -*-
# 谁跟不上谁：thm(磁实测航向,col144) / thp(EKF姿态预测航向,col145) / 旧姿态偏航 三者同段变化量
import numpy as np
P = r'R:\raw_v9.bin'; NCH = 148
raw = np.fromfile(P, dtype='<f4'); N = raw.size // NCH
b = raw[:N*NCH].reshape(-1, NCH).astype(np.float64)
dt = b[:, 25]*1e-6; t = np.cumsum(dt)
D = 180/np.pi
yawL = np.unwrap(np.arctan2(2*(b[:, 0]*b[:, 3]+b[:, 1]*b[:, 2]),
                            1-2*(b[:, 2]**2+b[:, 3]**2)))*D
yawE = np.unwrap(np.arctan2(2*(b[:, 89]*b[:, 92]+b[:, 90]*b[:, 91]),
                            1-2*(b[:, 91]**2+b[:, 92]**2)))*D
thm = np.unwrap(np.radians(b[:, 144]))*D
thp = np.unwrap(np.radians(b[:, 145]))*D
inn = (b[:, 144]-b[:, 145]+180) % 360-180
st = (b[:, 8] == 1)
edges = [0]+list(np.where(np.diff(st.astype(np.int8)) != 0)[0]+1)+[N]
seg = [(edges[i], edges[i+1]-1, bool(st[edges[i]])) for i in range(len(edges)-1)]


def m(a, z, x):
    k = max(5, (z-a)//20)
    return np.median(x[z-k:z+1])-np.median(x[a:a+k])


print('  各段内"航向变化量"(度)： 旧姿态 / EKF姿态 / 磁实测thm / 姿态预测thp')
print('  #  起(s)  止(s) 性质 |  dYaw_旧  dYaw_EKF |  d_thm   d_thp  | d_thm-dYaw_旧  d_thp-dYaw_EKF | |r|p50')
for k, (a, z, s) in enumerate(seg):
    dL = yawL[z]-yawL[a]; dE = yawE[z]-yawE[a]
    dm = thm[z]-thm[a]; dp = thp[z]-thp[a]
    print('  %2d %6.2f %6.2f %s | %8.2f %8.2f | %8.2f %8.2f | %10.2f %13.2f | %7.3f'
          % (k, t[a], t[z], '静' if s else '动', dL, dE, dm, dp, dm-dL, dp-dE,
             np.median(np.abs(inn[a:z+1]))))
print()
print('  若 d_thm == dYaw_旧 而 d_thp == dYaw_EKF，则磁实测与旧姿态一致 -> 是 EKF 姿态没跟上')
print()
print('=== 段 11 (12.02~13.00s) 细节：每 0.05 s ===')
a, z, s = seg[11]
for tt in np.arange(0, t[z]-t[a], 0.05):
    i = a+int(np.searchsorted(t[a:z+1]-t[a], tt))
    if i >= z: break
    print('   t=%6.2f  thm %9.3f  thp %9.3f  r %8.2f  旧yaw %9.3f  EKFyaw %9.3f  off %8.3f'
          % (t[i], thm[i], thp[i], inn[i], yawL[i]-yawL[a], yawE[i]-yawE[a],
             (yawE[i]-yawL[i])-(yawE[a]-yawL[a])))
