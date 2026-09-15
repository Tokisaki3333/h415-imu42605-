# -*- coding: utf-8 -*-
# 决定性判别：把"磁实测矢量 mf"与"用EKF姿态把模型场 b0 转到机体系"的差，
# 分解成绕竖直轴(偏航)分量 + 倾斜分量。
#   · 若差主要是绕竖直轴 -> 磁真的在说偏航错了(环没错, 是状态被拉)
#   · 若差主要是倾斜 -> 磁/姿态的时间对不齐(投影泄漏), 环把它当成偏航
# mf = cols 43..45 (|B| 恒定、随机体转, 已验);  b0 由 cols132/133 + |b0|=1 定
import numpy as np
P = r'R:\raw_v9.bin'; NCH = 148
raw = np.fromfile(P, dtype='<f4'); N = raw.size // NCH
b = raw[:N*NCH].reshape(-1, NCH).astype(np.float64)
dt = b[:, 25]*1e-6; t = np.cumsum(dt)
D = 180/np.pi


def rq(q):
    w, x, y, z = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
    n = np.sqrt(w*w+x*x+y*y+z*z); n[n < 1e-9] = 1.0
    w, x, y, z = w/n, x/n, y/n, z/n
    R = np.empty((len(q), 3, 3))
    R[:, 0, 0] = 1-2*(y*y+z*z); R[:, 0, 1] = 2*(x*y-w*z); R[:, 0, 2] = 2*(x*z+w*y)
    R[:, 1, 0] = 2*(x*y+w*z); R[:, 1, 1] = 1-2*(x*x+z*z); R[:, 1, 2] = 2*(y*z-w*x)
    R[:, 2, 0] = 2*(x*z-w*y); R[:, 2, 1] = 2*(y*z+w*x); R[:, 2, 2] = 1-2*(x*x+y*y)
    return R


Re = rq(b[:, 89:93])
b0x, b0y = b[0, 132], b[0, 133]
b0z = -np.sqrt(max(0.0, 1.0-b0x*b0x-b0y*b0y))
print('b0 = (%.6f, %.6f, %.6f)  |b0|=%.6f' % (b0x, b0y, b0z, np.sqrt(b0x*b0x+b0y*b0y+b0z*b0z)))
mf = b[:, 43:46].copy()
mn = np.linalg.norm(mf, axis=1)
mf = mf/mn[:, None]
print('mf(cols43-45): |B| p50 %.4f  p05 %.4f p95 %.4f ; 静止p50 %.4f 运动p50 %.4f'
      % (np.median(mn), np.percentile(mn, 5), np.percentile(mn, 95),
         np.median(mn[b[:, 8] == 1]), np.median(mn[b[:, 8] != 1])))
# 模型场转到机体系: v = R^T b0
v = np.einsum('nji,j->ni', Re, np.array([b0x, b0y, b0z]))
v /= np.linalg.norm(v, axis=1)[:, None]
# ab = 姿态的 up 在机体系 = R^T z
ab = np.einsum('nji,j->ni', Re, np.array([0.0, 0.0, 1.0]))
ab /= np.linalg.norm(ab, axis=1)[:, None]
# 从 v 到 mf 的旋转：轴 = v x mf, 角 = acos(v·mf)
dot = np.clip(np.sum(v*mf, axis=1), -1, 1)
ang = np.degrees(np.arccos(dot))
crs = np.cross(v, mf)
cn = np.linalg.norm(crs, axis=1)
crs = crs/np.maximum(cn, 1e-12)[:, None]
cos_tilt = np.abs(np.sum(crs*ab, axis=1))          # 1=绕竖直轴(纯偏航), 0=纯倾斜
yaw_part = ang*cos_tilt
tilt_part = np.degrees(np.arccos(np.clip(1-2*np.sin(np.radians(ang/2))**2*(1-cos_tilt**2), -1, 1)))
# 更直观: 把旋转角按轴分解成 绕up 与 垂直up 两部分
perp = np.sqrt(np.maximum(0.0, 1-cos_tilt**2))
print()
st = (b[:, 8] == 1)
inn = (b[:, 144]-b[:, 145]+180) % 360-180
print('=== mf 与 模型场(用EKF姿态转到机体系) 的夹角分解 ===')
print('  静止帧: 总夹角 p50 %.3f 度 ; 绕竖直轴分量 p50 %.3f ; 倾斜分量 p50 %.3f'
      % (np.median(ang[st]), np.median(yaw_part[st]), np.median(ang[st]*perp[st])))
print('  运动帧: 总夹角 p50 %.3f p95 %.2f ; 绕竖直轴 p50 %.3f p95 %.2f ; 倾斜 p50 %.3f p95 %.2f'
      % (np.median(ang[~st]), np.percentile(ang[~st], 95),
         np.median(yaw_part[~st]), np.percentile(yaw_part[~st], 95),
         np.median(ang[~st]*perp[~st]), np.percentile(ang[~st]*perp[~st], 95)))
print()
print('=== 与固件上报新息 r 对照 (只在 EKF 周期帧) ===')
cyc = np.concatenate([[True], (np.abs(np.diff(b[:, 89]))+np.abs(np.diff(b[:, 90]))
                               + np.abs(np.diff(b[:, 91]))+np.abs(np.diff(b[:, 92]))) > 0])
ci = np.where(cyc)[0]
om = np.degrees(2*np.linalg.norm(
    np.cross(b[:, 89:93][:-1], b[:, 89:93][1:])[:, 1:] if False else
    (b[1:, 89:93]-b[:-1, 89:93])[:, 1:], axis=1))/dt[1:]
om = np.concatenate([[0], om])
m = om[ci] > 5
print('  运动周期 %d 个' % m.sum())
print('  corr(|r|, 总夹角)=%+.3f  corr(|r|, 绕竖直分量)=%+.3f  corr(|r|, 倾斜分量)=%+.3f'
      % (np.corrcoef(np.abs(inn[ci][m]), ang[ci][m])[0, 1],
         np.corrcoef(np.abs(inn[ci][m]), yaw_part[ci][m])[0, 1],
         np.corrcoef(np.abs(inn[ci][m]), (ang*perp)[ci][m])[0, 1]))
print('  回归 |r| ~ 绕竖直分量: 斜率 %.3f ; ~ 倾斜分量: 斜率 %.3f'
      % (np.linalg.lstsq(np.vstack([yaw_part[ci][m], np.ones(m.sum())]).T, np.abs(inn[ci][m]), rcond=None)[0][0],
         np.linalg.lstsq(np.vstack([(ang*perp)[ci][m], np.ones(m.sum())]).T, np.abs(inn[ci][m]), rcond=None)[0][0]))
print('  corr(r 带符号, mf相对模型绕up的有符号角)=%+.3f'
      % np.corrcoef(inn[ci][m], (yaw_part*np.sign(np.sum(crs*ab, axis=1)*np.sign(np.sum(crs*mf, axis=1))))[ci][m])[0, 1])
print()
print('=== 逐段：mf 与模型场的总夹角 / 绕竖直分量 (p50) ===')
edges = [0]+list(np.where(np.diff(st.astype(np.int8)) != 0)[0]+1)+[N]
for k in range(len(edges)-1):
    a, z = edges[k], edges[k+1]-1
    if z-a < 60:
        continue
    s = bool(st[a])
    print('  %2d %6.2f %6.2f %s  总夹角 %7.3f  绕竖直 %7.3f  倾斜 %7.3f  |r|p50 %7.3f'
          % (k, t[a], t[z], '静' if s else '动', np.median(ang[a:z+1]),
             np.median(yaw_part[a:z+1]), np.median((ang*perp)[a:z+1]),
             np.median(np.abs(inn[a:z+1]))))
