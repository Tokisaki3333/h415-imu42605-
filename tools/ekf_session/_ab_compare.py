# -*- coding: utf-8 -*-
# 对比两种"重力法向"来源对航向新息的影响（用 VER=78 日志，改前先验）
#   A(旧/VER=78):  a_up 取加计方向, 预测 thp = psi_hat - atan2(b0y,b0x)   <- 不对称
#   B(新/建议)  :  a_up = R(q_hat)^T z_nav  (直接由已知姿态解出), 测量与预测同一根轴
import numpy as np
P = r'R:\raw_v9.bin'; NCH = 148
r = np.fromfile(P, dtype='<f4'); N = r.size // NCH
b = r[:N*NCH].reshape(-1, NCH).astype(np.float64); t = np.cumsum(b[:,25]*1e-6)
D = -7.53; ci = 1/np.sqrt(1+2.08**2)
B0 = np.array([ci*np.sin(np.radians(D)), ci*np.cos(np.radians(D)), -2.08*ci])
acc = b[:,32:35]; magf = b[:,42:45]
an = np.linalg.norm(acc,axis=1); fn = np.linalg.norm(magf,axis=1)
fu = magf/np.maximum(fn,1e-9)[:,None]
q = b[:,89:93]; qn = np.linalg.norm(q,axis=1); qu = q/np.maximum(qn,1e-9)[:,None]

def R_of(qu):
    w,x,y,z = qu[:,0],qu[:,1],qu[:,2],qu[:,3]
    return np.stack([np.stack([1-2*(y*y+z*z),2*(x*y-w*z),2*(x*z+w*y)],-1),
                     np.stack([2*(x*y+w*z),1-2*(x*x+z*z),2*(y*z-w*x)],-1),
                     np.stack([2*(x*z-w*y),2*(y*z+w*x),1-2*(x*x+y*y)],-1)],-2)
R = R_of(qu)
def rot_nb(R,v):      # a = R^T v
    return np.einsum('nji,nj->ni', R, v)
def rot_bn(R,v):      # a = R v
    return np.einsum('nij,nj->ni', R, v)

# B: a_up 由姿态解出
up_nav = np.tile([0.0,0,1],(len(b),1))
ab_B = rot_nb(R, up_nav)
# A: a_up 取加计
ab_A = acc/np.maximum(an,1e-9)[:,None]

def innov(a_up, mode):
    d = np.sum(fu*a_up,1); mh = fu - d[:,None]*a_up
    xv = np.stack([1-a_up[:,0]*a_up[:,0], -a_up[:,0]*a_up[:,1], -a_up[:,0]*a_up[:,2]],1)
    mn = np.linalg.norm(mh,axis=1); xn = np.linalg.norm(xv,axis=1)
    cs = np.cross(a_up, mh)
    thm = np.arctan2(np.sum(cs*xv,1)/(mn*xn), np.sum(mh*xv,1)/(mn*xn))
    if mode == 'A':      # 不对称: 预测只用姿态偏航
        w,x,y,z = qu[:,0],qu[:,1],qu[:,2],qu[:,3]
        psi = np.arctan2(2*(w*z+x*y), 1-2*(y*y+z*z))
        thp = psi - np.arctan2(B0[1], B0[0])
    else:                # B: 预测用同一根轴
        fp = rot_nb(R, np.tile(B0,(len(b),1)))
        d2 = np.sum(fp*a_up,1); mh2 = fp - d2[:,None]*a_up
        m2 = np.linalg.norm(mh2,axis=1)
        cs2 = np.cross(a_up, mh2)
        thp = np.arctan2(np.sum(cs2*xv,1)/(m2*xn), np.sum(mh2*xv,1)/(m2*xn))
    return np.degrees((thm - thp + np.pi) % (2*np.pi) - np.pi)

inA = innov(ab_A, 'A')
inB = innov(ab_B, 'B')
gyr = np.linalg.norm(b[:,26:29],axis=1)
print('  用加计方向推出的"上" 与 由姿态解出的"上" 夹角: p50 %.2f p90 %.2f max %.2f 度'
      % (np.median(np.degrees(np.arccos(np.clip(np.sum(ab_A*ab_B,1),-1,1)))),
         np.percentile(np.degrees(np.arccos(np.clip(np.sum(ab_A*ab_B,1),-1,1))),90),
         np.degrees(np.arccos(np.clip(np.sum(ab_A*ab_B,1),-1,1))).max()))
print()
print('  |w| 档        n       A(加计轴+不对称预测)   B(姿态轴+同轴预测)')
for lo,hi in [(0,1),(1,10),(10,100),(100,500),(500,1500),(1500,3000)]:
    m = (gyr>=lo)&(gyr<hi)
    if m.sum()<200: continue
    print('  %5d~%-5d %7d   |新息| p50 %7.2f p90 %8.2f   |新息| p50 %7.2f p90 %8.2f'
          % (lo,hi,m.sum(),np.median(np.abs(inA[m])),np.percentile(np.abs(inA[m]),90),
             np.median(np.abs(inB[m])),np.percentile(np.abs(inB[m]),90)))
print()
st = gyr < 1.0
print('  静止段: A |新息| p50 %.3f p90 %.3f ; B |新息| p50 %.3f p90 %.3f'
      % (np.median(np.abs(inA[st])),np.percentile(np.abs(inA[st]),90),
         np.median(np.abs(inB[st])),np.percentile(np.abs(inB[st]),90)))
print()
print('  ★ 上报的 mag_r(119) 与 A 的一致性: 中位差 %.4f 度 (确认 A 就是固件在跑的那套)'
      % np.median(np.abs(np.abs(inA)-b[:,119])))
print('  ★ B 相对 A 的新息缩小倍数(运动段 |w|>500): %.1f 倍'
      % (np.median(np.abs(inA[gyr>500]))/max(1e-9,np.median(np.abs(inB[gyr>500])))))
