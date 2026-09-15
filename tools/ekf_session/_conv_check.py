# -*- coding: utf-8 -*-
# 验证"投影到重力法平面后一定是偏航错"这条判断:
#   若新息是真实偏航误差, 环路施加修正后 |新息| 应当缩小; 若是倾角假象, 则会追不上/来回摆
import numpy as np
P = r'R:\raw_v9.bin'; NCH = 148
r = np.fromfile(P, dtype='<f4'); N = r.size // NCH
b = r[:N*NCH].reshape(-1, NCH).astype(np.float64); t = np.cumsum(b[:,25]*1e-6)
D = -7.53; ci = 1/np.sqrt(1+2.08**2)
B0 = np.array([ci*np.sin(np.radians(D)), ci*np.cos(np.radians(D)), -2.08*ci])
magf = b[:,42:45]; fn = np.linalg.norm(magf,axis=1); fu = magf/np.maximum(fn,1e-9)[:,None]
q = b[:,89:93]; qn=np.linalg.norm(q,axis=1); qu=q/np.maximum(qn,1e-9)[:,None]
w_,x_,y_,z_ = qu[:,0],qu[:,1],qu[:,2],qu[:,3]
up_nav = np.tile([0.0,0,1],(len(b),1))
# a_up = R^T z  (R = body->nav)
Rt = np.stack([np.stack([1-2*(y_*y_+z_*z_),2*(x_*y_+w_*z_),2*(x_*z_-w_*y_)],-1),
               np.stack([2*(x_*y_-w_*z_),1-2*(x_*x_+z_*z_),2*(y_*z_+w_*x_)],-1),
               np.stack([2*(x_*z_+w_*y_),2*(y_*z_-w_*x_),1-2*(x_*x_+y_*y_)],-1)],-2)
ab = np.einsum('nji,nj->ni', Rt.T, up_nav) if False else np.einsum('nij,nj->ni', np.transpose(Rt,(0,2,1)), up_nav)
d = np.sum(fu*ab,1); mh = fu - d[:,None]*ab
fp = np.einsum('nij,nj->ni', np.transpose(Rt,(0,2,1)), np.tile(B0,(len(b),1)))
d2 = np.sum(fp*ab,1); mh2 = fp - d2[:,None]*ab
crs = np.cross(mh, mh2)
inn = np.degrees(np.arctan2(np.sum(crs*ab,1), np.sum(mh*mh2,1)))   # 绕 a_up 的有符号夹角

gyr = np.linalg.norm(b[:,26:29],axis=1)
ch = np.where(np.diff(b[:,134])!=0)[0]+1        # 磁更新边界(用 yawpre 变化检测)
i0,i1 = ch[:-1],ch[1:]
dt = t[i1]-t[i0]; k=(dt>1e-4)&(dt<2e-2); i0,i1=i0[k],i1[k]
dinn = np.abs(inn[i1]) - np.abs(inn[i0])        # 相邻更新间 |新息| 的变化
print('磁更新对 n=%d' % len(dinn))
print('  d|新息| p50 %+0.4f 度 ; 缩小占 %.1f%%' % (np.median(dinn), 100*(dinn<0).mean()))
print()
print('=== 按运动强度分档（看环路能不能收敛它）===')
print('  |w| 档        n       |新息|p50    d|新息|p50   缩小占比')
for lo,hi in [(0,1),(1,10),(10,100),(100,500),(500,1500),(1500,3000)]:
    m=(gyr[i0]>=lo)&(gyr[i0]<hi)
    if m.sum()<200: continue
    print('  %5d~%-5d %7d %12.3f %13.4f %9.1f%%' % (lo,hi,m.sum(),
      np.median(np.abs(inn[i0][m])), np.median(dinn[m]), 100*(dinn[m]<0).mean()))
print()
print('=== 施加的修正量 vs 新息（应 ≈ k_cap*|r| 且方向收敛）===')
u=(b[i0,120]==1)&(np.abs(b[i0,137])>1e-4)
print('  可施加周期 %d ; corr(新息, dqz) = %+.3f ; 同号(收敛)占 %.1f%%'
      % (u.sum(), np.corrcoef(inn[i0][u], b[i0,137][u])[0,1],
         100*(np.sign(inn[i0][u])==np.sign(b[i0,137][u])).mean()))
print('  |dqz| 与 k_cap*|r| 之比中位 %.3f (0.05*|r|*57.3 为上限)'
      % np.median(np.abs(b[i0,137][u])/np.maximum(0.05*np.abs(np.radians(inn[i0][u]))*57.3,1e-9)))
