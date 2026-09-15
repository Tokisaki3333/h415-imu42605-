# -*- coding: utf-8 -*-
# VER=81 验收：轻量摆动下，用**旧路径**当参考校验 EKF
#  旧路径在轻摆时可信 -> att.q(0~3) 是参考, ekf_q(89~92) 是被检对象
#  若两者都正确, 相对旋转 R_legacy^T R_ekf 应当是一个**常值偏航**(绕垂直轴),
#  其倾角分量应≈0、偏航分量应恒定。
import numpy as np
P = r'R:\raw_v9.bin'; NCH = 148
r = np.fromfile(P, dtype='<f4'); N = r.size // NCH
b = r[:N*NCH].reshape(-1, NCH).astype(np.float64); t = np.cumsum(b[:,25]*1e-6)
R2D = 57.2957795
TAG = int(round(b[0,76]))
print('fw_tag %d -> VER=%d  帧 %d  列 %d  时长 %.2f s  (VER=81 应 5346311)'
      % (TAG, TAG>>16, N, NCH, t[-1]))
print('|w| p50 %.2f p90 %.2f max %.1f dps  (轻量摆动)' %
      (np.median(np.linalg.norm(b[:,26:29],axis=1)),
       np.percentile(np.linalg.norm(b[:,26:29],axis=1),90),
       np.linalg.norm(b[:,26:29],axis=1).max()))
B0X,B0Y=-0.0567804,0.4296474
good=((np.abs(b[:,132]-B0X)<5e-3)&(np.abs(b[:,133]-B0Y)<5e-3)&(b[:,118]>0)&(b[:,119]>=0)&((b[:,120]==0)|(b[:,120]==1)))
print('好帧 %.2f%%' % (100*good.mean()))

def qn_(q):
    n=np.linalg.norm(q,axis=1); n[n<1e-9]=1e-9; return q/n[:,None]
def R_of(q):
    w,x,y,z=q[:,0],q[:,1],q[:,2],q[:,3]
    return np.stack([np.stack([1-2*(y*y+z*z),2*(x*y-w*z),2*(x*z+w*y)],-1),
                     np.stack([2*(x*y+w*z),1-2*(x*x+z*z),2*(y*z-w*x)],-1),
                     np.stack([2*(x*z-w*y),2*(y*z+w*x),1-2*(x*x+y*y)],-1)],-2)
def yaw_of(q):
    w,x,y,z=q[:,0],q[:,1],q[:,2],q[:,3]
    n=np.sqrt(w*w+x*x+y*y+z*z); n[n<1e-9]=1e-9
    return np.degrees(np.arctan2(2*(w/n)*(z/n)+2*(x/n)*(y/n),1-2*((y/n)**2+(z/n)**2)))
ql=qn_(b[:,0:4]); qe=qn_(b[:,89:93])
yl=yaw_of(ql); ye=yaw_of(qe)
print()
print('=== 1) 偏航：EKF vs 旧链 ===')
d=(ye-yl+180)%360-180
print('  差值 p50 %+.3f  p10 %+.3f  p90 %+.3f  std %.3f 度' %
      (np.median(d),np.percentile(d,10),np.percentile(d,90),np.std(d)))
K=200
dye=(ye[K:]-ye[:-K]+180)%360-180; dyl=(yl[K:]-yl[:-K]+180)%360-180
m=np.abs(dyl)>0.5
print('  转动>0.5度的样本 n=%d' % m.sum())
if m.sum()>50:
    print('  corr(EKF偏航变化, 旧链偏航变化) = %+.4f ; 比值中位 %+.3f'
          % (np.corrcoef(dye[m],dyl[m])[0,1], np.median(dye[m]/dyl[m])))
    print('  |EKF变化| p50 %.3f ; |旧链变化| p50 %.3f 度' % (np.median(np.abs(dye[m])),np.median(np.abs(dyl[m]))))

print()
print('=== 2) 相对旋转 R_rel = R_legacy^T R_ekf：应为一个常值偏航 ===')
Rl=R_of(ql); Re=R_of(qe)
Rr=np.einsum('nji,njk->nik', Rl, Re)          # Rl^T Re
# 相对旋转的四元数 -> 轴角
tr=np.trace(Rr,axis1=1,axis2=2)
ang=np.degrees(np.arccos(np.clip((tr-1)/2,-1,1)))
axis=np.stack([Rr[:,2,1]-Rr[:,1,2], Rr[:,0,2]-Rr[:,2,0], Rr[:,1,0]-Rr[:,0,1]],1)
na=np.linalg.norm(axis,axis=1); na[na<1e-9]=1e-9
axis=axis/na[:,None]
up=np.tile([0.0,0,1],(len(b),1))
tilt_ang=np.degrees(np.arccos(np.clip(np.abs(np.sum(axis*up,1)),-1,1)))  # 轴与垂直轴的夹角
print('  相对转角 p50 %.3f p90 %.3f 度 (应≈|常值偏航|)' % (np.median(ang),np.percentile(ang,90)))
print('  相对轴与垂直轴夹角 p50 %.3f p90 %.3f 度 (若纯偏航应≈0)'
      % (np.median(tilt_ang),np.percentile(tilt_ang,90)))
print('  相对转角的时间稳定度: p10 %.3f p50 %.3f p90 %.3f 度 -> 波动 %.3f 度'
      % (np.percentile(ang,10),np.median(ang),np.percentile(ang,90),
         np.percentile(ang,90)-np.percentile(ang,10)))

print()
print('=== 3) 磁环诊断 ===')
print('  |新息| p50 %.3f p90 %.3f 度 ; mag_used %.1f%%' %
      (np.median(np.abs(b[:,119])),np.percentile(np.abs(b[:,119]),90),100*(b[:,120]==1).mean()))
print('  cmp_amn(姿态"上"与加计夹角) p50 %.2f p90 %.2f 度 ; cmp_mhn p50 %.3f'
      % (np.median(b[:,146]),np.percentile(b[:,146],90),np.median(b[:,147])))
print('  p_yy p50 %.4f ; sigma_yaw p50 %.2f 度' % (np.median(b[:,121]),np.median(b[:,104])))
u=(b[:,120]==1)&(np.abs(b[:,137])>1e-4)
if u.sum()>10:
    print('  施加 dqz 与新息同号(收敛)占 %.1f%%' % (100*(np.sign((b[:,144]-b[:,145]))==np.sign(b[:,137]))[u].mean()))

print()
print('=== 4) 分时段：偏航差是否漂移 ===')
print('  t(s)  |w|p50  yaw_EKF  yaw_legacy   差     |新息|p50  相对转角')
for lo in range(0,int(t[-1]),3):
    q=(t>=lo)&(t<lo+3)&good
    if q.sum()<100: continue
    print('%5d %7.2f %9.2f %10.2f %8.2f %10.3f %10.3f'
          % (lo,np.median(np.linalg.norm(b[q,26:29],axis=1)),np.median(ye[q]),np.median(yl[q]),
             (np.median(ye[q])-np.median(yl[q])+180)%360-180,np.median(np.abs(b[q,119])),np.median(ang[q])))
