# -*- coding: utf-8 -*-
# 检查: 现有"水平投影 2 维观测"对倾角误差的灵敏度, 以及能否构造出倾角灵敏度为零的观测
import numpy as np
D = np.radians(-7.53); I = np.radians(64.32)
ci, si = np.cos(I), np.sin(I)
B0 = np.array([ci*np.sin(D), ci*np.cos(D), -si])          # 真世界磁场(导航系)
print('B0 = (%.4f, %.4f, %.4f)  |B0|=%.4f  水平 |v0|=%.4f  b0z=%.4f'
      % (*B0, np.linalg.norm(B0), np.hypot(B0[0], B0[1]), B0[2]))

def ExpR(v):
    """旋转矢量 -> R"""
    a = np.linalg.norm(v)
    if a < 1e-12: return np.eye(3)
    k = v/a
    K = np.array([[0,-k[2],k[1]],[k[2],0,-k[0]],[-k[1],k[0],0]])
    return np.eye(3) + np.sin(a)*K + (1-np.cos(a))*(K@K)

def Rv(v):    # 旋转矢量 -> 四元数(w,x,y,z)
    a = np.linalg.norm(v)
    if a < 1e-12: return np.array([1,0,0,0.0])
    return np.concatenate([[np.cos(a/2)], np.sin(a/2)*v/a])
def qm(a,b):
    return np.array([a[0]*b[0]-a[1]*b[1]-a[2]*b[2]-a[3]*b[3],
                     a[0]*b[1]+a[1]*b[0]+a[2]*b[3]-a[3]*b[2],
                     a[0]*b[2]-a[1]*b[3]+a[2]*b[0]+a[3]*b[1],
                     a[0]*b[3]+a[1]*b[2]-a[2]*b[1]+a[3]*b[0]])
def quat_yaw(q):
    w,x,y,z = q
    return np.arctan2(2*(w*z+x*y), 1-2*(y*y+z*z))

# 真姿态: 取一个有倾角的机体
R = ExpR(np.radians([25.0, -15.0, 40.0]))      # body->nav
f = R.T @ B0                                   # 机体系测得磁场(理想)
up_nav = np.array([0,0,1.0])
a_body = R.T @ up_nav                          # 加计测得的"上"(机体系)

print()
print('=== A) 现有观测 r = (Bn_x-b0x, Bn_y-b0y) 的雅可比 (对导航系误差 dth) ===')
J = np.zeros((2,3))
for k in range(3):
    d = np.zeros(3); d[k] = 1e-6
    Bn = (ExpR(d) @ R) @ f
    J[:,k] = (Bn[:2] - B0[:2]) / 1e-6
np.set_printoptions(precision=4, suppress=True)
print('            dth_x      dth_y      dth_z')
for i,ax in enumerate(['Bn_x','Bn_y']):
    print('  %-6s %9.4f %10.4f %10.4f' % (ax, J[i,0], J[i,1], J[i,2]))
print('  -> 倾角灵敏度 %.4f ; 偏航灵敏度 %.4f ; 倾角/偏航 = %.2f 倍'
      % (np.hypot(J[0,1],J[1,0]), np.hypot(J[0,2],J[1,2]),
         np.hypot(J[0,1],J[1,0])/np.hypot(J[0,2],J[1,2])))

print()
print('=== B) 只取"绕重力轴的分量"(用加计测得的 a_body 当轴) ===')
def y_new(dth):
    Rhat = ExpR(dth) @ R
    f_pred = Rhat.T @ B0
    # 绕 a_body 轴的夹角: (f x f_pred).a_body
    return np.dot(np.cross(f, f_pred), a_body) / (np.linalg.norm(f)*np.linalg.norm(f_pred))
Jn = np.zeros(3)
for k in range(3):
    d = np.zeros(3); d[k] = 1e-6
    Jn[k] = (y_new(d) - y_new(np.zeros(3))) / 1e-6
print('   dth_x %+9.4f   dth_y %+9.4f   dth_z %+9.4f' % tuple(Jn))
print('   -> 倾角灵敏度 %.4f ; 偏航灵敏度 %.4f ; 倾角/偏航 = %.2f 倍'
      % (np.hypot(Jn[0],Jn[1]), abs(Jn[2]), np.hypot(Jn[0],Jn[1])/abs(Jn[2])))

print()
print('=== C) 标量磁航向观测: 测量用"加计重力轴", 预测只用姿态的偏航 ===')
def theta_m(f, a):
    e1 = np.array([1.0,0,0]) - np.dot([1.0,0,0], a)*a
    e1 /= np.linalg.norm(e1); e2 = np.cross(a, e1)
    return np.arctan2(np.dot(f,e2), np.dot(f,e1))
def y_head(dth):
    Rhat = ExpR(dth) @ R
    th_p = quat_yaw(Rv(np.zeros(3)) if False else qm(Rv(dth), Rv(np.radians([25.0,-15.0,40.0]))))
    return theta_m(f, a_body) - th_p
Jh = np.zeros(3)
for k in range(3):
    d = np.zeros(3); d[k] = 1e-6
    Jh[k] = (y_head(d) - y_head(np.zeros(3))) / 1e-6
print('   dth_x %+9.4f   dth_y %+9.4f   dth_z %+9.4f' % tuple(Jh))
print('   -> 倾角灵敏度 %.4f ; 偏航灵敏度 %.4f ; 倾角/偏航 = %.2f 倍'
      % (np.hypot(Jh[0],Jh[1]), abs(Jh[2]), np.hypot(Jh[0],Jh[1])/abs(Jh[2])))
print()
print('   注: theta_m 只由 (f, a_body) 两个**测量**决定, 与 dth 无关 ->')
print('       它是常数, 所以 y_head 的雅可比 = -d(quat_yaw)/d(dth), 即纯偏航通道的雅可比')
