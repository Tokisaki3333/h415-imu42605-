# -*- coding: utf-8 -*-
"""VER=13 -> 14：修一次性对齐里偏航旋转的符号。

h = atan2(Bx, By)（B = R(q)·f，导航系 x=东 y=北）。对 q <- Exp(phi*z) (x) q：
    R_z(phi)·(0,1,0) = (-sin phi, cos phi, 0)  =>  h_new = atan2(-sin phi, cos phi) = h - phi
所以 dh/dphi = -1（H[0][8] = -1 是对的），要把 h 拉到 D 需要 phi = azi - D。
原来写的是 D - azi -> 对齐后 h = 2*azi - D，初始偏航差了一倍。
后果实测（VER=13）：nis_mag 恒 5.2e3（残差约 50 度）、chi2 剔 10%、
mag_yaw 门只有 36%（全靠逃脱阀软拉）、|a_nav| 峰值 14 m/s^2（姿态不对，
去重力抵不掉）、bg_z 被这个持续偏航差推成 -1.96 dps。
"""
import shutil

P = r'C:\Users\33\Documents\v2\h415-imu42605-\V5F\User\src\proc_ekf.c'
T = r'C:\Users\33\Documents\v2\h415-imu42605-\V5F\User\inc\v5f_tune.h'

t = open(P, 'rb').read().decode('gbk')
a = """            azi  = atan2f(Bn[0], Bn[1]);
            dpsi = wrap_pi(V5F_MAG_DECL_RAD - azi);"""
b = """            azi  = atan2f(Bn[0], Bn[1]);
            /* ★ 符号：h = atan2(Bx, By)，而 q <- Exp(phi z) (x) q 使 h_new = h - phi
             *   （R_z(phi)·(0,1,0) = (-sin phi, cos phi, 0)）。所以要把 h 拉到 D，
             *   需要 phi = azi - D。写成 D - azi 会把偏差放大成两倍（对齐后 h = 2*azi - D）。
             *   实测后果：nis_mag 恒 5.2e3、chi2 剔 10%、|a_nav| 峰值 14 m/s^2。 */
            dpsi = wrap_pi(azi - V5F_MAG_DECL_RAD);"""
assert t.count(a) == 1, t.count(a)
t = t.replace(a, b, 1)
assert t.count('/*') == t.count('*/')
shutil.copy2(P, P + '.bak_s1f')
open(P, 'wb').write(t.encode('gbk'))
print('对齐符号已修: dpsi = azi - D')

u = open(T, 'rb').read().decode('gbk')
assert u.count('#define V5F_FW_VER        13u') == 1
u = u.replace('#define V5F_FW_VER        13u', '#define V5F_FW_VER        14u', 1)
shutil.copy2(T, T + '.bak_s1f')
open(T, 'wb').write(u.encode('gbk'))
print('VER 13 -> 14')
print('fw_tag 期望 = %d' % ((14 << 16) | (112 << 8) | 1 | 2 | 4))

# 数值自检：用固件同一套公式验证 H 的符号与对齐公式
import math
def qmul(a, b):
    w = a[0]*b[0]-a[1]*b[1]-a[2]*b[2]-a[3]*b[3]
    x = a[0]*b[1]+a[1]*b[0]+a[2]*b[3]-a[3]*b[2]
    y = a[0]*b[2]-a[1]*b[3]+a[2]*b[0]+a[3]*b[1]
    z = a[0]*b[3]+a[1]*b[2]-a[2]*b[1]+a[3]*b[0]
    return [w, x, y, z]
def q2R(q):
    w, x, y, z = q
    return [[1-2*(y*y+z*z), 2*(x*y-w*z), 2*(x*z+w*y)],
            [2*(x*y+w*z), 1-2*(x*x+z*z), 2*(y*z-w*x)],
            [2*(x*z-w*y), 2*(y*z+w*x), 1-2*(x*x+y*y)]]
def rot(R, v):
    return [sum(R[i][j]*v[j] for j in range(3)) for i in range(3)]
def azi(q, f):
    B = rot(q2R(q), f)
    return math.degrees(math.atan2(B[0], B[1]))

D = -7.53
f = [0.1, 0.9, -0.4]                      # 任意机体系地磁方向
a0 = azi([1, 0, 0, 0], f)
print()
print('数值自检（固件同一套公式）：')
print('  初始 azi = %.4f 度' % a0)
for phi_deg in (10.0, -10.0):
    ph = math.radians(phi_deg)
    q = [math.cos(ph/2), 0, 0, math.sin(ph/2)]
    print('  q <- Exp(%+.0f 度 z)(x)q  ->  azi = %+.4f 度   (azi 变化 %+.4f => dh/dphi = %+.3f)'
          % (phi_deg, azi(q, f), azi(q, f) - a0, (azi(q, f) - a0)/phi_deg))
# 对齐：phi = azi - D 之后 h 应等于 D
ph = math.radians(a0 - D)
q = [math.cos(ph/2), 0, 0, math.sin(ph/2)]
print('  用 dpsi = azi - D 对齐后 azi = %.4f 度  (应为 D = %.2f)' % (azi(q, f), D))
ph = math.radians(D - a0)
q = [math.cos(ph/2), 0, 0, math.sin(ph/2)]
print('  旧公式 dpsi = D - azi 对齐后 azi = %.4f 度  (偏差 %.4f 度)'
      % (azi(q, f), azi(q, f) - D))
