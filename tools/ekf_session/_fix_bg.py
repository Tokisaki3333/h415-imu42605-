# -*- coding: utf-8 -*-
"""VER=11 -> 12：EKF 预积分必须减去估计的陀螺零偏（结构性 bug）。"""
import shutil

p = r'C:\Users\33\Documents\v2\h415-imu42605-\V5F\User\src\proc_ekf.c'
t = open(p, 'rb').read().decode('gbk')

a = """        raw_w_rads(h, w);
        raw_f_mps2(h, f);
        for (i = 0u; i < 3u; i++) {
            s_dth[i] += w[i] * dt;
            s_dvb[i] += f[i] * dt;
        }"""
b = """        raw_w_rads(h, w);
        raw_f_mps2(h, f);
        /* 必须**减去估计的陀螺零偏**再积分。误差状态里 F 写的是 dth' = -R dbg，
         * 那正是在说"标称角速度 = 量测 - bg_hat"。第一版忘了减：bg 被新息推着一路涨
         * （实测 -803 dps，真值只有 0.0x dps）却从不反馈到姿态，等于让偏航误差全被
         * 一个假偏置状态吃掉 —— 偏航环看着很稳（sigma 0.06 度），实际是拿 bg 当垃圾桶。
         * 加计那一路本来就是对的（下面 dv 里减了 ba*dt_e）。 */
        for (i = 0u; i < 3u; i++) {
            s_dth[i] += (w[i] - s_x[IX_BG + i]) * dt;
            s_dvb[i] += f[i] * dt;
        }"""
assert t.count(a) == 1, t.count(a)
t = t.replace(a, b, 1)

a2 = "    fb[0] = f[0]/e2; fb[1] = f[1]/e2; fb[2] = f[2]/e2;"
b2 = """    for (i = 0u; i < 3u; i++) f[i] -= s_x[IX_BA + i];   /* 观测模型的比力要去掉零偏残差 */
    e2 = sqrtf(f[0]*f[0] + f[1]*f[1] + f[2]*f[2]);
    if (e2 <= 1e-6f) return;
    fb[0] = f[0]/e2; fb[1] = f[1]/e2; fb[2] = f[2]/e2;"""
assert t.count(a2) == 1, t.count(a2)
t = t.replace(a2, b2, 1)

# 上面那句 e2 的旧计算要去掉（改成减 ba 之后再算）
a3 = """    e2 = sqrtf(f[0]*f[0] + f[1]*f[1] + f[2]*f[2]);
    if (e2 <= 1e-6f) return;
    for (i = 0u; i < 3u; i++) f[i] -= s_x[IX_BA + i];"""
assert t.count(a3) == 1, t.count(a3)
t = t.replace(a3, "    for (i = 0u; i < 3u; i++) f[i] -= s_x[IX_BA + i];", 1)
# 重新补回只算一次 e2（放在减 ba 之后）
a4 = """    for (i = 0u; i < 3u; i++) f[i] -= s_x[IX_BA + i];   /* 观测模型的比力要去掉零偏残差 */
    e2 = sqrtf(f[0]*f[0] + f[1]*f[1] + f[2]*f[2]);
    if (e2 <= 1e-6f) return;
    fb[0] = f[0]/e2; fb[1] = f[1]/e2; fb[2] = f[2]/e2;"""
assert t.count(a4) == 1, t.count(a4)

assert t.count('/*') == t.count('*/'), '注释不配平'
shutil.copy2(p, p + '.bak_s1d')
open(p, 'wb').write(t.encode('gbk'))
print('已修: 预积分减去 bg 估计；M6 比力先减 ba 残差再归一化')

q = r'C:\Users\33\Documents\v2\h415-imu42605-\V5F\User\inc\v5f_tune.h'
u = open(q, 'rb').read().decode('gbk')
assert u.count('#define V5F_FW_VER        11u') == 1
u = u.replace('#define V5F_FW_VER        11u', '#define V5F_FW_VER        12u', 1)
shutil.copy2(q, q + '.bak_s1d')
open(q, 'wb').write(u.encode('gbk'))
print('VER 11 -> 12')
print('fw_tag 期望 = %d' % ((12 << 16) | (112 << 8) | 1 | 2 | 4))

for l in t.split('\n'):
    if 's_dth[i] +=' in l or 'f[i] -= s_x[IX_BA' in l:
        print('   ', l.strip())
