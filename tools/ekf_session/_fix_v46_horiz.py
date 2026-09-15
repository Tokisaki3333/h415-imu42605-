# -*- coding: utf-8 -*-
"""VER=45 -> 46：M7 改成"以重力方向为法向的平面上的二维投影观测"。

原理（用户方案）：
  B0 = (cosI*sinD, cosI*cosD, -sinI)   当地磁场（导航系 ENU，常量）
       tanI = V5F_EKF_DIP_TAN(=2.08) -> I=64.3, cosI=0.4335, sinI=0.9012
       D    = V5F_MAG_DECL_RAD
  Bn = R(q) * f_meas                   实测磁场转到导航系
  v  = (Bn[0], Bn[1])                  投影到水平面（法向 = 重力方向）
  v0 = (B0[0], B0[1])                  预测的水平二维矢量
  r  = v - v0                          二维新息
    -> 矢量**方向**说明偏航应当在哪；**大小 |v|** 说明确定程度（=|B_h|，实测 0.44）
  雅可比（导航系扰动约定，与本 EKF 的 s_H[0][8]=-1 同源）：
    H[0][6..8] = ( 0,     B0_z, -B0_y )      dv_x = dth_y*B0_z - dth_z*B0_y
    H[1][6..8] = ( -B0_z, 0,     B0_x )      dv_y = dth_z*B0_x - dth_x*B0_z
  偏航列 = (-B0_y, B0_x)，模恰为 |B0_h| = cosI = 0.4335
  （交叉验证：由 DIP_TAN=2.08 推出的 0.4335 与实测 mag_bh 中位 0.4541 只差 5%）

为什么这样对：
  1) **全程没有除法**。旧的 hh = atan2(Bn_x,Bn_y) 里有除以 B_h，退化会被放大；
     逐分量比较不可能放大。
  2) 偏航灵敏度就是 |B0_h|（当地常数），不是机体系的 fhb。**VER=45 的 fhb 门
     是错的量**（fhb 与偏航可观测性无关），会误杀约 10% 的完好帧 -> 一并删除。
  3) 掩码 0x0100 -> 0x01C0：H 的倾角列非零，新息按几何分摊到三轴。
     "只注入偏航"才是把倾角误差硬塞给偏航的那条通路。

列数不变（仍 128），读取端无需改动。119 列 mag_r 改为"隐含偏航误差(度)"。
"""
import re
import shutil

R = r'C:\Users\33\Documents\v2\h415-imu42605-'
U = R + r'\V5F\User'
P = U + r'\src\proc_ekf.c'
T = U + r'\inc\v5f_tune.h'


def dump(path, text, enc, tag):
    data = text.encode(enc)
    shutil.copy2(path, path + '.bak_' + tag)
    with open(path, 'wb') as f:
        f.write(data)


t = open(P, 'rb').read().decode('gbk')
tn = open(T, 'rb').read().decode('gbk')
for mac in ('V5F_EKF_DIP_TAN', 'V5F_MAG_DECL_RAD', 'V5F_EKF_MAG_SIG_RAD',
            'V5F_EKF_NIS_MAX_2', 'V5F_EKF_MAG_R_MAX_DEG', 'V5F_EKF_MAG_K_MAX'):
    assert re.search(r'#define\s+' + mac + r'\b', tn), '缺少宏 ' + mac
print('  宏齐备 ok')

# ---- 1) 局部数组：R[1]->R[4], r[1]->r[2]，去掉 hh ----
a = "    float R[1], r[1], Rt[3][3], Bn[3], mf[3], hh, fhb2;"
assert t.count(a) == 1
t = t.replace(a, "    float R[4], r[2], Rt[3][3], Bn[3], mf[3], fhb2;", 1)
print('  ok 1-局部数组 R[4]/r[2]，去 hh')

# ---- 2) 删除 VER=45 的 fhb 硬门（保留 fhb 计算，仅作参考上报）----
a = t.index("    if (fhb2 < V5F_EKF_MAG_BHB_MIN")
b = t.index("    }\n", a) + len("    }\n")
blk = t[a:b]
assert 'return;' in blk and len(blk) < 400, 'fhb 门块异常'
t = t[:a] + t[b:]
print('  ok 2-删 fhb 硬门（%d 字节）' % len(blk))

# ---- 3) 替换观测 + 新息/更新整块 ----
i0 = t.index("    hh = atan2f(Bn[0], Bn[1]);")
i1 = t.index("    r[0] = wrap_pi(V5F_MAG_DECL_RAD - hh);", i0)
j1 = t.index("\n}\n", i1) + 1          # ekf_m7_mag 函数体结束
old = t[i0:j1]
assert 'bh2' in old and '0x0100u, V5F_EKF_MAG_K_MAX' in old and len(old) < 3000
new = """    /* \u2605VER=46 \u4e8c\u7ef4\u6c34\u5e73\u6295\u5f71\u89c2\u6d4b\uff08\u6cd5\u5411 = \u91cd\u529b\u65b9\u5411\uff09\u3002
     * v = (Bn[0],Bn[1]) \u662f\u5b9e\u6d4b\u78c1\u573a\u8f6c\u5230\u5bfc\u822a\u7cfb\u540e\u7684\u6c34\u5e73\u4e8c\u7ef4\u77e2\u91cf\uff0c
     * v0 = (B0x,B0y) \u662f\u5f53\u5730\u78c1\u573a\u7684\u6c34\u5e73\u4e8c\u7ef4\u77e2\u91cf\uff08\u5e38\u91cf\uff09\u3002
     *   \u65b9\u5411 -> \u504f\u822a\u5e94\u5f53\u5728\u54ea\uff1b\u5927\u5c0f |v| -> \u786e\u5b9a\u7a0b\u5ea6\u3002
     * \u65e0\u4efb\u4f55\u9664\u6cd5 -> \u4e0d\u53ef\u80fd\u51fa\u73b0\u9000\u5316\u653e\u5927\uff1b
     * \u504f\u822a\u7075\u654f\u5ea6\u5c31\u662f |B0_h| = cos(\u78c1\u503e\u89d2)\uff0c\u4e0e\u673a\u4f53\u59ff\u6001\u65e0\u5173\u3002 */
    q_to_R(&s_x[IX_Q], Rt);
    rot_bn(Rt, mf, Bn);                    /* Bn = R(q) f\uff1a\u5bfc\u822a\u7cfb\u78c1\u573a */
    {
        float ci = 1.0f / sqrtf(1.0f + V5F_EKF_DIP_TAN * V5F_EKF_DIP_TAN);
        float si = V5F_EKF_DIP_TAN * ci;
        float sd = sinf(V5F_MAG_DECL_RAD), cd = cosf(V5F_MAG_DECL_RAD);
        float b0x = ci * sd, b0y = ci * cd, b0z = -si;
        float sig2 = V5F_EKF_MAG_SIG_RAD * V5F_EKF_MAG_SIG_RAD;
        float rn;
        s_mag_bh = sqrtf(Bn[0]*Bn[0] + Bn[1]*Bn[1]);   /* = |v|\uff0c\u786e\u5b9a\u7a0b\u5ea6 */
        H_zero(2u);
        s_H[0][6] = 0.0f;  s_H[0][7] =  b0z;  s_H[0][8] = -b0y;
        s_H[1][6] = -b0z;  s_H[1][7] = 0.0f;  s_H[1][8] =  b0x;
        r[0] = Bn[0] - b0x;
        r[1] = Bn[1] - b0y;
        R[0] = sig2; R[1] = 0.0f; R[2] = 0.0f; R[3] = sig2;
        /* \u4ec5\u8bca\u65ad\uff1a|r|/|v| \u5373\u9690\u542b\u7684\u504f\u822a\u8bef\u5dee\uff08\u5f27\u5ea6 -> \u5ea6\uff09*/
        rn = sqrtf(r[0]*r[0] + r[1]*r[1]);
        s_mag_r = (s_mag_bh > 1e-3f) ? (rn / s_mag_bh) * RAD2DEG : 0.0f;
        if (s_mag_r > V5F_EKF_MAG_R_MAX_DEG) {
            if (s_rej[3] < 250u) s_rej[3]++;
            s_gate_bits |= V5F_EKF_GB_CHI2;
        } else {
            st = ekf_update(R, 2u, r, V5F_EKF_NIS_MAX_2, &s_nis[4], &s_rej[3],
                            0x01C0u, V5F_EKF_MAG_K_MAX);
            if (st == 0u) { s_gate_bits |= V5F_EKF_GB_MAG; s_mag_used = 1u; }
        }
    }
}
"""
assert new.count('{') == new.count('}')
t = t[:i0] + new + t[j1:]
print('  ok 3-二维水平投影观测（%d -> %d 字节）' % (len(old), len(new)))

dump(P, t, 'gbk', 'v46horiz')

u = open(T, 'rb').read().decode('gbk')
assert u.count('#define V5F_FW_VER        45u') == 1
u = u.replace('#define V5F_FW_VER        45u', '#define V5F_FW_VER        46u', 1)
dump(T, u, 'gbk', 'v46horiz')
print('  ok 4-VER=46')

c = open(P, 'rb').read().decode('gbk')
t2 = open(T, 'rb').read().decode('gbk')
ver = int(re.search(r'#define V5F_FW_VER\s+(\d+)u', t2).group(1))
nch = int(re.search(r'#define JF_CH_NUM\s+(\d+)u',
                    open(U + r'\src\SPI_rx.c', 'rb').read().decode('gbk')).group(1))
CK = [('VER==46', ver == 46),
      ('无 atan2 方位角', 'atan2f(Bn[0], Bn[1])' not in c),
      ('无 bh2 除法', 'Bn[0]*Bn[2]/bh2' not in c),
      ('无 fhb 硬门', 'if (fhb2 < V5F_EKF_MAG_BHB_MIN' not in c),
      ('无 fhb 加权', 'V5F_EKF_MAG_FERR / sqrtf(fhb2)' not in c),
      ('二维新息在', 'r[0] = Bn[0] - b0x;' in c and 'r[1] = Bn[1] - b0y;' in c),
      ('H yaw 列 = (-b0y,b0x)', 's_H[0][8] = -b0y;' in c and 's_H[1][8] =  b0x;' in c),
      ('掩码 0x01C0', '0x01C0u, V5F_EKF_MAG_K_MAX' in c),
      ('m=2', 'ekf_update(R, 2u, r, V5F_EKF_NIS_MAX_2, &s_nis[4]' in c),
      ('R[4]/r[2]', 'float R[4], r[2], Rt[3][3], Bn[3], mf[3], fhb2;' in c),
      ('prop 自锁修复仍在', 'while (s_prop_row < EKF_N) ekf_prop_row();' in c),
      ('列数仍 128', nch == 128),
      ('{ } 平衡', c.count('{') == c.count('}')),
      ('/* */ 平衡', c.count('/*') == c.count('*/'))]
for k, v in CK:
    print('  %-24s %s' % (k, 'PASS' if v else 'FAIL'))
    assert v, k
print('\nfw_tag = %d  (VER=46, %d ch)' % ((ver << 16) | (nch << 8) | 7, nch))
