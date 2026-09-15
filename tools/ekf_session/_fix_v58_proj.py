# -*- coding: utf-8 -*-
"""VER=57 -> 58：慢环用"只含角度"的投影新息，snap 仍用完整新息。

证据（VER=56 数据）：
  mag_dq 是唯一驱动者（均值 0.4422 度/次 = 154 度/秒），tilt_dq 仅 0.01（M6 无罪）；
  且 VER=55（原符号）与 VER=56（翻转）**两个符号都跑飞** -> 不是符号问题，
  而是观测在追一个自相矛盾的目标：|v| = 0.29 而模型 |v0| = 0.4335（差 33%）。
  偏航转不掉这个模值差，但它会经 K 的偏航列持续被吃进去。

=> 分工：
   慢环 ekf_update 只吃**投影后**的新息（沿 v0 的分量去掉 => 纯角度信息，与模值无关）
   snap（大误差一步式）仍用**完整** r + 精确角 atan2(cross,dot)（破 180 度鞍点）
   诊断 mag_rx/ry 仍报完整 r（口径不变）
"""
import re, shutil
U = r'C:\Users\33\Documents\v2\h415-imu42605-\V5F\User'
P = U + r'\src\proc_ekf.c'
T = U + r'\inc\v5f_tune.h'
def dump(p, s, e, tag):
    d = s.encode(e); shutil.copy2(p, p + '.bak_' + tag)
    open(p, 'wb').write(d)

t = open(P, 'rb').read().decode('gbk')
a = "            st = ekf_update(R, 2u, r, V5F_EKF_NIS_MAX_2, &s_nis[4], &s_rej[3],"
assert t.count(a) == 1, 'update 锚点 %d' % t.count(a)
b = ("            /* \u2605VER=58 \u6162\u73af\u53ea\u5403\u6295\u5f71\u540e\u7684\u65b0\u606f\uff08\u7eaf\u89d2\u5ea6\uff0c\u4e0e\u6a21\u503c\u65e0\u5173\uff09\uff1b\n"
     "             * snap \u4ecd\u7528\u5b8c\u6574 r + \u7cbe\u786e\u89d2\uff08\u783415 180 \u5ea6\u978d\u70b9\uff09\u3002\n"
     "             * \u5b9e\u6d4b |v|=0.29 \u800c\u6a21\u578b |v0|=0.4335\uff08\u5dee 33%\uff09\uff0c\u8fd9\u4e2a\u6a21\u503c\u5dee\n"
     "             * \u504f\u822a\u8f6c\u4e0d\u6389\uff0c\u7559\u5728\u73af\u91cc\u5c31\u4f1a\u88ab K \u7684\u504f\u822a\u5217\u6301\u7eed\u5403\u8fdb\u53bb -> \u8dd1\u98de\u3002 */\n"
     "            {\n"
     "                float rp[2], n2p = b0x * b0x + b0y * b0y, kk2;\n"
     "                kk2 = (n2p > 1e-6f) ? (r[0] * b0x + r[1] * b0y) / n2p : 0.0f;\n"
     "                rp[0] = r[0] - kk2 * b0x;\n"
     "                rp[1] = r[1] - kk2 * b0y;\n"
     + a.replace("2u, r,", "2u, rp,").replace("            st", "                st")
     + "\n            }")
t = t.replace(a, b, 1)
assert t.count('{') == t.count('}'), '括号不平'
dump(P, t, 'gbk', 'v58')
u = open(T, 'rb').read().decode('gbk')
assert u.count('#define V5F_FW_VER        57u') == 1
u = u.replace('#define V5F_FW_VER        57u', '#define V5F_FW_VER        58u', 1)
dump(T, u, 'gbk', 'v58')
c = open(P, 'rb').read().decode('gbk'); t2 = open(T, 'rb').read().decode('gbk')
i0 = c.index('static void ekf_m7_mag'); m7 = c[i0:c.index('\nstatic void ', i0+10)]
ver = int(re.search(r'#define V5F_FW_VER\s+(\d+)u', t2).group(1))
nch = int(re.search(r'#define JF_CH_NUM\s+(\d+)u', open(U + r'\src\SPI_rx.c','rb').read().decode('gbk')).group(1))
CK = [('VER==58', ver == 58),
      ('慢环吃投影', 'ekf_update(R, 2u, rp,' in m7),
      ('snap 仍完整 r', 'dpsi = atan2f(crs, dt2);' in m7),
      ('大误差触发在', 'fabsf(r[0]) + fabsf(r[1]) > 0.5f' in m7),
      ('自校验仍在', 'if (e1 > e0) {' in m7),
      ('无符号取反', 'r[0] = -r[0]' not in m7),
      ('掩码仍 0x0100', '0x0100u, V5F_EKF_MAG_K_MAX' in m7),
      ('列数仍 139', nch == 139),
      ('{ } 平衡', c.count('{') == c.count('}')),
      ('/* */ 平衡', c.count('/*') == c.count('*/'))]
for k, v in CK:
    print('  %-18s %s' % (k, 'PASS' if v else 'FAIL')); assert v, k
print('\nfw_tag = %d  (VER=%d, %d ch)' % ((ver << 16) | (nch << 8) | 7, ver, nch))
