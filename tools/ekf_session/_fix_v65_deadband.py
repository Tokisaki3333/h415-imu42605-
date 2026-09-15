# -*- coding: utf-8 -*-
"""VER=64 -> 65：加"隐含偏航误差"死区，阈值大于磁角典型误差 10 度。

理由（用户）：磁角典型误差可达 10 度，死区必须**大于**它，
否则环路会在误差范围内反复修正 —— 等于把测量自身的误差当误差去追。
取 V5F_EKF_MAG_DEAD_DEG = 12.0 度（> 10 度）。
"""
import re, shutil
U = r'C:\Users\33\Documents\v2\h415-imu42605-\V5F\User'
P = U + r'\src\proc_ekf.c'
T = U + r'\inc\v5f_tune.h'

t = open(P, 'rb').read().decode('gbk')
a = "        if (s_mag_r > V5F_EKF_MAG_R_MAX_DEG) {"
assert t.count(a) == 1, 'R_MAX 门锚点 %d' % t.count(a)
b = ("        /* \u2605VER=65 \u4f4d\u79fb\u6b7b\u533a\uff1a\u78c1\u89d2\u5178\u578b\u8bef\u5dee\u53ef\u8fbe 10 \u5ea6\uff0c\n"
     "         * \u6b7b\u533a\u5fc5\u987b**\u5927\u4e8e**\u5b83\uff0c\u5426\u5219\u73af\u8def\u5728\u8bef\u5dee\u8303\u56f4\u5185\u53cd\u590d\u4fee\u6b63\uff0c\n"
     "         * \u7b49\u4e8e\u628a\u6d4b\u91cf\u81ea\u8eab\u7684\u8bef\u5dee\u5f53\u6210\u8bef\u5dee\u53bb\u8ffd\u3002\n"
     "         * |\u9690\u542b\u504f\u822a\u8bef\u5dee| < MAG_DEAD_DEG(12 \u5ea6) -> \u672c\u5468\u671f\u4e0d\u4fee\u6b63\u3002 */\n"
     "        if (s_mag_r < V5F_EKF_MAG_DEAD_DEG) {\n"
     "            return;                     /* \u6b63\u5e38\u6b7b\u533a\uff0c\u4e0d\u7b97\u62d2\u7edd\uff0c\u4e0d\u7f6e\u4f4d */\n"
     "        }\n"
     + a)
t = t.replace(a, b, 1)
assert t.count('{') == t.count('}')
assert re.search(r'st = ekf_update\(R, 2u, r, [^;]*0x0100u, V5F_EKF_MAG_K_MAX\);', t, re.S), 'update 调用被破坏'
shutil.copy2(P, P + '.bak_v65'); open(P, 'wb').write(t.encode('gbk'))

u = open(T, 'rb').read().decode('gbk')
m = re.search(r'^[ \t]*#define[ \t]+V5F_EKF_MAG_BH_MIN[^\r\n]*$', u, re.M)
assert m
u = u[:m.end()] + ('\n/* \u2605VER=65 \u5730\u78c1\u504f\u822a\u8bef\u5dee\u7684\u6b7b\u533a\uff08\u5ea6\uff09\u3002\n'
                   ' * \u78c1\u89d2\u5178\u578b\u8bef\u5dee\u53ef\u8fbe 10 \u5ea6 -> \u6b7b\u533a\u53d6 12 \u5ea6\uff0c**\u5fc5\u987b\u5927\u4e8e\u8bef\u5dee\u8303\u56f4**\u3002 */\n'
                   '#define V5F_EKF_MAG_DEAD_DEG      12.0f') + u[m.end():]
assert u.count('#define V5F_FW_VER        64u') == 1
u = u.replace('#define V5F_FW_VER        64u', '#define V5F_FW_VER        65u', 1)
shutil.copy2(T, T + '.bak_v65'); open(T, 'wb').write(u.encode('gbk'))

c = open(P, 'rb').read().decode('gbk'); t2 = open(T, 'rb').read().decode('gbk')
i0 = c.index('static void ekf_m7_mag'); m7 = c[i0:c.index('\nstatic void ', i0 + 10)]
ver = int(re.search(r'#define V5F_FW_VER\s+(\d+)u', t2).group(1))
nch = int(re.search(r'#define JF_CH_NUM\s+(\d+)u', open(U + r'\src\SPI_rx.c', 'rb').read().decode('gbk')).group(1))
CK = [('VER==65', ver == 65),
      ('死区在', 'if (s_mag_r < V5F_EKF_MAG_DEAD_DEG) {' in m7),
      ('死区在 R_MAX 门之前', m7.index('s_mag_r < V5F_EKF_MAG_DEAD_DEG') < m7.index('s_mag_r > V5F_EKF_MAG_R_MAX_DEG')),
      ('tune DEAD=12', 'V5F_EKF_MAG_DEAD_DEG      12.0f' in t2),
      ('时间窗 snap 在', 'if (s_boot_t < V5F_EKF_MAG_ANCHOR_TS) {' in m7),
      ('二维水平观测在', 'r[0] = Bn[0] - b0x;' in m7),
      ('无符号取反', 'r[0] = -r[0]' not in m7),
      ('update 调用完整', bool(re.search(r'ekf_update\(R, 2u, r, [^;]*0x0100u, V5F_EKF_MAG_K_MAX\);', m7, re.S))),
      ('列数 139', nch == 139),
      ('{ } 平衡', c.count('{') == c.count('}')),
      ('括号差 -4', c.count('(') - c.count(')') == -4),
      ('/* */ 平衡', c.count('/*') == c.count('*/'))]
for k, v in CK:
    print('  %-22s %s' % (k, 'PASS' if v else 'FAIL')); assert v, k
print('\nfw_tag = %d  (VER=%d, %d ch)' % ((ver << 16) | (nch << 8) | 7, ver, nch))
