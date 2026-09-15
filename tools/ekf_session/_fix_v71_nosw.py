# -*- coding: utf-8 -*-
"""VER=70 -> 71：给地磁那一次 ekf_update 关闭 chi2 软加权（用专用 nis_max）。

数据（VER=70, fw_tag 4624391）：
  mag_r p50 106 度，只有 22.6% 落在 12 度死区内；
  mag_dqz p50 仅 0.0685 度 —— 而 K 上限 0.05 时应给 0.05*rz ≈ 1.23 度，小 18 倍
  => 软加权把 K 压死，环路拉不动 106 度残差，于是被扰走就回不来，
     偶尔猛修一次（实测有 ±150~166 度/0.25s 的修正段）= 用户说的"偶尔大漂一下"。
注意：V5F_EKF_NIS_MAX_2 同时被 M1(GPS位置)/M4(速度) 使用，**不能改宏**，
      只把 M7 这次调用换成专用常量 V5F_EKF_NIS_MAX_MAG_NOSW。
"""
import re, shutil
U = r'C:\Users\33\Documents\v2\h415-imu42605-\V5F\User'
P = U + r'\src\proc_ekf.c'
T = U + r'\inc\v5f_tune.h'

t = open(P, 'rb').read().decode('gbk')
pat = re.compile(r'ekf_update\(R, 2u, r, (V5F_EKF_NIS_MAX_2), &s_nis\[4\], &s_rej\[3\],(\s*\n\s*)0x0100u, V5F_EKF_MAG_K_MAX\);')
ms = list(pat.finditer(t))
assert len(ms) == 1, 'M7 调用匹配 %d 次' % len(ms)
m = ms[0]
old = m.group(0)
new = old.replace('&s_nis[4], &s_rej[3],', '&s_nis[4], &s_rej[3],', 1)
new = new.replace('V5F_EKF_NIS_MAX_2', 'V5F_EKF_NIS_MAX_MAG_NOSW', 1)
new = ("/* \u2605VER=71 \u5730\u78c1\u5173\u95ed chi2 \u8f6f\u52a0\u6743\uff1a\u5b9e\u6d4b mag_r=106 \u5ea6\u65f6\u5355\u6b65\u53ea\u7ed9 0.0685 \u5ea6\uff0c\n"
       "                        * \u800c K \u4e0a\u9650 0.05 \u672c\u5e94\u7ed9 ~1.23 \u5ea6\uff0c\u5c0f 18 \u500d -> \u62c9\u4e0d\u56de\u6765\u3002\n"
       "                        * \u5355\u6b65\u4ecd\u7531 k_cap(MAG_K_MAX) \u9650\u5e45\u3002 */\n"
       "                        " + new)
t = t[:m.start()] + new + t[m.end():]
assert t.count('{') == t.count('}')
assert re.search(r'ekf_update\(R, 2u, r, V5F_EKF_NIS_MAX_MAG_NOSW,[^;]*0x0100u, V5F_EKF_MAG_K_MAX\);', t, re.S)
shutil.copy2(P, P + '.bak_v71'); open(P, 'wb').write(t.encode('gbk'))

u = open(T, 'rb').read().decode('gbk')
if 'V5F_EKF_NIS_MAX_MAG_NOSW' not in u:
    m2 = re.search(r'^[ \t]*#define[ \t]+V5F_EKF_NIS_MAX_MAG[ \t]+[^\r\n]*$', u, re.M)
    anchor = m2.end() if m2 else None
    if anchor is None:
        m2 = re.search(r'^[ \t]*#define[ \t]+V5F_EKF_MAG_K_MAX[ \t]+[^\r\n]*$', u, re.M)
        anchor = m2.end()
    u = u[:anchor] + ('\n/* \u2605VER=71 \u5730\u78c1\u4e13\u7528 nis_max\uff1a\u7f6e\u6781\u5927 = \u5173\u95ed chi2 \u8f6f\u52a0\u6743\u3002\n'
                      ' * \u5b9e\u6d4b\u5927\u6b8b\u5dee\u65f6\u8f6f\u52a0\u6743\u628a K \u4ece 0.05 \u538b\u5230 ~0.003\uff0c\u5f62\u6210\u81ea\u9501\uff1b\n'
                      ' * \u5355\u6b65\u53e6\u6709 MAG_K_MAX \u9650\u5e45\uff0c\u4e0d\u9700\u8981\u8f6f\u52a0\u6743\u3002\n'
                      ' * \u6ce8\uff1a\u4e0d\u80fd\u76f4\u63a5\u6539 V5F_EKF_NIS_MAX_2\uff0c\u5b83\u540c\u65f6\u88ab M1/M4 \u7528\u3002 */\n'
                      '#define V5F_EKF_NIS_MAX_MAG_NOSW   1.0e9f') + u[anchor:]
assert u.count('#define V5F_FW_VER        70u') == 1
u = u.replace('#define V5F_FW_VER        70u', '#define V5F_FW_VER        71u', 1)
shutil.copy2(T, T + '.bak_v71'); open(T, 'wb').write(u.encode('gbk'))

c = open(P, 'rb').read().decode('gbk'); t2 = open(T, 'rb').read().decode('gbk')
i0 = c.index('static void ekf_m7_mag'); m7 = c[i0:c.index('\nstatic void ', i0 + 10)]
ver = int(re.search(r'#define V5F_FW_VER\s+(\d+)u', t2).group(1))
nch = int(re.search(r'#define JF_CH_NUM\s+(\d+)u', open(U + r'\src\SPI_rx.c', 'rb').read().decode('gbk')).group(1))
CK = [('VER==71', ver == 71),
      ('M7 用 NOSW', 'V5F_EKF_NIS_MAX_MAG_NOSW' in m7),
      ('M7 不再用 _2', 'r, V5F_EKF_NIS_MAX_2,' not in m7),
      ('tune 定义 NOSW', 'V5F_EKF_NIS_MAX_MAG_NOSW   1.0e9f' in t2),
      ('_NIS_MAX_2 宏本身未动', re.search(r'#define\s+V5F_EKF_NIS_MAX_2\s+\S+', t2) is not None),
      ('M1/M4 仍用 _2', c.count('V5F_EKF_NIS_MAX_2') >= 2),
      ('轴变换仍无', 'mf[0] =  mf[1];' not in m7 and 'mf[0] = -mf[0];' not in m7),
      ('死区在', 's_mag_r < V5F_EKF_MAG_DEAD_DEG' in m7),
      ('时间窗 snap 在', 's_boot_t < V5F_EKF_MAG_ANCHOR_TS' in m7),
      ('update 完整', bool(re.search(r'ekf_update\(R, 2u, r, V5F_EKF_NIS_MAX_MAG_NOSW,[^;]*0x0100u, V5F_EKF_MAG_K_MAX\);', m7, re.S))),
      ('列数 144', nch == 144),
      ('{ } 平衡', c.count('{') == c.count('}')),
      ('括号差 -4', c.count('(') - c.count(')') == -4),
      ('/* */ 平衡', c.count('/*') == c.count('*/'))]
for k, v in CK:
    print('  %-20s %s' % (k, 'PASS' if v else 'FAIL')); assert v, k
print('\nfw_tag = %d  (VER=%d, %d ch)' % ((ver << 16) | (nch << 8) | 7, ver, nch))
