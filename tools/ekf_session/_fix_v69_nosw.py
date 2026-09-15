# -*- coding: utf-8 -*-
"""VER=68 -> 69：地磁观测关闭 χ² 软加权（自锁陷阱）。

数据（VER=68, fw_tag 4493319）：
  mag_r p50 71 度；mag_dqz p50 仅 0.117 度；p_yy 0.2965（rad^2 -> sigma_yaw 31 度）
  按 K = Pyy/(Pyy+R) 远超 MAG_K_MAX=0.05，单步应为 0.05*rz ~ 1.15 度，
  实测只有 0.117 度 —— 小 10 倍 => 软加权把 K 又压死了（nis 在大残差时爆掉，
  Si /= (nis/nis_max)）。这正是我早前仿真量到的自锁：nis 1.1->220, K 0.05->0.0005。

改：地磁那一次 ekf_update 的 nis_max 放到 1e9（软加权永不触发）。
    单步仍由 k_cap=MAG_K_MAX(0.05) 限幅；死区 12 度、开机时间窗精确角、轴修正 全部保留。
    其它观测（重力/气压/GPS/ZUPT）的软加权不动。
"""
import re, shutil
U = r'C:\Users\33\Documents\v2\h415-imu42605-\V5F\User'
P = U + r'\src\proc_ekf.c'
T = U + r'\inc\v5f_tune.h'

# 1) 把 M7 的 nis_max 换成专用的"不软加权"常量
t = open(P, 'rb').read().decode('gbk')
a = "ekf_update(R, 2u, r, V5F_EKF_NIS_MAX_MAG, &s_nis[4], &s_rej[3],"
assert t.count(a) == 1, 'M7 nis_max 锚点 %d' % t.count(a)
b = ("/* \u2605VER=69 \u5730\u78c1\u8fd9\u4e00\u8def\u5173\u95ed chi2 \u8f6f\u52a0\u6743\uff1a\n"
     "                         * \u5b9e\u6d4b mag_r=71 \u5ea6\u65f6\u5355\u6b65\u53ea\u7ed9 0.117 \u5ea6\uff0c\u800c K \u4e0a\u9650\n"
     "                         * \u672c\u5e94\u7ed9 0.05*rz ~ 1.15 \u5ea6 -> \u8f6f\u52a0\u6743\u628a K \u53c8\u538b\u6b7b\u4e86\u3002\n"
     "                         * \u5355\u6b65\u4ecd\u7531 k_cap(MAG_K_MAX=0.05) \u9650\u5e45\u3002 */\n"
     "                        ekf_update(R, 2u, r, V5F_EKF_NIS_MAX_MAG_NOSW, &s_nis[4], &s_rej[3],")
t = t.replace(a, b, 1)
assert t.count('{') == t.count('}')
assert re.search(r'ekf_update\(R, 2u, r, V5F_EKF_NIS_MAX_MAG_NOSW, [^;]*0x0100u, V5F_EKF_MAG_K_MAX\);', t, re.S), '调用不完整'
shutil.copy2(P, P + '.bak_v69'); open(P, 'wb').write(t.encode('gbk'))

# 2) tune 里加常量 + VER
u = open(T, 'rb').read().decode('gbk')
m = re.search(r'^[ \t]*#define[ \t]+V5F_EKF_NIS_MAX_MAG[ \t]+[^\r\n]*$', u, re.M)
assert m, '未找到 V5F_EKF_NIS_MAX_MAG'
u = u[:m.end()] + ('\n/* \u2605VER=69 \u5730\u78c1\u4e13\u7528\u7684 nis_max\uff1a\u7f6e\u6781\u5927 = \u5173\u95ed chi2 \u8f6f\u52a0\u6743\u3002\n'
                   ' * \u5b9e\u6d4b\u5927\u6b8b\u5dee\u65f6\u8f6f\u52a0\u6743\u4f1a\u628a K \u4ece 0.05 \u538b\u5230 0.0005\uff0c\u5f62\u6210\u81ea\u9501\uff1b\n'
                   ' * \u5355\u6b65\u53e6\u6709 MAG_K_MAX \u9650\u5e45\uff0c\u4e0d\u9700\u8981\u8f6f\u52a0\u6743\u3002 */\n'
                   '#define V5F_EKF_NIS_MAX_MAG_NOSW   1.0e9f') + u[m.end():]
assert u.count('#define V5F_FW_VER        68u') == 1
u = u.replace('#define V5F_FW_VER        68u', '#define V5F_FW_VER        69u', 1)
shutil.copy2(T, T + '.bak_v69'); open(T, 'wb').write(u.encode('gbk'))

c = open(P, 'rb').read().decode('gbk'); t2 = open(T, 'rb').read().decode('gbk')
i0 = c.index('static void ekf_m7_mag'); m7 = c[i0:c.index('\nstatic void ', i0 + 10)]
ver = int(re.search(r'#define V5F_FW_VER\s+(\d+)u', t2).group(1))
nch = int(re.search(r'#define JF_CH_NUM\s+(\d+)u', open(U + r'\src\SPI_rx.c', 'rb').read().decode('gbk')).group(1))
CK = [('VER==69', ver == 69),
      ('M7 用 NOSW 常量', 'V5F_EKF_NIS_MAX_MAG_NOSW' in m7),
      ('旧软加权常量不再用于 M7', 'r, V5F_EKF_NIS_MAX_MAG,' not in m7),
      ('tune 有 NOSW=1e9', 'V5F_EKF_NIS_MAX_MAG_NOSW   1.0e9f' in t2),
      ('轴修正仍在', 'mf[0] =  mf[1];' in m7),
      ('死区仍在', 'if (s_mag_r < V5F_EKF_MAG_DEAD_DEG) {' in m7),
      ('时间窗 snap 仍在', 's_boot_t < V5F_EKF_MAG_ANCHOR_TS' in m7),
      ('k_cap 仍在', '0x0100u, V5F_EKF_MAG_K_MAX' in m7),
      ('update 调用完整', bool(re.search(r'ekf_update\(R, 2u, r, V5F_EKF_NIS_MAX_MAG_NOSW, [^;]*0x0100u, V5F_EKF_MAG_K_MAX\);', m7, re.S))),
      ('其它观测软加权未动', c.count('V5F_EKF_NIS_MAX_3') == 3 and c.count('V5F_EKF_NIS_MAX_2') >= 1),
      ('列数 144', nch == 144),
      ('{ } 平衡', c.count('{') == c.count('}')),
      ('括号差 -4', c.count('(') - c.count(')') == -4),
      ('/* */ 平衡', c.count('/*') == c.count('*/'))]
for k, v in CK:
    print('  %-22s %s' % (k, 'PASS' if v else 'FAIL')); assert v, k
print('\nfw_tag = %d  (VER=%d, %d ch)' % ((ver << 16) | (nch << 8) | 7, ver, nch))
