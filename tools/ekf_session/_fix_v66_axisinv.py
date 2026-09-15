# -*- coding: utf-8 -*-
"""VER=65 -> 66：磁力计水平两轴取反（PC 仿真已判定这是总根因）。

仿真证据（用旧链姿态反推世界磁场）：
  水平分量 (+0.0846, -0.3930) 与模型 (-0.0568, +0.4296) **反向**，模 0.436 vs 0.433；
  垂直分量 -0.9147 与模型 -0.9013 同号同量；方位角差 +175.4 度。
  => 磁力计 X/Y 两轴反了（绕机体 z 转 180 度）。

仿真验证（重力法向平面投影法，未改动数学）：
  未修磁轴:  末 err -45.55  p50 31.99 max 45.55   （= 实机卡住的样子）
  水平轴取反: 末 err  -5.24  p50  5.26 max  7.07   <- 收敛到 5 度以内
  （死区 12 度在修好后起互补作用；修好前残差恒 110 度，死区不起作用）

改法：M7 取到机体系磁场后，把水平两轴取反，再进入原投影观测。
"""
import re, shutil
U = r'C:\Users\33\Documents\v2\h415-imu42605-\V5F\User'
P = U + r'\src\proc_ekf.c'
T = U + r'\inc\v5f_tune.h'

t = open(P, 'rb').read().decode('gbk')
# M7 内的 mf 取值行（该文本全文出现 2 次，限定 M7）
anc = "    for (i = 0u; i < 3u; i++) mf[i] = h->mag.f[i];\n"
f0 = t.index('static void ekf_m7_mag')
i0 = t.index(anc, f0)
lim = t.index('static void ', f0 + 10)
assert t.count(anc, f0, lim) == 1, 'M7 内 mf 取值行 %d 次' % t.count(anc, f0, lim)
ins = anc + (
    "    /* \u2605VER=66 \u78c1\u529b\u8ba1\u6c34\u5e73\u4e24\u8f74\u53d6\u53cd\u3002\n"
    "     * \u5b9e\u6d4b\uff1a\u7531\u65e7\u94fe\u59ff\u6001\u53cd\u63a8\u7684\u4e16\u754c\u78c1\u573a\uff0c\u6c34\u5e73\u5206\u91cf\n"
    "     *   (+0.0846, -0.3930) \u4e0e\u6a21\u578b (-0.0568, +0.4296) **\u53cd\u5411**\uff08\u6a21 0.436 vs 0.433\uff09\uff0c\n"
    "     *   \u5782\u76f4\u5206\u91cf -0.9147 \u4e0e\u6a21\u578b -0.9013 \u540c\u53f7\u540c\u91cf\uff0c\u65b9\u4f4d\u89d2\u5dee 175.4 \u5ea6\n"
    "     *   -> X/Y \u4e24\u8f74\u53cd\u4e86\uff08\u7ed5\u673a\u4f53 z \u8f6c 180 \u5ea6\uff09\u3002\n"
    "     * PC \u4eff\u771f\uff1a\u53d6\u53cd\u540e\u540c\u4e00\u5957\u6295\u5f71\u89c2\u6d4b\u4ece -45.5 \u5ea6\u5361\u4f4f\u53d8\u4e3a\u6536\u655b\u5230 5 \u5ea6\u4ee5\u5185\u3002 */\n"
    "    mf[0] = -mf[0];\n"
    "    mf[1] = -mf[1];\n")
t = t[:i0] + ins + t[i0 + len(anc):]
assert t.count('{') == t.count('}')
assert re.search(r'st = ekf_update\(R, 2u, r, [^;]*0x0100u, V5F_EKF_MAG_K_MAX\);', t, re.S), 'update 调用被破坏'
shutil.copy2(P, P + '.bak_v66'); open(P, 'wb').write(t.encode('gbk'))

u = open(T, 'rb').read().decode('gbk')
assert u.count('#define V5F_FW_VER        65u') == 1
u = u.replace('#define V5F_FW_VER        65u', '#define V5F_FW_VER        66u', 1)
shutil.copy2(T, T + '.bak_v66'); open(T, 'wb').write(u.encode('gbk'))

c = open(P, 'rb').read().decode('gbk'); t2 = open(T, 'rb').read().decode('gbk')
i0 = c.index('static void ekf_m7_mag'); m7 = c[i0:c.index('\nstatic void ', i0 + 10)]
ver = int(re.search(r'#define V5F_FW_VER\s+(\d+)u', t2).group(1))
nch = int(re.search(r'#define JF_CH_NUM\s+(\d+)u', open(U + r'\src\SPI_rx.c', 'rb').read().decode('gbk')).group(1))
CK = [('VER==66', ver == 66),
      ('水平轴取反在', 'mf[0] = -mf[0];' in m7 and 'mf[1] = -mf[1];' in m7),
      ('取反在观测之前', m7.index('mf[0] = -mf[0];') < m7.index('r[0] = Bn[0] - b0x;')),
      ('死区 12 仍在', 'if (s_mag_r < V5F_EKF_MAG_DEAD_DEG) {' in m7),
      ('时间窗 snap 仍在', 'if (s_boot_t < V5F_EKF_MAG_ANCHOR_TS) {' in m7),
      ('二维水平观测仍在', 'r[0] = Bn[0] - b0x;' in m7),
      ('无符号取反', 'r[0] = -r[0]' not in m7),
      ('update 调用完整', bool(re.search(r'ekf_update\(R, 2u, r, [^;]*0x0100u, V5F_EKF_MAG_K_MAX\);', m7, re.S))),
      ('列数 139', nch == 139),
      ('{ } 平衡', c.count('{') == c.count('}')),
      ('括号差 -4', c.count('(') - c.count(')') == -4),
      ('/* */ 平衡', c.count('/*') == c.count('*/'))]
for k, v in CK:
    print('  %-20s %s' % (k, 'PASS' if v else 'FAIL')); assert v, k
print('\nfw_tag = %d  (VER=%d, %d ch)' % ((ver << 16) | (nch << 8) | 7, ver, nch))
