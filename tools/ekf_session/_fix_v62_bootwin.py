# -*- coding: utf-8 -*-
"""VER=61 -> 62：改为**开机时间窗**触发整角对齐。

用户指示：基于开机时间触发。
  - 不依赖 s_mag_anchor（对齐块会复位它，与 M7 的先后次序会把"启动那一次"吃掉）；
  - 不依赖新息大小（宽触发 = 每周期对打 = 闪现）。
  开机后固定窗口内，每个有效地磁周期都做一次精确角整角修正 -> 窗内必收敛；
  出窗后完全交给正常慢环（二维水平投影观测）。
"""
import re, shutil
U = r'C:\Users\33\Documents\v2\h415-imu42605-\V5F\User'
P = U + r'\src\proc_ekf.c'
T = U + r'\inc\v5f_tune.h'

t = open(P, 'rb').read().decode('gbk')
n = 0

# 1) 新增开机计时静态量
m = re.search(r'^(static uint8_t\s+s_mag_anchor;.*)$', t, re.M)
assert m, '未找到 s_mag_anchor 声明'
t = t[:m.end()] + ("\nstatic float    s_boot_t;       /* \u2605VER=62 \u5f00\u673a\u8ba1\u65f6(s)\uff0c\u7528\u4e8e\u6574\u89d2\u5bf9\u9f50\u7684\u65f6\u95f4\u7a97 */") + t[m.end():]
n += 1; print('  ok 1-s_boot_t 声明')

# 2) 累加 dt
a = "    s_baro_wait += dt;"
assert t.count(a) == 1, 's_baro_wait 锚点 %d' % t.count(a)
t = t.replace(a, a + "\n    s_boot_t += dt;      /* \u2605VER=62 \u5f00\u673a\u8ba1\u65f6 */", 1)
n += 1; print('  ok 2-s_boot_t 累加')

# 3) 触发条件改为时间窗
a = "        if (!s_mag_anchor || (fabsf(r[0]) + fabsf(r[1]) > 0.8f)) {"
assert t.count(a) == 1, 'snap 条件锚点 %d' % t.count(a)
b = ("        /* \u2605VER=62 \u6309\u5f00\u673a\u65f6\u95f4\u89e6\u53d1\uff1a\u5f00\u673a\u540e V5F_EKF_MAG_ANCHOR_TS \u79d2\u5185\uff0c\n"
     "         * \u6bcf\u4e2a\u6709\u6548\u5730\u78c1\u5468\u671f\u90fd\u505a\u4e00\u6b21\u7cbe\u786e\u89d2\u6574\u89d2\u4fee\u6b63\uff08\u7a97\u5185\u5fc5\u6536\u655b\uff09\uff1b\n"
     "         * \u51fa\u7a97\u540e\u5b8c\u5168\u4ea4\u7ed9\u6b63\u5e38\u6162\u73af\u3002\u4e0d\u518d\u4f9d\u8d56 s_mag_anchor\uff0c\u4e5f\u4e0d\u4f9d\u8d56\u65b0\u606f\u5927\u5c0f\u3002 */\n"
     "        if (s_boot_t < V5F_EKF_MAG_ANCHOR_TS) {")
t = t.replace(a, b, 1)
n += 1; print('  ok 3-时间窗触发')

assert t.count('{') == t.count('}'), '括号不平'
assert re.search(r'st = ekf_update\(R, 2u, r, [^;]*0x0100u, V5F_EKF_MAG_K_MAX\);', t, re.S), 'update 调用被破坏'
shutil.copy2(P, P + '.bak_v62'); open(P, 'wb').write(t.encode('gbk'))

# 4) tune：加窗口时长 + VER
u = open(T, 'rb').read().decode('gbk')
m = re.search(r'^[ \t]*#define[ \t]+V5F_EKF_MAG_BH_MIN[^\r\n]*$', u, re.M)
assert m, '未找到 V5F_EKF_MAG_BH_MIN'
u = u[:m.end()] + ('\n/* \u2605VER=62 \u6574\u89d2\u5bf9\u9f50\u7684\u5f00\u673a\u65f6\u95f4\u7a97\uff08\u79d2\uff09\uff1a\n'
                   ' * \u5f00\u673a\u540e\u8be5\u7a97\u53e3\u5185\u6bcf\u4e2a\u6709\u6548\u5730\u78c1\u5468\u671f\u90fd\u7528\u7cbe\u786e\u89d2\u4e00\u6b65\u8f6c\u5230\u4f4d\uff0c\n'
                   ' * \u7528\u4e8e\u7834\u6389 180 \u5ea6\u5047\u70b9\uff1b\u51fa\u7a97\u540e\u4ea4\u7ed9\u6162\u73af\u3002\u53d6 3.0 s \u7ed9\u521d\u59cb\u52a8\u4f5c\u7559\u4f59\u91cf\u3002 */\n'
                   '#define V5F_EKF_MAG_ANCHOR_TS     3.0f') + u[m.end():]
assert u.count('#define V5F_FW_VER        61u') == 1
u = u.replace('#define V5F_FW_VER        61u', '#define V5F_FW_VER        62u', 1)
shutil.copy2(T, T + '.bak_v62'); open(T, 'wb').write(u.encode('gbk'))
n += 1; print('  ok 4-tune +ANCHOR_TS, VER=62')

c = open(P, 'rb').read().decode('gbk'); t2 = open(T, 'rb').read().decode('gbk')
i0 = c.index('static void ekf_m7_mag'); m7 = c[i0:c.index('\nstatic void ', i0 + 10)]
ver = int(re.search(r'#define V5F_FW_VER\s+(\d+)u', t2).group(1))
nch = int(re.search(r'#define JF_CH_NUM\s+(\d+)u', open(U + r'\src\SPI_rx.c', 'rb').read().decode('gbk')).group(1))
CK = [('VER==62', ver == 62), ('编辑数==4', n == 4),
      ('时间窗触发在', 'if (s_boot_t < V5F_EKF_MAG_ANCHOR_TS) {' in m7),
      ('旧条件已去', 'fabsf(r[0]) + fabsf(r[1]) > 0.8f' not in m7),
      ('精确角在', 'dpsi = atan2f(crs, dt2);' in m7),
      ('自校验在', 'if (e1 > e0) {' in m7),
      ('二维水平观测在', 'r[0] = Bn[0] - b0x;' in m7),
      ('s_boot_t 累加在', 's_boot_t += dt;' in c),
      ('update 调用完整', bool(re.search(r'ekf_update\(R, 2u, r, [^;]*0x0100u, V5F_EKF_MAG_K_MAX\);', m7, re.S))),
      ('无 rp/取反/v0投影', 'rp[' not in m7 and 'r[0] = -r[0]' not in m7 and 'kk2' not in m7),
      ('tune 有 ANCHOR_TS', 'V5F_EKF_MAG_ANCHOR_TS     3.0f' in t2),
      ('列数 139', nch == 139),
      ('{ } 平衡', c.count('{') == c.count('}')),
      ('括号差 -4', c.count('(') - c.count(')') == -4),
      ('/* */ 平衡', c.count('/*') == c.count('*/'))]
for k, v in CK:
    print('  %-20s %s' % (k, 'PASS' if v else 'FAIL')); assert v, k
print('\nfw_tag = %d  (VER=%d, %d ch)' % ((ver << 16) | (nch << 8) | 7, ver, nch))
