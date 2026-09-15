# -*- coding: utf-8 -*-
"""VER=49 -> 50：只加上报，点出"谁在绕错轴"。

问题（用户实测）：运动全程角速度 < 30 度/秒，但有时"两个轴以约 90 度/秒互换"。
VER=49 数据显示：地磁已被限制为只动偏航（|dq| x/y/z = 0/0/100%），
而静止段 EKF roll 仍变化 9160 度 -> **凶手不是地磁**。
能动全姿态的只剩两条：M6 重力/倾角观测（掩码 0x7EC0，含姿态三轴+ba+bg，门开 99%）
和传播。陀螺 <30 dps、bg 有 ±10 dps 钳位，传播造不出 90 度/秒。
=> 嫌疑集中在 M6：H 的行列/约定一错，本该绕横滚的修正就落到俯仰上 —— 正是"两轴互换"，
   且门常开会**持续**这么做，速率稳定成一档。

本次新增 6 列（不做任何修正）：
  133/134/135  tilt_dqx/dqy/dqz  M6 本次**实际注入**到姿态三轴的角度(度)
  136/137/138  tilt_prx/pry/prz  M6 预测的机体系重力方向（静止应≈(0,0,1)且不随姿态变）
判据：两轴互换发生时，看 tilt_dq* 落在哪一轴、tilt_pr* 是否异常；
      并与 mag_dq*（130~132）对照，区分是地磁还是重力在动姿态。
"""
import re
import shutil

R = r'C:\Users\33\Documents\v2\h415-imu42605-'
U = R + r'\V5F\User'
P = U + r'\src\proc_ekf.c'
H = U + r'\inc\SPI_rx.h'
S = U + r'\src\SPI_rx.c'
T = U + r'\inc\v5f_tune.h'


def dump(path, text, enc, tag):
    data = text.encode(enc)
    shutil.copy2(path, path + '.bak_' + tag)
    with open(path, 'wb') as f:
        f.write(data)


NEW = [('tilt_dqx', 'float', 's_tilt_dqx'), ('tilt_dqy', 'float', 's_tilt_dqy'),
       ('tilt_dqz', 'float', 's_tilt_dqz'), ('tilt_prx', 'float', 's_tilt_prx'),
       ('tilt_pry', 'float', 's_tilt_pry'), ('tilt_prz', 'float', 's_tilt_prz')]
n = 0
t = open(P, 'rb').read().decode('gbk')

# 1) 静态量
a = "static float    s_mag_dqx, s_mag_dqy, s_mag_dqz;"
assert t.count(a) == 1, '静态量锚点 %d' % t.count(a)
t = t.replace(a, a + "\n"
              "static float    s_tilt_dqx, s_tilt_dqy, s_tilt_dqz;  /* \u2605VER=50 M6 \u5b9e\u9645\u6ce8\u5165\u7684\u59ff\u6001\u4fee\u6b63(\u5ea6) */\n"
              "static float    s_tilt_prx, s_tilt_pry, s_tilt_prz;  /* \u2605VER=50 M6 \u9884\u6d4b\u91cd\u529b\u65b9\u5411(\u673a\u4f53\u7cfb) */", 1)
n += 1; print('  ok 1-新增 6 个静态量')

# 2) M6 内采集：锚在它自己的结束标志上
a = "    if (st == 0u) s_gate_bits |= V5F_EKF_GB_TILT;"
assert t.count(a) == 1, 'M6 结束标志锚点 %d' % t.count(a)
b = ("    s_tilt_prx = pr[0]; s_tilt_pry = pr[1]; s_tilt_prz = pr[2];   /* \u2605VER=50 */\n"
     "    if (st == 0u) {\n"
     "        s_gate_bits |= V5F_EKF_GB_TILT;\n"
     "        s_tilt_dqx = s_dx[IX_Q + 0] * RAD2DEG;\n"
     "        s_tilt_dqy = s_dx[IX_Q + 1] * RAD2DEG;\n"
     "        s_tilt_dqz = s_dx[IX_Q + 2] * RAD2DEG;\n"
     "    }")
t = t.replace(a, b, 1)
n += 1; print('  ok 2-M6 内采集 6 个量')

# 3) publish
m = re.search(r'^([ \t]*)h->ekf\.mag_dqz[^\n]*$', t, re.M)
assert m, '未找到 publish mag_dqz 行'
t = t[:m.end()] + ''.join('\n%s h->ekf.%s = %s;' % (m.group(1), f, src) for f, _, src in NEW) + t[m.end():]
n += 1; print('  ok 3-publish +6')
dump(P, t, 'gbk', 'v50')

# 4) 结构体
h = open(H, 'rb').read().decode('gbk')
m = re.search(r'^([ \t]*)float[ \t]+mag_dqz[^\n]*$', h, re.M)
assert m, '未找到 struct mag_dqz 字段'
h = h[:m.end()] + ''.join('\n%s %-8s %s;   /* VER=50 */' % (m.group(1), ty, f) for f, ty, _ in NEW) + h[m.end():]
dump(H, h, 'gbk', 'v50')
n += 1; print('  ok 4-struct +6')

# 5) 通道 + 列数
s = open(S, 'rb').read().decode('gbk')
m = re.search(r'^([ \t]*)ch\[c\+\+\][ \t]*=[ \t]*g_v5f_hold\.ekf\.mag_dqz[^\n]*$', s, re.M)
assert m, '未找到 mag_dqz 通道行'
s = s[:m.end()] + ''.join('\n%s ch[c++] = g_v5f_hold.ekf.%s;   /* VER=50 */' % (m.group(1), f) for f, _, _ in NEW) + s[m.end():]
mm = re.search(r'#define[ \t]+JF_CH_NUM[ \t]+(\d+)u', s)
assert mm and mm.group(1) == '133', 'JF_CH_NUM=%s' % (mm and mm.group(1))
s = s[:mm.start(1)] + '139' + s[mm.end(1):]
dump(S, s, 'gbk', 'v50')
n += 1; print('  ok 5-通道 +6, JF_CH_NUM=139')

# 6) VER
u = open(T, 'rb').read().decode('gbk')
assert u.count('#define V5F_FW_VER        49u') == 1
u = u.replace('#define V5F_FW_VER        49u', '#define V5F_FW_VER        50u', 1)
dump(T, u, 'gbk', 'v50')
n += 1; print('  ok 6-VER=50')

# 校验
c = open(P, 'rb').read().decode('gbk')
h2 = open(H, 'rb').read().decode('gbk')
s2 = open(S, 'rb').read().decode('gbk')
t2 = open(T, 'rb').read().decode('gbk')
ver = int(re.search(r'#define V5F_FW_VER\s+(\d+)u', t2).group(1))
nch = int(re.search(r'#define JF_CH_NUM\s+(\d+)u', s2).group(1))
i0 = c.index('static void ekf_m6_tilt')
m6 = c[i0:c.index('\nstatic void ', i0 + 10)]
CK = [('VER==50', ver == 50), ('编辑数==6', n == 6),
      ('M6 采集在', all('s_%s =' % f for f, _, _ in NEW if ('s_%s' % f) and ('s_' + f) in m6)),
      ('M6 内 dq 采集', 's_tilt_dqz = s_dx[IX_Q + 2] * RAD2DEG;' in m6),
      ('M6 内 pr 采集', 's_tilt_prz = pr[2];' in m6),
      ('M7 只看偏航仍在', '0x0100u, V5F_EKF_MAG_K_MAX' in c),
      ('M6 掩码仍 0x7EC0', '0x7EC0u, V5F_EKF_TILT_K_MAX' in c),
      ('publish +6', all('h->ekf.%s =' % f in c for f, _, _ in NEW)),
      ('struct +6', all(re.search(r'\b%s\b' % f, h2) for f, _, _ in NEW)),
      ('channel +6', all('g_v5f_hold.ekf.%s' % f in s2 for f, _, _ in NEW)),
      ('JF_CH_NUM==139', nch == 139),
      ('{ } 平衡', c.count('{') == c.count('}')),
      ('/* */ 平衡', c.count('/*') == c.count('*/'))]
for k, v in CK:
    print('  %-22s %s' % (k, 'PASS' if v else 'FAIL'))
    assert v, k
print('\nfw_tag = %d  (VER=%d, %d ch)' % ((ver << 16) | (nch << 8) | 7, ver, nch))
print('新列 133 tilt_dqx 134 tilt_dqy 135 tilt_dqz 136 tilt_prx 137 tilt_pry 138 tilt_prz')

# 读取端
J = R + r'\tools\calib\jf_load.py'
j = open(J, 'rb').read().decode('utf-8')
if 'CH_139' not in j:
    add = ("\nCH_139 = dict(CH_133)\n"
           "CH_139.update({'ekf_tilt_dqx': 133, 'ekf_tilt_dqy': 134, 'ekf_tilt_dqz': 135,\n"
           "               'ekf_tilt_prx': 136, 'ekf_tilt_pry': 137, 'ekf_tilt_prz': 138})\n")
    j = j.replace("\nCH_BY_NCH = {", add + "\nCH_BY_NCH = {", 1)
    j = j.replace('133: CH_133}', '133: CH_133, 139: CH_139}', 1)
    j = j.replace('    if nch == 133:\n        return CH_133\n',
                  '    if nch == 133:\n        return CH_133\n'
                  '    if nch == 139:\n        return CH_139\n', 1)
    open(J, 'wb').write(j.encode('utf-8'))
j2 = open(J, 'rb').read().decode('utf-8')
ok = 'CH_139 = dict(CH_133)' in j2 and '139: CH_139' in j2 and 'if nch == 139' in j2
print('  jf_load.py CH_139: %s' % ('PASS' if ok else 'FAIL'))
assert ok
K = r'C:\Users\33\Documents\v2\tools\calib\count_cols.py'
k = open(K, 'rb').read().decode('utf-8', 'ignore')
open(K, 'wb').write(re.sub(r'assert n == \d+', 'assert n == 139', k).encode('utf-8'))
print('  count_cols assert:',
      re.findall(r'assert n == (\d+)', open(K, 'rb').read().decode('utf-8', 'ignore')))
