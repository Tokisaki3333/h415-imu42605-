# -*- coding: utf-8 -*-
"""VER=43 -> 44：
 A) 修掉"地磁不启动"的自锁：EKF_N(17) > V5F_EKF_DECIM(16)
 B) 追加 5 列 CDC 诊断（122 -> 127）：prop_ok / f_ok / prop_row / stage / mag_rej

--- A 的根因（实测+代码双证）---
  ekf_prop_row() 每次只推 1 行（row = s_prop_row; ...; s_prop_row++）。
  阶段机： if (s_stage < V5F_EKF_DECIM=16) { if (s_F_ok && s_prop_row < EKF_N) ekf_prop_row(); s_stage++; }
  => 16 帧最多推 16 行，而 EKF_N = 17（p3+v3+q4+ba3+bg3+bb1）
  => s_prop_ok = (s_F_ok && s_prop_row >= EKF_N) 恒为 0
  => if (s_prop_ok) ekf_m6_tilt / ekf_m5_zupt / ekf_m7_mag / ekf_m1m2_gps /
     ekf_m4_vel / ekf_finalize() 全部被跳过。
  后果：EKF = 纯陀螺积分器。实测 VER=44/47/48 三个日志全部：
     mag_r 恒 0、mag_used 0%、chi2_rej 100%、gate_bits 只有 aligned|step|chi2
     （而运动段跟踪仍有 0.17%，因为 ekf_step_nominal() 在 stage 16 不受 s_prop_ok 约束）
  => 这就是"地磁为何不启动"：不是门、不是新息、不是坐标，是观测从来没被执行。

修法：前 15 帧仍各推 1 行（保持 ISR 负载均摊、不引入尖峰），
      最后一帧（stage 15）把剩余行补完（17 行 -> 该帧 2 行）。
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


NEW = [('prop_ok', 'uint8_t', 1), ('f_ok', 'uint8_t', 2), ('prop_row', 'uint8_t', 3),
       ('stage', 'uint8_t', 4), ('mag_rej', 'uint16_t', 5)]
n_ed = 0

# ============ A) 自锁修复 ============
t = open(P, 'rb').read().decode('gbk')
a = "        if (s_F_ok && s_prop_row < EKF_N) ekf_prop_row();\n        s_stage++;\n"
assert t.count(a) == 1, '传播锚点匹配 %d 次' % t.count(a)
b = ("        /* \u2605VER=44 \u4fee\u6b63\u81ea\u9501\uff1aEKF_N(17) > V5F_EKF_DECIM(16)\u3002\n"
     "         * \u539f\u6765 16 \u5e27\u6700\u591a\u63a8 16 \u884c -> s_prop_row \u6c38\u8fdc\u5230\u4e0d\u4e86 EKF_N\n"
     "         * -> s_prop_ok \u6052 0 -> \u6240\u6709\u89c2\u6d4b(M6/M5/M7/M1M2/M4)\u4e0e ekf_finalize()\n"
     "         *   \u5168\u90e8\u88ab\u8df3\u8fc7\uff0cEKF \u9000\u5316\u6210\u7eaf\u9640\u87ba\u79ef\u5206\u5668\n"
     "         *   \uff08\u5b9e\u6d4b VER=44/47/48\uff1amag_r \u6052 0\u3001mag_used 0%\u3001chi2 100%\u3001\n"
     "         *    gate_bits \u53ea\u6709 aligned|step|chi2\uff09\u3002\u8fd9\u5c31\u662f\u5730\u78c1\u4e0d\u542f\u52a8\u7684\u771f\u56e0\u3002\n"
     "         * \u524d 15 \u5e27\u4ecd\u5404\u63a8 1 \u884c\uff08ISR \u8d1f\u8f7d\u5747\u644a\uff09\uff0c\u6700\u540e\u4e00\u5e27\u628a\u4f59\u4e0b\u7684\u884c\u8865\u5b8c\u3002*/\n"
     "        if (s_F_ok) {\n"
     "            if (s_stage == V5F_EKF_DECIM - 1u) {\n"
     "                while (s_prop_row < EKF_N) ekf_prop_row();\n"
     "            } else if (s_prop_row < EKF_N) {\n"
     "                ekf_prop_row();\n"
     "            }\n"
     "        }\n"
     "        s_stage++;\n")
t = t.replace(a, b, 1)
n_ed += 1
print('  ok A-自锁修复')

# ============ B) ekf_publish 追加字段 ============
m = re.search(r'^([ \t]*)h->ekf\.p_yy[ \t]*=[^\n]*$', t, re.M)
assert m, '未找到 ekf_publish 的 p_yy 行'
ins = ''.join('\n%s h->ekf.%s = %s;' % (m.group(1), f, {
    'prop_ok': '(uint8_t)s_prop_ok', 'f_ok': '(uint8_t)s_F_ok',
    'prop_row': '(uint8_t)s_prop_row', 'stage': '(uint8_t)s_stage',
    'mag_rej': 's_rej[3]'}[f]) for f, _, _ in NEW)
t = t[:m.end()] + ins + t[m.end():]
n_ed += 1
print('  ok B-ekf_publish +5 字段')
dump(P, t, 'gbk', 'v44fix')

# ============ B2) SPI_rx.h 结构体 ============
h = open(H, 'rb').read().decode('gbk')
m = re.search(r'^([ \t]*)(?:float|uint\d+_t|int\d+_t)[ \t]+p_yy[^\n]*$', h, re.M)
assert m, '未找到 v5f_ekf_t 的 p_yy 字段'
ins = ''.join('\n%s %-12s %s;   /* VER=44 EKF \u5185\u90e8\u72b6\u6001\uff1a\u4f20\u64ad\u662f\u5426\u5b8c\u6210 */'
              % (m.group(1), ty, nm) for nm, ty, _ in NEW)
h = h[:m.end()] + ins + h[m.end():]
dump(H, h, 'gbk', 'v44fix')
n_ed += 1
print('  ok B2-SPI_rx.h +5 字段')

# ============ B3) SPI_rx.c 通道 + JF_CH_NUM ============
s = open(S, 'rb').read().decode('gbk')
m = re.search(r'^([ \t]*)ch\[c\+\+\][ \t]*=[ \t]*g_v5f_hold\.ekf\.p_yy[^\n]*$', s, re.M)
assert m, '未找到 SPI_rx.c 的 p_yy 通道'
ins = ''.join('\n%s ch[c++] = %s;   /* VER=44 %s */' % (m.group(1), {
    'prop_ok': '(float)g_v5f_hold.ekf.prop_ok', 'f_ok': '(float)g_v5f_hold.ekf.f_ok',
    'prop_row': '(float)g_v5f_hold.ekf.prop_row', 'stage': '(float)g_v5f_hold.ekf.stage',
    'mag_rej': '(float)g_v5f_hold.ekf.mag_rej'}[f], f) for f, _, _ in NEW)
s = s[:m.end()] + ins + s[m.end():]
old_n = re.search(r'#define[ \t]+JF_CH_NUM[ \t]+(\d+)u', s)
assert old_n and old_n.group(1) == '122', 'JF_CH_NUM 不是 122: %s' % (old_n and old_n.group(1))
s = s[:old_n.start(1)] + '127' + s[old_n.end(1):]
if 'V5F_MAG_BH_MIN' in s:
    pass
dump(S, s, 'gbk', 'v44fix')
n_ed += 1
print('  ok B3-SPI_rx.c +5 通道, JF_CH_NUM=127')

# ============ VER 44 ============
u = open(T, 'rb').read().decode('gbk')
assert u.count('#define V5F_FW_VER        43u') == 1
u = u.replace('#define V5F_FW_VER        43u', '#define V5F_FW_VER        44u', 1)
dump(T, u, 'gbk', 'v44fix')
n_ed += 1
print('  ok VER=44')

# ============ 校验 ============
c = open(P, 'rb').read().decode('gbk')
h2 = open(H, 'rb').read().decode('gbk')
s2 = open(S, 'rb').read().decode('gbk')
t2 = open(T, 'rb').read().decode('gbk')
ver = int(re.search(r'#define V5F_FW_VER\s+(\d+)u', t2).group(1))
nch = int(re.search(r'#define JF_CH_NUM\s+(\d+)u', s2).group(1))
CK = [('VER==44', ver == 44), ('编辑数==5', n_ed == 5),
      ('传播补完在', 'while (s_prop_row < EKF_N) ekf_prop_row();' in c),
      ('s_prop_ok 判据未动', 's_prop_ok = (uint8_t)(s_F_ok && s_prop_row >= EKF_N);' in c),
      ('publish +5', all('h->ekf.%s =' % f in c for f, _, _ in NEW)),
      ('struct +5', all(re.search(r'\b%s\b' % f, h2) for f, _, _ in NEW)),
      ('channel +5', all('g_v5f_hold.ekf.%s' % f in s2 for f, _, _ in NEW)),
      ('JF_CH_NUM==127', nch == 127),
      ('{ } 平衡', c.count('{') == c.count('}')),
      ('/* */ 平衡', c.count('/*') == c.count('*/'))]
for k, v in CK:
    print('  %-20s %s' % (k, 'PASS' if v else 'FAIL'))
    assert v, k
print('\nfw_tag = %d  (VER=44, %d ch)' % ((ver << 16) | (nch << 8) | 7, nch))
print('新列: 122 prop_ok, 123 f_ok, 124 prop_row, 125 stage, 126 mag_rej')
