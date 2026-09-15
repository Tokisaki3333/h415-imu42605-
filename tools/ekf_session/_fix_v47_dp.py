# -*- coding: utf-8 -*-
"""VER=46 -> 47：给二维水平投影观测加死点 —— 向量模小于阈值时**强制归零**。

理由（用户）：磁角（磁倾角）标定有误差，投影矢量模小时方向不可信：
  方位角误差 ~ delta_B / |v|，|v| -> 0 时发散。
  标称 |v| = cos(磁倾角) = cos(64.3) = 0.4335，实测 mag_bh 中位 0.4541。
  例：delta_B = 10.5 度 = 0.183 rad，|v| = 0.30 时方位角误差 ~ 35 度；
      |v| = 0.4335 时 ~ 24 度 —— 所以小模值必须直接归零，不能让它牵引。

做法：|v| = s_mag_bh < V5F_EKF_MAG_BH_MIN(0.30) 时
  - 本次不做任何更新（修正量强制为 0）
  - 把 5 个诊断量（mag_r / rx / ry / dqx / dqy / dqz）全部清 0，避免上报里
    出现"上一帧的残留修正"，让日志能明确区分"死点归零"与"没跑"
  - 置 chi2/gate 位、mag_rej 计数
可见判据：118 mag_bh < 0.30 且 120 mag_used = 0 就是死点帧。
"""
import re
import shutil

U = r'C:\Users\33\Documents\v2\h415-imu42605-\V5F\User'
P = U + r'\src\proc_ekf.c'
T = U + r'\inc\v5f_tune.h'


def dump(path, text, enc, tag):
    data = text.encode(enc)
    shutil.copy2(path, path + '.bak_' + tag)
    with open(path, 'wb') as f:
        f.write(data)


t = open(P, 'rb').read().decode('gbk')
a = "        s_mag_bh = sqrtf(Bn[0]*Bn[0] + Bn[1]*Bn[1]);"
assert t.count(a) == 1, 'mag_bh 锚点 %d 次' % t.count(a)
ins = (a + "   /* = |v|\uff1a\u786e\u5b9a\u7a0b\u5ea6 */\n"
       "        /* \u2605VER=47 \u6b7b\u70b9\uff1a\u5411\u91cf\u6a21\u8fc7\u5c0f -> \u5f3a\u5236\u5f52\u96f6\u3002\n"
       "         * \u78c1\u89d2(\u78c1\u503e\u89d2)\u6807\u5b9a\u6709\u8bef\u5dee\uff0c|v| \u5c0f\u65f6\u65b9\u4f4d\u89d2\u8bef\u5dee ~ delta_B/|v| \u53d1\u6563\uff0c\n"
       "         * \u4e0d\u80fd\u8ba9\u5b83\u53c2\u4e0e\u7275\u5f15\u3002\u6807\u79f0 |v| = cos(64.3) = 0.4335\uff0c\u5b9e\u6d4b\u4e2d\u4f4d 0.4541\u3002\n"
       "         * \u5f52\u96f6\u65f6\u628a\u8bca\u65ad\u91cf\u5168\u90e8\u6e05 0\uff0c\u4f7f\u65e5\u5fd7\u80fd\u533a\u5206\"\u6b7b\u70b9\u5f52\u96f6\"\u4e0e\"\u6ca1\u8dd1\"\u3002*/\n"
       "        if (s_mag_bh < V5F_EKF_MAG_BH_MIN) {\n"
       "            s_mag_r = 0.0f;\n"
       "            s_mag_rx = 0.0f; s_mag_ry = 0.0f;\n"
       "            s_mag_dqx = 0.0f; s_mag_dqy = 0.0f; s_mag_dqz = 0.0f;\n"
       "            if (s_rej[3] < 250u) s_rej[3]++;\n"
       "            s_gate_bits |= V5F_EKF_GB_CHI2;\n"
       "            return;\n"
       "        }\n")
t = t.replace(a, ins, 1)
dump(P, t, 'gbk', 'v47dp')

u = open(T, 'rb').read().decode('gbk')
assert re.search(r'#define\s+V5F_EKF_MAG_BH_MIN\b', u), 'tune 里没有 V5F_EKF_MAG_BH_MIN'
assert u.count('#define V5F_FW_VER        46u') == 1
u = u.replace('#define V5F_FW_VER        46u', '#define V5F_FW_VER        47u', 1)
dump(T, u, 'gbk', 'v47dp')

c = open(P, 'rb').read().decode('gbk')
t2 = open(T, 'rb').read().decode('gbk')
i0 = c.index('static void ekf_m7_mag')
m7 = c[i0:c.index('\nstatic void ', i0 + 10)]
bmin = re.search(r'#define\s+V5F_EKF_MAG_BH_MIN\s+(\S+)', t2).group(1)
ver = int(re.search(r'#define V5F_FW_VER\s+(\d+)u', t2).group(1))
nch = int(re.search(r'#define JF_CH_NUM\s+(\d+)u',
                    open(U + r'\src\SPI_rx.c', 'rb').read().decode('gbk')).group(1))
CK = [('VER==47', ver == 47),
      ('死点门在 M7 内', 'if (s_mag_bh < V5F_EKF_MAG_BH_MIN)' in m7),
      ('死点在 H_zero 之前', m7.index('s_mag_bh < V5F_EKF_MAG_BH_MIN') < m7.index('H_zero(2u)')),
      ('强制清零 6 个量', all(('%s = 0.0f;' % k) in m7 for k in
                          ['s_mag_r', 's_mag_rx', 's_mag_ry', 's_mag_dqx', 's_mag_dqy', 's_mag_dqz'])),
      ('死点 return 不做更新', 's_gate_bits |= V5F_EKF_GB_CHI2;\n            return;' in m7),
      ('二维观测仍在', 'r[0] = Bn[0] - b0x;' in m7),
      ('掩码仍 0x01C0', '0x01C0u, V5F_EKF_MAG_K_MAX' in m7),
      ('prop 自锁修复仍在', 'while (s_prop_row < EKF_N) ekf_prop_row();' in c),
      ('列数仍 133', nch == 133),
      ('{ } 平衡', c.count('{') == c.count('}')),
      ('/* */ 平衡', c.count('/*') == c.count('*/'))]
for k, v in CK:
    print('  %-24s %s' % (k, 'PASS' if v else 'FAIL'))
    assert v, k
print('\n  V5F_EKF_MAG_BH_MIN = %s   （标称 |v| = 0.4335，实测中位 0.4541）' % bmin)
print('  fw_tag = %d  (VER=%d, %d ch)' % ((ver << 16) | (nch << 8) | 7, ver, nch))
