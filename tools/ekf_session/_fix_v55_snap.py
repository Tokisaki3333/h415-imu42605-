# -*- coding: utf-8 -*-
"""VER=54 -> 55：大误差一律一步整角修正，不再依赖 s_mag_anchor。

数据（VER=54, fw_tag 3574535, 14.27s）：
  mag_r 从第 0 帧就 136.5 度且全程 135~160 不变；
  mag_dqz p50 0.463 max 2.544 度 —— **一步式一次大角度都没打出来**；
  p_yy 从 0.231 一路涨到 0.690（无有效修正）。
=> 偏航差 180 度（mag_r≈136 = 180 度特征的 114.6 度再叠加倾角误差），
   线性化修正 ∝ sin(Δ) 在 180 度处为 0 -> 卡住；
   而精确角一步式挂在 `if (!s_mag_anchor)` 上，s_mag_anchor 早就为 1，
   所以大误差永远轮不到它 —— 这就是"更弱智"的原因。

改：触发条件加上"新息模长大"这一项。
  r = v - v0；误差 30 度 -> |r|≈0.22；60 度 -> 0.43；180 度 -> 0.87（|v0|=0.4335）
  取 |r0|+|r1| > 0.5 => 约 >60 度的误差一律立刻整角转到位。
  自校验（转完重算、变差就反向）仍在，符号错了它会自己反回来。
"""
import re
import shutil

U = r'C:\Users\33\Documents\v2\h415-mu42605-'.replace('h415-mu', 'h415-imu')
U = r'C:\Users\33\Documents\v2\h415-imu42605-\V5F\User'
P = U + r'\src\proc_ekf.c'
T = U + r'\inc\v5f_tune.h'


def dump(path, text, enc, tag):
    data = text.encode(enc)
    shutil.copy2(path, path + '.bak_' + tag)
    with open(path, 'wb') as f:
        f.write(data)


t = open(P, 'rb').read().decode('gbk')
a = "        if (!s_mag_anchor) {"
assert t.count(a) == 1, 'anchor 条件锚点 %d 次' % t.count(a)
b = ("        /* \u2605VER=55 \u5927\u8bef\u5dee\uff08\u542b \u00b1180 \u978d\u70b9\uff09\u4e00\u5f8b\u4e00\u6b65\u6574\u89d2\u4fee\u6b63\uff0c\n"
     "         * \u4e0d\u518d\u4f9d\u8d56 s_mag_anchor \u2014\u2014 \u5b9e\u6d4b VER=54 \u56e0\u4e3a anchor \u5df2\u4e3a 1\uff0c\n"
     "         * 180 \u5ea6\u8fd9\u79cd\u5927\u8bef\u5dee\u6c38\u8fdc\u8f6e\u4e0d\u5230\u4e00\u6b65\u5f0f\uff0c\u4e8e\u662f\u5361\u4f4f\u4e0d\u52a8\u3002\n"
     "         * r = v - v0\uff1a\u8bef\u5dee 30 \u5ea6 -> |r|\u22480.22\uff0c60 \u5ea6 -> 0.43\uff0c180 \u5ea6 -> 0.87\n"
     "         * \u53d6 |r0|+|r1| > 0.5 \u7ea6\u7b49\u4e8e\u8bef\u5dee > 60 \u5ea6\u3002 */\n"
     "        if (!s_mag_anchor || (fabsf(r[0]) + fabsf(r[1]) > 0.5f)) {")
t = t.replace(a, b, 1)
dump(P, t, 'gbk', 'v55')

u = open(T, 'rb').read().decode('gbk')
assert u.count('#define V5F_FW_VER        54u') == 1
u = u.replace('#define V5F_FW_VER        54u', '#define V5F_FW_VER        55u', 1)
dump(T, u, 'gbk', 'v55')

c = open(P, 'rb').read().decode('gbk')
t2 = open(T, 'rb').read().decode('gbk')
i0 = c.index('static void ekf_m7_mag')
m7 = c[i0:c.index('\nstatic void ', i0 + 10)]
ver = int(re.search(r'#define V5F_FW_VER\s+(\d+)u', t2).group(1))
nch = int(re.search(r'#define JF_CH_NUM\s+(\d+)u',
                    open(U + r'\src\SPI_rx.c', 'rb').read().decode('gbk')).group(1))
CK = [('VER==55', ver == 55),
      ('大误差触发在', 'fabsf(r[0]) + fabsf(r[1]) > 0.5f' in m7),
      ('精确角仍在', 'dpsi = atan2f(crs, dt2);' in m7),
      ('自校验仍在', 'if (e1 > e0) {' in m7),
      ('无投影残留', 'kk' not in m7),
      ('掩码仍 0x0100', '0x0100u, V5F_EKF_MAG_K_MAX' in m7),
      ('死点仍 0.12', re.search(r'#define\s+V5F_EKF_MAG_BH_MIN\s+(\S+)', t2).group(1).startswith('0.12')),
      ('列数仍 139', nch == 139),
      ('{ } 平衡', c.count('{') == c.count('}')),
      ('/* */ 平衡', c.count('/*') == c.count('*/'))]
for k, v in CK:
    print('  %-20s %s' % (k, 'PASS' if v else 'FAIL'))
    assert v, k
print('\nfw_tag = %d  (VER=%d, %d ch)' % ((ver << 16) | (nch << 8) | 7, ver, nch))
