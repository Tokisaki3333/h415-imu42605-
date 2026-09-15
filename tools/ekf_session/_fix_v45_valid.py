# -*- coding: utf-8 -*-
"""VER=44 -> 45：把"地磁向量有效判据"改成正确的量（机体系条件数），并追加 1 列上报。

--- 现有三条判据，没有一条量到会退化的量 ---
 L1  1362: gate->ekf_mag_yaw = (s_mn_ok && V5F_EKF_YAW_OBS_EN)
     1361: s_mn_ok = (dev < V5F_EKF_MAG_NORM_DEV),  dev = |mag_norm - 慢基线|
     -> 只测**模值**。而退化是**方向**退化（|f| 仍是 1），模值判据看不见。
 L2  610/614: bh2 = Bn[0]^2+Bn[1]^2（**导航系**水平分量，且用 EKF 自己的姿态算出来，
     自我参照），门限 V5F_EKF_MAG_BH_MIN = 0.30。
     实测该量恒在 0.436~0.470、bh<0.30 占 **0.00%** -> **从未拒过任何一个样本，是死判据**。
 L3  638: |r| > 45 度 -> 丢。这是**新息/状态一致性**门，不是测量有效门；
     而且它同时把"测量坏"和"状态偏"混为一谈（状态越偏，有效向量越像无效）。

--- 真正决定偏航可观测性的量（机体系，与姿态无关、非自我参照）---
 fhb = hypot(f_x, f_y)  （= 磁场与"偏航所绕轴(机体 z)"的夹角的正弦）
 实测：min 0.0526  p10 0.0960  p50 0.3686  -> 约 10% 的帧真的退化，
 正是"有时地磁向量退化导致误差极大"。fhb->0 时方位角由姿态模型与标定误差决定，
 新息爆炸 -> 修正量是垃圾。

--- 修法（"向量有效就实时启动修正"）---
 1) 硬底线：fhb < V5F_EKF_MAG_BHB_MIN(0.15) -> 该帧无效，跳过（不 latch 任何状态）。
 2) 连续加权：R_mag += (FERR/fhb)^2 -> 越靠近退化，权重越小而不是突然开关，
    有效时立刻以正常权重实时修正。
 3) 模值判据(L1)保留（它能抓真正的模值异常，与方向判据互补）；
    导航系 bh2 门保留但降级为极端兜底，不再当主判据。
 4) 新增上报列 127 ekf_mag_fhb：把 fhb 直接打出来，这是本判据的唯一直接证据。
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


n = 0
t = open(P, 'rb').read().decode('gbk')

# 1) 局部变量
a = "    float R[1], r[1], Rt[3][3], Bn[3], mf[3], hh;"
assert t.count(a) == 1
t = t.replace(a, "    float R[1], r[1], Rt[3][3], Bn[3], mf[3], hh, fhb2;", 1)
n += 1
print('  ok 1-局部变量 fhb2')

# 2) 机体系有效性判据（插在 mf 取值之后；注意该行全文出现 2 次，限定 M7 内）
anc = "    for (i = 0u; i < 3u; i++) mf[i] = h->mag.f[i];\n"
f0 = t.index('static void ekf_m7_mag')
i0 = t.index(anc, f0)
lim = t.index('static void ', f0 + 10)
assert t.count(anc, f0, lim) == 1
blk = ("    /* \u2605VER=45 \u6b63\u786e\u7684\u5411\u91cf\u6709\u6548\u5224\u636e\uff1a**\u673a\u4f53\u7cfb** fhb = hypot(f_x,f_y)\u3002\n"
       "     * \u65e7\u4e09\u6761\u90fd\u6ca1\u91cf\u5230\u4f1a\u9000\u5316\u7684\u91cf\uff1a\n"
       "     *   L1 s_mn_ok \u53ea\u6d4b\u6a21\u503c\uff08\u9000\u5316\u662f\u65b9\u5411\u9000\u5316\uff0c|f| \u4ecd\u4e3a 1\uff09\uff1b\n"
       "     *   L2 \u5bfc\u822a\u7cfb bh2 \u6052 0.44~0.47\u3001\u4ece\u672a\u89e6\u53d1 = \u6b7b\u5224\u636e\uff1b\n"
       "     *   L3 |r|>45 \u662f\u65b0\u606f\u4e00\u81f4\u6027\u95e8\uff0c\u4e0d\u662f\u6d4b\u91cf\u6709\u6548\u95e8\u3002\n"
       "     * fhb->0 \u65f6\u65b9\u4f4d\u89d2\u7531\u59ff\u6001\u6a21\u578b\u4e0e\u6807\u5b9a\u8bef\u5dee\u51b3\u5b9a\uff0c\u65b0\u606f\u7206\u70b8\u3002\n"
       "     * \u5b9e\u6d4b fhb: min 0.053 p10 0.096 p50 0.369 -> \u7ea6 10% \u7684\u5e27\u771f\u9000\u5316\u3002*/\n"
       "    fhb2 = h->mag.f[0]*h->mag.f[0] + h->mag.f[1]*h->mag.f[1];\n"
       "    if (fhb2 < V5F_EKF_MAG_BHB_MIN * V5F_EKF_MAG_BHB_MIN) {\n"
       "        if (s_rej[3] < 250u) s_rej[3]++;\n"
       "        s_gate_bits |= V5F_EKF_GB_CHI2;\n"
       "        return;                     /* \u5411\u91cf\u9000\u5316\uff0c\u672c\u5e27\u4e0d\u53c2\u4e0e\u4fee\u6b63 */\n"
       "    }\n")
t = t[:i0 + len(anc)] + blk + t[i0 + len(anc):]
n += 1
print('  ok 2-机体系有效性判据')

# 3) R 连续加权
a = "        R[0] = V5F_EKF_MAG_SIG_RAD * V5F_EKF_MAG_SIG_RAD;"
assert t.count(a) == 1
b = ("        /* \u2605VER=45 \u8fde\u7eed\u52a0\u6743\uff1a\u65b9\u4f4d\u89d2\u8bef\u5dee ~ sigma_B/fhb\uff0c\n"
     "         * \u6709\u6548\u65f6\u7acb\u523b\u4ee5\u6b63\u5e38\u6743\u91cd\u4fee\u6b63\uff0c\u8d8a\u9760\u8fd1\u9000\u5316\u6743\u91cd\u8d8a\u5c0f\u3002*/\n"
     "        {\n"
     "            float sf = V5F_EKF_MAG_FERR / sqrtf(fhb2);\n"
     "            R[0] = V5F_EKF_MAG_SIG_RAD * V5F_EKF_MAG_SIG_RAD + sf * sf;\n"
     "        }")
t = t.replace(a, b, 1)
n += 1
print('  ok 3-R 连续加权')

# 4) 上报 fhb：加持久静态量 s_mag_fhb，M7 里赋值，publish 里发出
assert t.count("static uint8_t  s_mag_gate;") == 1
t = t.replace("static uint8_t  s_mag_gate;",
              "static float    s_mag_fhb;      /* \u2605VER=45 \u673a\u4f53\u7cfb\u6c34\u5e73\u5360\u6bd4 = \u5411\u91cf\u6709\u6548\u6027\u5224\u636e */\n"
              "static uint8_t  s_mag_gate;", 1)
assert t.count("    fhb2 = h->mag.f[0]*h->mag.f[0] + h->mag.f[1]*h->mag.f[1];\n") == 1
t = t.replace("    fhb2 = h->mag.f[0]*h->mag.f[0] + h->mag.f[1]*h->mag.f[1];\n",
              "    fhb2 = h->mag.f[0]*h->mag.f[0] + h->mag.f[1]*h->mag.f[1];\n"
              "    s_mag_fhb = sqrtf(fhb2);\n", 1)
m = re.search(r'^([ \t]*)h->ekf\.mag_rej[ \t]*=[ \t]*s_rej\[3\];[^\n]*$', t, re.M)
assert m, '未找到 publish 的 mag_rej 行'
t = t[:m.end()] + '\n%s h->ekf.mag_fhb = s_mag_fhb;' % m.group(1) + t[m.end():]
n += 1
print('  ok 4-上报 mag_fhb')
dump(P, t, 'gbk', 'v45valid')

# 5) 结构体
h = open(H, 'rb').read().decode('gbk')
a = re.search(r'^([ \t]*)uint16_t[ \t]+mag_rej[^\n]*$', h, re.M)
assert a, '未找到 mag_rej 字段'
h = h[:a.end()] + '\n%s float     mag_fhb;   /* VER=45 \u673a\u4f53\u7cfb\u6c34\u5e73\u5360\u6bd4\uff08\u5411\u91cf\u6709\u6548\u6027\uff09 */' % a.group(1) + h[a.end():]
dump(H, h, 'gbk', 'v45valid')
n += 1
print('  ok 5-SPI_rx.h mag_fhb')

# 6) 通道
s = open(S, 'rb').read().decode('gbk')
a = re.search(r'^([ \t]*)ch\[c\+\+\][ \t]*=[ \t]*\(float\)g_v5f_hold\.ekf\.mag_rej[^\n]*$', s, re.M)
assert a, '未找到 mag_rej 通道行'
s = s[:a.end()] + '\n%s ch[c++] = g_v5f_hold.ekf.mag_fhb;   /* VER=45 \u673a\u4f53\u7cfb\u6c34\u5e73\u5360\u6bd4 */' % a.group(1) + s[a.end():]
m = re.search(r'#define[ \t]+JF_CH_NUM[ \t]+(\d+)u', s)
assert m and m.group(1) == '127', 'JF_CH_NUM=%s' % (m and m.group(1))
s = s[:m.start(1)] + '128' + s[m.end(1):]
dump(S, s, 'gbk', 'v45valid')
n += 1
print('  ok 6-SPI_rx.c mag_fhb, JF_CH_NUM=128')

# 7) tune
u = open(T, 'rb').read().decode('gbk')
a = re.search(r'^[ \t]*#define[ \t]+V5F_EKF_MAG_BH_MIN[^\r\n]*$', u, re.M)
assert a
u = u[:a.end()] + ('\n/* \u2605VER=45 \u5411\u91cf\u6709\u6548\u6027\u5224\u636e\uff08**\u673a\u4f53\u7cfb**\uff09\uff1a\n'
                   ' * fhb = hypot(f_x,f_y)\uff0c\u5373\u78c1\u573a\u4e0e\u504f\u822a\u6240\u7ed5\u8f74(\u673a\u4f53 z)\u7684\u5939\u89d2\u6b63\u5f26\u3002\n'
                   ' * fhb < \u672c\u503c -> \u65b9\u4f4d\u89d2\u4e0d\u53ef\u7528\uff0c\u672c\u5e27\u4e0d\u4fee\u6b63\u3002\u5b9e\u6d4b fhb p10=0.096\u3002 */\n'
                   '#define V5F_EKF_MAG_BHB_MIN      0.15f\n'
                   '/* \u2605VER=45 \u5730\u78c1\u6a21\u578b\u8bef\u5dee\uff08\u5355\u4f4d\u77e2\u91cf\uff09\uff0c\u7528\u4e8e R \u8fde\u7eed\u52a0\u6743\uff1aR += (FERR/fhb)^2\u3002 */\n'
                   '#define V5F_EKF_MAG_FERR         0.010f') + u[a.end():]
assert u.count('#define V5F_FW_VER        44u') == 1
u = u.replace('#define V5F_FW_VER        44u', '#define V5F_FW_VER        45u', 1)
dump(T, u, 'gbk', 'v45valid')
n += 1
print('  ok 7-tune +2 定义, VER=45')

# 校验
c = open(P, 'rb').read().decode('gbk')
h2 = open(H, 'rb').read().decode('gbk')
s2 = open(S, 'rb').read().decode('gbk')
t2 = open(T, 'rb').read().decode('gbk')
ver = int(re.search(r'#define V5F_FW_VER\s+(\d+)u', t2).group(1))
nch = int(re.search(r'#define JF_CH_NUM\s+(\d+)u', s2).group(1))
CK = [('VER==45', ver == 45), ('编辑数==7', n == 7),
      ('机体系判据在', 'fhb2 < V5F_EKF_MAG_BHB_MIN * V5F_EKF_MAG_BHB_MIN' in c),
      ('R 加权在', 'V5F_EKF_MAG_FERR / sqrtf(fhb2)' in c),
      ('fhb 有持久量', 's_mag_fhb = sqrtf(fhb2);' in c),
      ('publish mag_fhb', 'h->ekf.mag_fhb     = s_mag_fhb;' in c),
      ('struct mag_fhb', 'mag_fhb' in h2),
      ('channel mag_fhb', 'g_v5f_hold.ekf.mag_fhb' in s2),
      ('JF_CH_NUM==128', nch == 128),
      ('prop 自锁修复仍在', 'while (s_prop_row < EKF_N) ekf_prop_row();' in c),
      ('{ } 平衡', c.count('{') == c.count('}')),
      ('/* */ 平衡', c.count('/*') == c.count('*/'))]
for k, v in CK:
    print('  %-20s %s' % (k, 'PASS' if v else 'FAIL'))
    assert v, k
print('\nfw_tag = %d  (VER=45, %d ch)   新列 127 = ekf_mag_fhb' % ((ver << 16) | (nch << 8) | 7, nch))
