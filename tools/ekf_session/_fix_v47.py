# -*- coding: utf-8 -*-
"""VER=46 -> 47：wrap_pi() 死循环防护。

--- 现场 ---
VER=46 记录（fw_tag 3045895，38932 帧 / 4.85 s）：
  整个 EKF 上报块（justfloat 83~121，含 ekf_q=FireWater CH0~3）**逐位恒定**，
  而同报文里 att.q(0~3)/level_dps/att_tilt 都在变 -> ISR 与旧链正常，只有 EKF 停了。
  冻结值 = 对齐刚完成时的原始状态：gate_bits=0x080（只有 aligned，无 step/tilt/mag/chi2）、
  p=v=ba=bg=0、sigma_yaw=3.00/tilt=0.50/pos_h=1.00 全为 P0、p_yy=0.002742=(3deg)^2。
  => 对齐那次 publish 之后，v5f_proc_ekf 再没走完一个周期，即**卡在 1053 行之后出不来**。

--- 根因 ---
全文件仅两处循环（第 153/154 行），都在 wrap_pi()：
    while (a >  EKF_PI) a -= EKF_TWOPI;
    while (a < -EKF_PI) a += EKF_TWOPI;
a = +/-Inf 时 a -/+ TWOPI 仍是 Inf，条件永真 -> **死循环**。
（NaN 反而能退出，因为 NaN 比较恒为假；所以只有 Inf 会锁死。）
VER=45 新增了一处 wrap_pi 调用（dr = fabsf(wrap_pi(r[0] - s_mag_r_prev))），
并且第一次让 M7 走进"持续大残差=框架偏置"这条原来永远不走的支路 ——
新的输入路径把 Inf 喂进了 wrap_pi，主循环随即死在 EKF 里。

--- 修法 ---
给两个 while 加整数计数上限（-Ofast 的 -ffinite-math-only 会把
isfinite()/x!=x 优化掉，所以不能用浮点判据兜底），超过即返回 0。
这一类"某个量变成 Inf -> 主循环锁死"的故障就此永久消除；
下一份日志即使还有 Inf，也不会再死机，并且能从各上报量看出是谁。
"""
import re
import shutil

R = r'C:\Users\33\Documents\v2\h415-imu42605-'
P = R + r'\V5F\User\src\proc_ekf.c'
T = R + r'\V5F\User\inc\v5f_tune.h'


def dump(path, text, enc, tag):
    data = text.encode(enc)
    shutil.copy2(path, path + '.bak_' + tag)
    with open(path, 'wb') as f:
        f.write(data)


t = open(P, 'rb').read().decode('gbk')
L = t.split('\n')
print('--- 改动前 148~160 ---')
for i in range(147, 160):
    print(i + 1, L[i][:100])

old = re.search(r'^([ \t]*)while \(a >[ ]*EKF_PI\) a -= EKF_TWOPI;[ \t]*\r?\n'
                r'([ \t]*)while \(a <[ ]*-EKF_PI\) a \+= EKF_TWOPI;[ \t]*$',
                t, re.M)
assert old, '未找到 wrap_pi 的两个 while'
i0 = t.rindex('\n', 0, old.start()) + 1
new = ('    /* \u2605VER=47 \u6b7b\u5faa\u73af\u9632\u62a4\uff1aa = +/-Inf \u65f6 a -/+ TWOPI \u4ecd\u662f Inf\uff0c\n'
       '     * \u4e24\u4e2a while \u6c38\u4e0d\u9000\u51fa -> \u4e3b\u5faa\u73af\u6b7b\u5728 EKF \u91cc\uff08ISR \u7167\u8dd1\u3001CDC \u7167\u51fa\u5e27\u3001\n'
       '     * ekf_q \u9010\u4f4d\u51bb\u7ed3\u5728\u5bf9\u9f50\u90a3\u4e00\u5e27\uff09\u3002\n'
       '     * -Ofast \u5e26 -ffinite-math-only\uff0cisfinite()/x!=x \u4f1a\u88ab\u4f18\u5316\u6389\uff0c\u6240\u4ee5\u7528\u6574\u6570\u8ba1\u6570\u515c\u5e95\u3002 */\n'
       '    {\n'
       '        uint32_t n = 0u;\n'
       '        while (a >  EKF_PI) { a -= EKF_TWOPI; if (++n > 64u) return 0.0f; }\n'
       '        while (a < -EKF_PI) { a += EKF_TWOPI; if (++n > 64u) return 0.0f; }\n'
       '    }')
t = t[:i0] + new + t[old.end():]
assert 'while (a >  EKF_PI) a -=' not in t and 'while (a < -EKF_PI) a +=' not in t
assert t.count('while (a >  EKF_PI) {') == 1 and t.count('while (a < -EKF_PI) {') == 1
assert t.count('if (++n > 64u) return 0.0f;') == 2
dump(P, t, 'gbk', 's2s')

# VER 47
u = open(T, 'rb').read().decode('gbk')
assert u.count('#define V5F_FW_VER        46u') == 1
u = u.replace('#define V5F_FW_VER        46u', '#define V5F_FW_VER        47u', 1)
dump(T, u, 'gbk', 's2s')

# 校验
c = open(P, 'rb').read().decode('gbk')
h = open(T, 'rb').read().decode('gbk')
ver = int(re.search(r'#define V5F_FW_VER\s+(\d+)u', h).group(1))
nch = int(re.search(r'#define JF_CH_NUM\s+(\d+)u',
                    open(R + r'\V5F\User\src\SPI_rx.c', 'rb').read().decode('gbk')).group(1))
CK = [('VER==47', ver == 47),
      ('两处 while 已加护栏', c.count('if (++n > 64u) return 0.0f;') == 2),
      ('无裸 while(a>)', 'while (a >  EKF_PI) a -=' not in c),
      ('{ } 平衡', c.count('{') == c.count('}')),
      ('/* */ 平衡', c.count('/*') == c.count('*/')),
      ('括号差仍 -4', c.count('(') - c.count(')') == -4)]
h.encode('gbk'); c.encode('gbk')
print()
for k, v in CK:
    print('  %-22s %s' % (k, 'PASS' if v else 'FAIL'))
    assert v, k
L = c.split('\n')
j = c.index('★VER=47')
j = c[:j].count('\n')
print('\n--- 改动后 ---')
for i in range(j - 2, j + 12):
    print(i + 1, L[i][:100])
print('\nfw_tag = %d  (VER=47, %d ch)' % ((ver << 16) | (nch << 8) | 7, nch))
