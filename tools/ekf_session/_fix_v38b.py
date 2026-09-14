# -*- coding: utf-8 -*-
"""重做 VER=38 的 proc_ekf.c 部分（复位锚点有两处，改用正则一次替换）。"""
import re
import shutil

R = r'C:\Users\33\Documents\v2\h415-imu42605-'
P = R + r'\V5F\User\src\proc_ekf.c'
T = R + r'\V5F\User\inc\v5f_tune.h'


def sw(path, text, enc, tag):
    data = text.encode(enc)
    shutil.copy2(path, path + '.bak_' + tag)
    with open(path, 'wb') as f:
        f.write(data)


t = open(P, 'rb').read().decode('gbk')


def sub(a, b, nm):
    global t
    n = t.count(a)
    assert n == 1, '锚点 %s 匹配 %d 次' % (nm, n)
    t = t.replace(a, b, 1)
    print('  ok:', nm)


sub("static float    s_dth[3], s_dvb[3];",
    "static float    s_dth[3], s_dvb[3];\n"
    "static float    s_dq[4];              /* 本周期姿态增量的**复合**结果（逐帧右乘）*/\n"
    "                                       /* 不能拿 s_dth 相加代替：三轴同时转动时\n"
    "                                        * 相加与复合差一个交换子 = 圆锥误差 */", 'var')

sub("""    for (i = 0u; i < 3u; i++) {
        s_dth[i] += (w[i] - s_x[IX_BG + i]) * dt;
        s_dvb[i] += f[i] * dt;
        s_mag_dth[i] += w[i] * dt;      /* 用未扣 bg 的量测即可：bg 只有 0.1 dps 量级 */
    }""",
    """    for (i = 0u; i < 3u; i++) {
        s_dth[i] += (w[i] - s_x[IX_BG + i]) * dt;
        s_dvb[i] += f[i] * dt;
        s_mag_dth[i] += w[i] * dt;      /* 用未扣 bg 的量测即可：bg 只有 0.1 dps 量级 */
    }
    /* ★ 姿态增量**逐帧复合成四元数**，而不是把轴角矢量相加。
     *   相加只在单轴转动时等于复合；三轴同时转动时丢掉交换子项，表现为绕第三轴的
     *   虚假转动（偏航误差），即一阶圆锥误差 —— 实测"三轴同时转偏航极大误差"，
     *   而单轴转动完全正常。旧链 8 kHz 下每帧复合一次，所以没有这个误差。
     *   用精确半角（sin/cos），不用一阶近似；顺序：机体增量在右侧、按时间右乘。 */
    {
        float wx = w[0] - s_x[IX_BG], wy = w[1] - s_x[IX_BG + 1], wz = w[2] - s_x[IX_BG + 2];
        float dqf[4], qt[4], th2 = (wx*wx + wy*wy + wz*wz) * dt * dt;
        if (th2 > 1e-12f) {
            float th = sqrtf(th2), h = 0.5f * th, sc = sinf(h) / th;
            dqf[0] = cosf(h);
            dqf[1] = sc * wx * dt;
            dqf[2] = sc * wy * dt;
            dqf[3] = sc * wz * dt;
        } else {
            dqf[0] = 1.0f;
            dqf[1] = 0.5f * wx * dt;
            dqf[2] = 0.5f * wy * dt;
            dqf[3] = 0.5f * wz * dt;
        }
        q_norm(dqf);
        q_mul(s_dq, dqf, qt);
        for (i = 0u; i < 4u; i++) s_dq[i] = qt[i];
        q_norm(s_dq);
    }""", 'compose')

sub("""    /* 姿态：q <- q (x) Exp(dth)。dth 是**机体**系增量 -> 右乘。 */
    dq[0] = 1.0f; dq[1] = 0.5f*s_dth[0]; dq[2] = 0.5f*s_dth[1]; dq[3] = 0.5f*s_dth[2];
    q_norm(dq);
    q_mul(&s_x[IX_Q], dq, qn);
    for (i = 0u; i < 4u; i++) s_x[IX_Q + i] = qn[i];
    q_norm(&s_x[IX_Q]);""",
    """    /* 姿态：q <- q (x) s_dq。s_dq 是这 16 帧**逐帧复合**出来的四元数增量，
     * 不是把轴角矢量相加（那样三轴同时转动会产生圆锥误差 -> 偏航大误差）。 */
    q_mul(&s_x[IX_Q], s_dq, qn);
    for (i = 0u; i < 4u; i++) s_x[IX_Q + i] = qn[i];
    q_norm(&s_x[IX_Q]);""", 'use')

# 两处 s_dth 复位后都补上 s_dq 复位
pat = r'(\n(\s+)for \(i = 0u; i < 3u; i\+\+\) \{ s_dth\[i\] = 0\.0f; s_dvb\[i\] = 0\.0f; \})'
t, n = re.subn(pat,
               r'\1\n\2s_dq[0] = 1.0f; s_dq[1] = 0.0f; s_dq[2] = 0.0f; s_dq[3] = 0.0f;',
               t)
assert n == 2, n
print('  ok: 复位 x%d' % n)

# dq[] 不再用于姿态增量 -> 避免 unused 警告：直接删掉它的声明
t2 = t.replace('    float dq[4], qn[4];', '    float qn[4];')
if t2 != t:
    t = t2
    print('  ok: 去掉未用的 dq[]')

assert t.count('/*') == t.count('*/') and t.count('{') == t.count('}')
sw(P, t, 'gbk', 's2h')

u = open(T, 'rb').read().decode('gbk')
if '#define V5F_FW_VER        37u' in u:
    u = u.replace('#define V5F_FW_VER        37u', '#define V5F_FW_VER        38u', 1)
    sw(T, u, 'gbk', 's2h')

c = open(P, 'rb').read().decode('gbk')
ver = int(re.search(r'#define V5F_FW_VER\s+(\d+)u', open(T, 'rb').read().decode('gbk')).group(1))
nch = int(re.search(r'#define JF_CH_NUM\s+(\d+)u',
                    open(R + r'\V5F\User\src\SPI_rx.c', 'rb').read().decode('gbk')).group(1))
for k, v in [('VER=38', ver == 38),
             ('逐帧复合在', 'q_mul(s_dq, dqf, qt)' in c),
             ('周期末用它', 'q_mul(&s_x[IX_Q], s_dq, qn)' in c),
             ('精确半角', 'sc = sinf(h) / th' in c),
             ('两处复位', c.count('s_dq[0] = 1.0f;') == 2),
             ('无 dq 残留', 'dq[0] = 1.0f; dq[1] = 0.5f*s_dth' not in c),
             ('花括号平衡', c.count('{') == c.count('}')),
             ('注释配平', c.count('/*') == c.count('*/'))]:
    print('  %-14s %s' % (k, v))
    assert v, k
print()
print('fw_tag 期望 = %d' % ((ver << 16) | (nch << 8) | 1 | 2 | 4))
