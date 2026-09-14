# -*- coding: utf-8 -*-
"""VER=37 -> 38：修**一阶圆锥误差** —— 姿态增量必须逐帧复合，不能相加。

病（用户实测）：只要三轴同时转动，偏航就出现极大误差；单轴转动没有。
根因：我的预积分把 16 帧的**机体系轴角矢量相加**（s_dth += w*dt），最后一次性
      构造 dq。而"相加"只在单轴转动时等于"复合"（可交换）；三轴同时转动时
      两者相差交换子项 [th_i, th_j]/2 -> 表现为绕第三轴的虚假转动 = 偏航误差，
      即经典的一阶圆锥误差（coning）。速率越高越显著（误差 ~ w^2）。
      旧链 proc_attitude 在 8 kHz 下**每帧复合一次**，所以没有这个误差 ——
      这就是 CH4~7 稳、CH0~3 崩的真正来源。
修：每帧按时间顺序把增量复合成四元数 s_dq（s_dq <- s_dq (x) Exp(w*dt)，
    用精确半角，不是一阶近似），周期末 q <- q (x) s_dq。
    顺序不能反：机体增量在**右**侧，时间序也是右乘。
"""
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


# 1) 新增四元数增量累加器
sub("static float    s_dth[3], s_dvb[3];",
    "static float    s_dth[3], s_dvb[3];\n"
    "static float    s_dq[4];              /* 本周期姿态增量的**复合**结果（逐帧右乘）*/\n"
    "                                       /* 不能用 s_dth 相加代替：三轴同时转动时\n"
    "                                        * 相加与复合差一个交换子 = 圆锥误差 */", 'var')

# 2) 逐帧复合（替换原来的相加）
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
     *   虚假转动（偏航误差），即一阶圆锥误差 —— 实测"三轴同时转偏航极大误差"。
     *   这里用精确半角（sin/cos），不是一阶近似。顺序：机体增量在右侧、按时间右乘。 */
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

# 3) 周期末用它
sub("""    /* 姿态：q <- q (x) Exp(dth)。dth 是**机体**系增量 -> 右乘。 */
    dq[0] = 1.0f; dq[1] = 0.5f*s_dth[0]; dq[2] = 0.5f*s_dth[1]; dq[3] = 0.5f*s_dth[2];
    q_norm(dq);
    q_mul(&s_x[IX_Q], dq, qn);
    for (i = 0u; i < 4u; i++) s_x[IX_Q + i] = qn[i];
    q_norm(&s_x[IX_Q]);""",
    """    /* 姿态：q <- q (x) s_dq。s_dq 是这 16 帧**逐帧复合**出来的四元数增量
     * （不是把轴角矢量相加 —— 那会在三轴同时转动时产生圆锥误差/偏航误差）。
     * dq 只用于下面建 F 的地方，不再构造姿态增量。 */
    q_mul(&s_x[IX_Q], s_dq, qn);
    for (i = 0u; i < 4u; i++) s_x[IX_Q + i] = qn[i];
    q_norm(&s_x[IX_Q]);""", 'use')

# 4) 周期末/对齐时复位
sub("""        for (i = 0u; i < 3u; i++) { s_dth[i] = 0.0f; s_dvb[i] = 0.0f; }""",
    """        for (i = 0u; i < 3u; i++) { s_dth[i] = 0.0f; s_dvb[i] = 0.0f; }
        s_dq[0] = 1.0f; s_dq[1] = 0.0f; s_dq[2] = 0.0f; s_dq[3] = 0.0f;""", 'reset')
sub("""            for (i = 0u; i < 3u; i++) { s_dth[i] = 0.0f; s_dvb[i] = 0.0f; }""",
    """            for (i = 0u; i < 3u; i++) { s_dth[i] = 0.0f; s_dvb[i] = 0.0f; }
            s_dq[0] = 1.0f; s_dq[1] = 0.0f; s_dq[2] = 0.0f; s_dq[3] = 0.0f;""", 'reset2')

# dq 变量若不再使用会有 unused 警告 -> 保留声明但加 (void)
assert t.count('/*') == t.count('*/') and t.count('{') == t.count('}')
sw(P, t, 'gbk', 's2h')

u = open(T, 'rb').read().decode('gbk')
assert u.count('#define V5F_FW_VER        37u') == 1
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
             ('花括号平衡', c.count('{') == c.count('}')),
             ('注释配平', c.count('/*') == c.count('*/'))]:
    print('  %-14s %s' % (k, v))
    assert v, k
print()
print('fw_tag 期望 = %d' % ((ver << 16) | (nch << 8) | 1 | 2 | 4))
