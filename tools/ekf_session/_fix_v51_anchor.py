# -*- coding: utf-8 -*-
"""VER=50 -> 51：两件事
 A) 投影：去掉二维新息里"平行于 v"的分量 —— 偏航修不掉的那部分（模值/倾角误差），
    留着会被 K 的偏航列持续吃进去 -> 偏航被永久驱动（实测 mag_dqz 均值 0.0757、
    max 1.636 度/次，349 Hz -> 最高 571 度/秒，静止也照转；而 mag_bh=0.5166 与模型
    |v0|=0.4335 差 19%，这个差值就是那个修不掉的分量）。
 B) 启动一次性快速偏航对齐：初始偏航完全随机、很可能远离磁北，慢环拉太慢。
    第一帧有效地磁观测直接按几何一步转到位（绕导航系 z），之后永久交给正常慢环。

数学（两轴都自洽）：
  绕导航 z 转 dθ 时  δv = dθ*(−b0y, b0x)   —— 恰好垂直于 v0
  => 只有垂直于 v0 的分量是偏航可修的；平行分量投影掉
  => 一步对齐角 dpsi = (r_x*b0y − r_y*b0x)/(b0x^2+b0y^2)
  施加方式与 ekf_inject 同一约定：q <- Exp((0,0,dpsi)) ⊗ q
"""
import re
import shutil

R = r'C:\Users\33\Documents\v2\h415-imu42605-'
U = R + r'\V5F\User'
P = U + r'\src\proc_ekf.c'
T = U + r'\inc\v5f_tune.h'


def dump(path, text, enc, tag):
    data = text.encode(enc)
    shutil.copy2(path, path + '.bak_' + tag)
    with open(path, 'wb') as f:
        f.write(data)


t = open(P, 'rb').read().decode('gbk')
n = 0

# 1) 静态量 + 快照
a = "static float    s_tilt_prx, s_tilt_pry, s_tilt_prz;"
assert t.count(a) == 1
t = t.replace(a, a + "\n"
              "static uint8_t  s_mag_anchor;   /* \u2605VER=51 \u542f\u52a8\u4e00\u6b21\u5feb\u901f\u504f\u822a\u5bf9\u9f50\u662f\u5426\u5df2\u505a */", 1)
n += 1; print('  ok 1-s_mag_anchor')

# 2) 投影 + 一次性对齐（插在 r[1] 赋值之后、诊断量之前）
a = "        r[0] = Bn[0] - b0x;\n        r[1] = Bn[1] - b0y;\n"
assert t.count(a) == 1, 'r 赋值锚点 %d' % t.count(a)
b = a + """        /* \u2605VER=51-A \u6295\u5f71\uff1a\u53bb\u6389\u5e73\u884c\u4e8e v \u7684\u5206\u91cf\u3002
         * \u7ed5\u504f\u822a\u8f6c\u65f6 dv = dth*(-b0y, b0x)\uff0c\u6070\u597d\u5782\u76f4\u4e8e v0\uff1b
         * \u5e73\u884c\u5206\u91cf\u662f\u78c1\u573a\u6a21\u503c/\u503e\u89d2\u8bef\u5dee\uff0c\u504f\u822a\u6c38\u8fdc\u4fee\u4e0d\u6389\uff0c
         * \u7559\u7740\u5c31\u4f1a\u88ab K \u7684\u504f\u822a\u5217\u6301\u7eed\u5403\u8fdb\u53bb -> \u504f\u822a\u88ab\u6c38\u4e45\u9a71\u52a8\u3002 */
        {
            float n2 = b0x * b0x + b0y * b0y;
            if (n2 > 1e-6f) {
                float kk = (r[0] * b0x + r[1] * b0y) / n2;
                r[0] -= kk * b0x;
                r[1] -= kk * b0y;
            }
        }
"""
t = t.replace(a, b, 1)
n += 1; print('  ok 2-A 投影')

# 3) 一次性快速对齐：插在 R 赋值之前
a = "        R[0] = sig2; R[1] = 0.0f; R[2] = 0.0f; R[3] = sig2;"
assert t.count(a) == 1
b = ("""        /* \u2605VER=51-B \u542f\u52a8\u4e00\u6b21\u5feb\u901f\u504f\u822a\u5bf9\u9f50\uff1a\u521d\u59cb\u504f\u822a\u5b8c\u5168\u968f\u673a\u3001
         * \u5f88\u53ef\u80fd\u8fdc\u79bb\u78c1\u5317\uff0c\u6162\u73af\u62c9\u592a\u6162\u3002\u7b2c\u4e00\u5e27\u6709\u6548\u89c2\u6d4b\u76f4\u63a5\u6309\u51e0\u4f55
         * \u4e00\u6b65\u8f6c\u5230\u4f4d\uff08\u7ed5\u5bfc\u822a\u7cfb z\uff09\uff0c\u4e4b\u540e\u6c38\u4e45\u4ea4\u7ed9\u6b63\u5e38\u6162\u73af\u3002
         * \u65bd\u52a0\u65b9\u5f0f\u4e0e ekf_inject \u540c\u4e00\u7ea6\u5b9a\uff1aq <- Exp((0,0,dpsi)) (x) q\u3002 */
        if (!s_mag_anchor) {
            float n2 = b0x * b0x + b0y * b0y;
            float dpsi = 0.0f;
            if (n2 > 1e-6f) dpsi = (r[0] * b0y - r[1] * b0x) / n2;
            if (dpsi > 3.14159265f) dpsi = 3.14159265f;
            if (dpsi < -3.14159265f) dpsi = -3.14159265f;
            {
                float hh2 = 0.5f * dpsi;
                float dq[4], qt[4];
                uint32_t i2;
                dq[0] = cosf(hh2); dq[1] = 0.0f; dq[2] = 0.0f; dq[3] = sinf(hh2);
                q_mul(dq, &s_x[IX_Q], qt);
                for (i2 = 0u; i2 < 4u; i2++) s_x[IX_Q + i2] = qt[i2];
                q_norm(&s_x[IX_Q]);
            }
            s_mag_anchor = 1u;
            s_mag_used = 1u;
            s_mag_dqx = 0.0f; s_mag_dqy = 0.0f; s_mag_dqz = dpsi * RAD2DEG;
            s_gate_bits |= V5F_EKF_GB_MAG;
            r[0] = 0.0f; r[1] = 0.0f;
            s_mag_rx = 0.0f; s_mag_ry = 0.0f;
            s_mag_r = 0.0f;
            return;
        }
""" + a)
t = t.replace(a, b, 1)
n += 1; print('  ok 3-B 一次性对齐')

# 4) 对齐块复位 anchor
a = "            s_bh_idx = 0u; s_bh_fill = 0u;"
assert t.count(a) == 1, '对齐块锚点 %d' % t.count(a)
t = t.replace(a, a + "\n            s_mag_anchor = 0u;   /* \u2605VER=51 \u91cd\u65b0\u5bf9\u9f50\u540e\u5141\u8bb8\u518d\u505a\u4e00\u6b21\u5feb\u901f\u504f\u822a\u5bf9\u9f50 */", 1)
n += 1; print('  ok 4-对齐时复位 anchor')
dump(P, t, 'gbk', 'v51')

u = open(T, 'rb').read().decode('gbk')
assert u.count('#define V5F_FW_VER        50u') == 1
u = u.replace('#define V5F_FW_VER        50u', '#define V5F_FW_VER        51u', 1)
dump(T, u, 'gbk', 'v51')
n += 1; print('  ok 5-VER=51')

c = open(P, 'rb').read().decode('gbk')
t2 = open(T, 'rb').read().decode('gbk')
i0 = c.index('static void ekf_m7_mag')
m7 = c[i0:c.index('\nstatic void ', i0 + 10)]
ver = int(re.search(r'#define V5F_FW_VER\s+(\d+)u', t2).group(1))
nch = int(re.search(r'#define JF_CH_NUM\s+(\d+)u',
                    open(U + r'\src\SPI_rx.c', 'rb').read().decode('gbk')).group(1))
CK = [('VER==51', ver == 51), ('编辑数==5', n == 5),
      ('投影在', 'r[0] -= kk * b0x;' in m7 and 'r[1] -= kk * b0y;' in m7),
      ('投影在诊断之前', m7.index('r[0] -= kk * b0x;') < m7.index('s_mag_rx = r[0];')),
      ('一次性对齐在', 'if (!s_mag_anchor)' in m7 and 'q_mul(dq, &s_x[IX_Q], qt);' in m7),
      ('对齐在 R 赋值之前', m7.index('if (!s_mag_anchor)') < m7.index('R[0] = sig2;')),
      ('对齐后 return', 's_mag_anchor = 1u;' in m7 and 'return;' in m7),
      ('anchor 复位在', c.count('s_mag_anchor = 0u;') == 1),
      ('掩码仍 0x0100', '0x0100u, V5F_EKF_MAG_K_MAX' in m7),
      ('死点仍在', 's_mag_bh < V5F_EKF_MAG_BH_MIN' in m7),
      ('列数仍 139', nch == 139),
      ('{ } 平衡', c.count('{') == c.count('}')),
      ('/* */ 平衡', c.count('/*') == c.count('*/'))]
for k, v in CK:
    print('  %-22s %s' % (k, 'PASS' if v else 'FAIL'))
    assert v, k
print('\nfw_tag = %d  (VER=%d, %d ch)' % ((ver << 16) | (nch << 8) | 7, ver, nch))
