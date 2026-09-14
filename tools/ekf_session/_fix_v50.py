# -*- coding: utf-8 -*-
"""VER=49 -> 50：单步状态修正量限幅（整体等比缩放 K），根治"疯转/不可抑制漂移"。

--- 实测（VER=47 日志，静止段，陀螺 <5 dps，bg 恒 0）---
   yaw 斜率 +1.49 / +30.60 / +28.68 / -315.90 度/秒
   roll 斜率 -24.73 / +28.77 / -36.29 / +73.39 度/秒
   sigma_yaw 13.5~16.5（滤波器自己知道烂）
   p_yy: p50 0.0599，1.32% 的帧塌到 <1e-6
   => 静止时姿态以 30~316 度/秒自转，而陀螺只有 5 dps：**是修正在驱动它，不是陀螺。**

--- 根因 ---
k_cap（VER=40）只限制了 |K| 的每个元素，**没有限制 |K*r|**。
实测 r 最大 178 度，MAG_K_MAX=0.05 -> 单步 8.9 度；更新率 349 Hz
-> 修正速率上限 3100 度/秒。M6 同理：TILT_K_MAX=0.010、r 可达 90 度 -> 314 度/秒。
于是任何大新息都能把姿态拽着"疯转"，且地磁/重力的牵引抵消不掉 -> 不可抑制漂移。
附带：bg/ba 恒 0（从未被估计），陀螺零偏无从补偿。

--- 修法 ---
在 ekf_update 里，算出 dx 后检查**姿态三轴的单步转角** |dx[6..8]|；
超过 V5F_EKF_DX_MAX_DEG 就把**整个 K 等比缩放** a = cap/|dx|，再用同一个 K
重算 dx 和 P：
    dx' = a K r
    P'  = P - (aK) S (aK)^T = P - a^2 K S K^T
两者同源，**协方差不会失配**（VER=36/37 那次是"回滚状态却不动 P"，才会崩溃性漂移；
这里是对 K 本体缩放，不是事后回滚状态）。
限幅只作用在"修正速率"上；真实运动由陀螺传播承担（不受限），
所以正常机动的手感不受影响，只把病态的每秒上千度修正压到 100 度/秒量级。
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
n = 0


def sub(a, b, nm):
    global t, n
    c = t.count(a)
    assert c == 1, '锚点[%s] 匹配 %d 次（应为 1）' % (nm, c)
    t = t.replace(a, b, 1)
    n += 1
    print('  ok  %s' % nm)


# 1) 诊断计数
sub("static uint8_t  s_mag_dp;",
    "static uint8_t  s_mag_dp;\n"
    "static uint8_t  s_step_lim;     /* \u2605VER=50 \u5355\u6b65\u4fee\u6b63\u88ab\u9650\u5e45\u7684\u6b21\u6570\uff08\u8bca\u65ad\uff09*/",
    '1-s_step_lim')

# 2) dx 之后插入限幅
old = """    /* dx = K r */
    for (i = 0u; i < EKF_N; i++) {
        s = 0.0f;
        for (k = 0u; k < m; k++) s += s_K[i][k] * r[k];
        s_dx[i] = s;
    }
"""
assert t.count(old) == 1
new = old + """    /* \u2605VER=50 \u5355\u6b65\u72b6\u6001\u4fee\u6b63\u9650\u5e45\uff08\u6574\u4f53\u7b49\u6bd4\u7f29\u653e K\uff0cdx \u4e0e P \u540c\u4e00\u4e2a K\uff09\u3002
     * \u4e3a\u4ec0\u4e48\u5fc5\u987b\u9650\uff1ak_cap \u53ea\u9650 |K|\uff0c\u4e0d\u9650 |K*r|\u3002\u5b9e\u6d4b r \u53ef\u8fbe 178 \u5ea6\uff0c
     * K=0.05 -> \u5355\u6b65 8.9 \u5ea6\uff0c\u800c\u66f4\u65b0\u7387 349 Hz -> \u4fee\u6b63\u901f\u7387\u4e0a\u9650 3100 \u5ea6/\u79d2
     * -> \u59ff\u6001\u201c\u75af\u8f6c\u201d\u3001\u4e0d\u53ef\u6291\u5236\u6f02\u79fb\uff08VER=47 \u5b9e\u6d4b\u9759\u6b62\u6bb5 yaw \u659c\u7387
     * -315.9 \u5ea6/\u79d2\u3001roll \u659c\u7387 -36~+73 \u5ea6/\u79d2\uff0c\u800c\u9640\u87ba\u53ea\u6709 <5 dps\u3001bg \u6052 0\uff09\u3002
     * \u9650\u5e45\u53ea\u4f5c\u7528\u4e8e\u201c\u4fee\u6b63\u901f\u7387\u201d\uff1b\u771f\u5b9e\u8fd0\u52a8\u7531\u9640\u87ba\u4f20\u64ad\u627f\u62c5\uff08\u4e0d\u53d7\u9650\uff09\uff0c
     * \u6240\u4ee5\u6b63\u5e38\u673a\u52a8\u624b\u611f\u4e0d\u53d7\u5f71\u54cd\u3002 */
    {
        float dxq = s_dx[IX_Q]*s_dx[IX_Q] + s_dx[IX_Q+1]*s_dx[IX_Q+1]
                  + s_dx[IX_Q+2]*s_dx[IX_Q+2];
        float cap = V5F_EKF_DX_MAX_DEG * DEG2RAD;
        if (dxq > cap * cap) {
            float sc = cap / sqrtf(dxq);
            for (i = 0u; i < EKF_N; i++) {
                for (j = 0u; j < m; j++) s_K[i][j] *= sc;
            }
            for (i = 0u; i < EKF_N; i++) {
                s = 0.0f;
                for (k = 0u; k < m; k++) s += s_K[i][k] * r[k];
                s_dx[i] = s;
            }
            s_step_lim = 1u;
        }
    }
"""
t = t.replace(old, new, 1)
n += 1
print('  ok  2-\u5355\u6b65\u9640\u8f6c\u9650\u5e45')

dump(P, t, 'gbk', 's2v')

# 3) tune
u = open(T, 'rb').read().decode('gbk')
m = re.search(r'^[ \t]*#define[ \t]+V5F_EKF_MAG_K_MAX[^\r\n]*$', u, re.M)
assert m, '未找到 V5F_EKF_MAG_K_MAX'
ins = ('\n/* \u2605VER=50 \u5355\u6b65\u59ff\u6001\u4fee\u6b63\u9650\u5e45\uff08\u5ea6\uff09\u3002k_cap \u53ea\u9650 |K| \u4e0d\u9650 |K*r|\uff0c\n'
       ' * r=178\u5ea6\u3001K=0.05 -> \u5355\u6b65 8.9\u5ea6 / 349Hz = 3100 \u5ea6/\u79d2\u7684\u4fee\u6b63\u901f\u7387 -> \u59ff\u6001\u75af\u8f6c\u3002\n'
       ' * \u8d85\u8fc7\u672c\u503c\u5219\u628a\u6574\u4e2a K \u7b49\u6bd4\u7f29\u653e\uff08dx \u4e0e P \u540c\u6e90\uff0c\u4e0d\u4f1a\u5931\u914d\uff09\u3002\n'
       ' * 0.30 \u5ea6 -> \u4fee\u6b63\u901f\u7387\u4e0a\u9650\u7ea6 105 \u5ea6/\u79d2\uff1b\u771f\u5b9e\u8fd0\u52a8\u7531\u9640\u87ba\u4f20\u64ad\uff0c\u4e0d\u53d7\u6b64\u9650\u3002 */\n'
       '#define V5F_EKF_DX_MAX_DEG        0.30f')
u = u[:m.end()] + ins + u[m.end():]
assert u.count('#define V5F_FW_VER        49u') == 1
u = u.replace('#define V5F_FW_VER        49u', '#define V5F_FW_VER        50u', 1)
dump(T, u, 'gbk', 's2v')

c = open(P, 'rb').read().decode('gbk')
h = open(T, 'rb').read().decode('gbk')
ver = int(re.search(r'#define V5F_FW_VER\s+(\d+)u', h).group(1))
nch = int(re.search(r'#define JF_CH_NUM\s+(\d+)u',
                    open(R + r'\V5F\User\src\SPI_rx.c', 'rb').read().decode('gbk')).group(1))
CK = [('VER==50', ver == 50), ('编辑数==2', n == 2),
      ('限幅在', 'if (dxq > cap * cap)' in c),
      ('K 整体缩放', 's_K[i][j] *= sc;' in c),
      ('重算 dx', c.count('s_dx[i] = s;') == 2),
      ('tune 在', h.count('V5F_EKF_DX_MAX_DEG') == 1),
      ('机体系死点门在', 'V5F_EKF_MAG_BHB_MIN' in c),
      ('M6 无偏航位', '0x7EC0u' in c),
      ('{ } 平衡', c.count('{') == c.count('}')),
      ('/* */ 平衡', c.count('/*') == c.count('*/')),
      ('括号差仍 -4', c.count('(') - c.count(')') == -4)]
h.encode('gbk'); c.encode('gbk')
for k, v in CK:
    print('  %-18s %s' % (k, 'PASS' if v else 'FAIL'))
    assert v, k
print('\nfw_tag = %d  (VER=50, %d ch)' % ((ver << 16) | (nch << 8) | 7, nch))
