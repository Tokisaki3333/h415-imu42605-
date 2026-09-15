# -*- coding: utf-8 -*-
"""VER=51 -> 52：启动一步对齐改成**自校验**，不再依赖我推的符号。

背景：VER=51 的一步对齐是我自己推的几何式
    dpsi = (r_x*b0y − r_y*b0x)/n2
而 EKF 慢环走的是标准 Kalman（dx = K r, K = P H^T S^-1）。两者若约定不一致就反号。
数据证明慢环是对的（mag_r 零点几度且稳定、|mag_dqz| <= 1 度，收敛非发散），
那么反号的就是一步式 —— 后果正是"真实误差 theta 被反向施加成 2theta"，
误差 90 度 -> 180 度，与"拿起旋转后偏航转了 180 度"吻合（对齐块会复位 anchor，
拿起时的重对齐会再打一次一步式）。

修法：不猜符号 —— 转完用新姿态重算二维新息（含同样的投影），
      若模长反而变大，就还原并反向重做。对错都能自洽。
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
i0 = t.index('        /* \u2605VER=51-B \u542f\u52a8\u4e00\u6b21\u5feb\u901f\u504f\u822a\u5bf9\u9f50')
i1 = t.index('            return;\n        }\n', i0) + len('            return;\n        }\n')
old = t[i0:i1]
assert 'if (!s_mag_anchor)' in old and 'q_mul(dq, &s_x[IX_Q], qt);' in old and len(old) < 3000
new = """        /* \u2605VER=52 \u542f\u52a8\u4e00\u6b21\u5feb\u901f\u504f\u822a\u5bf9\u9f50\uff08**\u81ea\u6821\u9a8c**\uff09\u3002
         * \u521d\u59cb\u504f\u822a\u968f\u673a\u3001\u53ef\u80fd\u8fdc\u79bb\u78c1\u5317\uff0c\u6162\u73af\u592a\u6162\u3002
         * \u4e0d\u518d\u4fe1\u4efb\u63a8\u5bfc\u7684\u7b26\u53f7\uff1a\u5148\u8bd5\u8f6c +dpsi\uff0c\u7136\u540e\u7528\u65b0\u59ff\u6001
         * \u91cd\u7b97\u4e00\u6b21\u4e8c\u7ef4\u65b0\u606f\uff08\u542b\u540c\u6837\u7684\u6295\u5f71\uff09\uff0c\u6a21\u957f\u53d8\u5927\u5219
         * \u8fd8\u539f\u5e76\u53cd\u5411\u91cd\u505a\u3002\u5bf9\u9519\u90fd\u80fd\u81ea\u6d3d\u3002 */
        if (!s_mag_anchor) {
            float n2 = b0x * b0x + b0y * b0y;
            float dpsi = 0.0f, e0, e1, h2;
            float qb[4], dq[4], qt[4];
            uint32_t i2;
            e0 = sqrtf(r[0] * r[0] + r[1] * r[1]);
            if (n2 > 1e-6f) dpsi = (r[0] * b0y - r[1] * b0x) / n2;
            if (dpsi >  3.14159265f) dpsi =  3.14159265f;
            if (dpsi < -3.14159265f) dpsi = -3.14159265f;
            for (i2 = 0u; i2 < 4u; i2++) qb[i2] = s_x[IX_Q + i2];
            h2 = 0.5f * dpsi;
            dq[0] = cosf(h2); dq[1] = 0.0f; dq[2] = 0.0f; dq[3] = sinf(h2);
            q_mul(dq, &s_x[IX_Q], qt);
            for (i2 = 0u; i2 < 4u; i2++) s_x[IX_Q + i2] = qt[i2];
            q_norm(&s_x[IX_Q]);
            q_to_R(&s_x[IX_Q], Rt);
            rot_bn(Rt, mf, Bn);
            {
                float rx2 = Bn[0] - b0x, ry2 = Bn[1] - b0y, kk2;
                kk2 = (n2 > 1e-6f) ? (rx2 * b0x + ry2 * b0y) / n2 : 0.0f;
                rx2 -= kk2 * b0x; ry2 -= kk2 * b0y;
                e1 = sqrtf(rx2 * rx2 + ry2 * ry2);
            }
            if (e1 > e0) {
                for (i2 = 0u; i2 < 4u; i2++) s_x[IX_Q + i2] = qb[i2];
                h2 = -0.5f * dpsi;
                dq[0] = cosf(h2); dq[1] = 0.0f; dq[2] = 0.0f; dq[3] = sinf(h2);
                q_mul(dq, &s_x[IX_Q], qt);
                for (i2 = 0u; i2 < 4u; i2++) s_x[IX_Q + i2] = qt[i2];
                q_norm(&s_x[IX_Q]);
                dpsi = -dpsi;
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
"""
assert new.count('{') == new.count('}') and new.count('(') == new.count(')')
t = t[:i0] + new + t[i1:]
dump(P, t, 'gbk', 'v52')

u = open(T, 'rb').read().decode('gbk')
assert u.count('#define V5F_FW_VER        51u') == 1
u = u.replace('#define V5F_FW_VER        51u', '#define V5F_FW_VER        52u', 1)
dump(T, u, 'gbk', 'v52')

c = open(P, 'rb').read().decode('gbk')
t2 = open(T, 'rb').read().decode('gbk')
i0 = c.index('static void ekf_m7_mag')
m7 = c[i0:c.index('\nstatic void ', i0 + 10)]
ver = int(re.search(r'#define V5F_FW_VER\s+(\d+)u', t2).group(1))
nch = int(re.search(r'#define JF_CH_NUM\s+(\d+)u',
                    open(U + r'\src\SPI_rx.c', 'rb').read().decode('gbk')).group(1))
CK = [('VER==52', ver == 52),
      ('自校验在', 'if (e1 > e0) {' in m7),
      ('校验重算新息', 'rot_bn(Rt, mf, Bn);' in m7 and m7.count('rot_bn(Rt, mf, Bn);') == 2),
      ('反向重做', 'h2 = -0.5f * dpsi;' in m7),
      ('一步式仍是一次性', 'if (!s_mag_anchor)' in m7),
      ('投影仍在', 'r[0] -= kk * b0x;' in m7),
      ('掩码仍 0x0100', '0x0100u, V5F_EKF_MAG_K_MAX' in m7),
      ('死点仍在', 's_mag_bh < V5F_EKF_MAG_BH_MIN' in m7),
      ('列数仍 139', nch == 139),
      ('{ } 平衡', c.count('{') == c.count('}')),
      ('/* */ 平衡', c.count('/*') == c.count('*/'))]
for k, v in CK:
    print('  %-20s %s' % (k, 'PASS' if v else 'FAIL'))
    assert v, k
print('\nfw_tag = %d  (VER=%d, %d ch)' % ((ver << 16) | (nch << 8) | 7, ver, nch))
