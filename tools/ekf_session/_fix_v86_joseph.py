# -*- coding: utf-8 -*-
"""VER=86：修 VER=85 的根本性错误
 ① 去掉新息钳位（新息是"我错多少"的唯一信息，钳了就看不见误差）
 ② 恢复 K 逐元素钳位（限步）
 ③ P 改用 Joseph 形式: P <- (I-KH)P(I-KH)' + K S_eff K' = P - KHP - KHP' + K S_eff K'
    对任意 K 成立；K 最优时退化为 P - K S K'（严格更正确，不会过度自信）
 ④ s_KS 用 S_eff = soft*S（chi2 软加权时保持精确）
"""
import os, shutil

ROOT = r'C:\Users\33\Documents\v2\h415-imu42605-'
TAG = '.bak_v86'
P_EKF = os.path.join(ROOT, r'V5F\User\src\proc_ekf.c')
P_T = os.path.join(ROOT, r'V5F\User\inc\v5f_tune.h')


def load(p, e): return open(p, 'rb').read().decode(e)


def save(p, t, e):
    d = t.encode(e)
    if not os.path.exists(p + TAG):
        shutil.copy2(p, p + TAG)
    old = open(p, 'rb').read()
    open(p, 'wb').write(d)
    os.utime(p, None)
    assert open(p, 'rb').read() == d
    print('  OK %-16s %6d -> %6d B' % (os.path.basename(p), len(old), len(d)))


def sub1(t, old, new, what):
    n = t.count(old)
    assert n == 1, '%s: anchor %d times\n%r' % (what, n, old[:110])
    return t.replace(old, new, 1)


t = load(P_EKF, 'gbk')

# --- 新增 s_KHP 静态 ---
t = sub1(t, 'static float    s_KS[EKF_N][3];',
         'static float    s_KS[EKF_N][3];\n'
         'static float    s_KHP[EKF_N][EKF_N];   /* \u2605VER=86 K*(H P)\uff0cJoseph \u5f62\u5f0f\u7528 */',
         's_KHP decl')

# --- s_soft 局部变量 ---
t = sub1(t, '    float det, nis = 0.0f, s;',
         '    float det, nis, sc;\n'
         '    float s_soft = 1.0f;   /* \u2605VER=86 chi2 \u8f6f\u52a0\u6743\u7684\u5b9e\u9645 R \u653e\u5927\u500d\u6570 */',
         's_soft decl')

# --- chi2 分支里记录 soft ---
old = '        for (i = 0u; i < m; i++) {\n            for (j = 0u; j < m; j++) Si[i][j] /= sc;\n        }\n        s_chi2_soft = 1u;'
assert t.count(old) == 1, 'chi2 scale: %d' % t.count(old)
t = t.replace(old, '        for (i = 0u; i < m; i++) {\n            for (j = 0u; j < m; j++) Si[i][j] /= sc;\n        }\n'
                   '        s_soft = sc;                     /* \u2605VER=86 S_eff = soft*S */\n'
                   '        s_chi2_soft = 1u;', 1)

# --- 主体：去 rr 钳位、恢复 K 钳位、Joseph 更新 ---
OLD = ('    {\n'
       '        float rr[3];\n'
       '        for (k = 0u; k < m; k++) {\n'
       '            rr[k] = r[k];\n'
       '            if (k_cap > 0.0f) {\n'
       '                if (rr[k] >  k_cap) rr[k] =  k_cap;\n'
       '                if (rr[k] < -k_cap) rr[k] = -k_cap;\n'
       '            }\n'
       '        }\n'
       '        /* dx = K r */\n'
       '        for (i = 0u; i < EKF_N; i++) {\n'
       '            s = 0.0f;\n'
       '            for (k = 0u; k < m; k++) s += s_K[i][k] * rr[k];\n'
       '            s_dx[i] = s;\n'
       '        }\n'
       '    }\n'
       '    for (i = 0u; i < EKF_N; i++) {\n'
       '        for (j = 0u; j < m; j++) {\n'
       '            s = 0.0f;\n'
       '            for (k = 0u; k < m; k++) s += s_K[i][k] * S[k][j];\n'
       '            s_KS[i][j] = s;\n'
       '        }\n'
       '    }\n'
       '    for (i = 0u; i < EKF_N; i++) {\n'
       '        for (j = 0u; j < EKF_N; j++) {\n'
       '            s = 0.0f;\n'
       '            for (k = 0u; k < m; k++) s += s_KS[i][k] * s_K[j][k];\n'
       '            s_Pn[i][j] -= s;\n'
       '        }\n'
       '    }')
NEW = ('    /* \u2605VER=86 \u6062\u590d K \u7684\u9010\u5143\u7d20\u94b3\u4f4d\uff08\u9650\u6b65\uff09\u3002\n'
       '     * VER=85 \u6539\u6210\u94b3\u65b0\u606f + \u7528"\u6700\u4f18 K"\u7684\u516c\u5f0f\u66f4\u65b0 P \u662f\u6839\u672c\u9519\u8bef\uff1a\n'
       '     * \u5b9e\u9645\u53ea\u63d0\u53d6\u4e86\u6781\u5c0f\u4e00\u90e8\u5206\u4fe1\u606f\uff0cP \u5374\u6309\u5168\u90e8\u4fe1\u606f\u6536\u7f29 -> \u8fc7\u5ea6\u81ea\u4fe1\u3002\n'
       '     * \u5b9e\u6d4b\uff1asigma_yaw \u9489\u5728 2.500 \u5ea6\u4e0b\u754c\u3001sigma_tilt 0.21 \u5ea6\uff0c\n'
       '     *       \u800c |\u65b0\u606f| p90 27.6 / max 33.8 \u5ea6 -> \u589e\u76ca\u584c\u9677\u3001\u59ff\u6001\u81ea\u7531\u6f02\u3002\n'
       '     * \u6b63\u786e\u4e0d\u53d8\u91cf\uff1aP \u5fc5\u987b\u5bf9\u5e94"\u5b9e\u9645\u7528\u6389\u7684\u589e\u76ca"\u3002 */\n'
       '    if (k_cap > 0.0f) {\n'
       '        for (i = 0u; i < EKF_N; i++) {\n'
       '            for (j = 0u; j < m; j++) {\n'
       '                if (s_K[i][j] >  k_cap) s_K[i][j] =  k_cap;\n'
       '                if (s_K[i][j] < -k_cap) s_K[i][j] = -k_cap;\n'
       '            }\n'
       '        }\n'
       '    }\n'
       '    /* dx = K r\uff08r \u4e0d\u94b3\uff1a\u65b0\u606f\u662f"\u6211\u9519\u591a\u5c11"\u7684\u552f\u4e00\u4fe1\u606f\uff0c\u94b3\u4e86\u5c31\u770b\u4e0d\u89c1\u8bef\u5dee\uff09 */\n'
       '    for (i = 0u; i < EKF_N; i++) {\n'
       '        s = 0.0f;\n'
       '        for (k = 0u; k < m; k++) s += s_K[i][k] * r[k];\n'
       '        s_dx[i] = s;\n'
       '    }\n'
       '    /* s_KS = K * S_eff\uff08S_eff = soft*S\uff0cchi2 \u8f6f\u52a0\u6743\u65f6\u4fdd\u6301\u7cbe\u786e\uff09 */\n'
       '    for (i = 0u; i < EKF_N; i++) {\n'
       '        for (j = 0u; j < m; j++) {\n'
       '            s = 0.0f;\n'
       '            for (k = 0u; k < m; k++) s += s_K[i][k] * S[k][j];\n'
       '            s_KS[i][j] = s * s_soft;\n'
       '        }\n'
       '    }\n'
       '    /* s_KHP = K (H P) = K * PHt\' */\n'
       '    for (i = 0u; i < EKF_N; i++) {\n'
       '        for (j = 0u; j < EKF_N; j++) {\n'
       '            s = 0.0f;\n'
       '            for (k = 0u; k < m; k++) s += s_K[i][k] * s_PHt[j][k];\n'
       '            s_KHP[i][j] = s;\n'
       '        }\n'
       '    }\n'
       '    /* Joseph: P <- P - KHP - KHP\' + K S_eff K\'\n'
       '     * K \u53d6\u6700\u4f18\u65f6\u6070\u597d\u9000\u5316\u4e3a P - K S K\'\uff08=\u539f\u516c\u5f0f\uff09\uff0c\n'
       '     * K \u88ab\u94b3\u65f6\u5219\u662f\u5bf9\u8be5 K \u7cbe\u786e\u7684\u540e\u9a8c\u534f\u65b9\u5dee -> \u4e0d\u4f1a\u8fc7\u5ea6\u81ea\u4fe1\u3002 */\n'
       '    for (i = 0u; i < EKF_N; i++) {\n'
       '        for (j = 0u; j < EKF_N; j++) {\n'
       '            s = 0.0f;\n'
       '            for (k = 0u; k < m; k++) s += s_KS[i][k] * s_K[j][k];\n'
       '            s_Pn[i][j] += s - s_KHP[i][j] - s_KHP[j][i];\n'
       '        }\n'
       '    }')
assert t.count(OLD) == 1, 'main block: %d' % t.count(OLD)
t = t.replace(OLD, NEW, 1)
save(P_EKF, t, 'gbk')

# ---------------- v5f_tune.h ----------------
t = load(P_T, 'gbk')
t = sub1(t, '#define V5F_FW_VER        85u', '#define V5F_FW_VER        86u', 'FW_VER')
save(P_T, t, 'gbk')

print()
print('fw_tag = (86<<16)|(154<<8)|7 = %d' % ((86 << 16) | (154 << 8) | 7))
