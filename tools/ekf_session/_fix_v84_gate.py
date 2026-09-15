# -*- coding: utf-8 -*-
"""VER=84：落实仿真验证过的机制 + 上报分析量
 1) v5f_tune.h  VER 83->84, 加 MAG_GT_HOLD_S / MAG_RSCALE_MAX / GB_MAGHOLD
 2) proc_ekf.c  磁 R 含 (tan(dip)*sigma_tilt)^2 ; 倾角参考失效超时暂停磁 ;
                倾角失效计时(每帧) ; 6 个上报量
 3) SPI_rx.h    v5f_ekf_t 加 6 个字段
 4) SPI_rx.c    JF_CH_NUM 149->155, 末尾追加 6 列
 5) jf_load.py  加 CH_155
"""
import os, shutil

ROOT = r'C:\Users\33\Documents\v2\h415-imu42605-'
TAG = '.bak_v84'
P_T = os.path.join(ROOT, r'V5F\User\inc\v5f_tune.h')
P_EKF = os.path.join(ROOT, r'V5F\User\src\proc_ekf.c')
P_H = os.path.join(ROOT, r'V5F\User\inc\SPI_rx.h')
P_C = os.path.join(ROOT, r'V5F\User\src\SPI_rx.c')
P_L = os.path.join(ROOT, r'tools\calib\jf_load.py')
chg = []


def load(p, e): return open(p, 'rb').read().decode(e)


def save(p, t, e):
    d = t.encode(e)
    if not os.path.exists(p + TAG):
        shutil.copy2(p, p + TAG)
    old = open(p, 'rb').read()
    open(p, 'wb').write(d)
    os.utime(p, None)
    assert open(p, 'rb').read() == d
    chg.append((os.path.basename(p), len(old), len(d)))
    print('  OK %-16s %6d -> %6d B' % (os.path.basename(p), len(old), len(d)))


def sub1(t, old, new, what):
    n = t.count(old)
    assert n == 1, '%s: 锚点 %d 次\n%r' % (what, n, old[:100])
    return t.replace(old, new, 1)


# ---------------- 1) v5f_tune.h ----------------
t = load(P_T, 'gbk')
t = sub1(t, '#define V5F_FW_VER        83u', '#define V5F_FW_VER        84u', 'FW_VER')
t = sub1(t, '#define V5F_EKF_MAG_R_MAX_DEG    150.0f',
         '#define V5F_EKF_MAG_R_MAX_DEG    150.0f\n'
         '/* \u2605VER=84 \u78c1\u878d\u5408\u7684\u95e8\u63a7\u5224\u636e = \u503e\u89d2\u53c2\u8003\u662f\u5426\u6709\u6548\uff08\u4e0d\u662f |a| \u5e26\u5bbd\uff09\u3002\n'
         ' * \u51e0\u4f55\u4e8b\u5b9e\uff1a\u78c1\u65b0\u606f\u5bf9\u503e\u89d2\u8bef\u5dee\u7684\u7075\u654f\u5ea6 = tan(dip) = V5F_EKF_DIP_TAN = 2.08\n'
         ' *   \uff08= b0z/b0h\uff1a\u503e\u89d2\u8bef\u5dee\u628a\u78c1\u573a\u7684\u957f\u7ad6\u76f4\u5206\u91cf\u8f6c\u8fdb\u6c34\u5e73\u9762\uff09\uff0c\u800c H \u53ea\u4fdd\u7559\u504f\u822a\u5217\n'
         ' *   -> \u503e\u89d2\u8bef\u5dee\u4f1a\u4ee5 2.08 \u500d\u88ab\u5f53\u6210\u504f\u822a\u6fc0\u52b1\u3002\u6240\u4ee5\uff1a\n'
         ' *   1) R \u91cc\u5fc5\u987b\u542b (tan(dip)*sigma_tilt)^2\uff1b\n'
         ' *   2) \u91cd\u529b\u89c2\u6d4b\u8fde\u7eed\u5931\u6548\u8d85 MAG_GT_HOLD_S -> \u6682\u505c\u78c1\u66f4\u65b0\uff08\u5426\u5219\n'
         ' *      \u78c1\u66f4\u65b0\u7684\u503e\u659c\u884c dx[6:8]=P[6:8,8]*r/S \u4f1a\u5728\u503e\u89d2\u65e0\u89c2\u6d4b\u65f6\u65e0\u754c\u7d2f\u79ef\uff09\u3002 */\n'
         '#define V5F_EKF_MAG_GT_HOLD_S     0.30f\n'
         '#define V5F_EKF_MAG_RSCALE_MAX    100.0f', 'MAG tunables')
t = sub1(t, '#define V5F_EKF_GB_ALIGN_FORCE   0x1000u',
         '#define V5F_EKF_GB_ALIGN_FORCE   0x1000u   /* \u89e3\u7b97\u5668\u5f3a\u5236\u5bf9\u9f50\uff08\u5bf9\u9f50\u5f02\u5e38\uff09 */\n'
         '#define V5F_EKF_GB_MAGHOLD       0x2000u   /* \u2605VER=84 \u78c1\u56e0\u503e\u89d2\u53c2\u8003\u5931\u6548\u88ab\u6682\u505c */',
         'GB_MAGHOLD')
save(P_T, t, 'gbk')

# ---------------- 2) proc_ekf.c ----------------
t = load(P_EKF, 'gbk')
t = sub1(t, 'static float    s_mag_age_ms;',
         'static float    s_mag_age_ms;\n'
         'static float    s_tilt_bad_s;         /* \u2605VER=84 \u503e\u89d2\u53c2\u8003\u8fde\u7eed\u5931\u6548\u65f6\u957f(s) */\n'
         'static float    s_tilt_inv_ms;        /* \u2605VER=84 \u4e0a\u62a5\uff1a\u503e\u89d2\u53c2\u8003\u5931\u6548\u65f6\u957f(ms) */\n'
         'static uint8_t  s_grav_ok;            /* \u2605VER=84 \u4e0a\u62a5\uff1a\u91cd\u529b\u89c2\u6d4b\u95e8\u72b6\u6001 */\n'
         'static float    s_tilt_sig_deg;       /* \u2605VER=84 \u4e0a\u62a5\uff1a\u503e\u89d2 1sigma(\u5ea6) */\n'
         'static float    s_mag_rs;             /* \u2605VER=84 \u4e0a\u62a5\uff1a\u78c1 R \u653e\u5927\u500d\u6570 */\n'
         'static uint8_t  s_mag_hold;           /* \u2605VER=84 \u4e0a\u62a5\uff1a\u78c1\u56e0\u503e\u89d2\u5931\u6548\u88ab\u6682\u505c */',
         'statics')

# 每帧累加倾角参考失效时长
t = sub1(t, '        s_mag_age_ms = (ts != 0ULL && tk > ts) ? (float)(tk - ts) * 1e-5f : 0.0f;\n    }',
         '        s_mag_age_ms = (ts != 0ULL && tk > ts) ? (float)(tk - ts) * 1e-5f : 0.0f;\n'
         '    }\n'
         '    /* \u2605VER=84 \u503e\u89d2\u53c2\u8003\u5931\u6548\u7d2f\u8ba1\uff08\u6bcf\u5e27\uff09\uff0c\u9a71\u52a8\u78c1\u6682\u505c\u95e8\u3002\n'
         '     * \u95e8\u4fe1\u53f7\u662f\u4e0a\u4e00\u5e27\u7b97\u597d\u7684 gate->ekf_tilt\uff0c0.3s \u7684\u95e8\u4e0d\u5728\u4e4e\u4e00\u5e27\u6ede\u540e\u3002 */\n'
         '    s_grav_ok = gate->ekf_tilt;\n'
         '    if (gate->ekf_tilt) s_tilt_bad_s = 0.0f;\n'
         '    else if (s_tilt_bad_s < 10.0f) s_tilt_bad_s += dt;\n'
         '    s_tilt_inv_ms = s_tilt_bad_s * 1e3f;',
         'tilt timer')

# M7: R 含倾角项
t = sub1(t, '    sig2 = V5F_EKF_MAG_SIG_RAD * V5F_EKF_MAG_SIG_RAD;',
         '    /* \u2605VER=84 R \u542b\u503e\u89d2\u4e0d\u786e\u5b9a\u5ea6\u9879\uff1a\u78c1\u65b0\u606f\u5bf9\u503e\u89d2\u8bef\u5dee\u7684\u7075\u654f\u5ea6 = tan(dip) = 2.08\uff0c\n'
         '     * \u6240\u4ee5\u503e\u89d2\u4e0d\u786e\u5b9a\u5ea6 sigma_tilt \u4f1a\u4ee5 2.08 \u500d\u6df7\u8fdb\u504f\u822a\u65b0\u606f\u91cc\u3002 */\n'
         '    s_tilt_sig_deg = sqrtf(s_P[6][6] + s_P[7][7]) * RAD2DEG;\n'
         '    {\n'
         '        float rb = V5F_EKF_MAG_SIG_RAD * V5F_EKF_MAG_SIG_RAD;\n'
         '        float dl = V5F_EKF_DIP_TAN * (s_tilt_sig_deg * DEG2RAD);\n'
         '        float ra = rb + dl * dl;\n'
         '        if (ra > rb * V5F_EKF_MAG_RSCALE_MAX) ra = rb * V5F_EKF_MAG_RSCALE_MAX;\n'
         '        sig2 = ra;\n'
         '        s_mag_rs = ra / rb;\n'
         '    }',
         'mag R tilt term')

# M7: 开头清 hold 标志
t = sub1(t, '    if (!V5F_EKF_YAW_OBS_EN || !gate->ekf_mag_yaw) return;',
         '    if (!V5F_EKF_YAW_OBS_EN || !gate->ekf_mag_yaw) return;\n'
         '    s_mag_hold = 0u;                    /* \u2605VER=84 */',
         'M7 hold clear')

# M7: 倾角参考失效 -> 暂停
t = sub1(t, '    } else {\n        /* \u2605VER=73 \u53ea\u5728\u65b0\u6837\u672c\u5230\u6765\u7684\u66f4\u65b0\u65f6\u65bd\u52a0\u4e00\u6b21 */',
         '    } else {\n'
         '        /* \u2605VER=84 \u503e\u89d2\u53c2\u8003\u5931\u6548\u8d85\u65f6 -> \u6682\u505c\u78c1\u66f4\u65b0\uff08\u5e76\u8bb0\u95e8\u4f4d\uff09\u3002\n'
         '         * \u4e0d\u505a\u8fd9\u4e2a\uff0c\u78c1\u66f4\u65b0\u7684\u503e\u659c\u884c\u4f1a\u5728\u503e\u89d2\u65e0\u89c2\u6d4b\u65f6\u65e0\u754c\u7d2f\u79ef\uff1b\n'
         '         * \u4eff\u771f\uff1a30 \u5ea6\u78c1\u6fc0\u52b1\u4e0b\u503e\u89d2 52\u5ea6 -> 1.1\u5ea6\u3001\u504f\u822a 20\u5ea6 -> 0.75\u5ea6\u3002 */\n'
         '        if (s_tilt_bad_s > V5F_EKF_MAG_GT_HOLD_S) {\n'
         '            s_mag_hold = 1u;\n'
         '            if (s_rej[3] < 250u) s_rej[3]++;\n'
         '            s_gate_bits |= V5F_EKF_GB_MAGHOLD;\n'
         '            return;\n'
         '        }\n'
         '        /* \u2605VER=73 \u53ea\u5728\u65b0\u6837\u672c\u5230\u6765\u7684\u66f4\u65b0\u65f6\u65bd\u52a0\u4e00\u6b21 */',
         'M7 pause gate')

# 发布
t = sub1(t, '       h->ekf.mag_age_ms = s_mag_age_ms;',
         '       h->ekf.mag_age_ms = s_mag_age_ms;\n'
         '       h->ekf.sigma_tilt_deg = s_tilt_sig_deg;\n'
         '       h->ekf.mag_rs = s_mag_rs;\n'
         '       h->ekf.tilt_inv_ms = s_tilt_inv_ms;\n'
         '       h->ekf.mag_hold = s_mag_hold;\n'
         '       h->ekf.grav_ok = s_grav_ok;\n'
         '       h->ekf.grav_nis = s_nis[3];',
         'publish')
save(P_EKF, t, 'gbk')

# ---------------- 3) SPI_rx.h ----------------
t = load(P_H, 'gbk')
t = sub1(t, '        float    mag_age_ms;',
         '        float    mag_age_ms;\n'
         '        float    sigma_tilt_deg;   /* \u2605VER=84 \u503e\u89d2 1sigma(\u5ea6) */\n'
         '        float    mag_rs;          /* \u2605VER=84 \u78c1 R \u653e\u5927\u500d\u6570 */\n'
         '        float    tilt_inv_ms;     /* \u2605VER=84 \u503e\u89d2\u53c2\u8003\u5931\u6548\u65f6\u957f(ms) */\n'
         '        uint8_t  mag_hold;        /* \u2605VER=84 \u78c1\u88ab\u6682\u505c */\n'
         '        uint8_t  grav_ok;         /* \u2605VER=84 \u91cd\u529b\u89c2\u6d4b\u95e8 */\n'
         '        float    grav_nis;        /* \u2605VER=84 \u91cd\u529b\u89c2\u6d4b NIS */',
         'hdr fields')
save(P_H, t, 'gbk')

# ---------------- 4) SPI_rx.c ----------------
t = load(P_C, 'gbk')
t = sub1(t, '#define JF_CH_NUM     149u', '#define JF_CH_NUM     155u', 'JF_CH_NUM')
t = sub1(t, '        ch[c++] = g_v5f_hold.ekf.mag_age_ms;   /* \u2605VER=83 \u78c1\u6570\u636e\u5e74\u9f84(ms) */',
         '        ch[c++] = g_v5f_hold.ekf.mag_age_ms;   /* \u2605VER=83 \u78c1\u6570\u636e\u5e74\u9f84(ms) */\n'
         '        ch[c++] = g_v5f_hold.ekf.sigma_tilt_deg;   /* \u2605VER=84 \u503e\u89d2 1sigma(\u5ea6) */\n'
         '        ch[c++] = g_v5f_hold.ekf.mag_rs;           /* \u2605VER=84 \u78c1 R \u653e\u5927\u500d\u6570 */\n'
         '        ch[c++] = g_v5f_hold.ekf.tilt_inv_ms;      /* \u2605VER=84 \u503e\u89d2\u53c2\u8003\u5931\u6548\u65f6\u957f(ms) */\n'
         '        ch[c++] = (float)g_v5f_hold.ekf.mag_hold;  /* \u2605VER=84 \u78c1\u88ab\u6682\u505c */\n'
         '        ch[c++] = (float)g_v5f_hold.ekf.grav_ok;   /* \u2605VER=84 \u91cd\u529b\u89c2\u6d4b\u95e8 */\n'
         '        ch[c++] = g_v5f_hold.ekf.grav_nis;         /* \u2605VER=84 \u91cd\u529b\u89c2\u6d4b NIS */',
         'report cols')
save(P_C, t, 'gbk')

# ---------------- 5) jf_load.py ----------------
t = load(P_L, 'utf-8')
t = sub1(t, 'CH_BY_NCH[149] = CH_149\nCH = CH_149',
         'CH_BY_NCH[149] = CH_149\n'
         'CH_155 = dict(CH_149)\n'
         "CH_155.update({'ekf_sigma_tilt_deg': 149, 'ekf_mag_rs': 150, 'ekf_tilt_inv_ms': 151,\n"
         "               'ekf_mag_hold': 152, 'ekf_grav_ok': 153, 'ekf_grav_nis': 154})  # \u2605VER=84\n"
         'CH_BY_NCH[155] = CH_155\n'
         'CH = CH_155', 'CH_155')
save(P_L, t, 'utf-8')

print()
print('fw_tag = (84<<16)|(155<<8)|7 = %d ; 帧长 = %d B' % ((84 << 16) | (155 << 8) | 7, 155*4+6))
