# -*- coding: utf-8 -*-
"""VER=83：磁陈旧补偿改用磁自己的 DRDY 时间戳 + 上报磁数据年龄
   1) proc_ekf.c  补偿角 = int(w dt) over [t_drdy, now]，不再用 ist.hdr.cnt 变化清零
   2) proc_ekf.c  新增 s_mag_age_ms = (IMU tick - mag DRDY tick) 并发布
   3) SPI_rx.h    结构体加 mag_age_ms
   4) SPI_rx.c    JF_CH_NUM 148 -> 149，末尾追加一列
   5) v5f_tune.h  V5F_FW_VER 82 -> 83
   6) jf_load.py  加 CH_149
"""
import os, re, shutil, sys

ROOT = r'C:\Users\33\Documents\v2\h415-imu42605-'
TAG = '.bak_v83'
P_EKF = os.path.join(ROOT, r'V5F\User\src\proc_ekf.c')
P_H = os.path.join(ROOT, r'V5F\User\inc\SPI_rx.h')
P_C = os.path.join(ROOT, r'V5F\User\src\SPI_rx.c')
P_T = os.path.join(ROOT, r'V5F\User\inc\v5f_tune.h')
P_L = os.path.join(ROOT, r'tools\calib\jf_load.py')

changed = []


def load(p, enc):
    return open(p, 'rb').read().decode(enc)


def save(p, text, enc):
    data = text.encode(enc)          # 先编码，失败就不动文件
    if not os.path.exists(p + TAG):
        shutil.copy2(p, p + TAG)
    old = open(p, 'rb').read()
    open(p, 'wb').write(data)
    os.utime(p, None)                # 必须刷新 mtime
    assert open(p, 'rb').read() == data
    changed.append((p, len(old), len(data)))
    print('  OK %-46s %6d -> %6d B' % (os.path.basename(p), len(old), len(data)))


def sub1(text, old, new, what):
    n = text.count(old)
    assert n == 1, '%s: 锚点出现 %d 次(应为1)\n%r' % (what, n, old[:90])
    return text.replace(old, new, 1)


# ---------------- 1) proc_ekf.c ----------------
t = load(P_EKF, 'gbk')

t = sub1(t, 'static uint32_t s_ist_last;',
         'static uint32_t s_ist_last;\n'
         'static float    s_w_int[3];           /* \u2605VER=83 \u6c38\u4e0d\u5f52\u96f6\u7684\u9640\u87ba\u89d2\u589e\u91cf\u79ef\u5206 int(w dt) (rad) */\n'
         'static float    s_w_snap[3];          /* \u2605VER=83 \u78c1 DRDY \u65f6\u523b\u7684\u79ef\u5206\u5feb\u7167 */\n'
         'static uint64_t s_mag_ts_last;        /* \u2605VER=83 \u4e0a\u6b21\u89c1\u5230\u7684\u78c1 DRDY \u65f6\u95f4\u6233 (10ns \u5355\u4f4d) */\n'
         'static float    s_mag_age_ms;         /* \u2605VER=83 \u4e0a\u62a5\uff1a\u78c1\u6570\u636e\u5230\u4f7f\u7528\u65f6\u523b\u7684\u5e74\u9f84 (ms) */',
         'statics')

t = sub1(t, '            s_mag_dth[0] = s_mag_dth[1] = s_mag_dth[2] = 0.0f;',
         '            s_mag_dth[0] = s_mag_dth[1] = s_mag_dth[2] = 0.0f;\n'
         '            s_w_int[0] = s_w_int[1] = s_w_int[2] = 0.0f;\n'
         '            s_w_snap[0] = s_w_snap[1] = s_w_snap[2] = 0.0f;\n'
         '            s_mag_ts_last = 0ULL;',
         'align reset')

OLD_BLOCK = (
    '    /* \u6b64\u5904\u4e0d\u518d\u770b\u8ba1\u6570\u5668\uff0c\u7528"\u81ea\u6d4b\u91cf\u5230\u4f7f\u7528\u7684\u65cb\u8f6c\u89d2"\u6765\u7b97\uff08\u4e0b\u9762\u65b0\u65e7\u5747\u7528\uff09 */\n'
    '    {\n'
    '        uint32_t ic = g_shm ? g_shm->ist.hdr.cnt : 0u;\n'
    '        if (ic != s_ist_last) {\n'
    '            s_ist_last = ic;\n'
    '            s_mag_dth[0] = 0.0f; s_mag_dth[1] = 0.0f; s_mag_dth[2] = 0.0f;\n'
    '        }\n'
    '    }\n'
    '    for (i = 0u; i < 3u; i++) {\n'
    '        s_dth[i] += (w[i] - s_x[IX_BG + i]) * dt;\n'
    '        s_dvb[i] += f[i] * dt;\n'
    '        s_mag_dth[i] += w[i] * dt;      /* \u6b64\u5904\u672a\u51cf bg \u65e0\u6240\u8c13\uff08bg \u53ea\u6709 0.1 dps \u91cf\u7ea7\uff09 */\n'
    '    }')
# 上面那段注释文字随文件而异，改用逐行锚定
lines = t.split('\n')
i0 = next(i for i, l in enumerate(lines) if l.strip().startswith('uint32_t ic = g_shm ? g_shm->ist.hdr.cnt'))
assert lines[i0 - 1].strip() == '{' and lines[i0 - 2].strip().startswith('/*'), lines[i0 - 2][:60]
assert lines[i0 + 5].strip() == '}', lines[i0 + 5]
assert lines[i0 + 6].strip() == 'for (i = 0u; i < 3u; i++) {', lines[i0 + 6]
assert lines[i0 + 9].strip().startswith('s_mag_dth[i] += w[i] * dt;'), lines[i0 + 9]
assert lines[i0 + 10].strip() == '}', lines[i0 + 10]
blk_lo, blk_hi = i0 - 2, i0 + 10          # 含上面那条注释

NEW_BLOCK = [
 '    /* \u2605VER=83 \u78c1\u9648\u65e7\u8865\u507f\u6539\u7528\u78c1\u81ea\u5df1\u7684 DRDY \u65f6\u95f4\u6233\u3002',
 '     * \u65e7\u505a\u6cd5\u5728 ist.hdr.cnt \u53d8\u5316\u65f6\u628a s_mag_dth \u6e05\u96f6\uff0c\u7b49\u4ef7\u4e8e\u628a"\u6211\u53d1\u73b0\u8ba1\u6570\u5668\u53d8\u4e86"',
 '     * \u5f53\u6210\u91c7\u6837\u65f6\u523b\uff1aDRDY \u5230\u53d1\u5e03/\u53d1\u73b0\u7684\u6d41\u6c34\u7ebf\u5ef6\u8fdf\u88ab\u7b97\u6210 0\uff0c\u800c\u8fd9\u4efd\u5ef6\u8fdf\u6b63\u662f',
 '     * "\u62ff\u65e7\u78c1\u6570\u636e\u4fee\u65b0\u9640\u87ba\u4eea"\u7684\u91cf\u3002\u73b0\u6539\u4e3a\uff1as_w_int \u6c38\u4e0d\u5f52\u96f6\u5730\u79ef\u5206 w*dt\uff0c',
 '     * \u78c1\u65f6\u95f4\u6233\u4e00\u53d8\u5c31\u5feb\u7167 s_w_snap\uff0c\u8865\u507f\u89d2 = s_w_int(now) - s_w_snap',
 '     * = \u4ece t_drdy \u5230\u73b0\u5728\u7684\u771f\u5b9e\u8f6c\u89d2\u3002\u540c\u65f6\u4e0a\u62a5\u78c1\u6570\u636e\u5e74\u9f84\u7528\u4e8e\u5224\u5b9a\u5ef6\u8fdf\u91cf\u7ea7\u3002 */',
 '    {',
 '        uint32_t ic = g_shm ? g_shm->ist.hdr.cnt : 0u;',
 '        uint64_t ts = h->mag.fresh.drdy_tick;',
 '        if (ic != s_ist_last || ts != s_mag_ts_last) {',
 '            s_ist_last = ic;',
 '            s_mag_ts_last = ts;',
 '            for (i = 0u; i < 3u; i++) s_w_snap[i] = s_w_int[i];',
 '        }',
 '        s_mag_age_ms = (ts != 0ULL && tk > ts) ? (float)(tk - ts) * 1e-5f : 0.0f;',
 '    }',
 '    for (i = 0u; i < 3u; i++) {',
 '        s_dth[i] += (w[i] - s_x[IX_BG + i]) * dt;',
 '        s_dvb[i] += f[i] * dt;',
 '        s_w_int[i] += w[i] * dt;',
 '        s_mag_dth[i] = s_w_int[i] - s_w_snap[i];',
 '    }',
]
lines[blk_lo:blk_hi + 1] = NEW_BLOCK
t = '\n'.join(lines)

t = sub1(t, '       h->ekf.mag_cmp_mhn = s_mag_cmp_mhn;',
         '       h->ekf.mag_cmp_mhn = s_mag_cmp_mhn;\n'
         '       h->ekf.mag_age_ms = s_mag_age_ms;',
         'publish')

save(P_EKF, t, 'gbk')

# ---------------- 2) SPI_rx.h ----------------
t = load(P_H, 'gbk')
t = sub1(t, '        float    mag_cmp_mhn;',
         '        float    mag_age_ms;    /* \u2605VER=83 \u78c1\u6570\u636e\u5e74\u9f84(ms) = IMU tick - mag DRDY tick */\n'
         '        float    mag_cmp_mhn;', 'SPI_rx.h 结构体')
save(P_H, t, 'gbk')

# ---------------- 3) SPI_rx.c ----------------
t = load(P_C, 'gbk')
t = sub1(t, '#define JF_CH_NUM     148u', '#define JF_CH_NUM     149u', 'JF_CH_NUM')
i = t.index('ch[c++] = g_v5f_hold.ekf.mag_cmp_mhn;')
e = t.index('\n', i)
t = t[:e + 1] + '        ch[c++] = g_v5f_hold.ekf.mag_age_ms;   /* \u2605VER=83 \u78c1\u6570\u636e\u5e74\u9f84(ms) */\n' + t[e + 1:]
save(P_C, t, 'gbk')

# ---------------- 4) v5f_tune.h ----------------
t = load(P_T, 'gbk')
t = sub1(t, '#define V5F_FW_VER        82u', '#define V5F_FW_VER        83u', 'FW_VER')
save(P_T, t, 'gbk')

# ---------------- 5) jf_load.py ----------------
t = load(P_L, 'utf-8')
t = sub1(t, 'CH_BY_NCH[148] = CH_148\nCH = CH_148',
         'CH_BY_NCH[148] = CH_148\n'
         'CH_149 = dict(CH_148)\n'
         "CH_149.update({'ekf_mag_age_ms': 148})   # \u2605VER=83 \u78c1\u6570\u636e\u5e74\u9f84(ms)\n"
         'CH_BY_NCH[149] = CH_149\n'
         'CH = CH_149', 'jf_load CH_149')
save(P_L, t, 'utf-8')

print()
print('fw_tag 期望 = (83<<16)|(149<<8)|7 = %d' % ((83 << 16) | (149 << 8) | 7))
print('帧长 = 149*4+6 = %d B' % (149 * 4 + 6))
