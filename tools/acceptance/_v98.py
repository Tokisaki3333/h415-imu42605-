# -*- coding: utf-8 -*-
"""VER=98: 磁观测 R 重定 (基准 20->3.5°, 膨胀项 sigma_tilt 输入钳 10°)
   GBK 字节级替换; 备份 .bak_v98; 写后行级校验; 同步 sim 并编译验证。"""
import os, re, shutil

def patch(path, subs, bak):
    raw = open(path, 'rb').read()
    txt = raw.decode('gbk')                      # 严格解码
    eol = '\r\n' if b'\r\n' in raw else '\n'
    if not os.path.exists(bak):
        shutil.copy2(path, bak); print('备份 ->', bak)
    st = os.stat(path)
    for name, pat, rep, cnt in subs:
        txt, n = re.subn(pat, rep.replace('\n', eol), txt, count=cnt)
        assert n == cnt, '%s: %s 命中 %d' % (path, name, n)
        print('  [ok] %-16s 命中 %d' % (name, n))
    open(path, 'wb').write(txt.encode('gbk'))
    os.utime(path, (st.st_atime, st.st_mtime))
    a = raw.decode('gbk').split(eol); b = txt.split(eol)
    lost = [x for x in a if x not in b]
    print('  %s: %d -> %d 行; 消失旧行 %s' % (os.path.basename(path), len(a), len(b), [x[:55] for x in lost] or '无'))
    return len(a), len(b)

T = r'h415-imu42605-\V5F\User\inc\v5f_tune.h'
E = r'h415-imu42605-\V5F\User\src\proc_ekf.c'

NEWC = """/* \u2605VER=98 重定 R 基准与膨胀项输入(口径换了):
 *  旧口径(VER=86, 剧烈运动里的 thm-旧姿态偏航) 混杂了"旧姿态自身误差 + 手部/环境对场的扰动",
 *  因此取到 p99 = 20 度。VER=98 改用**干净慢转**标定: 朝北 4x90 度、低干扰、以旧链 att.q 为量尺,
 *  地磁相对偏航误差 rms 2.81 / max 4.2 度 -> 基准取 3.5 度。
 *  剧烈运动下地磁自身误差会涨到 4~14 度, 这部分由膨胀项覆盖: sigma_tilt 输入钳到
 *  V5F_EKF_MAG_TILT_CAP_DEG=10 度(动态段 EKF 实际倾角误差实测 8~10 度, 而 sigma_tilt 被
 *  整流项 Q 吹到 45~68 度) -> R 由 96~143 度降到 <=21.1 度, 磁增益由 0.0063 回到
 *  MAG_K_MAX=0.015 钳位(=2.4 倍权限)。不改门、不加死区、EKF 结构不变。 */
#define V5F_EKF_MAG_R_DEG       3.5f
#define V5F_EKF_MAG_TILT_CAP_DEG 10.0f   /* VER=98 膨胀项 sigma_tilt 输入上限(度) */"""

patch(T, [
    ('VER 97->98', r'(#define V5F_FW_VER\s+)97u', r'\g<1>98u', 1),
    ('R 基准与上限', r'#define V5F_EKF_MAG_R_DEG\s+20\.0f', NEWC, 1),
], T + '.bak_v98')

DL = '        float dl = V5F_EKF_DIP_TAN * (s_tilt_sig_deg * DEG2RAD);'
NEWDL = """        float stc = s_tilt_sig_deg;                    /* VER=98 膨胀项输入钳位 */
        float dl;
        if (stc > V5F_EKF_MAG_TILT_CAP_DEG) stc = V5F_EKF_MAG_TILT_CAP_DEG;
        dl = V5F_EKF_DIP_TAN * (stc * DEG2RAD);"""
patch(E, [('dl 加钳位', re.escape(DL), NEWDL, 1)], E + '.bak_v98')

# 回读校验
L = open(E, 'rb').read().decode('gbk').split('\n')
i = next(i for i, x in enumerate(L) if 'VER=98 膨胀项输入钳位' in x)
print('\nproc_ekf.c 回读:'); [print('  %4d %s' % (k+1, L[k].rstrip())) for k in range(i-5, i+4)]
L2 = open(T, 'rb').read().decode('gbk').split('\n')
j = next(j for j, x in enumerate(L2) if 'V5F_EKF_MAG_R_DEG' in x and '#define' in x)
print('v5f_tune.h 回读:'); [print('  %4d %s' % (k+1, L2[k].rstrip()[:110])) for k in range(j-2, j+3)]
