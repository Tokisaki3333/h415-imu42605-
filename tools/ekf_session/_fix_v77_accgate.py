# -*- coding: utf-8 -*-
"""
VER=77：给新观测加"加计=重力"门

VER=76 的观测把**加计实测重力方向**当水平面法向（这是它能做到"倾角不干涉偏航"
的唯一办法）。但倾角补偿式磁航向的前提就是**加计此刻测的确实是重力**。
VER=76 录制实测（44.26 s）：
  corr(||a|-1|, |新息|) = +0.478
  ||a|-1| < 0.01 g : 新息 p50 0.34 度 p90 1.39 度   (|w| 0.3 dps)
  ||a|-1| > 1.0  g : 新息 p50 61.16 度              (|w| 1650 dps)
  加计最大 16.76 g —— |w|=1966 dps 时离心加速度把加计打饱和, a_up 是垃圾,
  观测被污染成 61 度。这不是几何退化门(BH_MIN 那种不可能触发的), 是物理上
  真实成立的门: 没有可信的重力参考就无法在重力法平面里定义磁北。
干净帧(||a|-1|<0.01g, 占 28.5%)上: 静止段 |新息| p50 0.299 度 p90 0.890 度。

改动：M7 在算出比力模长后加一道判据，用的是 M6 倾斜观测同一个容差常量
      V5F_EKF_TILT_AMAG_TOL (|a|^2 偏离 1, 约等于 |a| 偏离 3%)，并复用
      h->imu.acc_valid，与 M6 的 `gate->ekf_tilt` 判据口径一致。
      不满足 -> 本周期不施加磁观测（s_rej 记数），但不影响任何其它量。
版本 76 -> 77。
"""
import os, shutil
ENC = 'gbk'
ROOT = 'h415-imu42605-'
TUNE = os.path.join(ROOT, r'V5F\User\inc\v5f_tune.h')
EKF  = os.path.join(ROOT, r'V5F\User\src\proc_ekf.c')

def load(p):
    with open(p, 'rb') as f: return f.read().decode(ENC)
def save(p, s, tag):
    d = s.encode(ENC)
    shutil.copy2(p, p + '.bak_' + tag)
    with open(p, 'wb') as f: f.write(d)
    with open(p, 'rb') as f: assert f.read().decode(ENC) == s
    os.utime(p, None)
    return len(d)
def sub1(s, a, b, what):
    n = s.count(a); assert n == 1, '[%s] 命中 %d 次' % (what, n)
    return s.replace(a, b, 1)

e = load(EKF)
e = sub1(e,
    '    raw_f_mps2(h, fb);                          /* 只用原始 LSB + 离线标定常量 */\n'
    '    an = sqrtf(fb[0]*fb[0] + fb[1]*fb[1] + fb[2]*fb[2]);\n'
    '    if (an < 1e-6f) return;\n',
    '    raw_f_mps2(h, fb);                          /* 只用原始 LSB + 离线标定常量 */\n'
    '    an = sqrtf(fb[0]*fb[0] + fb[1]*fb[1] + fb[2]*fb[2]);\n'
    '    if (an < 1e-6f) return;\n'
    '    /* ★VER=77 "加计=重力"门：本观测把加计测得的重力方向当水平面法向，\n'
    '     * 所以前提是加计此刻真的只测重力。实测 VER=76：||a|-1| 与新息强相关\n'
    '     * (corr +0.478)，干净档(<0.01g)新息 p50 0.34 度，而 ||a|-1|>1g 时\n'
    '     * (|w|=1650 dps、加计到 16.76 g 饱和) 新息 p50 61 度 —— 那不是航向\n'
    '     * 误差，是 a_up 变成垃圾。判据口径与 M6 的 gate->ekf_tilt 一致。 */\n'
    '    {\n'
    '        float ag = an / V5F_EKF_G_MPS2;\n'
    '        if (!h->imu.acc_valid || fabsf(ag*ag - 1.0f) >= V5F_EKF_TILT_AMAG_TOL) {\n'
    '            if (s_rej[3] < 250u) s_rej[3]++;\n'
    '            s_gate_bits |= V5F_EKF_GB_CHI2;\n'
    '            return;\n'
    '        }\n'
    '    }\n',
    'gate77')
n_e = save(EKF, e, 'v77')

t = load(TUNE)
t = sub1(t, '#define V5F_FW_VER        76u', '#define V5F_FW_VER        77u', 'VER')
n_t = save(TUNE, t, 'v77')

# ---- 自检 ----
t2, e2 = load(TUNE), load(EKF)
assert '#define V5F_FW_VER        77u' in t2
i = e2.find('static void ekf_m7_mag'); j = e2.find('/* M1 水平位置', i)
blk = e2[i:j]
ob, cb = e2.count('{'), e2.count('}')
oc, cc = e2.count('/*'), e2.count('*/')
print('proc_ekf.c  { } %d/%d 差 %+d   /* */ %d/%d 差 %+d' % (ob, cb, ob-cb, oc, cc, oc-cc))
assert ob == cb and oc == cc
for nm, ok in (('加计门已插入', blk.count('V5F_EKF_TILT_AMAG_TOL') == 1),
               ('acc_valid 判据', 'h->imu.acc_valid' in blk),
               ('仍是 1 维观测', blk.count('ekf_update(R, 1u, r,') == 1),
               ('H 仍只有偏航项', blk.count('s_H[0][8]') == 1),
               ('mask 0x0100', blk.count('0x0100u') == 1),
               ('BH_MIN 未复活', 'V5F_EKF_MAG_BH_MIN' not in blk),
               ('DEAD 未复活', 'V5F_EKF_MAG_DEAD_DEG' not in blk)):
    print('  [%s] %s' % ('OK' if ok else '!!', nm))
    assert ok, nm
print('v5f_tune.h %d 字节 ; proc_ekf.c %d 字节' % (n_t, n_e))
print('PASS: VER=77 已写入')
