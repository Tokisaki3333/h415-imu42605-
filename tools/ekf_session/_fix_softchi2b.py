# -*- coding: utf-8 -*-
"""只做 proc_ekf.c 部分（v5f_tune.h 上一次已写好）。"""
import shutil

P = r'C:\Users\33\Documents\v2\h415-imu42605-\V5F\User\src\proc_ekf.c'
t = open(P, 'rb').read().decode('gbk')


def sub(a, b, nm):
    global t
    n = t.count(a)
    assert n == 1, '锚点 %s 匹配 %d 次' % (nm, n)
    t = t.replace(a, b, 1)


sub("static uint8_t  s_sat;                /* 本周期见过加计削顶帧 */",
    "static uint8_t  s_sat;                /* 本周期见过加计削顶帧 */\n"
    "static uint8_t  s_chi2_soft;          /* 本周期有观测被软加权（记 bit9）*/\n"
    "static float    s_mn_lp;              /* mag_norm 的稳健基线（只跟看起来干净的样本）*/\n"
    "static uint8_t  s_mn_seen;", 'vars')

sub("""    if (nis > nis_max) {
        if (rej && *rej < 250u) (*rej)++;
        if (!rej || *rej < V5F_EKF_CHI2_ESCAPE) return 1u;
        /* 逃脱：连续被剔够多次 -> 用放大 ESC_R 倍的 R 放行一次。
         * 等价于把 Si 缩小 ESC_R 倍（K = PHt·Si，P 修正 = K·S·K' 同比例缩小）。 */
        for (i = 0u; i < m; i++) {
            for (j = 0u; j < m; j++) Si[i][j] *= (1.0f / V5F_EKF_CHI2_ESC_R);
        }
    }
    if (rej) *rej = 0u;""",
    """    if (nis > nis_max) {
        /* 软加权，不硬剔：Si /= (nis/nis_max)，等价 R_eff = R*(nis/nis_max)。
         * 为什么不能硬剔：剔一次不改 P -> S 不变 -> nis 永远超限 -> 环永久锁死
         * （实测三次；最近一次是 VER=20 里 nis_baro p50=11.92、53.5% 的帧被剔、
         *  p_z 漂到 ±287 m 而 b_baro 冻在 0）。极端野值权重趋 0，仍然挡得住。 */
        float sc = nis / nis_max;
        if (sc > V5F_EKF_CHI2_SC_MAX) sc = V5F_EKF_CHI2_SC_MAX;
        for (i = 0u; i < m; i++) {
            for (j = 0u; j < m; j++) Si[i][j] /= sc;
        }
        s_chi2_soft = 1u;
        if (rej && *rej < 250u) (*rej)++;
    } else if (rej) {
        *rej = 0u;
    }""", 'soft')

sub(" *   返回 0 = 已更新；1 = 内层 chi2 剔除；2 = 数值失败",
    """ *   返回 0 = 已更新（**含被软加权的情况**，观测确实进去了、只是权重小）；2 = 数值失败。""",
    'retdoc')

sub("""            if (s_sat) { s_gate_bits |= V5F_EKF_GB_SAT; s_sat = 0u; }""",
    """            if (s_sat) { s_gate_bits |= V5F_EKF_GB_SAT; s_sat = 0u; }
            s_chi2_soft = 0u;""", 'chi2clr')

sub("""            if (s_origin_ok) s_gate_bits |= V5F_EKF_GB_ORIGIN;""",
    """            if (s_origin_ok) s_gate_bits |= V5F_EKF_GB_ORIGIN;
            if (s_chi2_soft) s_gate_bits |= V5F_EKF_GB_CHI2;   /* 本周期有观测被软加权 */""",
    'chi2bit')

sub("""    gate->ekf_mag_yaw = (uint8_t)((h->mag.ok != 0u) && (V5F_EKF_YAW_OBS_EN != 0u));""",
    """    {
        /* 磁力计模长：与**稳健基线**比，而不是与常数 1.0 比。
         * 实测本场次 mag_norm p50=0.9321（场次硬铁漂移 6.8%），|mag_norm-1| 的
         * p50=0.0977 正好压在 V5F_MAG_ERR_LIM=0.10 上 -> 门开 53.7% 来回抖。
         * 与 ZUPT 那个 a_lin 是同一个病：阈值压在统计量中位数上。
         * 基线只在"看起来干净"（偏离 < 门限）时才跟，所以外部磁干扰（局部突变）
         * 不会被基线吃掉，而场次偏差（常数）会被跟上。 */
        float dev;
        if (!s_mn_seen) { s_mn_lp = h->mag.mag_norm; s_mn_seen = 1u; }
        dev = fabsf(h->mag.mag_norm - s_mn_lp);
        if (dev < V5F_EKF_MAG_NORM_DEV) {
            s_mn_lp += (h->mag.mag_norm - s_mn_lp) * V5F_EKF_MAG_NORM_ALPHA;
        }
        gate->ekf_mag_yaw = (uint8_t)((dev < V5F_EKF_MAG_NORM_DEV)
                                      && (V5F_EKF_YAW_OBS_EN != 0u));
    }""", 'maggate')

# st == 1 的分支清掉（软加权现在返回 0）
for old, new in [
    ("""        if      (st == 0u) s_gate_bits |= V5F_EKF_GB_TILT;
        else if (st == 1u) s_gate_bits |= V5F_EKF_GB_CHI2;""",
     """        if (st == 0u) s_gate_bits |= V5F_EKF_GB_TILT;"""),
    ("""        if      (st == 0u) s_gate_bits |= V5F_EKF_GB_MAG;
        else if (st == 1u) s_gate_bits |= V5F_EKF_GB_CHI2;""",
     """        if (st == 0u) s_gate_bits |= V5F_EKF_GB_MAG;"""),
    ("""        if      (st == 0u) s_gate_bits |= V5F_EKF_GB_TILT;
            else if (st == 1u) s_gate_bits |= V5F_EKF_GB_CHI2;""",
     """        if (st == 0u) s_gate_bits |= V5F_EKF_GB_TILT;"""),
]:
    if t.count(old) == 1:
        t = t.replace(old, new, 1)
        print('  调用点已改')

assert 'st == 1u' not in t, '还有 st==1 的分支'
assert t.count('/*') == t.count('*/')
shutil.copy2(P, P + '.bak_s1m')
open(P, 'wb').write(t.encode('gbk'))
print('proc_ekf.c: 软加权 + 磁基线门 完成')

import re
T = r'C:\Users\33\Documents\v2\h415-imu42605-\V5F\User\inc\v5f_tune.h'
ut = open(T, 'rb').read().decode('gbk')
print('VER =', re.search(r'#define V5F_FW_VER\s+(\d+)u', ut).group(1))
S = r'C:\Users\33\Documents\v2\h415-imu42605-\V5F\User\src\SPI_rx.c'
st = open(S, 'rb').read().decode('gbk')
nch = int(re.search(r'#define JF_CH_NUM\s+(\d+)u', st).group(1))
print('fw_tag 期望 = %d' % ((21 << 16) | (nch << 8) | 1 | 2 | 4))
