# -*- coding: utf-8 -*-
"""补上真正没写进去的两处（上次断言失败在写文件之前）：软加权 + 磁稳健基线门。
★ 教训：脚本里"先全部改内存、最后一次写文件"的写法，中间任何断言失败都会让
   我以为改了其实没改。这次每步都 veriy。"""
import shutil

P = r'C:\Users\33\Documents\v2\h415-imu42605-\V5F\User\src\proc_ekf.c'
t = open(P, 'rb').read().decode('gbk')
print('改前检查: 软加权=%s 磁基线=%s 硬剔escape=%s'
      % ('Si[i][j] /= sc' in t, 's_mn_lp' in t, 'V5F_EKF_CHI2_ESC_R' in t))


def sub(a, b, nm):
    global t
    n = t.count(a)
    assert n == 1, '锚点 %s 匹配 %d 次' % (nm, n)
    t = t.replace(a, b, 1)
    print('  ok:', nm)


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
    " *   返回 0 = 已更新（**含被软加权**：观测确实进去了、只是权重小）；2 = 数值失败。",
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

assert 'st == 1u' not in t
assert t.count('/*') == t.count('*/')
# 写前最后核一遍，写后立刻回读
shutil.copy2(P, P + '.bak_s1n')
open(P, 'wb').write(t.encode('gbk'))
chk = open(P, 'rb').read().decode('gbk')
for k, v in [('软加权', 'Si[i][j] /= sc' in chk),
             ('磁基线', 's_mn_lp' in chk),
             ('chi2 标志清零', 's_chi2_soft = 0u' in chk),
             ('chi2 上报位', 'if (s_chi2_soft) s_gate_bits' in chk),
             ('escape 已移除', 'V5F_EKF_CHI2_ESC_R' not in chk),
             ('无 return 1u 硬剔', 'return 1u;' not in chk)]:
    print('  %-14s %s' % (k, v))
    assert v, k
print('proc_ekf.c 两处补齐，回读校验通过')
