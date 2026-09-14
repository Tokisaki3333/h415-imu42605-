# -*- coding: utf-8 -*-
"""VER=20 -> 21：两个"阈值压在统计量中位数上"的结构性毛病。

A) chi2 由**硬剔除**改成**软加权**（R_eff = R * nis/nis_max）。
   硬剔的机理缺陷：剔一次不改 P -> S 不变 -> nis 永远超限 -> 环被永久锁死。
   实测三次：气压 nis 恒 180（b_baro 差 13.5 m）、偏航差 80 度时 nis 恒 1.3e4、
   VER=20 这份里 nis_baro p50=11.92 > 10.83，53.5% 的帧被剔，p_z 因此漂到 ±287 m
   而 b_baro 冻在 0。软加权保留"挡野值"（极端 nis 权重趋 0）但永不锁死。
   —— 原来那个"连续剔 N 次就放行"的逃脱阀因此可以退休（只留计数供上报）。

B) mag 门改成"相对**稳健基线**的偏离"。mag_norm 实测 p50=0.9321（本场次硬铁漂移
   6.8%），于是 |mag_norm-1| 的 p50=0.0977 正好压在 V5F_MAG_ERR_LIM=0.10 上，
   门开 53.7% 来回抖。绝对模长门对"场次间的硬铁漂移"没有免疫力。
   新判据：与一个只跟踪"看起来干净"的样本的慢基线比，偏离 <10% 即可信 ——
   外部磁干扰是**局部突变**，而场次偏差是常数，这样两者就分开了。
"""
import re
import shutil

P = r'C:\Users\33\Documents\v2\h415-imu42605-\V5F\User\src\proc_ekf.c'
T = r'C:\Users\33\Documents\v2\h415-imu42605-\V5F\User\inc\v5f_tune.h'

u = open(T, 'rb').read().decode('gbk')
a = '#define V5F_EKF_CHI2_ESCAPE     8u'
assert u.count(a) == 1
u = u.replace(a, """/* ---- chi2：软加权，不再硬剔除，也不再需要逃脱阀 ----
 * 硬剔的机理缺陷：剔一次不改 P -> S 不变 -> nis 永远超限 -> 环永久锁死。
 * 实测三次（气压 nis 恒 180 / 偏航差 80 度时 nis 恒 1.3e4 / VER=20 里 nis_baro
 * p50=11.92 而 53.5% 的帧被剔、p_z 漂到 ±287 m）。
 * 现在：nis > 上限时把 Si 缩小 (nis/上限) 倍，等价于 R_eff = R*(nis/上限) ——
 * 极端野值的权重趋 0（仍然挡得住），但永不锁死。 */
#define V5F_EKF_CHI2_SC_MAX      1000.0f  /* 软加权倍率上限（权重不低于 1/1000）*/
/* 以下两个是旧逃脱阀的常量，已无用，留名以防旧记录对不上 */
#define V5F_EKF_CHI2_ESCAPE     8u""", 1)
a = '#define V5F_EKF_ZUPT_AMAG_TOL    0.03f'
assert u.count(a) == 1
u = u.replace(a, """/* 磁力计模长的**基线偏离**门限。绝对模长门（V5F_MAG_ERR_LIM=0.10）对场次间的
 * 硬铁漂移没有免疫力：实测本场次 mag_norm p50=0.9321（漂 6.8%），
 * |mag_norm-1| 的 p50=0.0977 正好压在 0.10 上，门来回抖（开 53.7%）。
 * 外部磁干扰是**局部突变**，场次偏差是常数 —— 与稳健基线比就把两者分开了。 */
#define V5F_EKF_MAG_NORM_DEV     0.10f
#define V5F_EKF_MAG_NORM_ALPHA   0.002f  /* 基线跟踪系数（约 500 周期 ≈ 1.4 s）*/

#define V5F_EKF_ZUPT_AMAG_TOL    0.03f""", 1)
assert u.count('#define V5F_FW_VER        20u') == 1
u = u.replace('#define V5F_FW_VER        20u', '#define V5F_FW_VER        21u', 1)
assert u.count('/*') == u.count('*/')
shutil.copy2(T, T + '.bak_s1m')
open(T, 'wb').write(u.encode('gbk'))
print('v5f_tune.h: 软加权 + 磁基线 + VER 21')

t = open(P, 'rb').read().decode('gbk')


def sub(a, b, nm):
    global t
    n = t.count(a)
    assert n == 1, '锚点 %s 匹配 %d 次' % (nm, n)
    t = t.replace(a, b, 1)


sub("static uint8_t  s_chi2_soft;", "static uint8_t  s_chi2_soft;", 'noop') if 's_chi2_soft' in t else None
sub("static uint8_t  s_sat;                /* 本周期见过加计削顶帧 */",
    "static uint8_t  s_sat;                /* 本周期见过加计削顶帧 */\n"
    "static uint8_t  s_chi2_soft;          /* 本周期有观测被软加权（记 bit9）*/\n"
    "static float    s_mn_lp;              /* mag_norm 的稳健基线（只用看起来干净的样本跟）*/\n"
    "static uint8_t  s_mn_seen;", 'vars')

# A) 软加权
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
         * （实测三次，最近一次是 VER=20 里 nis_baro p50=11.92、53.5% 被剔、
         *  p_z 漂到 ±287 m 而 b_baro 冻在 0）。极端野值的权重趋 0，仍然挡得住。 */
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

# 返回约定注释
sub(""" *   返回 0 = 已更新；1 = 内层 chi2 剔除；2 = 数值失败""",
    """ *   返回 0 = 已更新（含被软加权的情况）；2 = 数值失败""", 'retdoc')
sub(""" *   返回：0 = 已更新；1 = 内层 chi2 剔除；2 = 数值失败（不更新）""",
    """ *   返回：0 = 已更新（含被软加权）；2 = 数值失败（不更新）
 *   —— 软加权不再作为"没更新"上报：观测确实进去了，只是权重小。""", 'retdoc2')

# 调用点：st==1 的分支不再存在，改成只看 st==0
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
        print('  调用点已改:', old.strip()[:40])

# bit9 从标志取
sub("""            if (s_sat) { s_gate_bits |= V5F_EKF_GB_SAT; s_sat = 0u; }""",
    """            if (s_sat) { s_gate_bits |= V5F_EKF_GB_SAT; s_sat = 0u; }
            s_chi2_soft = 0u;""", 'chi2clr')
sub("""            if (s_aligned)   s_gate_bits |= V5F_EKF_GB_ALIGN;
            if (s_origin_ok) s_gate_bits |= V5F_EKF_GB_ORIGIN;""",
    """            if (s_aligned)   s_gate_bits |= V5F_EKF_GB_ALIGN;
            if (s_origin_ok) s_gate_bits |= V5F_EKF_GB_ORIGIN;
            if (s_chi2_soft) s_gate_bits |= V5F_EKF_GB_CHI2;   /* 本周期有观测被软加权 */""", 'chi2bit')

# B) 磁门：稳健基线
sub("""    gate->ekf_mag_yaw = (uint8_t)((h->mag.ok != 0u) && (V5F_EKF_YAW_OBS_EN != 0u));""",
    """    {
        /* 磁力计模长：与**稳健基线**比，而不是与常数 1.0 比。
         * 实测本场次 mag_norm p50=0.9321（场次硬铁漂移 6.8%），
         * |mag_norm-1| 的 p50=0.0977 正好压在 V5F_MAG_ERR_LIM=0.10 上，
         * 门开 53.7% 来回抖 —— 与 ZUPT 那个 a_lin 是同一个病：阈值压在统计量中位数上。
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

assert t.count('/*') == t.count('*/')
assert 'st == 1u' not in t, '还有 st==1 的分支没清'
shutil.copy2(P, P + '.bak_s1m')
open(P, 'wb').write(t.encode('gbk'))
print('proc_ekf.c: 软加权 + 磁基线门')

# 工具：NIS 基准列随 NCH 走
for p in [r'C:\Users\33\Documents\v2\tools\calib\val_ekf.py',
          r'C:\Users\33\Documents\v2\tools\calib\seg_gates.py']:
    s = open(p, encoding='utf-8').read()
    s = s.replace('a[:, 107+i]', 'a[:, NIS0+i]')
    s = s.replace('a[:, 107 + i]', 'a[:, NIS0 + i]')
    if 'NIS0' in s and 'NIS0 =' not in s:
        s = s.replace("a = np.fromfile(P, dtype='<f4').reshape(-1, NCH)",
                      "NIS0 = NCH - 5          # nis[0..4] 永远在最后 5 列\n"
                      "a = np.fromfile(P, dtype='<f4').reshape(-1, NCH)", 1)
    open(p, 'w', encoding='utf-8', newline='\n').write(s)
print('val_ekf/seg_gates: NIS 基准列改成 NCH-5')
print()
print('fw_tag 期望 = %d' % ((21 << 16) | (113 << 8) | 1 | 2 | 4))
