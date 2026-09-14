# -*- coding: utf-8 -*-
"""VER=15 -> 16：把 S1 定成"非 GPS 全融合"，补齐三处：

1) 陀螺离线模型补上**对称交叉项**。旧链 proc_gyro_bias.c 有
   KXY=0.001528 / KXZ=-0.001923 / KYZ=0.000005，EKF 原来只有逐轴标度。
   100 dps 时串扰 0.15~0.19 dps（快转时约 0.2 度倾角误差），是系统性的。
   ★ 加速度敏感度**不加**：本项目实测标量模型不成立（不补 8.649 度 ->
     补了 8.997 度，三条摇晃记录全部变差），常量已被故意置 0。这里的
     "与旧链同一套离线模型"就包含"它关着的我也不开"。

2) 气压观测按设计降到 6.1 Hz。原来每个 EKF 周期（最多 187 Hz）都喂，
   而 press_avg 的噪声相关时间就是 W=32 后的 165 ms -> 58 倍冗余，
   它的 NIS（实测 0.19 vs 维数 1）因此**没有意义**，而且等于偷偷给气压加权。
   降频后相邻观测才近似独立，R=(0.13 m)^2 才是对的 —— 这也正是设计里
   "气压更新 6.1 Hz"的理由。

3) 明确这一阶段的输出边界（注释写进 v5f_tune.h，代码不动）：
   没有 GPS 时只有 p_z + b_baro 这个**和**是可观测的（气压高度），
   p_z 单独报出来的值没有绝对意义；水平位置/速度在运动段不可用。
"""
import shutil

P = r'C:\Users\33\Documents\v2\h415-imu42605-\V5F\User\src\proc_ekf.c'
T = r'C:\Users\33\Documents\v2\h415-imu42605-\V5F\User\inc\v5f_tune.h'

# ---------- 1) 陀螺模型：补对称交叉 ----------
t = open(P, 'rb').read().decode('gbk')
a = """static void raw_w_rads(const volatile v5f_hold_t *h, float *w)
{
    static const float g0[3] = { V5F_GYRO_BIAS_LSB_X, V5F_GYRO_BIAS_LSB_Y, V5F_GYRO_BIAS_LSB_Z };
    uint32_t i;
    for (i = 0u; i < 3u; i++) {
        w[i] = ((float)h->imu.gyro_lsb[i] - g0[i]) / g_v5f_gyro_lsb_per_dps[i] * DEG2RAD;
    }
}"""
b = """/* 陀螺的**完整离线模型**：逐轴标度 -> 对称交叉项。顺序与常量与旧链
 * proc_gyro_bias.c 完全一致（那里是 dv[i] -> 交叉 -> 输出，再在 proc_attitude.c 里
 * 做加速度敏感度）。唯一不含的是**在线牵引出来的零偏**，那只由 EKF 自己的 bg 状态承担。
 * ★ 加速度敏感度按项目结论**不做**：实测标量模型不成立
 *   （不补 8.649 度 -> 补了 8.997 度，三条摇晃记录全部变差），
 *   V5F_GYRO_GSENS_* 已被故意置 0。"与旧链同一套离线模型"就包含"它关着的我也不开"。 */
static void raw_w_rads(const volatile v5f_hold_t *h, float *w)
{
    static const float g0[3] = { V5F_GYRO_BIAS_LSB_X, V5F_GYRO_BIAS_LSB_Y, V5F_GYRO_BIAS_LSB_Z };
    float dv[3];
    uint32_t i;

    for (i = 0u; i < 3u; i++) {
        dv[i] = ((float)h->imu.gyro_lsb[i] - g0[i]) / g_v5f_gyro_lsb_per_dps[i];
    }
    /* 对称交叉：dps_out[i] = dps[i] + sum_{j!=i} k_ij * dps[j]（0.15~0.19%） */
    w[0] = (dv[0] + V5F_GYRO_KXY * dv[1] + V5F_GYRO_KXZ * dv[2]) * DEG2RAD;
    w[1] = (dv[1] + V5F_GYRO_KXY * dv[0] + V5F_GYRO_KYZ * dv[2]) * DEG2RAD;
    w[2] = (dv[2] + V5F_GYRO_KXZ * dv[0] + V5F_GYRO_KYZ * dv[1]) * DEG2RAD;
}"""
assert t.count(a) == 1, t.count(a)
t = t.replace(a, b, 1)

# ---------- 2) 气压降频到 6.1 Hz ----------
a2 = """static float    s_pos_wait;
static uint8_t  s_gps_new;"""
assert t.count(a2) == 1
t = t.replace(a2, """static float    s_pos_wait;
static float    s_baro_wait;          /* 距上次气压观测的秒数（按设计降到 6.1 Hz） */
static uint8_t  s_gps_new;""", 1)

a3 = """        case 19u:
            if (s_prop_ok) ekf_m3_baro(gate);
            if (s_prop_ok) ekf_m7_mag(h, gate);
            s_baro_new = 0u;                   /* 号志在被消费的这一帧才清 */
            break;"""
b3 = """        case 19u:
            /* 气压按设计降到 6.1 Hz：press_avg 的噪声相关时间就是 W=32 后的 165 ms，
             * 原来每个周期（最多 187 Hz）都喂是 58 倍冗余 —— 那会让 R=(0.13 m)^2 名不副实，
             * 也让它 0.19 的 NIS 完全失去意义。降频后相邻观测才近似独立。 */
            if (s_prop_ok && s_baro_wait >= V5F_EKF_BARO_PERIOD_S) {
                ekf_m3_baro(gate);
                s_baro_wait = 0.0f;
            }
            if (s_prop_ok) ekf_m7_mag(h, gate);
            s_baro_new = 0u;                   /* 号志在被消费的这一帧才清 */
            break;"""
assert t.count(a3) == 1
t = t.replace(a3, b3, 1)

a4 = """    s_pos_wait += dt;
    /* 多普勒直流偏置一阶牵引"""
assert t.count(a4) == 1
t = t.replace(a4, """    s_pos_wait  += dt;
    s_baro_wait += dt;
    /* 多普勒直流偏置一阶牵引""", 1)

a5 = """            s_dt_e = 0.0f; s_dt_prev = 0.0f;
            s_prop_row = 0u; s_stage = 0u; s_F_ok = 0u;"""
assert t.count(a5) == 1
t = t.replace(a5, """            s_dt_e = 0.0f; s_dt_prev = 0.0f;
            s_prop_row = 0u; s_stage = 0u; s_F_ok = 0u;
            s_pos_wait = 0.0f; s_baro_wait = 0.0f;""", 1)

assert t.count('/*') == t.count('*/')
shutil.copy2(P, P + '.bak_s1h')
open(P, 'wb').write(t.encode('gbk'))
print('proc_ekf.c: 陀螺补对称交叉项 + 气压降到 6.1 Hz')

# ---------- v5f_tune.h ----------
u = open(T, 'rb').read().decode('gbk')
a = '#define V5F_EKF_BARO_WARMUP_S    5.0f'
assert u.count(a) == 1
u = u.replace(a, a + """
#define V5F_EKF_BARO_PERIOD_S    0.164f  /* 气压观测降频 = 1/6.1，设计指定的带宽。
                                          * press_avg 的噪声相关时间就是 165 ms，
                                          * 不降频就是 58 倍冗余，NIS 会失真 */""", 1)

a = '/* ---- gate_bits 位定义（上报列 104 的原始值）---- */'
DOC = """/* =====================================================================
 * S1 的范围 = **非 GPS 传感器的全融合**（用户 2026-09-15 定的）
 *
 * 没有 GPS 时 7 道观测里活着的是 4 道：M3 气压 / M5 ZUPT / M6 重力倾斜 / M7 磁偏航；
 * M1/M2/M4 与 ENU 原点自然关闭（gate_bits 的 bit0/1/3/10 恒 0）。这不是降级：
 * 同一个滤波器、同一个状态，只是那三道观测缺席。
 *
 * ★ 这一阶段**能承诺**的（VER=15 实测）：
 *     姿态 q（重力系世界四元数）：偏航误差 p50 0.272 度；380 度累计转动跟踪
 *       误差 0.65%（=1.0065 比值）；静止段 0.13 +- 0.22 度。
 *     合加速度 |a_nav|（标准单位 m/s^2）：静止 0.0143。
 *     相对高度：**只有 p_z + b_baro 这个和**（气压高度）是可观测的。
 * ★ 这一阶段**不能承诺**的：
 *     水平位置/速度在运动段不可用 —— 没有绝对基准，ZUPT 只能把它按在"静止"上。
 *     下游必须看 gate_bits 的 bit10（ENU 原点）与 sigma_pos_h / sigma_vel_h，
 *     不许把 p 当绝对坐标用。
 *     p_z 的**单独**读数没有绝对意义（与 b_baro 不可分，要等 M2/GPS 高度）。
 * ===================================================================== */

"""
assert u.count(a) == 1
u = u.replace(a, DOC + a, 1)
assert u.count('#define V5F_FW_VER        15u') == 1
u = u.replace('#define V5F_FW_VER        15u', '#define V5F_FW_VER        16u', 1)
assert u.count('/*') == u.count('*/')
shutil.copy2(T, T + '.bak_s1h')
open(T, 'wb').write(u.encode('gbk'))
print('v5f_tune.h: 降频常量 + S1 范围声明 + VER 16')
print()
print('fw_tag 期望 = %d' % ((16 << 16) | (112 << 8) | 1 | 2 | 4))
