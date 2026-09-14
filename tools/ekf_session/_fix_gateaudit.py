# -*- coding: utf-8 -*-
"""VER=18 -> 19：把 7 道门按"判据是否属于这道观测 / 是否自环 / 是否在最需要时关掉 /
是否有死判据"逐条审一遍，修掉同类失误 4 处，并把 2 处有意的偏离写清楚。

审计结论（逐道门）：
  M1 GPS 水平位置 — 基本对。两处问题：
     (a) 违反了本项目自己的"判空查位、勿用值判空"纪律：sat_num / hdop 直接用值判，
         没查 SHM_GGA_SV / SHM_GGA_HDOP。字段一旦没解析出来就是 0 -> 门永久关闭
         -> GPS 整路静默失效。修：条件挂在对应的 flags 位上。
     (b) fixq >= 1（设计写 {2,4}）：这是**有意偏离**，因为近场全是单点定位
         fixq=1，按设计写就永远打不开。已在注释里标死。
  M2 GPS 高度 — **与气压同一个错**：误差是慢随机游走（同一条接收机解算，
     水平实测 lag1 自相关 0.999），却按每个 GPS 样本喂进来而**没有降频**，
     R=(5.5 m)^2 名不副实、等于偷偷给垂直通道超配权重。修：与 M1 同周期降频。
  M3 气压 — 降频已修。"|dh| < 50 m"这条原来是**死判据**：s_baro_h_last 只在跳变
     小时更新，而真正被用的 s_baro_h 无条件赋值 —— 等于从来没拒绝过任何样本。
     修：真的拒绝；并补上设计要求的"温度稳定"（相邻样本 |dT| 上限）。
  M4 多普勒速度 — 对。它**不需要**降频：多普勒是每历元独立的白噪声
     （与位置误差的慢游走不同），R=(0.15 m/s)^2 成立。这条要写清楚，免得以后误改。
  M5 软 ZUPT — 最大的隐患是**无 GPS 时退化成纯 is_static**：speed 那一路被
     `!(flags & SHM_RMC_SPEED)` 旁路掉，而等效原理下匀速平动对陀螺/加计不可见
     -> 走动时也会 ZUPT。补一条**外生**的交流能量判据（实测静止 e_ac 0.048~0.057，
     运动 0.37~84），它只可能让门更保守。
  M6 重力/倾斜 — 对，且要说明一处有意偏离：设计写"长窗净|w|<2 dps"（256 ms），
     我用的是 stat.level_dps（128 帧 ≈ 16 ms）。短窗其实**更对**：要排除转动是因为
     离心项 a_c=w^2*r 会淹没重力方向，而 a_c 只在高 w 时才显著，16 ms 窗抓的正是它。
  M7 磁偏航 — 上一版已修（mag.ok + 自适应 R + 样本陈旧补偿）。

自环检查：七道门的判据全部只用外生量（原始 LSB、离线常量、stat、acc_valid、
mag.ok、GPS 报文质量位）。a_lin 是旧链的量，但它**外生于** EKF，而 EKF 不喂回
旧链，所以不构成环。M7 的自适应 R 用了 P，那是更新内部的 R，不是门。
"""
import shutil

P = r'C:\Users\33\Documents\v2\h415-imu42605-\V5F\User\src\proc_ekf.c'
T = r'C:\Users\33\Documents\v2\h415-imu42605-\V5F\User\inc\v5f_tune.h'

u = open(T, 'rb').read().decode('gbk')
a = '#define V5F_EKF_BARO_STEP_MAX_M  50.0f'
assert u.count(a) == 1
u = u.replace(a, a + """
#define V5F_EKF_BARO_DT_MAX_C    1.0f    /* 相邻气压样本的温差上限（设计的"温度在窗口内稳定"）。
                                          * BMP388 的 die 温度变化很慢，1 度已经是很宽的限 */""", 1)
a = '#define V5F_EKF_ZUPT_ALIN2       (0.05f * 0.05f)   /* |a_lin| < 0.05 g（a_lin 单位就是 g）*/'
assert u.count(a) == 1
u = u.replace(a, a + """
#define V5F_EKF_ZUPT_EAC_MAX     0.20f   /* ZUPT 追加的**交流能量**上限 dps^2（外生）。
                                          * 为什么必须有：没有 GPS 时 speed 那一路被旁路
                                          * （见下面 M5 的注释），门退化成纯 is_static，
                                          * 而等效原理下匀速平动对陀螺/加计不可见 ——
                                          * 实测走动时 is_static 可以 100% 成立。
                                          * 实测静止 e_ac 0.048~0.057、运动 0.37~84，
                                          * 门限 0.20 的分离度足够。它只能让门更保守。
                                          * ★ 用独立常量，不借 V5F_DET_AC_E_ON：
                                          *   一环一门，改一个不许动另一个。 */""", 1)
assert u.count('#define V5F_FW_VER        18u') == 1
u = u.replace('#define V5F_FW_VER        18u', '#define V5F_FW_VER        19u', 1)
assert u.count('/*') == u.count('*/')
shutil.copy2(T, T + '.bak_s1k')
open(T, 'wb').write(u.encode('gbk'))
print('v5f_tune.h: BARO_DT_MAX_C + ZUPT_EAC_MAX + VER 19')

t = open(P, 'rb').read().decode('gbk')


def sub(a, b, nm):
    global t
    n = t.count(a)
    assert n == 1, '锚点 %s 匹配 %d 次' % (nm, n)
    t = t.replace(a, b, 1)


# ---- 新增状态 ----
sub("static float    s_pos_wait;",
    """static float    s_alt_wait;           /* 距上次 GPS 高度观测的秒数（同位置一起降频） */
static float    s_baro_t_last;
static uint8_t  s_baro_t_seen;
static float    s_pos_wait;""", 'vars')

# ---- M3 的 |dh| / |dT| 真正生效 ----
sub("""            if (p > V5F_EKF_BARO_PA_LO && p < V5F_EKF_BARO_PA_HI) {
                float hnew = V5F_EKF_BARO_M_PER_PA * (V5F_EKF_BARO_PA0 - p);
                if (!s_baro_seen || fabsf(hnew - s_baro_h_last) < V5F_EKF_BARO_STEP_MAX_M) {
                    s_baro_h_last = hnew;
                }
                s_baro_h = hnew;
                s_baro_seen = 1u;
                s_baro_new  = 1u;
            }""",
    """            if (p > V5F_EKF_BARO_PA_LO && p < V5F_EKF_BARO_PA_HI) {
                float hnew = V5F_EKF_BARO_M_PER_PA * (V5F_EKF_BARO_PA0 - p);
                /* ★ 这里原来是**死判据**：s_baro_h_last 只在跳变小时更新，而真正被使用的
                 *   s_baro_h 无条件赋值 —— 等于 |dh| 门从来没有拒绝过任何样本。
                 *   现在真的拒绝：跳变或温差超限就整帧丢弃（不更新 s_baro_h、不置号志）。 */
                float dth = fabsf(h->baro.temp_celsius - s_baro_t_last);
                if (s_baro_seen
                    && (fabsf(hnew - s_baro_h_last) >= V5F_EKF_BARO_STEP_MAX_M
                        || (s_baro_t_seen && dth > V5F_EKF_BARO_DT_MAX_C))) {
                    /* 野值 / 温度跳变：丢本样本 */
                } else {
                    s_baro_h_last = hnew;
                    s_baro_h = hnew;
                    s_baro_seen = 1u;
                    s_baro_new  = 1u;
                }
                s_baro_t_last = h->baro.temp_celsius;
                s_baro_t_seen = 1u;
            }""", 'baro')

# ---- M2 降频 ----
sub("""    if (gate->ekf_gps_alt) {
        H_zero(1u);
        s_H[0][2] = 1.0f;
        R[0] = V5F_GPS_ALT_R_M * V5F_GPS_ALT_R_M;
        r[0] = h->gps_gga.alt_m - s_x[IX_P + 2];
        st = ekf_update(R, 1u, r, V5F_EKF_NIS_MAX_1, NULL, &s_rej[4]);
        if (st == 0u) s_gate_bits |= V5F_EKF_GB_GPS_ALT;
    }""",
    """    /* GPS 高度必须与位置一起降频：它和水平位置是**同一条接收机解算**出来的，
     * 误差同样是 5 s 以上相关时间的慢随机游走（水平实测 lag1 自相关 0.999），
     * 按每个历元喂就是让 R=(5.5 m)^2 名不副实、偷偷给垂直通道超配权重 ——
     * 与气压原来那个错完全同类。 */
    if (gate->ekf_gps_alt && s_alt_wait >= V5F_EKF_POS_PERIOD_S) {
        H_zero(1u);
        s_H[0][2] = 1.0f;
        R[0] = V5F_GPS_ALT_R_M * V5F_GPS_ALT_R_M;
        r[0] = h->gps_gga.alt_m - s_x[IX_P + 2];
        st = ekf_update(R, 1u, r, V5F_EKF_NIS_MAX_1, NULL, &s_rej[4]);
        if (st == 0u) { s_gate_bits |= V5F_EKF_GB_GPS_ALT; s_alt_wait = 0.0f; }
    }""", 'm2')

# ---- 每帧累加 s_alt_wait ----
sub("""    s_pos_wait  += dt;
    s_baro_wait += dt;""",
    """    s_pos_wait  += dt;
    s_alt_wait  += dt;
    s_baro_wait += dt;""", 'wait')
sub("""            s_pos_wait = 0.0f; s_baro_wait = 0.0f;""",
    """            s_pos_wait = 0.0f; s_baro_wait = 0.0f; s_alt_wait = 0.0f;
            s_baro_t_last = 0.0f; s_baro_t_seen = 0u;""", 'waitclr')

# ---- M1/M2 的 flags 纪律 ----
sub("""    gq_ok  = (uint8_t)((h->gps_gga.fix_quality >= 1u)
                    && (h->gps_gga.sat_num >= V5F_EKF_SV_MIN)
                    && (h->gps_gga.hdop > 0.0f)
                    && (h->gps_gga.hdop <= V5F_EKF_HDOP_MAX));""",
    """    /* ★ 本项目的铁律"判空查位、勿用值判空"：每个定标字段都要先查它对应的 flags 位
     *   再信它的值。原来 sat_num / hdop 直接用值判 —— 字段一旦没解析出来就是 0，
     *   门会**永久关闭**，GPS 整路静默失效（与 fixq>=2 那类"永远打不开的门"同型）。
     *   现在条件挂在 flags 上：没这个字段就不拿它当判据。
     * ★ fixq >= 1 是**有意偏离设计**（设计写 {2,4}）：近场全部是单点定位 fixq=1，
     *   照设计写就永远打不开；R 已经按 4 m（实测 2D RMS 4.24~5.97）给足了。 */
    gq_ok  = (uint8_t)((h->gps_gga.fix_quality >= 1u)
                    && ((!(h->gps_gga.fresh.flags & SHM_GGA_SV))
                        || (h->gps_gga.sat_num >= V5F_EKF_SV_MIN))
                    && ((!(h->gps_gga.fresh.flags & SHM_GGA_HDOP))
                        || (h->gps_gga.hdop <= V5F_EKF_HDOP_MAX)));""", 'gq')

sub("""    vd_ok  = (uint8_t)((h->gps_gsa.vdop <= 0.0f) || (h->gps_gsa.vdop <= V5F_EKF_VDOP_MAX));""",
    """    vd_ok  = (uint8_t)((!(h->gps_gsa.fresh.flags & SHM_GSA_DOP))
                    || (h->gps_gsa.vdop <= V5F_EKF_VDOP_MAX));""", 'vd')

# ---- M4 为什么不需要降频（写清楚免得以后误改） ----
sub("""    gate->ekf_gps_vel = (uint8_t)(gq_ok && gn_ok && snr_ok""",
    """    /* M4 **不需要**降频：多普勒速度是每个历元独立解算的白噪声（与位置那种
     * 5 s 相关的慢游走不同），实测扣掉直流偏置后 std 0.15 m/s，R=(0.15 m/s)^2 成立。
     * 这条与 M1/M2/M3 的处理不同是**有依据的**，别为了"统一"去给它加降频。 */
    gate->ekf_gps_vel = (uint8_t)(gq_ok && gn_ok && snr_ok""", 'm4doc')

# ---- M5 追加交流能量 ----
sub("""    gate->ekf_zupt = (uint8_t)(h->stat.is_static
                    && ((h->imu.a_lin[0]*h->imu.a_lin[0]
                       + h->imu.a_lin[1]*h->imu.a_lin[1]
                       + h->imu.a_lin[2]*h->imu.a_lin[2]) < V5F_EKF_ZUPT_ALIN2)
                    && ((spac < V5F_EKF_ZUPT_SPEED_MAX_MPS)
                        || !(h->gps_rmc.fresh.flags & SHM_RMC_SPEED)));""",
    """    /* ★ 无 GPS 时 speed 那一路被 flags 旁路掉，门退化成纯 is_static —— 而等效原理下
     *   匀速平动对陀螺与加计都不可见（实测走动时 is_static 可 100% 成立）。
     *   追加**外生**的交流能量判据 e_ac（三轴滑窗方差之和，对直流零偏不可见）：
     *   实测静止 0.048~0.057、运动 0.37~84，门限 0.20。它只能让门更保守。 */
    gate->ekf_zupt = (uint8_t)(h->stat.is_static
                    && (h->stat.e_ac < V5F_EKF_ZUPT_EAC_MAX)
                    && ((h->imu.a_lin[0]*h->imu.a_lin[0]
                       + h->imu.a_lin[1]*h->imu.a_lin[1]
                       + h->imu.a_lin[2]*h->imu.a_lin[2]) < V5F_EKF_ZUPT_ALIN2)
                    && ((spac < V5F_EKF_ZUPT_SPEED_MAX_MPS)
                        || !(h->gps_rmc.fresh.flags & SHM_RMC_SPEED)));""", 'zupt')

# ---- M6 的窗口差异写清楚 ----
sub("""        gate->ekf_tilt = (uint8_t)(!sat && h->imu.acc_valid""",
    """        /* ★ 与设计的一处**有意偏离**：设计写"长窗净 |w| < 2 dps"（256 ms 窗），
         *   这里用的是 stat.level_dps（128 帧 ≈ 16 ms）。短窗其实更对 —— 要排除转动
         *   是因为离心项 a_c = w^2*r 会淹没重力方向，而 a_c 只在高 w 时才显著，
         *   16 ms 窗抓的正是它；慢转时 a_c 可忽略，不该因此关掉倾斜观测。
         *   独立字段：与 gate->att_tilt 同源判据但各自成位，改一个不动另一个。 */
        gate->ekf_tilt = (uint8_t)(!sat && h->imu.acc_valid""", 'm6doc')

assert t.count('/*') == t.count('*/')
shutil.copy2(P, P + '.bak_s1k')
open(P, 'wb').write(t.encode('gbk'))
print('proc_ekf.c: 4 处同类失误已修 + 2 处有意偏离已标注')
print()
print('fw_tag 期望 = %d' % ((19 << 16) | (112 << 8) | 1 | 2 | 4))
