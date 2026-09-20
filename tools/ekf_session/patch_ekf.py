# -*- coding: utf-8 -*-
"""S1：把 proc_ekf.c（新文件）+ J 组常量 + 结构/门字段 + 第 7/8 步 + 上报列 + makefile 一次落地。

纪律（与前面所有 patch 一致）：
  * 每个锚点必须恰好匹配 1 次（先做 CRLF 归一化再数）；
  * 每个文件改完必须变大；
  * 插入文本的 /* 与 */ 个数必须相等；
  * 反斜杠续行后不许有行尾空格；
  * 写回前必须能用原编码编回去（V5F/User 全是 GBK）；
  * 逐文件 .bak_ekf 备份。
"""
import os
import shutil
import sys

# --- 备份重定向 shim（2026-09-21）：严禁把 *.bak* 写进源码目录（V5F/、Common/），
#     IDE 按目录扫描会把它们当源文件包含；统一改写到 <repo>/bak_src/ 的镜像路径。
import os as _os
import shutil as _sh

_REPO = r'C:\Users\33\Documents\v2\h415-imu42605-'
_orig_copy2 = _sh.copy2


def _copy2_redirect(src, dst, *a, **kw):
    d = str(dst)
    if '.bak' in d and d.lower().startswith(_REPO.lower()):
        d = _os.path.join(_REPO, 'bak_src', _os.path.relpath(d, _REPO))
        _os.makedirs(_os.path.dirname(d), exist_ok=True)
        print('-- 备份(重定向到 bak_src) %s' % _os.path.relpath(d, _REPO))
    return _orig_copy2(src, d, *a, **kw)


_sh.copy2 = _copy2_redirect


R = r'C:\Users\33\Documents\v2\h415-imu42605-'
TUNE = R + r'\V5F\User\inc\v5f_tune.h'
HDR  = R + r'\V5F\User\inc\SPI_rx.h'
PROC = R + r'\V5F\User\inc\v5f_proc.h'
SRC  = R + r'\V5F\User\src\SPI_rx.c'
NEW  = R + r'\V5F\User\src\proc_ekf.c'
SRCUTF = r'C:\Users\33\Documents\v2\_proc_ekf_utf8.c'
MK   = R + r'\V5F\obj\User\src\subdir.mk'

LOG = []


def rd(p, enc=None):
    b = open(p, 'rb').read()
    for e in ([enc] if enc else ['gbk', 'utf-8']):
        try:
            return b.decode(e), e
        except Exception:
            pass
    raise SystemExit('decode fail: ' + p)


def wr(p, t, enc, tag='ekf'):
    shutil.copy2(p, p + '.bak_' + tag)
    open(p, 'wb').write(t.encode(enc))


def sub(t, a, b, name):
    n = t.count(a)
    assert n == 1, '锚点 %s 匹配 %d 次 (期望 1)' % (name, n)
    return t.replace(a, b, 1)


def chk(t, name, grew_from):
    assert len(t) > grew_from, '%s 没有变大' % name
    assert t.count('/*') == t.count('*/'), '%s 注释不配平 %d/%d' % (
        name, t.count('/*'), t.count('*/'))
    for i, l in enumerate(t.split('\n')):
        if l.rstrip('\r').endswith('\\') and l.rstrip('\r') != l.rstrip():
            raise SystemExit('%s 第 %d 行续行后有空白' % (name, i + 1))
    LOG.append('  %-22s OK  (+%d B)' % (name, len(t) - grew_from))


# ===========================================================================
# 0) proc_ekf.c：UTF-8 -> GBK
# ===========================================================================
u = open(SRCUTF, 'rb').read().decode('utf-8')
try:
    g = u.encode('gbk')
except UnicodeEncodeError as e:
    print('GBK 编不了：', e)
    sys.exit(1)
if os.path.exists(NEW):
    shutil.copy2(NEW, NEW + '.bak_ekf')
open(NEW, 'wb').write(g)
n_open, n_close = u.count('/*'), u.count('*/')
assert n_open == n_close, 'proc_ekf.c 注释不配平 %d/%d' % (n_open, n_close)
print('proc_ekf.c  %d 行 / %d B (GBK)，注释配平 %d' % (u.count('\n') + 1, len(g), n_open))

# ===========================================================================
# 1) v5f_tune.h：版本 + 开关 + J 组
# ===========================================================================
t, e = rd(TUNE)
n0 = len(t)
t = sub(t, '#define V5F_FW_VER        9u', '#define V5F_FW_VER        10u', 'VER')
t = sub(t, '#define V5F_EKF_EN        0u      /* 阶段 1 打开 */',
        '#define V5F_EKF_EN        1u      /* 阶段 1（S1）已落地：16 维 ESKF 影子模式 */', 'EKF_EN')

J = r'''
/* =====================================================================
 * J 组：导航 EKF（处理函数 7/8）—— 设计见仓库根的 ekf_design.md
 *
 * ★ 允许在线改的只有**标量**（Q/R、门阈值、开关、抽取率）；数组尺寸与状态维
 *   数一律编译期固定（否则 P 的布局会随参数变，在线改就不可能安全）。
 *
 * ★ S0.3 实测（长静止记录逐差 + Allan）：加计零偏 / 陀螺零偏 / 气压基准三个
 *   随机游走的漂移**全部低于 300 s 的检测门限**，所以 Q 的三个 RW 项取 0。
 *   这不是"没测"，是"测到的上界比观测噪声还低"。陀螺零偏不稳定性实测
 *   0.000131 dps = 0.472 度/h，重复两次一致。
 * ===================================================================== */

/* ---- 开关与抽取 ---- */
#define V5F_EKF_DECIM            16u     /* 每 16 帧一个 EKF 步 = 8021.9/16 = 501.4 Hz */
#define V5F_EKF_YAW_OBS_EN       1u      /* M7 磁偏航在 EKF 内闭环（默认开） */
#define V5F_EKF_TILT_EN          1u      /* M6 重力/倾斜观测 */
#define V5F_EKF_DT_MAX_S         0.2f    /* 单帧 dt 上限，超过视为时间戳异常 */
#define V5F_EKF_G_MPS2           9.7985f /* 本地重力。与 V5F_VEL_G_LOCAL 同值但**独立成
                                          * 常量**：两环不许通过同一个宏耦合 */
#define V5F_EKF_R_EARTH_M        6378137.0f  /* WGS84 长半轴，经纬度 -> 米 */
#define V5F_EKF_ALIGN_AMAG_TOL   0.02f   /* 对齐时 |a| 必须落在 1 g +-2% */

/* ---- 过程噪声（除 RW 外都来自实测）---- */
#define V5F_EKF_SIG_G_DPS        0.1224f /* 陀螺单样本噪声 dps（实测，三轴平均） */
#define V5F_EKF_SIG_G_RADS       (V5F_EKF_SIG_G_DPS * 0.017453292f)
#define V5F_EKF_SIG_A_MPS2       0.0217f /* 加计噪声 2.21 mg -> m/s^2（实测） */
#define V5F_EKF_SIG_BA_RW        0.0f    /* 加计零偏随机游走：实测低于门限（S0.3） */
#define V5F_EKF_SIG_BG_RW        0.0f    /* 陀螺零偏随机游走：同上 */
#define V5F_EKF_SIG_BARO_RW      0.0f    /* 气压基准随机游走：同上 */
#define V5F_EKF_Q_FLOOR          1.0e-12f
#define V5F_EKF_P_FLOOR          1.0e-12f

/* ---- P0（对齐那一帧）---- */
#define V5F_EKF_P0_POS_M         1.0f
#define V5F_EKF_P0_VEL_MPS       0.1f
#define V5F_EKF_P0_TILT_RAD      (0.5f * 0.017453292f)   /* 对齐残差实测中位 0.49 度 */
#define V5F_EKF_P0_YAW_RAD       (3.0f * 0.017453292f)   /* 磁航向初始化误差；此后由 M7 收紧 */
#define V5F_EKF_P0_BA_MPS2       0.02f
#define V5F_EKF_P0_BG_RADS       (0.02f * 0.017453292f)
#define V5F_EKF_P0_BARO_M        5.0f

/* ---- 观测噪声 R（数值来源都是 I 组的实测值）---- */
#define V5F_EKF_MAG_SIG_RAD      (V5F_MAG_YAW_R_DEG * 0.017453292f)  /* 0.7 度 */
#define V5F_EKF_TILT_SIG_RAD     (0.5f * 0.017453292f)               /* 单位矢量切向 0.5 度 */
#define V5F_EKF_BARO_SIG_M       V5F_BARO_R_M                        /* 0.13 m */
#define V5F_EKF_ZUPT_SIG_MPS     0.05f   /* vel_soft=255 时的速度 1sigma；按 lam 反比缩放 */
#define V5F_EKF_C_N0_REF_DBHZ    33.0f   /* C/N0 加权参考（实测 27.8->28.6 时 UERE 7.46->3.74 m）*/
#define V5F_EKF_POS_PERIOD_S     (1.0f / V5F_GPS_POS_DECIM_HZ)       /* 3.333 s */
#define V5F_MAG_DECL_RAD         (V5F_MAG_DECL_DEG * 0.017453292f)   /* -7.53 度 -> rad */

/* ---- 门阈值（第 8 步用；判据只用外生量）---- */
#define V5F_EKF_SV_MIN           6u
#define V5F_EKF_HDOP_MAX         2.0f
#define V5F_EKF_VDOP_MAX         4.0f
#define V5F_EKF_SNR_MIN_DBHZ     25.0f
#define V5F_EKF_VEL_SPEED_MIN_MPS    0.4f
#define V5F_EKF_ZUPT_SPEED_MAX_MPS   0.5f
#define V5F_EKF_ZUPT_ALIN2       (0.05f * 0.05f)   /* |a_lin| < 0.05 g（a_lin 单位就是 g）*/
#define V5F_EKF_TILT_AMAG_TOL    0.06f             /* |a|^2 偏离 1，约等于 |a| 偏离 3% */
#define V5F_EKF_TILT_LEV_DPS     2.0f
#define V5F_EKF_BARO_WARMUP_S    5.0f   /* 实测上电 5 s 内气压有暂态：std 2481 Pa vs 5~6 Pa */
#define V5F_EKF_BARO_PA0         101325.0f
#define V5F_EKF_BARO_M_PER_PA    0.08326f  /* dh/dp = 44330*0.190263/101325，线性化 */
#define V5F_EKF_BARO_PA_LO       30000.0f
#define V5F_EKF_BARO_PA_HI       110000.0f
#define V5F_EKF_BARO_STEP_MAX_M  50.0f
#define V5F_EKF_DOP_TAU_S        5.0f   /* 多普勒直流偏置牵引时间常数 */

/* ---- 内层 chi2 门限（与"外部门"分开：门=物理可用性，chi2=野值剔除）
 *   卡方分布上侧 99.9%：1 维 10.83 / 2 维 13.82 / 3 维 16.27 ---- */
#define V5F_EKF_NIS_MAX_1        10.83f
#define V5F_EKF_NIS_MAX_2        13.82f
#define V5F_EKF_NIS_MAX_3        16.27f

/* ---- gate_bits 位定义（上报列 104 的原始值）---- */
#define V5F_EKF_GB_GPS_POS       0x0001u
#define V5F_EKF_GB_GPS_ALT       0x0002u
#define V5F_EKF_GB_BARO          0x0004u
#define V5F_EKF_GB_GPS_VEL       0x0008u
#define V5F_EKF_GB_ZUPT          0x0010u
#define V5F_EKF_GB_TILT          0x0020u
#define V5F_EKF_GB_MAG           0x0040u
#define V5F_EKF_GB_ALIGN         0x0080u   /* 已对齐（一次性，之后恒置） */
#define V5F_EKF_GB_STEP          0x0100u   /* 本周期完成了一次 EKF 步 */
#define V5F_EKF_GB_CHI2          0x0200u   /* 本周期有观测被内层 chi2 剔除 */
#define V5F_EKF_GB_ORIGIN        0x0400u   /* ENU 原点已建立（位置列才有绝对意义） */

'''
t = sub(t, '#endif /* __V5F_TUNE_H */', J.lstrip('\n') + '#endif /* __V5F_TUNE_H */', 'J组')
chk(t, 'v5f_tune.h', n0)
wr(TUNE, t, e)

# ===========================================================================
# 2) SPI_rx.h：v5f_ekf_t + 挂进保持器
# ===========================================================================
t, e = rd(HDR)
n0 = len(t)
EKFT = r'''/* ---- 处理结果：导航 EKF（v5f_proc_ekf 写；门由 v5f_proc_ekf_gate 写） ----
 * 16 维误差状态 ESKF，标称量就是下面这些。内部一律 SI + 弧度，只有上报换成度/dps。
 * 导航系 = **真 ENU**（x 东, y 北, z 上），由对齐那一帧用磁力计一次性把 x 定到真东。
 * ★ 偏航闭环在**滤波器内部**（观测 M7，z = 磁偏角 D 这个已知常数）；
 *   旧 proc_attitude 链仍然不闭环，两者分工不混。 */
typedef struct {
    float            p[3];            /* 导航系位置 E/N/U，m；原点 = ENU 对齐点 */
    float            v[3];            /* 导航系速度 m/s */
    float            q[4];            /* 重力系世界四元数（机体->导航） */
    float            a_nav[3];        /* 导航系线性加速度 m/s^2（已去重力与零偏） */
    float            ba[3];           /* 加计零偏残差 m/s^2（在离线常量之上） */
    float            bg[3];           /* 陀螺零偏残差 dps（内部是 rad/s，发布时换算） */
    float            b_baro;          /* 气压高度零偏 m；气压高度 = p_z + b_baro */
    float            sigma_yaw_deg;   /* 偏航 1sigma（度）—— "航向现在能不能信"的直接读数：
                                       * 门开被 M7 收紧、门关按 Q_bg 增长。 */
    float            sigma_pos_h;     /* 水平位置 1sigma，m */
    float            sigma_vel_h;     /* 水平速度 1sigma，m/s */
    float            nis[5];          /* 归一化新息平方：位置/速度/气压/重力/磁偏航。
                                       * 应分别趋近 2/2/1/3/1；>> 说明 R 给小了。 */
    uint16_t         gate_bits;       /* V5F_EKF_GB_* 位（v5f_tune.h J 组） */
    volatile uint8_t aligned;         /* 1 = 已完成重力 + 磁力计对齐 */
    volatile uint8_t origin_ok;       /* 1 = ENU 原点已建立 */
    volatile uint8_t valid;           /* 1 = 本帧状态有效 */
    uint8_t          _rsv_ekf[1];
} v5f_ekf_t;

'''
t = sub(t, '/* ---- 保持器总成 ---- */', EKFT + '/* ---- 保持器总成 ---- */', 'ekf_t')
t = sub(t, '    v5f_gps_gsv_t gps_gsv;\n} v5f_hold_t;',
        '    v5f_gps_gsv_t gps_gsv;\n    v5f_ekf_t     ekf;       /* 导航 EKF（处理函数 7/8） */\n} v5f_hold_t;', 'hold')
chk(t, 'SPI_rx.h', n0)
wr(HDR, t, e)

# ===========================================================================
# 3) v5f_proc.h：门字段 + 两个原型
# ===========================================================================
t, e = rd(PROC)
n0 = len(t)
GF = r'''    /* ===== EKF 的七道门（第 8 步 v5f_proc_ekf_gate **下游**写，第 7 步
     *       v5f_proc_ekf **下一帧**读）。与上面几条同一纪律，一字不改：
     *         · 一环一门：每道门只门一个观测，改一个不动另一个；
     *         · 判据只用**外生量**（原始 LSB、离线常量、stat、acc_valid、
     *           mag.trust、GPS 报文质量字段），绝不取滤波器自己的输出；
     *         · 门的"物理可用性"与内层新息 chi2 的"野值剔除"是两件事，
     *           chi2 在第 7 步算并单独记进 gate_bits 的 bit9。
     *   ekf_gps_pos  fixq>=1 + 星数 + HDOP + C/N0 + RMC/GGA 有效位
     *   ekf_gps_alt  同上且 VDOP 合格 + GGA 高度位
     *   ekf_baro     上电预热满 5 s + 温度合理 + 未饱和
     *   ekf_gps_vel  同上且**原始** speed > 0.4 m/s（低速时 course 无意义）
     *   ekf_zupt     is_static + |a_lin| < 0.05 g + 原始 GPS 速度 < 0.5 m/s
     *                （只用 is_static 不行：等效原理下匀速平动对它不可见）
     *   ekf_tilt     | |a| - 1 | < 3% + 长窗净 |w| 小（或启动旁路），独立字段
     *   ekf_mag_yaw  只用 mag.trust（已含 |y| 模长门与 |a_lin| 倾斜门） */
    volatile uint8_t ekf_gps_pos;
    volatile uint8_t ekf_gps_alt;
    volatile uint8_t ekf_baro;
    volatile uint8_t ekf_gps_vel;
    volatile uint8_t ekf_zupt;
    volatile uint8_t ekf_tilt;
    volatile uint8_t ekf_mag_yaw;
    volatile uint8_t _rsv_ekf;
} v5f_proc_gate_t;'''
t = sub(t, '} v5f_proc_gate_t;', GF, 'gate')

PR = r'''/* 7) 导航 EKF（16 维误差状态，影子模式；**新建文件** proc_ekf.c）
 *    输入：imu.gyro_lsb / accel_lsb（**原始**，只套离线标定常量 —— 不用被在线牵引
 *          改过的 gyro_dps / accel_g，否则零偏状态与旧牵引环互相污染且不可辨识）、
 *          att.q（只在**一次性对齐**那一帧用）、mag.f、baro.press_avg、gps_rmc/gga/gsa/gsv、
 *          以及**上一帧**第 8 步写下的七道门；
 *    输出：h->ekf.*（p / v / q / a_nav / ba / bg / b_baro / 各 1sigma / NIS / gate_bits）。
 *    频率：每 V5F_EKF_DECIM(16) 帧一步 = 501.4 Hz；F P F^T 按行摊到那 16 帧里做，
 *          单帧新增约 512 flops，避免把 ISR 顶出 124.7 us 的预算。
 *    失败/越界时返回 V5F_PROC_ERR_TICK 之外的值：本函数只报 V5F_PROC_OK。 */
uint8_t v5f_proc_ekf(volatile v5f_hold_t *h, const volatile v5f_proc_gate_t *gate);

/* 8) EKF 各观测的**外部门控**（下游写、上游下一帧读）
 *    读最新传感器量，写 gate->ekf_* 七个字段与 ENU 原点（首个有效定位那一帧建立）。
 *    与第 5 步（加速度牵引门）完全同构：门永远由下游算，绝不在环内部现算。 */
uint8_t v5f_proc_ekf_gate(volatile v5f_hold_t *h, volatile v5f_proc_gate_t *gate);

'''
t = sub(t, '/* 后续处理函数依次在此声明、在 src/ 下各加一个 .c */',
        PR + '/* 后续处理函数依次在此声明、在 src/ 下各加一个 .c */', 'proto')
chk(t, 'v5f_proc.h', n0)
wr(PROC, t, e)

# ===========================================================================
# 4) SPI_rx.c：列数 + 误差量 + 第 7/8 步 + 上报块
# ===========================================================================
t, e = rd(SRC)
n0 = len(t)
t = sub(t, '#define JF_CH_NUM     83u    /* 80 + 下行验证 3 列 */',
        '#define JF_CH_NUM     111u   /* 80 + 下行验证 3 + 导航 EKF 28 列 */', 'NCH')
t = sub(t, '#define JF_FRAME_LEN  (JF_CH_NUM * 4u + 6u)      /* 338 = 帧头2+帧长2+载荷332+帧尾2 */',
        '#define JF_FRAME_LEN  (JF_CH_NUM * 4u + 6u)      /* 450 = 帧头2+帧长2+载荷444+帧尾2 */', 'FLEN')

t = sub(t, 'static volatile uint8_t s_proc_err_velocity  = V5F_PROC_OK;',
        'static volatile uint8_t s_proc_err_velocity  = V5F_PROC_OK;\n'
        'static volatile uint8_t s_proc_err_ekf       = V5F_PROC_OK;\n'
        'static volatile uint8_t s_proc_err_ekf_gate  = V5F_PROC_OK;', 'err')

# 上报块（追加在最后一列之后、有限性扫描之前）
REP = r'''    /* 84~111 导航 EKF（处理函数 7/8）—— 影子模式：与旧链并行跑、全部记录，
     * 达标前不影响任何旧输出。设计见仓库根 ekf_design.md，常量见 v5f_tune.h J 组。
     *  84~86   ekf.p[3]        导航系位置 E/N/U，m（原点 = ENU 对齐点，见列 104 的 bit10）
     *  87~89   ekf.v[3]        导航系速度 m/s
     *  90~93   ekf.q[4]        重力系世界四元数（机体->导航）
     *  94~96   ekf.a_nav[3]    导航系线性加速度 m/s^2（已去重力与零偏；"合加速度"的 EKF 版）
     *  97~99   ekf.ba[3]       加计零偏残差 m/s^2
     * 100~102  ekf.bg[3]       陀螺零偏残差 dps
     * 103      ekf.b_baro      气压高度零偏 m（气压高度 = p_z + b_baro）
     * 104      ekf.gate_bits   V5F_EKF_GB_* 位：bit0 GPS位置 bit1 GPS高度 bit2 气压
     *                          bit3 GPS速度 bit4 ZUPT bit5 重力/倾斜 bit6 磁偏航(闭环)
     *                          bit7 已对齐 bit8 本周期有EKF步 bit9 本周期有观测被chi2剔除
     *                          bit10 ENU原点已建立
     * 105      sigma_yaw_deg   偏航 1sigma（度）—— 门开被 M7 收紧、门关按 Q_bg 增长，
     *                          "航向现在能不能信"的直接读数（失效可观测，不是静默失效）
     * 106      sigma_pos_h     水平位置 1sigma m
     * 107      sigma_vel_h     水平速度 1sigma m/s
     * 108~112  ekf.nis[5]      归一化新息平方：位置/速度/气压/重力/磁偏航。
     *                          应分别趋近 2/2/1/3/1；远大于维数 -> R 给小了。
     * ★ 1~83 列号一个都不动，新量只追加在尾部。 */
    for (i = 0u; i < 3u; i++) ch[c++] = g_v5f_hold.ekf.p[i];
    for (i = 0u; i < 3u; i++) ch[c++] = g_v5f_hold.ekf.v[i];
    for (i = 0u; i < 4u; i++) ch[c++] = g_v5f_hold.ekf.q[i];
    for (i = 0u; i < 3u; i++) ch[c++] = g_v5f_hold.ekf.a_nav[i];
    for (i = 0u; i < 3u; i++) ch[c++] = g_v5f_hold.ekf.ba[i];
    for (i = 0u; i < 3u; i++) ch[c++] = g_v5f_hold.ekf.bg[i];
    ch[c++] = g_v5f_hold.ekf.b_baro;
    ch[c++] = (float)g_v5f_hold.ekf.gate_bits;
    ch[c++] = g_v5f_hold.ekf.sigma_yaw_deg;
    ch[c++] = g_v5f_hold.ekf.sigma_pos_h;
    ch[c++] = g_v5f_hold.ekf.sigma_vel_h;
    for (i = 0u; i < 5u; i++) ch[c++] = g_v5f_hold.ekf.nis[i];

'''
t = sub(t, '    ch[c++] = (float)g_v5f_hold.stat.ac_bypass;\n',
        '    ch[c++] = (float)g_v5f_hold.stat.ac_bypass;\n\n' + REP, 'report')

# 第 7/8 步
t = sub(t, '''                st = v5f_proc_velocity(&g_v5f_hold, &g_v5f_proc_gate);
                if (st != V5F_PROC_OK) s_proc_err_velocity = st;
''', '''                st = v5f_proc_velocity(&g_v5f_hold, &g_v5f_proc_gate);
                if (st != V5F_PROC_OK) s_proc_err_velocity = st;

                /* 第七步：导航 EKF —— 读**上一帧**第 8 步写下的七道门 */
                st = v5f_proc_ekf(&g_v5f_hold, &g_v5f_proc_gate);
                if (st != V5F_PROC_OK) s_proc_err_ekf = st;

                /* 第八步：EKF 各观测的外部门控 —— 写本帧的门，第 7 步下一帧读
                 *        （与"第 2 步写 -> 第 1 步下一帧读"完全同构） */
                st = v5f_proc_ekf_gate(&g_v5f_hold, &g_v5f_proc_gate);
                if (st != V5F_PROC_OK) s_proc_err_ekf_gate = st;
''', 'steps')

# 循环计数器加宽：111 列已经接近 uint8_t 的边界，别留第二次踩坑的机会
t = sub(t, '    uint8_t i;\n', '    uint32_t i;   /* 列数已达 111：计数器类型必须 >= 上界，见 uint8_t 死循环那次事故 */\n', 'ctype')
chk(t, 'SPI_rx.c', n0)
wr(SRC, t, e)

# ===========================================================================
# 5) subdir.mk：把 proc_ekf.c 加进构建
# ===========================================================================
t, e = rd(MK, 'utf-8')
n0 = len(t)
if 'proc_ekf' not in t:
    t = sub(t, '../User/src/proc_attitude.c \\', '../User/src/proc_attitude.c \\\n../User/src/proc_ekf.c \\', 'mk-src')
    t = sub(t, './User/src/proc_attitude.d \\', './User/src/proc_attitude.d \\\n./User/src/proc_ekf.d \\', 'mk-dep')
    t = sub(t, './User/src/proc_attitude.o \\', './User/src/proc_attitude.o \\\n./User/src/proc_ekf.o \\', 'mk-obj')
    chk(t, 'subdir.mk', n0)
    open(MK, 'wb').write(t.encode('utf-8'))
else:
    LOG.append('  subdir.mk              已有 proc_ekf, 跳过')

print('\n'.join(LOG))
print()
print('fw_tag 期望 = %d   (VER=10, 通道=111, EKF=1, MAGCAL=1, AC=1)'
      % ((10 << 16) | (111 << 8) | 1 | 2 | 4))
