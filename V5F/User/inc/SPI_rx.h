#ifndef __SPI_RX_H
#define __SPI_RX_H

#include <stdint.h>
#include "mipc_shm.h"   /* 共享区 flags 位定义（SHM_RMC_ / SHM_GGA_ / SHM_GSA_ 前缀）与布局 */

/* ==================== 数据保持器（原 observer 模块并入本文件） ====================
 * 职责：集中保存各传感器"最新一份数据 + 新鲜度信息 + 有效性"，供主循环等消费者读取。
 *   - IMU（陀螺/加速度/温度）：来自 SPI DMA 帧，在 DMA1_Channel2 中断里解析后写入；
 *   - 磁力计 / 气压计 / GPS：来自双核共享区，中断里按 cnt 变化搬运（SPI_rx.c 的 hold_poll）。
 * 姿态解算(attitude.c)、惯性预处理(ins.c) 已弃用删除，本模块不做任何解算。
 *
 * -------------------------------- 新鲜度与有效性 --------------------------------
 * 每个通道自带一份 v5f_fresh_t：
 *   drdy_tick —— 该通道 DRDY（数据就绪）时刻，单位 10 ns 计数（1 tick = 10 ns）。
 *                共享区所有通道统一由 V3F 用 GetTime64_10Ns() 写入，故这里原样搬运、不做换算，
 *                可直接与 GetTime64_10Ns() 比较。
 *                跨核读取一律按"检测到 cnt 变化 → acquire 屏障 → 再读数据/ts"进行（见 hold_poll）。
 *   new_data  —— 新数据信号：写入方（DMA 中断）搬完新数据后置 1，读取方处理完自行清 0。
 *                读取顺序建议：先读 new_data 判断有无新数据 → 再读数据/drdy_tick/flags → 最后清 0。
 *   flags     —— 共享区该通道 flags 位掩码的**原样拷贝**（SHM_RMC_ / SHM_GGA_ / SHM_GSA_ 前缀）。
 *                ★ 判空必须查位，不可用值判空：0 值本身合法（速度 0、经度 0、HDOP 0、海拔 0）。
 *                陀螺/磁力计/气压计通道在共享区里没有 flags 字段，其 flags 恒为 0（有新数据即有效）。
 *
 * GPS 三态判读（用 gps_rmc.fresh.flags + .status，可区分"没启动"与"有信号无定位"）：
 *   (flags & SHM_RMC_STATUS) == 0   → 从未收到带状态字段的 RMC：GPS 未启动 / 未接 / 波特率不符
 *   (flags & SHM_RMC_STATUS) && status == 'V' → 已收到语句但定位无效（有信号、无定位）
 *   status == 'A'                   → 定位有效
 *   （status 保存 NMEA 的 ASCII 原字符 'A'/'V'；经纬度是否可用另看 SHM_RMC_POS 位）
 *
 * -------------------------------- 数值表示约定 --------------------------------
 * 1) 能用 float 精确表达物理量的地方一律用 float（温度 ℃、气压 Pa、速度 m/s、高度 m、
 *    HDOP/PDOP/VDOP 无量纲），共享区的十进制定标整数在搬运时即换算。
 * 2) 只有 float 精度不够时才保留定点，并写明定标方式：
 *      Qm.n          二进制定点：m 位（含符号位）整数部分 + n 位小数，例 Q1.31 = int32 / 2^31；
 *      ×1e7 度       十进制定标整数（scaled fixed-point，定标 10^-7，不是 Q 格式）；
 *      cm / cm/s   十进制定标整数，1 LSB = 0.01 m / 0.01 m/s。
 * 说明：本结构体唯一必须定点的是 GPS 经纬度（float 仅约 7.2 位十进制有效数字，在 ±180° 内
 *       分辨不出 10^-7°≈1.1 cm）；目前没有 Qm.n 形式的字段。
 */

/* 通道新鲜度 + 有效性 */
typedef struct {
    uint64_t         drdy_tick;  /* DRDY 时刻，单位 10 ns 计数（64 位，非原子；按文件头约定读取） */
    volatile uint8_t new_data;   /* 新数据信号：1 = 有尚未处理的新数据 */
    uint8_t          flags;      /* 共享区 flags 原样；无 flags 的通道恒 0（判空查位，勿用值判空） */
} v5f_fresh_t;

/* ---- IMU：ICM-42605（SPI DMA 帧，DMA 中断更新） ---- */
typedef struct {
    v5f_fresh_t fresh;           /* drdy_tick 来自共享区陀螺通道（V3F 在 DRDY 上升沿先发布再触发 SPI 读） */
    int16_t     gyro_lsb[3];     /* 角速度原始 LSB（按物理量定标的整数）：满量程 ±2000 °/s 对应 ±32768，
                                    即 16.4 LSB/(°/s) */
    int16_t     accel_lsb[3];    /* 加速度原始 LSB：满量程 ±4 g 对应 ±32768，即 8192 LSB/g */
    float       temp_celsius;    /* 陀螺温度 ℃（换算自 SPI 原始值：132.48 LSB/℃，25 ℃ 为零点） */
} v5f_imu_t;

/* ---- IST8310 磁力计（共享区搬运；该通道无 flags） ---- */
typedef struct {
    v5f_fresh_t fresh;
    int16_t     lsb[3];          /* 磁场原始 LSB（16 位补码）：0.3 μT/LSB */
} v5f_mag_t;

/* ---- BMP388 气压计（共享区搬运；该通道无 flags） ---- */
typedef struct {
    v5f_fresh_t fresh;
    float       temp_celsius;    /* 温度 ℃（共享区已是 float，无定点换算） */
    float       press_pascal;    /* 气压 Pa（同上） */
} v5f_baro_t;

/* ---- GPS RMC：定位 / 速度 / 日期（共享区搬运） ---- */
typedef struct {
    v5f_fresh_t fresh;           /* flags: SHM_RMC_STATUS / POS / SPEED / DATE */
    uint8_t     status;          /* NMEA RMC 状态原字符：'A'=有效 / 'V'=无效 */
    int32_t     lat_e7;          /* 纬度：十进制定标整数，1 LSB = 10^-7 °（北正南负），与共享区同格式 */
    int32_t     lon_e7;          /* 经度：同上（东正西负） */
    float       speed_mps;       /* 对地速度 m/s（共享区为 cm/s 定标整数） */
    uint32_t    date_ddmmyy;     /* UTC 日期 ddmmyy（十进制字面量，非时间戳） */
} v5f_gps_rmc_t;

/* ---- GPS GGA：定位质量 / 高度（共享区搬运） ---- */
typedef struct {
    v5f_fresh_t fresh;           /* flags: SHM_GGA_QUALITY / SV / HDOP / ALT */
    float       alt_m;           /* 海拔 m（共享区为 cm 定标整数） */
    uint8_t     fix_quality;     /* 定位质量：0=无效 1=单点 2=差分 4=RTK 固定 5=RTK 浮点 */
    uint8_t     sat_num;         /* 使用卫星数 */
    float       hdop;            /* 水平精度因子，无量纲（共享区为 ×100 定标整数） */
} v5f_gps_gga_t;

/* ---- GPS GSA：PDOP / VDOP（共享区搬运） ---- */
typedef struct {
    v5f_fresh_t fresh;           /* flags: SHM_GSA_DOP */
    float       pdop;            /* 位置精度因子，无量纲（共享区为 ×100 定标整数） */
    float       vdop;            /* 垂直精度因子，无量纲（同上） */
} v5f_gps_gsa_t;

/* ---- 保持器总成：按传感器分块，每块 = 新鲜度 + 数据 ---- */
typedef struct {
    v5f_imu_t     imu;
    v5f_mag_t     mag;
    v5f_baro_t    baro;
    v5f_gps_rmc_t gps_rmc;
    v5f_gps_gga_t gps_gga;
    v5f_gps_gsa_t gps_gsa;
} v5f_hold_t;

extern volatile v5f_hold_t g_v5f_hold;

/* ---------------- 函数原型 ---------------- */
void SPI_DMA_Init(void);

#endif /* __SPI_RX_H */
