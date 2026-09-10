#ifndef __SPI_RX_H
#define __SPI_RX_H

#include <stdint.h>
#include "mipc_shm.h"   /* 共享区 flags 位定义（SHM_RMC_ / SHM_GGA_ / SHM_GSA_ 前缀）与布局 */

/* ==================== 数据保持器（原 observer 模块并入本文件） ====================
 * 集中保存各传感器最新数据 + 新鲜度 + 有效性，供消费者读取；不做任何解算。
 *   - IMU（陀螺/加速度/温度）：SPI DMA 帧，DMA1_Channel2 中断解析后写入；
 *   - 磁力计 / 气压计 / GPS：共享区，中断里按 cnt 变化搬运（见 SPI_rx.c 的 hold_poll）。
 *
 * v5f_fresh_t 三项：
 *   drdy_tick —— DRDY 时刻，单位 10 ns 计数（与共享区同基，可直接与 GetTime64_10Ns() 比较）
 *   new_data  —— 新数据信号，中断置 1，消费者处理完自行清 0
 *   flags     —— 共享区 flags 原样拷贝；★ 判空查位、勿用值判空；陀螺/磁力计/气压计无 flags，恒 0
 *
 * GPS 三态：无 SHM_RMC_STATUS → 未启动/无语句；status=='V' → 有信号无定位；'A' → 定位有效。
 * 各单位见字段注释；唯一必须定点的是经纬度（×1e7 度）。 */

/* 通道新鲜度 + 有效性 */
typedef struct {
    uint64_t         drdy_tick;  /* DRDY 时刻，10 ns 计数 */
    volatile uint8_t new_data;   /* 新数据信号：1 = 有尚未处理的新数据 */
    uint8_t          flags;      /* 共享区 flags 原样；无 flags 的通道恒 0 */
} v5f_fresh_t;

/* ---- IMU：ICM-42605（SPI DMA 帧，DMA 中断更新；drdy_tick 来自共享区陀螺通道） ---- */
typedef struct {
    v5f_fresh_t fresh;
    int16_t     gyro_lsb[3];     /* 角速度原始 LSB：16.4 LSB/(°/s)，满量程 ±2000 °/s 对应 ±32768 */
    int16_t     accel_lsb[3];    /* 加速度原始 LSB：8192 LSB/g，满量程 ±4 g 对应 ±32768 */
    float       temp_celsius;    /* 陀螺温度 ℃（132.48 LSB/℃，25 ℃ 零点） */
} v5f_imu_t;

/* ---- IST8310 磁力计（共享区搬运） ---- */
typedef struct {
    v5f_fresh_t fresh;
    int16_t     lsb[3];          /* 磁场原始 LSB（16 位补码）：0.3 μT/LSB */
} v5f_mag_t;

/* ---- BMP388 气压计（共享区搬运，float 直通） ---- */
typedef struct {
    v5f_fresh_t fresh;
    float       temp_celsius;    /* ℃ */
    float       press_pascal;    /* Pa */
} v5f_baro_t;

/* ---- GPS RMC：定位 / 速度 / 日期（共享区搬运；flags: SHM_RMC_STATUS/POS/SPEED/DATE） ---- */
typedef struct {
    v5f_fresh_t fresh;
    uint8_t     status;          /* NMEA 原字符：'A'=有效 / 'V'=无效 */
    int32_t     lat_e7;          /* 纬度 ×1e7 度（1 LSB = 10^-7°，北正南负） */
    int32_t     lon_e7;          /* 经度 ×1e7 度（东正西负） */
    float       speed_mps;       /* 对地速度 m/s（共享区同为 float m/s） */
    uint32_t    date_ddmmyy;     /* UTC 日期 ddmmyy（十进制字面量） */
} v5f_gps_rmc_t;

/* ---- GPS GGA：定位质量 / 高度（共享区搬运；flags: SHM_GGA_QUALITY/SV/HDOP/ALT） ---- */
typedef struct {
    v5f_fresh_t fresh;
    float       alt_m;           /* 海拔 m（共享区为 cm 定标整数） */
    uint8_t     fix_quality;     /* 0=无效 1=单点 2=差分 4=RTK 固定 5=RTK 浮点 */
    uint8_t     sat_num;         /* 卫星数 */
    float       hdop;            /* 水平精度因子，无量纲（共享区为 ×100 定标整数） */
} v5f_gps_gga_t;

/* ---- GPS GSA：PDOP / VDOP（共享区搬运；flags: SHM_GSA_DOP） ---- */
typedef struct {
    v5f_fresh_t fresh;
    float       pdop;            /* 位置精度因子（共享区为 ×100 定标整数） */
    float       vdop;            /* 垂直精度因子（同上） */
} v5f_gps_gsa_t;

/* ---- 保持器总成 ---- */
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
