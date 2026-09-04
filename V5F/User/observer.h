#ifndef __OBSERVER_H
#define __OBSERVER_H

#include <stdint.h>

/* ==================== 观察器结构体（供主循环轮询；数据由 DMA 中断经 observer_* 维护） ==================== */
typedef struct {
    /* ----- 中断标志 & 统计 & 延迟 ----- */
    volatile uint32_t log_pending;
    volatile uint32_t gyro_pending;
    volatile uint32_t ist_pending;
    volatile uint32_t bmp_pending;
    volatile uint32_t gps_rmc_pending;
    volatile uint32_t gps_gga_pending;
    volatile uint32_t gps_gsa_pending;

    volatile uint32_t stat_gyro;
    volatile uint32_t stat_ist;
    volatile uint32_t stat_bmp;
    volatile uint32_t stat_gps_rmc;
    volatile uint32_t stat_gps_gga;
    volatile uint32_t stat_gps_gsa;

    volatile uint32_t gyro_err;
    volatile uint32_t gyro_lat_us;
    volatile uint32_t ist_lat_us;
    volatile uint32_t bmp_lat_us;
    volatile uint32_t gps_rmc_lat_us;
    volatile uint32_t gps_gga_lat_us;
    volatile uint32_t gps_gsa_lat_us;

    uint8_t calib_done;                     /* 校准完成 */

    /* ----- 传感器有效数据 ----- */
    float gyro_temp_celsius;                /* 陀螺温度 ℃ */
    float q0, q1, q2, q3;                  /* 姿态四元数（姿态解算更新） */

    /* 加速度计（ICM-42605，SPI rxfifo[3..8]）：原始 LSB（±4g，8192 LSB/g） */
    int16_t accel_x, accel_y, accel_z;

    /* IST8310（磁力计）原始 LSB（16 位补码） */
    int16_t mag_x, mag_y, mag_z;

    /* BMP388（气压计）定点直通 */
    int32_t bmp_temp_x1000;                 /* 温度 ℃×1000 */
    int32_t bmp_press_x1000;                /* 气压 Pa×1000 */

    /* GPS（RMC+GGA 综合）定点直通 */
    int32_t  lat_e7;          /* 纬度 ×1e7 度（北正南负） */
    int32_t  lon_e7;          /* 经度 ×1e7 度（东正西负） */
    uint32_t speed_cmps;      /* 对地速度 cm/s */
    int32_t  alt_cm;          /* 高度 cm */
    uint8_t  fix_quality;     /* 定位质量 */
    uint8_t  sat_num;         /* 卫星数 */
    uint16_t hdop_x100;       /* 水平精度因子 hdop ×100（定点，供显示/质控） */

} v5f_observer_t;

extern volatile v5f_observer_t g_v5f_obs;

/* DMA 中断调用：轮询各共享通道 cnt，维护 pending/stat/lat/传感器数据；
 * 返回本帧陀螺 DRDY 时间戳差分 dt_us（0=首帧或无效），供姿态积分用 */
uint32_t observer_poll(uint64_t now_us);

/* DMA 中断调用：由 SPI 温度原始值更新 gyro_temp_celsius */
void observer_update_gyro_temp(int16_t temp_raw);

/* DMA 中断调用：由 SPI 加速度原始值（rxfifo[3..8] 大端）更新 accel_x/y/z */
void observer_update_accel(int16_t ax, int16_t ay, int16_t az);

#endif /* __OBSERVER_H */
