#ifndef __SPI_RX_H
#define __SPI_RX_H

#include <stdint.h>

/* ==================== 数据保持器（原 observer 模块并入本文件） ====================
 * 保持器职责（仅 DMA 中断内由 hold_* 维护）：
 *   轮询各共享通道 cnt → 保存各传感器最新值 → 维护 pending / 统计 / 延迟，
 *   供主循环（OLED）与 USB-HID 上行帧读取。
 * 姿态解算(attitude.c)、惯性预处理(ins.c) 已弃用删除，本模块不做任何解算。 */

/* ==================== 保持器结构体 ==================== */
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

    /* 陀螺角速度原始 LSB（SPI rxfifo[9..14] 大端）：此前只被 attitude 消费，现由保持器留档 */
    int16_t gyro_x, gyro_y, gyro_z;

    /* 姿态四元数容器：解算模块已弃用，恒为单位四元数 (1,0,0,0) */
    float q0, q1, q2, q3;

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

} v5f_hold_t;

extern volatile v5f_hold_t g_v5f_hold;

/* ---------------- 函数原型 ---------------- */
void SPI_DMA_Init(void);

#endif /* __SPI_RX_H */
