#ifndef __SPI_RX_H
#define __SPI_RX_H

#include <stdint.h>

/* ==================== 数据保持器（原 observer 模块并入本文件） ====================
 * 职责：把各传感器"最新一份数据"集中存放在 g_v5f_hold，供主循环与 USB-HID 帧读取。
 *   - 陀螺 / 加速度 / 温度：来自 SPI DMA 帧，在 DMA1_Channel2 中断里解析后写入；
 *   - 磁力计 / 气压计 / GPS：来自双核共享区，中断里按 cnt 变化搬运（SPI_rx.c 的 hold_poll）。
 * 姿态解算(attitude.c)、惯性预处理(ins.c) 已弃用删除，本模块不做任何解算。 */

typedef struct {
    /* ----- 陀螺仪 / 加速度计原始值（DMA 中断更新） ----- */
    int16_t gyro_x, gyro_y, gyro_z;         /* 角速度原始 LSB（SPI rxfifo[9..14] 大端） */
    int16_t accel_x, accel_y, accel_z;      /* 加速度原始 LSB（SPI rxfifo[3..8]，±4g / 8192 LSB/g） */
    float   gyro_temp_celsius;              /* 陀螺温度 ℃ */

    /* 姿态四元数：解算模块已弃用，恒为单位四元数 (1,0,0,0) */
    float   q0, q1, q2, q3;

    /* ----- 低频传感器（DMA 中断按 cnt 变化搬运） ----- */
    int16_t  mag_x, mag_y, mag_z;           /* IST8310 原始 LSB（16 位补码） */
    int32_t  bmp_temp_x1000;                /* BMP388 温度 ℃×1000 */
    int32_t  bmp_press_x1000;               /* BMP388 气压 Pa×1000 */
    int32_t  lat_e7, lon_e7;                /* GPS 纬度/经度 ×1e7 度（北正南负 / 东正西负） */
    uint32_t speed_cmps;                    /* GPS 对地速度 cm/s */
    int32_t  alt_cm;                        /* GPS 高度 cm */
    uint8_t  fix_quality;                   /* GPS 定位质量 */
    uint8_t  sat_num;                       /* GPS 卫星数 */
    uint16_t hdop_x100;                     /* GPS 水平精度因子 ×100 */

} v5f_hold_t;

extern volatile v5f_hold_t g_v5f_hold;

/* ---------------- 函数原型 ---------------- */
void SPI_DMA_Init(void);

#endif /* __SPI_RX_H */
