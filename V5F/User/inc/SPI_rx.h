#ifndef __SPI_RX_H
#define __SPI_RX_H

#include <stdint.h>

/* ==================== 数据保持器（原 observer 模块并入本文件） ====================
 * 职责：集中保存各传感器"最新一份数据 + 新鲜度信息"，供主循环与 USB-HID 帧读取。
 *   - IMU（陀螺/加速度/温度）：来自 SPI DMA 帧，在 DMA1_Channel2 中断里解析后写入；
 *   - 磁力计 / 气压计 / GPS：来自双核共享区，中断里按 cnt 变化搬运（见 SPI_rx.c 的 hold_poll）。
 * 姿态解算(attitude.c)、惯性预处理(ins.c) 已弃用删除，本模块不做任何解算。
 *
 * -------------------------------- 新鲜度信息 --------------------------------
 * 每个通道自带一份 v5f_fresh_t：
 *   drdy_tick —— 该通道 DRDY（数据就绪）时刻，全结构体统一为 10 ns 计数（1 tick = 10 ns）。
 *                注意共享区里单位并不统一：陀螺通道由 V3F 用 GetTime64_10Ns() 写入（本就是
 *                10 ns 计数），磁力计/气压计/GPS 由 GetTime64_Us() 写入（μs）；后者在搬运时
 *                ×100 归一到 10 ns（无损），这样全结构体的时间戳可直接与 GetTime64_10Ns() 比较。
 *   new_data  —— 新数据信号：写入方（DMA 中断）搬完新数据后置 1，读取方处理完自行清 0。
 *                读取顺序建议：先读 new_data 判断有无新数据 → 再读数据与 drdy_tick → 最后清 0。
 *
 * -------------------------------- 数值表示约定 --------------------------------
 * 1) 能用 float 精确表达物理量的地方一律用 float（温度 ℃、气压 Pa、速度 m/s、高度 m、HDOP），
 *    共享区里的十进制定标整数在搬运时即换算成 float，本结构体不再保留其定标形式。
 * 2) 只有 float 精度不够时才保留定点，并逐个写明定标方式，记法如下：
 *      Qm.n          二进制定点：m 位（含符号位）整数部分 + n 位小数，例 Q1.31 = int32 / 2^31；
 *      十进制定标整数  scaled fixed-point，定标 10^-k（不是 Q 格式，须单独注明 k）；
 *      原始 LSB        按物理量定标的整数，注明 1 LSB 对应的物理量。
 *
 * 说明：本结构体目前没有 Qm.n 形式的字段——唯一必须定点的是 GPS 经纬度，它是十进制定标整数；
 *       其余物理量都用 float。若日后引入 Q 格式字段，按上面的记法注明即可。
 */

/* 通道新鲜度：DRDY 时间戳 + 新数据信号 */
typedef struct {
    uint64_t         drdy_tick;  /* DRDY 时刻，单位 10 ns 计数（64 位，非原子；读取方见文件头约定） */
    volatile uint8_t new_data;   /* 新数据信号：1 = 有尚未处理的新数据 */
} v5f_fresh_t;

/* ---- IMU：ICM-42605（SPI DMA 帧，DMA 中断更新） ---- */
typedef struct {
    v5f_fresh_t fresh;           /* drdy_tick 取共享区陀螺通道（V3F 在 DRDY 上升沿先发布时间戳，再触发 SPI 读） */
    int16_t     gyro_lsb[3];     /* 角速度原始 LSB（按物理量定标的整数）：满量程 ±2000 °/s 对应 ±32768，即 16.4 LSB/(°/s) */
    int16_t     accel_lsb[3];    /* 加速度原始 LSB：满量程 ±4 g 对应 ±32768，即 8192 LSB/g */
    float       temp_celsius;    /* 陀螺温度 ℃（换算自 SPI 原始值：132.48 LSB/℃，25 ℃ 为零点） */
    float       quat[4];         /* 姿态四元数 w,x,y,z；解算模块已弃用，恒为 {1,0,0,0} */
} v5f_imu_t;

/* ---- IST8310 磁力计（共享区搬运） ---- */
typedef struct {
    v5f_fresh_t fresh;
    int16_t     lsb[3];          /* 磁场原始 LSB（16 位补码）：0.3 μT/LSB */
} v5f_mag_t;

/* ---- BMP388 气压计（共享区搬运） ---- */
typedef struct {
    v5f_fresh_t fresh;
    float       temp_celsius;    /* 温度 ℃（共享区为 ℃×1000 十进制定标整数） */
    float       press_pascal;    /* 气压 Pa（共享区为 Pa×1000 十进制定标整数） */
} v5f_baro_t;

/* ---- GPS RMC：定位 / 速度（共享区搬运） ---- */
typedef struct {
    v5f_fresh_t fresh;
    int32_t     lat_e7;          /* 纬度：十进制定标整数，1 LSB = 10^-7 °（北正南负），与共享区同格式未转换。
                                    必须定点的理由：float 仅约 7.2 位十进制有效数字，在 ±180° 内
                                    分辨不出 10^-7 °（≈1.1 cm），故不能改用 float。 */
    int32_t     lon_e7;          /* 经度：同上（东正西负） */
    float       speed_mps;       /* 对地速度 m/s（共享区为 cm/s 定标整数，搬运算成 float） */
} v5f_gps_rmc_t;

/* ---- GPS GGA：定位质量 / 高度（共享区搬运） ---- */
typedef struct {
    v5f_fresh_t fresh;
    float       alt_m;           /* 海拔 m（共享区为 cm 定标整数） */
    uint8_t     fix_quality;     /* 定位质量：0=无效 1=单点 2=差分 4=RTK 固定 5=RTK 浮点 */
    uint8_t     sat_num;         /* 使用卫星数 */
    float       hdop;            /* 水平精度因子，无量纲（共享区为 ×100 十进制定标整数） */
} v5f_gps_gga_t;

/* ---- 保持器总成：按传感器分块，每块 = 新鲜度 + 数据 ---- */
typedef struct {
    v5f_imu_t     imu;
    v5f_mag_t     mag;
    v5f_baro_t    baro;
    v5f_gps_rmc_t gps_rmc;
    v5f_gps_gga_t gps_gga;
} v5f_hold_t;

extern volatile v5f_hold_t g_v5f_hold;

/* ---------------- 函数原型 ---------------- */
void SPI_DMA_Init(void);

#endif /* __SPI_RX_H */
