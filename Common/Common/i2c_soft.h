#ifndef _I2C_SOFT_H
#define _I2C_SOFT_H

#include <stdint.h>

#include "bmp388.h"

/* ========== 用户配置 ========== */
#define SOFTWARE_I2C_PORT    GPIOD
#define I2C_SCL_PIN          GPIO_Pin_12
#define I2C_SDA_PIN          GPIO_Pin_13

/* I2C 时序常量（单位：10ns）
 * ★ 1/10 极限测试（三个时序常量整体 ×10）：20/5/5 实测约 2.94 MHz，而 IST8310
 *   的 I2C 规格上限是 400 kHz —— 也就是说一直在**超规格约 7 倍**跑。
 *   本次降到约 294 kHz（落回规格内），用来判定"原始磁读数被压弯"（|raw| 到 2.5 倍、
 *   方向塌到近竖直 |raw_z|/|raw| -> 1.0）是否由过速 I2C 造成。
 *   对比指标：全零样本率(个/s)、模长异常率(>25%)、离线方向一致性残差，
 *   以及免标定判据 |raw_z|/|raw| 是否仍在 0.88 附近而不冲向 1.0。
 *   注：更早版本曾注释掉的 125/25/25 约等于 400 kHz。 */
#define I2C_HALF_10NS         200    // 半周期 2000 ns -> 约 294 kHz（原 20 -> 约 2.94 MHz）
#define I2C_SETUP_10NS        50     // 500 ns（原 50 ns）
#define I2C_ACK_DELAY_10NS    50     // 500 ns（原 50 ns）

/* BMP388 */
#define BMP388_DEV7BIT       0x76
#define BMP388_DRDY_PORT     GPIOD
#define BMP388_DRDY_PIN      GPIO_Pin_15
#define BMP388_DRDY_TIMEOUT  1000000UL

/* IST8310 */
#define IST8310_DEV7BIT      0x0E
#define IST8310_DRDY_PORT    GPIOD
#define IST8310_DRDY_PIN     GPIO_Pin_14
#define IST8310_DRDY_TIMEOUT 1000000UL
/* ============================== */

/* ----- I2C 基础 ----- */
void SOFT_I2C_GPIO_Init(void);
uint8_t I2C_ReadReg(uint8_t dev7bit, uint8_t reg);
uint8_t I2C_ReadMulti(uint8_t dev7bit, uint8_t reg, uint8_t *buf, uint16_t len);
void I2C_WriteReg(uint8_t dev7bit, uint8_t reg, uint8_t data);
void DumpRegTable(uint8_t dev7bit, uint8_t endReg);

/* ----- 传感器初始化 ----- */
void IST8310_Init(void);
void BMP388_Init(void);

/* ----- 触发传感器测量 ----- */
void IST8310_TriggerMeasurement(void);
void BMP388_TriggerMeasurement(void);

/* ----- 传感器读取（假定DRDY已经就绪） ----- */
uint8_t IST8310_Read(int16_t *mx, int16_t *my, int16_t *mz);
uint8_t BMP388_Read(float *temp_c, float *press_pa);   /* 直接调用 bmp388 库的补偿函数 */

/* ----- 等待传感器DRDY（返回实际等待 us；>= timeout_us 视为超时且不写 out_ts_drdy；
 *        out_ts_drdy 为 DRDY 就绪时刻，单位 10 ns 计数） ----- */
uint32_t IST8310_WaitDRDY(uint32_t timeout_us, uint64_t *out_ts_drdy);
uint32_t BMP388_WaitDRDY(uint32_t timeout_us, uint64_t *out_ts_drdy);

/* ----- 全局外设初始化（含两个传感器） ----- */
void soft_i2c_silm_Init(void);

#endif