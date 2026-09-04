#ifndef IST8310_H
#define IST8310_H

#include "ch32h417.h"

/* 用户配置：IST8310 7 位地址 + DRDY 引脚（PD14）*/
#define IST8310_DEV7BIT       0x0E
#define IST8310_DRDY_PORT     GPIOD
#define IST8310_DRDY_PIN      GPIO_Pin_14
#define IST8310_DRDY_TIMEOUT  1000000UL   /* 等 DRDY 超时上限（循环计数）*/

/**
 * @brief  IST8310 初始化：软复位 + set/reset 正常脉冲 + 低噪声平均 + DRDY 高有效
 * @note   配置与规格书官方例程一致（PDCNTL/AVGCNTL/CNTL2）
 */
void IST8310_Init(void);

/**
 * @brief  触发单次测量，等 PD14(DRDY) 拉高，一次总线事务连读六轴数据
 * @param  mx,my,mz : 输出三轴磁力计原始值（2 的补码）
 * @retval 0 成功；1 等 DRDY 超时；0xFF I2C 读失败
 */
uint8_t IST8310_Read(int16_t *mx, int16_t *my, int16_t *mz);

#endif /* IST8310_H */
