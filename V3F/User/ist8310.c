#include "ist8310.h"
#include "i2c_soft.h"
#include "bmp388.h"
/**
 * @brief  等待 DRDY 引脚拉高（替代原 Delay_Ms 干等），带超时防死锁
 * @retval 0 就绪；1 超时
 */
static uint8_t IST8310_WaitDRDY(void)
{
    uint32_t t = IST8310_DRDY_TIMEOUT;

    while (GPIO_ReadInputDataBit(IST8310_DRDY_PORT, IST8310_DRDY_PIN) == 0)
        if (--t == 0)
            return 1;

    return 0;
}

/**
 * @brief  IST8310 初始化：软复位 + set/reset 正常脉冲 + 低噪声平均 + DRDY 高有效
 * @note   与规格书官方例程一致：
 *         PDCNTL(0x42)=0xC0  set/reset 脉冲 = Normal（性能优化）
 *         AVGCNTL(0x41)=0x24 Y×4 + X/Z×16 平均（低噪声，官方推荐）
 *         CNTL2(0x0B)=0x0C   DREN=1 + DRP=1（DRDY 高有效）
 */
void IST8310_Init(void)
{
    I2C_WriteReg(IST8310_DEV7BIT, 0x0B, 0x01);   /* CNTL2: SRST=1 软复位（POR 后自动清 0）*/
    Delay_Ms(10);                                 /* 软复位阶段无 DRDY 可等，保留固定延时 */

    I2C_WriteReg(IST8310_DEV7BIT, 0x42, 0xC0);   /* PDCNTL: set/reset 脉冲 = Normal */
    I2C_WriteReg(IST8310_DEV7BIT, 0x41, 0x24);   /* AVGCNTL: Y×4 + X/Z×16 平均 */
    I2C_WriteReg(IST8310_DEV7BIT, 0x0B, 0x0C);   /* CNTL2: DREN=1 + DRP=1（DRDY 高有效）*/
}

/**
 * @brief  触发单次测量，等 PD14(DRDY) 拉高，一次总线事务连读六轴
 * @param  mx,my,mz : 输出三轴磁力计原始值（2 的补码）
 * @retval 0 成功；1 等 DRDY 超时；0xFF I2C 读失败
 * @note   数据寄存器 0x03~0x08 连续（DATAXL..DATAZH，低字节在前）；
 *         读完数据寄存器后 DRDY 自动清 0
 */
uint8_t IST8310_Read(int16_t *mx, int16_t *my, int16_t *mz)
{
    uint8_t d[6];

    I2C_WriteReg(IST8310_DEV7BIT, 0x0A, 0x01);   /* CNTL1: 触发单次测量 */
    if (IST8310_WaitDRDY() != 0)
        return 1;                                 /* DRDY 超时 */

    if (I2C_ReadMulti(IST8310_DEV7BIT, 0x03, d, 6) != 0)
        return 0xFF;                              /* I2C 读失败 */

    *mx = (int16_t)((uint16_t)d[1] << 8 | d[0]);  /* 0x03=XL, 0x04=XH */
    *my = (int16_t)((uint16_t)d[3] << 8 | d[2]);  /* 0x05=YL, 0x06=YH */
    *mz = (int16_t)((uint16_t)d[5] << 8 | d[4]);  /* 0x07=ZL, 0x08=ZH */
    return 0;
}
