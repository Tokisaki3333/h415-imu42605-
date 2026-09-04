#ifndef BMP388_H
#define BMP388_H

#include "ch32h417.h"

/* 数学缩放系数宏，datasheet附录9.1固定值 */
#define T1_scale         2.560000000000e+02f
#define T2_scale_inv     9.313225746155e-10f
#define T3_scale_inv     3.552713678801e-15f

#define P1_scale_inv     9.536743164062e-07f
#define P2_scale_inv     1.862645149231e-09f
#define P3_scale_inv     2.328306436539e-10f
#define P4_scale_inv     7.275957614183e-12f

#define P5_scale         8.000000000000e+00f
#define P6_scale_inv     1.562500000000e-02f
#define P7_scale_inv     3.906250000000e-03f
#define P8_scale_inv     3.051757812500e-05f

#define P9_scale_inv     3.552713678801e-15f
#define P10_scale_inv    3.552713678801e-15f
#define P11_scale_inv    2.710505431214e-20f

#define BMP388_P1_OFFSET 16384.0f   /* 2^14 */

#define NVM_T1      0x6BF2U
#define NVM_T2      0x48D6U
#define NVM_T3      ((int8_t)0xF6)

#define NVM_P1      ((int16_t)0xFF92)
#define NVM_P2      ((int16_t)0xF5D7)
#define NVM_P3      ((int8_t)0x23)
#define NVM_P4      ((int8_t)0x00)

#define NVM_P5      0x6427U
#define NVM_P6      0x78AAU

#define NVM_P7      ((int8_t)0xF3)
#define NVM_P8      ((int8_t)0xF6)

#define NVM_P9      ((int16_t)0x3FF3)
#define NVM_P10     ((int8_t)0x11)
#define NVM_P11     ((int8_t)0xC4)

/**
 * @brief 读取原始ADC并自动执行全套补偿
 * @param dev7bit 器件7bitI2C地址
 * @param out_temp_c 输出补偿后温度，单位℃
 * @param out_press_pa 输出补偿后压力，单位Pa
 * @param bPrintTable 1打印寄存器dump，0关闭
 * @retval 0成功；-1芯片ID错误
 */
int BMP388_ReadCompensated(uint8_t dev7bit, float *out_temp_c, float *out_press_pa, uint8_t bPrintTable);

float bmp388_compensate_temp(uint32_t uncomp_temp);
float bmp388_compensate_press(uint32_t uncomp_press, float t_lin);
int BMP388_LinkTest_ReadRaw(uint8_t dev7bit,
                            uint32_t* out_uncomp_press,
                            uint32_t* out_uncomp_temp,
                            uint8_t bPrintTable);

#endif
