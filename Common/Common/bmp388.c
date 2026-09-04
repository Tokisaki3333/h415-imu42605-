#include "bmp388.h"
#include "i2c_soft.h"

/**
 * @brief BMP388温度补偿，NVM取自头文件实测宏，字节序严格对齐datasheet Table23
 * @param uncomp_temp 24bit原始ADC温度
 * @return t_lin 线性化温度(℃)
 */
float bmp388_compensate_temp(uint32_t uncomp_temp)
{
    float pd1 = (float)uncomp_temp - ((float)NVM_T1 * T1_scale);
    float pd2 = pd1 * ((float)NVM_T2 * T2_scale_inv);
    return pd2 + (pd1 * pd1) * ((float)NVM_T3 * T3_scale_inv);
}

/**
 * @brief BMP388压力补偿
 * @param uncomp_press 24bit原始ADC压力
 * @param t_lin 由bmp388_compensate_temp返回的线性温度
 * @return 补偿后压力，单位 Pa
 */
float bmp388_compensate_press(uint32_t uncomp_press, float t_lin)
{
    float t2 = t_lin * t_lin;
    float t3 = t2 * t_lin;

    float po1  =  ((float)NVM_P5 * P5_scale)
                + ((float)NVM_P6 * P6_scale_inv) * t_lin
                + ((float)NVM_P7 * P7_scale_inv) * t2
                + ((float)NVM_P8 * P8_scale_inv) * t3;

    float term_p1 = ((float)NVM_P1 - BMP388_P1_OFFSET) * P1_scale_inv;
    float term_p2 = ((float)NVM_P2 - BMP388_P1_OFFSET) * P2_scale_inv;
    float term_p3 = ((float)NVM_P3 * P3_scale_inv);
    float term_p4 = ((float)NVM_P4 * P4_scale_inv);

    float po2 = (float)uncomp_press * ( term_p1 + term_p2 * t_lin + term_p3 * t2 + term_p4 * t3 );

    float up2 = (float)uncomp_press * (float)uncomp_press;
    float up3 = up2 * (float)uncomp_press;

    float pdB2 = ((float)NVM_P9 * P9_scale_inv) + ((float)NVM_P10 * P10_scale_inv) * t_lin;
    float pdB3 = up2 * pdB2;
    float pdB4 = pdB3 + up3 * ((float)NVM_P11 * P11_scale_inv);

    return po1 + po2 + pdB4;
}

int BMP388_LinkTest_ReadRaw(uint8_t dev7bit,
                            uint32_t* out_uncomp_press,
                            uint32_t* out_uncomp_temp,
                            uint8_t bPrintTable)
{
    uint8_t chip_id = I2C_ReadReg(dev7bit, 0x00);
    if(chip_id != 0x50)
    {
        return -1;
    }

    uint8_t d[6];   /* 0x04~0x09：P_XLSB..P_MSB, T_XLSB..T_MSB，一次事务连读 */

    if(bPrintTable)
    {
        DumpRegTable(dev7bit, 0x0F);
    }

    /* 一次总线事务连续读压力+温度共 6 字节（寄存器地址自动递增）*/
    if (I2C_ReadMulti(dev7bit, 0x04, d, 6) != 0)
    {
        return -1;
    }

    *out_uncomp_press  = ((uint32_t)d[2] << 16U) | ((uint32_t)d[1] << 8U) | d[0];
    *out_uncomp_temp   = ((uint32_t)d[5] << 16U) | ((uint32_t)d[4] << 8U) | d[3];

    return 0;
}

int BMP388_ReadCompensated(uint8_t dev7bit, float *out_temp_c, float *out_press_pa, uint8_t bPrintTable)
{
    uint32_t raw_press, raw_temp;
    int ret = BMP388_LinkTest_ReadRaw(dev7bit, &raw_press, &raw_temp, bPrintTable);
    if(ret != 0)
    {
        return ret;
    }

    float t_lin = bmp388_compensate_temp(raw_temp);
    float press_pa = bmp388_compensate_press(raw_press, t_lin);

    *out_temp_c = t_lin;
    *out_press_pa = press_pa;

    return 0;
}
