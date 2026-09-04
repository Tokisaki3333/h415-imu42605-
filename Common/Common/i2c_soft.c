#include "i2c_soft.h"
#include "hardware.h"
#include "debug.h"
#include "sys_clk.h"
#include <stdio.h>

/* ==================== 内部宏 ==================== */
#define I2C_SCL_HIGH()   GPIO_SetBits(SOFTWARE_I2C_PORT, I2C_SCL_PIN)
#define I2C_SCL_LOW()    GPIO_ResetBits(SOFTWARE_I2C_PORT, I2C_SCL_PIN)
#define I2C_SDA_HIGH()   GPIO_SetBits(SOFTWARE_I2C_PORT, I2C_SDA_PIN)
#define I2C_SDA_LOW()    GPIO_ResetBits(SOFTWARE_I2C_PORT, I2C_SDA_PIN)
#define I2C_READ_SDA()   GPIO_ReadInputDataBit(SOFTWARE_I2C_PORT, I2C_SDA_PIN)

/* ==================== I2C GPIO ==================== */
void SOFT_I2C_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure = {0};
    RCC_HB2PeriphClockCmd(RCC_HB2Periph_GPIOD, ENABLE);

    GPIO_InitStructure.GPIO_Pin   = I2C_SCL_PIN | I2C_SDA_PIN;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_OD;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_Very_High;
    GPIO_Init(SOFTWARE_I2C_PORT, &GPIO_InitStructure);

    I2C_SCL_HIGH();
    I2C_SDA_HIGH();
}

/* ==================== I2C 时序（基于 Delay_10Ns） ==================== */

static void I2C_Start(void)
{
    I2C_SDA_HIGH();
    Delay_10Ns(I2C_HALF_10NS);
    I2C_SCL_HIGH();
    Delay_10Ns(I2C_HALF_10NS);
    I2C_SDA_LOW();
    Delay_10Ns(I2C_HALF_10NS);
    I2C_SCL_LOW();
    Delay_10Ns(I2C_HALF_10NS);
}

static void I2C_Stop(void)
{
    I2C_SDA_LOW();
    Delay_10Ns(I2C_HALF_10NS);
    I2C_SCL_HIGH();
    Delay_10Ns(I2C_HALF_10NS);
    I2C_SDA_HIGH();
    Delay_10Ns(I2C_HALF_10NS);
}

static uint8_t I2C_SendByte(uint8_t data)
{
    uint8_t i, ack;
    for (i = 0; i < 8; i++) {
        if (data & 0x80) I2C_SDA_HIGH();
        else I2C_SDA_LOW();
        data <<= 1;
        Delay_10Ns(I2C_SETUP_10NS);                     // 数据建立
        I2C_SCL_HIGH();
        Delay_10Ns(I2C_HALF_10NS);                      // 时钟高电平
        I2C_SCL_LOW();
        Delay_10Ns(I2C_HALF_10NS - I2C_SETUP_10NS);     // 时钟低电平 - 建立时间
    }
    I2C_SDA_HIGH();                                     // 释放 SDA
    Delay_10Ns(I2C_ACK_DELAY_10NS);                     // 原硬编码 5us 改为 250ns
    I2C_SCL_HIGH();
    Delay_10Ns(I2C_HALF_10NS);
    ack = I2C_READ_SDA();
    I2C_SCL_LOW();
    Delay_10Ns(I2C_HALF_10NS);
    return ack;
}

static uint8_t I2C_RecvByte(uint8_t ack_en)
{
    uint8_t i, data = 0;
    I2C_SDA_HIGH();                                     // 释放总线
    for (i = 0; i < 8; i++) {
        data <<= 1;
        I2C_SCL_HIGH();
        Delay_10Ns(I2C_HALF_10NS);
        if (I2C_READ_SDA()) data |= 0x01;
        I2C_SCL_LOW();
        Delay_10Ns(I2C_HALF_10NS);
    }
    if (ack_en) I2C_SDA_LOW();
    else I2C_SDA_HIGH();
    Delay_10Ns(I2C_SETUP_10NS);                         // 应答建立
    I2C_SCL_HIGH();
    Delay_10Ns(I2C_HALF_10NS);
    I2C_SCL_LOW();
    Delay_10Ns(I2C_HALF_10NS);
    I2C_SDA_HIGH();
    return data;
}

/* ==================== 寄存器操作 ==================== */
uint8_t I2C_ReadReg(uint8_t dev7bit, uint8_t reg)
{
    uint8_t val;
    I2C_Start();
    if (I2C_SendByte((dev7bit << 1) | 0x00) != 0) { I2C_Stop(); return 0xFF; }
    if (I2C_SendByte(reg) != 0) { I2C_Stop(); return 0xFF; }
    I2C_Start();
    if (I2C_SendByte((dev7bit << 1) | 0x01) != 0) { I2C_Stop(); return 0xFF; }
    val = I2C_RecvByte(0);
    I2C_Stop();
    return val;
}

uint8_t I2C_ReadMulti(uint8_t dev7bit, uint8_t reg, uint8_t *buf, uint16_t len)
{
    uint16_t i;
    if (!buf || len == 0) return 0xFF;
    I2C_Start();
    if (I2C_SendByte((dev7bit << 1) | 0x00) != 0) { I2C_Stop(); return 0xFF; }
    if (I2C_SendByte(reg) != 0) { I2C_Stop(); return 0xFF; }
    I2C_Start();
    if (I2C_SendByte((dev7bit << 1) | 0x01) != 0) { I2C_Stop(); return 0xFF; }
    for (i = 0; i < len; i++)
        buf[i] = I2C_RecvByte(i + 1 < len);
    I2C_Stop();
    return 0;
}

void I2C_WriteReg(uint8_t dev7bit, uint8_t reg, uint8_t data)
{
    I2C_Start();
    if (I2C_SendByte((dev7bit << 1) | 0x00) != 0) { I2C_Stop(); return; }
    if (I2C_SendByte(reg) != 0) { I2C_Stop(); return; }
    I2C_SendByte(data);
    I2C_Stop();
}

void DumpRegTable(uint8_t dev7bit, uint8_t endReg)
{
    uint8_t col, addr;
    printf("      ");
    for (col = 0; col < 16; col++) printf("%02X ", col);
    printf("\r\n");
    for (addr = 0; addr <= endReg; addr += 16) {
        printf("0x%02X: ", addr);
        for (col = 0; col < 16; col++) {
            if (addr + col <= endReg)
                printf("%02X ", I2C_ReadReg(dev7bit, addr + col));
            else
                printf("   ");
        }
        printf("\r\n");
    }
}

/* ==================== 传感器辅助 ==================== */

uint32_t IST8310_WaitDRDY(uint32_t timeout_us, uint64_t *out_ts_drdy)
{
    uint64_t t0      = GetTime64_Us();                /* 起始时刻 */
    uint64_t deadline = t0 + timeout_us;
    while (GPIO_ReadInputDataBit(IST8310_DRDY_PORT, IST8310_DRDY_PIN) == 0) {
        Delay_Us(1);
        if (GetTime64_Us() >= deadline) {
            return (uint32_t)(GetTime64_Us() - t0);   /* 超时：不写时间戳，返回满等待 */
        }
    }
    if (out_ts_drdy) *out_ts_drdy = GetTime64_Us();   /* DRDY 就绪时刻（紧贴检测到高）*/
    return (uint32_t)(GetTime64_Us() - t0);           /* 实际等待 us */
}

uint32_t BMP388_WaitDRDY(uint32_t timeout_us, uint64_t *out_ts_drdy)
{
    /* 轮询 STATUS(0x03) 的 drdy_press(bit5)|drdy_temp(bit6)，不依赖 INT 引脚（DRDY 未连接）。
     * STATUS 仅 bit4/5/6 有定义（保留位读 0），0xFF 必为 I2C 读失败（SDA 无 ACK 上拉全 1），
     * 不能当作就绪——否则转换未完成就读取，会读到上一帧旧数据（偶发连续两帧相同）。*/
    uint64_t t0      = GetTime64_Us();                /* 起始时刻 */
    uint64_t deadline = t0 + timeout_us;
    for (;;) {
        uint8_t st = I2C_ReadReg(BMP388_DEV7BIT, 0x03);
        if (st != 0xFF && (st & 0x60)) break;         /* 有效读取且 drdy 置位才算就绪 */
        Delay_Us(1);
        if (GetTime64_Us() >= deadline) {
            return (uint32_t)(GetTime64_Us() - t0);   /* 超时：不写时间戳，返回满等待 */
        }
    }
    if (out_ts_drdy) *out_ts_drdy = GetTime64_Us();   /* DRDY 就绪时刻（紧贴轮询到 drdy）*/
    return (uint32_t)(GetTime64_Us() - t0);           /* 实际等待 us */
}
/* ==================== 传感器初始化 ==================== */
void IST8310_Init(void)
{
    I2C_WriteReg(IST8310_DEV7BIT, 0x0B, 0x01);
    Delay_Ms(10);
    I2C_WriteReg(IST8310_DEV7BIT, 0x42, 0xC0);
    I2C_WriteReg(IST8310_DEV7BIT, 0x41, 0x24);
    I2C_WriteReg(IST8310_DEV7BIT, 0x0B, 0x0C);
}

void BMP388_Init(void)
{
    /* INT_CTRL: drdy_en + 高有效 + 锁存（0x46 = bit6 drdy_en | bit1 int_level 高有效 | bit2 int_latch 锁存）
     * 必须用锁存模式：非锁存下 DRDY 引脚只保持 2.5ms 脉冲，主循环是"同时触发、错峰读取"
     * （先等/读 IST8310 约 7ms 才轮到 BMP388），极易错过窗口；
     * 锁存后 DRDY 保持高电平，直到读取数据寄存器(0x04~0x09)才清除，等待函数在任何时序下都可靠。 */
    I2C_WriteReg(BMP388_DEV7BIT, 0x19, 0x46);
    /* 过采样 ×1（默认）。实测 ×8 后相邻读数相同更频繁——证实"连续相同"是真实读数量化所致
     * （高过采样更平滑），非读取错误；×1 的偶发两帧相同属正常现象，保持最低开销。 */
    I2C_WriteReg(BMP388_DEV7BIT, 0x1C, 0x00);
}

/* ==================== 全局初始化（含两个传感器） ==================== */
void soft_i2c_silm_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure = {0};

    SOFT_I2C_GPIO_Init();

    RCC_HB2PeriphClockCmd(RCC_HB2Periph_GPIOD, ENABLE);
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_14 | GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IPU;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_Very_High;
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    BMP388_Init();
    IST8310_Init();
}

/* ==================== 触发测量（对称命名） ==================== */

void IST8310_TriggerMeasurement(void)
{
    I2C_WriteReg(IST8310_DEV7BIT, 0x0A, 0x01);   /* 单次测量 */
}

void BMP388_TriggerMeasurement(void)
{
    I2C_WriteReg(BMP388_DEV7BIT, 0x1B, 0x13);    /* 强制模式，压力+温度 */
}

/* ==================== 读取数据（仅读，假定 DRDY 已就绪） ==================== */

uint8_t IST8310_Read(int16_t *mx, int16_t *my, int16_t *mz)
{
    uint8_t d[6];
    if (I2C_ReadMulti(IST8310_DEV7BIT, 0x03, d, 6) != 0) return 0xFF;
    *mx = (int16_t)((uint16_t)d[1] << 8 | d[0]);
    *my = (int16_t)((uint16_t)d[3] << 8 | d[2]);
    *mz = (int16_t)((uint16_t)d[5] << 8 | d[4]);
    return 0;
}

uint8_t BMP388_Read(float *temp_c, float *press_pa)
{
    /* BMP388_ReadCompensated 不触发、不等待，只补偿计算 */
    int ret = BMP388_ReadCompensated(BMP388_DEV7BIT, temp_c, press_pa, 0);
    return (ret == 0) ? 0 : 0xFF;
}
