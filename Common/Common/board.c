#include "hardware.h"

/* 软件 SPI —— 按网表实际连接：PF4=CS, PF5=SCK, PD7=MOSI, PF3=MISO */
#define SPI_CS_PIN     GPIO_Pin_4    /* GPIOF */
#define SPI_SCK_PIN    GPIO_Pin_5    /* GPIOF */
#define SPI_MOSI_PIN   GPIO_Pin_7    /* GPIOD */
#define SPI_MISO_PIN   GPIO_Pin_3    /* GPIOF */

void SPI_Init1(void)
{
    GPIO_InitTypeDef g = {0};
    RCC_HB2PeriphClockCmd(RCC_HB2Periph_GPIOF | RCC_HB2Periph_GPIOD, ENABLE);

    g.GPIO_Speed = GPIO_Speed_Very_High;
    g.GPIO_Mode = GPIO_Mode_Out_PP;
    g.GPIO_Pin = SPI_CS_PIN | SPI_SCK_PIN;
    GPIO_Init(GPIOF, &g);
    g.GPIO_Pin = SPI_MOSI_PIN;
    GPIO_Init(GPIOD, &g);

    g.GPIO_Mode = GPIO_Mode_IPU;          /* MISO 上拉输入 */
    g.GPIO_Pin = SPI_MISO_PIN;
    GPIO_Init(GPIOF, &g);

    GPIO_SetBits(GPIOF, SPI_CS_PIN);      /* CS 空闲高 */
    GPIO_ResetBits(GPIOF, SPI_SCK_PIN);   /* mode0：SCK 空闲低 */
}

/* 一个字节收发：SCK 空闲低、上升沿采样（mode 0），半周期 1us */
static uint8_t SPI_RW(uint8_t byte)
{
    uint8_t i, rx = 0;
    for (i = 0; i < 8; i++) {
        GPIO_ResetBits(GPIOF, SPI_SCK_PIN);
        if (byte & 0x80) GPIO_SetBits(GPIOD, SPI_MOSI_PIN);
        else             GPIO_ResetBits(GPIOD, SPI_MOSI_PIN);
        byte <<= 1;
        Delay_Us(1);
        GPIO_SetBits(GPIOF, SPI_SCK_PIN); /* 上升沿：从机输出数据 */
        Delay_Us(1);
        rx <<= 1;
        if (GPIO_ReadInputDataBit(GPIOF, SPI_MISO_PIN)) rx |= 1;
        Delay_Us(1);
    }
    return rx;
}

uint8_t SPI_ReadReg(uint8_t reg)
{
    uint8_t v;
    GPIO_ResetBits(GPIOF, SPI_CS_PIN);
    Delay_Us(1);
    SPI_RW(reg | 0x80);        /* 读命令：bit7=1 */
    v = SPI_RW(0x00);
    Delay_Us(1);
    GPIO_SetBits(GPIOF, SPI_CS_PIN);
    return v;
}

/* 读全部寄存器 0x00~0x7F 并打印 */
void DumpAllRegs(void)
{
    uint8_t a, i;
    printf("      ");
    for (i = 0; i < 16; i++) printf("%02X ", i);
    printf("\r\n");
    for (a = 0; a < 0x80; a += 16) {
        printf("0x%02X: ", a);
        for (i = 0; i < 16; i++)
            printf("%02X ", SPI_ReadReg(a + i));
        printf("\r\n");
    }
}

void SPI_WriteReg(uint8_t reg, uint8_t data)
{
    GPIO_ResetBits(GPIOF, SPI_CS_PIN);
    Delay_Us(1);
    SPI_RW(reg & 0x7F);      /* 写命令：bit7=0 */
    SPI_RW(data);
    Delay_Us(1);
    GPIO_SetBits(GPIOF, SPI_CS_PIN);
}

void I2C_GPIO_Init (void);
uint8_t I2C_ReadReg (uint8_t dev7bit, uint8_t reg);
void I2C_WriteReg (uint8_t dev7bit, uint8_t reg, uint8_t data);

#define SOFTWARE_I2C_PORT GPIOD
#define I2C_SCL_PIN GPIO_Pin_12
#define I2C_SDA_PIN GPIO_Pin_13

#define I2C_SCL_HIGH() GPIO_SetBits (SOFTWARE_I2C_PORT, I2C_SCL_PIN)
#define I2C_SCL_LOW() GPIO_ResetBits (SOFTWARE_I2C_PORT, I2C_SCL_PIN)
#define I2C_SDA_HIGH() GPIO_SetBits (SOFTWARE_I2C_PORT, I2C_SDA_PIN)
#define I2C_SDA_LOW() GPIO_ResetBits (SOFTWARE_I2C_PORT, I2C_SDA_PIN)
#define I2C_READ_SDA() GPIO_ReadInputDataBit (SOFTWARE_I2C_PORT, I2C_SDA_PIN)

#define I2C_DELAY_HALF 5
#define I2C_DELAY_SETUP 1

// Delay_Us 已在 debug.c 中定义，此处无需重复

void I2C_GPIO_Init (void) {
    GPIO_InitTypeDef GPIO_InitStructure = {0};
    RCC_HB2PeriphClockCmd (RCC_HB2Periph_GPIOD, ENABLE);

    GPIO_InitStructure.GPIO_Pin = I2C_SCL_PIN | I2C_SDA_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_Very_High;
    GPIO_Init (SOFTWARE_I2C_PORT, &GPIO_InitStructure);

    I2C_SCL_HIGH();
    I2C_SDA_HIGH();
}

static void I2C_Start (void) {
    I2C_SDA_HIGH();
    Delay_Us (I2C_DELAY_HALF);
    I2C_SCL_HIGH();
    Delay_Us (I2C_DELAY_HALF);
    I2C_SDA_LOW();
    Delay_Us (I2C_DELAY_HALF);
    I2C_SCL_LOW();
    Delay_Us (I2C_DELAY_HALF);
}

static void I2C_Stop (void) {
    I2C_SDA_LOW();
    Delay_Us (I2C_DELAY_HALF);
    I2C_SCL_HIGH();
    Delay_Us (I2C_DELAY_HALF);
    I2C_SDA_HIGH();
    Delay_Us (I2C_DELAY_HALF);
}

static uint8_t I2C_SendByte (uint8_t data) {
    uint8_t i, ack;

    for (i = 0; i < 8; i++) {
        if (data & 0x80)
            I2C_SDA_HIGH();
        else
            I2C_SDA_LOW();
        data <<= 1;

        Delay_Us (I2C_DELAY_SETUP);

        I2C_SCL_HIGH();
        Delay_Us (I2C_DELAY_HALF);

        I2C_SCL_LOW();
        Delay_Us (I2C_DELAY_HALF - I2C_DELAY_SETUP);
    }

    I2C_SDA_HIGH();
    Delay_Us (5);  // ★ 唯一修改：1us → 5us
    I2C_SCL_HIGH();
    Delay_Us (I2C_DELAY_HALF);
    ack = I2C_READ_SDA();
    I2C_SCL_LOW();
    Delay_Us (I2C_DELAY_HALF);

    return ack;
}

static uint8_t I2C_RecvByte (uint8_t ack_en) {
    uint8_t i, data = 0;

    I2C_SDA_HIGH();

    for (i = 0; i < 8; i++) {
        data <<= 1;
        I2C_SCL_HIGH();
        Delay_Us (I2C_DELAY_HALF);
        if (I2C_READ_SDA())
            data |= 0x01;
        I2C_SCL_LOW();
        Delay_Us (I2C_DELAY_HALF);
    }

    if (ack_en)
        I2C_SDA_LOW();
    else
        I2C_SDA_HIGH();

    Delay_Us (I2C_DELAY_SETUP);
    I2C_SCL_HIGH();
    Delay_Us (I2C_DELAY_HALF);
    I2C_SCL_LOW();
    Delay_Us (I2C_DELAY_HALF);
    I2C_SDA_HIGH();

    return data;
}

uint8_t I2C_ReadReg (uint8_t dev7bit, uint8_t reg) {
    uint8_t val;

    I2C_Start();
    if (I2C_SendByte ((dev7bit << 1) | 0x00) != 0) {
        I2C_Stop();
        return 0xFF;
    }
    if (I2C_SendByte (reg) != 0) {
        I2C_Stop();
        return 0xFF;
    }

    I2C_Start();
    if (I2C_SendByte ((dev7bit << 1) | 0x01) != 0) {
        I2C_Stop();
        return 0xFF;
    }
    val = I2C_RecvByte (0);
    I2C_Stop();
    return val;
}

void I2C_WriteReg (uint8_t dev7bit, uint8_t reg, uint8_t data) {
    I2C_Start();
    if (I2C_SendByte ((dev7bit << 1) | 0x00) != 0) {
        I2C_Stop();
        return;
    }
    if (I2C_SendByte (reg) != 0) {
        I2C_Stop();
        return;
    }
    I2C_SendByte (data);
    I2C_Stop();
}

/* 打印指定设备寄存器总表：从 0x00 扫到 endReg，每行 16 字节 */
void DumpRegTable (uint8_t dev7bit, uint8_t endReg) {
    uint8_t col, addr;

    printf (" ");
    for (col = 0; col < 16; col++) printf ("%02X ", col);
    printf ("\r\n");
    for (addr = 0; addr <= endReg; addr += 16) {
        printf ("0x%02X: ", addr);
        for (col = 0; col < 16; col++) {
            if (addr + col <= endReg)
                printf ("%02X ", I2C_ReadReg (dev7bit, addr + col));
            else
                printf (" ");
        }
        printf ("\r\n");
    }
}
/* IST8310 @0x0E 激活（官方 v1.2） */
void IST8310_Init(void)
{
    I2C_WriteReg(0x0E, 0x0B, 0x01);   /* CNTL2: SRST=1 软复位，POR 后自动清 0 */
    Delay_Ms(10);

    I2C_WriteReg(0x0E, 0x42, 0xC0);   /* PDCNTL: set/reset 脉冲 = Normal（性能优化）*/
    I2C_WriteReg(0x0E, 0x41, 0x24);   /* AVGCNTL: Y×4 + X/Z×16 平均（低噪声，官方推荐）*/
    I2C_WriteReg(0x0E, 0x0B, 0x0C);   /* CNTL2: DREN=1 + DRP=1（DRDY 高有效）*/
}

/* 触发一次单次测量并读三轴（含超时保护）*/
uint8_t IST8310_Read(int16_t *mx, int16_t *my, int16_t *mz)
{
    uint32_t timeout = 1000;

    I2C_WriteReg(0x0E, 0x0A, 0x01);                    /* CNTL1: 触发单次测量 */
    while ((I2C_ReadReg(0x0E, 0x02) & 0x01) == 0) {    /* 等 STAT1.DRDY */
        if (--timeout == 0) return 1;                  /* 超时返回失败 */
    }
    *mx = (int16_t)((I2C_ReadReg(0x0E, 0x04) << 8) | I2C_ReadReg(0x0E, 0x03));
    *my = (int16_t)((I2C_ReadReg(0x0E, 0x06) << 8) | I2C_ReadReg(0x0E, 0x05));
    *mz = (int16_t)((I2C_ReadReg(0x0E, 0x08) << 8) | I2C_ReadReg(0x0E, 0x07));
    return 0;                                          /* 读数据后 DRDY 自动清 0 */
}
void BMP388_Measure(void)
{
    /* 配置：OSR x4/x1（0x02），ODR 无所谓（forced 不受 odr_sel 约束）*/
    I2C_WriteReg(0x76, 0x1C, 0x02);

    /* 触发：press_en + temp_en + mode=01(forced) */
    I2C_WriteReg(0x76, 0x1B, 0x13);
    Delay_Ms(15);                       /* x4 测量 typ 10.9ms / max 12.5ms，留裕量 */

    /* 读原始值：压力 0x04~0x06（XLSB/LSB/MSB），温度 0x07~0x09 */
    uint8_t p0 = I2C_ReadReg(0x76, 0x04), p1 = I2C_ReadReg(0x76, 0x05), p2 = I2C_ReadReg(0x76, 0x06);
    uint8_t t0 = I2C_ReadReg(0x76, 0x07), t1 = I2C_ReadReg(0x76, 0x08), t2 = I2C_ReadReg(0x76, 0x09);
    int32_t press_raw = ((int32_t)p2 << 16) | ((int32_t)p1 << 8) | p0;   /* 20 位有效 */
    int32_t temp_raw  = ((int32_t)t2 << 16) | ((int32_t)t1 << 8) | t0;

    printf("P_raw=%ld  T_raw=%ld\r\n", (long)press_raw, (long)temp_raw);
}
void MAG_Loop(void)
{
    int16_t mx, my, mz;

    IST8310_Init();
    while (1) {
        if (IST8310_Read(&mx, &my, &mz) == 0) {
            printf("MAG: %d %d %d\r\n", mx, my, mz);
        }
        Delay_Ms(2);   /* 低噪声配置最小间隔 6ms，配合触发+等待实际 ~166Hz */
    }
}
/* 三根 DRDY 输入初始化：PD14=IST8310, PD15=BMP388, PE0=42605 */
void Sensor_INT_Init(void)
{
    GPIO_InitTypeDef g = {0};
    RCC_HB2PeriphClockCmd(RCC_HB2Periph_GPIOD | RCC_HB2Periph_GPIOE, ENABLE);

    g.GPIO_Mode = GPIO_Mode_IPU;
    g.GPIO_Speed = GPIO_Speed_Very_High;
    g.GPIO_Pin = GPIO_Pin_14 | GPIO_Pin_15;
    GPIO_Init(GPIOD, &g);
    g.GPIO_Pin = GPIO_Pin_0;
    GPIO_Init(GPIOE, &g);
}

/* IST8310：基线 + 捕获标志 + 先看引脚再读数据 */
void IST8310_Verify_Once(void)
{
    uint32_t t = 200000;
    int16_t mx, my, mz;
    uint8_t caught;

    printf("IST8310 base PD14=%d\r\n", (int)GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_14)); /* 触发前基线：应为 0 */

    I2C_WriteReg(0x0E, 0x0A, 0x01);                    /* 触发单次测量 */
    while (GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_14) == 0)
        if (--t == 0) break;
    caught = (t > 0);

    printf("IST8310 DRDY caught=%d  now PD14=%d\r\n",  /* 读数据前先看：抓到时应为 1 */
           caught, (int)GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_14));

    mx = (int16_t)((I2C_ReadReg(0x0E, 0x04) << 8) | I2C_ReadReg(0x0E, 0x03));
    my = (int16_t)((I2C_ReadReg(0x0E, 0x06) << 8) | I2C_ReadReg(0x0E, 0x05));
    mz = (int16_t)((I2C_ReadReg(0x0E, 0x08) << 8) | I2C_ReadReg(0x0E, 0x07));
    printf("IST8310: mx=%d my=%d mz=%d\r\n", mx, my, mz);
}

/* BMP388：同样加基线 + 捕获标志，另外用 STATUS 寄存器兜底（兼容品 INT 引脚可能未实现）*/
void BMP388_Verify_Once(void)
{
    uint32_t t = 200000;
    uint8_t p0, p1, p2, t0, t1, t2, caught = 0;

    printf("BMP388 base PD15=%d\r\n", (int)GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_15));
    I2C_WriteReg(0x76, 0x19, 0x42);                   /* INT_CTRL: drdy_en + 高有效 */
    I2C_WriteReg(0x76, 0x1C, 0x02);
    I2C_WriteReg(0x76, 0x1B, 0x13);                   /* 触发 forced */

    while (GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_15) == 0)
        if (--t == 0) break;
    caught = (t > 0);

    /* 兜底：STATUS(0x03) bit5 = drdy_press（寄存器轮询，不依赖 INT 引脚）*/
    t = 200000;
    while ((I2C_ReadReg(0x76, 0x03) & 0x20) == 0)
        if (--t == 0) break;

    printf("BMP388 PD15 caught=%d  now=%d  reg_drdy=%d\r\n",
           caught,
           (int)GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_15),
           (I2C_ReadReg(0x76, 0x03) & 0x20) ? 1 : 0);

    p0 = I2C_ReadReg(0x76, 0x04); p1 = I2C_ReadReg(0x76, 0x05); p2 = I2C_ReadReg(0x76, 0x06);
    t0 = I2C_ReadReg(0x76, 0x07); t1 = I2C_ReadReg(0x76, 0x08); t2 = I2C_ReadReg(0x76, 0x09);
    printf("BMP388: P=0x%02X%02X%02X T=0x%02X%02X%02X\r\n", p2, p1, p0, t2, t1, t0);
}
/* 3) 42605：轮询 INT_STATUS(0x2D) DRDY → 读六轴（大端），顺带看 PE0 */
void ICM42605_Verify_Once(void)
{
    uint32_t t = 200000;
    int16_t ax, ay, az, gx, gy, gz;

    while ((SPI_ReadReg(0x2D) & 0x08) == 0)   /* 0x2D bit3 = DRDY */
        if (--t == 0) break;

    ax = (int16_t)((SPI_ReadReg(0x1F) << 8) | SPI_ReadReg(0x20));
    ay = (int16_t)((SPI_ReadReg(0x21) << 8) | SPI_ReadReg(0x22));
    az = (int16_t)((SPI_ReadReg(0x23) << 8) | SPI_ReadReg(0x24));
    gx = (int16_t)((SPI_ReadReg(0x25) << 8) | SPI_ReadReg(0x26));
    gy = (int16_t)((SPI_ReadReg(0x27) << 8) | SPI_ReadReg(0x28));
    gz = (int16_t)((SPI_ReadReg(0x29) << 8) | SPI_ReadReg(0x2A));
    printf("42605: a=%d %d %d  g=%d %d %d  PE0=%d\r\n",
           ax, ay, az, gx, gy, gz,
           (int)GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_0));
}

/* 三个传感器各验证一次 */
void Sensor_Verify_Once(void)
{
    Sensor_INT_Init();
    BMP388_Verify_Once();
    IST8310_Verify_Once();
    ICM42605_Verify_Once();
}
void Board_init()
{
    printf("NOTHING\r\n");

    I2C_GPIO_Init();
    Delay_Ms(50);
    BMP388_Measure();

    /* 1) IST8310 磁力计：7位地址 0x0E（规格书记录），有效寄存器 0x00~0x3F */
    printf ("=== IST8310 @0x0E Reg Table ===\r\n");
    DumpRegTable (0x0E, 0x3F);

    /* 2) BMP388 气压计：7位地址 0x76，寄存器 0x00~0x7F */
    printf ("\r\n=== BMP388 @0x76 Reg Table ===\r\n");
    DumpRegTable (0x76, 0x7F);

    SPI_Init1();
    SPI_WriteReg(0x4E, 0x3F);
    Delay_Ms(50);
    printf("WHO_AM_I = 0x%02X\r\n", SPI_ReadReg(0x00));
    DumpAllRegs();

    Sensor_Verify_Once();
    //MAG_Loop();
    return;
}

// void exam(void)
// {
    // I2C_WriteReg(0x0E, 0x0B, 0x01);   // CNTL2: SRST=1 软复位
    // Delay_Ms(10);

    // I2C_WriteReg(0x0E, 0x42, 0xC0);   // PDCNTL: set/reset 脉冲 = Normal
    // I2C_WriteReg(0x0E, 0x41, 0x24);   // AVGCNTL: Y×4 + X/Z×16 平均
    // I2C_WriteReg(0x0E, 0x0B, 0x0C);   // CNTL2: DREN=1 + DRP=1（DRDY 高有效）

    // I2C_WriteReg(0x0E, 0x0A, 0x01);

    // Delay_Ms(10);

    // // ===== 读取并打印 IST8310（地址 0x0E）的全部寄存器 =====
    // printf("\r\n=== IST8310 @0x0E Reg Table ===\r\n");
    // DumpRegTable(0x0E, 0x3F);   // 寄存器范围 0x00~0x3F
// }
