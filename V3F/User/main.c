#include "debug.h"
#include "hardware.h"
#include "mipc_v3.h"
#include "sys_clk.h"

#include "spi_hw.h"
#include "i2c_soft.h"
#include "bmp388.h"
#include "ist8310.h"
#include "GPS.h"
#include "oled_ssd1306.h"

/* 空窗期可用上限（us）：从触发采集到 IST8310 就绪，扣除 GPS 最差周期后剩 4940us */
#define OLED_WINDOW_US   4940u
/* 每页(128B) I2C 传输耗时预算（us）：实测 ~855us，取 900us 留余量 */
#define OLED_PAGE_US     900u

/* PE0 = IMU INT1(DRDY) 外部中断：记录 DRDY 时间戳并共享，SPI 读温度+六轴（走 SPI_rx DMA）*/
void EXTI7_0_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void EXTI7_0_IRQHandler(void)
{
    if (EXTI_GetITStatus(EXTI_Line0) != RESET) {
        EXTI_ClearITPendingBit(EXTI_Line0);
        /* 严谨：实际读 DRDY 引脚确认，防虚假/毛刺中断 */
        if (GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_0) != 0) {
            uint64_t ts_drdy = GetTime64_10Ns();   /* DRDY 时间戳：全传感器统一 10 ns 计数 */
            SPI_ReadMulti(0x1D, 14);             /* 读温度+六轴（不进共享区）*/
            shm_publish_gyro(ts_drdy);           /* 共享：ts(高→低)→屏障→cnt++ */
        }
    }
}

int main(void)
{
    SystemInit();
    SystemAndCoreClockUpdate();
    Delay_Init();
    USART_Printf_Init(921600);
    SYS_CLK_Init();
    soft_i2c_silm_Init();

    ipc_comm_init();
    while (g_shm->v5f_progress < 2) Delay_Us(1);   /* wait V5F step2 (SPI1_Init done) */
    icm52605_Init_A();

    oled_printf(0, 0, "启动中");

    Delay_Ms(20);
    
    oled_ssd1306_init();        /* 自有库：SCL=PA14 SDA=PA13，共享显存 oled_fb */
    GPS_USART_Init();

    printf("[V3F] SystemClk:%d\r\n", SystemClock);
    printf("[V3F] V3F SystemCoreClk:%d\r\n", SystemCoreClock);

    icm52605_Init_B();
    g_shm->v3f_cfg_done = 1;   /* notify V5F: ICM42605 config done (V3F->V5F) */
    while (g_shm->v5f_progress < 5) Delay_Us(1);
    g_shm->v3f_cfg_done = 0;
    // while (g_shm->v3f_cfg_done != 0) Delay_Us(1);

    NVIC_SetPriority(EXTI7_0_IRQn, 0<<7);
    NVIC_EnableIRQ(EXTI7_0_IRQn);
    mipc_v3_printf("v3f weak up exti enabled\r\n");

    printf("V3 UP\r\n");

    USART_DMACmd(USART3, USART_DMAReq_Rx, ENABLE);

    float temp_c = 0.0f, press_pa = 0.0f;
    int16_t mx, my, mz;
    uint64_t ts_ist, ts_bmp;
    while (1)
    {
        BMP388_TriggerMeasurement();

        uint64_t t_win0 = GetTime64_Us();   /* 空窗期起点：触发采集时刻 */

//空窗期开始

        GPS_Check();

        /* 自适应刷屏：按空窗期剩余时间（4940us 上限的 90% 裕量）决定本次刷几页 */
        uint64_t t_now = GetTime64_Us();
        uint32_t remain_us = (t_now >= t_win0 + OLED_WINDOW_US) ? 0u : (uint32_t)(t_win0 + OLED_WINDOW_US - t_now);
        uint32_t budget_us = remain_us * 10 / 10;             /* 取 100% 作裕量 */
        uint8_t pages = (uint8_t)(budget_us / OLED_PAGE_US); /* 本次最多刷的页数 */
        if (pages > 8) pages = 8;

        oled_refresh(pages);
        
//空窗期结束,从触发采集到IST8310数据就绪之间，扣除GPS事务最差周期耗时后，有4940us可用
        ts_ist = 0;
        ts_bmp = 0;
        IST8310_WaitDRDY(10000, &ts_ist);
        
        if (IST8310_Read(&mx, &my, &mz) == 0 && ts_ist != 0)
            shm_publish_ist(ts_ist, mx, my, mz);

//IST8310就绪后，到等待BMP388就绪前有70us可用

        BMP388_WaitDRDY(10000, &ts_bmp);

        IST8310_TriggerMeasurement();

//BMP388就绪后无空闲时间

        if (BMP388_Read(&temp_c, &press_pa) == 0 && ts_bmp != 0)
            shm_publish_bmp(ts_bmp, (int32_t)(temp_c * 1000.0f), (int32_t)(press_pa * 1000.0f));
    }
}
