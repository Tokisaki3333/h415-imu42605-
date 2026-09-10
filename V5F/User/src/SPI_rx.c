#include "SPI_rx.h"
#include "mipc_shm.h"               /* g_shm：双核共享区（V3F 小核定义） */
#include "hardware.h"               /* ch32h417.h + debug.h：外设寄存器 / 中断 / Core_ID_V5F */
#include "spi_hw.h"                 /* SPI1_DMA_Rx_Setup */
#include "sys_clk.h"                /* GetTime64_Us */

/* ==================== 内部私有变量 ==================== */
static volatile uint8_t rxfifo[512];

/* DMA1_Channel2 中断单次执行耗时历史最劣值 us（备用，暂无消费者） */
static volatile uint32_t s_dma1_irq_max_us = 0;

/* ==================== 数据保持器实例 ==================== */
volatile v5f_hold_t g_v5f_hold = { 0 };

/* ==================== 各共享通道 cnt 快照（变化检测用） ==================== */
static uint32_t s_last_gyro_cnt = 0, s_last_ist_cnt = 0, s_last_bmp_cnt = 0;
static uint32_t s_last_gps_rmc_cnt = 0, s_last_gps_gga_cnt = 0, s_last_gps_gsa_cnt = 0;

/* ==================== 保持器维护 ====================
 * 每通道都是"检测到 cnt 变化 → __sync_synchronize() → 读 ts/数据/flags"（与写端 release 配对；
 * RISC-V 弱内存序下 load 可重排，缺此屏障会读到上一帧）。低频文本由主循环直接读 g_shm->log。 */
static void hold_poll(void)
{
    uint32_t c;

    /* 陀螺通道：只有 DRDY 时间戳（六轴原始值走 SPI DMA 帧） */
    c = g_shm->gyro.hdr.cnt;
    if (c != s_last_gyro_cnt) {
        s_last_gyro_cnt = c;
        __sync_synchronize();
        g_v5f_hold.imu.fresh.drdy_tick = shm_chan_ts_read(&g_shm->gyro.hdr);
    }

    /* IST8310（磁力计）：原始 LSB 直通 */
    c = g_shm->ist.hdr.cnt;
    if (c != s_last_ist_cnt) {
        s_last_ist_cnt = c;
        __sync_synchronize();
        g_v5f_hold.mag.fresh.drdy_tick = shm_chan_ts_read(&g_shm->ist.hdr);
        g_v5f_hold.mag.lsb[0] = g_shm->ist.mx;
        g_v5f_hold.mag.lsb[1] = g_shm->ist.my;
        g_v5f_hold.mag.lsb[2] = g_shm->ist.mz;
        g_v5f_hold.mag.fresh.new_data = 1;      /* 数据写完后才置新数据信号 */
    }

    /* BMP388（气压计）：float 直通 */
    c = g_shm->bmp.hdr.cnt;
    if (c != s_last_bmp_cnt) {
        s_last_bmp_cnt = c;
        __sync_synchronize();
        g_v5f_hold.baro.fresh.drdy_tick = shm_chan_ts_read(&g_shm->bmp.hdr);
        g_v5f_hold.baro.temp_celsius  = g_shm->bmp.temp_celsius;   /* 共享区已是 float，无需换算 */
        g_v5f_hold.baro.press_pascal  = g_shm->bmp.press_pascal;
        g_v5f_hold.baro.fresh.new_data  = 1;
    }

    /* GPS RMC：经纬度保持共享区的 10^-7 ° 定标整数（float 精度不够），速度换算成 m/s */
    c = g_shm->gps_rmc.hdr.cnt;
    if (c != s_last_gps_rmc_cnt) {
        s_last_gps_rmc_cnt = c;
        __sync_synchronize();  
        g_v5f_hold.gps_rmc.fresh.drdy_tick = shm_chan_ts_read(&g_shm->gps_rmc.hdr);
        g_v5f_hold.gps_rmc.fresh.flags     = g_shm->gps_rmc.flags;
        g_v5f_hold.gps_rmc.lat_e7        = g_shm->gps_rmc.lat_e7;
        g_v5f_hold.gps_rmc.lon_e7        = g_shm->gps_rmc.lon_e7;
        g_v5f_hold.gps_rmc.speed_mps     = g_shm->gps_rmc.speed_mps;
        g_v5f_hold.gps_rmc.status         = g_shm->gps_rmc.status;
        g_v5f_hold.gps_rmc.date_ddmmyy    = g_shm->gps_rmc.date_ddmmyy;
        g_v5f_hold.gps_rmc.fresh.new_data = 1;
    }

    /* GPS GGA：高度 cm→m、HDOP ×100→无量纲 float，定位质量/星数本就是小整数 */
    c = g_shm->gps_gga.hdr.cnt;
    if (c != s_last_gps_gga_cnt) {
        s_last_gps_gga_cnt = c;
        __sync_synchronize();  
        g_v5f_hold.gps_gga.fresh.drdy_tick = shm_chan_ts_read(&g_shm->gps_gga.hdr);
        g_v5f_hold.gps_gga.fresh.flags     = g_shm->gps_gga.flags;
        g_v5f_hold.gps_gga.alt_m        = (float)g_shm->gps_gga.alt_cm * 1e-2f;
        g_v5f_hold.gps_gga.fix_quality  = g_shm->gps_gga.quality;
        g_v5f_hold.gps_gga.sat_num      = g_shm->gps_gga.sv;
        g_v5f_hold.gps_gga.hdop         = (float)g_shm->gps_gga.hdop_x100 * 1e-2f;
        g_v5f_hold.gps_gga.fresh.new_data = 1;
    }

    /* GPS GSA：PDOP/VDOP（共享区 ×100 定标整数 → 无量纲 float） */
    c = g_shm->gps_gsa.hdr.cnt;
    if (c != s_last_gps_gsa_cnt) {
        s_last_gps_gsa_cnt = c;
        __sync_synchronize();               
        g_v5f_hold.gps_gsa.fresh.drdy_tick = shm_chan_ts_read(&g_shm->gps_gsa.hdr);
        g_v5f_hold.gps_gsa.fresh.flags     = g_shm->gps_gsa.flags;
        g_v5f_hold.gps_gsa.pdop = (float)g_shm->gps_gsa.pdop_x100 * 1e-2f;
        g_v5f_hold.gps_gsa.vdop = (float)g_shm->gps_gsa.vdop_x100 * 1e-2f;
        g_v5f_hold.gps_gsa.fresh.new_data = 1;
    }
}

/* ==================== DMA 中断服务函数：只负责调度分配 ==================== */
void DMA1_Channel2_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void DMA1_Channel2_IRQHandler(void)
{
    if (DMA_GetITStatus(DMA1, DMA1_IT_TC2)) {
        DMA_ClearITPendingBit(DMA1, DMA1_IT_TC2);
        if (g_shm) {
            uint64_t isr_t0 = GetTime64_Us();   /* ISR 计时起点 */

            /* SPI 帧解析（温度 + 六轴）→ 保持器（DRDY 时刻由 hold_poll 搬入） */
            g_v5f_hold.imu.temp_celsius = (int16_t)((rxfifo[1] << 8) | rxfifo[2]) / 132.48f + 25.0f;
            g_v5f_hold.imu.gyro_lsb[0]  = (int16_t)((rxfifo[9]  << 8) | rxfifo[10]);
            g_v5f_hold.imu.gyro_lsb[1]  = (int16_t)((rxfifo[11] << 8) | rxfifo[12]);
            g_v5f_hold.imu.gyro_lsb[2]  = (int16_t)((rxfifo[13] << 8) | rxfifo[14]);
            g_v5f_hold.imu.accel_lsb[0] = (int16_t)((rxfifo[3]  << 8) | rxfifo[4]);
            g_v5f_hold.imu.accel_lsb[1] = (int16_t)((rxfifo[5]  << 8) | rxfifo[6]);
            g_v5f_hold.imu.accel_lsb[2] = (int16_t)((rxfifo[7]  << 8) | rxfifo[8]);
            hold_poll();
            g_v5f_hold.imu.fresh.new_data = 1;  /* 所有字段写完后才置新数据信号 */

            /* 主任务区开始 */

            ;

            /* 主任务区结束 */

            /* -------- 重新触发 DMA 接收 -------- */
            GPIO_SetBits(GPIOF, GPIO_Pin_4);
            DMA_Cmd(DMA1_Channel2, DISABLE);
            DMA1_Channel2->CNTR = 15;
            DMA_Cmd(DMA1_Channel2, ENABLE);

            /* 记录本次 ISR 用时的历史最劣(最大)值，us */
            uint32_t d = (uint32_t)(GetTime64_Us() - isr_t0);
            if (d > s_dma1_irq_max_us) s_dma1_irq_max_us = d;
        }
    }
}

/* ==================== DMA 初始化函数 ==================== */
void SPI_DMA_Init(void)
{
    RCC_HBPeriphClockCmd(RCC_HBPeriph_DMA1, ENABLE);
    SPI1_DMA_Rx_Setup((uint8_t*)rxfifo, 15);

    NVIC_SetAllocateIRQ(DMA1_Channel2_IRQn, Core_ID_V5F);
    NVIC_SetPriority(DMA1_Channel2_IRQn, 0<<5);
    NVIC_EnableIRQ(DMA1_Channel2_IRQn);
}
