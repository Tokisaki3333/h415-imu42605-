#include "SPI_rx.h"
#include "mipc_shm.h"               /* g_shm：双核共享区（V3F 小核定义） */
#include "hardware.h"               /* ch32h417.h + debug.h：外设寄存器 / 中断 / Core_ID_V5F */
#include "spi_hw.h"                 /* SPI1_DMA_Rx_Setup */
#include "sys_clk.h"                /* GetTime64_Us */
#include "usbd_compatibility_hid.h" /* hid_up_enqueue */

/* ==================== 内部私有变量（仅本文件，DMA 缓冲） ==================== */
static volatile uint8_t rxfifo[512];

/* DMA1_Channel2 中断单次执行耗时的最劣（最大）值，us；作为 HID 帧 ch8 上报 ISR 负载。 */
static volatile uint32_t s_dma1_irq_max_us = 0;

/* ==================== 数据保持器实例 ==================== */
volatile v5f_hold_t g_v5f_hold = { .imu = { .quat = { 1.0f, 0.0f, 0.0f, 0.0f } } };

/* ==================== 各共享通道 cnt 快照（变化检测用） ==================== */
static uint32_t s_last_ist_cnt = 0, s_last_bmp_cnt = 0;
static uint32_t s_last_gps_rmc_cnt = 0, s_last_gps_gga_cnt = 0;

/* 共享区所有通道的时间戳已统一为 10 ns 计数（V3F 侧一律用 GetTime64_10Ns() 取），
 * 这里直接搬运即可，不再需要任何单位换算。 */

/* ==================== 保持器维护：共享通道有新数据就搬进保持器 ====================
 * 发布端顺序为"写数据 → 写 ts → 屏障 → cnt++"（见 mipc_shm.h），
 * 故检测到 cnt 变化后直接读数据即可。
 * 只轮询带数据字段的通道：陀螺通道仅有 DRDY 时间戳（六轴数据走 SPI DMA 帧）、
 * GSA 只有 PDOP/VDOP（保持器不存）、低频文本由主循环直接读 g_shm->log，均无需在此搬运。 */
static void hold_poll(void)
{
    uint32_t c;

    /* IST8310（磁力计）：原始 LSB 直通 */
    c = g_shm->ist.cnt;
    if (c != s_last_ist_cnt) {
        s_last_ist_cnt = c;
        g_v5f_hold.mag.fresh.drdy_tick = shm_ts_read(&g_shm->ist.ts_drdy_tick);
        g_v5f_hold.mag.lsb[0] = g_shm->ist.mx;
        g_v5f_hold.mag.lsb[1] = g_shm->ist.my;
        g_v5f_hold.mag.lsb[2] = g_shm->ist.mz;
        g_v5f_hold.mag.fresh.new_data = 1;      /* 数据写完后才置新数据信号 */
    }

    /* BMP388（气压计）：共享区是 ℃×1000 / Pa×1000 十进制定标整数 → 换算成 float 物理量 */
    c = g_shm->bmp.cnt;
    if (c != s_last_bmp_cnt) {
        s_last_bmp_cnt = c;
        g_v5f_hold.baro.fresh.drdy_tick = shm_ts_read(&g_shm->bmp.ts_drdy_tick);
        g_v5f_hold.baro.temp_celsius    = (float)g_shm->bmp.temp_x1000  * 1e-3f;
        g_v5f_hold.baro.press_pascal    = (float)g_shm->bmp.press_x1000 * 1e-3f;
        g_v5f_hold.baro.fresh.new_data  = 1;
    }

    /* GPS RMC：经纬度保持共享区的 10^-7 ° 定标整数（float 精度不够），速度换算成 m/s */
    c = g_shm->gps_rmc.cnt;
    if (c != s_last_gps_rmc_cnt) {
        s_last_gps_rmc_cnt = c;
        g_v5f_hold.gps_rmc.fresh.drdy_tick = shm_ts_read(&g_shm->gps_rmc.ts_drdy_tick);
        g_v5f_hold.gps_rmc.lat_e7        = g_shm->gps_rmc.lat_e7;
        g_v5f_hold.gps_rmc.lon_e7        = g_shm->gps_rmc.lon_e7;
        g_v5f_hold.gps_rmc.speed_mps     = (float)g_shm->gps_rmc.speed_cmps * 1e-2f;
        g_v5f_hold.gps_rmc.fresh.new_data = 1;
    }

    /* GPS GGA：高度 cm→m、HDOP ×100→无量纲 float，定位质量/星数本就是小整数 */
    c = g_shm->gps_gga.cnt;
    if (c != s_last_gps_gga_cnt) {
        s_last_gps_gga_cnt = c;
        g_v5f_hold.gps_gga.fresh.drdy_tick = shm_ts_read(&g_shm->gps_gga.ts_drdy_tick);
        g_v5f_hold.gps_gga.alt_m        = (float)g_shm->gps_gga.alt_cm * 1e-2f;
        g_v5f_hold.gps_gga.fix_quality  = g_shm->gps_gga.quality;
        g_v5f_hold.gps_gga.sat_num      = g_shm->gps_gga.sv;
        g_v5f_hold.gps_gga.hdop         = (float)g_shm->gps_gga.hdop_x100 * 1e-2f;
        g_v5f_hold.gps_gga.fresh.new_data = 1;
    }
}

/* ==================== JustFloat 帧（DMA ISR 尾组装，注入 USBHS-HID） ====================
 * 帧(10 x f32 = 40B 小端)：ch0-3 四元数 q0..q3 / ch4-6 bias / ch7 静止计数
 *                          / ch8 本次 ISR 最劣耗时 us / ch9 尾部 0x7F800000(+Inf)
 * attitude.c / ins.c 已弃用：ch0-3 恒为单位四元数，ch4-6 与 ch7 暂以 0 占位（通道语义待定）。 */
static void hud_frame_push(void)
{
    union { float f; uint32_t u; } out;
    float   q[10];
    uint8_t b[40];
    int     i;

    out.u = 0x7F800000u;                     /* 帧尾 +Inf */
    q[0]=g_v5f_hold.imu.quat[0]; q[1]=g_v5f_hold.imu.quat[1];
    q[2]=g_v5f_hold.imu.quat[2]; q[3]=g_v5f_hold.imu.quat[3];
    q[4]=0.0f;          q[5]=0.0f;          q[6]=0.0f;   /* 零偏通道：解算模块已弃用 */
    q[7]=0.0f;                                          /* 静止计数：解算模块已弃用 */
    q[8]=(float)s_dma1_irq_max_us;
    q[9]=out.f;

    for(i=0;i<10;i++){ out.f=q[i]; b[i*4]  =(uint8_t)(out.u);
                       b[i*4+1]=(uint8_t)(out.u>>8 );
                       b[i*4+2]=(uint8_t)(out.u>>16);
                       b[i*4+3]=(uint8_t)(out.u>>24); }
    /* 空间不足(极端背压)整帧放弃，仍保持严格顺序不串帧 */
    hid_up_enqueue(b, 40);
}

/* ==================== DMA 中断服务函数：只负责调度分配 ==================== */
void DMA1_Channel2_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void DMA1_Channel2_IRQHandler(void)
{
    if (DMA_GetITStatus(DMA1, DMA1_IT_TC2)) {
        DMA_ClearITPendingBit(DMA1, DMA1_IT_TC2);
        if (g_shm) {
            uint64_t isr_t0 = GetTime64_Us();   /* ISR 计时起点 */

            /* 共享区低频通道：有新数据就搬进保持器 */
            hold_poll();

            /* SPI 帧解析（温度 + 六轴）→ 保持器；DRDY 时刻取共享区陀螺通道（统一 10 ns 计数） */
            g_v5f_hold.imu.fresh.drdy_tick = shm_ts_read(&g_shm->gyro.ts_drdy_tick);
            g_v5f_hold.imu.temp_celsius = (int16_t)((rxfifo[1] << 8) | rxfifo[2]) / 132.48f + 25.0f;
            g_v5f_hold.imu.gyro_lsb[0]  = (int16_t)((rxfifo[9]  << 8) | rxfifo[10]);
            g_v5f_hold.imu.gyro_lsb[1]  = (int16_t)((rxfifo[11] << 8) | rxfifo[12]);
            g_v5f_hold.imu.gyro_lsb[2]  = (int16_t)((rxfifo[13] << 8) | rxfifo[14]);
            g_v5f_hold.imu.accel_lsb[0] = (int16_t)((rxfifo[3]  << 8) | rxfifo[4]);
            g_v5f_hold.imu.accel_lsb[1] = (int16_t)((rxfifo[5]  << 8) | rxfifo[6]);
            g_v5f_hold.imu.accel_lsb[2] = (int16_t)((rxfifo[7]  << 8) | rxfifo[8]);
            g_v5f_hold.imu.fresh.new_data = 1;  /* 所有字段写完后才置新数据信号 */

            /* -------- 重新触发 DMA 接收 -------- */
            GPIO_SetBits(GPIOF, GPIO_Pin_4);
            DMA_Cmd(DMA1_Channel2, DISABLE);
            DMA1_Channel2->CNTR = 15;
            DMA_Cmd(DMA1_Channel2, ENABLE);

            /* 记录本次 ISR 用时的历史最劣(最大)值，us */
            uint32_t d = (uint32_t)(GetTime64_Us() - isr_t0);
            if (d > s_dma1_irq_max_us) s_dma1_irq_max_us = d;

            /* 组一帧 JustFloat 注入 USBHS-HID(顺序 = 该次采样) */
            hud_frame_push();
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
