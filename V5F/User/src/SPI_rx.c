#include "SPI_rx.h"
#include "mipc_shm.h"      /* g_shm / shm_ts_read（共享区，V3F 小核定义） */
#include "debug.h"
#include "hardware.h"
#include "spi_hw.h"
#include "mipc_v5.h"
#include "sys_clk.h"
#include "ch32h417_usbhs_device.h"
#include "usbd_compatibility_hid.h"
#include <inttypes.h>

/* ==================== 内部私有变量（仅本文件，DMA 缓冲） ==================== */
static volatile uint8_t  rxfifo[512];
static volatile uint32_t cnt = 0;

/* DMA1_Channel2 中断单次执行耗时的最劣（最大）值，us；供主循环上报监视 ISR 负载。 */
volatile uint32_t v5f_dma1_irq_max_us = 0;

/* ==================== 数据保持器实例（原 observer.c 的 g_v5f_obs） ====================
 * q0=1 单位四元数初值；calib_done=1（无校准流程，直接可用）。 */
volatile v5f_hold_t g_v5f_hold = { .q0 = 1.0f, .calib_done = 1 };

/* ==================== 内部轮询状态（cnt 变化检测） ==================== */
static uint32_t s_last_log_cnt = 0, s_last_gyro_cnt = 0, s_last_ist_cnt = 0, s_last_bmp_cnt = 0;
static uint32_t s_last_gps_rmc_cnt = 0, s_last_gps_gga_cnt = 0, s_last_gps_gsa_cnt = 0;
static uint64_t s_last_gyro_ts = 0;

/* ==================== 各通道轮询 + 数据拷贝 + 延迟统计 ==================== */
static void hold_poll(uint64_t now_us)
{
    uint32_t c;

    /* 低频文本 */
    c = g_shm->log.cnt;
    if (c != s_last_log_cnt) { s_last_log_cnt = c; g_v5f_hold.log_pending = 1; }

    /* 陀螺：DRDY 时间戳重复检测 + cnt 轮询 → pending/stat/lat
     * （原 observer 在此顺带算 DRDY 差分 dt_us 供 attitude 积分；解算已弃用，故不再计算） */
    {
        uint64_t gyro_ts = shm_ts_read(&g_shm->gyro.ts_drdy_us);
        if (gyro_ts != 0) {
            if (gyro_ts == s_last_gyro_ts) g_v5f_hold.gyro_err++;
            else s_last_gyro_ts = gyro_ts;
        }
        c = g_shm->gyro.cnt;
        if (c != s_last_gyro_cnt) {
            s_last_gyro_cnt = c;
            g_v5f_hold.gyro_pending = 1;
            g_v5f_hold.stat_gyro++;
            if (gyro_ts != 0) g_v5f_hold.gyro_lat_us = (uint32_t)(now_us - (gyro_ts / 100)); /* gyro ts 为 10Ns，/100 对齐其中 now μs 语义 */
        /* ts 与 now 均为 10Ns 计数；/100 得 ?s（原 μs 语义保留） */
        }
    }

    /* IST8310（磁力计） */
    c = g_shm->ist.cnt;
    if (c != s_last_ist_cnt) {
        s_last_ist_cnt = c;
        g_v5f_hold.ist_pending = 1;
        g_v5f_hold.stat_ist++;
        uint64_t ist_ts = shm_ts_read(&g_shm->ist.ts_drdy_us);
        if (ist_ts != 0) g_v5f_hold.ist_lat_us = (uint32_t)(now_us - ist_ts);
        g_v5f_hold.mag_x = g_shm->ist.mx;
        g_v5f_hold.mag_y = g_shm->ist.my;
        g_v5f_hold.mag_z = g_shm->ist.mz;
    }

    /* BMP388（气压计）：定点直通 */
    c = g_shm->bmp.cnt;
    if (c != s_last_bmp_cnt) {
        s_last_bmp_cnt = c;
        g_v5f_hold.bmp_pending = 1;
        g_v5f_hold.stat_bmp++;
        uint64_t bmp_ts = shm_ts_read(&g_shm->bmp.ts_drdy_us);
        if (bmp_ts != 0) g_v5f_hold.bmp_lat_us = (uint32_t)(now_us - bmp_ts);
        g_v5f_hold.bmp_temp_x1000  = g_shm->bmp.temp_x1000;
        g_v5f_hold.bmp_press_x1000 = g_shm->bmp.press_x1000;
    }

    /* GPS RMC */
    c = g_shm->gps_rmc.cnt;
    if (c != s_last_gps_rmc_cnt) {
        s_last_gps_rmc_cnt = c;
        g_v5f_hold.gps_rmc_pending = 1;
        g_v5f_hold.stat_gps_rmc++;
        uint64_t gps_ts = shm_ts_read(&g_shm->gps_rmc.ts_drdy_us);
        if (gps_ts != 0) g_v5f_hold.gps_rmc_lat_us = (uint32_t)(now_us - gps_ts);
        g_v5f_hold.lat_e7     = g_shm->gps_rmc.lat_e7;
        g_v5f_hold.lon_e7     = g_shm->gps_rmc.lon_e7;
        g_v5f_hold.speed_cmps = g_shm->gps_rmc.speed_cmps;
    }

    /* GPS GGA */
    c = g_shm->gps_gga.cnt;
    if (c != s_last_gps_gga_cnt) {
        s_last_gps_gga_cnt = c;
        g_v5f_hold.gps_gga_pending = 1;
        g_v5f_hold.stat_gps_gga++;
        uint64_t gps_ts = shm_ts_read(&g_shm->gps_gga.ts_drdy_us);
        if (gps_ts != 0) g_v5f_hold.gps_gga_lat_us = (uint32_t)(now_us - gps_ts);
        g_v5f_hold.alt_cm     = g_shm->gps_gga.alt_cm;
        g_v5f_hold.fix_quality = g_shm->gps_gga.quality;
        g_v5f_hold.sat_num     = g_shm->gps_gga.sv;
        g_v5f_hold.hdop_x100   = g_shm->gps_gga.hdop_x100;
    }

    /* GPS GSA */
    c = g_shm->gps_gsa.cnt;
    if (c != s_last_gps_gsa_cnt) {
        s_last_gps_gsa_cnt = c;
        g_v5f_hold.gps_gsa_pending = 1;
        g_v5f_hold.stat_gps_gsa++;
        uint64_t gps_ts = shm_ts_read(&g_shm->gps_gsa.ts_drdy_us);
        if (gps_ts != 0) g_v5f_hold.gps_gsa_lat_us = (uint32_t)(now_us - gps_ts);
    }
}

/* 陀螺温度（SPI 0x1D/0x1E 原始值）→ ℃ */
static void hold_update_gyro_temp(int16_t temp_raw)
{
    g_v5f_hold.gyro_temp_celsius = temp_raw / 132.48f + 25.0f;
}

/* 加速度计（SPI 0x1F~0x24 原始值，大端）→ 保持器原始 LSB */
static void hold_update_accel(int16_t ax, int16_t ay, int16_t az)
{
    g_v5f_hold.accel_x = ax;
    g_v5f_hold.accel_y = ay;
    g_v5f_hold.accel_z = az;
}

/* ==================== JustFloat 帧（DMA ISR 尾组装，注入 USBHS-HID） ====================
 * 帧(10 x f32 = 40B 小端)：ch0-3 四元数 q0..q3 / ch4-6 bias_gx,gy,gz / ch7 s_still_cnt
 *                          / ch8 DMA1_Channel2 IRQ 最劣耗时 us / ch9 尾部 0x7F800000(+Inf)
 * 注意：attitude.c / ins.c 已弃用，ch0-3 恒为单位四元数、ch4-6 与 ch7 暂以 0 占位；
 *       帧格式暂时保持不变（通道语义待定），改动前先确认上位机解析。 */
static void hud_frame_push(void)
{
    union { float f; uint32_t u; } out;
    float   q[10];
    uint8_t b[40];
    int     i;

    out.u = 0x7F800000u;                     /* 帧尾 +Inf */
    q[0]=g_v5f_hold.q0; q[1]=g_v5f_hold.q1; q[2]=g_v5f_hold.q2; q[3]=g_v5f_hold.q3;
    q[4]=0.0f;          q[5]=0.0f;          q[6]=0.0f;   /* 零偏通道：解算模块已弃用 */
    q[7]=0.0f;                                          /* 静止计数：解算模块已弃用 */
    q[8]=(float)v5f_dma1_irq_max_us;
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
            cnt++;

            /* 保持器维护：各共享通道 cnt 轮询 + 数据拷贝 + 延迟统计 */
            hold_poll(GetTime64_Us());

            /* SPI 解析（温度 + 六轴）→ 保持器 */
            int16_t temp_raw = (int16_t)((rxfifo[1] << 8) | rxfifo[2]);
            hold_update_gyro_temp(temp_raw);
            g_v5f_hold.gyro_x = (int16_t)((rxfifo[9]  << 8) | rxfifo[10]);
            g_v5f_hold.gyro_y = (int16_t)((rxfifo[11] << 8) | rxfifo[12]);
            g_v5f_hold.gyro_z = (int16_t)((rxfifo[13] << 8) | rxfifo[14]);
            int16_t ax = (int16_t)((rxfifo[3] << 8) | rxfifo[4]);
            int16_t ay = (int16_t)((rxfifo[5] << 8) | rxfifo[6]);
            int16_t az = (int16_t)((rxfifo[7] << 8) | rxfifo[8]);
            hold_update_accel(ax, ay, az);

            /* 姿态解算 (attitude_update) 与惯性预处理 (ins_update) 已随模块弃用删除 */

            /* -------- 重新触发 DMA 接收 -------- */
            GPIO_SetBits(GPIOF, GPIO_Pin_4);
            DMA_Cmd(DMA1_Channel2, DISABLE);
            DMA1_Channel2->CNTR = 15;
            DMA_Cmd(DMA1_Channel2, ENABLE);

            /* 记录本次 ISR 用时的历史最劣(最大)值，us */
            uint32_t d = (uint32_t)(GetTime64_Us() - isr_t0);
            if (d > v5f_dma1_irq_max_us) v5f_dma1_irq_max_us = d;

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
