#include "SPI_rx.h"
#include "observer.h"
#include "attitude.h"
#include "ins.h"
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

/* ==================== JustFloat 帧（DMA ISR 尾组装，注入 USBHS-HID） ====================
 * 帧(10 x f32 = 40B 小端)：ch0-3 四元数 q0..q3 / ch4-6 bias_gx,gy,gz / ch7 s_still_cnt
 *                          / ch8 DMA1_Channel2 IRQ 最劣耗时 us / ch9 尾部 0x7F800000(+Inf)
 * 数据取自已同 ISR 更新的 observer/attitude/ins；经 hid_up_enqueue 严格顺序进上行字节流。 */
static void hud_frame_push(void)
{
    union { float f; uint32_t u; } out;
    float   bx = bias_gx, by = bias_gy, bz = bias_gz;   /* 各取一次，避免撕裂 */
    float   q[10];
    uint8_t b[40];
    int     i;

    out.u = 0x7F800000u;                     /* 帧尾 +Inf */
    q[0]=g_v5f_obs.q0; q[1]=g_v5f_obs.q1; q[2]=g_v5f_obs.q2; q[3]=g_v5f_obs.q3;
    q[4]=bx;           q[5]=by;              q[6]=bz;
    q[7]=(float)s_still_cnt;
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

            /* 观察器维护：各共享通道 cnt 轮询 + 数据拷贝 + 延迟统计（返回 gyro dt_us） */
            uint32_t dt_us = observer_poll(GetTime64_Us());
            uint64_t gyro_ts = shm_ts_read(&g_shm->gyro.ts_drdy_us);   /* DRDY 时间戳 */

            /* SPI 解析（温度 + 六轴） */
            int16_t temp_raw = (int16_t)((rxfifo[1] << 8) | rxfifo[2]);
            observer_update_gyro_temp(temp_raw);
            int16_t gx = (int16_t)((rxfifo[9]  << 8) | rxfifo[10]);
            int16_t gy = (int16_t)((rxfifo[11] << 8) | rxfifo[12]);
            int16_t gz = (int16_t)((rxfifo[13] << 8) | rxfifo[14]);
            int16_t ax = (int16_t)((rxfifo[3] << 8) | rxfifo[4]);
            int16_t ay = (int16_t)((rxfifo[5] << 8) | rxfifo[6]);
            int16_t az = (int16_t)((rxfifo[7] << 8) | rxfifo[8]);
            observer_update_accel(ax, ay, az);

            /* 姿态解算（bias + 四元数积分 + 在线零偏跟踪）→ attitude 模块 */
            attitude_update(gx, gy, gz, dt_us, gyro_ts);

            /* 惯性预处理（线加速度 + 静止判定）→ ins 模块（EKF 前置） */
            ins_update(ax, ay, az, g_v5f_obs.q0, g_v5f_obs.q1, g_v5f_obs.q2, g_v5f_obs.q3, dt_us, gyro_ts);

            /* -------- 重新触发 DMA 接收 -------- */
            GPIO_SetBits(GPIOF, GPIO_Pin_4);
            DMA_Cmd(DMA1_Channel2, DISABLE);
            DMA1_Channel2->CNTR = 15;
            DMA_Cmd(DMA1_Channel2, ENABLE);

            /* 记录本次 ISR 用时的历史最劣(最大)值，us */
            uint32_t d = (uint32_t)(GetTime64_Us() - isr_t0);
            if (d > v5f_dma1_irq_max_us) v5f_dma1_irq_max_us = d;

            /* 组一帧 JustFloat 注入 USBHS-HID(顺序 = 该次解算) */
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
