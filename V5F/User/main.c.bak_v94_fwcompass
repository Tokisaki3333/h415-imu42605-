#include "debug.h"
#include "hardware.h"
#include "spi_hw.h"
#include "mipc_v5.h"
#include "SPI_rx.h"        /* SPI_DMA_Init / g_v5f_hold（数据保持器） */
#include "sys_clk.h"
#include "spi_flash_w25n.h"
#include <inttypes.h>
#include <string.h>

#include "oled_ssd1306.h"
#include "USBCDC.h"

int main(void)
{
    SystemAndCoreClockUpdate();
    Delay_Init();
    mipc_v5_get_shared();

    RCC_HB2PeriphClockCmd(RCC_HB2Periph_SPI1, ENABLE);

    SPI1_Init();
    mipc_v5_sync(2);
    HSEM_ReleaseOneSem(HSEM_ID0, 0);

    while (g_shm->v3f_cfg_done == 0) Delay_Ms(1);
    mipc_v5_sync(3);

    SPI_DMA_Init();                     /* DMA 中断里维护保持器 + 组 HID 帧 */

    mipc_v5_sync(4);
    mipc_v5_sync(5);

    while (g_shm->log.cnt == 0) Delay_Ms(1);
    printf("%s", (const char *)g_shm->log.data_buf);

    mipc_v5_sync(6);

    printf("[V5F] Ready.\r\n");

    usbhs_hid_enable();                 /* 内部先禁用 SWJ，再使能 USBHS */

    /* 屏显任务停用（与上报无关）：恢复时把这行与下面整块一起去掉注释 */
    // uint64_t last_print_tim = GetTime64_Us(), tim = 0;
    while (1)
    {
        usbhs_hid_poll();               /* 上行环有字节就灌 1 个 EP2 包 */

        /* ---- FireWater（VOFA+ 文本协议）：20 Hz 输出四元数供检查 ----
         * 放在主循环、**不放中断**：printf 是阻塞式的，115200 baud 下一行约 6 ms，
         * 丢进 8 kHz 的 DMA 中断会直接毁掉时间轴。
         * 8 路：ekf_qw,qx,qy,qz, att_qw,qx,qy,qz（逗号分隔，\n 结束）
         * ★ ekf_q 的导航系是真 ENU（磁力计一次性对齐），att_q 是上电时 x 轴的水平
         *   投影，两者只差一个绕竖直轴的**固定旋转**（实测 = -D = +7.53 度）。
         *   直接比分量会看到固定偏航差，那不是误差；比精度请比转角的变化量。
         * ★ 阻塞打印期间主循环无法灌 EP2，上行环只有约 2 ms 余量 —— 若发现 HID
         *   采集掉帧，先把这里的输出降到 10 Hz 或把调试串口波特率提高。 */
        {
            static uint64_t s_fw_last;
            uint64_t fw_tv = GetTime64_Us();
            if (fw_tv - s_fw_last >= 50000ULL)      /* 20 Hz */
            {
                s_fw_last = fw_tv;
                printf("%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\n",
                       (double)g_v5f_hold.ekf.q[0], (double)g_v5f_hold.ekf.q[1],
                       (double)g_v5f_hold.ekf.q[2], (double)g_v5f_hold.ekf.q[3],
                       (double)g_v5f_hold.att.q[0], (double)g_v5f_hold.att.q[1],
                       (double)g_v5f_hold.att.q[2], (double)g_v5f_hold.att.q[3]);
                usbhs_hid_poll();                   /* 打印前后各灌一次，尽量少压上行环 */
            }
        }

        // tim = GetTime64_Us();
        // if(tim - last_print_tim > 50000)
        // {
        //     last_print_tim = tim;
        //
        //     /* 经纬度是 10^-7 ° 定标整数，按整数打印；HDOP 是 float，拆成整数与小数两位显示 */
        //     oled_printf(0, 0, "a%12d", (int)(g_v5f_hold.gps_rmc.lat_e7));
        //     oled_printf(0, 16, "o%12d", (int)(g_v5f_hold.gps_rmc.lon_e7));
        //     oled_printf(0, 32, "P%02d S%02d H%02d.%02d",
        //                 (int)g_v5f_hold.gps_gga.fix_quality, (int)g_v5f_hold.gps_gga.sat_num,
        //                 (int)g_v5f_hold.gps_gga.hdop,
        //                 (int)(g_v5f_hold.gps_gga.hdop * 100.0f) % 100);
        // }
    }
}
