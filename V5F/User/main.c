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

    // usbhs_hid_enable();                 /* 内部先禁用 SWJ，再使能 USBHS */

    uint64_t last_print_tim = GetTime64_Us(), tim = 0;
    while (1)
    {
        // usbhs_hid_poll();

        tim = GetTime64_Us();
        if(tim - last_print_tim > 50000)
        {
            last_print_tim = tim;

            /* 经纬度是 10^-7 ° 定标整数，按整数打印；HDOP 是 float，拆成整数与小数两位显示 */
            oled_printf(0, 0, "a%12d", (int)(g_v5f_hold.gps_rmc.lat_e7));
            oled_printf(0, 16, "o%12d", (int)(g_v5f_hold.gps_rmc.lon_e7));
            oled_printf(0, 32, "P%02d S%02d H%02d.%02d",
                        (int)g_v5f_hold.gps_gga.fix_quality, (int)g_v5f_hold.gps_gga.sat_num,
                        (int)g_v5f_hold.gps_gga.hdop,
                        (int)(g_v5f_hold.gps_gga.hdop * 100.0f) % 100);
        }
    }
}
