#include "USBCDC.h"
#include "ch32h417_usbhs_device.h"
#include "usbd_compatibility_hid.h"

/* USBHS-HID 使能：先禁用 SWJ，再配置中断归属并初始化设备 */
void usbhs_hid_enable(void)
{
    GPIO_PinRemapConfig(GPIO_Remap_SWJ_Disable, ENABLE);   /* 关闭调试口，USBHS 独占 */

    RCC_HB2PeriphClockCmd(RCC_HB2Periph_GPIOC | RCC_HB2Periph_GPIOF, ENABLE);
    GPIO_InitTypeDef gpio = {0};
    gpio.GPIO_Pin  = GPIO_Pin_9, gpio.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOC, &gpio);
    gpio.GPIO_Pin  = GPIO_Pin_0, gpio.GPIO_Mode = GPIO_Mode_Out_PP;
    gpio.GPIO_Speed = GPIO_Speed_High;
    GPIO_Init(GPIOF, &gpio);
    GPIO_SetBits(GPIOF, GPIO_Pin_0);    /* 初始:灭 = PF0 高 */

    NVIC_SetAllocateIRQ(USBHS_IRQn, Core_ID_V5F);
    USBHS_Device_Init(ENABLE);
}

/* 主循环轮询：HID 上行灌数 + PC9 松开沿单次触发切回 SWJ */
void usbhs_hid_poll(void)
{
    static uint8_t trig_armed = 1;      /* 每次上电可切一次(单向不再回) */
    static uint8_t key_prev    = 1;

    if (hid_up_avail())
        hid_up_flush();

    if (trig_armed)
    {
        uint8_t key_now = (GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_9) == RESET) ? 0 : 1;
        if (key_prev == 0 && key_now == 1)
        {
            trig_armed = 0;
            GPIO_WriteBit(GPIOF, GPIO_Pin_0, Bit_RESET);  /* PF0 低(亮) */

            /* 固定序列：先失能 USBHS，再使能 SWJ(可连调试器) */
            USBHS_Device_Init(DISABLE);
            GPIO_PinRemapConfig(GPIO_Remap_SWJ_Disable, DISABLE);
        }
        key_prev = key_now;
    }
}
