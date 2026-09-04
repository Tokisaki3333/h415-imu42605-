#include "sys_clk.h"

//使能TIM9、10并级联
void SYS_CLK_Init(void)
{
    RCC_HB2PeriphClockCmd(RCC_HB2Periph_TIM9 | RCC_HB2Periph_TIM10, ENABLE);

    TIM9_12_TimeBaseInitTypeDef TimInit;

    TimInit.TIM_Prescaler       = 0;       //分频值
    TimInit.TIM_CounterMode     = TIM_CounterMode_Up;
    TimInit.TIM_Period          = 99999999;   //重装值
    TimInit.TIM_ClockDivision   = TIM_CKD_DIV1;

    TIM9_12_TimeBaseInit(TIM9, &TimInit);
    TIM_ARRPreloadConfig(TIM9, ENABLE);

    TimInit.TIM_Prescaler       = 0;       //分频值
    TimInit.TIM_CounterMode     = TIM_CounterMode_Up;
    TimInit.TIM_Period          = 0xFFFFFFFF;   //重装值
    TimInit.TIM_ClockDivision   = TIM_CKD_DIV1;

    TIM9_12_TimeBaseInit(TIM10, &TimInit);

    TIM_SelectOutputTrigger(TIM9, TIM_TRGOSource_Update);
    TIM_SelectInputTrigger(TIM10, TIM_TS_ITR3);
    TIM_SelectSlaveMode(TIM10, TIM_SlaveMode_External1);

    TIM_Cmd(TIM10, ENABLE);
    TIM_Cmd(TIM9, ENABLE);
    return;
}

uint64_t GetTime64_Us(void)
{
    uint32_t high1, high2, low;
    do {
        high1 = TIM10->CNT_32;
        low   = TIM9->CNT_32;    /* low: 10ns tick, 0..99999999, "+"1s=1e8 tick */
        high2 = TIM10->CNT_32;
    } while (high1 != high2);
    return (uint64_t)high1 * 1000000ULL + (uint32_t)(low / 100);
}

void HWT_DelayUs(uint64_t us)
{
    uint64_t start = GetTime64_Us();
    while ((GetTime64_Us() - start) < us);
}

void HWT_DelayMs(uint64_t ms)
{
    HWT_DelayUs(ms * 1000ULL);   // 无溢出风险
}

uint64_t GetTime64_10Ns(void)
{
    uint32_t high1, high2, low;
    do {
        high1 = TIM10->CNT_32;   /* high: 鏁寸锛圱IM10, 鐢辨瘡绉掍竴娆? update 椹卞姩锛? */
        low   = TIM9->CNT_32;    /* low: 10ns tick, 0..99999999, "+"1s=1e8 tick */
        high2 = TIM10->CNT_32;
    } while (high1 != high2);
    return (uint64_t)high1 * 100000000ULL + low;
}
