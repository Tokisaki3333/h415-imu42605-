/********************************** (C) COPYRIGHT *******************************
 * File Name          : usbd_compatibility_hid.c
 * Author             : WCH
 * Version            : V1.0.0
 * Date               : 2024/07/31
 * Description        : USBHS-HID 字节流上传的最小实现。
 *                        不依赖 USART4 / DMA1_CH1·CH2 / TIM2 —— 高频中断只留 USBHS。
 *                        数据由任意上下文(含 IMU 的 DMA-RX 中断)经 hid_up_enqueue()
 *                        按严格顺序写入上行环形缓冲;USB 侧发送在主循环经
 *                        hid_up_flush() 主动拆包灌 EP2,USBHS_IRQHandler 的 EP2
 *                        T_DONE 中断续传下一包。
 *********************************************************************************
 * Copyright (c) 2025 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/

#include "ch32h417_usbhs_device.h"
#include "usbd_compatibility_hid.h"

/******************************************************************************/
/* ---- USBHS-HID 上行：严格顺序字节流双指针环形缓冲 ----
 *  单生产者(可被 DMA-RX 等任一中断调用)只写 tx_in,tx_remain 只增;
 *  单消费者(主循环发送 + USBHS 中断续传)只读 tx_out,读到即签核。
 *  生产/消费侧寄存器访问互不重叠,配合 ring 首尾留空一位即天然免锁双缓冲。
 */

__attribute__((aligned(4))) uint8_t  UART_RxBuffer[DEF_UART_BUF_SIZE];   /* 上行字节流环形存区 */

volatile uint16_t hid_RdPtr = 0;                                        /* 待消费者读取的游标(输出),尾指针 */
volatile uint16_t hid_WrPtr = 0;                                        /* 生产者已写入游标(输入),头指针 */
volatile uint16_t hid_BytesAvail = 0;                                   /* 待上传字节数(= 头-尾),消费者扣减 */

volatile uint16_t Data_Pack_Max_Len = 0;                                /* 单个 HID 报告可载净字节(按速度) */
volatile uint16_t Head_Pack_Len = 0;                                    /* HID 报告头部长度(存净长, HS=2/FS=1) */

#define HID_TXRING_FREE()  ((uint16_t)(DEF_UART_BUF_SIZE - 1 - hid_BytesAvail)) /* 可再写字节(留 1 位判空/满) */

/*********************************************************************
 * @fn      hid_up_avail
 *
 * @brief   查询上行字节流剩余可写字节数(供生产者判断整帧可否一次入)。
 *
 * @return  剩余可写字节数
 */
uint16_t hid_up_avail(void)
{
    uint16_t used = hid_BytesAvail;
    return (uint16_t)(DEF_UART_BUF_SIZE - 1 - used);
}

/*********************************************************************
 * @fn      hid_up_enqueue
 *
 * @brief   把 len 字节按 STRICT 顺序写入 USBHS-HID 上行环形缓冲。
 *          可安全地在中断(尤其 IMU 的 DMA-RX/DRDY 中断)调用。
 *          应保证 len <= 当前空闲(= hid_up_avail()),否则整体丢弃返回 0;
 *          一次写不超过容量,跨环尾自动折返,生产/消费读指针不重叠。
 *
 * @param   p   - 源(字节流/整帧均可,外部保证其内存持续到调用返回)
 *          len - 想写入字节数(<= DEF_UART_BUF_SIZE-1)
 *
 * @return  1:成功写入 len 字节;0:缓冲不足,本次整段放弃(严格顺序不受破坏)
 */
uint8_t hid_up_enqueue(const uint8_t *p, uint16_t len)
{
    uint16_t head, tail, first;

    if(len == 0) return 1;

    /* 并发协调：生产者可能被高频中断打断、消费者在主循环减 avail。
       这里两次都只建立在本地变量的快照判断上，仍是单写者；仅当写空余不够就放弃。 */
    if(len > (DEF_UART_BUF_SIZE - 1)) return 0;      /* 请求本身超容量 */

    head = hid_WrPtr;
    tail = hid_RdPtr;

    /* 可写字节 = 容量 - (当前在途量) - 1 */
    if((hid_BytesAvail + len) > (DEF_UART_BUF_SIZE - 1)) return 0;

    first = (uint16_t)(DEF_UART_BUF_SIZE - head);     /* 到尾的空档 */
    if(len > first)
    {
        memcpy(UART_RxBuffer + head, p, first);
        memcpy(UART_RxBuffer, p + first, (uint16_t)(len - first));
    }
    else
    {
        memcpy(UART_RxBuffer + head, p, len);
    }
    hid_WrPtr = (uint16_t)((head + len) % DEF_UART_BUF_SIZE);
    hid_BytesAvail = (uint16_t)(hid_BytesAvail + len);  /* 入队完成后一次性发布,避免半写可见 */
    return 1;
}

/*********************************************************************
 * @fn      _hid_ep_send_one
 *
 * @brief   内部：若 EP2(CDC 数据 IN bulk)空闲(NAK)且有可发字节，则从环形
 *          尾取至多一个 bulk 包净长(HS 512 / FS 64)直接灌 EP2 送出。
 *          CDC/ACM 上传是“裸字节流”——不带任何长度头；单一消费者在主
 *          循环调用，严格顺序。若环形恰好是整倍 bulk，末尾短包再续发。
 *
 * @return  1 = 本包已装载交 USB；0 = 暂无可发(缓冲空/端点忙)
 */
static uint8_t _hid_ep_send_one(void)
{
    uint16_t bulk, pkg, rd, availo;
    uint8_t  spdHS;

    /* 端点必须空闲(NAK)才可装载 */
    if( (USBHSD->UEP2_TX_CTRL & USBHS_UEP_T_RES_MASK) != USBHS_UEP_T_RES_NAK ) return 0;

    if(hid_BytesAvail == 0) return 0;

    spdHS = (USBHS_DevSpeed == USBHS_SPEED_HIGH);
    bulk  = spdHS ? DEF_USBD_HS_PACK_SIZE : DEF_USBD_FS_PACK_SIZE;
    availo = hid_BytesAvail;
    pkg = (availo >= bulk) ? bulk : availo;

    rd = hid_RdPtr;
    /* 环尾不足则硬切 */
    if(pkg > (DEF_UART_BUF_SIZE - rd)) pkg = (uint16_t)(DEF_UART_BUF_SIZE - rd);
    if(pkg == 0) return 0;

    /* CDC:直接装净字节,无头部;非整包实际长度即 pkg(短尾本身即 bulk 结束零长后随下包) */
    memcpy(USBHS_EP2_Tx_Buf, UART_RxBuffer + rd, pkg);

    USBHSD->UEP2_TX_DMA  = (uint32_t)(uint8_t *)USBHS_EP2_Tx_Buf;
    USBHSD->UEP2_TX_LEN  = pkg;                       /* bulk 按实际字节上报(非定长) */
    USBHSD->UEP2_TX_CTRL = ((USBHSD->UEP2_TX_CTRL)&~USBHS_UEP_T_RES_MASK) | USBHS_UEP_T_RES_ACK;

    rd = (uint16_t)((rd + pkg) % DEF_UART_BUF_SIZE);
    hid_RdPtr = rd;
    hid_BytesAvail = (uint16_t)(availo - pkg);
    return 1;
}

/*********************************************************************
 * @fn      hid_up_flush
 *
 * @brief   主循环高频调用：当前端点空闲(NAK)且有可发字节时，至多发一个
 *          HID 报告灌 EP2。非阻塞、每次至多一包，由调用方高频推进；
 *          USBHS_IRQHandler(EP2 T_DONE)只做清标志与回 NAK，不做装载
 *          —— 装载统一在主循环完成，单一消费者无并发、严格顺序。
 *
 * @return  none(调用方自己调度)
 */
void hid_up_flush(void)
{
    _hid_ep_send_one();
}
