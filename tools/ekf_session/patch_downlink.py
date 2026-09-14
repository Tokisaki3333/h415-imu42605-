# -*- coding: utf-8 -*-
"""打通下行（主机→设备）＋命令通道。不碰 USB 枚举/描述符 —— 官方库的 EP2 OUT 已就绪。

改动 5 处：
  1 common/USBHS/usbd_compatibility_hid.h  声明 hid_down_* / hid_cmd_poll / 命令状态
  2 common/USBHS/usbd_compatibility_hid.c  下行环形缓冲 + 命令解析器
  3 common/USBHS/ch32h417_usbhs_device.c   EP2 OUT 中断里把"丢弃"改成"入环"
  4 V5F/User/src/USBCDC.c                  usbhs_hid_poll() 里调用 hid_cmd_poll()
  5 V5F/User/src/SPI_rx.c                  上报追加 3 列 cmd_echo/cmd_cnt/cmd_last（80→83）

命令帧（与上行对称）：A5 5A | len(u16 小端) | 命令 | 5A A5
  'T' <f32>            测试：写专用字段（原样回显到 cmd_echo）
  'P' <u8 id> <f32>    设参数      'G' <u8 id>  读回
  'M' <u8 mask>        M1~M7 使能  'A'         强制重新对齐
"""
import re
import shutil

R = r'C:\Users\33\Documents\v2\h415-imu42605-'
H = R + r'\Common\USBHS\usbd_compatibility_hid.h'
C = R + r'\Common\USBHS\usbd_compatibility_hid.c'
D = R + r'\Common\USBHS\ch32h417_usbhs_device.c'
U = R + r'\V5F\User\src\USBCDC.c'
S = R + r'\V5F\User\src\SPI_rx.c'
T = R + r'\V5F\User\inc\v5f_tune.h'


def rd(p):
    b = open(p, 'rb').read()
    for e in ('utf-8', 'gbk'):
        try:
            return b.decode(e), e
        except Exception:
            pass
    raise SystemExit('解码失败 %s' % p)


def wr(p, t, e):
    shutil.copy2(p, p + '.bak_down')
    open(p, 'wb').write(t.encode(e))


# ---------- 1. .h ----------
t, e = rd(H)
A = 'extern void     hid_up_flush(void);                /* 主循环高频:有字节就把 1 个 CDC bulk 包灌 EP2 */'
assert t.count(A) == 1
t = t.replace(A, A + '''

/* ---- 下行（主机→设备）----
 * 生产者 = USBHS 中断里的 EP2 bulk OUT（只入环，不解析）
 * 消费者 = 主循环里的 hid_cmd_poll()（解析并执行）
 * 整包或丢弃：放不下就整段放弃，保证字节流严格顺序。 */
#define DEF_DOWN_BUF_SIZE               1024        /* 下行环形容量 */

extern void     hid_down_push(const uint8_t*, uint16_t len);  /* 中断里调用：只入环 */
extern void     hid_cmd_poll(void);                           /* 主循环调用：解析并执行 */

extern volatile float    g_cmd_echo;    /* 'T' 写进来的值，原样上报（下行验证用） */
extern volatile uint16_t g_cmd_cnt;     /* 收到的合法命令数 */
extern volatile uint8_t  g_cmd_last;    /* 最近一条命令的 opcode */''', 1)
wr(H, t, e)
print('1 .h  OK')

# ---------- 2. .c ----------
t, e = rd(C)
A = 'void hid_up_flush(void)\n\n{\n\n    _hid_ep_send_one();\n\n}'
if t.count(A) != 1:
    A = 'void hid_up_flush(void)'
    assert t.count(A) == 1, '.c 锚点匹配 %d' % t.count(A)
    i = t.index(A)
    j = t.index('}', t.index('_hid_ep_send_one();', i)) + 1
    t = t[:j] + '''

/******************************************************************************

 *                       下行：主机 → 设备

 ******************************************************************************/

#define DEF_DOWN_MASK  (DEF_DOWN_BUF_SIZE - 1u)

static uint8_t           s_down_buf[DEF_DOWN_BUF_SIZE];
static volatile uint16_t s_down_w = 0;
static volatile uint16_t s_down_r = 0;
static volatile uint16_t s_down_avail = 0;

volatile float    g_cmd_echo = 0.0f;
volatile uint16_t g_cmd_cnt  = 0u;
volatile uint8_t  g_cmd_last = 0u;

/* 中断里调用：整包入环，放不下就整段放弃 */
void hid_down_push(const uint8_t *p, uint16_t len)
{
    uint16_t first;

    if (len == 0u || len > (DEF_DOWN_BUF_SIZE - 1u)) return;
    if ((uint16_t)(s_down_avail + len) > (DEF_DOWN_BUF_SIZE - 1u)) return;

    first = (uint16_t)(DEF_DOWN_BUF_SIZE - s_down_w);
    if (len > first)
    {
        memcpy(s_down_buf + s_down_w, p, first);
        memcpy(s_down_buf, p + first, (uint16_t)(len - first));
    }
    else
    {
        memcpy(s_down_buf + s_down_w, p, len);
    }
    s_down_w = (uint16_t)((s_down_w + len) & DEF_DOWN_MASK);
    s_down_avail = (uint16_t)(s_down_avail + len);
}

/* 执行一条命令（len = 命令字节数） */
static void hid_cmd_exec(const uint8_t *p, uint16_t len)
{
    float f;

    switch (p[0])
    {
    case 'T':                                   /* 测试：写专用字段，原样回显 */
        if (len < 5u) return;
        memcpy(&f, p + 1u, 4u);
        g_cmd_echo = f;
        break;
    case 'P':                                   /* 设参数 id = 值（S2 接 J 组） */
        if (len < 6u) return;
        break;
    case 'G':                                   /* 读回参数 */
        if (len < 2u) return;
        break;
    case 'M':                                   /* M1~M7 使能位图 */
        if (len < 2u) return;
        break;
    case 'A':                                   /* 强制重新对齐 */
        break;
    default:
        return;                                 /* 未知命令：不计入、不改变状态 */
    }
    g_cmd_last = p[0];
    if (g_cmd_cnt < 0xFFFFu) g_cmd_cnt++;
}

/* 主循环调用：从下行环取字节，按 A5 5A | len | 命令 | 5A A5 解析 */
void hid_cmd_poll(void)
{
    static uint8_t  st = 0u;
    static uint8_t  lenlo = 0u, lenhi = 0u;
    static uint8_t  cbuf[64];
    static uint16_t need = 0u, got = 0u;
    uint8_t b;

    while (s_down_avail > 0u)
    {
        b = s_down_buf[s_down_r];
        s_down_r = (uint16_t)((s_down_r + 1u) & DEF_DOWN_MASK);
        s_down_avail--;

        switch (st)
        {
        case 0u: if (b == 0xA5u) st = 1u; break;
        case 1u: st = (b == 0x5Au) ? 2u : ((b == 0xA5u) ? 1u : 0u); break;
        case 2u: lenlo = b; st = 3u; break;
        case 3u:
            lenhi = b;
            need = (uint16_t)(lenlo | ((uint16_t)lenhi << 8));
            if (need == 0u || need > (uint16_t)sizeof(cbuf)) { st = 0u; break; }
            got = 0u; st = 4u;
            break;
        case 4u:
            cbuf[got++] = b;
            if (got >= need) st = 5u;
            break;
        case 5u: st = (b == 0x5Au) ? 6u : 0u; break;
        case 6u:
            if (b == 0xA5u) hid_cmd_exec(cbuf, need);
            st = 0u;
            break;
        default: st = 0u; break;
        }
    }
}'''
    wr(C, t, e)
    print('2 .c  OK')

# ---------- 3. EP2 OUT 中断 ----------
t, e = rd(D)
A = '''            case DEF_UEP2:
                USBHSD->UEP2_RX_CTRL &= ~USBHS_UEP_R_DONE;'''
assert t.count(A) == 1, 'EP2 锚点匹配 %d' % t.count(A)
t = t.replace(A, '''            case DEF_UEP2:
                /* 下行：主机→设备 bulk OUT。中断里只入环、不解析（与上行同纪律） */
                hid_down_push((const uint8_t *)USBHS_EP2_Rx_Buf, (uint16_t)USBHSD->UEP2_RX_LEN);
                USBHSD->UEP2_RX_CTRL &= ~USBHS_UEP_R_DONE;''', 1)
A2 = '#include "ch32h417_usbhs_device.h"'
assert t.count(A2) >= 1
if 'usbd_compatibility_hid.h' not in t:
    t = t.replace(A2, A2 + '\n#include "usbd_compatibility_hid.h"', 1)
wr(D, t, e)
print('3 EP2 OUT OK')

# ---------- 4. USBCDC.c ----------
t, e = rd(U)
A = '    if (hid_up_avail())\n\n        hid_up_flush();'
if t.count(A) != 1:
    A = 'if (hid_up_avail())'
    assert t.count(A) == 1, 'USBCDC 锚点 %d' % t.count(A)
    i = t.index(A)
    j = t.index(';', t.index('hid_up_flush()', i)) + 1
    t = t[:j] + '\n\n    hid_cmd_poll();                     /* 下行：解析主机命令（非阻塞） */' + t[j:]
wr(U, t, e)
print('4 USBCDC.c OK')

# ---------- 5. SPI_rx.c 追加 3 列 ----------
t, e = rd(S)
A = '#define JF_CH_NUM     80u'
assert t.count(A) == 1
t = t.replace(A, '#define JF_CH_NUM     83u    /* 80 + 下行验证 3 列 */', 1)
A = '    ch[c++] = JF_FW_TAG;                            /* 构建指纹：每次刷完先核对它 */'
assert t.count(A) == 1, 'ch 锚点 %d' % t.count(A)
t = t.replace(A, A + '''

    /* 81~83 下行（主机→设备）验证：发命令后看这三列
     *   'T' <f32> 写 cmd_echo -> 上报原样回显，且 cmd_cnt 递增，即"下行通 + 上行看得见" */
    ch[c++] = g_cmd_echo;
    ch[c++] = (float)g_cmd_cnt;
    ch[c++] = (float)g_cmd_last;''', 1)
for op, cl in (('(', ')'), ('{', '}')):
    d = (t.count(op) - rd(S)[0].count(op)) - (t.count(cl) - rd(S)[0].count(cl))
    assert d == 0, 'SPI_rx.c 括号不配平 %d' % d
t = t.replace('#define JF_FRAME_LEN  (JF_CH_NUM * 4u + 6u)      /* 326 = 帧头2+帧长2+载荷+帧尾2 */',
              '#define JF_FRAME_LEN  (JF_CH_NUM * 4u + 6u)      /* 338 = 帧头2+帧长2+载荷332+帧尾2 */', 1)
wr(S, t, e)
print('5 SPI_rx.c OK')

# ---------- 6. 版本 ----------
t, e = rd(T)
assert t.count('#define V5F_FW_VER        8u') == 1
t = t.replace('#define V5F_FW_VER        8u', '#define V5F_FW_VER        9u', 1)
wr(T, t, e)
print('6 VER 8->9, fw_tag = %d' % ((9 << 16) | (83 << 8) | 0 | 2 | 4))

# ---------- 自检 ----------
print()
for p in (H, C, D, U, S, T):
    _, enc = rd(p)
    tt, _ = rd(p)
    a, b = tt.count('/*'), tt.count('*/')
    print('  %-28s %s  注释配平=%s' % (p.split('\\')[-1], enc, a == b))
