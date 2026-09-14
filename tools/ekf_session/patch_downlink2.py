# -*- coding: utf-8 -*-
"""patch_downlink 的续跑：CRLF 自适应 + 已改过的文件跳过。"""
import re
import shutil

R = r'C:\Users\33\Documents\v2\h415-imu42605-'
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
    raise SystemExit('decode fail %s' % p)


def wr(p, t, e):
    shutil.copy2(p, p + '.bak_down')
    open(p, 'wb').write(t.encode(e))


def fit(t, s):
    return s.replace('\n', '\r\n') if '\r\n' in t else s


# ---- 3. EP2 OUT 中断 ----
t, e = rd(D)
if 'hid_down_push' in t:
    print('3 EP2 OUT  已改过, 跳过')
else:
    A = fit(t, '''            case DEF_UEP2:
                USBHSD->UEP2_RX_CTRL &= ~USBHS_UEP_R_DONE;''')
    assert t.count(A) == 1, 'EP2 锚点 %d (CRLF=%s)' % (t.count(A), '\r\n' in t)
    t = t.replace(A, fit(t, '''            case DEF_UEP2:
                /* 下行：主机→设备 bulk OUT。中断里只入环、不解析（与上行同纪律） */
                hid_down_push((const uint8_t *)USBHS_EP2_Rx_Buf, (uint16_t)USBHSD->UEP2_RX_LEN);
                USBHSD->UEP2_RX_CTRL &= ~USBHS_UEP_R_DONE;'''), 1)
    A2 = '#include "ch32h417_usbhs_device.h"'
    if 'usbd_compatibility_hid.h' not in t and t.count(A2) >= 1:
        t = t.replace(A2, A2 + fit(t, '\n#include "usbd_compatibility_hid.h"'), 1)
    wr(D, t, e)
    print('3 EP2 OUT  OK')

# ---- 4. USBCDC.c ----
t, e = rd(U)
if 'hid_cmd_poll' in t:
    print('4 USBCDC   已改过, 跳过')
else:
    i = t.index('hid_up_flush()')
    j = t.index(';', i) + 1
    t = t[:j] + fit(t, '\n\n    hid_cmd_poll();                     /* 下行：解析主机命令（非阻塞） */') + t[j:]
    wr(U, t, e)
    print('4 USBCDC   OK')

# ---- 5. SPI_rx.c ----
t, e = rd(S)
if 'g_cmd_echo' in t:
    print('5 SPI_rx   已改过, 跳过')
else:
    A = '#define JF_CH_NUM     80u'
    assert t.count(A) == 1
    t = t.replace(A, '#define JF_CH_NUM     83u    /* 80 + 下行验证 3 列 */', 1)
    A = fit(t, '    ch[c++] = JF_FW_TAG;                            /* 构建指纹：每次刷完先核对它 */')
    assert t.count(A) == 1, 'ch 锚点 %d' % t.count(A)
    t = t.replace(A, A + fit(t, '''

    /* 81~83 下行（主机→设备）验证：发命令后看这三列
     *   'T' <f32> 写 cmd_echo -> 上报原样回显，且 cmd_cnt 递增，即"下行通 + 上行看得见" */
    ch[c++] = g_cmd_echo;
    ch[c++] = (float)g_cmd_cnt;
    ch[c++] = (float)g_cmd_last;'''), 1)
    A = '#define JF_FRAME_LEN  (JF_CH_NUM * 4u + 6u)      /* 326 = 帧头2+帧长2+载荷+帧尾2 */'
    if t.count(A) == 1:
        t = t.replace(A, '#define JF_FRAME_LEN  (JF_CH_NUM * 4u + 6u)      /* 338 = 帧头2+帧长2+载荷332+帧尾2 */', 1)
    wr(S, t, e)
    print('5 SPI_rx   OK')

# ---- 6. 版本 ----
t, e = rd(T)
if '#define V5F_FW_VER        9u' in t:
    print('6 VER      已是 9, 跳过')
else:
    assert t.count('#define V5F_FW_VER        8u') == 1
    t = t.replace('#define V5F_FW_VER        8u', '#define V5F_FW_VER        9u', 1)
    wr(T, t, e)
    print('6 VER 8->9 OK')

print()
print('fw_tag 期望 = %d  (VER=9, 通道=83, EKF=0, MAGCAL=1, AC=1)' % ((9 << 16) | (83 << 8) | 0 | 2 | 4))
for p in (D, U, S, T):
    tt, enc = rd(p)
    print('  %-26s %s  注释配平=%s' % (p.split('\\')[-1], enc, tt.count('/*') == tt.count('*/')))
