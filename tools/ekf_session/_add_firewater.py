# -*- coding: utf-8 -*-
"""V5F printf 串口上以 VOFA+ FireWater（文本）协议 20 Hz 输出四元数。

放在**主循环**，不放中断：printf 是阻塞式的，115200 baud 下一行约 6 ms，
放进 8 kHz 的 DMA 中断会直接把时间轴毁掉。

通道（8 路，逗号分隔、\n 结束）：
    ekf_q.w, ekf_q.x, ekf_q.y, ekf_q.z,  att_q.w, att_q.x, att_q.y, att_q.z
★ 注意：ekf_q 的导航系是**真 ENU**（用磁力计一次性对齐），att_q 的导航系是
  上电时 x 轴的水平投影 —— 两者只差一个**绕竖直轴的固定旋转**（实测该偏移
  = -D = +7.53 度）。所以直接比分量会看到一个固定偏航差，那不是误差；
  要比精度请比**转角的变化量**。
"""
import shutil

M = r'C:\Users\33\Documents\v2\h415-imu42605-\V5F\User\main.c'
t = open(M, 'rb').read().decode('gbk')

old = """        usbhs_hid_poll();               /* 上行环有字节就灌 1 个 EP2 包 */"""
assert t.count(old) == 1, t.count(old)
new = """        usbhs_hid_poll();               /* 上行环有字节就灌 1 个 EP2 包 */

        /* ---- FireWater（VOFA+ 文本协议）：20 Hz 输出四元数供检查 ----
         * 放在主循环、**不放中断**：printf 是阻塞式的，115200 baud 下一行约 6 ms，
         * 丢进 8 kHz 的 DMA 中断会直接毁掉时间轴。
         * 8 路：ekf_qw,qx,qy,qz, att_qw,qx,qy,qz（逗号分隔，\\n 结束）
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
                printf("%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\\n",
                       (double)g_v5f_hold.ekf.q[0], (double)g_v5f_hold.ekf.q[1],
                       (double)g_v5f_hold.ekf.q[2], (double)g_v5f_hold.ekf.q[3],
                       (double)g_v5f_hold.att.q[0], (double)g_v5f_hold.att.q[1],
                       (double)g_v5f_hold.att.q[2], (double)g_v5f_hold.att.q[3]);
                usbhs_hid_poll();                   /* 打印前后各灌一次，尽量少压上行环 */
            }
        }"""
t = t.replace(old, new, 1)
assert t.count('/*') == t.count('*/')
assert t.count('{') == t.count('}')

data = t.encode('gbk')                      # 先编码，失败则文件不动
shutil.copy2(M, M + '.bak_fw')
with open(M, 'wb') as f:
    f.write(data)

c = open(M, 'rb').read().decode('gbk')
for k, v in [('FireWater 已加', 'FireWater' in c),
             ('20 Hz 判据', '>= 50000ULL' in c),
             ('8 路', c.count('(double)g_v5f_hold') == 8),
             ('花括号平衡', c.count('{') == c.count('}')),
             ('注释配平', c.count('/*') == c.count('*/'))]:
    print('  %-16s %s' % (k, v))
    assert v, k
print()
print('main.c 已改（%d B -> %d B）' % (len(open(M + '.bak_fw', 'rb').read()), len(data)))
print('提示：固件版本号未变（VER 仍为 35），但源码改了 —— 重新编译即可，')
print('      若要分辨是否刷入新版，可让我把 VER 提到 36。')
