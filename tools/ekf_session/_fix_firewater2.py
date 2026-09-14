# -*- coding: utf-8 -*-
"""FireWater 输出改进：
 ① **原子读取**：ekf.q / att.q 是各 4 个 float，中断 349 Hz 在改、主循环在读，
    读到一半被打断就拼出"新 w,x + 旧 y,z"的混合四元数 -> 看着就是弹跳/自震。
    旧链每周期只被陀螺平滑积分改动，混合偏差极小所以看不出；EKF 每周期被观测做
    一次离散修正，混合偏差就明显了 —— 正是"CH0~3 弹跳、CH4~7 稳定"。
    修：读的时候关中断（16 字节的拷贝，几微秒）。
 ② 诊断通道（从 8 路扩到 12 路）：
    ch0-3 ekf_q(w,x,y,z)   ch4-7 att_q(w,x,y,z)
    ch8   |ekf_q|^2        （撕裂会立刻让它偏离 1）
    ch9-11 gyro_dps(x,y,z) （原始角速度：判断抖动是不是真运动）
"""
import shutil

M = r'C:\Users\33\Documents\v2\h415-imu42605-\V5F\User\main.c'
t = open(M, 'rb').read().decode('gbk')

i0 = t.index('        /* ---- FireWater')
i1 = t.index('    }\n}', i0)
old = t[i0:i1]
assert 'FireWater' in old and 'printf' in old

new = '''        /* ---- FireWater（VOFA+ 文本协议）：20 Hz 输出四元数与诊断量 ----
         * 放在主循环、**不放中断**：printf 是阻塞式的，115200 baud 下一行约 6~9 ms，
         * 丢进 8 kHz 的 DMA 中断会直接毁掉时间轴。
         * 12 路（逗号分隔，\\n 结束）：
         *   ch0-3   ekf_q.w,.x,.y,.z     导航系 = 真 ENU（磁力计一次性对齐）
         *   ch4-7   att_q.w,.x,.y,.z     旧链，导航系 = 上电时 x 轴的水平投影
         *   ch8     |ekf_q|^2            必须恒为 1.0000；偏离就是**读取撕裂**
         *   ch9-11  gyro_dps.x,.y,.z     原始角速度：判断抖动是不是真运动
         * ★ ekf_q 与 att_q 只差一个绕竖直轴的**固定旋转**（实测 = -D = +7.53 度），
         *   直接比分量会看到固定偏航差，那不是误差；比精度请比转角变化量。
         * ★ 阻塞打印期间主循环无法灌 EP2，上行环只有约 2 ms 余量 —— 若 HID 采集
         *   掉帧，先把这里降到 10 Hz 或提高调试串口波特率。 */
        {
            static uint64_t s_fw_last;
            uint64_t fw_tv = GetTime64_Us();
            if (fw_tv - s_fw_last >= 50000ULL)      /* 20 Hz */
            {
                float eq[4], aq[4], n2;
                s_fw_last = fw_tv;
                /* ★ 原子拷贝：四个 float 必须一次读完，否则会拼出新旧混合的四元数，
                 *   在曲线上就是弹跳/自震。关中断只有十几条指令的时间。 */
                __disable_irq();
                eq[0] = g_v5f_hold.ekf.q[0]; eq[1] = g_v5f_hold.ekf.q[1];
                eq[2] = g_v5f_hold.ekf.q[2]; eq[3] = g_v5f_hold.ekf.q[3];
                aq[0] = g_v5f_hold.att.q[0]; aq[1] = g_v5f_hold.att.q[1];
                aq[2] = g_v5f_hold.att.q[2]; aq[3] = g_v5f_hold.att.q[3];
                __enable_irq();
                n2 = eq[0]*eq[0] + eq[1]*eq[1] + eq[2]*eq[2] + eq[3]*eq[3];
                printf("%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.5f,"
                       "%.2f,%.2f,%.2f\\n",
                       (double)eq[0], (double)eq[1], (double)eq[2], (double)eq[3],
                       (double)aq[0], (double)aq[1], (double)aq[2], (double)aq[3],
                       (double)n2,
                       (double)g_v5f_hold.imu.gyro_dps[0],
                       (double)g_v5f_hold.imu.gyro_dps[1],
                       (double)g_v5f_hold.imu.gyro_dps[2]);
                usbhs_hid_poll();                   /* 打印前后各灌一次，少压上行环 */
            }
        }
'''
t = t[:i0] + new + t[i1:]
assert t.count('/*') == t.count('*/') and t.count('{') == t.count('}')
data = t.encode('gbk')
shutil.copy2(M, M + '.bak_fw2')
open(M, 'wb').write(data)

c = open(M, 'rb').read().decode('gbk')
for k, v in [('临界区在读之前', c.index('__disable_irq();') < c.index('eq[0] = g_v5f_hold.ekf.q[0]')),
             ('临界区已配对', c.count('__disable_irq();') == c.count('__enable_irq();')),
             ('12 路', c.count('%.4f') == 8 and c.count('%.2f') == 3 and c.count('%.5f') == 1),
             ('花括号平衡', c.count('{') == c.count('}')),
             ('注释配平', c.count('/*') == c.count('*/'))]:
    print('  %-18s %s' % (k, v))
    assert v, k
print()
print('main.c 已改（%d -> %d B）' % (len(open(M + '.bak_fw2', 'rb').read()), len(data)))
