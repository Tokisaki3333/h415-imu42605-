# -*- coding: utf-8 -*-
"""把 JF_CH_NUM 从 111 校正为运行时真值 112，并把上报块的列号注释改成 0 基、与读取端一致。"""
import shutil

S = r'C:\Users\33\Documents\v2\h415-imu42605-\V5F\User\src\SPI_rx.c'
t = open(S, 'rb').read().decode('gbk')
n0 = len(t)

a = '#define JF_CH_NUM     111u   /* 80 + 下行验证 3 + 导航 EKF 28 列 */'
b = ('#define JF_CH_NUM     112u   /* **运行时**实际写入的列数。不是估的：tools/calib/count_cols.py\n'
     '                              * 逐行数 ch[c++] 得到静态 114，减掉 DRDY 间隔那条 if/else\n'
     '                              * 链的 2 个未执行分支 = 112。ch[] 是栈上数组，多写一格就是\n'
     '                              * 栈踩踏而编译器一个字都不会说 —— 改列必须用那个脚本复核。*/')
assert t.count(a) == 1
t = t.replace(a, b, 1)

a = '#define JF_FRAME_LEN  (JF_CH_NUM * 4u + 6u)      /* 450 = 帧头2+帧长2+载荷444+帧尾2 */'
b = '#define JF_FRAME_LEN  (JF_CH_NUM * 4u + 6u)      /* 454 = 帧头2+帧长2+载荷448+帧尾2 */'
assert t.count(a) == 1
t = t.replace(a, b, 1)

a = '''    /* 84~111 导航 EKF（处理函数 7/8）—— 影子模式：与旧链并行跑、全部记录，
     * 达标前不影响任何旧输出。设计见仓库根 ekf_design.md，常量见 v5f_tune.h J 组。
     *  84~86   ekf.p[3]        导航系位置 E/N/U，m（原点 = ENU 对齐点，见列 104 的 bit10）
     *  87~89   ekf.v[3]        导航系速度 m/s
     *  90~93   ekf.q[4]        重力系世界四元数（机体->导航）
     *  94~96   ekf.a_nav[3]    导航系线性加速度 m/s^2（已去重力与零偏；"合加速度"的 EKF 版）
     *  97~99   ekf.ba[3]       加计零偏残差 m/s^2
     * 100~102  ekf.bg[3]       陀螺零偏残差 dps
     * 103      ekf.b_baro      气压高度零偏 m（气压高度 = p_z + b_baro）
     * 104      ekf.gate_bits   V5F_EKF_GB_* 位：bit0 GPS位置 bit1 GPS高度 bit2 气压
     *                          bit3 GPS速度 bit4 ZUPT bit5 重力/倾斜 bit6 磁偏航(闭环)
     *                          bit7 已对齐 bit8 本周期有EKF步 bit9 本周期有观测被chi2剔除
     *                          bit10 ENU原点已建立
     * 105      sigma_yaw_deg   偏航 1sigma（度）—— 门开被 M7 收紧、门关按 Q_bg 增长，
     *                          "航向现在能不能信"的直接读数（失效可观测，不是静默失效）
     * 106      sigma_pos_h     水平位置 1sigma m
     * 107      sigma_vel_h     水平速度 1sigma m/s
     * 108~112  ekf.nis[5]      归一化新息平方：位置/速度/气压/重力/磁偏航。
     *                          应分别趋近 2/2/1/3/1；远大于维数 -> R 给小了。
     * ★ 1~83 列号一个都不动，新量只追加在尾部。 */'''
b = '''    /* 85~111 导航 EKF（处理函数 7/8）—— 影子模式：与旧链并行跑、全部记录，
     * 达标前不影响任何旧输出。设计见仓库根 ekf_design.md，常量见 v5f_tune.h J 组。
     * ★ 下面一律 **0 基**列号，与 PC 端 tools/calib/jf_load.py 的表一致
     *   （0 基 25 = 当帧 dt_us，0 基 78 = fw_tag，0 基 79~81 = 下行验证三列，
     *     0 基 82 = 气压软件平均，0 基 83/84 = e_ac / ac_bypass）。
     *  85..87   ekf.p[3]        导航系位置 E/N/U，m（原点 = ENU 对齐点，见 gate_bits bit10）
     *  88..90   ekf.v[3]        导航系速度 m/s
     *  91..94   ekf.q[4]        重力系世界四元数（机体->导航）
     *  95..97   ekf.a_nav[3]    导航系线性加速度 m/s^2（已去重力与零偏；"合加速度"的 EKF 版）
     *  98..100  ekf.ba[3]       加计零偏残差 m/s^2
     * 101..103  ekf.bg[3]       陀螺零偏残差 dps
     * 104       ekf.b_baro      气压高度零偏 m（气压高度 = p_z + b_baro）
     * 105       ekf.gate_bits   V5F_EKF_GB_* 位：bit0 GPS位置 bit1 GPS高度 bit2 气压
     *                          bit3 GPS速度 bit4 ZUPT bit5 重力/倾斜 bit6 磁偏航(闭环)
     *                          bit7 已对齐 bit8 本周期有EKF步 bit9 本周期有观测被chi2剔除
     *                          bit10 ENU原点已建立
     * 106       sigma_yaw_deg   偏航 1sigma（度）—— 门开被 M7 收紧、门关按 Q_bg 增长，
     *                          "航向现在能不能信"的直接读数（失效可观测，不是静默失效）
     * 107       sigma_pos_h     水平位置 1sigma m
     * 108       sigma_vel_h     水平速度 1sigma m/s
     * 109..113  ekf.nis[5]      归一化新息平方：位置/速度/气压/重力/磁偏航。
     *                          应分别趋近 2/2/1/3/1；远大于维数 -> R 给小了。
     * ★ 0~84 列号一个都不动，新量只追加在尾部。 */'''
assert t.count(a) == 1
t = t.replace(a, b, 1)

assert len(t) > n0 and t.count('/*') == t.count('*/')
shutil.copy2(S, S + '.bak_ekf3')
open(S, 'wb').write(t.encode('gbk'))
print('JF_CH_NUM 111 -> 112 OK')
print('fw_tag 期望 = %d  (VER=10, 112 列, EKF=1, MAGCAL=1, AC=1)'
      % ((10 << 16) | (112 << 8) | 1 | 2 | 4))
