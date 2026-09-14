# -*- coding: utf-8 -*-
"""把 VER=10 / 112 列的**权威列名表**加进 tools/calib/jf_load.py。

为什么必须新建而不是改旧表：旧的 CH_92 是"计划里"的布局（连 gps_pdop、gps_status 这种
当前固件根本没发的列都有），列号与实际发射顺序**已经漂了**；CH_80 也没算下行三列。
列号这种事只能有一个来源：tools/calib/count_cols.py 逐行数 ch[c++] 的结果。
本表就是那份结果，0 基，112 项。
"""
import shutil

P = r'C:\Users\33\Documents\v2\h415-imu42605-\tools\calib\jf_load.py'
t = open(P, 'rb').read().decode('utf-8')
n0 = len(t)

TBL = '''
# ---- VER=10（S1 落地）：112 列，**权威表** -------------------------------------
# 来源：tools/calib/count_cols.py 逐行数 justfloat_report 的 ch[c++]（运行时口径，
# 即 if/else 链只算一个分支）。不是从别处抄的，也不是估的。
#   为什么另起一张表：CH_92 是"计划布局"，含当前固件根本没发的列（gps_pdop/gps_status），
#   列号已整体漂移；CH_80 也没把下行验证三列算进去。列号只能有一个来源。
# 关键列： dt_us=25  fw_tag=76  下行三列=77/78/79  press_avg=80  e_ac=81/82  EKF=83..111
# ★ 本表只有一种列号：**0 基**。
CH_112 = {
    'q': 0,                    # 重力系世界四元数（旧链 proc_attitude）
    'v_nav': 4,                # 旧链世界系速度（处理函数 6）
    'level_dps': 7,            # 判静统计量
    'is_static': 8,
    'acc_traction': 9,         # 加速度零偏牵引门
    'a_lin': 10,               # 牵引驱动量（g）
    'accel_bias_g': 13,
    'flags': 16,               # 打包标志位（bit0..bit11）
    'dma1_irq_us': 17,         # 上一帧 DMA1 ISR 耗时
    'gyro_lsb': 18,
    'accel_lsb': 21,
    'temp_imu': 24,
    'dt_us': 25,               # 当帧 DRDY 间隔（1 tick = 10 ns）
    'gyro_dps': 26,
    'gyro_bias_dps': 29,
    'accel_g': 32,
    'bias_evidence_gran': 35,
    'att_tilt': 36,            # 重力/倾斜融合门（门B，旧链）
    'vel_soft': 37,            # 软 ZUPT 置信度 0..255
    'mag_lsb': 38,
    'ist_cnt': 41,
    'mag_f': 42,               # 机体系地磁单位方向（H 组标定后）
    'mag_norm': 45,            # 归一化前的模长（污染门观测量，理论 1.0）
    'mag_Bw': 46,              # R(q_旧链) f，导航系地磁方向
    'psi_true': 49,            # 旧链的真航向（= psi_mag + D）
    'mag_trust': 50,
    'baro_press': 51,          # 单样本压力 Pa
    'baro_temp': 52,
    'baro_cnt': 53,
    'gps_speed': 54,           # RMC 对地速度 m/s
    'gps_rmc_flags': 55,       # SHM_RMC_*（status/pos/speed/date/course/magvar）
    'gps_rmc_cnt': 56,
    'gps_alt': 57,             # GGA 海拔 m
    'gps_fixq': 58,            # 0 无 / 1 单点 / 2 差分 / 4 RTK 固定 / 5 RTK 浮点
    'gps_sv': 59,              # 参与定位卫星数
    'gps_hdop': 60,
    'gps_gga_flags': 61,       # SHM_GGA_*
    'gps_gga_cnt': 62,
    'gps_vdop': 63,            # 只报 VDOP；PDOP 当前固件不发
    'gps_gsa_cnt': 64,
    'gps_course': 65,          # 对地航向（真北，度）
    'gps_lat_int': 66,         # 经纬度无损重建：lat_deg()/lon_deg()
    'gps_lat_frac': 67,
    'gps_lon_int': 68,
    'gps_lon_frac': 69,
    'gps_sats_view': 70,       # GSV 视野内卫星数
    'gps_snr_avg': 71,         # 全天空加权 C/N0 dBHz
    'gps_snr_min': 72,
    'gps_snr_n': 73,
    'gps_gsv_cnt': 74,
    'gps_gsv_talkers': 75,     # 本轮 talker 数（轮次聚合是否正常的直接读数）
    'fw_tag': 76,              # 构建指纹 (VER<<16)|(通道数<<8)|开关位
    # --- 下行（主机->设备）验证三列 ---
    'cmd_echo': 77,            # 'T' <f32> 写进来的值原样回显
    'cmd_cnt': 78,             # 收到的合法命令数（单调递增 = 下行真的通了）
    'cmd_last': 79,            # 最近一条命令的 opcode（ASCII）
    # --- B 组/处理函数 2 ---
    'baro_press_avg': 80,      # 软件 OSR 平均后压力（V5F_BARO_AVG_W 个样本）
    'stat_e_ac': 81,           # 三轴交流能量 dps^2（对直流零偏不可见）
    'stat_ac_bypass': 82,      # 启动阶段旁路门
    # --- 导航 EKF（处理函数 7/8，S1 影子模式）---
    'ekf_p': 83,               # 导航系位置 E/N/U，m（原点 = ENU 对齐点，见 gate bit10）
    'ekf_v': 86,               # 导航系速度 m/s
    'ekf_q': 89,               # 重力系世界四元数（机体->导航），导航系 = 真 ENU
    'ekf_a_nav': 93,           # 导航系线性加速度 m/s^2（已去重力与零偏）
    'ekf_ba': 96,              # 加计零偏残差 m/s^2
    'ekf_bg': 99,              # 陀螺零偏残差 dps
    'ekf_b_baro': 102,         # 气压高度零偏 m（气压高度 = p_z + b_baro）
    'ekf_gate_bits': 103,      # 见 v5f_tune.h 的 V5F_EKF_GB_*
    'ekf_sigma_yaw_deg': 104,  # 偏航 1sigma（度）：门开被 M7 收紧、门关按 Q_bg 增长
    'ekf_sigma_pos_h': 105,
    'ekf_sigma_vel_h': 106,
    'ekf_nis': 107,            # 5 项：位置/速度/气压/重力/磁偏航，应趋近 2/2/1/3/1
}

# EKF gate_bits 的位名（上报列 103）
EKF_GB = [
    (0x0001, 'gps_pos'), (0x0002, 'gps_alt'), (0x0004, 'baro'),
    (0x0008, 'gps_vel'), (0x0010, 'zupt'), (0x0020, 'tilt'),
    (0x0040, 'mag_yaw'), (0x0080, 'aligned'), (0x0100, 'step'),
    (0x0200, 'chi2_rej'), (0x0400, 'origin'),
]

'''

a = "CH_BY_NCH = {90: CH_92, 92: CH_92, 78: CH_78, 80: CH_80}"
assert t.count(a) == 1
t = t.replace(a, TBL.lstrip('\n') +
              "CH_BY_NCH = {90: CH_92, 92: CH_92, 78: CH_78, 80: CH_80, 112: CH_112}", 1)

a = "# 默认最新版；读历史记录请用 ch_for(a.shape[1])\nCH = CH_80"
assert t.count(a) == 1
t = t.replace(a, "# 默认最新版；读历史记录请用 ch_for(a.shape[1])\nCH = CH_112", 1)

a = """    if nch == 80:
        return CH_80"""
assert t.count(a) == 1
t = t.replace(a, """    if nch == 80:
        return CH_80
    if nch == 112:
        return CH_112""", 1)

a = '    raise KeyError("未知帧长 %d 通道；已知 78/80 与所有 <=92 的历史版" % nch)'
assert t.count(a) == 1
t = t.replace(a, '    raise KeyError("未知帧长 %d 通道；已知 78/80/112 与所有 <=92 的历史版" % nch)', 1)

# 92 的历史前缀语义：112 > 92，所以旧注释里"凡是 <=92 都是 CH_92 前缀"仍然成立
assert len(t) > n0
shutil.copy2(P, P + '.bak_s1')
open(P, 'wb').write(t.encode('utf-8'))
print('jf_load.py: CH_112 已加入 (112 列, fw_tag@76, EKF@83..111)')

# ---- 自检：列名索引必须严格递增、互不重叠 ----
import importlib
import sys
sys.path.insert(0, r'C:\Users\33\Documents\v2\h415-imu42605-\tools\calib')
import jf_load
importlib.reload(jf_load)
c = jf_load.CH_112
span = {'q': 4, 'v_nav': 3, 'a_lin': 3, 'accel_bias_g': 3, 'gyro_lsb': 3,
        'accel_lsb': 3, 'gyro_dps': 3, 'gyro_bias_dps': 3, 'accel_g': 3,
        'mag_lsb': 3, 'mag_f': 3, 'mag_Bw': 3, 'ekf_p': 3, 'ekf_v': 3,
        'ekf_q': 4, 'ekf_a_nav': 3, 'ekf_ba': 3, 'ekf_bg': 3, 'ekf_nis': 5}
used = {}
for k, v in c.items():
    for q in range(span.get(k, 1)):
        assert (v + q) not in used, '列 %d 被 %s 与 %s 抢' % (v + q, used.get(v + q), k)
        used[v + q] = k
miss = [i for i in range(112) if i not in used]
print('自检: 覆盖 %d/112 列, 冲突 0, 未命名列 %s' % (len(used), miss))
print('     ch_for(112) 的 fw_tag =', jf_load.ch_for(112)['fw_tag'])
