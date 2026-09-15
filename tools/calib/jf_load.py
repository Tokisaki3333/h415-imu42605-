# -*- coding: utf-8 -*-
"""
JustFloat 串口日志的公共读取器（流式 + npy 缓存）

为什么需要它：日志是文本，单条记录 25~48 MB。原先的写法
    raw = b''.join(bytes.fromhex(...) for ... in re.findall(pat, open(fn).read()))
会把整份文件读成一个 Python str、再由 findall 造出几十万个小字符串，
峰值内存几百 MB，而且每跑一次脚本就把大文件重读一遍（Defender 还要再扫一遍）
—— 分析脚本反复跑就会看到上百 MiB 的磁盘读。
本模块按块流式解析，峰值只有输出数组那么大（几十 MB），并把结果缓存成
`<日志>.npy`，重跑只读 11 MB 的二进制缓存。.npy 已在 .gitignore 里。

帧格式：JustFloat N 通道 float + 4 字节帧尾 00 00 80 7F，全是 float。
        **N 由固件决定，本模块按帧长自动识别**，所以新旧记录都能读。历史/当前：
          4  = 四元数
          7  = 四元数 + 世界系速度
          10 = 四元数 + 速度 + 加速度原始 LSB
          18 = 四元数 + 速度 + 判静统计量 + 三个门状态 + 牵引驱动量 a_lin
               + 加速度零偏估计 + 打包标志位
          19 = 18 路 + DMA1 中断耗时历史最大值 us
          49 = 19 路 + 陀螺原始 LSB + 加速度原始 LSB + 温度 + dt + vel.age_s
               + up_ref + gyro_dps + gyro_bias_dps + accel_g + 证据粒度
               + 三个门/回退 + vel_soft + 地磁原始 LSB + IST8310 样本序号
          54 = 49 路 + **校准后的地磁单位方向 f[3]** + |y| + mag.ok
          60 = 54 路 + **磁力计的偏航维度输出**：Bw[3] + psi_deg + psi_true_deg
               + mag.trust（当前固件；见 V5F/User/src/SPI_rx.c 的通道说明）
        49 通道 = 200 字节 = 300 个十六进制字符（去空格）
        54 通道 = 220 字节 = 330 个十六进制字符（去空格）
        60 通道 = 244 字节 = 366 个十六进制字符（去空格）
        78 通道 = 316 字节 = 474 个十六进制字符（去空格）
        80 通道 = 324 字节 = 486 个十六进制字符（去空格）
        84 通道 = 340 字节 = 510 个十六进制字符（去空格）
        90 通道 = 364 字节 = 546 个十六进制字符（去空格）
          92 = 90 路 + **处理函数 2 的启动阶段旁路判据**：三轴交流能量 stat.e_ac
               + 旁路门 stat.ac_bypass（用交流量绕开"零偏自锁"，见 v5f_tune.h）
        92 通道 = 372 字节 = 558 个十六进制字符（去空格）

列名映射见下面的 CH 字典（0 基）。用它取值，不要在各脚本里硬写列号：
    a = jf_load.load_jf(fn);  t = a[:, jf_load.CH['gps_lat']]
低速率通道（GPS 1~10 Hz、气压计 ~180 Hz、磁力计 ~187 Hz）在上报里是重复的，
取新样本用 edges_of()，它同时返回丢包数。
"""
import binascii
import os
import numpy as np

# ---- 78 通道布局的列名映射（0 基）。逐条依据见 V5F/User/src/SPI_rx.c 的通道说明 ----
CH_92 = {
    'q': 0, 'v_nav': 4, 'level_dps': 7, 'is_static': 8, 'acc_trust': 9,
    'acc_traction': 10, 'a_lin': 11, 'accel_bias_g': 14, 'flags': 17,
    'dma1_irq_us': 18, 'gyro_lsb': 19, 'accel_lsb': 22, 'temp_imu': 25,
    'dt_us': 26, 'vel_age_s': 27, 'up_ref': 28, 'gyro_dps': 31,
    'gyro_bias_dps': 34, 'accel_g': 37, 'bias_evidence_gran': 40,
    'att_tilt': 41, 'rollback_granules': 42, 'acc_rollback_granules': 43,
    'vel_soft': 44, 'mag_lsb': 45, 'ist_cnt': 48,
    'mag_f': 49, 'mag_norm': 52, 'mag_ok': 53,
    'mag_Bw': 54, 'psi_mag': 57, 'psi_true': 58, 'mag_trust': 59,
    'baro_press': 60, 'baro_temp': 61, 'baro_cnt': 62,
    'gps_status': 63, 'gps_lat': 64, 'gps_lon': 65, 'gps_speed': 66,
    'gps_rmc_flags': 67, 'gps_rmc_cnt': 68,
    'gps_alt': 69, 'gps_fixq': 70, 'gps_sv': 71, 'gps_hdop': 72,
    'gps_gga_flags': 73, 'gps_gga_cnt': 74,
    'gps_pdop': 75, 'gps_vdop': 76, 'gps_gsa_cnt': 77,
    'gps_course': 78, 'gps_magvar': 79,
    # C/N0 统计（GSV）：把"信号弱"和"多径"分开
    'gps_sats_view': 84, 'gps_snr_avg': 85, 'gps_snr_min': 86,
    'gps_snr_n': 87, 'gps_gsv_cnt': 88,
    'baro_press_avg': 89,   # 软件 OSR 平均后的压力（V5F_BARO_AVG_W 个样本滑动均值）
    # 处理函数 2 的启动阶段旁路判据（见 v5f_tune.h 的 V5F_DET_AC_*）：
    #   e_ac 是三轴交流能量（W=128 帧滑窗方差之和，dps^2），**对直流零偏完全不可见**，
    #   所以主路 DC 判据被上电零偏残差顶住时，它仍然认得出"静止"。
    #   ac_bypass 只在 imu.bias_ok 锁存前参与（flags 的 bit10 是同一路门）。
    'stat_e_ac': 90, 'stat_ac_bypass': 91,
    # 经纬度的精确形式（追加）。64/65 是便利的单列度值（有 ~0.42 m 量化），
    # 精确重建请用 lat_deg()/lon_deg()，它们用 80~83 这两对 int+frac。
    'gps_lat_int': 80, 'gps_lat_frac': 81,
    'gps_lon_int': 82, 'gps_lon_frac': 83,
}


# ---- 报表瘦身后的新布局（78 列，见 SPI_rx.c 的通道说明）----
# 由 tools 侧脚本从**补丁前**的发射顺序重新解析生成，与固件同一套逻辑。
CH_78 = {
    'q': 0,
    'v_nav': 4,
    'level_dps': 7,
    'is_static': 8,
    'acc_traction': 9,
    'a_lin': 10,
    'accel_bias_g': 13,
    'flags': 16,
    'dma1_irq_us': 17,
    'gyro_lsb': 18,
    'accel_lsb': 21,
    'temp_imu': 24,
    'dt_us': 25,
    'gyro_dps': 26,
    'gyro_bias_dps': 29,
    'accel_g': 32,
    'bias_evidence_gran': 35,
    'att_tilt': 36,
    'vel_soft': 37,
    'mag_lsb': 38,
    'ist_cnt': 41,
    'mag_f': 42,
    'mag_norm': 45,
    'mag_Bw': 46,
    'psi_true': 49,
    'mag_trust': 50,
    'baro_press': 51,
    'baro_temp': 52,
    'baro_cnt': 53,
    'gps_speed': 54,
    'gps_rmc_flags': 55,
    'gps_rmc_cnt': 56,
    'gps_alt': 57,
    'gps_fixq': 58,
    'gps_sv': 59,
    'gps_hdop': 60,
    'gps_gga_flags': 61,
    'gps_gga_cnt': 62,
    'gps_vdop': 63,
    'gps_gsa_cnt': 64,
    'gps_course': 65,
    'gps_lat_int': 66,
    'gps_lat_frac': 67,
    'gps_lon_int': 68,
    'gps_lon_frac': 69,
    'gps_sats_view': 70,
    'gps_snr_avg': 71,
    'gps_snr_min': 72,
    'gps_snr_n': 73,
    'gps_gsv_cnt': 74,
    'baro_press_avg': 75,
    'stat_e_ac': 76,
    'stat_ac_bypass': 77,
}

# ---- 阶段 0（S0.1+S0.2）：末尾追加 2 列 -> 80 列 ----
#   79 gps_gsv_talkers 本轮 GSV 的 talker 数（轮次聚合是否正常）
#   80 fw_tag          构建指纹 (VER<<16)|(通道数<<8)|开关位
# ★ 实测更正：这两列是插在 gps_gsv_cnt(74) **之后**，不是追加在末尾，
#   所以它后面三列各后移 2 位。由 20260915_003518 的实测列内容确认：
#   col76 恒 86022 = fw_tag，col75 恒 0 = talkers（该记录无 GPS）。
CH_80 = dict(CH_78)
CH_80.update({k: (v + 2 if v > 74 else v)
              for k, v in list(CH_80.items())})
CH_80.update({'gps_gsv_talkers': 75, 'fw_tag': 76})

# 按帧长选表：90/92 = 历史两版（前 90 列含义相同），78 = 瘦身后
# ---- VER=10..19：112 列，**权威表**（列号由 tools/calib/count_cols.py 逐行核出）----
#   关键列: dt_us=25  fw_tag=76  下行三列=77/78/79  press_avg=80  e_ac/ac_bypass=81/82
#           EKF 块 = 83..111
CH_112 = {
    'q': 0, 'v_nav': 4, 'level_dps': 7, 'is_static': 8, 'acc_traction': 9,
    'a_lin': 10, 'accel_bias_g': 13, 'flags': 16, 'dma1_irq_us': 17,
    'gyro_lsb': 18, 'accel_lsb': 21, 'temp_imu': 24, 'dt_us': 25,
    'gyro_dps': 26, 'gyro_bias_dps': 29, 'accel_g': 32, 'bias_evidence_gran': 35,
    'att_tilt': 36, 'vel_soft': 37, 'mag_lsb': 38, 'ist_cnt': 41,
    'mag_f': 42, 'mag_norm': 45, 'mag_Bw': 46, 'psi_true': 49, 'mag_trust': 50,
    'baro_press': 51, 'baro_temp': 52, 'baro_cnt': 53,
    'gps_speed': 54, 'gps_rmc_flags': 55, 'gps_rmc_cnt': 56,
    'gps_alt': 57, 'gps_fixq': 58, 'gps_sv': 59, 'gps_hdop': 60,
    'gps_gga_flags': 61, 'gps_gga_cnt': 62, 'gps_vdop': 63, 'gps_gsa_cnt': 64,
    'gps_course': 65,
    'gps_lat_int': 66, 'gps_lat_frac': 67, 'gps_lon_int': 68, 'gps_lon_frac': 69,
    'gps_sats_view': 70, 'gps_snr_avg': 71, 'gps_snr_min': 72, 'gps_snr_n': 73,
    'gps_gsv_cnt': 74, 'gps_gsv_talkers': 75, 'fw_tag': 76,
    'cmd_echo': 77, 'cmd_cnt': 78, 'cmd_last': 79,
    'baro_press_avg': 80, 'stat_e_ac': 81, 'stat_ac_bypass': 82,
    'ekf_p': 83, 'ekf_v': 86, 'ekf_q': 89, 'ekf_a_nav': 93,
    'ekf_ba': 96, 'ekf_bg': 99, 'ekf_b_baro': 102, 'ekf_gate_bits': 103,
    'ekf_sigma_yaw_deg': 104, 'ekf_sigma_pos_h': 105, 'ekf_sigma_vel_h': 106,
    'ekf_nis': 107,
}

# EKF gate_bits（列 103）的位名
EKF_GB = [
    (0x0001, 'gps_pos'), (0x0002, 'gps_alt'), (0x0004, 'baro'),
    (0x0008, 'gps_vel'), (0x0010, 'zupt'), (0x0020, 'tilt'),
    (0x0040, 'mag_yaw'), (0x0080, 'aligned'), (0x0100, 'step'),
    (0x0200, 'chi2_rej'), (0x0400, 'origin'), (0x0800, 'acc_sat'),
]

# ---- VER=20：113 列（在 112 列上插 ekf_sigma_tilt_deg=107，107 起整体后移一位）----
CH_113 = dict(CH_112)
CH_113.update({k: (v + 1 if v >= 107 else v) for k, v in list(CH_113.items())})
CH_113.update({'ekf_sigma_tilt_deg': 107})

# ---- VER=25：115 列（末尾追加 ekf_pzz / ekf_pbb 两个诊断列）----
CH_115 = dict(CH_113)
CH_115.update({'ekf_pzz': 113, 'ekf_pbb': 114})

# ---- VER=26：117 列（再追加 ekf_p_pzz / ekf_p_pbb 两个 P0 自检列）----
CH_117 = dict(CH_115)
CH_117.update({'ekf_p_pzz': 115, 'ekf_p_pbb': 116})

# ---- VER=43：122 列（末尾追加 5 个磁偏航专项诊断列）----
CH_122 = dict(CH_117)
CH_122.update({'ekf_mag_gate': 117, 'ekf_mag_bh': 118, 'ekf_mag_r_deg': 119,
               'ekf_mag_used': 120, 'ekf_p_yy': 121})

CH_127 = dict(CH_122)
CH_127.update({'ekf_prop_ok': 122, 'ekf_f_ok': 123, 'ekf_prop_row': 124,
               'ekf_stage': 125, 'ekf_mag_rej': 126, 'ekf_mag_fhb': 127})

CH_133 = dict(CH_127)
CH_133.update({'ekf_mag_rx': 128, 'ekf_mag_ry': 129, 'ekf_mag_dqx': 130,
               'ekf_mag_dqy': 131, 'ekf_mag_dqz': 132})

CH_139 = dict(CH_133)
CH_139.update({'ekf_tilt_dqx': 133, 'ekf_tilt_dqy': 134, 'ekf_tilt_dqz': 135,
               'ekf_tilt_prx': 136, 'ekf_tilt_pry': 137, 'ekf_tilt_prz': 138})

CH_BY_NCH = {90: CH_92, 92: CH_92, 78: CH_78, 80: CH_80, 112: CH_112, 113: CH_113,
             115: CH_115, 117: CH_117, 122: CH_122, 127: CH_127, 128: CH_127, 133: CH_133, 139: CH_139}

# 默认最新版；读历史记录请用 ch_for(a.shape[1])
CH = CH_127


def ch_for(nch):
    """按通道数取对应的列名表。

    ★ 历史各版都严格遵守"新量只追加、旧列号不动"，所以**凡是 <= 92 的历史版都是
    CH_92 的前缀**（19/31/45/48/49/54/60/84/90/92 ...），直接用 CH_92 即可 ——
    多出来的名字取不到对应列，不影响。只有瘦身之后的 78/80 要另算。
    """
    if nch == 78:
        return CH_78
    if nch == 80:
        return CH_80
    if nch == 112:
        return CH_112
    if nch == 113:
        return CH_113
    if nch == 115:
        return CH_115
    if nch == 117:
        return CH_117
    if nch == 122:
        return CH_122
    if nch == 127:
        return CH_127
    if nch == 128:
        return CH_127
    if nch == 133:
        return CH_133
    if nch == 139:
        return CH_139
    if 0 < nch <= 92:
        return CH_92
    raise KeyError("未知帧长 %d 通道；已知 78/80/112/113 与所有 <=92 的历史版" % nch)


def lat_deg(a):
    """用经纬度的 int+frac 两路无损重建纬度（度）。负纬度也成立（C 整除向零截断）。
    ★ 按 a 的通道数取自适应的列表，所以历史记录（<=92 列）与新报表都能用。"""
    c = ch_for(a.shape[1])
    return a[:, c['gps_lat_int']].astype(np.float64) + \
           a[:, c['gps_lat_frac']].astype(np.float64) * 1e-7


def lon_deg(a):
    """用经纬度的 int+frac 两路无损重建经度（度）。"""
    c = ch_for(a.shape[1])
    return a[:, c['gps_lon_int']].astype(np.float64) + \
           a[:, c['gps_lon_frac']].astype(np.float64) * 1e-7


def edges_of(cnt_col):
    """给定某一路"样本序号"列的整列值，返回 (新样本出现的帧下标, 丢样本总数)。
    序号跳 >1 即上游丢了样本，这里把它累加出来。"""
    c = np.asarray(cnt_col)
    d = np.diff(c)
    idx = np.where(d > 0)[0] + 1
    return idx, int(np.sum(d[d > 0] - 1))

PAT = b'[RX] '
NBYTES = 80          # 默认值：当前固件 19 float + 4 字节帧尾（实际按首行自动识别）
HEXLEN = NBYTES * 3 - 1   # "XX ... XX" 共 239 个字符
HEXCHR = NBYTES * 2       # 去空格后 160 个十六进制字符
NCH = 19


def load_jf(fn, cache=True, verbose=False):
    """返回 (N, NCH_declared) float32。列含义随固件版本而变：
       7 通道 = q[4] + 加速度原始 LSB[3]
      10 通道 = q[4] + 加速度原始 LSB[3] + 陀螺原始 LSB[3]
    帧长**自动识别**（从第一条数据行的十六进制字段长度推出），所以新旧记录都能读。"""
    npy = fn + '.npy'
    # 缓存比源文件新就用缓存。★ 也允许源 .txt 已经被清理掉 —— 日志目录会轮转，
    #   而 .npy 是分析用的二进制快照，应当能独立存活（实测 .txt 被清空后只剩 .npy）。
    if cache and os.path.exists(npy) and (
            not os.path.exists(fn) or os.path.getmtime(npy) > os.path.getmtime(fn)):
        a = np.load(npy)
        if verbose:
            print("  [jf_load] 命中缓存 %s  %s" % (os.path.basename(npy), a.shape))
        return a

    # 先探数据行，定出本文件的帧长。
    # ★ 取头若干行 token 数的**众数**，不能只取第一条：抓包起始常常是残缺帧
    #   （实测 20260914_051140 首行 16 token、后续全 80），只取首行会把整个文件
    #   按错误帧长错位解析，而且是"能跑但全错"的那种。
    # 按 token 数数而不是空格数：行尾常多一个空格，按空格会多算一个字节。
    nbytes, hexlen, hexchr = NBYTES, HEXLEN, HEXCHR
    from collections import Counter
    cnt = Counter()
    with open(fn, 'rb') as fp:
        for _ in range(500):
            ln = fp.readline()
            if not ln:
                break
            i = ln.find(PAT)
            if i < 0:
                continue
            tail = ln[i + 5:].rstrip(b'\r\n')
            n = len(tail.split())
            if n > 0:
                cnt[n] += 1
    if cnt:
        nbytes = cnt.most_common(1)[0][0]
        hexlen = nbytes * 3 - 1
        hexchr = nbytes * 2
    nch = nbytes // 4 - 1                       # 扣掉 4 字节帧尾
    if nch <= 0:
        raise ValueError(
            "帧长识别失败：token 众数 %d -> %d 通道。这份日志多半不是十六进制文本"
            "（例如被上位机按 UTF-8 解码、非 UTF-8 字节变成 U+FFFD）。"
            "先跑 tools/calib/parse_check.py 确认。" % (nbytes, nch))

    out = bytearray()
    rem = b''
    with open(fn, 'rb') as fp:
        while True:
            blk = fp.read(1 << 22)          # 4 MB 一块
            if not blk:
                break
            parts = (rem + blk).split(b'\n')
            rem = parts.pop()               # 末尾可能是半行，留到下一块
            for ln in parts:
                i = ln.find(PAT)
                if i < 0:
                    continue
                h = ln[i + 5:i + 5 + hexlen].replace(b' ', b'')
                if len(h) == hexchr:
                    out += binascii.unhexlify(h)
    # 帧 = nch 个数据 float + 1 个帧尾 float（0x7F800000 = +inf），丢掉帧尾
    a = np.frombuffer(bytes(out), '<f4').reshape(-1, nch + 1)[:, :nch].copy()

    if a.shape[0] == 0:
        raise ValueError(
            "解析出 0 帧（帧长 %d B / %d 通道）—— 日志格式不对，不写缓存。"
            % (nbytes, nch))
    if cache:
        np.save(npy, a)
    if verbose:
        print("  [jf_load] %s -> %s（帧长 %d B，%d 通道），缓存写到 %s"
              % (os.path.basename(fn), a.shape, nbytes, nch, os.path.basename(npy)))
    return a


if __name__ == '__main__':
    import sys
    for fn in sys.argv[1:]:
        a = load_jf(fn, verbose=True)
        print("  首帧 q=%s acc=%s" % (a[0, :4], a[0, 4:7]))

# ---- ★VER=78 起：148 列（新增 4 个"罗盘量"）----
# 128 mag_rx / 129 mag_ry / 130 mag_vx / 131 mag_vy / 132 mag_v0x / 133 mag_v0y
# 134 mag_yawpre / 135..137 mag_dqx,dqy,dqz / 138..140 tilt_dq{x,y,z}
# 141..143 tilt_pr{x,y,z} / 144..147 mag_cmp_{thm,thp,amn,mhn}
CH_148 = dict(CH_127)
CH_148.update({'ekf_mag_rx': 128, 'ekf_mag_ry': 129,
               'ekf_mag_vx': 130, 'ekf_mag_vy': 131,
               'ekf_mag_v0x': 132, 'ekf_mag_v0y': 133,
               'ekf_mag_yawpre': 134,
               'ekf_mag_dqx': 135, 'ekf_mag_dqy': 136, 'ekf_mag_dqz': 137,
               'ekf_tilt_dqx': 138, 'ekf_tilt_dqy': 139, 'ekf_tilt_dqz': 140,
               'ekf_tilt_prx': 141, 'ekf_tilt_pry': 142, 'ekf_tilt_prz': 143,
               'mag_cmp_thm': 144,   # 罗盘实测航向(度)
               'mag_cmp_thp': 145,   # 罗盘预测航向(度)
               'mag_cmp_amn': 146,   # 姿态"上"与加计夹角(度)
               'mag_cmp_mhn': 147})  # 重力法平面内磁场模长
CH_BY_NCH[148] = CH_148
CH = CH_148
