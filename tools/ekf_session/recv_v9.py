# -*- coding: utf-8 -*-
"""COM4 采集 v9：自描述帧（通道数 = 帧长/4）+ 下行命令注入 + 回显闭环验证。

固件帧:  A5 5A | len(小端 u16 = 载荷字节数) | 载荷 | 5A A5
         VER=9, 83 通道 -> 2+2+332+2 = 338 B
         列 0..79   原有
         列 80 = cmd_echo('T' 写进来的 f32，原样回显)
         列 81 = cmd_cnt (收到的合法命令数)
         列 82 = cmd_last(最近一条 opcode 的 ASCII)

下行帧(主机->设备): A5 5A | len(小端 u16 = 命令字节数) | 命令 | 5A A5
        命令: 'T' <f32 LE>   测试回显(本脚本用)
              'A'            强制重新对齐
        USB 端 bulk OUT(EP2) -> 中断入环 -> 主循环 hid_cmd_poll() 解析

用法:
    python recv_v9.py                  # 只采集
    python recv_v9.py --test           # 采集 + 在 t=2/4/6/8s 发 4 条 T 命令
"""
import argparse
import os
import struct
import time

import numpy as np
import serial

def _expected_fw():
    """从源码算期望指纹 —— 唯一真值来源，脚本里不许再写死。"""
    import re
    root = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'h415-imu42605-')
    tune = open(os.path.join(root, 'V5F', 'User', 'inc', 'v5f_tune.h'), 'rb').read().decode('gbk')
    src = open(os.path.join(root, 'V5F', 'User', 'src', 'SPI_rx.c'), 'rb').read().decode('gbk')

    def g(pat, s):
        m = re.search(pat, s)
        if not m:
            raise SystemExit('源码里找不到 ' + pat)
        return int(m.group(1))
    ver = g(r'#define V5F_FW_VER\s+(\d+)u', tune)
    nch = g(r'#define JF_CH_NUM\s+(\d+)u', src)
    ekf = g(r'#define V5F_EKF_EN\s+(\d+)u', tune)
    mag = g(r'#define V5F_MAG_CAL_EN\s+(\d+)u', tune)
    ac = g(r'#define V5F_DET_AC_EN\s+(\d+)u', tune)
    return float((ver << 16) | (nch << 8) | (ekf << 0) | (mag << 1) | (ac << 2))


FW = _expected_fw()     # 由源码算出（VER/nch/开关），不再手写, 112ch, EKF=1, MAGCAL=1, AC=1
IDX_FW = 76            # 0 基列号，由 tools/calib/count_cols.py 逐行数 ch[c++] 得到
IDX_DT = 25
IDX_ECHO, IDX_CNT, IDX_LAST = 77, 78, 79
HDR = b'\xa5\x5a'
TL = b'\x5a\xa5'
OUT = r'R:\raw_v9.bin'


def down_frame(cmd):
    return HDR + struct.pack('<H', len(cmd)) + cmd + TL


def cmd_T(v):
    return down_frame(b'T' + struct.pack('<f', float(v)))


# 动态性能测试协议：(起, 止, 提示)。每个时段开始的瞬间发一条 T 命令，
# 于是"什么时候让我做什么"在数据里有独立的时间戳，不靠墙钟对表。
SEGS = [
    (0,  12, '静止：板子放平别碰，等对齐 + 零偏收敛'),
    (12, 26, '只绕竖直轴转：慢慢转到 +90 度停 2 秒 -> 回 0 -> 转到 -90 度停 2 秒 -> 回 0，来回两遍'),
    (26, 40, '只做倾斜：向前/后/左/右各缓慢倾到约 30 度再回平，不要平移'),
    (40, 50, '静止'),
    (50, 66, '手持走动：正常速度走 3~4 米再走回来，走两趟（GPS 无信号也没关系）'),
    (66, 78, '剧烈动作：快速左右晃动 + 轻敲桌面，故意做猛一点'),
    (78, 95, '静止：放回桌面别碰，看收敛与漂移'),
]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--port', default='COM4')
    ap.add_argument('--sec', type=float, default=300.0)
    ap.add_argument('--out', default=OUT)
    ap.add_argument('--test', action='store_true', help='按时序注入 T 命令验证下行')
    ap.add_argument('--seg', action='store_true', help='按动态性能协议提示动作（95 s）')
    a = ap.parse_args()

    ser = serial.Serial(a.port, 115200, timeout=0.02, write_timeout=0.5)
    try:
        ser.set_buffer_size(rx_size=1 << 23, tx_size=8192)
    except Exception as e:
        print('set_buffer_size: %s' % e)
    print('口 %s   帧头 A5 5A  期望 fw_tag %.0f   落地 %s' % (a.port, FW, a.out))

    # 时序：t 秒 -> 值
    sched = {2.0: 12345.0, 4.0: -678.25, 6.0: 3.5, 8.0: 1.0e6} if a.test else {}
    if a.seg:
        for k, (t0s, _t1, _m) in enumerate(SEGS):
            sched[float(t0s)] = 1000.0 + float(k)   # 每段一条标记命令，值 = 段号
        # 只在用户没显式给 --sec 时才默认拉长到协议总长；
        # 之前无条件改成 97 s，导致 --sec 25 根本停不下来。
        if a.sec == 300.0:
            a.sec = SEGS[-1][1] + 2.0
    sent = {}

    buf = bytearray()
    out = open(a.out, 'wb')
    t0 = time.time()
    n_ok = n_bad = 0
    n_tagbad = 0
    tag_seen = None
    lost = 0
    dt_sum = 0.0
    nch = None
    echo_seen = []
    last_rep = 0.0

    def sync():
        """buf 停在帧起点则返回载荷长度, 否则 None; 失败内部自行重同步。"""
        nonlocal lost
        while True:
            i = buf.find(HDR)
            if i < 0:
                if len(buf) > 8:
                    lost += len(buf) - 1
                    del buf[:len(buf) - 1]
                return None
            if i > 0:
                lost += i
                del buf[:i]
            if len(buf) < 4:
                return None
            ln = buf[2] | (buf[3] << 8)
            if ln == 0 or ln % 4 or ln < 320 or ln > 4096:
                lost += 1
                del buf[:1]
                continue
            if len(buf) < ln + 6:
                return None
            if buf[ln + 4:ln + 6] != TL:
                lost += 1
                del buf[:1]
                continue
            return ln

    try:
        while time.time() - t0 < a.sec:
            el = time.time() - t0
            for ts, v in list(sched.items()):
                if el >= ts and ts not in sent:
                    f = cmd_T(v)
                    werr = None
                    try:
                        ser.write(f)
                        ser.flush()
                    except Exception as ex:
                        werr = ex
                    sent[ts] = v
                    if v >= 1000.0:
                        k = int(v) - 1000
                        print()
                        print('  ======== t=%5.1f  【第 %d 段】%s' % (el, k, SEGS[k][2]))
                        print('  ======== (这一段到 t=%d s 为止)' % SEGS[k][1])
                    else:
                        print('  t=%5.1f  -> 下行 T %.4f  %d B  %s'
                              % (el, v, len(f), '写入失败:' + str(werr) if werr else 'OK'))
            nw = ser.in_waiting
            if nw == 0:
                time.sleep(0.0005)
                continue
            buf += ser.read(nw)
            while True:
                ln = sync()
                if ln is None:
                    break
                if nch is None:
                    nch = ln // 4
                    print('  自描述: 载荷 %d B -> %d 通道, 帧长 %d B' % (ln, nch, ln + 6))
                    if nch > IDX_FW:
                        _m0 = np.frombuffer(bytes(buf[4:4 + ln]), np.uint8).view('<f4')
                        print('  首帧实测: fw_tag=%.0f (期望 %.0f)  dt_us=%.2f  |q|=%.6f'
                              % (float(_m0[IDX_FW]), FW, float(_m0[IDX_DT]),
                                 float(np.linalg.norm(_m0[0:4].astype(np.float64)))))
                        del _m0
                m = np.frombuffer(bytes(buf[4:4 + ln]), np.uint8).view('<f4').copy()
                del buf[:ln + 6]
                q = float(np.linalg.norm(m[0:4].astype(np.float64)))
                tag = float(m[IDX_FW])
                if tag_seen is None:
                    tag_seen = tag
                    if tag != FW:
                        print('  ★ 指纹不符: 实测 %.0f  源码算出 %.0f —— **照收不误**，'
                              '只是提醒你分析的可能是别的固件' % (tag, FW))
                # 指纹不符**不**再丢帧：版本错配不该让整段数据消失（踩过：
                # 期望值写死没同步 -> 25 s 采集 200668 坏帧 / 0 好帧 / 0 字节）
                ok = (abs(q - 1.0) < 1e-3
                      and 100.0 < float(m[IDX_DT]) < 200.0)
                if ok and tag != FW:
                    n_tagbad += 1
                if ok:
                    out.write(m.tobytes())
                    dt_sum += float(m[IDX_DT]) * 1e-6
                    n_ok += 1
                    if nch and nch >= 112 and float(m[IDX_LAST]) == 84.0:   # 'T'
                        e = float(m[IDX_ECHO])
                        if not echo_seen or echo_seen[-1][1] != e:
                            echo_seen.append((float(m[IDX_CNT]), e))
                            print('  t=%5.1f  <- 回显 cmd_echo=%.3f  cmd_cnt=%.0f'
                                  % (el, e, float(m[IDX_CNT])))
                else:
                    n_bad += 1
            if el - last_rep >= 30:
                last_rep = el
                print('  t=%5.1f  好 %8d  坏 %6d  丢字节 %d  (%.1f MB)'
                      % (el, n_ok, n_bad, lost, out.tell() / 1e6))
    finally:
        out.close()
        ser.close()

    T = time.time() - t0
    tot = n_ok + n_bad
    print()
    print('=== VER=9 结果 ===')
    print('  时长 %.1f s  好帧 %d (%.2f Hz)  坏帧 %d (%.4f%%)  重同步丢弃 %d B'
          % (T, n_ok, n_ok / T, n_bad, 100.0 * n_bad / max(tot, 1), lost))
    print('  时间轴: dt_us 之和 %.3f s / 墙钟 %.1f s -> 缺口 %.2f s'
          % (dt_sum, T, T - dt_sum))
    print('  落地 %.1f MB -> %s' % (os.path.getsize(a.out) / 1e6, a.out))
    print('  指纹: 实测 %s  期望 %.0f  %s'
          % ('%.0f' % tag_seen if tag_seen is not None else '未收到帧', FW,
             'OK' if tag_seen == FW else ('★不符的帧 %d 个' % n_tagbad)))
    if a.test:
        print()
        print('  下行验证: 发出 %d 条, 回显 %d 条' % (len(sent), len(echo_seen)))
        for c, e in echo_seen:
            print('     cmd_cnt=%-6.0f cmd_echo=%.4f' % (c, e))
        if len(echo_seen) >= len(sent) and sent:
            print('  => 下行 + 上行闭环 OK')
        else:
            print('  => 未闭环: 查 EP2 bulk OUT / hid_cmd_poll / 主循环是否在跑')


if __name__ == '__main__':
    main()
