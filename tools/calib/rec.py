# -*- coding: utf-8 -*-
"""串口录制：A5 5A 帧的 JustFloat 调试流 -> 文本日志（与历史 serial_runtime_*_export.txt 同格式）。

为什么还要一个：仓库里的老工具对不上现在这版固件 ——
  * tools/calib/cap.py      : 认 **帧尾 00 00 80 7F**（验收模式 20 B / 老的 7 路 32 B），调试模式吃不下；
  * tools/ekf_session/recv_v9.py : A5 5A 但硬编码 VER=9 时代（写二进制到 R:\\raw_v9.bin、83ch 假设）。

本脚本产出与历史录像**逐字节同格式**的文本（mag360_cal.py / jf_load.py / 全部旧分析脚本直接可用）：
    === Serial logging started at YYYY-MM-DD HH:MM:SS.mmm ===
    [HH:MM:SS.mmm] [RX] A5 5A ... 5A A5
并逐帧校验：帧头/长度/帧尾、|att.q|≈1、dt_us 合理、fw_tag（**从源码算出**，不写死）、
逐帧校验和；运行中每 20 s 报一次 好帧/坏帧/丢字节/速率，结束时给判定。

用法：
  python tools/calib/rec.py --list                     # 列串口
  python tools/calib/rec.py -p COM5 -t 90              # 录 90 s
  python tools/calib/rec.py -p COM5 --seg              # 按地磁标定动作协议分段提示（每段回车继续）
  python tools/calib/rec.py --selftest                 # 无需硬件：合成流 -> 解析 -> 回读 全链自检
常用参数：
  -p/--port  串口（USB CDC 时波特率无意义）
  -b/--baud  默认 921600；-t/--sec 录制秒数（默认 60）；-o/--out 输出文本
  --bin PATH 同时落一份原始 payload 二进制（648 B/帧，快，供离线复核）
  --seg      分段动作提示（写进日志的 # 注释行，分析时可切段）
  --no-text  只落二进制
"""
import argparse
import datetime as dt
import os
import struct
import sys
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import cols_162 as C   # noqa: E402

HDR, TL = b'\xa5\x5a', b'\x5a\xa5'


# ------------------------- 从源码读期望指纹 -------------------------

def expected_fw():
    root = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
    tune = open(os.path.join(root, 'V5F', 'User', 'inc', 'v5f_tune.h'), 'rb').read().decode('gbk')
    src = open(os.path.join(root, 'V5F', 'User', 'src', 'SPI_rx.c'), 'rb').read().decode('gbk')

    def g(pat, s):
        import re
        m = re.search(pat, s)
        return int(m.group(1)) if m else None
    ver = g(r'#define V5F_FW_VER\s+(\d+)u', tune)
    nch = g(r'#define JF_CH_NUM\s+(\d+)u', src)
    ekf = g(r'#define V5F_EKF_EN\s+(\d+)u', tune)
    mag = g(r'#define V5F_MAG_CAL_EN\s+(\d+)u', tune)
    ac = g(r'#define V5F_DET_AC_EN\s+(\d+)u', tune)
    return ver, nch, float((ver << 16) | (nch << 8) | (ekf or 0) | ((mag or 0) << 1) | ((ac or 0) << 2))


# ------------------------- 标定动作协议 -------------------------

SEGS = [
    (12, '静止锚点：水平静置，别碰 (对齐/零偏收敛)'),
    (10, '静止锚点：前倾 ~30 度停住'),
    (10, '静止锚点：后倾 ~30 度停住'),
    (10, '静止锚点：左倾 ~30 度停住'),
    (10, '静止锚点：右倾 ~30 度停住'),
    (25, '绕 x 轴慢转整圈 (25~50 dps，正反各一次更好)'),
    (25, '绕 y 轴慢转整圈'),
    (25, '绕 z 轴(竖直)慢转整圈'),
    (30, '保持 ~45 度倾斜，绕竖直慢转 360 度'),
    (25, '完全翻转(≈180 度)后绕 x 轴慢转整圈'),
    (25, '完全翻转后绕 y 轴慢转整圈'),
    (15, '回到水平静止，别碰'),
]


# ------------------------- 解析与校验 -------------------------

class Parser:
    """增量解析 A5 5A 帧，逐帧做物理/指纹/校验和检查。"""

    def __init__(self, want_nch, want_tag):
        self.buf = bytearray()
        self.nch = None
        self.want_nch = want_nch
        self.want_tag = want_tag
        self.n_ok = self.n_bad = self.lost = 0
        self.chk_ok = self.chk_n = 0
        self.tag_seen = None
        self.t_pref = 1.0
        self.t_alt = 0.0
        self.qbad = 0
        self.dt_sum = 0.0

    def push(self, data):
        self.buf += data
        out = []
        while True:
            i = self.buf.find(HDR)
            if i < 0:
                if len(self.buf) > 8:
                    self.lost += len(self.buf) - 1
                    del self.buf[:len(self.buf) - 1]
                break
            if i > 0:
                self.lost += i
                del self.buf[:i]
            if len(self.buf) < 4:
                break
            ln = self.buf[2] | (self.buf[3] << 8)
            if ln == 0 or ln % 4 or ln < 4 or ln > 8192:
                self.lost += 1
                del self.buf[:1]
                continue
            if len(self.buf) < ln + 6:
                break
            if self.buf[ln + 4:ln + 6] != TL:
                self.lost += 1
                del self.buf[:1]
                continue
            pay = bytes(self.buf[4:4 + ln])
            del self.buf[:ln + 6]
            f = np.frombuffer(pay, dtype='<f4')
            nch = ln // 4
            ok = True
            if self.nch is None:
                self.nch = nch
            if nch == self.want_nch and nch >= 46:
                c = C.CH_162
                q = float(np.linalg.norm(f[c['att_q0']:c['att_q0'] + 4].astype(np.float64)))
                dtu = float(f[c['dt_us']])
                tag = float(f[c['fw_tag']])
                if self.tag_seen is None:
                    self.tag_seen = tag
                # 校验和：前 (nch-1)*4 字节 XOR，落在最后一列（值 0..255 的 float）
                xk = 0
                for b in pay[:(nch - 1) * 4]:
                    xk ^= b
                self.chk_n += 1
                if abs(float(f[nch - 1]) - float(xk & 0xFF)) < 0.5:
                    self.chk_ok += 1
                # 时间轴：dt 列可能是 1 帧差(固件全速) 或 N 帧差(抽帧后)；自动识别主档
                if 50.0 < dtu < 200.0:
                    self.t_pref = max(self.t_pref, 1.0)
                if abs(q - 1.0) > 2e-3:
                    self.qbad += 1
                    ok = False
                if not (1.0 < dtu < 20000.0):
                    ok = False
                self.dt_sum += dtu * 1e-6
            if ok:
                self.n_ok += 1
                # 文本日志要的是**整帧**（含 A5 5A 帧头/帧尾），二进制侧车要的是纯 payload
                out.append(HDR + bytes([ln & 0xFF, (ln >> 8) & 0xFF]) + pay + TL)
            else:
                self.n_bad += 1
        return out


# ------------------------- 合成流自检 -------------------------

def selftest():
    C.selfcheck()
    ver, nch, tag = expected_fw()
    rng = np.random.default_rng(7)
    frames = []
    q = np.array([1.0, 0.0, 0.0, 0.0])
    for k in range(400):
        ax = rng.normal(size=3)
        ax /= np.linalg.norm(ax)
        ang = np.radians(20.0) * 0.002966
        dq = np.concatenate(([np.cos(ang / 2)], ax * np.sin(ang / 2)))
        q = np.array([dq[0] * q[0] - dq[1] * q[1] - dq[2] * q[2] - dq[3] * q[3],
                      dq[0] * q[1] + dq[1] * q[0] + dq[2] * q[3] - dq[3] * q[2],
                      dq[0] * q[2] - dq[1] * q[3] + dq[2] * q[0] + dq[3] * q[1],
                      dq[0] * q[3] + dq[1] * q[2] - dq[2] * q[1] + dq[3] * q[0]])
        q /= np.linalg.norm(q)
        fr = np.zeros(C.NCH, dtype=np.float32)
        fr[C.CH_162['att_q0']:C.CH_162['att_q0'] + 4] = q
        fr[C.CH_162['flags']] = 0x04
        fr[C.CH_162['dt_us']] = 2966.0
        fr[C.CH_162['ist_cnt']] = k // 2
        fr[C.CH_162['mag_lsb0']:C.CH_162['mag_lsb0'] + 3] = [100, -40, 60]
        fr[C.CH_162['fw_tag']] = tag
        xk = 0
        for b in fr[:C.NCH - 1].astype('<f4').tobytes():
            xk ^= b
        fr[C.NCH - 1] = float(xk & 0xFF)
        frames.append(HDR + struct.pack('<H', C.NCH * 4) + fr.tobytes() + TL)
    stream = b''.join(frames) + b'\x11\x22'          # 尾部塞 2 B 垃圾，考验重同步
    p = Parser(nch, tag)
    pay = p.push(stream)
    print('解析 %d/%d 帧, 丢字节 %d, 校验和 %d/%d, tag 实测 %.0f 期望 %.0f'
          % (len(pay), len(frames), p.lost, p.chk_ok, p.chk_n, p.tag_seen, tag))
    assert len(pay) == len(frames), '帧数不符'
    assert p.chk_ok == p.chk_n == len(frames), '校验和失败'
    assert abs(p.tag_seen - tag) < 0.5, '指纹不符'

    # 写一份文本日志 -> 用 cols_162.load_frames 回读，列值必须逐列一致
    tmp = os.path.join(os.environ.get('TEMP', '.'), 'rec_selftest.txt')
    arr = np.stack([np.frombuffer(x[4:-2], dtype='<f4') for x in pay])
    with open(tmp, 'w') as fh:
        fh.write('=== Serial logging started at 2026-01-01 00:00:00.000 ===\n')
        for x in pay:
            fh.write('[00:00:00.000] [RX] ' + ' '.join('%02X' % b for b in x) + ' \n')
    back, info = C.load_frames(tmp)
    assert back.shape == arr.shape, '回读形状不符 %s vs %s' % (back.shape, arr.shape)
    assert np.array_equal(back, arr), '回读数值不一致'
    assert info['format'] == 'framed'
    print('文本回读一致: %s   %s' % (back.shape, info['format']))
    print('REC SELFTEST OK (合成流 + 文本回读)')


# ------------------------- 主流程 -------------------------

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('-p', '--port', default='COM4')
    ap.add_argument('-b', '--baud', type=int, default=921600)
    ap.add_argument('-t', '--sec', type=float, default=60.0)
    ap.add_argument('-o', '--out', default=None)
    ap.add_argument('--bin', default=None)
    ap.add_argument('--seg', action='store_true', help='按地磁标定协议分段提示')
    ap.add_argument('--no-text', action='store_true')
    ap.add_argument('--list', action='store_true')
    ap.add_argument('--selftest', action='store_true')
    a = ap.parse_args()

    if a.selftest:
        selftest()
        return 0

    import serial
    import serial.tools.list_ports as lp
    if a.list:
        for p in lp.comports():
            print('%-8s | %-28s | %s' % (p.device, p.description, p.hwid))
        print('\n提示：IMU 是 USB CDC 虚拟串口，插上板子后这里会多出一个（WCH-Link 那个不是）')
        return 0

    ver, nch, tag = expected_fw()
    C.selfcheck(verbose=False)
    print('固件期望: VER=%s  通道=%s  fw_tag=%.0f  (从 v5f_tune.h / SPI_rx.c 现算)' % (ver, nch, tag))
    if nch != C.NCH:
        print('!! 源码 JF_CH_NUM=%s 与 cols_162 的 %d 不一致，先更新列名表' % (nch, C.NCH))

    stamp = dt.datetime.now().strftime('%Y%m%d_%H%M%S_%f')[:-3]
    out = a.out or ('serial_runtime_%s_export.txt' % stamp)
    binp = a.bin
    fh = open(out, 'w', encoding='ascii', newline='\n') if not a.no_text else None
    fb = open(binp, 'wb') if binp else None
    if fh:
        fh.write('=== Serial logging started at %s ===\n'
                 % dt.datetime.now().strftime('%Y-%m-%d %H:%M:%S.') + stamp[-3:])
        fh.flush()

    ser = serial.Serial(a.port, a.baud, timeout=0.02, write_timeout=0.5)
    try:
        ser.set_buffer_size(rx_size=1 << 23, tx_size=8192)
    except Exception as e:
        print('set_buffer_size: %s' % e)
    print('口 %s @ %d  文本 %s  二进制 %s' % (a.port, a.baud, out if fh else '(off)', binp or '(off)'))

    p = Parser(nch, tag)
    t0 = time.time()
    last_rep = 0.0
    line_buf = []
    seg_i = 0
    seg_t = 0.0
    prompt_t = None
    frame_hz = 0.0
    try:
        while time.time() - t0 < a.sec:
            el = time.time() - t0
            if a.seg and seg_i < len(SEGS):
                if prompt_t is None:
                    dur, txt = SEGS[seg_i]
                    msg = '# ==== [%5.1f s] 第 %d/%d 段 (%d s): %s ====' % (el, seg_i + 1, len(SEGS), dur, txt)
                    print('\n' + msg)
                    if fh:
                        fh.write(msg + '\n')
                        fh.flush()
                    prompt_t = 0.0
                    input('      准备好后按回车开始这一段...')
                    seg_t = time.time() - t0
                elif el - seg_t >= SEGS[seg_i][0]:
                    seg_i += 1
                    prompt_t = None
            nw = ser.in_waiting
            if nw:
                pay = p.push(ser.read(nw))
            else:
                time.sleep(0.0005)
                continue
            if pay:
                if fb:
                    fb.write(b''.join(x[4:-2] for x in pay))
                if fh:
                    ts = dt.datetime.now().strftime('%H:%M:%S.%f')[:-3]
                    for x in pay:
                        line_buf.append('[%s] [RX] %s \n' % (ts, ' '.join('%02X' % b for b in x)))
                    if len(line_buf) >= 256:
                        fh.write(''.join(line_buf))
                        line_buf.clear()
            if el - last_rep >= 20.0:
                last_rep = el
                T = time.time() - t0
                frame_hz = p.n_ok / max(T, 1e-9)
                print('  t=%5.1f s  好 %8d (%.1f Hz)  坏 %6d  丢字节 %d  校验和 %.2f%%  落地 %.1f MB'
                      % (el, p.n_ok, frame_hz, p.n_bad, p.lost,
                         100.0 * p.chk_ok / max(p.chk_n, 1),
                         ((os.path.getsize(out) if fh else 0) + (os.path.getsize(binp) if fb else 0)) / 1e6))
            if p.n_ok == 0 and el > 6.0:
                print('!! 6 s 内没有解析出 A5 5A 帧。可能：')
                print('   1) 固件是验收模式(20B JustFloat 无帧头) -> 用 tools/calib/cap.py，或把')
                print('      v5f_tune.h 的 V5F_CDC_QUAT_ONLY 改成 0u 重编；')
                print('   2) 串口选错 / 没烧进去 / USB CDC 没起来。')
                break
    except KeyboardInterrupt:
        print('\n(用户中断)')
    finally:
        if line_buf and fh:
            fh.write(''.join(line_buf))
        if fh:
            fh.close()
        if fb:
            fb.close()
        ser.close()

    T = time.time() - t0
    tot = p.n_ok + p.n_bad
    print()
    print('=== 结果 ===')
    print('  时长 %.1f s   好帧 %d (%.1f Hz)   坏帧 %d (%.3f%%)   重同步丢字节 %d'
          % (T, p.n_ok, p.n_ok / max(T, 1e-9), p.n_bad, 100.0 * p.n_bad / max(tot, 1), p.lost))
    print('  dt 之和 %.3f s / 墙钟 %.1f s -> 缺口 %.2f s' % (p.dt_sum, T, T - p.dt_sum))
    print('  校验和通过 %d/%d (%.2f%%)   |q| 异常帧 %d'
          % (p.chk_ok, p.chk_n, 100.0 * p.chk_ok / max(p.chk_n, 1), p.qbad))
    tg = '未收到' if p.tag_seen is None else '%.0f' % p.tag_seen
    print('  指纹: 实测 %s  期望 %.0f  %s' % (tg, tag, 'OK' if p.tag_seen == tag else '★不符'))
    if fh:
        print('  文本 %.1f MB -> %s' % (os.path.getsize(out) / 1e6, out))
        print('  下一步: python tools/calib/mag360_cal.py %s' % os.path.basename(out))
    if fb:
        print('  二进制 %.1f MB -> %s' % (os.path.getsize(binp) / 1e6, binp))
    ok = (p.n_ok > 0 and p.tag_seen == tag and p.chk_ok / max(p.chk_n, 1) > 0.99)
    print('  判定: %s' % ('OK，可以用于标定/分析' if ok else '不合格，先解决指纹/校验和/串口问题'))
    return 0 if ok else 2


if __name__ == '__main__':
    sys.exit(main())
