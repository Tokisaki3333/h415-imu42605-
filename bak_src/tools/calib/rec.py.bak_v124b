# -*- coding: utf-8 -*-
"""串口录制（唯一动作：把 A5 5A 帧原样落成 R 盘一个带时间戳的二进制文件）。

    python tools/calib/rec.py          # 默认 COM4，录到 Ctrl-C
    python tools/calib/rec.py 90       # 只录 90 秒
    python tools/calib/rec.py 90 --port COM7

- 串口默认 **COM4**（隐式，不用输）；波特率对 USB CDC 无意义，固定 921600。
- 落地固定为 **R:\\imu_YYYYmmdd_HHMMSS.bin**（原始字节流，含 A5 5A 帧头/帧尾；R 盘不可用直接报错退出）。
- 录制中逐帧校验并打印：好帧/坏帧/重同步丢字节/校验和通过率/速率/大小。
- 结束打印汇总与判定；随后可直接
      python tools/calib/mag360_cal.py R:\\imu_YYYYmmdd_HHMMSS.bin
"""
import datetime as dt
import os
import struct
import sys
import time

HDR, TL = b'\xa5\x5a', b'\x5a\xa5'
PORT = 'COM4'
BAUD = 921600
RDIR = 'R:\\'


def expected_fw():
    """期望指纹从源码现算（不写死）。"""
    import re
    root = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
    tune = open(os.path.join(root, 'V5F', 'User', 'inc', 'v5f_tune.h'), 'rb').read().decode('gbk')
    src = open(os.path.join(root, 'V5F', 'User', 'src', 'SPI_rx.c'), 'rb').read().decode('gbk')

    def g(pat, s):
        m = re.search(pat, s)
        return int(m.group(1)) if m else 0
    ver = g(r'#define V5F_FW_VER\s+(\d+)u', tune)
    nch = g(r'#define JF_CH_NUM\s+(\d+)u', src)
    return ver, nch, float((ver << 16) | (nch << 8)
                           | g(r'#define V5F_EKF_EN\s+(\d+)u', tune)
                           | (g(r'#define V5F_MAG_CAL_EN\s+(\d+)u', tune) << 1)
                           | (g(r'#define V5F_DET_AC_EN\s+(\d+)u', tune) << 2))


class Parser:
    """增量解析 A5 5A 帧；逐帧查 |att.q|、dt、指纹、校验和。返回整帧原始字节。"""

    def __init__(self, want_nch, want_tag):
        self.buf = bytearray()
        self.want_nch, self.want_tag = want_nch, want_tag
        self.nch = None
        self.n_ok = self.n_bad = self.lost = self.qbad = 0
        self.chk_ok = self.chk_n = 0
        self.tag_seen = None
        self.dt_sum = 0.0
        self.first = None

    def push(self, data):
        import numpy as np
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
            frame = bytes(self.buf[:ln + 6])
            del self.buf[:ln + 6]
            nch = ln // 4
            if self.nch is None:
                self.nch = nch
            ok = True
            if nch == self.want_nch and nch > 76:
                f = np.frombuffer(frame[4:4 + ln], dtype='<f4')
                q = float(np.linalg.norm(f[0:4].astype(np.float64)))
                dtu = float(f[25])
                tag = float(f[76])
                if self.tag_seen is None:
                    self.tag_seen = tag
                xk = 0x5A5A          # 与固件 SPI_rx.c `uint16_t xk = 0x5A5Au;` 一致
                for b in frame[4:4 + (nch - 1) * 4]:
                    xk ^= b
                self.chk_n += 1
                if abs(float(f[nch - 1]) - float(xk)) < 0.5:
                    self.chk_ok += 1
                if abs(q - 1.0) > 2e-3:
                    self.qbad += 1
                    ok = False
                if not (1.0 < dtu < 20000.0):
                    ok = False
                self.dt_sum += dtu * 1e-6
                if self.first is None:
                    self.first = (nch, tag, q, dtu)
            if ok:
                self.n_ok += 1
                out.append(frame)
            else:
                self.n_bad += 1
        return out


def main():
    import argparse
    ap = argparse.ArgumentParser(usage='%(prog)s [秒数] [--port COM4]')
    ap.add_argument('sec', nargs='?', type=float, default=None, help='录制秒数；省略则录到 Ctrl-C')
    ap.add_argument('--port', default=PORT)
    a = ap.parse_args()

    if not os.path.isdir(RDIR):
        print('!! %s 不存在或不可写：录制必须落在 R 盘，先确认盘符' % RDIR)
        return 2
    out = os.path.join(RDIR, 'imu_%s.bin' % dt.datetime.now().strftime('%Y%m%d_%H%M%S'))
    k = 1
    while os.path.exists(out):
        out = os.path.join(RDIR, 'imu_%s_%d.bin' % (dt.datetime.now().strftime('%Y%m%d_%H%M%S'), k))
        k += 1

    try:
        import serial
    except ImportError:
        print('!! 缺 pyserial: pip install pyserial')
        return 2

    ver, nch, tag = expected_fw()
    ser = serial.Serial(a.port, BAUD, timeout=0.02, write_timeout=0.5)
    try:
        ser.set_buffer_size(rx_size=1 << 23, tx_size=8192)
    except Exception:
        pass
    print('%s @ %d   期望 VER=%s CH=%s fw_tag=%.0f   落地 %s' % (a.port, BAUD, ver, nch, tag, out))
    print('(Ctrl-C 结束)' if a.sec is None else '(录 %.0f s)' % a.sec)

    p = Parser(nch, tag)
    fb = open(out, 'wb')
    t0 = time.time()
    last = 0.0
    warn6 = False
    try:
        while a.sec is None or (time.time() - t0) < a.sec:
            el = time.time() - t0
            nw = ser.in_waiting
            if nw:
                fr = p.push(ser.read(nw))
                if fr:
                    fb.write(b''.join(fr))
            else:
                time.sleep(0.0005)
            if not warn6 and p.n_ok == 0 and el > 6.0:
                warn6 = True
                print('!! 6 s 没有 A5 5A 帧：固件可能是验收模式(20B JustFloat)先跑 cap.py，'
                      '或端口选错(用 --port)')
            if el - last >= 10.0:
                last = el
                print('  %5.1f s  好 %8d (%.0f Hz)  坏 %6d  丢字节 %d  校验和 %.1f%%  %.1f MB'
                      % (el, p.n_ok, p.n_ok / max(el, 1e-9), p.n_bad, p.lost,
                         100.0 * p.chk_ok / max(p.chk_n, 1), fb.tell() / 1e6))
    except KeyboardInterrupt:
        print('\n(中断)')
    finally:
        fb.close()
        ser.close()

    T = time.time() - t0
    print('\n=== 汇总 ===')
    print('  时长 %.1f s   好帧 %d (%.1f Hz)   坏帧 %d   重同步丢字节 %d'
          % (T, p.n_ok, p.n_ok / max(T, 1e-9), p.n_bad, p.lost))
    print('  校验和 %d/%d (%.2f%%)   |q| 异常 %d   dt 之和/墙钟 = %.2f/%.1f s'
          % (p.chk_ok, p.chk_n, 100.0 * p.chk_ok / max(p.chk_n, 1), p.qbad, p.dt_sum, T))
    if p.first:
        nch0, tag0, q0, dt0 = p.first
        print('  首帧: %d 通道  fw_tag=%.0f (期望 %.0f)  |q|=%.6f  dt=%.0f us'
              % (nch0, tag0, tag, q0, dt0))
    print('  文件 %.1f MB -> %s' % (os.path.getsize(out) / 1e6, out))
    ok = p.n_ok > 0 and p.tag_seen == tag and p.chk_ok / max(p.chk_n, 1) > 0.99
    print('  判定: %s' % ('OK' if ok else '不合格（指纹/校验和/端口）'))
    print('  下一步: python tools/calib/mag360_cal.py "%s"' % out)
    return 0 if ok else 2


if __name__ == '__main__':
    sys.exit(main())
