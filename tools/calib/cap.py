# -*- coding: utf-8 -*-
"""
接管串口，按固定采样率抽取 JustFloat 帧，二进制落盘。

帧 = N 路 float32 小端 + 4 B 帧尾 00 00 80 7F（N 由固件决定：当前 7 = 四元数 + 世界系速度，
     32 B 帧；历史上有过 4=q、7=q+加速度原始值、10=q+加速度+陀螺原始值）。
**帧长自动识别**（找相邻两个帧尾的间距整除），所以固件改通道数不用改本脚本。
分频按**帧计数**（不是按主机时间），采样点等间隔、不受主机调度抖动影响。
落盘: 每样本 (16 + 4N) B = float64 帧序号 + float64 主机时刻 + float32 v[N]；
      同时在 <out>.meta 里写 "通道数 帧长"，供 cap_read.py 解析。
落盘节奏: 每 5 min flush+fsync 一次。端口断了自动重连，继续追加同一个文件。
停止: 建一个 cap.stop 文件，或到 --hours。
"""
import os, struct, sys, time
import serial

PORT, RATE, HOURS, FLUSH = 'COM4', 10.0, 8.0, 300.0
OUT = sys.argv[1] if len(sys.argv) > 1 else time.strftime('static_%Y%m%d_%H%M%S.bin')
TAIL, STOP = b'\x00\x00\x80\x7f', 'cap.stop'


def detect_frame(buf):
    """相邻两个帧尾的间距 = 帧长；要求帧长 8..256 且 (帧长-4) 是 4 的倍数"""
    i = buf.find(TAIL)
    if i < 0:
        return None
    j = buf.find(TAIL, i + 4)
    if j < 0:
        return None
    n = j - i
    return n if (8 <= n <= 256 and (n - 4) % 4 == 0) else None


def open_port():
    sp = serial.Serial(PORT, 921600, timeout=0.2)
    try:
        sp.set_buffer_size(rx_size=1 << 20)
    except Exception:
        pass
    return sp


def drain_until_tail(sp, buf):
    """丢掉缓冲里帧尾之前的内容，保证从完整帧开始"""
    while True:
        i = buf.find(TAIL)
        if i >= 0:
            del buf[:i + 4]
            return
        if len(buf) > 64:
            del buf[:-64]
        buf += sp.read(65536)
        if not buf:
            return


def main():
    if os.path.exists(STOP):
        os.remove(STOP)
    sp = open_port()
    print("端口打开 %s" % PORT, flush=True)

    # 用前 2 s 量真实帧率，定分频比
    buf = bytearray()
    t0 = time.perf_counter()
    n = 0
    while time.perf_counter() - t0 < 2.0:
        buf += sp.read(65536)
        while True:
            i = buf.find(TAIL)
            if i < 0:
                break
            n += 1
            del buf[:i + 4]
    fps = n / (time.perf_counter() - t0)
    frame = detect_frame(buf)
    while frame is None:
        buf += sp.read(65536)
        frame = detect_frame(buf)
    nch = (frame - 4) // 4
    step = max(1, int(round(fps / RATE)))
    print("实测 %.0f fps  帧长 %d B -> %d 路 float  -> 每 %d 帧取 1 个 (%.2f Hz)"
          % (fps, frame, nch, step, fps / step), flush=True)
    open(OUT + '.meta', 'w').write("%d %d\n" % (nch, frame))

    f = open(OUT, 'wb', buffering=0)     # 无缓冲：每条样本立即进 OS 缓存，文件大小实时可见
    t_start = time.perf_counter()
    t_print = t_flush = t_start
    nframe = nsamp = 0
    cnt = 0
    print("落盘 %s  最长 %.1f h  flush 每 %.0f s" % (OUT, HOURS, FLUSH), flush=True)

    while True:
        now = time.perf_counter()
        if now - t_start > HOURS * 3600:
            print("到时长，收工", flush=True)
            break
        if os.path.exists(STOP):
            print("看到 %s，收工" % STOP, flush=True)
            break
        try:
            chunk = sp.read(65536)
        except Exception as ex:
            print("  串口异常(%s)，2 s 后重连" % type(ex).__name__, flush=True)
            try:
                sp.close()
            except Exception:
                pass
            time.sleep(2.0)
            try:
                sp = open_port()
                buf = bytearray()
                drain_until_tail(sp, buf)
                print("  已重连", flush=True)
            except Exception as ex2:
                print("  重连失败: %s" % ex2, flush=True)
                time.sleep(3.0)
            continue
        if chunk:
            buf += chunk
        while True:
            i = buf.find(TAIL)
            if i < 0:
                if len(buf) > 64:
                    del buf[:-64]
                break
            if i + 4 > len(buf):
                break
            if i >= frame - 4:
                v = struct.unpack_from('<%df' % nch, buf, i - (frame - 4))
                nframe += 1
                cnt += 1
                if cnt >= step:
                    cnt = 0
                    # 时间基准用**帧序号**（等间隔），主机时钟只作旁证
                    f.write(struct.pack('<dd', float(nframe), time.perf_counter() - t_start))
                    f.write(struct.pack('<%df' % nch, *v))
                    nsamp += 1
            del buf[:i + 4]
        now = time.perf_counter()
        if now - t_flush >= FLUSH:
            f.flush(); os.fsync(f.fileno()); t_flush = now
            print("  [落盘] t=%.0f s  收帧 %d  样本 %d  %.2f MB"
                  % (now - t_start, nframe, nsamp, f.tell() / 1048576.0), flush=True)
        elif now - t_print >= 60.0:
            t_print = now
            print("  t=%.0f s  收帧 %d (%.0f fps)  样本 %d"
                  % (now - t_start, nframe, nframe / max(now - t_start, 1e-9), nsamp), flush=True)

    f.flush(); os.fsync(f.fileno()); f.close()
    try:
        sp.close()
    except Exception:
        pass
    print("结束: %.0f s  收帧 %d  样本 %d  %.2f MB -> %s"
          % (time.perf_counter() - t_start, nframe, nsamp,
             os.path.getsize(OUT) / 1048576.0, OUT), flush=True)


main()
