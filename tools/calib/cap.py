# -*- coding: utf-8 -*-
"""
接管串口，按固定采样率抽取 JustFloat 四元数帧，二进制落盘。

帧: 4 x float32 小端 + 4 B 帧尾 00 00 80 7F = 20 B。
分频按**帧计数**（不是按主机时间），所以采样点等间隔、不受主机调度抖动影响。
落盘: 每样本 24 B = float64 t(s) + float32 q[4]。
落盘节奏: 每 5 min flush+fsync 一次。
端口断了（对端复位/重新枚举）自动重连，继续追加同一个文件。
停止: 建一个 cap.stop 文件，或到 --hours。
"""
import os, struct, sys, time
import serial

PORT, RATE, HOURS, FLUSH = 'COM4', 10.0, 8.0, 300.0
OUT = sys.argv[1] if len(sys.argv) > 1 else time.strftime('static_%Y%m%d_%H%M%S.bin')
TAIL, FRAME, STOP = b'\x00\x00\x80\x7f', 20, 'cap.stop'


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
    step = max(1, int(round(fps / RATE)))
    print("实测 %.0f fps -> 每 %d 帧取 1 个 (%.2f Hz)" % (fps, step, fps / step), flush=True)

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
            if i >= FRAME - 4:
                q = struct.unpack_from('<4f', buf, i - (FRAME - 4))
                nframe += 1
                cnt += 1
                if cnt >= step:
                    cnt = 0
                    # 时间基准用**帧序号**（等间隔），主机时钟只作旁证
                    f.write(struct.pack('<dd4f', float(nframe), time.perf_counter() - t_start, *q))
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
