# -*- coding: utf-8 -*-
"""rec.py 的自检（无硬件）：合成 A5 5A 流 -> Parser -> 写二进制 -> cols_162 回读，逐列一致。

另测：纯 payload 流（无帧头）的相位自动识别。
用法: python tools/calib/selftest_rec.py
"""
import os
import struct
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import cols_162 as C      # noqa: E402
import rec as R           # noqa: E402

TMP = os.environ.get('TEMP', '.')


def synth(n=400, framed=True):
    ver, nch, tag = R.expected_fw()
    rng = np.random.default_rng(7)
    q = np.array([1.0, 0.0, 0.0, 0.0])
    frames = []
    for k in range(n):
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
        fr[C.CH_162['mag_lsb0']:C.CH_162['mag_lsb0'] + 3] = [100, -40, 60]
        fr[C.CH_162['ist_cnt']] = k // 2
        fr[C.CH_162['fw_tag']] = tag
        xk = C.chk(fr[:C.NCH - 1].astype('<f4').tobytes())
        fr[C.NCH - 1] = float(xk)
        pay = fr.tobytes()
        frames.append(R.HDR + struct.pack('<H', len(pay)) + pay + R.TL if framed else pay)
    return frames, tag, nch


def main():
    C.selfcheck()
    for framed in (True, False):
        # 解析器永远吃"带帧"流；落盘格式才分带帧 / 纯 payload 两种
        frames, tag, nch = synth(400, True)
        stream = b''.join(frames) + b'\x11\x22'          # 尾部垃圾，考验重同步
        p = R.Parser(nch, tag)
        got = p.push(stream)
        path = os.path.join(TMP, 'rec_selftest_%s.bin' % ('fr' if framed else 'pl'))
        with open(path, 'wb') as f:
            f.write(b''.join(got) if framed else b''.join(x[4:-2] for x in got))
        back, info = C.load_frames(path)
        want = np.stack([np.frombuffer(x[4:-2], dtype='<f4') for x in got])
        same = back.shape == want.shape and np.array_equal(back, want)
        print('framed=%-5s 解析 %d/%d 帧 丢字节 %d 校验和 %d/%d  tag %.0f  回读 %s %s -> %s'
              % (framed, len(got), len(frames), p.lost, p.chk_ok, p.chk_n, p.tag_seen,
                 back.shape, info['format'], '一致' if same else '★不一致'))
        assert len(got) == len(frames) and p.chk_ok == p.chk_n == len(frames)
        assert abs(p.tag_seen - tag) < 0.5 and same
    print('SELFTEST OK（带帧 / 纯 payload 两种落盘都能回读）')


if __name__ == '__main__':
    main()
