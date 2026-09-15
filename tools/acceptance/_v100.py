# -*- coding: utf-8 -*-
"""VER=100: CDC(EP2 bulk IN) 上报改为 EKF 四元数, JustFloat 标准格式。
   载荷 = 4×float32(小端) + 帧尾 00 00 80 7F = 20 字节/帧, 每 IMU 帧一帧。
   用"函数开头早退"实现: 旧 162 通道组帧代码一字不删, 由 V5F_CDC_QUAT_ONLY 一行切回。"""
import os, re, shutil

def patch(path, subs, bak):
    raw = open(path, 'rb').read(); txt = raw.decode('gbk')
    eol = '\r\n' if b'\r\n' in raw else '\n'
    if not os.path.exists(bak):
        shutil.copy2(path, bak); print('备份 ->', bak)
    st = os.stat(path)
    for nm, pat, rep, cnt in subs:
        r2 = (lambda m: rep(m).replace('\n', eol)) if callable(rep) else rep.replace('\n', eol)
        txt, n = re.subn(pat, r2, txt, count=cnt)
        assert n == cnt, '%s %s 命中 %d' % (path, nm, n)
        print('  [ok] %-14s 命中 %d' % (nm, n))
    open(path, 'wb').write(txt.encode('gbk'))
    os.utime(path, (st.st_atime, st.st_mtime))
    a = raw.decode('gbk').split(eol); b = txt.split(eol)
    print('  %s: %d -> %d 行; 消失旧行 %s' %
          (os.path.basename(path), len(a), len(b), [x[:50] for x in a if x not in b] or '无'))

T = r'h415-imu42605-\V5F\User\inc\v5f_tune.h'
S = r'h415-imu42605-\V5F\User\src\SPI_rx.c'

NEWM = """#define V5F_CDC_QUAT_ONLY 1u   /* VER=100: CDC(EP2) 只报 EKF 四元数(JustFloat: 4 float + 00 00 80 7F)
                                * 置 0u 即切回旧 162 通道 JustFloat 日志, 其余一字不动 */"""
patch(T, [('VER 98->100', r'(#define V5F_FW_VER\s+)98u', r'\g<1>100u', 1),
          ('新增开关', r'(#define V5F_EKF_EN\s+1u[^\n]*)', lambda m: NEWM.replace('\n', '\n') + '\n' + m.group(1), 1)],
      T + '.bak_v100')

BLK = """#if (V5F_CDC_QUAT_ONLY != 0u)
    /* ---- VER=100: CDC(EP2 bulk IN) 上报改为 EKF 四元数, JustFloat 标准格式 ----
     * 载荷 = 4 x float32(小端) + 帧尾 00 00 80 7F  => 20 字节/帧, 每个 IMU 帧一帧(8 kHz)。
     * 只报 EKF 姿态; 不报 att.q、不报其它通道。帧内无版本指纹(JustFloat 无标签位)。
     * 切回旧 162 通道日志: v5f_tune.h 里 V5F_CDC_QUAT_ONLY 改 0u。 */
    {
        static uint8_t qbuf[20];        /* static: 本函数在 DMA1 中断最深层, 栈只有 2 KB */
        float qv[4];
        qv[0] = g_v5f_hold.ekf.q[0];
        qv[1] = g_v5f_hold.ekf.q[1];
        qv[2] = g_v5f_hold.ekf.q[2];
        qv[3] = g_v5f_hold.ekf.q[3];
        memcpy(qbuf, qv, 16u);
        qbuf[16] = 0x00u; qbuf[17] = 0x00u; qbuf[18] = 0x80u; qbuf[19] = 0x7Fu;
        (void)hid_up_enqueue(qbuf, 20u);
    }
    return;
#endif
"""
A = '    for (i = 0u; i < 4u; i++) ch[c++]     = g_v5f_hold.att.q[i];'
patch(S, [('早退块', re.escape(A), BLK + A, 1)], S + '.bak_v100')

L = open(S, 'rb').read().decode('gbk').split('\n')
i = next(i for i, x in enumerate(L) if 'VER=100: CDC(EP2' in x)
print('\nSPI_rx.c 回读:'); [print('  %4d %s' % (k+1, L[k].rstrip()[:115])) for k in range(i-4, i+22)]
L2 = open(T, 'rb').read().decode('gbk').split('\n')
print('v5f_tune.h 回读:')
for k, x in enumerate(L2):
    if 'V5F_FW_VER' in x or 'V5F_CDC_QUAT_ONLY' in x: print('  %4d %s' % (k+1, x.rstrip()[:115]))
# 括号配平检查
for p in (T, S):
    n = open(p, 'rb').read().decode('gbk')
    print('%s 花括号 %+d 圆括号 %+d' % (os.path.basename(p), n.count('{')-n.count('}'), n.count('(')-n.count(')')))
