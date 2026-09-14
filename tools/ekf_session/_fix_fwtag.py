# -*- coding: utf-8 -*-
"""recv_v9.py 两个致命毛病，一起修：

1) 期望指纹写死在脚本里。改了固件版本却忘了同步 -> 每一帧都被挡掉 ->
   实测 25 s 采集成 200668 坏帧 / 0 好帧 / 0 字节。现在**从源码算**：
   (V5F_FW_VER<<16)|(JF_CH_NUM<<8)|EKF|(MAGCAL<<1)|(AC<<2)，和 check_fw.py 同一套。

2) 指纹不符不再丢帧，只**警告一次**。帧长字段已经自描述（112 = 448/4），
   载荷内部还有 |q|=1 的自洽性，版本错配就整段丢掉数据是不可接受的。
"""
p = r'C:\Users\33\Documents\v2\recv_v9.py'
t = open(p, encoding='utf-8').read()

a = "FW = 880647.0          # VER=13"
assert t.count(a) == 1, t.count(a)
b = '''def _expected_fw():
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
    ver = g(r'#define V5F_FW_VER\\s+(\\d+)u', tune)
    nch = g(r'#define JF_CH_NUM\\s+(\\d+)u', src)
    ekf = g(r'#define V5F_EKF_EN\\s+(\\d+)u', tune)
    mag = g(r'#define V5F_MAG_CAL_EN\\s+(\\d+)u', tune)
    ac = g(r'#define V5F_DET_AC_EN\\s+(\\d+)u', tune)
    return float((ver << 16) | (nch << 8) | (ekf << 0) | (mag << 1) | (ac << 2))


FW = _expected_fw()     # 由源码算出（VER/nch/开关），不再手写'''
t = t.replace(a, b, 1)

a2 = """                q = float(np.linalg.norm(m[0:4].astype(np.float64)))
                ok = (abs(q - 1.0) < 1e-3
                      and float(m[IDX_FW]) == FW
                      and 100.0 < float(m[IDX_DT]) < 200.0)
                if ok:"""
b2 = """                q = float(np.linalg.norm(m[0:4].astype(np.float64)))
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
                if ok:"""
assert t.count(a2) == 1, t.count(a2)
t = t.replace(a2, b2, 1)

a3 = "    n_ok = n_bad = 0\n    lost = 0"
assert t.count(a3) == 1
t = t.replace(a3, "    n_ok = n_bad = 0\n    n_tagbad = 0\n    tag_seen = None\n    lost = 0", 1)

a4 = "    print('  落地 %.1f MB -> %s' % (os.path.getsize(a.out) / 1e6, a.out))"
assert t.count(a4) == 1
t = t.replace(a4, a4 + """
    print('  指纹: 实测 %s  期望 %.0f  %s'
          % ('%.0f' % tag_seen if tag_seen is not None else '未收到帧', FW,
             'OK' if tag_seen == FW else ('★不符的帧 %d 个' % n_tagbad)))""", 1)

open(p, 'w', encoding='utf-8', newline='\n').write(t)
import ast
ast.parse(t)
print('recv_v9.py: 指纹改为从源码算 + 不符只警告不丢帧  OK')
print('  源码算出的 fw_tag =', end=' ')
import subprocess
print(subprocess.run(['python', '-c',
                      'import sys;sys.path.insert(0,r"C:\\\\Users\\\\33\\\\Documents\\\\v2");'
                      'import importlib.util as u;'
                      's=u.spec_from_file_location("r",r"C:\\\\Users\\\\33\\\\Documents\\\\v2\\\\recv_v9.py")'],
                     capture_output=True).returncode if False else '')
