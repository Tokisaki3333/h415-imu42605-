# -*- coding: utf-8 -*-
"""让分析脚本自己从源码读列数（列数一变，硬编码 reshape 就会读成乱码）。"""
import re

SRC = r'C:\Users\33\Documents\v2\h415-imu42605-\V5F\User\src\SPI_rx.c'
SEL = """NCH = int(re.search(r'#define JF_CH_NUM\\s+(\\d+)u',
                   open(r'%s', 'rb').read().decode('gbk')).group(1))""" % SRC

for p, old in [
    (r'C:\Users\33\Documents\v2\tools\calib\val_ekf.py',
     "a = np.fromfile(P, dtype='<f4').reshape(-1, 112)"),
    (r'C:\Users\33\Documents\v2\tools\calib\seg_gates.py',
     "a = np.fromfile(P, dtype='<f4').reshape(-1, 112)"),
]:
    t = open(p, encoding='utf-8').read()
    assert t.count(old) == 1, (p, t.count(old))
    t = t.replace(old, SEL + "\na = np.fromfile(P, dtype='<f4').reshape(-1, NCH)", 1)
    open(p, 'w', encoding='utf-8', newline='\n').write(t)
    print('已改:', p.split('\\')[-1])
