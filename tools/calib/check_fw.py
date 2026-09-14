# -*- coding: utf-8 -*-
"""核对记录里的构建指纹与源码是否一致 —— **每次刷完先跑这个**。

为什么要有它：实测 20260914_233341 那次，磁盘上的 ELF 含 [1.0334 0.8494 12.0910]，
设备却按 [1.788 11.458 8.346] 跑，整条 V5F 链的永久死锁全部来自"分析了一份不是
你以为的固件产生的数据"。刷写没生效时，后面所有分析都是白做，所以这一步必须在最前面。

用法:  python check_fw.py <日志文件> [...]
退出码: 0 = 全部一致；1 = 有不一致或记录太旧无指纹。
"""
import os
import re
import sys

R = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(R, 'tools', 'calib'))
import jf_load  # noqa: E402


def expected():
    """从源码算出期望的 fw_tag（与 SPI_rx.c 的 JF_FW_TAG 同一编码）。"""
    t = open(os.path.join(R, 'V5F', 'User', 'inc', 'v5f_tune.h'), 'rb').read().decode('gbk')
    r = open(os.path.join(R, 'V5F', 'User', 'src', 'SPI_rx.c'), 'rb').read().decode('gbk')

    def g(pat, s, d=None):
        m = re.search(pat, s)
        if m:
            return int(m.group(1))
        if d is None:
            raise SystemExit('源码里找不到 %s' % pat)
        return d
    ver = g(r'#define V5F_FW_VER\s+(\d+)u', t)
    mag = g(r'#define V5F_MAG_CAL_EN\s+(\d+)u', t)
    ekf = g(r'#define V5F_EKF_EN\s+(\d+)u', t, 0)
    ac = g(r'#define V5F_DET_AC_EN\s+(\d+)u', t, 0)
    nch = g(r'#define JF_CH_NUM\s+(\d+)u', r)
    tag = (ver << 16) | (nch << 8) | (ekf << 0) | (mag << 1) | (ac << 2)
    return tag, dict(VER=ver, nch=nch, EKF=ekf, MAGCAL=mag, AC=ac)


def main(files):
    tag, info = expected()
    print('源码期望 fw_tag = %d   %s' % (tag, info))
    print()
    bad = 0
    for fn in files:
        try:
            a = jf_load.load_jf(fn)
        except Exception as e:
            print('  %-52s  读不到: %s' % (os.path.basename(fn), e))
            bad += 1
            continue
        nch = a.shape[1]
        ch = jf_load.ch_for(nch) if nch in jf_load.CH_BY_NCH else None
        if ch is None or 'fw_tag' not in ch:
            print('  %-52s  %d 通道 -> 早于阶段 0，无指纹（无法核对）' % (os.path.basename(fn), nch))
            bad += 1
            continue
        obs = float(a[0, ch['fw_tag']])
        uniq = set(float(x) for x in a[:, ch['fw_tag']])
        ok = (obs == float(tag)) and len(uniq) == 1
        print('  %-52s  %d 通道  实测 fw_tag = %-7g  %s' %
              (os.path.basename(fn), nch, obs, 'OK' if ok else '**不一致**'))
        if not ok:
            if len(uniq) != 1:
                print('      指纹在记录内还变了: %s' % sorted(uniq)[:5])
            else:
                print('      => 刷写没生效（或刷了别的镜像）！先重刷再分析。')
            bad += 1
    return 1 if bad else 0


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print(__doc__)
        raise SystemExit(0)
    raise SystemExit(main(sys.argv[1:]))
