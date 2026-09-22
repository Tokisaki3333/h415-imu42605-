# -*- coding: utf-8 -*-
r"""VER=124（正确版）：给**验收帧**加一路 float = 当前地磁入口模式。

⚠️ 作废记录：`patch_v124_magmode_col.py` 改错了对象 —— 它动的是 **调试帧**（162→163 通道），
   而用户要的是 **验收帧**。本脚本先把调试帧**完整回退**（从 .bak_v124 恢复 SPI_rx.c /
   cols_162.py / 6 个 acceptance 脚本），再改验收帧。

验收帧 = `V5F_CDC_QUAT_ONLY != 0` 分支（v5f_tune.h 里 `1u = 验收`）：
    原：`qbuf[20]` = EKF 四元数 4×float32 + 帧尾 00 00 80 7F  -> 20 B/帧，每个 IMU 帧一帧(8 kHz)
    新：`qbuf[24]` = EKF 四元数 4×float32 + **地磁入口模式 1×float32** + 帧尾
         -> **24 B/帧**；第 5 路取值 0/1/2（0=本帧未修正, 1=投影入口(只修正姿态角), 2=矢量入口）
    第 5 路放在帧尾**之前**（JustFloat 的帧尾必须最后 4 字节）—— 所以读取端要把"通道数"
    从 4 改成 5（含帧尾的记录就是 24 B = 6 float）。

调试帧（162 通道 654 B）**一个字节都不动**：`V5F_CDC_QUAT_ONLY` 置 0u 时它照旧工作。

同步更新的解析端（都按"4 通道 + 尾"写死的）：
    tools/acceptance/_accept3.py   20 B 行正则 -> 24 B，帧尾 [20:24]，并统计 mag_mode 分布
    tools/calib/zerodrift.py       20 -> 24 字节，帧尾 [20:24]
    tools/calib/scale_sim.py       reshape(-1,5) -> (-1,6)（4 分量 + 1 地磁 + 尾）
    tools/calib/rec.py             提示文本里的 20B -> 24B
    （tools/calib/jf_load.py 自动识别通道数，无需改；bias_uncertainty.py 是另一种 9 通道帧，不动）

用法: python tools/ekf_session/patch_v124_accept_magmode.py
回退: 见脚本末尾打印的三条 bak_v124 / bak_v123 命令
"""
import os
import re
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import bakpath as B

ROOT = B.REPO
TUNE = os.path.join(ROOT, 'V5F', 'User', 'inc', 'v5f_tune.h')
SPIC = os.path.join(ROOT, 'V5F', 'User', 'src', 'SPI_rx.c')
COLS = os.path.join(ROOT, 'tools', 'calib', 'cols_162.py')
ACC = os.path.join(ROOT, 'tools', 'acceptance')
TAG = '.bak_v124b'

SETTLE = [os.path.join(ROOT, 'V5F', 'User', 'src', 'SPI_rx.c'),
          os.path.join(ROOT, 'tools', 'calib', 'cols_162.py')] + \
         [os.path.join(ACC, f) for f in ('_k98.py', '_magpull.py', '_north3.py',
                                         '_v96unit.py', '_v97check.py', '_v98check.py')]


def _w(path, text):
    b = text.encode('gbk') if path.endswith(('.c', '.h')) else text.encode('utf-8')
    with open(path, 'wb') as fh:
        fh.write(b)
    print('  wrote %s (%d B)' % (os.path.basename(path), len(b)))


def main():
    spic = open(SPIC, 'rb').read().decode('gbk')
    done = 'memcpy(qbuf + 16u, &mm, 4u);' in spic

    if not done:
        # ---------- 0) 先把调试帧那套改动完整回退 ----------
        print('-- 回退调试帧改动（.bak_v124 = VER=123）')
        for p in SETTLE:
            B.restore(p, '.bak_v124')
        spic = open(SPIC, 'rb').read().decode('gbk')
        assert '#define JF_CH_NUM     162u' in spic, '调试帧未回到 162'
        assert 'g_v5f_hold.ekf.mag_mode' not in spic, '调试帧里还有 mag_mode 列'

        # ---------- 1) 验收帧：加第 5 路 ----------
        for a in ('static uint8_t qbuf[20];', 'memcpy(qbuf, qv, 16u);',
                  '(void)hid_up_enqueue(qbuf, 20u);',
                  'qbuf[16] = 0x00u; qbuf[17] = 0x00u; qbuf[18] = 0x80u; qbuf[19] = 0x7Fu;',
                  '4 x float32(小端)', '=> 20 '):
            assert spic.count(a) == 1, ('锚点 x%d: %r' % (spic.count(a), a))
        B.save(SPIC, TAG)
        spic = spic.replace('=> 20 ', '=> 24 ', 1)
        spic = spic.replace('4 x float32(小端)', '5 x float32(小端): q[4] + 地磁入口', 1)
        spic = spic.replace('static uint8_t qbuf[20];',
                            'static uint8_t qbuf[24];   /* VER=124: 5 float + 4 B 帧尾（原 4+尾）*/', 1)
        spic = spic.replace('        float qv[4];', '        float qv[4], mm;', 1)
        spic = spic.replace('        memcpy(qbuf, qv, 16u);',
                            '        memcpy(qbuf, qv, 16u);\n'
                            '        /* VER=124 第 5 路：当前地磁入口（0=未修正 1=投影(只姿态角) 2=矢量）。\n'
                            '         * 放在帧尾**之前** —— JustFloat 要求 00 00 80 7F 是最后 4 字节。 */\n'
                            '        mm = g_v5f_hold.ekf.mag_mode;\n'
                            '        memcpy(qbuf + 16u, &mm, 4u);', 1)
        spic = spic.replace('qbuf[16] = 0x00u; qbuf[17] = 0x00u; qbuf[18] = 0x80u; qbuf[19] = 0x7Fu;',
                            'qbuf[20] = 0x00u; qbuf[21] = 0x00u; qbuf[22] = 0x80u; qbuf[23] = 0x7Fu;', 1)
        spic = spic.replace('(void)hid_up_enqueue(qbuf, 20u);',
                            '(void)hid_up_enqueue(qbuf, 24u);', 1)
        _w(SPIC, spic)

        # ---------- 2) tune 注释：验收帧 20 B -> 24 B ----------
        tune = open(TUNE, 'rb').read().decode('gbk')
        a = '4 x float32 + 00 00 80 7F = 20 B/帧'
        assert tune.count(a) == 1
        tune = tune.replace(a, '5 x float32 + 00 00 80 7F = 24 B/帧', 1)
        tune = tune.replace(' *        * 该分支在 justfloat_report() 开头就 return；',
                            ' *        * VER=124 第 5 路 = 地磁入口模式(0/1/2)，放在帧尾之前；\n'
                            ' *        * 该分支在 justfloat_report() 开头就 return；', 1)
        _w(TUNE, tune)

        # ---------- 3) 解析端 ----------
        # 3a) tools/acceptance/_accept3.py
        p = os.path.join(ACC, '_accept3.py')
        t = open(p, encoding='utf-8').read()
        assert '((?:[0-9A-Fa-f]{2} ){19}[0-9A-Fa-f]{2})' in t and 'b[16:20]' in t
        B.save(p, TAG)
        t = t.replace('"""常规工况验收: VER=100 CDC 四元数流(每行一帧 20B)',
                      '"""常规工况验收: CDC 四元数流(每行一帧 24B = q[4] + 地磁入口 + 帧尾 00 00 80 7F)', 1)
        t = t.replace('((?:[0-9A-Fa-f]{2} ){19}[0-9A-Fa-f]{2})',
                      '((?:[0-9A-Fa-f]{2} ){23}[0-9A-Fa-f]{2})', 1)
        t = t.replace('ts, qs, nline, bad = [], [], 0, 0',
                      'ts, qs, mds, nline, bad = [], [], [], 0, 0', 1)
        t = t.replace("    qs.append(np.frombuffer(b[:16], dtype='<f4'))\n"
                      "    if b[16:20] != b'\\x00\\x00\\x80\\x7f': bad += 1",
                      "    qs.append(np.frombuffer(b[:16], dtype='<f4'))\n"
                      "    mds.append(float(np.frombuffer(b[16:20], dtype='<f4')[0]))   # VER=124 地磁入口\n"
                      "    if b[20:24] != b'\\x00\\x00\\x80\\x7f': bad += 1", 1)
        assert 'mds.append' in t and "b[20:24]" in t
        _w(p, t)
        # 3b) zerodrift.py
        p = os.path.join(ROOT, 'tools', 'calib', 'zerodrift.py')
        t = open(p, encoding='utf-8').read()
        assert '{20})' in t and 'b[16:20]' in t
        B.save(p, TAG)
        t = t.replace(r"((?:[0-9A-Fa-f]{2}\s*){20})", r"((?:[0-9A-Fa-f]{2}\s*){24})", 1)
        t = t.replace("b[16:20] != b'\\x00\\x00\\x80\\x7f'", "b[20:24] != b'\\x00\\x00\\x80\\x7f'", 1)
        _w(p, t)
        # 3c) scale_sim.py
        p = os.path.join(ROOT, 'tools', 'calib', 'scale_sim.py')
        t = open(p, encoding='utf-8').read()
        assert 'reshape(-1, 5)' in t
        B.save(p, TAG)
        t = t.replace('reshape(-1, 5)', 'reshape(-1, 6)', 1)
        _w(p, t)
        # 3d) rec.py 提示文本
        p = os.path.join(ROOT, 'tools', 'calib', 'rec.py')
        t = open(p, encoding='utf-8').read()
        if '20B JustFloat' in t:
            B.save(p, TAG)
            t = t.replace('20B JustFloat', '24B JustFloat', 1)
            _w(p, t)
        # 3e) cols_162.py：列名表不动（调试帧没变），只把 VER_EXPECT 跟上
        t = open(COLS, encoding='utf-8').read()
        if 'VER_EXPECT = 107' in t:
            B.save(COLS, TAG)
            t = t.replace('VER_EXPECT = 107', 'VER_EXPECT = 124', 1)
            _w(COLS, t)
        print('patched: 验收帧 20B -> 24B（+地磁入口），调试帧回到 162 通道')
    else:
        print('已是 VER=124 验收帧版（只校验）')

    # ---------------- verify ----------------
    ok = True
    tune = open(TUNE, 'rb').read().decode('gbk')
    spic = open(SPIC, 'rb').read().decode('gbk')
    ekf = open(os.path.join(ROOT, 'V5F', 'User', 'src', 'proc_ekf.c'), 'rb').read().decode('gbk')
    hs = open(os.path.join(ROOT, 'V5F', 'User', 'inc', 'SPI_rx.h'), 'rb').read().decode('gbk')
    assert re.search(r'#define V5F_FW_VER\s+124u', tune)
    assert re.search(r'#define V5F_CDC_QUAT_ONLY 1u', tune), '验收模式必须是开的'
    assert '5 x float32 + 00 00 80 7F = 24 B/帧' in tune
    # 验收帧
    assert 'static uint8_t qbuf[24];' in spic
    assert 'mm = g_v5f_hold.ekf.mag_mode;' in spic and 'memcpy(qbuf + 16u, &mm, 4u);' in spic
    assert 'qbuf[20] = 0x00u; qbuf[21] = 0x00u; qbuf[22] = 0x80u; qbuf[23] = 0x7Fu;' in spic
    assert '(void)hid_up_enqueue(qbuf, 24u);' in spic
    assert spic.count('g_v5f_hold.ekf.mag_mode') == 1
    # 调试帧**没被动**
    assert '#define JF_CH_NUM     162u' in spic and '*   159/160 ' in spic
    assert '*   161 ' in spic and '160/161' not in spic
    # 供验收帧取值的链路
    assert 'float            mag_mode;' in hs
    assert ekf.count('static float    s_mag_mode;') == 1
    assert ekf.count('if (st == 0u) s_mag_mode = 2.0f;') == 1
    assert ekf.count('if (st == 0u) s_mag_mode = 1.0f;') == 1
    assert ekf.count('h->ekf.mag_mode    = s_mag_mode;') == 1
    # 解析端
    t = open(os.path.join(ACC, '_accept3.py'), encoding='utf-8').read()
    assert '{23}' in t and 'b[20:24]' in t and 'mds.append' in t
    t = open(os.path.join(ROOT, 'tools', 'calib', 'zerodrift.py'), encoding='utf-8').read()
    assert '{24}' in t and 'b[20:24]' in t
    t = open(os.path.join(ROOT, 'tools', 'calib', 'scale_sim.py'), encoding='utf-8').read()
    assert 'reshape(-1, 6)' in t
    import subprocess
    r = subprocess.run([sys.executable, COLS], capture_output=True)
    _o = (r.stdout or r.stderr).decode('gbk', errors='replace').strip().splitlines()
    print('  cols_162 selfcheck rc=%d | %s'
          % (r.returncode, (_o[-1] if _o else '').encode('ascii', 'replace').decode('ascii')))
    ok &= (r.returncode == 0)
    for nm, txt in (('proc_ekf.c', ekf), ('SPI_rx.c', spic)):
        s2 = re.sub(r'/\*.*?\*/', '', txt, flags=re.S)
        assert s2.count('{') == s2.count('}') and s2.count('(') == s2.count(')'), nm + ' 括号不平衡'
    print('VERIFY', 'OK' if ok else 'FAIL')
    print('回退: 验收帧 -> Copy-Item bak_src\\V5F\\User\\src\\SPI_rx.c.bak_v124b V5F\\User\\src\\SPI_rx.c -Force')
    print('      调试帧那套 -> git checkout HEAD~1 -- V5F/User/src/SPI_rx.c tools/calib/cols_162.py（或 .bak_v124）')
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
