# -*- coding: utf-8 -*-
# ============================================================================
# 【作废 / 改错了帧】本脚本把 mag_mode 加到了**调试帧**（JF_CH_NUM 162->163）。
# 用户要的是**验收帧**（V5F_CDC_QUAT_ONLY != 0 那条 20 B JustFloat）。
# 正确脚本：patch_v124_accept_magmode.py（它第一步就把本脚本的改动从 .bak_v124 全量回退）。
# 本文件保留仅为记录这次错，不要再运行。
# ============================================================================
r"""VER=124：验收帧(JustFloat)新增一列 float —— 当前地磁入口模式。

用户要求："在验收模式上报加一个float位表示当前地磁模式"。

新增列（**追加在末尾、不改任何旧列号**，沿用 VER=95 的做法）：
    159  ekf.mag_mode   本观测帧**实际生效**的地磁入口
                          0 = 本帧没有地磁修正（门控/采样去重/剔除/更新失败）
                          1 = 入口 A：重力法平面投影（只修正姿态角，roll/pitch 不参与）
                          2 = 入口 B：矢量方式（e1 偏航 + e2 倾斜）
    160/161  DRDY 时间戳（原 159/160）
    162      整帧 XOR 校验和（原 161）
    NCH 162 -> 163（fw_tag 里的通道位自动跟着变 -> (124<<16)|(163<<8)|7）

改动文件：
    V5F/User/inc/v5f_tune.h   VER 123u -> 124u（改了帧就必须 bump）
    V5F/User/inc/SPI_rx.h     v5f_ekf_t 末尾加 float mag_mode
    V5F/User/src/proc_ekf.c   s_mag_mode：每观测帧清零 -> 成功执行入口时置 1/2 -> publish
    V5F/User/src/SPI_rx.c     JF_CH_NUM 163u + 新列 + 列号注释同步（159/160/161 -> 160/161/162）
    tools/calib/cols_162.py   权威列名表：NCH/GROUPS/ANCHOR/VER_EXPECT 同步（模块名保留，
                              因为 tools/calib/mag360_cal.py 还在 import 它）
    tools/acceptance/_*.py    6 个脚本的通道数自动识别改成"先试 163"

用法: python tools/ekf_session/patch_v124_magmode_col.py
回退: Copy-Item bak_src\V5F\User\{inc\v5f_tune.h,inc\SPI_rx.h,src\proc_ekf.c,src\SPI_rx.c}.bak_v123 ... -Force
      （BACKUP TAG = .bak_v124 = 本次改动前的 VER=123 状态）
"""
import os
import re
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import bakpath as B

ROOT = B.REPO
TUNE = os.path.join(ROOT, 'V5F', 'User', 'inc', 'v5f_tune.h')
SPIH = os.path.join(ROOT, 'V5F', 'User', 'inc', 'SPI_rx.h')
EKF = os.path.join(ROOT, 'V5F', 'User', 'src', 'proc_ekf.c')
SPIC = os.path.join(ROOT, 'V5F', 'User', 'src', 'SPI_rx.c')
COLS = os.path.join(ROOT, 'tools', 'calib', 'cols_162.py')
ACC = os.path.join(ROOT, 'tools', 'acceptance')
TAG = '.bak_v124'

NEW_EFK_FIELD = (
    '    float            mag_mode;        /* VER=124 本观测帧实际生效的地磁入口\n'
    '                                       *   0 = 未修正(门控/采样去重/剔除/更新失败)\n'
    '                                       *   1 = 入口A 重力法平面投影(只修正姿态角)\n'
    '                                       *   2 = 入口B 矢量方式(e1 偏航+e2 倾斜) */\n')

NEW_FRAME_COLS = [
    '',
    '        /* ---- VER=124 再追加一列（仍**不改任何旧列号**）------------------------------',
    '         *   159      ekf.mag_mode  本帧实际生效的地磁入口 0/1/2（见下）',
    '         *   160/161  DRDY 时间戳（原 159/160）',
    '         *   162      整帧 XOR 校验和（原 161）',
    '         *   NCH 162 -> 163；fw_tag 的通道位自动变成 163。',
    '         *   mag_mode 只在**入口真的执行成功**那一帧置 1/2，被门控/去重/剔除时是 0，',
    '         *   所以"地磁到底有没有在修姿态、用哪种方式"一眼可读。 */',
    '        ch[c++] = g_v5f_hold.ekf.mag_mode;   /* VER=124 地磁入口 0=未修正 1=投影(只姿态角) 2=矢量 */',
]


def _w(path, text):
    """先编码再打开写：否则 encode 抛异常时文件已被 open('wb') 截断成 0 字节（踩过一次）。"""
    b = text.encode('gbk') if path.endswith(('.c', '.h')) else text.encode('utf-8')
    with open(path, 'wb') as fh:
        fh.write(b)
    print('  wrote %s (%d B)' % (os.path.basename(path), len(b)))


def main():
    tune = open(TUNE, 'rb').read().decode('gbk')
    spih = open(SPIH, 'rb').read().decode('gbk')
    ekf = open(EKF, 'rb').read().decode('gbk')
    spic = open(SPIC, 'rb').read().decode('gbk')
    cols = open(COLS, encoding='utf-8').read()

    if re.search(r'#define V5F_FW_VER\s+124u', tune) and 'mag_mode' in spic:
        print('已是 VER=124（只校验）')
    else:
        assert re.search(r'#define V5F_FW_VER\s+123u', tune), '基线不是 VER=123'
        assert 'mag_mode' not in spic and 'mag_mode' not in ekf and 'mag_mode' not in spih
        assert spic.count('#define JF_CH_NUM') == 1 and '162u' in spic

        # 锚点自检
        assert spih.count('    uint8_t          _rsv_ekf[1];') == 1
        assert ekf.count('static float    s_mag_rs;') == 1
        assert ekf.count('            s_mag_used = 0u;') == 1
        assert ekf.count('h->ekf.mag_used') == 1
        assert ekf.count('        st = ekf_mag_entry_vec(r2v, RRv);') == 1
        assert ekf.count('        st = ekf_mag_entry_plane(thm, thp);') == 1
        assert spic.count('ch[c++] = (float)g_v5f_hold.ekf.mag_used;') == 2
        assert spic.count('*   159/160 ') == 1 and spic.count('(159-160)') == 1
        assert spic.count('*   161 ') == 1
        assert cols.count('NCH = 162') == 1 and cols.count('VER_EXPECT = 107') == 1

        for f in (TUNE, SPIH, EKF, SPIC):
            B.save(f, TAG)
        B.save(COLS, TAG)

        # ---------- 1) tune: VER ----------
        tune, n = re.subn(r'#define V5F_FW_VER\s+123u', '#define V5F_FW_VER        124u', tune, count=1)
        assert n == 1
        _w(TUNE, tune)

        # ---------- 2) SPI_rx.h: hold 结构加字段（追加在 _rsv_ekf 之前）----------
        spih = spih.replace('    uint8_t          _rsv_ekf[1];',
                            NEW_EFK_FIELD + '    uint8_t          _rsv_ekf[1];', 1)
        _w(SPIH, spih)

        # ---------- 3) proc_ekf.c ----------
        ekf = ekf.replace('static float    s_mag_rs;',
                          'static float    s_mag_rs;\n'
                          'static float    s_mag_mode;            /* VER=124 上报：本观测帧生效的地磁入口 0/1/2 */', 1)
        ekf = ekf.replace('            s_mag_used = 0u;',
                          '            s_mag_used = 0u; s_mag_mode = 0.0f;   /* VER=124 入口标志每观测帧清零 */', 1)
        ekf = ekf.replace('        st = ekf_mag_entry_vec(r2v, RRv);      /* 入口 B：矢量方式 */',
                          '        st = ekf_mag_entry_vec(r2v, RRv);      /* 入口 B：矢量方式 */\n'
                          '        if (st == 0u) s_mag_mode = 2.0f;       /* VER=124 上报：矢量入口 */', 1)
        ekf = ekf.replace('        st = ekf_mag_entry_plane(thm, thp);    /* 入口 A：重力法平面投影，只修正姿态角 */',
                          '        st = ekf_mag_entry_plane(thm, thp);    /* 入口 A：重力法平面投影，只修正姿态角 */\n'
                          '        if (st == 0u) s_mag_mode = 1.0f;       /* VER=124 上报：投影入口 */', 1)
        # publish：h->ekf.mag_used 那行后面加一行（用行插入，避免中文注释锚点）
        _L = ekf.split('\n')
        _mi = [i for i, l in enumerate(_L) if 'h->ekf.mag_used' in l]
        assert len(_mi) == 1
        _L[_mi[0] + 1:_mi[0] + 1] = ['    h->ekf.mag_mode    = s_mag_mode;   /* VER=124 */']
        ekf = '\n'.join(_L)
        _w(EKF, ekf)

        # ---------- 4) SPI_rx.c ----------
        spic = spic.replace('#define JF_CH_NUM     162u', '#define JF_CH_NUM     163u', 1)
        # 新列插到**第二处** mag_used 之后（VER=95 追加块末尾）
        _L = spic.split('\n')
        _mi = [i for i, l in enumerate(_L) if l.strip().startswith('ch[c++] = (float)g_v5f_hold.ekf.mag_used;')]
        assert len(_mi) == 2, _mi
        _L[_mi[1] + 1:_mi[1] + 1] = NEW_FRAME_COLS
        spic = '\n'.join(_L)
        # 帧尾注释里的列号同步
        spic = spic.replace('*   159/160 ', '*   160/161 ', 1)
        spic = spic.replace('(159-160)', '(160-161)', 1)
        spic = spic.replace('*   161 ', '*   162 ', 1)
        _w(SPIC, spic)

        # ---------- 5) cols_162.py（权威列名表）----------
        cols = cols.replace(""""VER=101 调试帧(JustFloat, 162 通道)的权威列名表。""",
                            """"VER=124 调试帧(JustFloat, 163 通道)的权威列名表。""", 1)
        cols = cols.replace("""      mag_used=120, tick=159/160, checksum=161）。""",
                            """      mag_used=120, mag_mode=159, tick=160/161, checksum=162）。""", 1)
        cols = cols.replace("""    ('ekf_mag_gate_out', 1), ('ekf_mag_used_out', 1),""",
                            """    ('ekf_mag_gate_out', 1), ('ekf_mag_used_out', 1),
    ('ekf_mag_mode', 1),                       # VER=124 追加：0=未修正 1=投影(只姿态角) 2=矢量""", 1)
        cols = cols.replace('NCH = 162\n', 'NCH = 163\n', 1)
        cols = cols.replace("""    'mag_ok': 154, 'tick_tk': 159, 'tick_md': 160, 'checksum': 161,""",
                            """    'mag_ok': 154, 'ekf_mag_mode': 159, 'tick_tk': 160, 'tick_md': 161,
    'checksum': 162,""", 1)
        cols = cols.replace('VER_EXPECT = 107', 'VER_EXPECT = 124', 1)
        cols = cols.replace("print('cols_162: 162 列 / %d 组 / 锚点全部命中' % len(GROUPS))",
                            "print('cols_162: 163 列 / %d 组 / 锚点全部命中' % len(GROUPS))", 1)
        _w(COLS, cols)

        # ---------- 6) tools/acceptance：通道数自动识别先试 163 ----------
        pat_old = '162 if (sz//4) % 162 == 0 else 159'
        pat_new = '163 if (sz//4) % 163 == 0 else (162 if (sz//4) % 162 == 0 else 159)'
        pat_old2 = '162 if (sz//4) % 162 == 0 else (159 if (sz//4) % 159 == 0 else 154)'
        pat_new2 = ('163 if (sz//4) % 163 == 0 else (162 if (sz//4) % 162 == 0 else '
                    '(159 if (sz//4) % 159 == 0 else 154))')
        nfix = 0
        for fn in sorted(os.listdir(ACC)):
            if not fn.endswith('.py'):
                continue
            p = os.path.join(ACC, fn)
            t = open(p, encoding='utf-8', errors='replace').read()
            t2 = t.replace(pat_old2, pat_new2).replace(pat_old, pat_new)
            if t2 != t:
                B.save(p, TAG)
                _w(p, t2)
                nfix += 1
            elif 'NCH = 162\n' in t:      # _v96unit.py 那种硬编码
                t2 = t.replace('NCH = 162\n', 'NCH = 163 if raw.size % 163 == 0 else 162   # VER=124\n', 1)
                B.save(p, TAG)
                _w(p, t2)
                nfix += 1
        print('  acceptance 脚本改了 %d 个' % nfix)

    # ---------------- verify ----------------
    ok = True
    tune = open(TUNE, 'rb').read().decode('gbk')
    spih = open(SPIH, 'rb').read().decode('gbk')
    ekf = open(EKF, 'rb').read().decode('gbk')
    spic = open(SPIC, 'rb').read().decode('gbk')
    cols = open(COLS, encoding='utf-8').read()
    assert re.search(r'#define V5F_FW_VER\s+124u', tune)
    assert '#define JF_CH_NUM     163u' in spic
    assert spic.count('g_v5f_hold.ekf.mag_mode') == 1
    assert spih.count('float            mag_mode;') == 1
    assert ekf.count('static float    s_mag_mode;') == 1
    assert ekf.count('s_mag_used = 0u; s_mag_mode = 0.0f;') == 1
    assert ekf.count('if (st == 0u) s_mag_mode = 2.0f;') == 1
    assert ekf.count('if (st == 0u) s_mag_mode = 1.0f;') == 1
    assert ekf.count('h->ekf.mag_mode    = s_mag_mode;') == 1
    # 各 2 处：我新增的说明块 + 原地更新的帧尾注释
    assert spic.count('*   160/161 ') == 2 and spic.count('(160-161)') == 1
    assert spic.count('*   162 ') == 2
    assert spic.count('*   159/160 ') == 0 and spic.count('(159-160)') == 0 and spic.count('*   161 ') == 0
    assert cols.count('NCH = 163') == 1 and cols.count('VER_EXPECT = 124') == 1
    assert cols.count("('ekf_mag_mode', 1)") == 1 and cols.count("'ekf_mag_mode': 159") == 1
    assert cols.count("'tick_tk': 160, 'tick_md': 161") == 1 and cols.count("'checksum': 162") == 1
    # cols_162.py 自检必须过（组数合计 == NCH、锚点全中）
    import subprocess
    r = subprocess.run([sys.executable, COLS], capture_output=True)
    _out = (r.stdout or r.stderr).decode('gbk', errors='replace').strip().splitlines()
    _last = _out[-1] if _out else ''
    print()
    print('  cols_162 selfcheck rc=%d | %s'
          % (r.returncode, _last.encode('ascii', 'replace').decode('ascii')))
    ok &= (r.returncode == 0)
    e2 = re.sub(r'/\*.*?\*/', '', ekf, flags=re.S)
    assert e2.count('{') == e2.count('}') and e2.count('(') == e2.count(')'), 'ekf 括号不平衡'
    s2 = re.sub(r'/\*.*?\*/', '', spic, flags=re.S)
    assert s2.count('{') == s2.count('}') and s2.count('(') == s2.count(')'), 'spi 括号不平衡'
    print('VERIFY', 'OK' if ok else 'FAIL')
    print('回退: Copy-Item bak_src\\V5F\\User\\inc\\v5f_tune.h.bak_v124 V5F\\User\\inc\\v5f_tune.h -Force')
    print('      Copy-Item bak_src\\V5F\\User\\inc\\SPI_rx.h.bak_v124 V5F\\User\\inc\\SPI_rx.h -Force')
    print('      Copy-Item bak_src\\V5F\\User\\src\\proc_ekf.c.bak_v124 V5F\\User\\src\\proc_ekf.c -Force')
    print('      Copy-Item bak_src\\V5F\\User\\src\\SPI_rx.c.bak_v124 V5F\\User\\src\\SPI_rx.c -Force')
    print('      Copy-Item bak_src\\tools\\calib\\cols_162.py.bak_v124 tools\\calib\\cols_162.py -Force')
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
