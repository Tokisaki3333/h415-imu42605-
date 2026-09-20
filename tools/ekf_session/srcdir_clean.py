# -*- coding: utf-8 -*-
r"""守门：源码树（V5F/、Common/）里**不允许**存在任何变体副本（*.bak_*、*.c.v101 …）。

IDE(Eclipse/MounRiver) 按目录扫描，会把这类文件当成源文件一起包含 ⇒ 必须在源码目录之外。
本脚本把它们搬进 `bak_src/`（镜像路径，git 里仍可追溯）。

用法:
  python tools/ekf_session/srcdir_clean.py            # 只列出（不动文件）
  python tools/ekf_session/srcdir_clean.py --apply     # 搬到 bak_src/
  python tools/ekf_session/srcdir_clean.py --check     # 有残留 -> exit 1（给流水线/习惯用）
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import bakpath as B


def main():
    check = '--check' in sys.argv
    apply = '--apply' in sys.argv
    hits, left = B.sweep(apply=apply)
    if not hits:
        print('源码树干净：V5F/ 与 Common/ 下没有变体副本。')
        return 0
    print('发现 %d 个变体副本（严禁留在源码目录）：' % len(hits))
    for p in hits[:20]:
        print('   ', os.path.relpath(p, B.REPO))
    if len(hits) > 20:
        print('    ... 其余 %d 个' % (len(hits) - 20))
    if apply:
        print('已搬到 %s；复查残留 %d 个' % (B.BAKDIR, len(left)))
        return 1 if left else 0
    print('\n跑 `--apply` 搬到 bak_src/（或在补丁脚本里改用 bakpath.save()）')
    return 1 if check else 0


if __name__ == '__main__':
    sys.exit(main())
