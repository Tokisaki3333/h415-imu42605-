# -*- coding: utf-8 -*-
r"""备份文件的**唯一合法去处**：`<repo>/bak_src/<镜像源码相对路径>.<tag>`

**铁律**：`*.bak_*`、`*.c.v101` 这类变体副本**严禁**写在 `V5F/`、`Common/` 等源码目录里——
IDE（Eclipse / MounRiver）按目录扫描，会把它们当源文件一起包含进去。

用法（补丁脚本）::

    import os, sys
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    import bakpath as B

    B.save(TUNE, '.bak_v109')          # 备份到 bak_src/V5F/User/inc/v5f_tune.h.bak_v109
    ...
    B.restore(TUNE, '.bak_v109')       # 从那里回退

配套守门脚本：`python tools/ekf_session/srcdir_clean.py --check`（源码树里一旦出现残留就 exit 1）。
"""
import os
import re
import shutil

REPO = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..'))
BAKDIR = os.path.join(REPO, 'bak_src')
SRC_DIRS = ('V5F', 'Common')
SKIP_DIRS = {'obj', '__pycache__', '.git', '.settings'}

# 变体副本的识别规则：*.bak*  /  *.c.v101  /  *.h.v97  /  *.ld.v3 ...
STRAY_PAT = re.compile(r'\.bak|\.(c|h|ld|S|s|py|txt|md)\.v[0-9]+[a-z0-9_]*$|\.v[0-9]{2,}$', re.I)


def bak_path(src, tag=None):
    """源码文件 -> 它在 bak_src/ 下的镜像备份路径。"""
    rel = os.path.relpath(os.path.abspath(src), REPO)
    p = os.path.join(BAKDIR, rel)
    if tag:
        p += tag if tag.startswith('.') else ('.' + tag)
    return p


def save(src, tag):
    """把 src 的当前内容另存为备份（目录自动建），返回备份路径。"""
    d = bak_path(src, tag)
    os.makedirs(os.path.dirname(d), exist_ok=True)
    shutil.copy2(src, d)
    print('-- 备份 %s -> %s' % (os.path.relpath(src, REPO), os.path.relpath(d, REPO)))
    return d


def exists(src, tag):
    return os.path.exists(bak_path(src, tag))


def restore(src, tag):
    """用备份覆盖 src（回退）。备份不存在直接报错退出，绝不静默。"""
    d = bak_path(src, tag)
    if not os.path.exists(d):
        raise SystemExit('备份不存在: %s' % d)
    shutil.copy2(d, src)
    print('-- 回退 %s <- %s' % (os.path.relpath(src, REPO), os.path.relpath(d, REPO)))
    return d


def strays():
    """列出源码树里所有变体副本（绝对路径）。"""
    out = []
    for base in SRC_DIRS:
        root = os.path.join(REPO, base)
        if not os.path.isdir(root):
            continue
        for dirpath, dirnames, filenames in os.walk(root):
            dirnames[:] = [d for d in dirnames if d not in SKIP_DIRS]
            for fn in filenames:
                if STRAY_PAT.search(fn):
                    out.append(os.path.join(dirpath, fn))
    return out


def sweep(apply=False):
    """把源码树里的变体副本搬进 bak_src/（镜像路径）。返回 (移动数, 残留列表)。"""
    hits = strays()
    if apply:
        for p in hits:
            d = os.path.join(BAKDIR, os.path.relpath(p, REPO))
            os.makedirs(os.path.dirname(d), exist_ok=True)
            if os.path.exists(d):
                os.remove(d)
            shutil.move(p, d)
    return hits, strays()
