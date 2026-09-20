# bak_src/ —— 源码变体副本的唯一存放处

**铁律：`V5F/`、`Common/` 等源码目录里严禁出现 `*.bak_*`、`*.c.v101`、`*.h.v97` 这类变体副本。**
IDE（Eclipse / MounRiver）按目录扫描，会把它们当成源文件一起包含 ⇒ 轻则编译报重复定义，重则选错文件。

本目录镜像原源码树路径，例如：

```
bak_src/V5F/User/inc/v5f_tune.h.bak_v108      <- 原来是 V5F/User/inc/v5f_tune.h.bak_v108
bak_src/V5F/User/src/proc_ekf.c.bak_v103_magvec
bak_src/Common/Common/GPS.c.bak_cn0
```

## 规矩

1. **补丁脚本一律用 `tools/ekf_session/bakpath.py`**：
   ```python
   import sys, os
   sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
   import bakpath as B
   B.save(TUNE, '.bak_v109')      # -> bak_src/V5F/User/inc/v5f_tune.h.bak_v109
   B.restore(TUNE, '.bak_v109')   # 回退
   ```
2. **任何改动后跑一次守门脚本**（源码树有残留就 exit 1）：
   ```
   python tools/ekf_session/srcdir_clean.py --check     # 只检查
   python tools/ekf_session/srcdir_clean.py --apply     # 把残留搬进本目录
   ```
3. 回退示例（PowerShell）：
   ```powershell
   Copy-Item bak_src\V5F\User\inc\v5f_tune.h.bak_v108 V5F\User\inc\v5f_tune.h -Force
   ```
4. 这些文件**仍在 git 里**（是 `git mv` 过来的），所以历史可追溯；不要 `git rm` 掉，
   除非确实不需要回退点了。

（2026-09-21 一次性搬入 321 个历史副本：`V5F/` 301 个 + `Common/` 20 个。）
