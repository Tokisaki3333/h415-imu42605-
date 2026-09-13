# tools/calib —— 标定与验证脚本

本目录是**固件常量的出处**。`V5F/User/inc/v5f_proc.h` 里每个标定值都能在这里找到
生成它的脚本；改常量前先跑对应脚本，不要手改。

## 依赖的数据（不入库）

原始记录是上位机抓的 JustFloat 串口日志，体积从几十 MB 到 200 MB，已由 `.gitignore`
排除（`serial_runtime_*.txt`）。脚本按文件名硬编码引用，重跑前需把对应记录放回仓库根目录。

| 记录 | 用途 |
|---|---|
| `serial_runtime_20260912_215604_069_export.txt` | 151 s 静止，零偏偏置初值 `GB_BIAS0_*` 的来源 |
| `serial_runtime_20260912_230325_916_export.txt` | 74.5 s 冷启动，牵引收敛/阈值表设计 |
| `serial_runtime_20260913_174952_015_export.txt` | 171 s 六面，零偏偏置不确定度 |
| `serial_runtime_20260913_181436_858` / `181548_799` | z 轴正反 360 度整转（**旧 9 通道格式**） |
| `serial_runtime_20260913_182119_546` / `182228_199` | y 轴正反 180 度（旧格式，夹具限位） |
| `serial_runtime_20260913_182626_777` / `182645_181` | x 轴正反 180 度（旧格式，夹具限位） |
| `serial_runtime_20260913_190158_640` / `190359_575` | x 轴单周 360 度（**四元数格式**） |
| `serial_runtime_20260913_190713_772` / `190745_795` | y 轴单周 360 度双向（四元数） |
| `serial_runtime_20260913_184905_223` / `191953_344` | 复杂长翻滚（四元数） |
| `serial_runtime_20260913_193108_876` / `193303_307` / `193339_436` | 三条不同形状长翻滚（四元数） |
| `serial_runtime_20260913_193816_365` | **样本外校验条**（四元数） |
| `serial_runtime_20260913_195111_831` | 400 s 纯静止，零漂/Allan |
| `static_*.bin` | `cap.py` 落的降频二进制静止记录（见下） |

两种记录格式：旧 9 通道 = 原始/校正/零偏各 3 路（40 B 帧）；新 = 4 路四元数（20 B 帧）。

## 脚本 → 固件常量

| 固件常量 | 脚本 |
|---|---|
| `V5F_GYRO_LSB_PER_DPS_X/Y/Z`、`V5F_GYRO_KXY/KXZ/KYZ` | **`joint_fit2.py`**（9 条四元数记录联立，6 未知 / 27 方程） |
| `s_thr_tab[20]`（`proc_static_detect.c`） | **`build_thr_table.py`** |
| `V5F_DET_THR_BASE` / `V5F_DET_W` / `V5F_DET_DEB_ON` / `V5F_DET_DEB_OFF` | `static_detect_design.py` |
| `V5F_DET_THR_OFF_RATIO`（迟滞比） | `detect_effect.py` |
| `V5F_PROC_ROLLBACK_GRANULES`（=24） | `rollback_sim.py` / `rollback_sizing.py` |
| `GB_TAU_TICKS`（12 s）/ `GB_TAU_BOOT_TICKS`（2 s） | `bias_traction_sweep.py` |
| `GB_BIAS0_LSB_X/Y/Z` | `bias_uncertainty.py`（**待六面校正替换**） |

## 脚本清单

**标度 / 耦合标定**
- `joint_fit2.py` —— 9 条记录联立解绝对测量阵（S + 对称 K）。**定稿出处。**
  关键点：每条记录录制时固件版本不同，传递阵 `M_fw` 必须分版本，否则 K 被作用两次。
- `joint_fit.py` —— 6 条记录版（历史，已被 `joint_fit2.py` 取代）
- `coupling_fit.py` —— 6 条旧整转离线拟合。**注意**：这里解出的非对角项混进了夹具转轴失准
  （反对称份），比真实对称交叉项大 3~5 倍，不要拿它当速率修正载入。
- `turn360.py` —— 单周 360 度整圈直接量该轴残余标度误差（只用姿态 + 固件 DRDY 时间戳，
  与主机帧率无关）。x/y 轴的实测出处。
- `sym_fit.py` / `peraxis_test.py` —— 对角 vs 对称 3x3 的对比试验（证明"每轴一个标度"不够）

**误差反解 / 校验**
- `scale_sim.py` —— 复杂运动末态误差反解：多大的标度误差能造出观测到的偏差；含灵敏度表
- `err_check.py` —— 同一激励在不同固件版本下的预测偏差（判断某条记录跑的哪一版）
- `valid.py` / `newcheck.py` / `ratio.py` —— 样本外校验条读取、误差/行程比例、固定底+比例项拟合

**零偏 / 噪声**
- `bias_uncertainty.py` —— 零偏不确定度、Allan、漂移
- `bias_traction_sweep.py` —— 牵引时间常数扫描 + 无牵引空窗代价
- `sixface.py` —— 六面数据分析
- `zerodrift.py` —— 纯静止记录的零漂 / 重叠 Allan / ARW / 零偏不稳定性

**动静判定 / 回溯**
- `static_detect_design.py` —— 统计量与阈值设计
- `build_thr_table.py` —— 生成 `s_thr_tab[]`
- `detect_effect.py` —— 阈值/去抖的实际检出效果
- `rollback_sim.py` / `rollback_sizing.py` —— 回溯粒度 N 的选型
- `vib_sim.py` —— 振动激励下的动态阈值仿真

**在线采集（降频落盘）**
- `cap.py` —— 接管串口，按帧计数分频（默认 10 Hz），无缓冲增量写盘、每 5 min fsync、
  断线自动重连。落盘格式：每样本 32 B = float64 帧序号 + float64 主机时刻 + float32 q[4]。
  时间基准用**帧序号 / 实测帧率**（等间隔），主机时钟只用于核对丢帧。
- `cap_read.py` —— 读上述二进制，输出漂移与 Allan（`--allan`）

## 复现步骤（标度为例）

```
# 需要下列四元数记录放在仓库根目录
python tools/calib/turn360.py          # 先看单轴整圈残余
python tools/calib/joint_fit2.py       # 联立解 S 与 K
# 把输出抄进 V5F/User/inc/v5f_proc.h
```
