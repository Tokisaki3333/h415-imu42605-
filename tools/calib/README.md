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
| `serial_runtime_20260913_222919_239` … `223149_605`（7 条） | 独立六面（**±4 g 档**），加速度计六面法 |
| `serial_runtime_20260913_224354_704` | 连续六面（±4 g） |
| `serial_runtime_20260913_225736_268` | 连续六面（**±16 g**），六面法定稿 |
| `serial_runtime_20260913_231221_019` | 桌面平动正方形 + 原地抬升 10.5 cm（±16 g），加速度动静门控验证 |
| `serial_runtime_20260913_233731_085` | **绕机体 z 单轴旋转**夹具 462°，11 个静止点（±16 g），椭圆法给 S_x/S_y/b_x/b_y |
| `serial_runtime_20260913_234829_229` | **绕机体 y 单轴旋转**夹具 649°，17 个静止点（±16 g），椭圆法给 S_x/S_z/b_x/b_z |
| `serial_runtime_20260914_000636_751` | **绕机体 x 单轴旋转**夹具 437°，14 个静止点（±16 g），椭圆法给 S_y/S_z/b_y/b_z |
| `static_*.bin` | `cap.py` 落的降频二进制静止记录（见下） |

三种记录格式：旧 9 通道 = 原始/校正/零偏各 3 路（40 B 帧）；四元数 = 4 路（20 B 帧）；
当前 = 四元数 4 路 + 加速度原始 LSB 3 路（**7 路，32 B 帧**）。`jf_load.py` 只认当前这种。

## 脚本 → 固件常量

| 固件常量 | 脚本 |
|---|---|
| `V5F_GYRO_LSB_PER_DPS_X/Y/Z`、`V5F_GYRO_KXY/KXZ/KYZ` | **`joint_fit2.py`**（9 条四元数记录联立，6 未知 / 27 方程） |
| **`V5F_ACCEL_LSB_PER_G_X/Y/Z`、`V5F_ACCEL_BIAS_LSB_X/Y/Z`** | **`accel_ellipse_cal.py`**（绕 x/y/z 三条单轴旋转记录，椭圆法，每轴两个独立来源；六面法 `sixface16.py` 作交叉校验）。**注意：b 只是上电初值**，零偏是每场次量，见 `bias_consistency.py` |
| `s_thr_tab[20]`（`proc_static_detect.c`） | **`build_thr_table.py`** |
| `V5F_DET_THR_BASE` / `V5F_DET_W` / `V5F_DET_DEB_ON` / `V5F_DET_DEB_OFF` | `static_detect_design.py` |
| `V5F_DET_THR_OFF_RATIO`（迟滞比） | `detect_effect.py` |
| `V5F_PROC_ROLLBACK_GRANULES`（=24） | `rollback_sim.py` / `rollback_sizing.py` |
| `GB_TAU_TICKS`（12 s）/ `GB_TAU_BOOT_TICKS`（2 s） | `bias_traction_sweep.py` |
| `GB_BIAS0_LSB_X/Y/Z` | `bias_uncertainty.py`（**待六面校正替换**） |

## 脚本清单

**公共读取器**
- `jf_load.py` —— JustFloat 串口日志的流式读取 + `<日志>.npy` 缓存。**新脚本一律用它**，
  不要再写 `re.findall(open(fn).read())`：那会把 25~48 MB 的文本整份读进内存、
  再造出几十万个小字符串。
- 另一条更重要的教训：`np.linalg.svd(A)` 的 `full_matrices` **默认 True**，
  对 (57601, 6) 的矩阵会返回 57601x57601 的 U —— 26.5 GB。所有调用一律显式写
  `full_matrices=False`。

**标度 / 耦合标定**
- `joint_fit2.py` —— 9 条记录联立解绝对测量阵（S + 对称 K）。**定稿出处。**
  关键点：每条记录录制时固件版本不同，传递阵 `M_fw` 必须分版本，否则 K 被作用两次。
- `joint_fit.py` —— 6 条记录版（历史，已被 `joint_fit2.py` 取代）
- `coupling_fit.py` —— 6 条旧整转离线拟合。**注意**：这里解出的非对角项混进了夹具转轴失准
  （反对称份），比真实对称交叉项大 3~5 倍，不要拿它当速率修正载入。
- `turn360.py` —— 单周 360 度整圈直接量该轴残余标度误差（只用姿态 + 固件 DRDY 时间戳，
  与主机帧率无关）。x/y 轴的实测出处。
- `sym_fit.py` / `peraxis_test.py` —— 对角 vs 对称 3x3 的对比试验（证明"每轴一个标度"不够）

**加速度计标定**
- `accel_ellipse_cal.py` —— **加速度计 6 个常量的定稿出处。** 单轴旋转夹具的椭圆法：
  静止时比力大小恒为 1 g，绕固定机体轴转就在垂直平面里画半径 1 g 的圆，读数把这个圆
  映射成椭圆，于是**椭圆中心 = 零偏、半轴 = 逐轴标度**，不需要知道任何停止角度。
  两条记录分别绕 z（覆盖 x/y）和绕 y（覆盖 x/z），x 轴因此有一次独立交叉校验
  （S_x 两源差 0.018%、b_x 差 0.040 mg）。**现在绕 x 的第三条也到手了**，于是六个常量
  每个都有两个完全独立的来源：S_z 两源差 **0.004%**、b_y 两源差 0.348 mg
  （b_y 原先椭圆法与六面法差 1.03 mg，就是靠这条判掉的：两个椭圆来源一致、六面法单独偏出）。
  **必须只用静止段**：转轴与传感器不共点，转动时有 a_c = w^2 r 与 alpha r；本夹具手转，
  转动段残差 rms 60~90 mg 且不随 w^2 变（是平动抖动而非向心项），两者都靠只取静止段排除。
  阈值从 0.25 dps 放宽到 5 dps 结果不变（污染 <= 0.2 mg）；不取静止段的对照组会把
  S_x 拉偏 1.0~1.5%、离面残差从 0.5 LSB 涨到 17~37 LSB。
- `ellipse_fit_check.py` —— **椭圆中心的拟合方式校验。** 085 的静止点在 180° 附近挤了三个
  （相邻 1.3°），而代数圆锥直接最小二乘对角度分布不均是**有偏的**，必须先证明中心没偏，
  才能说椭圆法与六面法在 b_y 上真的不一致。结论：圆锥拟合与 Fitzgibbon 直接拟合给出完全
  相同的中心（差 < 0.0001 LSB）；几何圆拟合偏 1.9/2.2 LSB，但它正交距离 rms 2.52 LSB
  而椭圆只有 0.2 LSB —— 圆是差 12 倍的模型，偏移算在它头上。故 b_y = −7.703 mg 可信。
- `sixface16.py` —— ±16 g 连续六面法，交叉校验用（三轴标度与椭圆法一致到 0.063 个百分点）
- `bias_consistency.py` —— **零偏的逐场次体检。** 用与姿态无关的硬事实（静止时比力模长
  恒为 1 g）在每条记录上量 `(|a_b| - 1g)`，不受倾角/拉平/积分影响。结论：离线标定管得住
  标度（两源互差 0.004%~0.06%）但管不住零偏，各场次有效零偏相差可达 ~1 mg（b_z 约 2 LSB）。
  详见 `v5f_proc.h` 的"零偏是每场次量"一节。
- `traj3d.py` —— 轨迹反解与正方形校验。`OPT_CAL=ell|six` 选标定组，`OPT_ZERO=1` 用运动
  开始前的第一段静止把 `|a_b|` 归一化到 1 g（即每场次零偏清零）。正方形记录实测：
  椭圆标定不清零末端 1.221 m；清零后 0.514 m；六面标定清零后 0.522 m —— 清零后标度用哪套
  没区别，主导误差就是零偏。
- `sixface_accel.py` —— 早期六面分析
- `continuous6.py` / `drift_track.py` —— 连续六面的零偏漂移跟踪

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

加速度计那条更简单（三条记录，几秒钟，输出直接就是那 6 个 `#define`）：

```
# 需要这三条 7 通道记录放在仓库根目录（绕 x / y / z 各一条）
python tools/calib/accel_ellipse_cal.py \
    serial_runtime_20260913_233731_085_export.txt \
    serial_runtime_20260913_234829_229_export.txt \
    serial_runtime_20260914_000636_751_export.txt
python tools/calib/ellipse_fit_check.py     # 确认椭圆中心没被角度分布带偏
python tools/calib/sixface16.py             # 六面法交叉校验（标度应一致到 0.07 个百分点）
```
