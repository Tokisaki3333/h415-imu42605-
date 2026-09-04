# CH32H417 + ICM-42605 串口陀螺仪(IMU) 工程

> **工程状态（重要）**
> 目前仅实现**基本驱动**：硬件外设、总线读写、各传感器寄存器配置与单点读数均可工作；
> **尚未做多传感器融合**（例如 EKF / Mahony 等姿态融合、轴对齐、零漂标定与本体系补偿），
> 亦无闭环运行验证。当前 README 描述的是**开发过程中的同步快照**，非最终稳定版。

## 简介

本项目基于 **沁恒 CH32H417**（RISC-V 内核 MounRiver/双核架构）开发的串口陀螺仪/惯性测量
工程，配套惯性传感器 **TDK ICM-42605**，并同步集成磁/气压/卫星等多路扩展传感器与 OLED/USB 输出。

工程沿袭旧目录时的演进节奏，当前采用双核任务划分（V3F / V5F 两块固件），通过
进程间共享内存（`mipc`）交换数据与同步。

## 目录结构

```
h415-imu42605-/
├── V3F/        # 3 号核固件 User 工程（传感器采集侧）
├── V5F/        # 5 号核固件 User 工程（姿态/输出侧，含多传感器汇总）
├── Common/     # 两核共享公共源码（外设驱动、LVGL、GPS、Debug 等）
├── _资料/      # 本地文档/手册（不入版本库）
└── CH32串口陀螺仪.wvsln
```

- `Common/Peripheral`、`Common/Core`、`Common/Startup`、`Common/USBHS`：WCH 标准外设库/Core，
  基本来自芯片 EVT 模板，除非修改否则无需关注。
- `Common/Common`：工程自有外设驱动与功能模块。
- `Common/Common/LVGL`：第三方图形库依赖，一般不改。

### 传感器与外设清单

| 项 | 说明 | 驱动/模块 |
|----|------|-----------|
| ICM-42605 | 六轴陀螺仪+加速度计（主 IMU） | `Common/Common` / 采集侧 |
| IST8310 | 三轴磁力计 | `IST8310` |
| BMP388 | 气压计 | `bmp388` |
| GPS | 卫星定位 | `GPS` |
| W25N01 | NAND Flash 存储 | `spi_flash_w25n` |
| SSD1306 OLED | 显示 | `oled_ssd1306` |
| USB CDC / HID | 上位机通信 | `USBHS` |
| SPI/I2C | 总线（软/硬） | `spi_*` / `i2c_*` |

## 版本管理说明

代码仓库使用 **Git 做所有备份/快照**，历史演进请以 `git log` 为准。
不再用 ZIP 快照方式留存旧版本（旧 ZIP 备份已归档在本地，不入库）。

- 每到一个可用里程碑，提交一次并打 tag（如 `v3`, `v5`, 定点回归等）。
- 编译产物（`obj/`、`*.elf/hex` 等）一律不纳入版本管理（见 `.gitignore`）。
- 本地整工程快照/安全存档可用 `git bundle` 或 `git tag`，见下。

### 常用备份（Git 机制）

```bash
git add -A && git commit -m "vX: <改动说明>"        # 常规提交(备份点)
git tag vX                                          # 打版本标签,随时可回退
git push origin main                                # 推送到 GitHub 远程备份

# 离线整仓打包快照(不经服务器):
git bundle create ../h415_backup_$(date +%Y%m%d).bundle --all
```

## 编译

各核代码按 CH32H417 工程导入编译工具链（MounRiver Studio / GCC + objcopy）编译即可，
生成对应核固件烧录。未内置一键脚本。

## 待办 / 演进方向

- [ ] EKF / Mahony 等多传感器融合（含 IST8310/BMP388/GPS 松耦合）
- [ ] 轴标定、零漂采样与温度补偿
- [ ] 闭环/稳定性验证与台架测试
- [ ] 进一步美化同步的工程注释与文档

## 参考文档
见 `_资料/`：芯片参考手册、传感器数据手册与中文寄存器手册、CASIC(GPS) 协议规范。
