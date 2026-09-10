#ifndef MIPC_SHM_H
#define MIPC_SHM_H

#include <stdint.h>

/* =====================================================================
 * 双核共享区定义（V3F 小核定义实例，V5F 大核经 IPC 基址映射同一地址）
 * 两端必须 include 同一份本头文件，保证结构体布局一致。
 *
 * 发布-订阅协议（每个通道独立）：
 *   写端（小核）：写数据 → 写 ts 高 32 → 写 ts 低 32 → __sync_synchronize() → cnt++
 *   读端（大核）：cur = cnt；if (cur != last) { __sync_synchronize(); last = cur; 读 ts 低→高; 读数据; }
 * 时间戳均为 DRDY 就绪时刻，单位统一为 10 ns 计数（64 位，1 tick = 10 ns，由 GetTime64_10Ns() 取），不是读完数据的时刻。
 * ===================================================================== */

/* 64 位时间戳：显式拆高/低 32 位，保证"低 32 位最后写 / 最先读"（防撕裂） */
typedef union {
    volatile uint64_t u64;
    volatile uint32_t u32[2];   /* [0]=低 32 位, [1]=高 32 位 */
} ts64_t;

/* 陀螺仪（ICM-42605）通道：2kHz DRDY 中断只共享时间戳；六轴数据走 SPI_rx DMA 通道 */
typedef struct {
    volatile uint32_t cnt;          /* 帧序号（发布标记，最后写）*/
    ts64_t            ts_drdy_tick;   /* DRDY 上升沿时刻，10 ns 计数 */
} gyro_chan_t;

/* IST8310 通道 */
typedef struct {
    volatile uint32_t cnt;          /* 帧序号（发布标记，最后写）*/
    ts64_t            ts_drdy_tick;   /* 就绪(DRDY)时刻，10 ns 计数 */
    volatile int16_t  mx, my, mz;   /* 磁力计原始值 */
} ist_chan_t;

/* BMP388 通道（定点，与串口 A: 格式一致，避免浮点传递）*/
typedef struct {
    volatile uint32_t cnt;          /* 帧序号（发布标记，最后写）*/
    ts64_t            ts_drdy_tick;   /* 就绪时刻（STATUS 轮询到 drdy），10 ns 计数 */
    volatile int32_t  temp_x1000;   /* 温度 ℃×1000（0.001℃） */
    volatile int32_t  press_x1000;  /* 压力 Pa×1000（0.001Pa，匹配 BMP388 分辨率） */
} bmp_chan_t;

/* GPS 通道：RMC / GGA / GSA 三种语句各一个独立通道、独立 cnt（严格 cnt 协议，
 * 时间戳只绑定本通道字段，互不覆盖）。某字段本帧无数据时 *_valid=0（显式空标志
 * ——0 值本身可能合法，勿用值判空）。
 * 定点单位：lat/lon ×1e7 度（南纬/西经为负）、speed cm/s、
 *          hdop/pdop/vdop ×100、alt cm、date ddmmyy */
typedef struct {
    volatile uint32_t cnt;          /* RMC 帧序号（发布标记，最后写）*/
    ts64_t            ts_drdy_tick;   /* RMC 语句解析批次时刻，10 ns 计数 */
    volatile uint8_t  status_valid; /* RMC 状态字段存在 */
    volatile uint8_t  status;       /* 'A' 有效 / 'V' 无效 */
    volatile uint8_t  pos_valid;    /* 经纬度存在 */
    volatile int32_t  lat_e7;
    volatile int32_t  lon_e7;
    volatile uint8_t  speed_valid;
    volatile uint32_t speed_cmps;
    volatile uint8_t  date_valid;
    volatile uint32_t date_ddmmyy;
} gps_rmc_chan_t;

typedef struct {
    volatile uint32_t cnt;          /* GGA 帧序号（发布标记，最后写）*/
    ts64_t            ts_drdy_tick;   /* GGA 语句解析批次时刻，10 ns 计数 */
    volatile uint8_t  quality_valid;
    volatile uint8_t  quality;      /* 0=无效 1=单点 2=差分 4=RTK固定 5=RTK浮点 */
    volatile uint8_t  sv_valid;
    volatile uint8_t  sv;           /* 卫星数 */
    volatile uint8_t  hdop_valid;
    volatile uint16_t hdop_x100;
    volatile uint8_t  alt_valid;
    volatile int32_t  alt_cm;
} gps_gga_chan_t;

typedef struct {
    volatile uint32_t cnt;          /* GSA 帧序号（发布标记，最后写）*/
    ts64_t            ts_drdy_tick;   /* GSA 语句解析批次时刻，10 ns 计数 */
    volatile uint8_t  dop_valid;    /* PDOP/VDOP 存在（各系统 GSA 独立计数，最新一条覆盖）*/
    volatile uint16_t pdop_x100;
    volatile uint16_t vdop_x100;
} gps_gsa_chan_t;

/* 低频文本通道（mipc_v3_printf，每秒一帧）*/
typedef struct {
    volatile uint32_t cnt;
    volatile uint8_t  data_buf[256];
} log_chan_t;

/* 整体共享区 */
typedef struct {
    volatile uint32_t v5f_progress; /* 原 channel0：V5F 握手进度 1..5 */
    volatile uint32_t v3f_cfg_done; /* 原 channel2：IMU 配置完成信号 */
    log_chan_t        log;          /* 原 channel4：低频文本 */
    gyro_chan_t       gyro;         /* 陀螺仪 DRDY 时间戳 */
    ist_chan_t        ist;
    bmp_chan_t        bmp;
    gps_rmc_chan_t    gps_rmc;
    gps_gga_chan_t    gps_gga;
    gps_gsa_chan_t    gps_gsa;
    /* OLED (SSD1306) 共享显存：128×64/8 = 1024 字节，页主序（8 页 × 128 列）；
     * 双核可读写，aligned(4) 保证 4 字节对齐（双核/DMA 访问安全）。 */
    volatile uint8_t  oled_fb[1024] __attribute__((aligned(4)));
} shared_mem_t;

/* 共享区指针：V3F 定义 storage 并由 g_shm 指向（地址经 IPC_MSG0 传给 V5F）；V5F 同样以 g_shm 命名 */
extern volatile shared_mem_t *g_shm;

/* ---------------- 写端（小核）---------------- */

/* 写 64 位时间戳（本工程统一 10 ns 计数）：高 32 位先写，低 32 位最后写，随后发布屏障 */
static inline void shm_ts_write(volatile ts64_t *ts, uint64_t value)
{
    ts->u32[1] = (uint32_t)(value >> 32);   /* 高 32 位先写 */
    ts->u32[0] = (uint32_t)value;           /* 低 32 位最后写 */
    __sync_synchronize();                   /* 屏障：保证 ts 写在 cnt++ 前全局可见 */
}

/* 陀螺仪：无数据字段，ts → cnt++ */
static inline void shm_publish_gyro(uint64_t ts_drdy)
{
    shm_ts_write(&g_shm->gyro.ts_drdy_tick, ts_drdy);
    g_shm->gyro.cnt++;
}

/* IST8310：先写数据，再写 ts，最后 cnt++ */
static inline void shm_publish_ist(uint64_t ts_drdy, int16_t mx, int16_t my, int16_t mz)
{
    g_shm->ist.mx = mx;
    g_shm->ist.my = my;
    g_shm->ist.mz = mz;
    shm_ts_write(&g_shm->ist.ts_drdy_tick, ts_drdy);
    g_shm->ist.cnt++;
}

/* BMP388：先写数据，再写 ts，最后 cnt++ */
static inline void shm_publish_bmp(uint64_t ts_drdy, int32_t temp_x1000, int32_t press_x1000)
{
    g_shm->bmp.temp_x1000  = temp_x1000;
    g_shm->bmp.press_x1000 = press_x1000;
    shm_ts_write(&g_shm->bmp.ts_drdy_tick, ts_drdy);
    g_shm->bmp.cnt++;
}

/* ---------------- 读端（大核）---------------- */

/* 读 64 位时间戳：低 32 位先读，再读高 32 位（与写端顺序配对）*/
static inline uint64_t shm_ts_read(const volatile ts64_t *ts)
{
    uint64_t lo = ts->u32[0];               /* 低 32 位先读 */
    uint64_t hi = ts->u32[1];
    return (hi << 32) | lo;
}

#endif /* MIPC_SHM_H */
