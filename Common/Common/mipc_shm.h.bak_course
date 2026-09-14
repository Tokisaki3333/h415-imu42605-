#ifndef MIPC_SHM_H
#define MIPC_SHM_H

#include <stdint.h>

/* =====================================================================
 * 双核共享区定义（V3F 小核定义实例，V5F 大核经 IPC 基址映射同一地址）
 * 两端必须 include 同一份本头文件，保证结构体布局完全一致。
 *
 * 发布-订阅协议（每通道独立）：
 *   写端：写数据/flags → 写 ts 高 32 → 写 ts 低 32 → __sync_synchronize() → cnt++
 *   读端：cur = cnt; if (cur != last) { __sync_synchronize(); last = cur; 读 ts 低→高; 读数据; }
 *   ts 为 DRDY（数据就绪）时刻，不是读完时刻；单位统一 10 ns 计数（GetTime64_10Ns()）。
 *
 * flags：每通道一个 uint8_t 位掩码，标明本帧哪些字段有数据（位 0 = 缺失、数值同时清零）。
 *   ★ 0 值本身可能合法（速度 0、经度 0、HDOP 0、海拔 0），判空必须查位，不要用值判空。
 *
 * 数值表示：能用 float 精确表达的物理量直接传 float（两核均按 -mabi=ilp32f 编译，硬浮点）；
 *   只有 float 精度不够时才保留定点，并逐字段注明定标（本文件的 ×1e7 度 / cm / ×100
 *   都是十进制定标整数，不是 Qm.n 二进制定点）。
 * ===================================================================== */

/* 通道头：12 B，4 字节对齐（全结构体因此无 8 字节对齐空洞） */
typedef struct {
    volatile uint32_t cnt;      /* 帧序号（发布标记，最后写） */
    volatile uint32_t ts_lo;    /* 时间戳低 32 位（最后写 / 最先读） */
    volatile uint32_t ts_hi;    /* 时间戳高 32 位 */
} chan_hdr_t;

/* ---------------- flags 位定义（每通道一个 uint8_t） ---------------- */
#define SHM_RMC_STATUS   0x01u   /* RMC 状态字段存在 */
#define SHM_RMC_POS      0x02u   /* 经纬度存在 */
#define SHM_RMC_SPEED    0x04u   /* 对地速度存在 */
#define SHM_RMC_DATE     0x08u   /* 日期存在 */
#define SHM_GGA_QUALITY  0x01u   /* 定位质量存在 */
#define SHM_GGA_SV       0x02u   /* 卫星数存在 */
#define SHM_GGA_HDOP     0x04u   /* HDOP 存在 */
#define SHM_GGA_ALT      0x08u   /* 海拔存在 */
#define SHM_GSA_DOP      0x01u   /* PDOP/VDOP 存在 */

/* ---- 陀螺仪（ICM-42605）：只有 DRDY 时间戳；六轴原始值走 SPI_rx DMA 帧 ---- */
typedef struct {
    chan_hdr_t hdr;
} gyro_chan_t;                          /* 12 B */

/* ---- IST8310 磁力计：原始 LSB 直通（0.3 μT/LSB，16 位补码） ---- */
typedef struct {
    chan_hdr_t hdr;                     /* 12 B */
    volatile int16_t mx, my, mz;
} ist_chan_t;                           /* 20 B（含尾部 2 B 对齐填充） */

/* ---- BMP388 气压计：float 直通 ---- */
typedef struct {
    chan_hdr_t hdr;                     /* 12 B */
    volatile float temp_celsius;        /* ℃ */
    volatile float press_pascal;        /* Pa */
} bmp_chan_t;                           /* 20 B */

/* ---- GPS RMC：定位 / 速度 / 日期 ----
 * 三态判读：无 SHM_RMC_STATUS → 未启动/无语句；status=='V' → 有信号无定位；'A' → 定位有效。 */
typedef struct {
    chan_hdr_t hdr;                     /* 12 B */
    volatile uint8_t  flags;
    volatile uint8_t  status;           /* NMEA 原字符：'A'=有效 / 'V'=无效 */
    volatile uint16_t _rsv0;            /* 对齐填充 */
    volatile float    speed_mps;        /* 对地速度 m/s（解析自 RMC 的节值 ×0.5144444） */
    volatile int32_t  lat_e7;           /* 纬度 ×1e7 度（1 LSB = 10^-7°，南纬为负）；float 精度不够，必须定点 */
    volatile int32_t  lon_e7;           /* 经度 ×1e7 度（西经为负） */
    volatile uint32_t date_ddmmyy;      /* UTC 日期 ddmmyy（十进制字面量） */
} gps_rmc_chan_t;                       /* 32 B，无空洞 */

/* ---- GPS GGA：定位质量 / 卫星数 / HDOP / 海拔 ---- */
typedef struct {
    chan_hdr_t hdr;                     /* 12 B */
    volatile uint16_t hdop_x100;        /* HDOP ×100（1 LSB = 0.01） */
    volatile uint8_t  flags;
    volatile uint8_t  quality;          /* 0=无效 1=单点 2=差分 4=RTK固定 5=RTK浮点 */
    volatile int32_t  alt_cm;           /* 海拔 cm */
    volatile uint8_t  sv;               /* 卫星数 */
    volatile uint8_t  _rsv[3];          /* 对齐填充 */
} gps_gga_chan_t;                       /* 24 B */

/* ---- GPS GSA：PDOP / VDOP（各系统 GSA 独立计数，最新一条覆盖） ---- */
typedef struct {
    chan_hdr_t hdr;                     /* 12 B */
    volatile uint8_t  flags;
    volatile uint8_t  _rsv0;
    volatile uint16_t pdop_x100;        /* ×100 */
    volatile uint16_t vdop_x100;        /* ×100 */
    volatile uint16_t _rsv1;
} gps_gsa_chan_t;                       /* 20 B */

/* ---- 低频文本通道（mipc_v3_printf，每秒一帧） ---- */
typedef struct {
    volatile uint32_t cnt;
    volatile uint8_t  data_buf[256];
} log_chan_t;                           /* 260 B */

/* ---- 整体共享区 ---- */
typedef struct {
    volatile uint32_t v5f_progress;     /* V5F 握手进度 1..6 */
    volatile uint32_t v3f_cfg_done;     /* IMU 配置完成信号 */
    log_chan_t        log;
    gyro_chan_t       gyro;
    ist_chan_t        ist;
    bmp_chan_t        bmp;
    gps_rmc_chan_t    gps_rmc;
    gps_gga_chan_t    gps_gga;
    gps_gsa_chan_t    gps_gsa;
    /* OLED 显存：128×64/8 = 1024 B，页主序 8 页 × 128 列；双核可读写，aligned(4) 便于双核/DMA 访问 */
    volatile uint8_t  oled_fb[1024] __attribute__((aligned(4)));
} shared_mem_t;

/* 共享区指针：V3F 定义 storage 并由 g_shm 指向（地址经 IPC_MSG0 传给 V5F）；V5F 同样以 g_shm 命名 */
extern volatile shared_mem_t *g_shm;

/* ---------------- 写端（小核）---------------- */

/* 写通道时间戳：高 32 位先写、低 32 位最后写，屏障保证数据/ts 先于 cnt++ 可见 */
static inline void shm_chan_ts_write(volatile chan_hdr_t *h, uint64_t ts)
{
    h->ts_hi = (uint32_t)(ts >> 32);
    h->ts_lo = (uint32_t)ts;
    __sync_synchronize();
}

static inline void shm_publish_gyro(uint64_t ts_drdy)
{
    shm_chan_ts_write(&g_shm->gyro.hdr, ts_drdy);
    g_shm->gyro.hdr.cnt++;
}

static inline void shm_publish_ist(uint64_t ts_drdy, int16_t mx, int16_t my, int16_t mz)
{
    g_shm->ist.mx = mx;
    g_shm->ist.my = my;
    g_shm->ist.mz = mz;
    shm_chan_ts_write(&g_shm->ist.hdr, ts_drdy);
    g_shm->ist.hdr.cnt++;
}

/* BMP388：温度/气压直接以 float 传递（无定点往返） */
static inline void shm_publish_bmp(uint64_t ts_drdy, float temp_celsius, float press_pascal)
{
    g_shm->bmp.temp_celsius = temp_celsius;
    g_shm->bmp.press_pascal = press_pascal;
    shm_chan_ts_write(&g_shm->bmp.hdr, ts_drdy);
    g_shm->bmp.hdr.cnt++;
}

/* ---------------- 读端（大核）---------------- */

/* 读通道时间戳：低 32 位先读、再读高 32 位；必须在"检测到 cnt 变化 + 屏障"之后调用 */
static inline uint64_t shm_chan_ts_read(const volatile chan_hdr_t *h)
{
    uint64_t lo = h->ts_lo;
    uint64_t hi = h->ts_hi;
    return (hi << 32) | lo;
}

#endif /* MIPC_SHM_H */
