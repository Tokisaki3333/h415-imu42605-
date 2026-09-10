#ifndef MIPC_SHM_H
#define MIPC_SHM_H

#include <stdint.h>

/* =====================================================================
 * 双核共享区定义（V3F 小核定义实例，V5F 大核经 IPC 基址映射同一地址）
 * 两端必须 include 同一份本头文件，保证结构体布局完全一致。
 *
 * ------------------------------ 发布-订阅协议（每通道独立） ------------------------------
 *   写端（小核）：写数据/标志 → 写 ts 高 32 → 写 ts 低 32 → __sync_synchronize() → cnt++
 *   读端（大核）：cur = cnt; if (cur != last) { __sync_synchronize(); last = cur; 读 ts 低→高; 读数据; }
 *   时间戳是 DRDY（数据就绪）时刻，不是读完数据的时刻；单位统一为 10 ns 计数
 *   （1 tick = 10 ns，由 GetTime64_10Ns() 取，64 位按高/低 32 位分两次搬运）。
 *
 * ------------------------------ 数据有效性（务必遵守） ------------------------------
 *   每通道用一个 uint8_t flags 位掩码标明"本帧哪些字段有数据"：位为 0 = 该字段本帧缺失，
 *   其数值同时被写 0。★ 0 值本身可能合法（速度 0、经纬度 0、HDOP 0、海拔 0），
 *   判空必须查 flags 位，不要用值判空。
 *
 * ------------------------------ 数值表示约定 ------------------------------
 *   1) 能用 float 精确表达的物理量直接传 float（两核均按 -mabi=ilp32f 编译，带单精度 FPU，
 *      无软浮点开销），避免"发送端 float→定点、接收端定点→float"的往返与量化。
 *   2) 只有 float 精度不够时才保留定点，并逐字段注明定标（学名）：
 *        Qm.n        二进制定点（m 位含符号整数部分 + n 位小数），如 Q1.31 = int32 / 2^31
 *        ×1e7 度     十进制定标整数（scaled fixed-point，不是 Q 格式），1 LSB = 10^-7°
 *        cm / cm·s⁻¹ 十进制定标整数，1 LSB = 0.01 m / 0.01 m·s⁻¹
 *        ×100        ×100 十进制定标整数，1 LSB = 0.01
 *      本工程目前没有 Qm.n 形式的字段。
 *
 * ------------------------------ v2 布局说明 ------------------------------
 *   通道头改为 3×uint32（12 B，4 字节对齐），取代旧版内含"从未使用的 uint64_t 成员"的
 *   ts64_t —— 旧版把每通道对齐要求抬到 8 字节，在 gps_rmc(11 B)/gga(4 B)/gsa(3 B) 等
 *   通道留下共 24 字节空洞；现在全结构体仅剩各通道尾部 ≤3 字节的对齐填充。
 * ===================================================================== */

/* 通道头：12 字节、4 字节对齐（全结构体因此无 8 字节对齐空洞） */
typedef struct {
    volatile uint32_t cnt;      /* 帧序号（发布标记，最后写） */
    volatile uint32_t ts_lo;    /* 时间戳低 32 位（最后写 / 最先读） */
    volatile uint32_t ts_hi;    /* 时间戳高 32 位 */
} chan_hdr_t;

/* ---------------- 各通道 flags 位定义（每通道一个 uint8_t） ---------------- */
/* RMC：状态字段 / 经纬度 / 对地速度 / 日期 */
#define SHM_RMC_STATUS   0x01u
#define SHM_RMC_POS      0x02u
#define SHM_RMC_SPEED    0x04u
#define SHM_RMC_DATE     0x08u
/* GGA：定位质量 / 卫星数 / HDOP / 海拔 */
#define SHM_GGA_QUALITY  0x01u
#define SHM_GGA_SV       0x02u
#define SHM_GGA_HDOP     0x04u
#define SHM_GGA_ALT      0x08u
/* GSA：PDOP/VDOP */
#define SHM_GSA_DOP      0x01u

/* ---- 陀螺仪（ICM-42605）通道：2 kHz DRDY 中断只共享时间戳；六轴原始值走 SPI_rx DMA 帧 ---- */
typedef struct {
    chan_hdr_t hdr;                     /* 12 B */
} gyro_chan_t;

/* ---- IST8310 磁力计：原始 LSB 直通（0.3 μT/LSB，16 位补码） ---- */
typedef struct {
    chan_hdr_t hdr;                     /* 12 B */
    volatile int16_t mx, my, mz;        /* 18 B 载荷 → 尾部 2 B 对齐填充（已无更多字段可放） */
} ist_chan_t;                           /* 20 B */

/* ---- BMP388 气压计（float 直通，无定点往返） ---- */
typedef struct {
    chan_hdr_t hdr;                     /* 12 B */
    volatile float temp_celsius;        /* 温度 ℃ */
    volatile float press_pascal;        /* 气压 Pa */
} bmp_chan_t;                           /* 20 B */

/* ---- GPS RMC：定位 / 速度 / 日期 ----
 * 三态判读（用 flags + status，可区分"GPS 未启动"与"有信号但无定位"）：
 *   (flags & SHM_RMC_STATUS) == 0        → 从未收到带状态字段的 RMC：GPS 未启动 / 未接 / 波特率不符
 *   (flags & SHM_RMC_STATUS) && 'V'      → 已收到语句但定位无效（有信号、无定位）
 *   status == 'A'                        → 定位有效
 * status 原样保存 NMEA 的 ASCII 字符（'A'/'V'），不改成枚举以保持无损。 */
typedef struct {
    chan_hdr_t hdr;                     /* 12 B */
    volatile uint8_t  flags;            /* SHM_RMC_* 位掩码 */
    volatile uint8_t  status;           /* 'A'=有效 / 'V'=无效（ASCII 原样） */
    volatile uint16_t speed_cmps;       /* 对地速度 cm/s（十进制定标整数，1 LSB = 0.01 m/s；
                                           uint16 足够：65535 cm/s = 655 m/s ≈ 1273 节） */
    volatile int32_t  lat_e7;           /* 纬度 ×1e7 度（1 LSB = 10^-7°，南纬为负）；不可改 float：
                                           float 仅约 7.2 位十进制有效数字，±180° 内分辨不出 10^-7°(≈1.1 cm) */
    volatile int32_t  lon_e7;           /* 经度 ×1e7 度（西经为负） */
    volatile uint32_t date_ddmmyy;      /* UTC 日期 ddmmyy（十进制字面量，非时间戳） */
} gps_rmc_chan_t;                       /* 28 B，0 空洞 */

/* ---- GPS GGA：定位质量 / 卫星数 / HDOP / 海拔 ---- */
typedef struct {
    chan_hdr_t hdr;                     /* 12 B */
    volatile uint16_t hdop_x100;        /* 水平精度因子 ×100（1 LSB = 0.01） */
    volatile uint8_t  flags;            /* SHM_GGA_* 位掩码 */
    volatile uint8_t  quality;          /* 0=无效 1=单点 2=差分 4=RTK固定 5=RTK浮点 */
    volatile int32_t  alt_cm;           /* 海拔 cm（1 LSB = 0.01 m） */
    volatile uint8_t  sv;               /* 使用卫星数 */
    volatile uint8_t  _rsv[3];          /* 尾部 3 B 对齐填充 */
} gps_gga_chan_t;                       /* 24 B */

/* ---- GPS GSA：PDOP / VDOP（各系统 GSA 独立计数，最新一条覆盖） ---- */
typedef struct {
    chan_hdr_t hdr;                     /* 12 B */
    volatile uint8_t  flags;            /* SHM_GSA_DOP */
    volatile uint8_t  _rsv0;
    volatile uint16_t pdop_x100;        /* ×100（1 LSB = 0.01） */
    volatile uint16_t vdop_x100;        /* ×100（1 LSB = 0.01） */
    volatile uint16_t _rsv1;            /* 尾部 3 B（含 _rsv0）对齐填充 */
} gps_gsa_chan_t;                       /* 20 B */

/* ---- 低频文本通道（mipc_v3_printf，每秒一帧） ---- */
typedef struct {
    volatile uint32_t cnt;
    volatile uint8_t  data_buf[256];
} log_chan_t;                           /* 260 B，4 字节对齐无需填充 */

/* ---- 整体共享区（v2：1464 → 1416 字节，空洞 24 → 8 字节） ---- */
typedef struct {
    volatile uint32_t v5f_progress;     /* 原 channel0：V5F 握手进度 1..6 */
    volatile uint32_t v3f_cfg_done;     /* 原 channel2：IMU 配置完成信号 */
    log_chan_t        log;              /* 原 channel4：低频文本 */
    gyro_chan_t       gyro;             /* 陀螺仪 DRDY 时间戳 */
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

/* 写通道时间戳（本工程统一 10 ns 计数）：高 32 位先写、低 32 位最后写，随后发布屏障。
 * 屏障保证"数据/ts 都在 cnt++ 之前全局可见"（读端以 acquire 屏障配对）。 */
static inline void shm_chan_ts_write(volatile chan_hdr_t *h, uint64_t ts)
{
    h->ts_hi = (uint32_t)(ts >> 32);        /* 高 32 位先写 */
    h->ts_lo = (uint32_t)ts;                /* 低 32 位最后写 */
    __sync_synchronize();
}

/* 陀螺仪：无数据字段，ts → cnt++ */
static inline void shm_publish_gyro(uint64_t ts_drdy)
{
    shm_chan_ts_write(&g_shm->gyro.hdr, ts_drdy);
    g_shm->gyro.hdr.cnt++;
}

/* IST8310：先写数据，再写 ts，最后 cnt++ */
static inline void shm_publish_ist(uint64_t ts_drdy, int16_t mx, int16_t my, int16_t mz)
{
    g_shm->ist.mx = mx;
    g_shm->ist.my = my;
    g_shm->ist.mz = mz;
    shm_chan_ts_write(&g_shm->ist.hdr, ts_drdy);
    g_shm->ist.hdr.cnt++;
}

/* BMP388：温度/气压直接以 float 传递（两核都有硬浮点，无需 ×1000 定点往返） */
static inline void shm_publish_bmp(uint64_t ts_drdy, float temp_celsius, float press_pascal)
{
    g_shm->bmp.temp_celsius  = temp_celsius;
    g_shm->bmp.press_pascal  = press_pascal;
    shm_chan_ts_write(&g_shm->bmp.hdr, ts_drdy);
    g_shm->bmp.hdr.cnt++;
}

/* ---------------- 读端（大核）---------------- */

/* 读通道时间戳：低 32 位先读、再读高 32 位（与写端顺序配对）。
 * 注意：必须在"检测到 cnt 变化 + __sync_synchronize()"之后调用，否则可能读到写入中途的值。 */
static inline uint64_t shm_chan_ts_read(const volatile chan_hdr_t *h)
{
    uint64_t lo = h->ts_lo;                 /* 低 32 位先读 */
    uint64_t hi = h->ts_hi;
    return (hi << 32) | lo;
}

#endif /* MIPC_SHM_H */
