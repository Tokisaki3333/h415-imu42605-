#include "GPS.h"
#include "mipc_shm.h"
#define GPS_RX_BUFFER_SIZE 4096

/*
 * NMEA 校验和 = '$' 与 '*' 之间所有字节的异或（'$' 本身不参与）。
 * 第一级已判定前导头，各命令前导头（命令名5字节 + ','）的异或结果
 * 直接作为第二级校验和初值，写成常量表达式让编译器预先折叠，
 * 第二级只需从第一个数据字段起继续异或 —— 整条语句每个字节只读一次。
 */
#define GNRMC_HEADER_CS ('G' ^ 'N' ^ 'R' ^ 'M' ^ 'C' ^ ',')
#define GNGGA_HEADER_CS ('G' ^ 'N' ^ 'G' ^ 'G' ^ 'A' ^ ',')
#define GNGSA_HEADER_CS ('G' ^ 'N' ^ 'G' ^ 'S' ^ 'A' ^ ',')

#define GPS_FIELDS_MAX 24

volatile uint8_t RxBuffer2[GPS_RX_BUFFER_SIZE] = {0};   /* DMA 环形接收缓冲，段边界由长度精确控制 */
uint32_t gps_rx_read_idx = 0;

typedef enum {
    CMD_NONE = 0,
    CMD_RMC,
    CMD_GGA,
    CMD_GSA
} CmdId;

/* ================= 第二级：通用增量解析状态 =================
 * 语句可能被 DMA 环形缓冲回绕截断在任意位置（缓冲区末尾是常见截断点），
 * 这些状态跨 parse_gps_data 的多次分段调用保持，实现跨段续读。 */
typedef struct {
    uint8_t  st;               /* 0=DATA 1=CS1 2=CS2 3=DONE */
    uint8_t  cs;               /* 校验和（初值已含前半段硬编码异或结果） */
    uint8_t  csrx[2];          /* 收到的两个校验字符 */
    uint8_t  nf;               /* 当前数据字段编号（1 起） */
    uint32_t fabs[GPS_FIELDS_MAX]; /* 各字段在 RxBuffer2 中的绝对下标 */
    uint8_t  flen[GPS_FIELDS_MAX]; /* 各字段长度 */
} GpsParser;

static GpsParser rmc_p, gga_p, gsa_p;

typedef struct {
    const char *name;          /* 命令名（5 字符） */
    uint8_t     cs_init;       /* 命令头校验和初值 */
    GpsParser  *p;
} CmdDef;

static const CmdDef cmd_table[] = {
    { "GNRMC", GNRMC_HEADER_CS, &rmc_p },
    { "GNGGA", GNGGA_HEADER_CS, &gga_p },
    { "GNGSA", GNGSA_HEADER_CS, &gsa_p },
};
#define CMD_COUNT (sizeof(cmd_table) / sizeof(cmd_table[0]))

static CmdId    gps_cmd = CMD_NONE;   /* 当前活跃语句的命令（NONE=空闲） */
static uint64_t gps_clk  = 0;         /* 当前语句到达时间戳（跨段沿用首次识别时刻） */

/* 将十六进制字符转换为数值 */
static uint8_t hex2val(char c)
{
    if (c >= '0' && c <= '9') return (uint8_t)(c - '0');
    if (c >= 'A' && c <= 'F') return (uint8_t)(c - 'A' + 10);
    if (c >= 'a' && c <= 'f') return (uint8_t)(c - 'a' + 10);
    return 0;
}

/* 取绝对下标处的单个字节（越过缓冲区末尾自动回绕） */
static char buf_at(uint32_t abs_idx)
{
    if (abs_idx >= GPS_RX_BUFFER_SIZE) abs_idx -= GPS_RX_BUFFER_SIZE;
    return (char)RxBuffer2[abs_idx];
}

/* 新语句：重置第二级状态，第一个数据字段从本段起点开始 */
static void parser_reset(GpsParser *p, uint8_t cs_init, uint32_t first_abs)
{
    p->st = 0;
    p->cs = cs_init;
    p->nf = 1;
    memset(p->fabs, 0, sizeof(p->fabs));
    memset(p->flen, 0, sizeof(p->flen));
    p->fabs[1] = first_abs;
}

/* ================= GPS 共享区发布（V3F → V5F） =================
 * 每条校验通过的 RMC/GGA/GSA 语句发布一帧；某字段本帧缺失时对应 flags 位为 0、
 * 数值同时清零（显式空标志，判空一律查 flags 位，勿用值判空）。
 * GPS 各字段保持定点整数（lat/lon ×1e7 度、speed cm/s、hdop/pdop/vdop ×100、alt cm）：
 * 字串解析天然是整数，且经纬度用 float 精度不够；跨核浮点的取舍见 mipc_shm.h。 */

/* 提取字段文本到栈缓冲（字段短；按绝对下标跨回绕安全） */
static void field_copy(char *dst, uint8_t dst_sz, uint32_t fabs, uint8_t flen)
{
    uint8_t i;
    if (flen >= dst_sz) flen = dst_sz - 1;
    for (i = 0; i < flen; i++) dst[i] = buf_at(fabs + i);
    dst[flen] = '\0';
}

/* "整数[.小数]" → ×10^decimals（空/非法返回 0） */
static int32_t parse_fixed(const char *s, int decimals)
{
    int32_t v = 0;
    int d = 0;
    while (*s >= '0' && *s <= '9') { v = v * 10 + (*s - '0'); s++; }
    if (*s == '.') {
        s++;
        while (*s >= '0' && *s <= '9') { v = v * 10 + (*s - '0'); d++; s++; }
        while (d < decimals) { v *= 10; d++; }
    }
    while (d > decimals) { v /= 10; d--; }
    return v;
}

/* "ddmm.mmmmm"（度分）→ 度 ×1e7，无符号（方向由 N/S/E/W 字段决定） */
static int32_t latlon_to_e7(const char *s)
{
    int32_t ddmm = 0, m;
    int64_t frac = 0, p10 = 1;
    int fd = 0;
    while (*s >= '0' && *s <= '9') { ddmm = ddmm * 10 + (*s - '0'); s++; }
    m = ddmm % 100;                              /* 分 */
    if (*s == '.') {
        s++;
        while (*s >= '0' && *s <= '9') { frac = frac * 10 + (*s - '0'); fd++; s++; }
    }
    while (fd--) p10 *= 10;
    return (int32_t)((int64_t)(ddmm / 100) * 10000000LL
                   + (m * p10 + frac) * 10000000LL / (60LL * p10));
}

/* "x.y" 节 → cm/s（1 节 = 51.4444 cm/s） */
static uint32_t knots_to_cmps(const char *s)
{
    return (uint32_t)((int64_t)parse_fixed(s, 2) * 514444LL / 1000000LL);
}

/* RMC 通道发布：先清本通道字段（本帧缺失显式空），再填，最后 ts + cnt++ */
static void gps_publish_rmc(void)
{
    char t[16];
    /* 本帧缺失的字段先显式清空并清 flags 位（0 值本身合法，判空一律查 flags） */
    g_shm->gps_rmc.flags = 0;
    g_shm->gps_rmc.status = 0;      g_shm->gps_rmc.lat_e7 = 0; g_shm->gps_rmc.lon_e7 = 0;
    g_shm->gps_rmc.speed_cmps = 0;  g_shm->gps_rmc.date_ddmmyy = 0;

    if (rmc_p.flen[2] > 0) {
        g_shm->gps_rmc.flags |= SHM_RMC_STATUS;
        g_shm->gps_rmc.status = (uint8_t)buf_at(rmc_p.fabs[2]);
    }
    if (rmc_p.flen[3] > 0 && rmc_p.flen[5] > 0) {
        field_copy(t, sizeof t, rmc_p.fabs[3], rmc_p.flen[3]);
        int32_t lat = latlon_to_e7(t);
        if (rmc_p.flen[4] > 0 && buf_at(rmc_p.fabs[4]) == 'S') lat = -lat;
        field_copy(t, sizeof t, rmc_p.fabs[5], rmc_p.flen[5]);
        int32_t lon = latlon_to_e7(t);
        if (rmc_p.flen[6] > 0 && buf_at(rmc_p.fabs[6]) == 'W') lon = -lon;
        g_shm->gps_rmc.flags |= SHM_RMC_POS;
        g_shm->gps_rmc.lat_e7 = lat;
        g_shm->gps_rmc.lon_e7 = lon;
    }
    if (rmc_p.flen[7] > 0) {
        field_copy(t, sizeof t, rmc_p.fabs[7], rmc_p.flen[7]);
        uint32_t cmps = knots_to_cmps(t);
        g_shm->gps_rmc.flags |= SHM_RMC_SPEED;
        g_shm->gps_rmc.speed_cmps = (cmps > 65535u) ? 65535u : (uint16_t)cmps;   /* uint16 饱和 */
    }
    if (rmc_p.flen[9] > 0) {
        field_copy(t, sizeof t, rmc_p.fabs[9], rmc_p.flen[9]);
        g_shm->gps_rmc.flags |= SHM_RMC_DATE;
        g_shm->gps_rmc.date_ddmmyy = (uint32_t)parse_fixed(t, 0);
    }
    shm_chan_ts_write(&g_shm->gps_rmc.hdr, gps_clk);
    g_shm->gps_rmc.hdr.cnt++;
}

/* GGA 通道发布 */
static void gps_publish_gga(void)
{
    char t[16];
    g_shm->gps_gga.flags = 0;
    g_shm->gps_gga.quality = 0; g_shm->gps_gga.sv = 0;
    g_shm->gps_gga.hdop_x100 = 0; g_shm->gps_gga.alt_cm = 0;

    if (gga_p.flen[6] > 0) {
        g_shm->gps_gga.flags |= SHM_GGA_QUALITY;
        g_shm->gps_gga.quality = (uint8_t)(buf_at(gga_p.fabs[6]) - '0');   /* '0'-'6' → 数值 0-6 */
    }
    if (gga_p.flen[7] > 0) {
        field_copy(t, sizeof t, gga_p.fabs[7], gga_p.flen[7]);
        g_shm->gps_gga.flags |= SHM_GGA_SV;
        g_shm->gps_gga.sv = (uint8_t)parse_fixed(t, 0);
    }
    if (gga_p.flen[8] > 0) {
        field_copy(t, sizeof t, gga_p.fabs[8], gga_p.flen[8]);
        g_shm->gps_gga.flags |= SHM_GGA_HDOP;
        g_shm->gps_gga.hdop_x100 = (uint16_t)parse_fixed(t, 2);
    }
    if (gga_p.flen[9] > 0) {
        field_copy(t, sizeof t, gga_p.fabs[9], gga_p.flen[9]);
        g_shm->gps_gga.flags |= SHM_GGA_ALT;
        g_shm->gps_gga.alt_cm = parse_fixed(t, 2);
    }
    shm_chan_ts_write(&g_shm->gps_gga.hdr, gps_clk);
    g_shm->gps_gga.hdr.cnt++;
}

/* GSA 通道发布（各系统一条，最新一条覆盖） */
static void gps_publish_gsa(void)
{
    char t[16];
    g_shm->gps_gsa.flags = 0;

    if (gsa_p.flen[15] > 0 && gsa_p.flen[17] > 0) {
        field_copy(t, sizeof t, gsa_p.fabs[15], gsa_p.flen[15]);
        g_shm->gps_gsa.pdop_x100 = (uint16_t)parse_fixed(t, 2);
        field_copy(t, sizeof t, gsa_p.fabs[17], gsa_p.flen[17]);
        g_shm->gps_gsa.vdop_x100 = (uint16_t)parse_fixed(t, 2);
        g_shm->gps_gsa.flags |= SHM_GSA_DOP;
    }
    shm_chan_ts_write(&g_shm->gps_gsa.hdr, gps_clk);
    g_shm->gps_gsa.hdr.cnt++;
}

/* 按当前活跃命令发布对应语句帧 */
static void gps_publish_frame(void)
{
    switch (gps_cmd) {
        case CMD_RMC: gps_publish_rmc(); break;
        case CMD_GGA: gps_publish_gga(); break;
        case CMD_GSA: gps_publish_gsa(); break;
        default: break;
    }
}

/* ================= 第二级：通用语句解析（增量状态机） =================
 * 校验和初值 = 命令头异或常量（parser_reset 已设），只从逗号后的
 * 第一个数据字段继续读，字段记录与校验和单遍完成。
 * 返回本段已消费的字节数；若到段尾语句仍未到行尾，gps_cmd 保持非
 * CMD_NONE 等待下一段续读。
 */
static uint32_t parse_body(uint8_t *data, uint32_t len, uint32_t abs_base, GpsParser *p)
{
    uint32_t consumed = 0;
    uint32_t cur_abs  = abs_base;
    uint8_t  c;

    while (consumed < len) {
        c = data[consumed];
        switch (p->st) {
            case 0:                          /* DATA：数据字段 */
                if (c == ',') {
                    p->cs ^= c;
                    p->nf++;
                    if (p->nf < GPS_FIELDS_MAX) {
                        p->fabs[p->nf] = cur_abs + 1;
                        p->flen[p->nf] = 0;
                    }
                } else if (c == '*') {
                    p->st = 1;               /* '*' 不参与校验和 */
                } else if (c == '\r' || c == '\n') {
                    p->st = 3;               /* 语句残缺，丢弃 */
                } else {
                    p->cs ^= c;
                    if (p->nf < GPS_FIELDS_MAX) p->flen[p->nf]++;
                }
                break;

            case 1:                          /* CS1：校验和第一字符 */
                if (c == '\r' || c == '\n') {
                    p->st = 3;
                } else {
                    p->csrx[0] = c;
                    p->st = 2;
                }
                break;

            case 2:                          /* CS2：校验和第二字符，判定 */
                if (c == '\r' || c == '\n') {
                    p->st = 3;
                } else {
                    p->csrx[1] = c;
                    if ((uint8_t)((hex2val((char)p->csrx[0]) << 4) |
                                  hex2val((char)p->csrx[1])) == p->cs) {
                        /* 小核改用 mipc_v3_printf 转发串口后，GPS 不再直接 printf（同一 USART 冲突），打印停用 */
                        // print_parsed();
                        gps_publish_frame();
                    }
                    p->st = 3;
                }
                break;

            default:                         /* DONE：等行尾 */
                break;
        }
        consumed++;
        cur_abs++;

        if (p->st == 3 && (c == '\r' || c == '\n')) {
            /* 语句结束（行尾已消费），复位并交还第一级 */
            p->st    = 0;
            gps_cmd  = CMD_NONE;
            return consumed;
        }
    }

    /* 段尾：语句未完成，gps_cmd 保持非 NONE，等下一段续读 */
    return consumed;
}

/* ================= 第一级：语句分派 =================
 * WAIT_START   找 '$'
 * IN_HEADER    读命令头直到第一个 ','，查命令表判定命令类型
 * SKIP_LINE    未命中的命令整行弃用（跳到行尾）
 * 命中后立即分派给对应第二级，从逗号后继续读；
 * 前半段校验和初值由第二级按自身命令类型硬编码获得，无需传递。
 */
void parse_gps_data(uint8_t *data, uint32_t len, uint32_t abs_base, uint64_t clk)
{
    enum {
        WAIT_START = 0,
        IN_HEADER,
        SKIP_LINE
    };

    static uint8_t st      = WAIT_START;
    static uint8_t hdr[6];
    static uint8_t hdr_len = 0;

    uint32_t pos = 0;

    /* 上一条语句跨段未读完：整段交给对应第二级续读，读完再从剩余处继续 */
    if (gps_cmd != CMD_NONE) {
        uint32_t n = parse_body(data, len, abs_base, cmd_table[gps_cmd - 1].p);
        if (gps_cmd != CMD_NONE) return;
        pos = n;
    }

    while (pos < len) {
        uint8_t c = data[pos];

        switch (st) {
            case WAIT_START:
                if (c == '$') {
                    hdr_len = 0;
                    st      = IN_HEADER;
                }
                break;

            case IN_HEADER:
                if (c == ',') {
                    /* 命令头结束：查表命中则分派第二级，未命中整行弃用 */
                    if (hdr_len == 5) {
                        for (uint8_t i = 0; i < CMD_COUNT; i++) {
                            if (memcmp(hdr, cmd_table[i].name, 5) == 0) {
                                gps_cmd = (CmdId)(i + 1);
                                gps_clk = clk;
                                parser_reset(cmd_table[i].p, cmd_table[i].cs_init,
                                             abs_base + pos + 1);
                                uint32_t n = parse_body(data + pos + 1,
                                                        len - (pos + 1),
                                                        abs_base + pos + 1,
                                                        cmd_table[i].p);
                                pos += 1 + n;   /* 跳过逗号和整条语句（已消费到行尾） */
                                st = WAIT_START; /* 复位第一级，等下一行 '$' */
                                if (gps_cmd != CMD_NONE) return;
                                continue;
                            }
                        }
                    }
                    st = SKIP_LINE;
                } else if (c == '\r' || c == '\n') {
                    st = WAIT_START;            /* 残缺语句 */
                } else if (hdr_len < (uint8_t)sizeof(hdr)) {
                    hdr[hdr_len++] = c;
                } else {
                    st = SKIP_LINE;             /* 命令头过长，不可能是已登记命令 */
                }
                break;

            case SKIP_LINE:
                if (c == '\r' || c == '\n') st = WAIT_START;
                break;
        }
        pos++;
    }
}

/* 通过 USART3 TX 向模块发送一条指令（阻塞轮询，命令很短，安全）。
 * 指令需自带 NMEA 校验和，例如 10Hz：GPS_Send_Cmd("$PCAS02,100*1E\r\n"); */
void GPS_Send_Cmd(const char *cmd)
{
    while (*cmd) {
        while ((USART3->STATR & 0x80) == 0);   /* 等 TXE（发送寄存器空） */
        USART3->DATAR = (uint8_t)*cmd++;
    }
    while ((USART3->STATR & 0x40) == 0);       /* 等 TC（发送完成） */
}

void GPS_USART_Init()
{
    DMA_InitTypeDef DMA_InitStructure = {0};
    RCC_HBPeriphClockCmd(RCC_HBPeriph_DMA2, ENABLE);

    DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)(&USART3->DATAR);
    DMA_InitStructure.DMA_Memory0BaseAddr    = (u32)RxBuffer2;
    DMA_InitStructure.DMA_DIR                = DMA_DIR_PeripheralSRC;
    DMA_InitStructure.DMA_BufferSize         = GPS_RX_BUFFER_SIZE;
    DMA_InitStructure.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc          = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryDataSize     = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_Mode               = DMA_Mode_Circular;
    DMA_InitStructure.DMA_Priority           = DMA_Priority_VeryHigh;
    DMA_InitStructure.DMA_M2M                = DMA_M2M_Disable;
    DMA_Init(DMA2_Channel2, &DMA_InitStructure);

    GPIO_InitTypeDef  GPIO_InitStructure  = {0};
    USART_InitTypeDef USART_InitStructure = {0};

    RCC_HB1PeriphClockCmd(RCC_HB1Periph_USART3, ENABLE);
    RCC_HB2PeriphClockCmd(RCC_HB2Periph_GPIOB, ENABLE);

    // USART3_RX PB11(AF7)
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF7);
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    // USART3_TX PB10(AF7)
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF7);
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_Very_High;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    USART_InitStructure.USART_BaudRate            = 115200;
    USART_InitStructure.USART_WordLength          = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits            = USART_StopBits_1;
    USART_InitStructure.USART_Parity              = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode                = USART_Mode_Tx | USART_Mode_Rx;

    USART_Init(USART3, &USART_InitStructure);

    DMA_Cmd(DMA2_Channel2, ENABLE); /* USART3 Rx */

    USART_Cmd(USART3, ENABLE);

    DMA_MuxChannelConfig(DMA_MuxChannel10, 90);  /* USART3_RX */

    /* 试：定位更新频率设为 10Hz（100ms）。恢复 1Hz 用 "$PCAS02,1000*2E\r\n" */
    GPS_Send_Cmd("$PCAS02,100*1E\r\n");

    /* 语句输出：RMC/GGA/GSA 每帧，GSV/VTG/ZDA/GLL 关闭（省带宽） */
    GPS_Send_Cmd("$PCAS03,1,0,1,0,1,0,0,0,0,0,,,1,1,,,,0*33\r\n");
    // GPS_Send_Cmd("$PCAS03,1,1,1,1,1,1,1,1,1,1,0,0,,,1,1,,,,1*33\r\n");
}

void GPS_Check()
{
    /* 计算当前写指针位置（处理回绕） */
    uint32_t wr_idx = GPS_RX_BUFFER_SIZE - DMA2_Channel2->CNTR;
    if (wr_idx == GPS_RX_BUFFER_SIZE) wr_idx = 0;

    if (wr_idx != gps_rx_read_idx) {
        uint64_t clk = GetTime64_10Ns();   /* 语句到达时刻：与全传感器统一 10 ns 计数 */
        if (wr_idx > gps_rx_read_idx) {
            /* 数据连续：按精确长度处理，不写哨兵（DMA 会覆盖哨兵导致误读旧数据） */
            parse_gps_data((uint8_t*)&RxBuffer2[gps_rx_read_idx],
                           wr_idx - gps_rx_read_idx,
                           gps_rx_read_idx, clk);
        } else {
            /* 数据回绕：分两段处理 */
            uint32_t len1 = GPS_RX_BUFFER_SIZE - gps_rx_read_idx;
            if (len1 > 0) {
                parse_gps_data((uint8_t*)&RxBuffer2[gps_rx_read_idx],
                               len1, gps_rx_read_idx, clk);
            }
            if (wr_idx > 0) {
                parse_gps_data((uint8_t*)&RxBuffer2[0],
                               wr_idx, 0, clk);
            }
        }
        gps_rx_read_idx = wr_idx;  /* 更新读索引 */
    }
}
