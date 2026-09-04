#include "observer.h"
#include "mipc_shm.h"     /* g_shm / shm_ts_read */

/* ==================== 观察器实例 ==================== */
/* q0=1 单位四元数初值（跳过校准时直接可用）；calib_done=1 使 main 不再等待校准完成 */
volatile v5f_observer_t g_v5f_obs = { .q0 = 1.0f, .calib_done = 1 };

/* ==================== 内部轮询状态（cnt 变化检测） ==================== */
static uint32_t s_last_log_cnt = 0, s_last_gyro_cnt = 0, s_last_ist_cnt = 0, s_last_bmp_cnt = 0;
static uint32_t s_last_gps_rmc_cnt = 0, s_last_gps_gga_cnt = 0, s_last_gps_gsa_cnt = 0;
static uint64_t s_last_gyro_ts = 0;

/* ==================== 各通道轮询 + 数据拷贝 + 延迟统计 ==================== */
uint32_t observer_poll(uint64_t now_us)
{
    uint32_t dt_us = 0;
    uint32_t c;

    /* 低频文本 */
    c = g_shm->log.cnt;
    if (c != s_last_log_cnt) { s_last_log_cnt = c; g_v5f_obs.log_pending = 1; }

    /* 陀螺：DRDY 时间戳差分 → dt_us（供姿态积分）；cnt 轮询 → pending/stat/lat */
    {
        uint64_t gyro_ts = shm_ts_read(&g_shm->gyro.ts_drdy_us);
        if (gyro_ts != 0) {
            if (gyro_ts == s_last_gyro_ts) g_v5f_obs.gyro_err++;
            else {
                if (s_last_gyro_ts != 0 && gyro_ts > s_last_gyro_ts)
                    dt_us = (uint32_t)(gyro_ts - s_last_gyro_ts);
                s_last_gyro_ts = gyro_ts;
            }
        }
        c = g_shm->gyro.cnt;
        if (c != s_last_gyro_cnt) {
            s_last_gyro_cnt = c;
            g_v5f_obs.gyro_pending = 1;
            g_v5f_obs.stat_gyro++;
            if (gyro_ts != 0) g_v5f_obs.gyro_lat_us = (uint32_t)(now_us - (gyro_ts / 100)); /* gyro ts 为 10Ns，/100 对齐其中 now µs 语义 */
        /* ts 与 now 均为 10Ns 计数；/100 得 ?s（原 μs 语义保留） */
        }
    }

    /* IST8310（磁力计） */
    c = g_shm->ist.cnt;
    if (c != s_last_ist_cnt) {
        s_last_ist_cnt = c;
        g_v5f_obs.ist_pending = 1;
        g_v5f_obs.stat_ist++;
        uint64_t ist_ts = shm_ts_read(&g_shm->ist.ts_drdy_us);
        if (ist_ts != 0) g_v5f_obs.ist_lat_us = (uint32_t)(now_us - ist_ts);
        g_v5f_obs.mag_x = g_shm->ist.mx;
        g_v5f_obs.mag_y = g_shm->ist.my;
        g_v5f_obs.mag_z = g_shm->ist.mz;
    }

    /* BMP388（气压计）：定点直通 */
    c = g_shm->bmp.cnt;
    if (c != s_last_bmp_cnt) {
        s_last_bmp_cnt = c;
        g_v5f_obs.bmp_pending = 1;
        g_v5f_obs.stat_bmp++;
        uint64_t bmp_ts = shm_ts_read(&g_shm->bmp.ts_drdy_us);
        if (bmp_ts != 0) g_v5f_obs.bmp_lat_us = (uint32_t)(now_us - bmp_ts);
        g_v5f_obs.bmp_temp_x1000  = g_shm->bmp.temp_x1000;
        g_v5f_obs.bmp_press_x1000 = g_shm->bmp.press_x1000;
    }

    /* GPS RMC */
    c = g_shm->gps_rmc.cnt;
    if (c != s_last_gps_rmc_cnt) {
        s_last_gps_rmc_cnt = c;
        g_v5f_obs.gps_rmc_pending = 1;
        g_v5f_obs.stat_gps_rmc++;
        uint64_t gps_ts = shm_ts_read(&g_shm->gps_rmc.ts_drdy_us);
        if (gps_ts != 0) g_v5f_obs.gps_rmc_lat_us = (uint32_t)(now_us - gps_ts);
        g_v5f_obs.lat_e7     = g_shm->gps_rmc.lat_e7;
        g_v5f_obs.lon_e7     = g_shm->gps_rmc.lon_e7;
        g_v5f_obs.speed_cmps = g_shm->gps_rmc.speed_cmps;
    }

    /* GPS GGA */
    c = g_shm->gps_gga.cnt;
    if (c != s_last_gps_gga_cnt) {
        s_last_gps_gga_cnt = c;
        g_v5f_obs.gps_gga_pending = 1;
        g_v5f_obs.stat_gps_gga++;
        uint64_t gps_ts = shm_ts_read(&g_shm->gps_gga.ts_drdy_us);
        if (gps_ts != 0) g_v5f_obs.gps_gga_lat_us = (uint32_t)(now_us - gps_ts);
        g_v5f_obs.alt_cm     = g_shm->gps_gga.alt_cm;
        g_v5f_obs.fix_quality = g_shm->gps_gga.quality;
        g_v5f_obs.sat_num     = g_shm->gps_gga.sv;
        g_v5f_obs.hdop_x100   = g_shm->gps_gga.hdop_x100;
    }

    /* GPS GSA */
    c = g_shm->gps_gsa.cnt;
    if (c != s_last_gps_gsa_cnt) {
        s_last_gps_gsa_cnt = c;
        g_v5f_obs.gps_gsa_pending = 1;
        g_v5f_obs.stat_gps_gsa++;
        uint64_t gps_ts = shm_ts_read(&g_shm->gps_gsa.ts_drdy_us);
        if (gps_ts != 0) g_v5f_obs.gps_gsa_lat_us = (uint32_t)(now_us - gps_ts);
    }

    return dt_us;
}

/* 陀螺温度（SPI 0x1D/0x1E 原始值）→ ℃ */
void observer_update_gyro_temp(int16_t temp_raw)
{
    g_v5f_obs.gyro_temp_celsius = temp_raw / 132.48f + 25.0f;
}

/* 加速度计（SPI 0x1F~0x24 原始值，大端）→ 观察器原始 LSB */
void observer_update_accel(int16_t ax, int16_t ay, int16_t az)
{
    g_v5f_obs.accel_x = ax;
    g_v5f_obs.accel_y = ay;
    g_v5f_obs.accel_z = az;
}
