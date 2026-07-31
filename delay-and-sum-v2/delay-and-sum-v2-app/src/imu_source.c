/*
 * imu_source.c - see imu_source.h.
 */
#include <string.h>
#include <math.h>

#include "imu_source.h"
#include "xparameters.h"
#include "lwip/def.h"   /* htons, htonl */
#if defined (__arm__) || defined (__aarch64__)
#include "xtime_l.h"    /* XTime, XTime_GetTime */
#endif

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* Global Timer runs at CPU clock / 2. */
#define IMU_GTIMER_HZ  (XPAR_CPU_CORTEXA9_0_CPU_CLK_FREQ_HZ / 2)

/* Latest published sample (host byte order). */
static struct {
    int16_t quat_i, quat_j, quat_k, quat_real;
    u8      accuracy;
    u32     timestamp_us;
    u16     seq;
    u8      valid;
} g_imu;

void imu_source_update(int16_t qi, int16_t qj, int16_t qk, int16_t qr,
                       u8 accuracy, u32 timestamp_us) {
    g_imu.quat_i       = qi;
    g_imu.quat_j       = qj;
    g_imu.quat_k       = qk;
    g_imu.quat_real    = qr;
    g_imu.accuracy     = accuracy;
    g_imu.timestamp_us = timestamp_us;
    g_imu.seq++;
    g_imu.valid = 1;
}

void imu_stamp_reserved(u8 *reserved) {
    imu_meta_t m;
    memset(&m, 0, sizeof(m));
    if (g_imu.valid) {
        m.marker       = htons(IMU_META_MARKER);
        m.valid        = 1;
        m.accuracy     = g_imu.accuracy;
        m.quat_i       = (int16_t)htons((u16)g_imu.quat_i);
        m.quat_j       = (int16_t)htons((u16)g_imu.quat_j);
        m.quat_k       = (int16_t)htons((u16)g_imu.quat_k);
        m.quat_real    = (int16_t)htons((u16)g_imu.quat_real);
        m.timestamp_us = htonl(g_imu.timestamp_us);
        m.seq          = htons(g_imu.seq);
    }
    memcpy(reserved, &m, sizeof(m));   /* reserved may be unaligned; copy bytes */
}

void imu_source_tick_synthetic(void) {
    static float yaw = 0.0f;
    yaw += 0.01f;                              /* ~slow rotation about Z per frame */
    if (yaw > 2.0f * (float)M_PI) yaw -= 2.0f * (float)M_PI;

    float half = yaw * 0.5f;
    int16_t qr = (int16_t)(cosf(half) * IMU_QUAT_SCALE);   /* real  */
    int16_t qk = (int16_t)(sinf(half) * IMU_QUAT_SCALE);   /* k (yaw) */

    u32 us = 0;
#if defined (__arm__) || defined (__aarch64__)
    XTime t;
    XTime_GetTime(&t);
    us = (u32)(t / (IMU_GTIMER_HZ / 1000000ULL));
#endif
    imu_source_update(0, 0, qk, qr, 3 /* accuracy: high */, us);
}
