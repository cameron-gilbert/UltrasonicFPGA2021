/*
 * imu_meta_layout.h - IMU/sensor metadata carried inside mic_packet_t.reserved[56].
 *
 * Single source of truth for BOTH ends of the link:
 *   - Board firmware  (UltrasonicFPGA2021/delay-and-sum-v2 : data_channel.c/build_mic_packet)
 *   - Host Qt app     (Ultrasonic-Beamforming-real-time-cpp : MicrophonePacket / parser)
 *
 * The mic packet header is unchanged (still 70 bytes); this struct simply
 * overlays the existing reserved[56] region, so packet size and the mic data
 * path are untouched. A frame with no fresh IMU sample sets marker=0/valid=0.
 *
 * Byte order: BIG-ENDIAN, matching the htonl/htons convention already used for
 * the header fields (sw_frame_id, mic_id, hw_frame_id, signature). Encode with
 * htons/htonl on the board; decode with qFromBigEndian / ntoh on the host.
 *
 * Keep this file byte-identical to packet_design/imu_meta_layout.h.
 */
#pragma once

#include <stdint.h>

/*
 * Field units / encoding:
 *   quat_i/j/k/real - orientation quaternion, unitless, range [-1,+1], Q14 (raw = q*16384).
 *                     BNO085 reports Q14 natively, so this packing is lossless.
 *   accuracy        - fusion confidence 0..3 (0 unreliable, 3 high).
 *   timestamp_us    - board Global Timer at IMU read, MICROSECONDS (drives latency comp).
 *   seq             - sample counter (unitless), detects stale/dropped IMU samples.
 */
#define IMU_META_MARKER  0x494D    /* 'IM' - reserved block carries valid metadata */
#define IMU_QUAT_Q       14        /* BNO085 rotation-vector fixed point: raw = q * 2^14 */
#define IMU_QUAT_SCALE   16384.0f  /* 2^IMU_QUAT_Q */

typedef struct {
    uint16_t marker;        /* IMU_META_MARKER when populated, else 0                */
    uint8_t  valid;         /* 1 = fresh IMU sample stamped into this frame          */
    uint8_t  accuracy;      /* BNO085 report status / calibration accuracy (0..3)    */
    int16_t  quat_i;        /* rotation-vector quaternion, Q14                       */
    int16_t  quat_j;
    int16_t  quat_k;
    int16_t  quat_real;
    uint32_t timestamp_us;  /* board Global Timer (XTime) microseconds at IMU read   */
    uint16_t seq;           /* IMU sample counter (wraps) - detects stale/dropped    */
    uint8_t  reserved[38];  /* future: GNSS position/velocity, barometer, etc.       */
} __attribute__((packed)) imu_meta_t;   /* MUST be exactly 56 bytes (== reserved[]) */

#if defined(__cplusplus)
static_assert(sizeof(imu_meta_t) == 56, "imu_meta_t must fill reserved[56]");
#else
_Static_assert(sizeof(imu_meta_t) == 56, "imu_meta_t must fill reserved[56]");
#endif
