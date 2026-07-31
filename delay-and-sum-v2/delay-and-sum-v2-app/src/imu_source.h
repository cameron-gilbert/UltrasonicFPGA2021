/*
 * imu_source.h - Latest IMU sample store + packet stamping.
 *
 * Decouples the sensor driver from the packet path: whoever reads the BNO085
 * calls imu_source_update(); build_mic_packet() calls imu_stamp_reserved() to
 * write the latest sample into the packet's reserved[56] region (big-endian).
 *
 * Until the BNO085 I2C0 driver (J12 / MIO10-11) is wired up, a synthetic
 * rotating-yaw source can stand in so the full path is demonstrable on the
 * existing bitstream with no PL rebuild. Set IMU_USE_SYNTHETIC to 0 once the
 * real sensor is publishing samples.
 */
#pragma once

#include "xil_types.h"
#include "imu_meta_layout.h"

/* 1 = feed a synthetic rotating-yaw quaternion (stand-in for the BNO085). */
#define IMU_USE_SYNTHETIC 1

/* Publish a new IMU sample (host byte order, Q14 quaternion). Called by the
 * sensor driver, or by imu_source_tick_synthetic() while standing in. */
void imu_source_update(int16_t qi, int16_t qj, int16_t qk, int16_t qr,
                       u8 accuracy, u32 timestamp_us);

/* Write the latest sample into reserved[56] as big-endian imu_meta_t.
 * If no sample has been published, writes all-zero (marker=0/valid=0). */
void imu_stamp_reserved(u8 *reserved);

/* Advance the synthetic stand-in one step and publish it. No-op-worthy once a
 * real sensor drives imu_source_update(); guard call sites with IMU_USE_SYNTHETIC. */
void imu_source_tick_synthetic(void);
