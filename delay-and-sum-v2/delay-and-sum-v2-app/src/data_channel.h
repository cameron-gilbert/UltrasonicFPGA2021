#pragma once

#include "xil_types.h"  /* u8, s16, u32, u16 */

/* -------------------------------------------------------------------------- */
/* Protocol constants                                                         */
/* -------------------------------------------------------------------------- */
#define MIC_COUNT        102
#define SAMPLES_PER_MIC  512
#define TOTAL_CHANNELS   128      // Physical channels in DDR
#define HEADER_BYTES     70
#define PAYLOAD_BYTES    (SAMPLES_PER_MIC * 2)
#define PACKET_BYTES     (HEADER_BYTES + PAYLOAD_BYTES)
#define PACKET_SIGNATURE 0x76543210

#define HOST_IP0         192
#define HOST_IP1         168
#define HOST_IP2           1
#define HOST_IP3         100
#define HOST_PORT_DATA   5000

/* -------------------------------------------------------------------------- */
/* Packet layout                                                              */
/* -------------------------------------------------------------------------- */
typedef struct {
    u32 sw_frame_id;          // 4   — software frame counter
    u16 mic_id;               // 2   — microphone index (0–101)
    u32 hw_frame_id;          // 4   — DMA hardware frame counter
    u32 signature;            // 4   — 0x76543210
    u8  reserved[56];         // 56  → total header = 70 bytes
    s16 samples[SAMPLES_PER_MIC];
} __attribute__((packed)) mic_packet_t;

/* -------------------------------------------------------------------------- */
/* Public state                                                               */
/* -------------------------------------------------------------------------- */
extern uint32_t g_ms;         // Coarse millisecond counter (incremented by on_500ms_tick)

/* -------------------------------------------------------------------------- */
/* Public functions                                                           */
/* -------------------------------------------------------------------------- */

// Called once after lwIP init to create UDP socket
void start_data_channel(void);

// Called from main loop — transmit all MIC_COUNT packets for one frame
void send_frame_packets(s16 *ddr_buf, u32 hw_frame_id);

// Build a single mic packet into pkt (exposed for testing)
void build_mic_packet(mic_packet_t *pkt, uint32_t sw_frame_no, uint32_t hw_frame_no,
                      uint16_t mic_no, const s16 *samples);

// Called from the 500ms lwIP slow timer
void on_500ms_tick(void);

// Returns 1 if UDP socket is up
int is_data_connected(void);

// Attempt socket restart if disconnected (call from main loop)
void check_reconnection(void);

// Periodic throughput log (every 5 seconds, call from slow timer)
void stream_stats_run(void);

// Legacy 2-second throughput log
void update_stats(void);

