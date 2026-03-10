#include <stdio.h>
#include <string.h>
#include <math.h>

// Simulation mode: 0 = test patterns, 1 = read from DMA/DDR
// Must match the same definition in main.c
#define USE_DDR_READS 1

#include "lwip/err.h"
#include "lwip/udp.h"
#include "lwip/pbuf.h"
#include "lwip/def.h"    /* htonl, htons */
#include "control_channel.h"
#include "data_channel.h"
#if defined (__arm__) || defined (__aarch64__)
#include "xil_printf.h"
#include "xil_cache.h"
#include "xtime_l.h"   /* XTime_GetTime — Global Timer, ~333 MHz */
#endif

/* Converts Global Timer ticks to microseconds.
 * Global Timer runs at XPAR_CPU_CORTEXA9_0_CPU_CLK_FREQ_HZ / 2. */
#define GTIMER_US(ticks)  ((uint32_t)((ticks) / (XPAR_CPU_CORTEXA9_0_CPU_CLK_FREQ_HZ / 2 / 1000000ULL)))

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* -------------------------------------------------------------------------- */
/* Module state                                                               */
/* -------------------------------------------------------------------------- */

// UDP socket
static struct udp_pcb *data_pcb            = NULL;
static int             data_connected_flag = 0;

// Frame tracking
static uint32_t g_frame_no  = 0;

// Reconnection back-off
static uint32_t g_last_reconnect_attempt_ms = 0;
static uint32_t g_reconnect_delay_ms        = 3000;

// Timing
uint32_t g_ms            = 0;       // Coarse ms, incremented by on_500ms_tick()

// Performance counters
static uint64_t g_bytes_sent          = 0;
static uint32_t g_packets_sent        = 0;
static uint32_t g_snd_buf_stalls      = 0;
static uint32_t g_scheduler_calls     = 0;
static uint32_t g_tcp_write_errors    = 0;
static uint32_t g_productive_calls    = 0;
static uint32_t g_immediate_stalls    = 0;
static uint32_t g_tcp_output_calls    = 0;
static uint32_t g_max_burst_this_period = 0;
static uint32_t g_ack_count           = 0;
static uint32_t g_bytes_acked         = 0;

// Bank timing stats (all in microseconds, using Global Timer)
static XTime   g_last_bank_ready_ticks  = 0;
static uint32_t g_bank_interval_sum_us  = 0;
static uint32_t g_bank_send_time_sum_us = 0;
static uint32_t g_bank_interval_count   = 0;
static uint32_t g_min_bank_interval_us  = 0xFFFFFFFF;
static uint32_t g_max_bank_interval_us  = 0;

// Legacy stats (update_stats)
static uint32_t bytes_sent   = 0;
static uint32_t timer_ticks  = 0;

// Test pattern selection (0 = ramp, 1 = sine)
static int g_pattern = 1;

// Reusable packet buffers
static int16_t      g_samples[SAMPLES_PER_MIC];
static mic_packet_t g_packet;

/* -------------------------------------------------------------------------- */
static void     init_sine_lut(void);
static void     gen_ramp_samples(uint16_t mic_id, int16_t *out);
static void     gen_sine_samples(uint16_t mic_id, int16_t *out);
static void     generate_samples(uint16_t mic_id, int16_t *out);

void start_data_channel(void) {
    ip_addr_t pc_addr;
    IP4_ADDR(&pc_addr, HOST_IP0, HOST_IP1, HOST_IP2, HOST_IP3);

    xil_printf("[DATA] Setting up UDP socket for PC at %d.%d.%d.%d:%d ...\r\n",
               HOST_IP0, HOST_IP1, HOST_IP2, HOST_IP3, HOST_PORT_DATA);

    data_pcb = udp_new();
    if (!data_pcb) {
        xil_printf("[DATA] ERROR: UDP PCB alloc failed\r\n");
        return;
    }

    err_t e = udp_connect(data_pcb, &pc_addr, HOST_PORT_DATA);
    if (e != ERR_OK) {
        xil_printf("[DATA] ERROR: udp_connect failed with err=%d\r\n", e);
        udp_remove(data_pcb);
        data_pcb = NULL;
        return;
    }

    // Reset all state
    g_frame_no       = 0;
    g_packets_sent   = 0;
    g_bytes_sent     = 0;
    g_snd_buf_stalls = 0;
    g_scheduler_calls= 0;
    g_tcp_write_errors=0;
    g_productive_calls=0;
    g_immediate_stalls=0;
    data_connected_flag = 1;

    xil_printf("\r\n========================================\r\n");
    xil_printf("[DATA] *** UDP SOCKET READY ***\r\n");
    xil_printf("[DATA] Ready to transmit data to PC\r\n");
    xil_printf("========================================\r\n\r\n");

#if USE_DDR_READS
    xil_printf("[DATA] DMA interrupt already enabled (Bank0/Bank1 flags active)\r\n");
#else
    xil_printf("[DATA] DMA SKIPPED (simulation mode)\r\n");
#endif
    xil_printf("[DATA] Waiting for frame ready signal (Bank flags)...\r\n");
}

void send_frame_packets(s16 *ddr_buf, u32 hw_frame_id) {
    if (!data_pcb || !data_connected_flag || !is_control_connected()) return;

    g_scheduler_calls++;

    // Bank timing: record interval since last bank was ready (microsecond resolution)
    XTime bank_start_ticks;
    XTime_GetTime(&bank_start_ticks);
    if (g_last_bank_ready_ticks != 0) {
        uint32_t interval_us = GTIMER_US(bank_start_ticks - g_last_bank_ready_ticks);
        g_bank_interval_sum_us += interval_us;
        g_bank_interval_count++;
        if (interval_us < g_min_bank_interval_us) g_min_bank_interval_us = interval_us;
        if (interval_us > g_max_bank_interval_us) g_max_bank_interval_us = interval_us;
    }
    g_last_bank_ready_ticks = bank_start_ticks;

    u32 packets_this_call = 0;

    // Debug first frame start
    static int first_tx = 1;
    if (first_tx) {
        xil_printf("[TX] Starting first frame transmission (frame %u)\r\n", g_frame_no);
        first_tx = 0;
    }

    for (uint16_t mic = 0; mic < MIC_COUNT; mic++) {
#if USE_DDR_READS
        for (uint32_t s = 0; s < SAMPLES_PER_MIC; s++)
            g_samples[s] = ddr_buf[s * TOTAL_CHANNELS + mic];
#else
        generate_samples(mic, g_samples);
#endif
        build_mic_packet(&g_packet, g_frame_no, hw_frame_id, mic, g_samples);

        struct pbuf *p = pbuf_alloc(PBUF_TRANSPORT, sizeof(mic_packet_t), PBUF_RAM);
        if (!p) {
            xil_printf("[TX_ERROR] pbuf_alloc failed for mic=%u\r\n", mic);
            g_tcp_write_errors++;
            break;
        }
        memcpy(p->payload, &g_packet, sizeof(mic_packet_t));
        err_t e = udp_send(data_pcb, p);
        pbuf_free(p);
        if (e != ERR_OK) {
            xil_printf("[TX_ERROR] udp_send err=%d mic=%u\r\n", e, mic);
            g_tcp_write_errors++;
            break;
        }


        g_packets_sent++;
        g_bytes_sent += PACKET_BYTES;
        packets_this_call++;
    }

    if (g_frame_no == 0) {
        xil_printf("[TX] *** FIRST FRAME COMPLETE *** (102 packets sent)\r\n");
        xil_printf("[TX] Total bytes sent so far: %u\r\n", g_bytes_sent);
    }

    g_frame_no++;
    if (packets_this_call > g_max_burst_this_period) g_max_burst_this_period = packets_this_call;
    if (packets_this_call > 0) g_productive_calls++;
    else g_immediate_stalls++;

    // Bank timing: accumulate send duration (microseconds)
    XTime bank_end_ticks;
    XTime_GetTime(&bank_end_ticks);
    g_bank_send_time_sum_us += GTIMER_US(bank_end_ticks - bank_start_ticks);
}

void build_mic_packet(mic_packet_t *pkt, uint32_t sw_frame_no, uint32_t hw_frame_no,
                      uint16_t mic_no, const s16 *samples) {
    pkt->sw_frame_id = htonl(sw_frame_no);
    pkt->mic_id      = htons(mic_no);
    pkt->hw_frame_id = htonl(hw_frame_no);
    pkt->signature   = htonl(PACKET_SIGNATURE);
    memset(pkt->reserved, 0, sizeof(pkt->reserved));
    memcpy(pkt->samples, samples, sizeof(pkt->samples));  // samples copied as-is (no byte swap)
}


//HELPERS
static int16_t g_sine_lut[SAMPLES_PER_MIC];
static int     g_sine_lut_inited = 0;

static void init_sine_lut(void) {
    if (g_sine_lut_inited) return;
    const double amp = 12000.0;
    for (int i = 0; i < SAMPLES_PER_MIC; ++i) {
        double t = (double)i / (double)SAMPLES_PER_MIC;
        g_sine_lut[i] = (int16_t)(amp * sin(2.0 * M_PI * t));
    }
    g_sine_lut_inited = 1;
}

static void gen_ramp_samples(uint16_t mic_id, int16_t *out) {
    for (int i = 0; i < SAMPLES_PER_MIC; ++i) {
        uint16_t u = (uint16_t)(i + mic_id);
        out[i] = (int16_t)(u - 32768);
    }
}

static void gen_sine_samples(uint16_t mic_id, int16_t *out) {
    init_sine_lut();
    int phase = (int)((mic_id * 5) % SAMPLES_PER_MIC);
    for (int i = 0; i < SAMPLES_PER_MIC; ++i)
        out[i] = g_sine_lut[(i + phase) & (SAMPLES_PER_MIC - 1)];
}

static void generate_samples(uint16_t mic_id, int16_t *out) {
    if (g_pattern == 0) gen_ramp_samples(mic_id, out);
    else                gen_sine_samples(mic_id, out);
}


void check_reconnection(void)
{
    // UDP is connectionless - if PCB exists we're ready
    if (data_pcb != NULL) return;

    if ((g_ms - g_last_reconnect_attempt_ms) >= g_reconnect_delay_ms) {
        xil_printf("[DATA] Restarting UDP socket...\r\n");
        g_last_reconnect_attempt_ms = g_ms;
        start_data_channel();
    }
}

void on_500ms_tick(void)
{
    g_ms += 500;
}

int is_data_connected(void)
{
    return data_connected_flag;
}

static uint32_t g_stats_last_ms = 0;
void stream_stats_run(void)
{
    if ((g_ms - g_stats_last_ms) >= 5000) {
        u32 elapsed_ms = g_ms - g_stats_last_ms;
        g_stats_last_ms = g_ms;

        // Bank timing (all in microseconds)
        u32 frames_in_period = g_bank_interval_count;
        u32 avg_interval_us = frames_in_period > 0 ? (g_bank_interval_sum_us / frames_in_period) : 0;
        u32 avg_send_us     = frames_in_period > 0 ? (g_bank_send_time_sum_us / frames_in_period) : 0;
        u32 min_us = g_min_bank_interval_us == 0xFFFFFFFF ? 0 : g_min_bank_interval_us;
        //xil_printf("[PERF] interval: avg %u us, min %u us, max %u us | send: avg %u us | errs: %u\r\n",
        //           avg_interval_us, min_us, g_max_bank_interval_us, avg_send_us, g_tcp_write_errors);

        g_bank_interval_sum_us  = 0;
        g_bank_send_time_sum_us = 0;
        g_bank_interval_count   = 0;
        g_min_bank_interval_us  = 0xFFFFFFFF;
        g_max_bank_interval_us  = 0;

        g_packets_sent = 0;
        g_bytes_sent = 0;
        g_snd_buf_stalls = 0;
        g_scheduler_calls = 0;
        g_tcp_write_errors = 0;
        g_productive_calls = 0;
        g_immediate_stalls = 0;
        g_tcp_output_calls = 0;
        g_max_burst_this_period = 0;
        g_ack_count = 0;
        g_bytes_acked = 0;
    }
}

void update_stats(void) {
    timer_ticks++;
    if (timer_ticks >= 4) {  /* 4 * 500ms = 2 seconds */
        xil_printf("[STATS] Throughput: %u KB/s\r\n", bytes_sent / 1024 / 2);
        bytes_sent = 0;
        timer_ticks = 0;
    }
}


