/* ===========================================================================
 * TCP Data Streaming - DMA Ping-Pong Integration
 * ===========================================================================
 * This module handles streaming ultrasound data from RAM to PC via TCP.
 * 
 * Data Flow (Frame processing now in frame_ready.c):
 *   1. DMA writes interleaved frame to Bank0 or Bank1
 *   2. DmaIsr fires → sets Bank0_Available_Flag or Bank1_Available_Flag
 *   3. Main loop calls frame_process() → de-interleaves DDR → RAM
 *   4. stream_scheduler_run() reads from RAM via frame_get_mic_samples()
 *   5. Builds packets and transmits via TCP
 * 
 * Deinterleaving is handled in frame_ready.c, not here.
 * =========================================================================== */

#include <stdio.h>
#include <string.h>

// Simulation mode: 0 = test patterns, 1 = read from DMA/DDR
// NOTE: Must match the same definition in main.c
#define USE_DDR_READS 1

#include "lwip/err.h"  /* Error codes (ERR_OK, ERR_MEM, etc.) */
#include "lwip/tcp.h"  /* TCP API: tcp_new, tcp_bind, tcp_listen, tcp_write, etc. */
#include "control_channel.h"  /* Port 6000 control channel */
#include "frame_ready.h"       /* Frame ready flags from DMA interrupts */
#if defined (__arm__) || defined (__aarch64__)
#include "xil_printf.h"
#include "xil_cache.h"         /* Cache invalidation functions */
#endif
// Math helpers
#include <math.h>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
//Creating packets
#define MIC_COUNT 102
#define SAMPLES_PER_MIC 512
#define HEADER_BYTES 70
#define PAYLOAD_BYTES (SAMPLES_PER_MIC * 2)
#define PACKET_BYTES (HEADER_BYTES + PAYLOAD_BYTES)
//Connection
#define HOST_IP0 192
#define HOST_IP1 168
#define HOST_IP2 1
#define HOST_IP3 100     // PC IP: 192.168.1.100 (confirmed from control connection)
#define HOST_PORT_DATA 5000
//OLD tracking
static uint8_t tx_buffer[1460]; // MTU-sized
static uint32_t seq = 0;
static struct tcp_pcb *active_pcb = NULL;
static uint32_t bytes_sent = 0;
static uint32_t timer_ticks = 0;  /* Incremented by slow timer (every 500ms) */
//Streaming control
static struct tcp_pcb *data_pcb = NULL;  // Client PCB for connection to PC
static int data_connected_flag = 0;
static uint32_t g_frame_no = 0;  // Software frame counter
static uint16_t g_mic_idx = 0;

// Hardware frame counter from DMA (set by DmaIsr in DMA_Interrupt.c)
extern u32 Frame_ID;

// Reconnection logic
static uint32_t g_last_reconnect_attempt_ms = 0;
static uint32_t g_reconnect_delay_ms = 3000;  // Try reconnect every 3 seconds

static uint32_t g_packets_sent = 0;

// Helper: check if scheduler should run
// Returns 1 if we should send next packet:
//   - If starting new frame (mic_idx==0): check if frame is ready
//   - If mid-frame (mic_idx>0): always continue sending
int should_run_scheduler(void) {
	if (g_mic_idx == 0) {
		// Starting new frame: wait for frame_ready
		return frame_ready();
	} else {
		// Mid-frame: keep sending regardless of frame_ready
		return 1;
	}
}
static uint64_t g_bytes_sent = 0;
static uint32_t g_snd_buf_stalls = 0;

//new diagnostics
static uint32_t g_scheduler_calls = 0;
static uint32_t g_tcp_write_errors = 0;
static uint32_t g_productive_calls = 0;  // Calls that sent >=1 packet
static uint32_t g_immediate_stalls = 0;  // Calls that sent 0 packets
static uint32_t g_tcp_output_calls = 0;  // How often tcp_output is called
static uint32_t g_max_burst_this_period = 0;  // Largest burst in current 5s window
static uint32_t g_ack_count = 0;  // How many ACKs received
static uint32_t g_bytes_acked = 0;  // Total bytes ACKed

//frame cadence
//static uint32_t g_frame_period_ms = 10; //100 fps giving 89Mbps output at 111KB/frame (unused - always runs at max speed)
static uint32_t g_last_frame_ms = 0;
uint32_t g_ms = 0; // coarse ms, updated by on_500ms_tick() (used in main.c for heartbeat)


// Write a 32-bit unsigned integer in big-endian format
//Static inline tells the compiler to replace the function call with the actual code
//Since these are tiny (3-4 instructions), inlining is faster and generates the same machine code

static inline void put_u32_be(uint8_t *dst, uint32_t v){
	dst[0] = (v >> 24) & 0xFF; //top 8 bits
	dst[1] = (v >> 16) & 0xFF;
	dst[2] = (v >> 8) & 0xFF;
	dst[3] = (v >> 0) & 0xFF; //bottom 8 bits
}

//Write a 16-bit unsigned integer in big-endian format
static inline void put_u16_be(uint8_t *dst, uint16_t v){
	dst[0] = (v >> 8) & 0xFF;
	dst[1] = (v >> 0) & 0xFF;
}

// Packet signature for validation
#define PACKET_SIGNATURE 0x76543210

void build_mic_packet(uint8_t *buf, uint32_t sw_frame_no, uint32_t hw_frame_no, uint16_t mic_no, const int16_t *samples){
	// Header: sw_frame_id (4 bytes BE)
	put_u32_be(buf + 0, sw_frame_no);

	// Header: mic number (2 bytes BE)
	put_u16_be(buf + 4, mic_no);

	// Header: hw_frame_id (4 bytes BE)
	put_u32_be(buf + 6, hw_frame_no);

	// Header: signature (4 bytes BE)
	put_u32_be(buf + 10, PACKET_SIGNATURE);

	// Header: reserved (56 bytes zero) - total header = 70 bytes
	memset(buf + 14, 0, 56);


	uint8_t *payload = buf + HEADER_BYTES;
	for (int i = 0; i < SAMPLES_PER_MIC; i++){
		uint16_t be = (uint16_t)samples[i];
		put_u16_be(payload + i*2, be);
	}
}
//Reusable buffers
static uint8_t g_packet_buf[PACKET_BYTES];
static int16_t g_samples[SAMPLES_PER_MIC];

//RAMP generator: centered int16 ramp, distinct per mic
static void gen_ramp_samples(uint16_t mic_id, int16_t *out){
	//Center around 0: map 0..65535 -> -32768..32767
	for (int i = 0; i < SAMPLES_PER_MIC; ++i){
		uint16_t u = (uint16_t)(i + mic_id);
		int16_t s = (int16_t)(u - 32768);
		out[i] = s;
	}
}

//SINE LUT
static int16_t g_sine_lut[SAMPLES_PER_MIC];
static int g_sine_lut_inited = 0;

static void init_sine_lut(void){
	if (g_sine_lut_inited) return;
	const double amp = 12000.0; //safe in16 amplitude
	for (int i = 0; i < SAMPLES_PER_MIC; ++i){
		double t = (double)i/(double)SAMPLES_PER_MIC;
		double v = amp * sin(2.0 * M_PI * t);
		g_sine_lut[i] = (int16_t)(v);
	}
	g_sine_lut_inited = 1;
}

static void gen_sine_samples(uint16_t mic_id, int16_t * out){
	init_sine_lut();
	int phase = (int)((mic_id * 5) % SAMPLES_PER_MIC);
	for (int i = 0; i < SAMPLES_PER_MIC; ++i) {
		out[i] = g_sine_lut[(i + phase) & (SAMPLES_PER_MIC - 1)];
	}
}

// static void example_build_one_packet(void) { /* unused */ }

//TCP DATA CHANNEL: CLIENT CALLBACKS
static err_t data_connected(void *arg, struct tcp_pcb *tpcb, err_t err);
static void data_err(void *arg, err_t err);
static err_t data_sent(void *arg, struct tcp_pcb *tpcb, u16_t len);

//Call once after lwIP init: connects to PC as TCP client
void start_data_channel(void){
	ip_addr_t pc_addr;
	IP4_ADDR(&pc_addr, HOST_IP0, HOST_IP1, HOST_IP2, HOST_IP3);
	
	xil_printf("[DATA] Connecting to PC at %d.%d.%d.%d:%d ...\r\n",
	           HOST_IP0, HOST_IP1, HOST_IP2, HOST_IP3, HOST_PORT_DATA);

	data_pcb = tcp_new();
	if (!data_pcb){
		xil_printf("[DATA] ERROR: pcb alloc failed\r\n");
		return;
	}

	// Register callbacks
	tcp_err(data_pcb, data_err);
	tcp_sent(data_pcb, data_sent);
	tcp_recv(data_pcb, NULL);  // No RX needed

	// Initiate connection to PC
	err_t e = tcp_connect(data_pcb, &pc_addr, HOST_PORT_DATA, data_connected);
	if (e != ERR_OK){
		xil_printf("[DATA] ERROR: tcp_connect failed with err=%d\r\n", e);
		tcp_abort(data_pcb);
		data_pcb = NULL;
		return;
	}

	xil_printf("[DATA] Connection initiated...\r\n");
}

static err_t data_connected(void *arg, struct tcp_pcb *tpcb, err_t err){
	if (err != ERR_OK) {
		xil_printf("[DATA] Connection failed, err=%d\r\n", err);
		return err;
	}

	ip_addr_t *addr = &(tpcb->remote_ip);
	xil_printf("[DATA] Connected to PC at %d.%d.%d.%d:%d\r\n",
	           ip4_addr1(addr), ip4_addr2(addr), ip4_addr3(addr), ip4_addr4(addr),
	           tpcb->remote_port);

	// data_pcb already set in start_data_channel()
	
	// Diagnostic: Check actual buffer sizes
	u16_t snd_buf = tcp_sndbuf(data_pcb);
	u16_t max_packets = snd_buf / PACKET_BYTES;
	xil_printf("[DATA] TCP_SND_BUF: %u bytes, can fit ~%u packets (%u bytes each)\r\n",
	           snd_buf, max_packets, PACKET_BYTES);

	// Reset stats
	g_frame_no = 0;
	g_mic_idx  = 0;
	g_packets_sent = 0;
	g_bytes_sent = 0;
	g_snd_buf_stalls = 0;
	g_scheduler_calls = 0;
	g_tcp_write_errors = 0;
	g_productive_calls = 0;
	g_immediate_stalls = 0;
	g_last_frame_ms = 0;
	data_connected_flag = 1;
	xil_printf("\r\n========================================\r\n");
	xil_printf("[DATA] *** TCP CONNECTION ESTABLISHED ***\r\n");
	xil_printf("[DATA] Ready to transmit data to PC\r\n");
	xil_printf("========================================\r\n\r\n");

	// DMA interrupt is already enabled by SetupSoundSystem
	// Bank0/Bank1 flags will be set by DmaIsr when frames arrive
#if USE_DDR_READS
	xil_printf("[DATA] DMA interrupt already enabled (Bank0/Bank1 flags active)\r\n");
#else
	xil_printf("[DATA] DMA SKIPPED (simulation mode)\r\n");
#endif

	// No packet timer - send full frame immediately on DMA interrupt
	xil_printf("[DATA] Ready to transmit - will send frames on DMA interrupt\r\n");
	xil_printf("[DATA] Waiting for frame ready signal (Bank flags)...\r\n");

	return ERR_OK;
}

static void data_err(void *arg, err_t err)
{
    xil_printf("[DATA] ERROR: connection failed, err=%d (pcb destroyed by lwIP)\r\n", err);
    
    // No packet timer to stop - just mark disconnected
    // DMA interrupt stays enabled (controlled by SetupSoundSystem)
    
    data_pcb = NULL; // pcb is gone
    data_connected_flag = 0;
    g_last_reconnect_attempt_ms = g_ms;  // Record disconnect time
}

static err_t data_sent(void *arg, struct tcp_pcb *tpcb, u16_t len)
{
    // Called when data is ACKed by receiver
    static int ack_count_local = 0;
    g_ack_count++;
    g_bytes_acked += len;
    ack_count_local++;
    
    // Always show ACKs for debugging (remove limit)
    xil_printf("[ACK] #%d: %u bytes acked (total: %u, buf_free: %u)\r\n", 
               ack_count_local, len, g_bytes_acked, tcp_sndbuf(tpcb));
    
    return ERR_OK;
}

// Check if we need to reconnect (call from main loop)
void check_reconnection(void)
{
    // Check if PCB exists but connection hasn't completed (stuck in SYN_SENT)
    if (data_pcb != NULL && !data_connected_flag) {
        // Connection stuck - check how long it's been
        if ((g_ms - g_last_reconnect_attempt_ms) >= 5000) {  // 5 seconds stuck
            xil_printf("[DATA] Connection timeout (PCB state: %d), aborting...\r\n", data_pcb->state);
            tcp_abort(data_pcb);
            data_pcb = NULL;
            g_last_reconnect_attempt_ms = g_ms;
            return;
        }
        return;  // Wait for connection to complete or timeout
    }
    
    // If already connected, nothing to do
    if (data_connected_flag) {
        return;
    }

    // Check if enough time has passed since last attempt
    if ((g_ms - g_last_reconnect_attempt_ms) >= g_reconnect_delay_ms) {
        xil_printf("[DATA] Attempting reconnection...\r\n");
        g_last_reconnect_attempt_ms = g_ms;
        start_data_channel();
    }
}

void on_500ms_tick(void)   // call this where you already call update_stats()
{
    g_ms += 500;
}

// Getter for connection status (for main.c debug output)
int is_data_connected(void)
{
    return data_connected_flag;
}

static int g_pattern = 1; // default to sine

static void generate_samples(uint16_t mic_id, int16_t *out)
{
    if (g_pattern == 0) {
        gen_ramp_samples(mic_id, out);
    } else {
        gen_sine_samples(mic_id, out);
    }
}

/* ========================================================================
 * DDR De-interleaving Function (for professor's DMA architecture)
 * ========================================================================
 * Reads samples for one microphone from interleaved DDR frame.
 * Memory layout: [S0:ch0..127][S1:ch0..127]...[S511:ch0..127]
 * Extracts: [S0:mic][S1:mic]...[S511:mic]
 * 
 * IMPORTANT: Uses professor's ping-pong buffer management:
 * - dma_get_ready_buffer_address() returns current buffer (A or B)
 * - Software reads from returned address
 * - After frame complete: dma_release_buffer() to free buffer
 * 
 * Cache Management: Professor will handle or advise on cache policy
 * ======================================================================== */

// Current frame buffer address (updated each frame from DMA manager)
#if USE_DDR_READS
// g_current_frame_base removed - now using frame_get_mic_samples() from frame_ready.c
static volatile uint32_t g_dma_frame_id = 0;
#endif

// DMA buffer management (defined in main.c, accessed here)
#if USE_DDR_READS
extern volatile uint16_t *dma_get_current_read_buffer(void);
extern uint32_t dma_get_current_frame_id(void);
extern void dma_frame_processed(void);
#endif

#if USE_DDR_READS
// Get pre-deinterleaved samples for one mic
// Data is deinterleaved in frame_ready.c when DMA interrupt fires
static void read_mic_samples_from_ddr(uint16_t mic_id, int16_t *out_samples)
{
    const int16_t *samples = frame_get_mic_samples(mic_id);
    
    if (samples == NULL) {
        xil_printf("[ERROR] Failed to get samples for mic %u!\r\n", mic_id);
        generate_samples(mic_id, out_samples);  // Fallback to test pattern
        return;
    }
    
    // Fast copy from pre-deinterleaved RAM buffer
    memcpy(out_samples, samples, SAMPLES_PER_MIC * sizeof(int16_t));
}
#endif

// Send as many packets as buffer allows for the current frame
// Returns: 1 when frame is complete, 0 otherwise
static int send_frame_step(void){
	if (!data_pcb) return 0;

	g_scheduler_calls++;
	u32 packets_this_call = 0;

	// Debug first frame start
	static int first_tx = 1;
	if (first_tx && g_mic_idx == 0) {
		xil_printf("[TX] Starting first frame transmission (frame %u)\r\n", g_frame_no);
		xil_printf("[TX] TCP buffer available: %u bytes\r\n", tcp_sndbuf(data_pcb));
		first_tx = 0;
	}

	// Keep pushing packets while we have buffer space
	while (g_mic_idx < MIC_COUNT){
		u16_t avail = tcp_sndbuf(data_pcb);
		if (avail < PACKET_BYTES){
			g_snd_buf_stalls++;
			// Buffer full - DON'T flush, let lwIP handle it naturally
			// Removing tcp_output() here allows packets to batch up efficiently
			static int stall_msg_count = 0;
			if (stall_msg_count < 5) {
				xil_printf("[TX_STALL] Buffer full: avail=%u, need=%u, mic=%u/%u\r\n",
				           avail, PACKET_BYTES, g_mic_idx, MIC_COUNT);
				stall_msg_count++;
			}
			if (packets_this_call > g_max_burst_this_period) g_max_burst_this_period = packets_this_call;
			break;
		}

		// Build current packet
#if USE_DDR_READS
		read_mic_samples_from_ddr(g_mic_idx, g_samples);
		
		// Debug first 4 samples of first packet
		if (g_mic_idx == 0 && g_frame_no < 2) {
			xil_printf("[TX_DATA] Mic 0 samples: %d, %d, %d, %d\r\n", 
			           g_samples[0], g_samples[1], g_samples[2], g_samples[3]);
		}
#else
		generate_samples(g_mic_idx, g_samples);
#endif
		// Build packet with both sw_frame_no and hw_frame_no (Frame_ID)
		build_mic_packet(g_packet_buf, g_frame_no, Frame_ID, g_mic_idx, g_samples);

		err_t e = tcp_write(data_pcb, g_packet_buf, PACKET_BYTES, TCP_WRITE_FLAG_COPY);
		if (e != ERR_OK) {
			xil_printf("[TX_ERROR] tcp_write err=%d mic=%u\r\n", e, g_mic_idx);
			g_tcp_write_errors++;
			break; // Back off and retry later
		}

		// Debug packet queueing
		if (g_packets_sent < 10) {  // First 10 packets ever
			xil_printf("[TX_OK] Packet #%u queued: mic=%u, frame=%u\r\n", 
			           g_packets_sent, g_mic_idx, g_frame_no);
		} else if (g_mic_idx < 3 && (g_frame_no % 100) == 0) {
			xil_printf("[TX_OK] Packet queued: mic=%u, frame=%u\r\n", g_mic_idx, g_frame_no);
		}

		g_packets_sent++;
		g_bytes_sent += PACKET_BYTES;
		g_mic_idx++;
		packets_this_call++;
	}
	
	// Frame complete?
	if (g_mic_idx >= MIC_COUNT) {
		if (g_frame_no == 0) {
			xil_printf("[TX] *** FIRST FRAME COMPLETE *** (102 packets sent)\r\n");
			xil_printf("[TX] Calling tcp_output to flush data...\r\n");
			xil_printf("[TX] Total bytes queued so far: %u\r\n", g_bytes_sent);
		} else if ((g_frame_no % 100) == 0) {
			xil_printf("[TX] Frame %u complete (102 mics sent)\r\n", g_frame_no);
		}
		
		g_mic_idx = 0;
		g_frame_no++;
		g_tcp_output_calls++;
		tcp_output(data_pcb); // Flush frame immediately to reduce latency
		dma_frame_processed(); // Mark frame as processed
		
		if (packets_this_call > g_max_burst_this_period) g_max_burst_this_period = packets_this_call;
		if (packets_this_call > 0) g_productive_calls++;
		else g_immediate_stalls++;
		return 1; // Frame complete
	}
	
	if (packets_this_call > 0) g_productive_calls++;
	else g_immediate_stalls++;
	return 0; // Frame incomplete
}

/* ============================================================================
 * stream_scheduler_run() - Main Frame Processing Function
 * ============================================================================
 * Called from main loop when frame_ready() flag is set (DMA interrupt fired).
 * 
 * Responsibilities:
 * 1. Read complete frame from DDR memory (DMA write location)
 * 2. De-interleave: convert from [S0:ch0..127][S1:ch0..127]... to per-mic packets
 * 3. Packetize and transmit over TCP with flow control
 * 
 * Memory Configuration (TODO - UPDATE THESE!):
 * - DMA_FRAME_BASE_ADDR: DDR address where DMA writes frames (from Vivado address editor)
 * - Frame size: 512 samples × 128 channels × 2 bytes = 131,072 bytes (128 KB)
 * - Interleaved layout: sample[i][channel[j]] stored at base + (i*128 + j)*2
 * 
 * De-interleaving Example:
 * - Mic 0 packet: samples[0][0], samples[1][0], ..., samples[511][0]
 * - Mic 1 packet: samples[0][1], samples[1][1], ..., samples[511][1]
 * - Read stride: +256 bytes (128 channels × 2 bytes) between consecutive samples
 * ============================================================================ */

// TODO: Set this to match your Vivado address editor DDR allocation for DMA
#define DMA_FRAME_BASE_ADDR  0x10000000  // Example: 256 MB offset in DDR

void stream_scheduler_run(void)
{
	if (!data_pcb || !data_connected_flag) return;

	// Init last frame time at first run
	if (g_last_frame_ms == 0) g_last_frame_ms = g_ms;

	// If we're mid-frame, keep trying to complete it
	if (g_mic_idx > 0){
		static int mid_frame_msg = 0;
		if (mid_frame_msg < 3) {
			xil_printf("[SCHED] Mid-frame continue: mic=%u/%u, buf_avail=%u\r\n",
			           g_mic_idx, MIC_COUNT, tcp_sndbuf(data_pcb));
			mid_frame_msg++;
		}
		send_frame_step();
		// Don't return - keep trying on every call until frame completes
	}

	// Send next frame immediately when previous completes and data is ready
	if (g_mic_idx == 0 && frame_data_ready()){
		g_last_frame_ms = g_ms;
		// Begin new frame by sending as many packets as buffer allows
		send_frame_step();
	}
}

static uint32_t g_stats_last_ms = 0;
void stream_stats_run(void)
{
    if ((g_ms - g_stats_last_ms) >= 5000) {
        u32 elapsed_ms = g_ms - g_stats_last_ms;
        g_stats_last_ms = g_ms;

        // Calculate key metrics
        u32 mbps_x100 = (u32)((g_bytes_sent * 8 * 100) / (elapsed_ms * 1000));  // Mbps * 100
        u32 avg_pkts_per_prod_call = g_productive_calls > 0 ? (g_packets_sent / g_productive_calls) : 0;
        u32 wasted_pct = g_scheduler_calls > 0 ? (g_immediate_stalls * 100 / g_scheduler_calls) : 0;
        u32 avg_bytes_per_ack = g_ack_count > 0 ? (g_bytes_acked / g_ack_count) : 0;
        u32 pkts_per_ack = avg_bytes_per_ack / PACKET_BYTES;  // How many packets per ACK
        
        xil_printf("[PERF] %u.%02u Mbps | %u pkts, %u frames\r\n", 
                   mbps_x100/100, mbps_x100%100, g_packets_sent, g_frame_no);
        xil_printf("  Calls: %u total, %u productive (%u%%), %u wasted (%u%%)\r\n",
                   g_scheduler_calls, g_productive_calls, 100-wasted_pct, g_immediate_stalls, wasted_pct);
        xil_printf("  Burst: avg ~%u, max %u | tcp_output: %u | Stalls: %u | Errors: %u\r\n",
                   avg_pkts_per_prod_call, g_max_burst_this_period, g_tcp_output_calls, g_snd_buf_stalls, g_tcp_write_errors);
        xil_printf("  ACKs: %u received, avg %u bytes/ACK (~%u pkts/ACK)\r\n",
                   g_ack_count, avg_bytes_per_ack, pkts_per_ack);
        xil_printf("  Buffer: %u/16384 free | TCP_WND: 16384\r\n",
                   tcp_sndbuf(data_pcb));
        
        // TCP flow control diagnostics
        if (data_pcb != NULL) {
            xil_printf("  TCP State: cwnd=%u, ssthresh=%u, snd_wnd=%u, snd_queuelen=%u\r\n",
                       data_pcb->cwnd, data_pcb->ssthresh, data_pcb->snd_wnd, data_pcb->snd_queuelen);
            xil_printf("  TCP Timing: rto=%u, rtt=%d, lastack=%u, unsent=%s, unacked=%s\r\n",
                       data_pcb->rto, data_pcb->rttest ? data_pcb->rttest : -1, 
                       data_pcb->lastack, 
                       data_pcb->unsent ? "yes" : "no",
                       data_pcb->unacked ? "yes" : "no");
        }

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

/* Hook for application-specific continuous transmission (unused in pure echo).
 * In DMA streaming: check buffer_ready flag, send buffer via tcp_write. */
int transfer_data() {
    if (active_pcb && tcp_sndbuf(active_pcb) >= sizeof(tx_buffer)) {
        // Fill with test pattern
        for (int i = 0; i < sizeof(tx_buffer); i += 4) {
            *(uint32_t*)(&tx_buffer[i]) = seq++;
        }
        err_t err = tcp_write(active_pcb, tx_buffer, sizeof(tx_buffer), TCP_WRITE_FLAG_COPY);
		if (err == ERR_OK) {
			tcp_output(active_pcb);
			bytes_sent += sizeof(tx_buffer);  // Only count if successful
		}
    }
    return 0;
}

/* Call this from main loop when TcpSlowTmrFlag is processed (every 500ms).
 * Prints throughput stats every 2 seconds (4 ticks). */
void update_stats() {
    timer_ticks++;
    if (timer_ticks >= 4) {  /* 4 * 500ms = 2 seconds */
        xil_printf("[STATS] Throughput: %u KB/s\r\n", bytes_sent / 1024 / 2);
        bytes_sent = 0;
        timer_ticks = 0;
    }
}


void print_app_header()
{
#if (LWIP_IPV6==0)
	xil_printf("\n\r\n\r-----lwIP TCP echo server ------\n\r");
#else
	xil_printf("\n\r\n\r-----lwIPv6 TCP echo server ------\n\r");
#endif
	/* NOTE: Template line refers to port 6001. Actual bound port is 7. */
	xil_printf("Template msg (port 6001); actual control port: 6000\n\r");
}


/* --------------------------------------------------------------------- */
/* start_application - Initialize network services                       */
/* --------------------------------------------------------------------- */
/* Starts the control channel server on port 6000 for parameter
 * configuration.
 *
 * Note:
 *   - Echo server (port 7) has been removed - no longer needed.
 *   - Data streaming on port 5000 starts automatically from main.c.
 *   - Control channel provides runtime parameter configuration only.
 * --------------------------------------------------------------------- */
int start_application()
{
	// Start control channel on port 6000 (parameter configuration)
	if (start_control_channel() != 0) {
		xil_printf("[CONTROL] Failed to start control channel\n\r");
		return -1;
	}
	xil_printf("[CONTROL] Parameter control ready on port 6000\n\r");

	return 0;
}
