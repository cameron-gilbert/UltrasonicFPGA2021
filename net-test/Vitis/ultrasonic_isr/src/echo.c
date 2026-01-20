
#include <stdio.h>
#include <string.h>

#include "lwip/err.h"  /* Error codes (ERR_OK, ERR_MEM, etc.) */
#include "lwip/tcp.h"  /* TCP API: tcp_new, tcp_bind, tcp_listen, tcp_write, etc. */
#include "control_channel.h"  /* Port 6000 control channel */
#include "packet_timer.h"      /* ISR-based packet timer */
#if defined (__arm__) || defined (__aarch64__)
#include "xil_printf.h"
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
#define HOST_IP3 100
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
static uint32_t g_frame_no = 0;
static uint16_t g_mic_idx = 0;

// Reconnection logic
static uint32_t g_last_reconnect_attempt_ms = 0;
static uint32_t g_reconnect_delay_ms = 3000;  // Try reconnect every 3 seconds

static uint32_t g_packets_sent = 0;
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
static uint32_t g_ms = 0; // coarse ms, updated by on_500ms_tick()


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

void build_mic_packet(uint8_t *buf, uint32_t frame_no, uint16_t mic_no, const int16_t *samples){
	//Header: frame number (4 bytes BE)
	put_u32_be(buf + 0, frame_no);

	//Header: mic number ( 2 bytes BE)
	put_u16_be(buf + 4, mic_no);

	//Header: reserved (64 bytes zero)
	memset(buf + 6, 0, 64);


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

	// Enable frame interrupt (DMA signals)
#if USE_DDR_READS
	frame_interrupt_enable();
	xil_printf("[DATA] Frame interrupt enabled (waiting for DMA frames from PL)\r\n");
#else
	xil_printf("[DATA] Frame interrupt SKIPPED (simulation mode)\r\n");
#endif

	// Start packet timer (100us pacing)
	packet_timer_start();
	xil_printf("[DATA] Packet timer started (10 kHz transmission rate)\r\n");

	return ERR_OK;
}

static void data_err(void *arg, err_t err)
{
    xil_printf("[DATA] ERROR: connection failed, err=%d (pcb destroyed by lwIP)\r\n", err);
    
    // Stop packet timer and disable frame interrupt when disconnected
    packet_timer_stop();
#if USE_DDR_READS
    frame_interrupt_disable();
#endif
    xil_printf("[DATA] Interrupts disabled\r\n");
    
    data_pcb = NULL; // pcb is gone
    data_connected_flag = 0;
    g_last_reconnect_attempt_ms = g_ms;  // Record disconnect time
}

static err_t data_sent(void *arg, struct tcp_pcb *tpcb, u16_t len)
{
    // Called when data is ACKed by receiver
    g_ack_count++;
    g_bytes_acked += len;
    return ERR_OK;
}

// Check if we need to reconnect (call from main loop)
void check_reconnection(void)
{
    // If already connected or PCB exists, nothing to do
    if (data_connected_flag || data_pcb != NULL) {
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

// Set to 1 for real DMA reads, 0 for simulation with test patterns
#define USE_DDR_READS 0

// Current frame buffer address (updated each frame from DMA manager)
#if USE_DDR_READS
static volatile int16_t *g_current_frame_base = NULL;
#endif

#if USE_DDR_READS
static void read_mic_samples_from_ddr(uint16_t mic_id, int16_t *out_samples)
{
    // Use current buffer address provided by DMA manager
    volatile int16_t *frame_base = g_current_frame_base;
    
    if (frame_base == NULL) {
        xil_printf("[ERROR] No buffer address set!\r\n");
        return;
    }
    
    for (int sample_idx = 0; sample_idx < SAMPLES_PER_MIC; sample_idx++) {
        int interleaved_index = (sample_idx * NUM_CHANNELS) + mic_id;
        out_samples[sample_idx] = frame_base[interleaved_index];
    }
}
#endif

// Send ONE packet per call (100us timer-paced transmission)
// Returns 1 when the frame is complete, 0 otherwise

static int send_frame_step(void){
	if (!data_pcb) return 0;
	if (g_mic_idx >= MIC_COUNT) return 1;  // Already complete

	g_scheduler_calls++;

	// Check if we have buffer space for ONE packet
	u16_t avail = tcp_sndbuf(data_pcb);
	if (avail < PACKET_BYTES){
		g_snd_buf_stalls++;
		g_immediate_stalls++;
		return 0;  // Buffer full - retry next 100us tick
	}

	// Build current packet (ONE mic only)
#if USE_DDR_READS
	read_mic_samples_from_ddr(g_mic_idx, g_samples);  // Real: read from DDR
#else
	generate_samples(g_mic_idx, g_samples);           // Sim: generate test pattern
#endif
	build_mic_packet(g_packet_buf, g_frame_no, g_mic_idx, g_samples);

	// Queue packet in lwIP
	err_t e = tcp_write(data_pcb, g_packet_buf, PACKET_BYTES, TCP_WRITE_FLAG_COPY);
	if (e != ERR_OK) {
		xil_printf("[DATA] tcp_write err=%d mic=%u\r\n", e, g_mic_idx);
		g_tcp_write_errors++;
		g_immediate_stalls++;
		return 0; // Retry next 100us tick
	}

	g_packets_sent++;
	g_bytes_sent += PACKET_BYTES;
	g_productive_calls++;
	g_mic_idx++;
	
	// Frame complete?
	if (g_mic_idx >= MIC_COUNT) {
		g_mic_idx = 0;
		g_frame_no++;
		g_tcp_output_calls++;
		tcp_output(data_pcb);  // Flush complete frame
		return 1;  // Signal: frame complete
	}
	
	return 0;  // Frame incomplete - more mics to send
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

	// On new frame: get buffer address from DMA manager
	if (g_mic_idx == 0 && frame_ready()) {
#if USE_DDR_READS
		g_current_frame_base = (volatile int16_t *)dma_get_ready_buffer_address();
		xil_printf("[FRAME] New buffer ready at 0x%08X\r\n", (uint32_t)g_current_frame_base);
#endif
	}

	//If we're mid frame, keep trying to complete it
	if (g_mic_idx > 0){
		send_frame_step();
		// Don't return - keep trying on every call until frame completes
	}

	//send next frame immediately when previous completes
	if (g_mic_idx == 0){
		g_last_frame_ms = g_ms;
		//Begin new frame by sending first mics
		if (send_frame_step()) {
			// Frame complete
#if USE_DDR_READS
			// Notify DMA we're done with this buffer
			dma_release_buffer();
			xil_printf("[FRAME] Buffer released to DMA\r\n");
			// Clear frame flag to accept next frame from DMA
			frame_clear();
#else
			// Simulation mode: immediately mark ready for next frame
			frame_clear();
#endif
		}
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
