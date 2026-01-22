#include "frame_ready.h"  // Frame ready flags and DMA interrupt interface
#include "xil_printf.h"
#include "xparameters.h"
#include "xil_cache.h"
#include <string.h>

/* De-interleaving buffer - stores processed frame data in RAM */
#define MIC_COUNT 102
#define SAMPLES_PER_MIC 512
#define NUM_CHANNELS 128

static int16_t g_deinterleaved_data[MIC_COUNT][SAMPLES_PER_MIC];
static volatile int g_frame_deinterleaved = 0;
static volatile uint16_t *g_last_frame_base = NULL;

/* Frame Interrupt Handler
 * Connects to HLS DMA end-of-frame interrupt from FPGA fabric.
 * 
 * Architecture:
 * - Professor's DMA writes complete frame to DDR (512 samples × 128 channels, interleaved)
 * - DMA asserts end-of-frame interrupt to PS fabric interrupt line
 * - This ISR catches interrupt, sets frame_ready_flag
 * - Main loop checks flag, reads frame from DDR, de-interleaves, and transmits
 */

// Flag set by ISR when frame is complete, cleared by main loop after processing
static volatile int frame_ready_flag = 0;

// Frame statistics
static volatile uint32_t frames_received = 0;
static volatile uint32_t frames_missed = 0;  // Incremented if flag already set (overrun)

/* Frame-Ready ISR (called from GIC when DMA finishes frame write)
 * Minimal latency: just set flag and clear interrupt status.
 */
static void frame_ready_isr(void *callback_ref)
{
    // Check if previous frame wasn't processed yet (overrun detection)
    if (frame_ready_flag) {
        frames_missed++;
        xil_printf("[FRAME_ISR] WARNING: Frame overrun! Previous frame not processed yet.\r\n");
    }
    
    // Set flag for main loop
    frame_ready_flag = 1;
    frames_received++;
    
    // Note: No explicit interrupt clear needed here - fabric interrupt auto-clears
    // when DMA de-asserts the interrupt line (edge-triggered or level-sensitive based on design)
}

int frame_interrupt_init(XScuGic *gic_inst)
{
    int status;
    
    xil_printf("[FRAME_INTR] Initializing frame-ready interrupt handler...\r\n");
    xil_printf("[FRAME_INTR] Connecting to fabric interrupt ID: %d\r\n", FRAME_INTERRUPT_ID);
    
    // Connect ISR to GIC
    status = XScuGic_Connect(gic_inst, FRAME_INTERRUPT_ID,
                            (Xil_ExceptionHandler)frame_ready_isr,
                            NULL);  // No callback data needed
    if (status != XST_SUCCESS) {
        xil_printf("[FRAME_INTR] ERROR: ISR connect failed (status=%d)\r\n", status);
        xil_printf("[FRAME_INTR] Check that DMA interrupt is connected to correct fabric IRQ in Vivado\r\n");
        return status;
    }
    
    // Set interrupt priority and trigger type
    // Priority: 0xA0 (mid-range, higher than lwIP timers)
    XScuGic_SetPriorityTriggerType(gic_inst, FRAME_INTERRUPT_ID, 0xA0, 0x3);
    // 0x3 = rising edge triggered (typical for fabric interrupts)
    
    xil_printf("[FRAME_INTR] Initialization complete (interrupt enabled on connect)\r\n");
    xil_printf("[FRAME_INTR] Frame parameters: %d samples/ch × %d channels × %d bytes\r\n",
               SAMPLES_PER_CHANNEL, NUM_CHANNELS, BYTES_PER_SAMPLE);
    xil_printf("[FRAME_INTR] Total frame size: %d bytes (%.1f KB)\r\n",
               SAMPLES_PER_CHANNEL * NUM_CHANNELS * BYTES_PER_SAMPLE,
               (float)(SAMPLES_PER_CHANNEL * NUM_CHANNELS * BYTES_PER_SAMPLE) / 1024.0f);
    
    return XST_SUCCESS;
}

void frame_interrupt_enable(void)
{
    // Enable interrupt in GIC
    extern XScuGic Intc;
    XScuGic_Enable(&Intc, FRAME_INTERRUPT_ID);
    
    xil_printf("[FRAME_INTR] Interrupt enabled (waiting for DMA frames)\r\n");
}

void frame_interrupt_disable(void)
{
    extern XScuGic Intc;
    XScuGic_Disable(&Intc, FRAME_INTERRUPT_ID);
    
    // Clear any pending flag
    frame_ready_flag = 0;
    
    xil_printf("[FRAME_INTR] Interrupt disabled\r\n");
}

int frame_ready(void)
{
    // Check if either bank has data available
    extern volatile u8 Bank0_Available_Flag;
    extern volatile u8 Bank1_Available_Flag;
    return (Bank0_Available_Flag || Bank1_Available_Flag);
}

void frame_clear(void)
{
    // Clear whichever flag was set
    extern volatile u8 Bank0_Available_Flag;
    extern volatile u8 Bank1_Available_Flag;
    
    if (Bank0_Available_Flag) {
        Bank0_Available_Flag = 0;
    }
    if (Bank1_Available_Flag) {
        Bank1_Available_Flag = 0;
    }
}

/* De-interleave frame from DDR to RAM buffer */
void frame_deinterleave(volatile uint16_t *frame_base)
{
    if (frame_base == NULL) {
        xil_printf("[ERROR] DMA buffer NULL!\r\n");
        return;
    }
    
    // Store buffer address for access by tcp_stream
    g_last_frame_base = frame_base;
    
    // Invalidate cache before reading DMA-written data
    // Frame size: 128 channels × 512 samples × 2 bytes = 131,072 bytes
    Xil_DCacheInvalidateRange((UINTPTR)frame_base, NUM_CHANNELS * SAMPLES_PER_MIC * sizeof(uint16_t));
    
    // Sequential read through interleaved buffer
    // Format: [S0_ch0, S0_ch1, ..., S0_ch127, S1_ch0, S1_ch1, ..., S511_ch127]
    for (int sample_idx = 0; sample_idx < SAMPLES_PER_MIC; sample_idx++) {
        int base_index = sample_idx * NUM_CHANNELS;
        for (int mic_id = 0; mic_id < MIC_COUNT; mic_id++) {
            g_deinterleaved_data[mic_id][sample_idx] = (int16_t)frame_base[base_index + mic_id];
        }
    }
    
    g_frame_deinterleaved = 1;
}

/* Get pre-deinterleaved samples for one mic (called from tcp_stream.c) */
const int16_t* frame_get_mic_samples(uint16_t mic_id)
{
    if (mic_id >= MIC_COUNT) {
        return NULL;
    }
    
    if (!g_frame_deinterleaved) {
        xil_printf("[ERROR] Frame not deinterleaved yet!\r\n");
        return NULL;
    }
    
    return g_deinterleaved_data[mic_id];
}

/* Check if deinterleaved data is available */
int frame_data_ready(void)
{
    return g_frame_deinterleaved;
}

/* Process frame when flag is set - called from main loop */
void frame_process(void)
{
    extern volatile u8 Bank0_Available_Flag;
    extern volatile u8 Bank1_Available_Flag;
    extern u8 Bank0[];
    extern u8 Bank1[];
    extern int is_data_connected(void);
    extern int is_control_connected(void);
    
    static int first_frame = 1;
    static uint32_t total_count = 0;
    static uint32_t bank0_count = 0;
    static uint32_t bank1_count = 0;
    static int waiting_msg_shown = 0;
    
    // Wait for both channels to be connected before processing frames
    if (!is_data_connected() || !is_control_connected()) {
        if (!waiting_msg_shown && (Bank0_Available_Flag || Bank1_Available_Flag)) {
            xil_printf("[FRAME] Waiting for both data and control channels to connect...\r\n");
            waiting_msg_shown = 1;
        }
        return;  // Don't process frames until both channels are ready
    }
    
    // Reset waiting message flag once both channels are connected
    if (waiting_msg_shown) {
        xil_printf("[FRAME] Both channels connected - starting frame processing\r\n");
        waiting_msg_shown = 0;
    }
    
    if (Bank0_Available_Flag) {
        if (first_frame) {
            xil_printf("\r\n[FRAME] *** FIRST FRAME DETECTED (Bank0) ***\r\n");
            xil_printf("[FRAME] Starting deinterleave from DDR to RAM...\r\n\r\n");
            first_frame = 0;
        }
        volatile uint16_t *frame_base = (volatile uint16_t *)Bank0;
        frame_deinterleave(frame_base);
        Bank0_Available_Flag = 0;
        bank0_count++;
        total_count++;
        if ((total_count % 100) == 0) {
            xil_printf("[FRAME] Processed %u frames total (Bank0=%u, Bank1=%u)\r\n", 
                       total_count, bank0_count, bank1_count);
        }
    } else if (Bank1_Available_Flag) {
        if (first_frame) {
            xil_printf("\r\n[FRAME] *** FIRST FRAME DETECTED (Bank1) ***\r\n");
            xil_printf("[FRAME] Starting deinterleave from DDR to RAM...\r\n\r\n");
            first_frame = 0;
        }
        volatile uint16_t *frame_base = (volatile uint16_t *)Bank1;
        frame_deinterleave(frame_base);
        Bank1_Available_Flag = 0;
        bank1_count++;
        total_count++;
        if ((total_count % 100) == 0) {
            xil_printf("[FRAME] Processed %u frames total (Bank0=%u, Bank1=%u)\r\n", 
                       total_count, bank0_count, bank1_count);
        }
    }
}

// Diagnostic function - returns frame statistics  
void frame_get_stats(uint32_t *received, uint32_t *missed)
{
    if (received) *received = frames_received;
    if (missed) *missed = frames_missed;
}

/* ============================================================================
 * Note: Packet timer code removed - no longer using timed packet pacing.
 * Frames are now transmitted at maximum TCP throughput immediately when ready.
 * ============================================================================ */
