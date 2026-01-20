#include "packet_timer.h"  // Will be renamed to frame_interrupt.h
#include "xil_printf.h"
#include "xparameters.h"

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
    // Enable interrupt in GIC - will be called in frame_interrupt_init already,
    // but this allows re-enabling after disable
    extern XScuGic gic_inst;  // Defined in main.c
    XScuGic_Enable(&gic_inst, FRAME_INTERRUPT_ID);
    
    xil_printf("[FRAME_INTR] Interrupt enabled (waiting for DMA frames)\r\n");
}

void frame_interrupt_disable(void)
{
    extern XScuGic gic_inst;  // Defined in main.c
    XScuGic_Disable(&gic_inst, FRAME_INTERRUPT_ID);
    
    // Clear any pending flag
    frame_ready_flag = 0;
    
    xil_printf("[FRAME_INTR] Interrupt disabled\r\n");
}

int frame_ready(void)
{
    return frame_ready_flag;
}

void frame_clear(void)
{
    frame_ready_flag = 0;
}

// Diagnostic function - returns frame statistics
void frame_get_stats(uint32_t *received, uint32_t *missed)
{
    if (received) *received = frames_received;
    if (missed) *missed = frames_missed;
}

/* ============================================================================
 * 100µs Packet Timer Implementation
 * ============================================================================
 * Separate timer that fires every 100 microseconds to pace packet transmission.
 * Works in conjunction with frame interrupt:
 * - Frame interrupt: Marks frame data as available in DDR
 * - Packet timer: Controls when to send next mic packet (rate limiting)
 * 
 * This prevents overwhelming TCP send buffer and provides smooth 10 kHz packet rate.
 * ============================================================================ */

static XScuTimer packet_timer_inst;
static volatile int packet_timer_flag = 0;

static void packet_timer_isr(void *callback_ref)
{
    XScuTimer *timer = (XScuTimer *)callback_ref;
    XScuTimer_ClearInterruptStatus(timer);
    packet_timer_flag = 1;
}

int packet_timer_init(XScuGic *gic_inst)
{
    int status;
    XScuTimer_Config *timer_config;
    
    xil_printf("[PKT_TIMER] Initializing 100us packet timer...\r\n");
    
    timer_config = XScuTimer_LookupConfig(PACKET_TIMER_DEVICE_ID);
    if (timer_config == NULL) {
        xil_printf("[PKT_TIMER] ERROR: Config lookup failed\r\n");
        return XST_FAILURE;
    }
    
    status = XScuTimer_CfgInitialize(&packet_timer_inst, timer_config, timer_config->BaseAddr);
    if (status != XST_SUCCESS) {
        xil_printf("[PKT_TIMER] ERROR: Init failed\r\n");
        return status;
    }
    
    status = XScuTimer_SelfTest(&packet_timer_inst);
    if (status != XST_SUCCESS) {
        xil_printf("[PKT_TIMER] ERROR: Self-test failed\r\n");
        return status;
    }
    
    // Calculate load value for 100us
    u32 timer_clock_hz = XPAR_CPU_CORTEXA9_0_CPU_CLK_FREQ_HZ / 2;
    u32 load_value = (timer_clock_hz * PACKET_INTERVAL_US) / 1000000;
    
    XScuTimer_LoadTimer(&packet_timer_inst, load_value);
    XScuTimer_EnableAutoReload(&packet_timer_inst);
    
    // Connect ISR to GIC (use different priority than frame interrupt)
    status = XScuGic_Connect(gic_inst, PACKET_TIMER_INTR_ID,
                            (Xil_ExceptionHandler)packet_timer_isr,
                            (void *)&packet_timer_inst);
    if (status != XST_SUCCESS) {
        xil_printf("[PKT_TIMER] ERROR: ISR connect failed\r\n");
        return status;
    }
    
    // Set lower priority than frame interrupt (0xB0 vs 0xA0)
    XScuGic_SetPriorityTriggerType(gic_inst, PACKET_TIMER_INTR_ID, 0xB0, 0x3);
    
    XScuGic_Enable(gic_inst, PACKET_TIMER_INTR_ID);
    
    xil_printf("[PKT_TIMER] Initialized (100us intervals, 10 kHz packet rate)\r\n");
    
    return XST_SUCCESS;
}

void packet_timer_start(void)
{
    packet_timer_flag = 0;
    XScuTimer_EnableInterrupt(&packet_timer_inst);
    XScuTimer_Start(&packet_timer_inst);
    xil_printf("[PKT_TIMER] Started (10 kHz packet pacing)\r\n");
}

void packet_timer_stop(void)
{
    XScuTimer_Stop(&packet_timer_inst);
    XScuTimer_DisableInterrupt(&packet_timer_inst);
    packet_timer_flag = 0;
    xil_printf("[PKT_TIMER] Stopped\r\n");
}

int packet_timer_ready(void)
{
    return packet_timer_flag;
}

void packet_timer_clear(void)
{
    packet_timer_flag = 0;
}
