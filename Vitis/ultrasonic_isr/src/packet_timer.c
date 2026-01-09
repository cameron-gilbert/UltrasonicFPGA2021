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
