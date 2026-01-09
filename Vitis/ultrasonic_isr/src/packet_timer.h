#pragma once

#include "xscugic.h"
#include "xparameters.h"
#include "xscutimer.h"

/* Dual Interrupt Architecture:
 * 1. Frame Interrupt: Signals when DMA has written complete frame to DDR
 * 2. Packet Timer: Fires every 100µs to pace individual packet transmission
 */

/* Frame Interrupt Configuration
 * This connects to the HLS DMA end-of-frame interrupt from PL (FPGA fabric).
 * The professor's DMA fires this interrupt when a complete frame (512 samples × 128 channels)
 * has been written to DDR memory in interleaved format.
 * 
 * Configuration:
 * - FRAME_INTERRUPT_ID: Set this to the fabric interrupt number used by your DMA
 *   (XPS_FPGA0_INT_ID through XPS_FPGA15_INT_ID, defined in xparameters_ps.h)
 * - Adjust this based on your Vivado block design's interrupt connection
 */

// TODO: Update this to match your DMA's actual interrupt connection in Vivado
// Check Vivado block design -> Zynq PS -> Interrupts -> Fabric Interrupts -> PL-PS Interrupt Ports
#define FRAME_INTERRUPT_ID   XPS_FPGA0_INT_ID  // Default: fabric interrupt 0 (ID 61)

/* Frame Parameters (from professor's design)
 * - 512 samples per channel
 * - 128 logical channels (102 real mics + padding)
 * - Interleaved storage: S0[ch0..127], S1[ch0..127], ..., S511[ch0..127]
 * - Total frame size: 512 * 128 * 2 bytes = 131,072 bytes (128 KB)
 */
#define SAMPLES_PER_CHANNEL  512
#define NUM_CHANNELS         128   // Logical channels (power-of-2 for addressing)
#define NUM_ACTIVE_MICS      102   // Actual microphones in use
#define BYTES_PER_SAMPLE     2     // 16-bit samples

// Initialize frame interrupt handler (connects to DMA interrupt)
int frame_interrupt_init(XScuGic *gic_inst);

// Start/stop interrupt handling
void frame_interrupt_enable(void);
void frame_interrupt_disable(void);

// Check if frame is ready (called from main loop)
int frame_ready(void);

// Clear ready flag after processing frame
void frame_clear(void);

/* ============================================================================
 * DMA Buffer Management Interface (Professor-Implemented)
 * ============================================================================
 * These functions will be provided by the professor's DMA control module.
 * They manage ping-pong buffering and provide buffer addresses to software.
 * 
 * Integration: Declare these as extern in your code, professor will link them.
 * ============================================================================ */

// Get address of current ready buffer (Buffer A or Buffer B)
// Returns: DDR address of buffer ready for transmission
// Called after frame interrupt fires
extern uint32_t* dma_get_ready_buffer_address(void);

// Get buffer size (number of frames per buffer)
// Returns: Size in bytes of each ping-pong buffer
extern uint32_t dma_get_buffer_size(void);

// Notify DMA that software finished reading buffer
// Allows DMA to reuse this buffer for next frame
// Call after completing transmission
extern void dma_release_buffer(void);

/* 100µs Packet Timer Configuration
 * Paces transmission: one mic packet every 100 microseconds (10 kHz rate)
 * Works with frame interrupt: only sends when frame data is available
 */
#define PACKET_TIMER_DEVICE_ID  XPAR_XSCUTIMER_0_DEVICE_ID
#define PACKET_TIMER_INTR_ID    XPS_SCU_TMR_INT_ID
#define PACKET_INTERVAL_US      100

// Initialize 100µs packet timer (separate from frame interrupt)
int packet_timer_init(XScuGic *gic_inst);

// Start/stop packet timer
void packet_timer_start(void);
void packet_timer_stop(void);

// Check if it's time to send next packet
int packet_timer_ready(void);
void packet_timer_clear(void);
