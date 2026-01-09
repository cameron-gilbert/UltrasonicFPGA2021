#pragma once

#include "xscugic.h"
#include "xparameters.h"

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
