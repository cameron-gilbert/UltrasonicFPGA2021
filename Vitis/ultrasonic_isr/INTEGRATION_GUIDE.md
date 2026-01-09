# Ultrasonic Array - Software Integration Guide

## System Overview

This software implements interrupt-driven data transmission for a 102-microphone ultrasonic array system. The architecture uses dual interrupts for efficient, real-time packet transmission.

---

## Architecture

### **Dual Interrupt Design**

1. **Frame Interrupt** (from FPGA DMA)
   - Triggered when DMA completes writing a frame to DDR
   - Signals: "Buffer ready for transmission"
   - Connected to fabric interrupt line (XPS_FPGA0-15)

2. **Packet Timer** (100µs period)
   - SCU Timer generating 10 kHz interrupts
   - Paces transmission: one mic packet per interrupt
   - Prevents TCP buffer overflow

### **Data Flow**

```
FPGA (PL):
  PDM Mics → CIC/FIR → Time-Multiplexed Processing → HLS DMA
                                                          ↓
                                              DDR (Ping-Pong Buffers)
                                              Buffer A ↔ Buffer B
                                                          ↓
                                                  Frame Interrupt
                                                          ↓
ARM PS:
  Frame ISR → Set frame_ready flag
                      ↓
  Main Loop: Check packet_timer_ready() AND frame_ready()
                      ↓
  Get buffer address from DMA manager
                      ↓
  Every 100µs: De-interleave next mic → Build packet → TCP send
                      ↓
  After 102 mics (10.2ms): Release buffer → Clear frame flag
```

---

## Memory Organization

### **Interleaved Frame Layout (128 KB per buffer)**

```
DDR Address Layout:
┌────────────────────────────────────────────────┐
│ Sample 0:   [ch0][ch1]...[ch126][ch127]       │  256 bytes
│ Sample 1:   [ch0][ch1]...[ch126][ch127]       │  256 bytes
│ Sample 2:   [ch0][ch1]...[ch126][ch127]       │  256 bytes
│ ...                                            │
│ Sample 511: [ch0][ch1]...[ch126][ch127]       │  256 bytes
└────────────────────────────────────────────────┘
Total: 512 samples × 128 channels × 2 bytes = 131,072 bytes
```

### **De-interleaving Algorithm**

To extract samples for microphone N:
```c
for (sample_idx = 0; sample_idx < 512; sample_idx++) {
    offset = (sample_idx * 128 + mic_id) * 2;  // bytes
    mic_samples[sample_idx] = buffer[offset];
}
```

**Stride**: 256 bytes between consecutive samples of same microphone

---

## Integration Points (Professor-Provided Functions)

These functions must be implemented by the DMA hardware control module:

### **1. Get Buffer Address**
```c
uint32_t* dma_get_ready_buffer_address(void);
```
- **Called**: When frame interrupt fires and mic_idx == 0
- **Returns**: DDR address of buffer ready for transmission (Buffer A or B)
- **Purpose**: Software reads from this address to de-interleave frame data

### **2. Get Buffer Size**
```c
uint32_t dma_get_buffer_size(void);
```
- **Called**: During initialization (optional, for diagnostics)
- **Returns**: Size in bytes of each ping-pong buffer
- **Purpose**: Validate buffer configuration

### **3. Release Buffer**
```c
void dma_release_buffer(void);
```
- **Called**: After all 102 mics sent (frame transmission complete)
- **Purpose**: Notify DMA that software finished reading buffer
- **Effect**: DMA can reuse this buffer for next frame write

---

## File Structure

### **Core Files**

| File | Purpose |
|------|---------|
| `main.c` | Initialization, main event loop, interrupt setup |
| `echo.c` | Frame processing, de-interleaving, TCP transmission |
| `packet_timer.c/h` | Dual interrupt handlers (frame + timer) |
| `control_channel.c/h` | TCP port 6000 parameter configuration server |
| `platform_zynq.c` | Platform-specific initialization (cache, GIC, timers) |

### **Key Functions**

#### **Interrupt Handlers**
- `frame_ready_isr()` - Frame complete interrupt from DMA
- `packet_timer_isr()` - 100µs timer interrupt for pacing

#### **Processing Pipeline**
- `stream_scheduler_run()` - Main packet transmission loop
- `read_mic_samples_from_ddr()` - De-interleave single mic from DDR
- `build_mic_packet()` - Construct 1094-byte Ethernet packet
- `send_frame_step()` - TCP flow control and packet transmission

#### **Network**
- `start_data_channel()` - Connect to PC on port 5000 (TCP client)
- `start_control_channel()` - Listen on port 6000 (TCP server)

---

## Configuration

### **Network Settings** (`echo.c`)
```c
#define HOST_IP0 192       // PC IP address
#define HOST_IP1 168
#define HOST_IP2 1
#define HOST_IP3 100       // UPDATE to match your PC
#define HOST_PORT_DATA 5000
```

Board IP: `192.168.1.10` (configured in `main.c`)

### **Frame Parameters** (`packet_timer.h`)
```c
#define SAMPLES_PER_CHANNEL  512   // Samples per mic per frame
#define NUM_CHANNELS         128   // Logical channels (power-of-2)
#define NUM_ACTIVE_MICS      102   // Real microphones
#define BYTES_PER_SAMPLE     2     // 16-bit samples
```

### **Interrupt Configuration** (`packet_timer.h`)
```c
#define FRAME_INTERRUPT_ID   XPS_FPGA0_INT_ID  // UPDATE from Vivado!
#define PACKET_TIMER_INTR_ID XPS_SCU_TMR_INT_ID
#define PACKET_INTERVAL_US   100
```

**CRITICAL**: Verify `FRAME_INTERRUPT_ID` matches the fabric interrupt connection in your Vivado block design (Zynq PS → Interrupts → Fabric Interrupts)

### **DDR Reads** (`echo.c`)
```c
#define USE_DDR_READS 1  // 1 = Real DMA, 0 = Simulation
```

---

## Packet Structure

Each microphone packet = **1094 bytes**:

```
┌─────────────────────────────────────────┐
│ Header (70 bytes)                       │
│  ├─ Frame Number (4 bytes, big-endian) │
│  ├─ Mic Number (2 bytes, big-endian)   │
│  └─ Reserved (64 bytes, zero-filled)   │
├─────────────────────────────────────────┤
│ Payload (1024 bytes)                    │
│  └─ 512 samples × 2 bytes (int16_t BE) │
└─────────────────────────────────────────┘
```

Frame progression: Frame N → Mics 0-101 sent → Frame N+1 → ...

---

## Performance Characteristics

### **Timing**
- **Packet rate**: 10 kHz (100µs intervals)
- **Frame transmission**: 102 packets × 100µs = 10.2 ms
- **Frame period**: ~51.2 ms @ 10 kHz sampling (512 samples)
- **Processing budget**: 10.2 ms < 51.2 ms ✓ (20% duty cycle)

### **Throughput**
- **Per packet**: 1094 bytes
- **Per frame**: 102 × 1094 = 111,588 bytes (109 KB)
- **Data rate**: 111,588 bytes / 51.2 ms = 2.18 MB/s = **17.4 Mbps**

### **TCP Buffer Management**
- **Send buffer**: 16 KB (default lwIP `TCP_SND_BUF`)
- **Capacity**: ~14 packets
- **Flow control**: Automatic via `tcp_sndbuf()` checks

---

## Error Detection

### **Frame Overrun**
```c
if (frame_ready_flag already set) {
    frames_missed++;  // Software too slow!
}
```
**Cause**: Frame processing slower than frame arrival rate  
**Solution**: Optimize de-interleaving, check network congestion

### **TCP Write Failure**
```c
if (tcp_write() != ERR_OK) {
    g_tcp_write_errors++;
}
```
**Cause**: Memory exhaustion or connection broken  
**Solution**: Check `tcp_sndbuf()`, verify TCP connection

---

## Cache Coherency

**CRITICAL**: Software must see DMA-written data in DDR

### **Options**:

1. **Invalidate cache range** (before reading frame):
```c
Xil_DCacheInvalidateRange(buffer_address, 131072);
```

2. **Disable D-cache for DDR region** (linker script):
```
Mark DMA buffer region as non-cacheable
```

3. **Use ACP (Accelerator Coherency Port)** (hardware):
```
DMA writes through ACP → automatic cache invalidation
```

Professor will advise on cache policy during integration.

---

## Build & Deployment

### **Prerequisites**
- Vitis IDE 2021.x (migrated from 2019)
- Vivado-generated XSA with DMA hardware
- Board: Zynq-7000 (e.g., AX7010, ZedBoard)

### **Build Steps**
1. Import project into Vitis workspace
2. Verify BSP includes: lwIP, standalone drivers
3. Link professor's DMA buffer management module
4. Build application (`ultrasonic_isr`)
5. Generate boot image (FSBL + application)

### **Deployment**
1. Program FPGA bitstream (Vivado)
2. Download ELF via JTAG or load from SD card
3. Connect Ethernet cable (direct or via switch)
4. Configure PC IP: `192.168.1.100` (same subnet as `192.168.1.10`)
5. Run host application on PC

---

## Testing Checklist

- [ ] Frame interrupt fires at expected rate (~20 Hz @ 51.2 ms/frame)
- [ ] Packet timer generates 10 kHz interrupts
- [ ] `dma_get_ready_buffer_address()` returns valid addresses
- [ ] Buffer address alternates between A and B (ping-pong)
- [ ] De-interleaved data matches expected microphone patterns
- [ ] TCP connection establishes to PC port 5000
- [ ] 102 packets received per frame on PC
- [ ] Frame numbers increment correctly
- [ ] No frame overruns (`frames_missed == 0`)
- [ ] No TCP errors (`g_tcp_write_errors == 0`)
- [ ] Cache coherency verified (data matches DMA writes)

---

## Debugging

### **UART Debug Output** (115200 baud)
- `[FRAME_INTR]` - Frame interrupt events
- `[PKT_TIMER]` - Packet timer events
- `[DATA]` - TCP connection status
- `[FRAME]` - Buffer address changes
- `[PERF]` - Performance statistics (every 5 seconds)

### **Common Issues**

| Problem | Symptom | Solution |
|---------|---------|----------|
| Frame interrupt never fires | No `[FRAME_INTR]` messages | Check `FRAME_INTERRUPT_ID`, verify Vivado IRQ connection |
| Packet timer not working | No packet transmission | Check SCU timer initialization, GIC enable |
| Buffer address NULL | Error messages | Ensure `dma_get_ready_buffer_address()` linked |
| Stale data from DDR | Incorrect sample values | Add `Xil_DCacheInvalidateRange()` |
| TCP connection fails | No packets on PC | Verify PC IP, firewall, subnet mask |
| Frame overruns | `frames_missed > 0` | Optimize processing, check network congestion |

---

## Contact & Collaboration

**Student**: Cameron  
**Supervisor**: Hossein Nikaein  
**Integration**: Professor provides DMA buffer functions, Cameron integrates with software stack  
**Documentation**: This guide + inline code comments

---

## Revision History

| Date | Version | Changes |
|------|---------|---------|
| 2026-01-09 | 1.0 | Initial documentation for board testing |

---

**Status**: Ready for professor's DMA buffer management integration and board testing.
