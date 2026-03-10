/*
 * Control Channel - Port 6000 TCP Server
 * Protocol:
 *   PC to FPGA: 12-byte control packet [Signature:4][ParamID:4][ParamValue:4]
 *   FPGA to PC: 12-byte ACK packet [Signature:4][ParamID:4][ParamValue:4]
 * 
 * Control Packet (PC to FPGA):
 *   Signature: 0x5555CCCC
 *   ParamID: 0-9 (10 configurable parameters)
 *   ParamValue: 32-bit parameter data
 * 
 * ACK Packet (FPGA to PC):
 *   Signature: 0xAAAA3333
 *   ParamID: Same as received
 *   ParamValue: Current value of parameter on FPGA
 * 
 * NOTE: FPGA acts as SERVER (listens), PC acts as CLIENT (connects)
 */

#include <stdio.h>
#include <string.h>
#include "lwip/err.h"
#include "lwip/tcp.h"
#include "xil_printf.h"
#include "DMA_Config.h"  // For applying fractional delays
#include "control_channel.h"

// Parameter storage (10 parameters: IDs 0-9)
#define NUM_PARAMETERS 10
static uint32_t g_parameters[NUM_PARAMETERS] = {0};

// Fine delay array (16 entries for control vector 0x100-0x10F)
// Each 32-bit word packs fractional delays for multiple mics
static volatile uint32_t g_fine_delay[16] = {0};

static struct tcp_pcb *control_listen_pcb = NULL;
static struct tcp_pcb *control_client_pcb = NULL;
static int control_connected = 0;

// Parse big-endian 32-bit value
static inline uint32_t get_u32_be(const uint8_t *src) {
    return ((uint32_t)src[0] << 24) | ((uint32_t)src[1] << 16) |
           ((uint32_t)src[2] << 8)  | ((uint32_t)src[3]);
}

// Write big-endian 32-bit value
static inline void put_u32_be(uint8_t *dst, uint32_t val) {
    dst[0] = (val >> 24) & 0xFF;
    dst[1] = (val >> 16) & 0xFF;
    dst[2] = (val >> 8) & 0xFF;
    dst[3] = val & 0xFF;
}

//DMA control functions
void Enable_Sampling(void);
void Disable_Sampling(void);
void Enable_Test(void);
void Disable_Test(void);
void Enable_Simulate(void);
void Disable_Simulate(void);

//Forward declarations
static err_t control_accept(void *arg, struct tcp_pcb *newpcb, err_t err);
static err_t control_recv(void *arg, struct tcp_pcb *tpcb,
                          struct pbuf *p, err_t err);
static void control_err(void *arg, err_t err);


// Initialize control channel server on port 6000
int start_control_channel(void)
{
    err_t err;

    if (control_listen_pcb != NULL) {
        xil_printf("[CONTROL] Already listening\r\n");
        return 0;
    }

    control_listen_pcb = tcp_new();
    if (!control_listen_pcb) {
        xil_printf("[CONTROL] ERROR: Failed to create PCB\r\n");
        return -1;
    }

    err = tcp_bind(control_listen_pcb, IP_ADDR_ANY, CONTROL_PORT);
    if (err != ERR_OK) {
        xil_printf("[CONTROL] ERROR: Bind failed on port %d, err=%d\r\n", CONTROL_PORT, err);
        tcp_close(control_listen_pcb);
        control_listen_pcb = NULL;
        return -1;
    }

    control_listen_pcb = tcp_listen(control_listen_pcb);
    if (!control_listen_pcb) {
        xil_printf("[CONTROL] ERROR: Listen failed\r\n");
        return -1;
    }

    tcp_accept(control_listen_pcb, control_accept);
    xil_printf("[CONTROL] Listening on port %d for commands\r\n", CONTROL_PORT);

    return 0;
}

// Accept incoming control connection
static err_t control_accept(void *arg, struct tcp_pcb *newpcb, err_t err)
{
    if (err != ERR_OK || newpcb == NULL) {
        return ERR_VAL;
    }

    xil_printf("[CONTROL] Client connected from %d.%d.%d.%d:%u\r\n",
               ip4_addr1(&newpcb->remote_ip),
               ip4_addr2(&newpcb->remote_ip),
               ip4_addr3(&newpcb->remote_ip),
               ip4_addr4(&newpcb->remote_ip),
               newpcb->remote_port);

    // Only allow one control connection at a time
    if (control_client_pcb != NULL) {
        xil_printf("[CONTROL] WARNING: Closing existing connection\r\n");
        tcp_close(control_client_pcb);
        control_client_pcb = NULL;
    }

    control_client_pcb = newpcb;
    control_connected = 1;

    // Set up callbacks
    tcp_arg(newpcb, NULL);
    tcp_recv(newpcb, control_recv);
    tcp_err(newpcb, control_err);

    return ERR_OK;
}

// Send ACK packet to PC
static void send_ack(struct tcp_pcb *tpcb, uint32_t param_id, uint32_t param_value)
{
    uint8_t ack_packet[CONTROL_PACKET_SIZE];
    
    put_u32_be(ack_packet + 0, SIG_CONTROL_ACK);
    put_u32_be(ack_packet + 4, param_id);
    put_u32_be(ack_packet + 8, param_value);
    
    err_t err = tcp_write(tpcb, ack_packet, CONTROL_PACKET_SIZE, TCP_WRITE_FLAG_COPY);
    if (err == ERR_OK) {
        tcp_output(tpcb);  // Flush immediately
        xil_printf("[CONTROL] Sent ACK: ParamID=%u, Value=0x%08X\r\n", param_id, param_value);
    } else {
        xil_printf("[CONTROL] ERROR: Failed to send ACK, err=%d\r\n", err);
    }
}

// Handle received control packet
static err_t control_recv(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err)
{
    if (err != ERR_OK || p == NULL) {
        if (p != NULL) {
            pbuf_free(p);
        }
        return ERR_OK;
    }

    // Expecting multiples of 12 bytes: [Signature:4][ParamID:4][ParamValue:4] per packet.
    // TCP may coalesce several packets into one segment, so process in a loop.
    if (p->tot_len % CONTROL_PACKET_SIZE != 0) {
        xil_printf("[CONTROL] WARNING: Invalid packet size %u (not a multiple of %u)\r\n",
                   p->tot_len, CONTROL_PACKET_SIZE);
        tcp_recved(tpcb, p->tot_len);
        pbuf_free(p);
        return ERR_OK;
    }

    // Copy entire segment into a flat buffer so we can stride through it safely
    // (pbuf chain may span multiple pbufs)
    uint8_t flat[CONTROL_PACKET_SIZE * 32];  // max 32 coalesced packets
    u16_t copy_len = p->tot_len < sizeof(flat) ? p->tot_len : sizeof(flat);
    pbuf_copy_partial(p, flat, copy_len, 0);
    tcp_recved(tpcb, p->tot_len);
    pbuf_free(p);

    u16_t offset = 0;
    while (offset + CONTROL_PACKET_SIZE <= copy_len) {
        uint8_t *data  = flat + offset;
        uint32_t signature = get_u32_be(data + 0);
        uint32_t param_id  = get_u32_be(data + 4);
        uint32_t param_val = get_u32_be(data + 8);
        offset += CONTROL_PACKET_SIZE;

        if (signature != SIG_CONTROL_REQUEST) {
            xil_printf("[CONTROL] WARNING: Invalid signature 0x%08X (expected 0x%08X)\r\n",
                       signature, SIG_CONTROL_REQUEST);
            continue;
        }

        xil_printf("[CONTROL] Received: Sig=0x%08X, ParamID=%u, Value=0x%08X\r\n",
                   signature, param_id, param_val);

        uint32_t ack_value = param_val;

        switch (param_id) {
                case PID_SAMPLING_ENABLE:
                    if (param_val) {
                        Enable_Sampling();
                        xil_printf("[CONTROL] Sampling ENABLED\r\n");
                    } else {
                        Disable_Sampling();
                        xil_printf("[CONTROL] Sampling DISABLED\r\n");
                    }
                    ack_value = param_val ? 1 : 0;
                    send_ack(tpcb, param_id, ack_value);
                    break;
                    
                case PID_TEST_ENABLE:
                    if (param_val) {
                        Enable_Test();
                        xil_printf("[CONTROL] Test pattern ENABLED\r\n");
                    } else {
                        Disable_Test();
                        xil_printf("[CONTROL] Test pattern DISABLED\r\n");
                    }
                    ack_value = param_val ? 1 : 0;
                    send_ack(tpcb, param_id, ack_value);
                    break;
                    
                case PID_SIM_ENABLE:
                    if (param_val) {
                        Enable_Simulate();
                        xil_printf("[CONTROL] Simulation ENABLED\r\n");
                    } else {
                        Disable_Simulate();
                        xil_printf("[CONTROL] Simulation DISABLED\r\n");
                    }
                    ack_value = param_val ? 1 : 0;
                    send_ack(tpcb, param_id, ack_value);
                    break;
                    
                case PID_SIM_FREQUENCY:
                    if (param_val <= 50) {
                        Set_DDS_Frequency(param_val);
                        xil_printf("[CONTROL] Sim frequency set to %u kHz\r\n", param_val);
                        ack_value = param_val;
                    } else {
                        xil_printf("[CONTROL] Invalid frequency %u (max 50 kHz)\r\n", param_val);
                        ack_value = 0xFFFFFFFF;
                    }
                    send_ack(tpcb, param_id, ack_value);
                    break;
                    
                default:
                    // Handle control vector (0x100-0x10F) - bulk fractional delay write
                    if (param_id >= PID_CTRL_VEC_BASE && param_id < (PID_CTRL_VEC_BASE + 16)) {
                        uint32_t index = param_id - PID_CTRL_VEC_BASE;
                        g_fine_delay[index] = param_val;
                        
                        // Bulk write entire Fine_Delay array to hardware
                        XDma_Config_Write32((uint32_t*)g_fine_delay);
                        
                        xil_printf("[CONTROL] Fine_Delay[%u] = 0x%08X (bulk write)\r\n", index, param_val);
                        send_ack(tpcb, param_id, g_fine_delay[index]);
                    }
                    else {
                        xil_printf("[CONTROL] ERROR: Invalid ParamID 0x%08X\r\n", param_id);
                        send_ack(tpcb, param_id, 0xFFFFFFFF);
                    }
                    break;
            }
    }  // end while

    return ERR_OK;
}

// Handle control channel connection error
static void control_err(void *arg, err_t err)
{
    xil_printf("[CONTROL] ERROR: client disconnected, err=%d\r\n", err);
    control_client_pcb = NULL;
    control_connected = 0;
}

// Check if control channel is connected
int is_control_connected(void)
{
    return control_connected;
}

// Get parameter value by ID
uint32_t get_parameter(uint32_t param_id)
{
    if (param_id < NUM_PARAMETERS) {
        return g_parameters[param_id];
    }
    return 0;  // Invalid ID returns 0
}

// Set parameter value by ID
int set_parameter(uint32_t param_id, uint32_t value)
{
    if (param_id < NUM_PARAMETERS) {
        g_parameters[param_id] = value;
        xil_printf("[CONTROL] Parameter[%u] set to 0x%08X\r\n", param_id, value);
        return 0;
    }
    xil_printf("[CONTROL] ERROR: Invalid ParamID %u\r\n", param_id);
    return -1;
}
