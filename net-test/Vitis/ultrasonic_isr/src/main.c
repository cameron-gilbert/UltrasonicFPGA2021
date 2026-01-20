/*
 * Copyright (C) 2009 - 2019 Xilinx, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
 * SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
 * OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
 * OF SUCH DAMAGE.
 *
 */

/* =====================================================================
 * lwIP TCP/IP Stack Integration - Educational Version
 * -----------------------------------------------------------------
 * Purpose:
 *   Initialize lwIP stack, configure network interface, run event loop.
 *   Bridges bare-metal application ↔ lwIP ↔ Ethernet MAC driver.
 *
 * Key Responsibilities:
 *   1. Platform init (caches, interrupts, timers).
 *   2. lwIP stack init (memory pools, protocol layers).
 *   3. Network interface registration (xemac_add → binds MAC driver).
 *   4. IP configuration (DHCP or static).
 *   5. Application start (echo server, etc.).
 *   6. Event loop: poll packets, service timers, app hooks.
 *
 * lwIP Architecture Layers (Bottom-Up):
 *   Hardware: PS GigE MAC (xemacps driver)
 *     ↓
 *   Netif Layer: xemacif adapter (Xilinx glue: xemacif_input, xemacif_output)
 *     ↓
 *   lwIP Core: IP, ICMP, TCP, UDP, DHCP, etc.
 *     ↓
 *   Application: Raw API callbacks (echo.c) or socket API (not used here)
 *
 * Event Loop Model (Polling):
 *   while(1) {
 *       Service lwIP timers (TCP retransmit, DHCP refresh);
 *       Poll for RX packets (xemacif_input);
 *       Application hook (transfer_data);
 *   }
 *   Timer ISR sets flags; main loop checks and calls timer functions.
 *
 * Why No OS:
 *   lwIP raw API = callback-driven, no threads.
 *   Standalone (bare-metal) with manual polling.
 *   For FreeRTOS: use lwIP socket API + sys_arch layer.
 *
 * Future DMA Integration:
 *   Replace transfer_data() hook with DMA buffer transmission logic.
 * ===================================================================== */

#include <stdio.h>

#include "xparameters.h"      /* Hardware definitions (base addresses, IDs) */

#include "netif/xadapter.h"   /* Xilinx lwIP-to-MAC adapter layer */

#include "platform.h"         /* Platform init/cleanup (cache, timers) */
#include "platform_config.h"  /* EMAC base address */
#if defined (__arm__) || defined(__aarch64__)
#include "xil_printf.h"
#endif

#include "lwip/tcp.h"         /* TCP API */
#include "xil_cache.h"        /* Cache management */
#include "packet_timer.h"     /* ISR-based packet timer */
#include "xscugic.h"          /* GIC for interrupt controller */

// Simulation mode: 0 = use simulated data, 1 = read from DDR
#define USE_DDR_READS 0  // Set to 1 when DMA hardware ready

#if LWIP_IPV6==1
#include "lwip/ip.h"
#else
#if LWIP_DHCP==1
#include "lwip/dhcp.h"        /* DHCP client API */
#endif
#endif

/* Application-defined functions (implemented in echo.c or similar). */
void print_app_header();     /* Print application banner */
int start_application();     /* Initialize TCP server / app logic */
void start_data_channel();
int transfer_data();         /* Hook for continuous TX (DMA buffer send) */
void update_stats();         /* Print throughput statistics (call with slow timer) */
void on_500ms_tick();
void stream_scheduler_run();
void stream_stats_run();
void check_reconnection();   /* Attempt reconnection if disconnected */

/* lwIP timer functions (called periodically from main loop). */
void tcp_fasttmr(void);      /* TCP fast timer: 250ms (retransmit, delayed ACK) */
void tcp_slowtmr(void);      /* TCP slow timer: 500ms (persist, keepalive) */

/* lwIP init (defined in lwIP core; no prototype in public headers). */
void lwip_init();

#if LWIP_IPV6==0
#if LWIP_DHCP==1
/* DHCP state: decremented in main loop; triggers fallback to static IP. */
extern volatile int dhcp_timoutcntr;
err_t dhcp_start(struct netif *netif);
#endif
#endif

/* Timer flags: set by platform timer ISR, checked in main loop.
 * Volatile ensures compiler doesn't optimize away checks. */
extern volatile int TcpFastTmrFlag;  /* Set every 250ms */
extern volatile int TcpSlowTmrFlag;  /* Set every 500ms */

/* Network interface structure.
 * Contains IP config, MAC address, driver callbacks, link state, etc. */
static struct netif server_netif;
struct netif *echo_netif;  /* Global pointer for app access */

/* GIC instance - non-static so packet_timer.c can access via extern */
XScuGic gic_inst;

#if LWIP_IPV6==1
void print_ip6(char *msg, ip_addr_t *ip)
{
	print(msg);
	xil_printf(" %x:%x:%x:%x:%x:%x:%x:%x\n\r",
			IP6_ADDR_BLOCK1(&ip->u_addr.ip6),
			IP6_ADDR_BLOCK2(&ip->u_addr.ip6),
			IP6_ADDR_BLOCK3(&ip->u_addr.ip6),
			IP6_ADDR_BLOCK4(&ip->u_addr.ip6),
			IP6_ADDR_BLOCK5(&ip->u_addr.ip6),
			IP6_ADDR_BLOCK6(&ip->u_addr.ip6),
			IP6_ADDR_BLOCK7(&ip->u_addr.ip6),
			IP6_ADDR_BLOCK8(&ip->u_addr.ip6));

}
#else
void
print_ip(char *msg, ip_addr_t *ip)
{
	print(msg);
	xil_printf("%d.%d.%d.%d\n\r", ip4_addr1(ip), ip4_addr2(ip),
			ip4_addr3(ip), ip4_addr4(ip));
}

void
print_ip_settings(ip_addr_t *ip, ip_addr_t *mask, ip_addr_t *gw)
{

	print_ip("Board IP: ", ip);
	print_ip("Netmask : ", mask);
	print_ip("Gateway : ", gw);
}
#endif

#if defined (__arm__) && !defined (ARMR5)
#if XPAR_GIGE_PCS_PMA_SGMII_CORE_PRESENT == 1 || XPAR_GIGE_PCS_PMA_1000BASEX_CORE_PRESENT == 1
int ProgramSi5324(void);
int ProgramSfpPhy(void);
#endif
#endif

#ifdef XPS_BOARD_ZCU102
#ifdef XPAR_XIICPS_0_DEVICE_ID
int IicPhyReset(void);
#endif
#endif

/* =====================================================================
 * main() - Network Application Entry Point
 * -----------------------------------------------------------------
 * Execution Flow:
 *   [Phase 1] Platform Init: Enable caches, GIC, start platform timer.
 *   [Phase 2] lwIP Init: Memory pools, protocol layers, callback setup.
 *   [Phase 3] Network Interface: Register MAC, assign MAC address.
 *   [Phase 4] IP Configuration: DHCP attempt or static fallback.
 *   [Phase 5] Start Application: Bind TCP server, install callbacks.
 *   [Phase 6] Event Loop: Service timers, poll packets, app hooks.
 * ===================================================================== */
int main()
{
#if LWIP_IPV6==0
	ip_addr_t ipaddr, netmask, gw;  /* IPv4 configuration structures */

#endif
	/* MAC Address Configuration:
	 * Each board must have unique MAC to avoid ARP conflicts.
	 * Format: 00:0a:35:00:01:02 (Xilinx OUI: 00:0a:35).
	 * Change last 3 bytes if multiple boards on same network. */
	unsigned char mac_ethernet_address[] =
	{ 0x00, 0x0a, 0x35, 0x00, 0x01, 0x02 };

	echo_netif = &server_netif;  /* Global pointer for app access */
#if defined (__arm__) && !defined (ARMR5)
#if XPAR_GIGE_PCS_PMA_SGMII_CORE_PRESENT == 1 || XPAR_GIGE_PCS_PMA_1000BASEX_CORE_PRESENT == 1
	ProgramSi5324();
	ProgramSfpPhy();
#endif
#endif

/* Define this board specific macro in order perform PHY reset on ZCU102 */
#ifdef XPS_BOARD_ZCU102
	if(IicPhyReset()) {
		xil_printf("Error performing PHY reset \n\r");
		return -1;
	}
#endif

	/* -----------------------------------------------------------------
	 * [Phase 1] Platform Initialization
	 * -----------------------------------------------------------------
	 * What it does:
	 *   - Enable instruction/data caches (performance).
	 *   - Initialize GIC (interrupt controller).
	 *   - Start platform timer (generates TcpFastTmrFlag/TcpSlowTmrFlag).
	 *
	 * Timer ISR Pattern:
	 *   250ms: TcpFastTmrFlag = 1; (main loop calls tcp_fasttmr)
	 *   500ms: TcpSlowTmrFlag = 1; (main loop calls tcp_slowtmr)
	 *
	 * Why timers matter:
	 *   lwIP needs periodic servicing (retransmit, keepalive, DHCP refresh).
	 *   Bare-metal: no OS scheduler, so app manually checks timer flags.
	 * ----------------------------------------------------------------- */
	init_platform();

	//print_app_header();  // Legacy echo server banner - not needed for streaming app
	xil_printf("\r\n[PHASE 1] Platform initialized (caches, GIC, timers)\r\n");

	/* Initialize GIC for frame-ready interrupt from DMA */
	XScuGic_Config *gic_config = XScuGic_LookupConfig(XPAR_SCUGIC_SINGLE_DEVICE_ID);
	if (gic_config == NULL) {
		xil_printf("[ERROR] GIC config lookup failed\r\n");
		return -1;
	}
	int status = XScuGic_CfgInitialize(&gic_inst, gic_config, gic_config->CpuBaseAddress);
	if (status != XST_SUCCESS) {
		xil_printf("[ERROR] GIC init failed\r\n");
		return -1;
	}

	/* Initialize frame interrupt handler (connects to DMA end-of-frame interrupt from PL)
	 * The professor's HLS DMA asserts a fabric interrupt when complete frame is written to DDR.
	 * Frame format: 512 samples × 128 channels (interleaved), 16-bit samples.
	 * Memory layout: S0[ch0..127], S1[ch0..127], ..., S511[ch0..127]
	 * 
	 * TODO: Verify FRAME_INTERRUPT_ID in packet_timer.h matches Vivado block design!
	 */
#if USE_DDR_READS
	if (frame_interrupt_init(&gic_inst) != XST_SUCCESS) {
		xil_printf("[ERROR] Frame interrupt init failed\r\n");
		xil_printf("[ERROR] Check that DMA interrupt is connected in Vivado block design\r\n");
		return -1;
	}
	xil_printf("[PHASE 1] Frame interrupt initialized (DMA end-of-frame ISR)\r\n");
#else
	xil_printf("[PHASE 1] Frame interrupt SKIPPED (simulation mode)\r\n");
#endif

	/* Initialize packet timer (100us interrupts for 10 kHz packet transmission rate)
	 * Works with frame interrupt: timer paces transmission, frame signals data availability
	 */
	if (packet_timer_init(&gic_inst) != XST_SUCCESS) {
		xil_printf("[ERROR] Packet timer init failed\r\n");
		return -1;
	}
	xil_printf("[PHASE 1] Packet timer initialized (100us packet pacing)\r\n");

	/* -----------------------------------------------------------------
	 * [Phase 2] lwIP Stack Initialization
	 * -----------------------------------------------------------------
	 * lwip_init() initializes:
	 *   - Memory pools (PBUF_POOL, MEMP_TCP_PCB, etc.).
	 *   - Protocol layers (IP, ICMP, TCP, UDP).
	 *   - Internal data structures (routing table, TCP timewait queue).
	 *
	 * After this call:
	 *   - lwIP APIs become available (tcp_new, tcp_bind, etc.).
	 *   - No network traffic yet (interface not registered).
	 * ----------------------------------------------------------------- */
	lwip_init();
	xil_printf("[PHASE 2] lwIP stack initialized (memory pools, protocol layers)\r\n");

	/* -----------------------------------------------------------------
	 * [Phase 3] Network Interface Registration
	 * -----------------------------------------------------------------
	 * xemac_add() does:
	 *   1. Allocates netif struct, links to lwIP.
	 *   2. Installs xemacif_input (RX poll), xemacif_output (TX send).
	 *   3. Initializes PS GigE MAC (xemacps driver).
	 *   4. Configures PHY (autonegotiation, link speed).
	 *   5. Sets MAC address in hardware.
	 *
	 * After this:
	 *   - Interface exists but not yet "up" (can't send/recv).
	 *   - Must call netif_set_up() to activate.
	 * ----------------------------------------------------------------- */
#if (LWIP_IPV6 == 0)
	/* IPv4 Mode: Assign IP configuration (static or DHCP placeholder). */
	#if (LWIP_DHCP==1)
	/* DHCP: Start with 0.0.0.0; will be assigned later. */
	xil_printf("\r\n[PHASE 3] Adding network interface (DHCP mode)...\r\n");
	ipaddr.addr = 0;
	gw.addr = 0;
	netmask.addr = 0;
	#else
	/* Static IP: Assign addresses now. */
	xil_printf("\r\n[PHASE 3] Adding network interface (static IP)...\r\n");
	IP4_ADDR(&ipaddr,  192, 168,   1, 10);   /* Board IP */
	IP4_ADDR(&netmask, 255, 255, 255,  0);   /* Subnet mask */
	IP4_ADDR(&gw,      192, 168,   1,  1);   /* Gateway */
	#endif

	/* Register interface with lwIP core. */
	if (!xemac_add(echo_netif, &ipaddr, &netmask,
						&gw, mac_ethernet_address,
						PLATFORM_EMAC_BASEADDR)) {
		xil_printf("Error adding N/W interface\n\r");
		return -1;
	}
#else
	/* IPv6 Mode: Interface auto-assigns link-local address. */
	xil_printf("\r\n[PHASE 3] Adding network interface (IPv6 mode)...\r\n");
	if (!xemac_add(echo_netif, NULL, NULL, NULL, mac_ethernet_address,
						PLATFORM_EMAC_BASEADDR)) {
		xil_printf("Error adding N/W interface\n\r");
		return -1;
	}
	echo_netif->ip6_autoconfig_enabled = 1;

	netif_create_ip6_linklocal_address(echo_netif, 1);
	netif_ip6_addr_set_state(echo_netif, 0, IP6_ADDR_VALID);

	print_ip6("\n\rBoard IPv6 address ", &echo_netif->ip6_addr[0].u_addr.ip6);

#endif
	/* Set this interface as lwIP default (all traffic routes here). */
	netif_set_default(echo_netif);

	/* Enable interrupts globally (GIC now routes timer/MAC events). */
	platform_enable_interrupts();

	/* Bring interface up: enables packet TX/RX in MAC driver. */
	netif_set_up(echo_netif);

#if (LWIP_IPV6 == 0)
	/* -----------------------------------------------------------------
	 * [Phase 4] IP Configuration - DHCP Attempt
	 * -----------------------------------------------------------------
	 * DHCP Client Lifecycle:
	 *   1. dhcp_start() → sends DHCP DISCOVER broadcast.
	 *   2. Poll loop calls xemacif_input() to process DHCP responses.
	 *   3. If DHCP succeeds: echo_netif->ip_addr assigned by server.
	 *   4. If timeout (6 seconds): fallback to static IP.
	 *
	 * Timeout Mechanism:
	 *   - dhcp_timoutcntr = 24 (24 * 250ms = 6 seconds).
	 *   - Decremented in main loop by TcpFastTmrFlag checks.
	 *   - If reaches 0 and still 0.0.0.0 → DHCP failed.
	 *
	 * Why DHCP might fail:
	 *   - No DHCP server on network (direct cable to PC).
	 *   - Network cable unplugged.
	 *   - VLAN/firewall blocking DHCP broadcasts.
	 *
	 * Static Fallback:
	 *   - Assigns 192.168.1.10 directly to netif struct.
	 *   - Must match PC's network settings to communicate.
	 * ----------------------------------------------------------------- */
#if (LWIP_DHCP==1)
	xil_printf("\r\n[PHASE 4] Starting DHCP client...\r\n");
	dhcp_start(echo_netif);
	dhcp_timoutcntr = 24;  /* 24 iterations * 250ms = 6 seconds */

	/* Poll for DHCP response until timeout or success. */
	xil_printf("Waiting for DHCP response (timeout in %d seconds)...\r\n",
			   dhcp_timoutcntr / 4);
	while(((echo_netif->ip_addr.addr) == 0) && (dhcp_timoutcntr > 0))
		xemacif_input(echo_netif);  /* Process RX packets (DHCP replies) */

	/* Check DHCP result. */
	if (dhcp_timoutcntr <= 0) {
		if ((echo_netif->ip_addr.addr) == 0) {
			/* DHCP failed; assign static IP. */
			xil_printf("DHCP Timeout\r\n");
			xil_printf("Configuring default IP of 192.168.1.10\r\n");
			IP4_ADDR(&(echo_netif->ip_addr),  192, 168,   1, 10);
			IP4_ADDR(&(echo_netif->netmask), 255, 255, 255,  0);
			IP4_ADDR(&(echo_netif->gw),      192, 168,   1,  1);
		}
	} else {
		xil_printf("DHCP succeeded! Assigned IP: ");
	}

	/* Copy final IP config to local variables for printing. */
	ipaddr.addr = echo_netif->ip_addr.addr;
	gw.addr = echo_netif->gw.addr;
	netmask.addr = echo_netif->netmask.addr;
#endif

	/* Display active network configuration. */
	print_ip_settings(&ipaddr, &netmask, &gw);

#endif
	/* -----------------------------------------------------------------
	 * [Phase 5] Start Application
	 * -----------------------------------------------------------------
	 * start_application() (defined in echo.c):
	 *   - Starts control channel server on port 6000 (parameter configuration).
	 *
	 * start_data_channel():
	 *   - Initiates TCP client connection to PC on port 5000.
	 *   - Begins automatic data streaming at boot.
	 *
	 * After this:
	 *   - Control server ready for parameter updates.
	 *   - Data streaming active to PC.
	 *   - Main loop services lwIP timers and polls packets.
	 * ----------------------------------------------------------------- */
	xil_printf("\r\n[PHASE 5] Starting application...\r\n");
	start_application();  // Starts port 6000 control channel only
	start_data_channel(); // Start data streaming immediately

	/* -----------------------------------------------------------------
	 * [Phase 6] Main Event Loop
	 * -----------------------------------------------------------------
	 * This infinite loop is the heart of the bare-metal network stack.
	 *
	 * Responsibilities:
	 *   1. Service lwIP Timers:
	 *      - tcp_fasttmr() (250ms): Delayed ACKs, fast retransmit.
	 *      - tcp_slowtmr() (500ms): Persist timer, keepalive, TIME_WAIT.
	 *
	 *   2. Poll for RX Packets:
	 *      - xemacif_input(): Reads packets from MAC RX queue.
	 *      - Injects packets into lwIP stack (IP to TCP/UDP to app callbacks).
	 *
	 *   3. Streaming Scheduler:
	 *      - stream_scheduler_run(): Sends data packets to PC port 5000.
	 *      - Respects TCP flow control (tcp_sndbuf checks).
	 *
	 * Timer Flag Pattern:
	 *   - Platform timer ISR (every 250ms/500ms) sets volatile flags.
	 *   - Main loop checks flags - calls timer functions - clears flags.
	 *   - Why not call timers directly from ISR? Minimize ISR latency.
	 *
	 * Packet Flow (RX - Control Channel):
	 *   Parameter packet arrives (port 6000) - xemacif_input polls
	 *   lwIP processes and control_recv() callback fires
	 *   Update g_parameters[] then Send ACK
	 *
	 * Packet Flow (TX - Data Channel):
	 *   stream_scheduler_run() generates packets -> tcp_write queues
	 *   Then tcp_output sends on xemacif_output and MAC DMA transmits
	 *
	 * Why No Blocking:
	 *   - Bare-metal = no threads, can't afford to block.
	 *   - Polling model ensures responsiveness to all events.
	 *   - For RTOS: use lwIP socket API with blocking calls.
	 * ----------------------------------------------------------------- */
	xil_printf("\r\n[PHASE 6] Entering main event loop...\r\n");
	xil_printf("Control channel ready on port 6000\r\n");
	xil_printf("Data streaming to PC on port 5000\r\n\r\n");

	while (1) {
		/* [Timer Service] Check if fast timer interval elapsed (250ms). */
		if (TcpFastTmrFlag) {
			tcp_fasttmr();      /* Handle delayed ACKs, retransmits */
			TcpFastTmrFlag = 0; /* Clear flag for next interval */
		}

		/* [Timer Service] Check if slow timer interval elapsed (500ms). */
		if (TcpSlowTmrFlag) {
			tcp_slowtmr();      /* Handle persist, keepalive, cleanup */
			//update_stats();     /* Update throughput statistics */
			on_500ms_tick();
			stream_stats_run();
			check_reconnection(); /* Only reconnects if START commanded and connection lost */
			TcpSlowTmrFlag = 0; /* Clear flag for next interval */
		}

		/* [Packet Poll] Check MAC for received packets, inject into lwIP. */
		xemacif_input(echo_netif);

		/* [Dual Interrupt Processing]
		 * Frame interrupt: Marks when new frame is available in DDR
		 * Packet timer: Paces transmission at 10 kHz (100us intervals)
		 * 
		 * Flow: Wait for frame ready, then send one packet per timer tick
		 * 
		 * SIMULATION MODE: If no DMA hardware, timer alone drives transmission
		 */
#if USE_DDR_READS
		// Real mode: require both interrupts
		if (packet_timer_ready() && frame_ready()) {
			stream_scheduler_run();  // Send next mic packet
			packet_timer_clear();
			// frame_clear() called by scheduler when all 102 mics sent
		}
#else
		// Simulation mode: just use timer (no DMA frame interrupt)
		if (packet_timer_ready()) {
			stream_scheduler_run();  // Send next mic packet
			packet_timer_clear();
		}
#endif

		/* [Application Hook] Periodic task for continuous data transmission.
		 * Current: empty placeholder.
		 * Future: Send DMA buffer data to active TCP connections. */
		//transfer_data();
	}

	/* never reached */
	cleanup_platform();

	return 0;
}
