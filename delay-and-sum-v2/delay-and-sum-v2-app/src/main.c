#include <stdio.h>
#include "xparameters.h"
#include "netif/xadapter.h"
#include "platform.h"
#include "platform_config.h"
#if defined (__arm__) || defined(__aarch64__)
#include "xil_printf.h"
#endif

#include "lwip/udp.h"
#include "xil_cache.h"
#include "xscugic.h"
#include "DMA_Config.h"
#include "data_channel.h"
#include "control_channel.h"

// Simulation mode: 0 = use simulated data, 1 = read from DDR
#define USE_DDR_READS 1  // Set to 1 when DMA hardware ready

#if LWIP_IPV6==1
#include "lwip/ip.h"
#else
#if LWIP_DHCP==1
#include "lwip/dhcp.h"        /* DHCP client API */
#endif
#endif

/* lwIP timer functions (called periodically from main loop). */
void tcp_fasttmr(void);      /* TCP fast timer: 250ms (retransmit, delayed ACK) */
void tcp_slowtmr(void);      /* TCP slow timer: 500ms (persist, keepalive) */

extern volatile int TcpFastTmrFlag;  /* Set every 250ms */
extern volatile int TcpSlowTmrFlag;  /* Set every 500ms */

/* lwIP init (defined in lwIP core; no prototype in public headers). */
void lwip_init();

#if LWIP_IPV6==0
#if LWIP_DHCP==1
/* DHCP state: decremented in main loop; triggers fallback to static IP. */
extern volatile int dhcp_timoutcntr;
err_t dhcp_start(struct netif *netif);
#endif
#endif

// Network interface structure.
static struct netif server_netif;
struct netif *echo_netif;  /* Global pointer for app access */
#if USE_DDR_READS
// External references to DMA_Config.c buffers and flags
extern u8 Bank0[TOTAL_BLOCKS * BLOCK_SIZE * 8];
extern u8 Bank1[TOTAL_BLOCKS * BLOCK_SIZE * 8];
extern volatile u8 Bank0_Available_Flag;
extern volatile u8 Bank1_Available_Flag;
extern u32 Frame_ID;
extern volatile u32 Bank0_Frame_ID;
extern volatile u32 Bank1_Frame_ID;
extern XDma Dma;

#endif

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
	 *   - Enable instruction/data caches (performance).
	 *   - Initialize GIC (interrupt controller).
	 *   - Start platform timer (generates TcpFastTmrFlag/TcpSlowTmrFlag).
	 *
	 *   - Initialize DMA system with GIC, interrupts, and ping-pong buffers
	 * ----------------------------------------------------------------- */
	init_platform();
	xil_printf("\r\n[PHASE 1] Platform initialized (caches, GIC, timers)\r\n");
#if USE_DDR_READS
	int Status = SetupSoundSystem();
	if (Status != XST_SUCCESS) {
		xil_printf("[FATAL] DMA system initialization failed! Halting.\r\n");
		return -1;
	}
	xil_printf("[PHASE 1] DMA system initialized (Bank0/Bank1 ping-pong, interrupts enabled)\r\n");
#else
	xil_printf("[PHASE 1] DMA SKIPPED (simulation mode - using test patterns)\r\n");
#endif

	/* -----------------------------------------------------------------
	 * [Phase 2] lwIP Stack Initialization
	 * -----------------------------------------------------------------
	 *   - Memory pools (PBUF_POOL, MEMP_TCP_PCB, etc.).
	 *   - Protocol layers (IP, ICMP, TCP, UDP).
	 *   - Internal data structures (routing table, TCP timewait queue).
	 * ----------------------------------------------------------------- */
	lwip_init();
	xil_printf("[PHASE 2] lwIP stack initialized (memory pools, protocol layers)\r\n");

	/* -----------------------------------------------------------------
	 * [Phase 3] Network Interface Registration
	 * -----------------------------------------------------------------
	 *   - Allocates netif struct, links to lwIP.
	 *   - Installs xemacif_input (RX poll), xemacif_output (TX send).
	 *   - Initializes PS GigE MAC (xemacps driver).
	 *   - Configures PHY (autonegotiation, link speed).
	 *   - Sets MAC address in hardware.
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
	 *   - dhcp_start() sends DHCP DISCOVER broadcast.
	 *   - Poll loop calls xemacif_input() to process DHCP responses.
	 *   - If DHCP succeeds: echo_netif->ip_addr assigned by server.
	 *   - If timeout (6 seconds): fallback to static IP.
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
	 * ----------------------------------------------------------------- */
	xil_printf("\r\n[PHASE 5] Starting application...\r\n");
	start_control_channel();
	start_data_channel();
	xil_printf("\r\n[PHASE 6] Entering main event loop...\r\n");
	xil_printf("Control channel ready on port 6000\r\n");
	xil_printf("Data streaming to PC on port 5000\r\n\r\n");

	while (1) {
		//250ms
		if (TcpFastTmrFlag) {
			tcp_fasttmr();
			TcpFastTmrFlag = 0;
		}

		//500ms
		if (TcpSlowTmrFlag) {
			tcp_slowtmr();
			on_500ms_tick();
			check_reconnection();
			TcpSlowTmrFlag = 0;

		}

	/* [Packet Poll] Check MAC for received packets, inject into lwIP. */
	xemacif_input(echo_netif);

	// Run data channel — check bank flags and send all mic packets
	if (Bank0_Available_Flag) {
	    Bank0_Available_Flag = 0;
	    send_frame_packets((s16 *)Bank0, Bank0_Frame_ID);
	}

	if (Bank1_Available_Flag) {
	    Bank1_Available_Flag = 0;
	    send_frame_packets((s16 *)Bank1, Bank1_Frame_ID);
	}

}

/* never reached */
cleanup_platform();

return 0;
}
