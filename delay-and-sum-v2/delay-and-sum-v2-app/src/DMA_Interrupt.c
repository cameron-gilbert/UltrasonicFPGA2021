#include "DMA_Config.h"
#include "platform.h"

/* -------------------------------------------------------------------------- */
/* Interrupt IDs                                                              */
/* -------------------------------------------------------------------------- */
#define INTC_DEVICE_ID   XPAR_SCUGIC_SINGLE_DEVICE_ID

#define DMA_INTR_ID     XPS_FPGA0_INT_ID
#define UART_INTR_ID    XPAR_XUARTPS_0_INTR

/* -------------------------------------------------------------------------- */
/* Priorities (0 = highest, 255 = lowest)                                     */
/* -------------------------------------------------------------------------- */
#define DMA_INTR_PRIORITY   0x5
#define UART_INTR_PRIORITY  0xC0

/* Trigger types */
#define INTR_LEVEL_SENSITIVE  0x1
#define INTR_EDGE_RISING      0x3

/* -------------------------------------------------------------------------- */
/* Global GIC instance (non-static so packet_timer.c can access)              */
/* -------------------------------------------------------------------------- */
XScuGic Intc;
extern XDma Dma;



extern u8 Bank0[TOTAL_BLOCKS * BLOCK_SIZE * 8];
extern u8 Bank1[TOTAL_BLOCKS * BLOCK_SIZE * 8];
/* Flags */
volatile u8 UartFlag     = 0;
volatile u8 Bank0_Available_Flag  = 0;
volatile u8 Bank1_Available_Flag  = 0;
u32 Frame_ID;
volatile u32 Bank0_Frame_ID = 0;
volatile u32 Bank1_Frame_ID = 0;
/* -------------------------------------------------------------------------- */
/* ISRs                                                                       */
/* -------------------------------------------------------------------------- */
void DmaIsr(void *CallbackRef)
{
	Frame_ID = XDma_Get_Frame_Number(&Dma);

	if(Frame_ID & 0x1){	 	// if 1 ==> DMA is writing now in Bank1 and Bank0 is available
		Bank0_Frame_ID = Frame_ID - 1;
		Bank0_Available_Flag=1;
		Xil_DCacheInvalidateRange((INTPTR)Bank0 , BLOCK_SIZE*TOTAL_BLOCKS*sizeof(u64));
	}

	else{					// if 0 ==> DMA is writing now in Bank0 and Bank1 is available
		Bank1_Frame_ID = Frame_ID - 1;
		Bank1_Available_Flag=1;
		Xil_DCacheInvalidateRange((INTPTR)Bank1, BLOCK_SIZE*TOTAL_BLOCKS*sizeof(u64));
	}
}

void UartIsr(void *CallbackRef)
{
	UartFlag = 1;
	/* Clear UART interrupt here */
}

/* -------------------------------------------------------------------------- */
/* Setup function                                                             */
/* -------------------------------------------------------------------------- */
int SetupInterruptSystem(void)
{
	int Status;
	XScuGic_Config *Cfg;

	/* Lookup config */
	Cfg = XScuGic_LookupConfig(INTC_DEVICE_ID);
	if (!Cfg) return XST_FAILURE;

	/* Initialize GIC */
	Status = XScuGic_CfgInitialize(
			&Intc, Cfg, Cfg->CpuBaseAddress);
	if (Status != XST_SUCCESS) return XST_FAILURE;

	/* Init exception handling */
	Xil_ExceptionInit();

	/* Register GIC handler */
	Xil_ExceptionRegisterHandler(
			XIL_EXCEPTION_ID_IRQ_INT,
			(Xil_ExceptionHandler)XScuGic_InterruptHandler,
			&Intc
	);

	/* ---------------- DMA interrupt ---------------- */
	XScuGic_SetPriorityTriggerType(
			&Intc,
			DMA_INTR_ID,
			DMA_INTR_PRIORITY,
			INTR_EDGE_RISING
	);

	XScuGic_Connect(
			&Intc,
			DMA_INTR_ID,
			(Xil_InterruptHandler)DmaIsr,
			NULL
	);

	XScuGic_Enable(&Intc, DMA_INTR_ID);

	/* Re-register the SCU timer with this GIC instance.
	 * platform_setup_interrupts() uses the device API, but this function
	 * replaces the IRQ exception handler with the instance-API dispatcher
	 * (XScuGic_InterruptHandler), which only dispatches handlers registered
	 * against &Intc.  Without this call, the platform timer fires but
	 * TcpSlowTmrFlag is never set and stream_stats_run() never executes.
	 */
	platform_register_timer_with_gic(&Intc);
	/* ---------------- UART interrupt ---------------- */
	//    XScuGic_SetPriorityTriggerType(
	//        &Intc,
	//        UART_INTR_ID,
	//        UART_INTR_PRIORITY,
	//        INTR_LEVEL_SENSITIVE
	//    );
	//
	//    XScuGic_Connect(
	//        &Intc,
	//        UART_INTR_ID,
	//        (Xil_InterruptHandler)UartIsr,
	//        NULL
	//    );

	//    XScuGic_Enable(&Intc, UART_INTR_ID);

	/* Enable IRQ at CPU level */
	Xil_ExceptionEnable();

	return XST_SUCCESS;
}
