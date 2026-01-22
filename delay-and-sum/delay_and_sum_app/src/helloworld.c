#include <stdio.h>
#include "platform.h"
#include "xil_printf.h"
#include "DMA_Config.h"


extern u8 Bank0[TOTAL_BLOCKS * BLOCK_SIZE * 8];
extern u8 Bank1[TOTAL_BLOCKS * BLOCK_SIZE * 8];

extern volatile u8 Bank0_Available_Flag;
extern volatile u8 Bank1_Available_Flag;
extern u32 Frame_ID;

u16 temp0[BLOCK_SIZE * TOTAL_BLOCKS*(sizeof(u64)/sizeof(u16))];
u16 temp1[BLOCK_SIZE * TOTAL_BLOCKS*(sizeof(u64)/sizeof(u16))];


int main()
{

	init_platform();
	xil_printf("DMA Ping-Pong Test\r\n");
	SetupSoundSystem();
	/* Start DMA here */
	u16 *buf0 = (u16 *)Bank0;
	u16 *buf1 = (u16 *)Bank1;
	int loop_idx=40;
	while (loop_idx) {

		if(Bank0_Available_Flag){

			Bank0_Available_Flag=0;
			for (int i = 0; i < BLOCK_SIZE * TOTAL_BLOCKS*(sizeof(u64)/sizeof(u16)); i++) {
				temp0[i]=buf0[i];
			}
			--loop_idx;
			xil_printf("Loop Index = %d Frame ID= 0x%08x completed\r\n",loop_idx, Frame_ID);
		}
		if(Bank1_Available_Flag){

			Bank1_Available_Flag=0;
			for (int i = 0; i < BLOCK_SIZE * TOTAL_BLOCKS*(sizeof(u64)/sizeof(u16)); i++) {
				temp1[i]=buf1[i];
			}
			--loop_idx;
			xil_printf("Loop Index = %d Frame ID= 0x%08x completed\r\n",loop_idx, Frame_ID);
		}
	}


	xil_printf("********************* Buffer #0   ************************** \r\n");
	for (int i = 0; i < BLOCK_SIZE * TOTAL_BLOCKS*4/32; i++) {
		xil_printf("[%d] = \t 0x%04x \r\n", i, temp0[i]);
	}
	xil_printf("********************* Buffer #1   ************************** \r\n");

	for (int i = 0; i < BLOCK_SIZE * TOTAL_BLOCKS*4/32; i++) {
		xil_printf("[%d] = \t 0x%04x \r\n", i, temp1[i]);
	}

	xil_printf("DMA Ping-Pong test done\r\n");
	cleanup_platform();
	return 0;
}
