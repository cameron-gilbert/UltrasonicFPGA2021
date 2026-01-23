#ifndef DMA_Configed_H
#define DMA_Configed_H

/***************************** Include Files *********************************/
#include "xparameters_ps.h"
#include "xparameters.h"
#include "xil_printf.h"
#include "xil_cache.h"
#include "xscugic.h"
#include "xil_exception.h"
#include "xdma.h"

/**************************** Type Definitions ******************************/

#define DMA_DEVICE_ID  	XPAR_XDMA_0_DEVICE_ID

#define BLOCK_SIZE     	128   /* each block contain 4 sample from each microphone */
#define One_Frame   	128
#define TOTAL_BLOCKS   	(1 * One_Frame) /* 512 samples per microphone */

/* ------------------------------------------------------------ */
/* Control bits definition                                      */
/* ------------------------------------------------------------ */
#define Sampling_Enable   (1U << 0)
#define Test_Enable       (1U << 1)
#define Simulate_Enable   (1U << 2)
#define Interrupt_En      (1U << 8)
#define Clear_Interrupt   (1U << 16)

/************************** Function Prototypes *****************************/

/* ------------------------------------------------------------ */
/* System setup                                                 */
/* ------------------------------------------------------------ */
int SetupInterruptSystem(void);
int SetupSoundSystem(void);

/* ------------------------------------------------------------ */
/* Control bit APIs                                             */
/* ------------------------------------------------------------ */
void Enable_Sampling(void);
void Disable_Sampling(void);

void Enable_Test(void);
void Disable_Test(void);

void Enable_Simulate(void);
void Disable_Simulate(void);

void Enable_Interrupt(void);
void Disable_Interrupt(void);

void Set_Clear_Interrupt(void);
void Reset_Clear_Interrupt(void);

/* ------------------------------------------------------------ */
/* DDS frequency control                                        */
/* ------------------------------------------------------------ */
void Set_DDS_Frequency(u32 Frequency_khz);
u32  Get_DDS_Frequency_kHz(void);

/* ------------------------------------------------------------ */
/* Fractional delay control (for beamforming)                   */
/* ------------------------------------------------------------ */
void Set_Fractional_Delay(u32 mic_index, u32 delay_value);
u32  Get_Fractional_Delay(u32 mic_index);

/* ------------------------------------------------------------ */
/* CFG_WORD configuration                                      */
/* ------------------------------------------------------------ */
int XDma_Config_Write32(const u32 *cfg_buf);

#endif /* DMA_Configed_H */
