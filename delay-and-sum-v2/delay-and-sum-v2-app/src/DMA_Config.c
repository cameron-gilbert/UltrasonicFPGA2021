#include "DMA_Config.h"

/* ------------------------------------------------------------ */
/* Global buffers                                               */
/* ------------------------------------------------------------ */
u8 Bank0[TOTAL_BLOCKS * BLOCK_SIZE * 8] __attribute__ ((aligned (0x100000)));
u8 Bank1[TOTAL_BLOCKS * BLOCK_SIZE * 8] __attribute__ ((aligned (0x100000)));

/* ------------------------------------------------------------ */
/* Global DMA instance                                          */
/* ------------------------------------------------------------ */
XDma Dma;
/* ------------------------------------------------------------ */
/* DDS configuration                                            */
/* ------------------------------------------------------------ */
#define DDS_CLK_KHZ        (48 * 32)   /* 1536 kHz */
#define DDS_MAX_FREQ_KHZ   64U         /* Constraint */

/* ------------------------------------------------------------ */
/* Internal control register shadow                             */
/* ------------------------------------------------------------ */
static u32 Dma_Control_State = 0;

/* ------------------------------------------------------------ */
/* CFG_WORD default values                                      */
/* ------------------------------------------------------------ */

u32 cfg_default[16] = {
    0x76543210, 0xFEDCBA98, 0x76543210, 0xFEDCBA98,
    0x55555555, 0x66666666, 0x77777777, 0x88888888,
    0x99999999, 0xAAAAAAAA, 0xBBBBBBBB, 0xCCCCCCCC,
    0xDDDDDDDD, 0xEEEEEEEE, 0xFFFFFFFF, 0x12345678
};

/* ------------------------------------------------------------ */
/* Setup function                                               */
/* ------------------------------------------------------------ */
int SetupSoundSystem(void)
{
    int Status;

    /* Initialize DMA */
    Status = XDma_Initialize(&Dma, DMA_DEVICE_ID);
    if (Status != XST_SUCCESS) {
        xil_printf("DMA init failed\r\n");
        return XST_FAILURE;
    }

    /* Interrupt system */
    if (SetupInterruptSystem() != XST_SUCCESS) {
        xil_printf("GIC setup failed\r\n");
        return XST_FAILURE;
    }

    /* Clear DDR buffers */
    for (int i = 0; i < BLOCK_SIZE * TOTAL_BLOCKS * 8; i++) {
        Bank0[i] = 0;
        Bank1[i] = 0;
    }

    Xil_DCacheFlushRange((UINTPTR)Bank0,BLOCK_SIZE * TOTAL_BLOCKS * sizeof(u64));
    Xil_DCacheFlushRange((UINTPTR)Bank1,BLOCK_SIZE * TOTAL_BLOCKS * sizeof(u64));

    /* DMA TOTAL_BLOCKS configuration */
    XDma_Set_Total_blocks(&Dma, TOTAL_BLOCKS);

    /* DMA interprets base address as u64 index */
    XDma_Set_Base_Addr_0(&Dma, ((u32)Bank0) / 8);
    XDma_Set_Base_Addr_1(&Dma, ((u32)Bank1) / 8);

    /* Enable required features */
    Enable_Sampling();
    Enable_Test();      // Test pattern generator
    // Enable_Simulate();  // Simulation mode (disabled for real hardware test)
    Enable_Interrupt();
    /* DDS frequency */
    Set_DDS_Frequency(20);   /* 4 kHz */

//    if (XDma_Config_Write32(cfg_default) != XST_SUCCESS) {
//        xil_printf("CFG32 programming failed\r\n");
//    }
    // Disable_Simulate();  // Already disabled above

    xil_printf("DMA configured, waiting for frames...\r\n");
    return XST_SUCCESS;
}

/* ============================================================ */
/* Control bit functions                                        */
/* ============================================================ */

void Enable_Sampling(void)
{
    Dma_Control_State |= Sampling_Enable;
    XDma_Set_Control_In(&Dma, Dma_Control_State);
}

void Disable_Sampling(void)
{
    Dma_Control_State &= ~Sampling_Enable;
    XDma_Set_Control_In(&Dma, Dma_Control_State);
}

void Enable_Test(void)
{
    Dma_Control_State |= Test_Enable;
    XDma_Set_Control_In(&Dma, Dma_Control_State);
}

void Disable_Test(void)
{
    Dma_Control_State &= ~Test_Enable;
    XDma_Set_Control_In(&Dma, Dma_Control_State);
}

void Enable_Simulate(void)
{
    Dma_Control_State |= Simulate_Enable;
    XDma_Set_Control_In(&Dma, Dma_Control_State);
}

void Disable_Simulate(void)
{
    Dma_Control_State &= ~Simulate_Enable;
    XDma_Set_Control_In(&Dma, Dma_Control_State);
}

void Enable_Interrupt(void)
{
    Dma_Control_State |= Interrupt_En;
    XDma_Set_Control_In(&Dma, Dma_Control_State);
}

void Disable_Interrupt(void)
{
    Dma_Control_State &= ~Interrupt_En;
    XDma_Set_Control_In(&Dma, Dma_Control_State);
}

void Set_Clear_Interrupt(void)
{
    Dma_Control_State |= Clear_Interrupt;
    XDma_Set_Control_In(&Dma, Dma_Control_State);
}

void Reset_Clear_Interrupt(void)
{
    Dma_Control_State &= ~Clear_Interrupt;
    XDma_Set_Control_In(&Dma, Dma_Control_State);
}

/* ============================================================ */
/* DDS frequency control                                        */
/* ============================================================ */

/**
 * @brief Set DDS output frequency
 *
 * @param Frequency_khz  0 <= Frequency_khz < 64
 */
void Set_DDS_Frequency(u32 Frequency_khz)
{
    if (Frequency_khz >= DDS_MAX_FREQ_KHZ)
        Frequency_khz = DDS_MAX_FREQ_KHZ - 1;

    u32 phase_inc =(0x10000UL * Frequency_khz) / DDS_CLK_KHZ;

    XDma_Set_simulate_frq_in(&Dma, phase_inc);
}

u32 Get_DDS_Frequency_kHz(void)
{
    u32 phase_inc = XDma_Get_simulate_frq_in(&Dma);

    return (phase_inc * DDS_CLK_KHZ) / 0x10000UL;
}


/* ============================================================ */
/* CFG_WORD write helper                                        */
/* ============================================================ */

/**
 * @brief Write 32-word configuration buffer to CFG_WORD registers
 *
 * @param cfg_buf  Pointer to buffer with exactly 32 u32 words
 *
 * @return XST_SUCCESS on success, XST_FAILURE otherwise
 */
int XDma_Config_Write32(const u32 *cfg_buf)
{
    int written_words;

    if (cfg_buf == NULL)
        return XST_FAILURE;

    written_words = XDma_Write_cfg_word_Words(
                        &Dma,
                        0,          /* offset in words */
                        (u32 *)cfg_buf,
                        16          /* exactly 16 words */
                    );

    if (written_words != 16) {
        xil_printf("CFG_WORD write32 failed! written=%d\r\n",
                   written_words);
        return XST_FAILURE;
    }

    return XST_SUCCESS;
}
