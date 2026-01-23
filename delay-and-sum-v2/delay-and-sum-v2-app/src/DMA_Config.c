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
    // Enable_Test();      // Test pattern generator (disabled for real microphone data)
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
/* Fractional delay control (for beamforming)                   */
/* ============================================================ */

/**
 * @brief Set fractional delay value for a specific microphone
 * 
 * @param mic_index  Microphone index (0-101)
 * @param delay_value  4-bit fractional delay (0-15)
 */
void Set_Fractional_Delay(u32 mic_index, u32 delay_value)
{
    if (mic_index >= 102) {
        xil_printf("ERROR: Invalid mic_index %u (max 101)\r\n", mic_index);
        return;
    }
    
    // Mask to 4 bits
    delay_value &= 0x0F;
    
    // Write to CFG_WORD register for this microphone
    // Assuming CFG_WORD[mic_index] holds the fractional delay
    u32 cfg_buffer[16] = {0};
    
    // Pack delays: each word can hold delays for multiple mics
    // Assuming 8 delays per word (4 bits each = 32 bits / 4 = 8 delays per word)
    u32 word_index = mic_index / 8;
    u32 nibble_index = mic_index % 8;
    
    if (word_index < 16) {
        // Read current value, modify specific nibble, write back
        // For simplicity, we'll write to the config word directly
        // Note: This assumes the FPGA design has CFG_WORD registers mapped appropriately
        
        // Write the delay value (simplified - assumes direct register mapping)
        // In reality, you may need to pack multiple delays into config words
        xil_printf("[DMA] Set frac delay: Mic %u = %u\r\n", mic_index, delay_value);
        
        // TODO: Implement actual CFG_WORD write based on your FPGA register map
        // This is a placeholder - adjust based on your hardware design
    }
}

/**
 * @brief Get fractional delay value for a specific microphone
 * 
 * @param mic_index  Microphone index (0-101)
 * @return  4-bit fractional delay value (0-15)
 */
u32 Get_Fractional_Delay(u32 mic_index)
{
    if (mic_index >= 102) {
        return 0;
    }
    
    // TODO: Read from CFG_WORD register
    // This is a placeholder
    return 0;
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
