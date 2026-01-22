// ==============================================================
// Vitis HLS - High-Level Synthesis from C, C++ and OpenCL v2021.1 (64-bit)
// Copyright 1986-2021 Xilinx, Inc. All Rights Reserved.
// ==============================================================
// control
// 0x00 : reserved
// 0x04 : reserved
// 0x08 : reserved
// 0x0c : reserved
// 0x10 : Data signal of Total_blocks
//        bit 31~0 - Total_blocks[31:0] (Read/Write)
// 0x14 : reserved
// 0x18 : Data signal of Control_In
//        bit 31~0 - Control_In[31:0] (Read/Write)
// 0x1c : reserved
// 0x20 : Data signal of Base_Addr_0
//        bit 31~0 - Base_Addr_0[31:0] (Read/Write)
// 0x24 : reserved
// 0x28 : Data signal of Base_Addr_1
//        bit 31~0 - Base_Addr_1[31:0] (Read/Write)
// 0x2c : reserved
// 0x30 : Data signal of Frame_Number
//        bit 31~0 - Frame_Number[31:0] (Read)
// 0x34 : Control signal of Frame_Number
//        bit 0  - Frame_Number_ap_vld (Read/COR)
//        others - reserved
// 0x80 : Data signal of simulate_frq_in
//        bit 15~0 - simulate_frq_in[15:0] (Read/Write)
//        others   - reserved
// 0x84 : reserved
// 0x40 ~
// 0x7f : Memory 'cfg_word' (16 * 32b)
//        Word n : bit [31:0] - cfg_word[n]
// (SC = Self Clear, COR = Clear on Read, TOW = Toggle on Write, COH = Clear on Handshake)

#define XDMA_CONTROL_ADDR_TOTAL_BLOCKS_DATA    0x10
#define XDMA_CONTROL_BITS_TOTAL_BLOCKS_DATA    32
#define XDMA_CONTROL_ADDR_CONTROL_IN_DATA      0x18
#define XDMA_CONTROL_BITS_CONTROL_IN_DATA      32
#define XDMA_CONTROL_ADDR_BASE_ADDR_0_DATA     0x20
#define XDMA_CONTROL_BITS_BASE_ADDR_0_DATA     32
#define XDMA_CONTROL_ADDR_BASE_ADDR_1_DATA     0x28
#define XDMA_CONTROL_BITS_BASE_ADDR_1_DATA     32
#define XDMA_CONTROL_ADDR_FRAME_NUMBER_DATA    0x30
#define XDMA_CONTROL_BITS_FRAME_NUMBER_DATA    32
#define XDMA_CONTROL_ADDR_FRAME_NUMBER_CTRL    0x34
#define XDMA_CONTROL_ADDR_SIMULATE_FRQ_IN_DATA 0x80
#define XDMA_CONTROL_BITS_SIMULATE_FRQ_IN_DATA 16
#define XDMA_CONTROL_ADDR_CFG_WORD_BASE        0x40
#define XDMA_CONTROL_ADDR_CFG_WORD_HIGH        0x7f
#define XDMA_CONTROL_WIDTH_CFG_WORD            32
#define XDMA_CONTROL_DEPTH_CFG_WORD            16

