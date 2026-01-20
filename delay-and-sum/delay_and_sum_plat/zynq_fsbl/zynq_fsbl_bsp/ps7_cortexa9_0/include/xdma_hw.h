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
// 0x30 : Data signal of Frame_ID
//        bit 31~0 - Frame_ID[31:0] (Read)
// 0x34 : Control signal of Frame_ID
//        bit 0  - Frame_ID_ap_vld (Read/COR)
//        others - reserved
// (SC = Self Clear, COR = Clear on Read, TOW = Toggle on Write, COH = Clear on Handshake)

#define XDMA_CONTROL_ADDR_TOTAL_BLOCKS_DATA 0x10
#define XDMA_CONTROL_BITS_TOTAL_BLOCKS_DATA 32
#define XDMA_CONTROL_ADDR_CONTROL_IN_DATA   0x18
#define XDMA_CONTROL_BITS_CONTROL_IN_DATA   32
#define XDMA_CONTROL_ADDR_BASE_ADDR_0_DATA  0x20
#define XDMA_CONTROL_BITS_BASE_ADDR_0_DATA  32
#define XDMA_CONTROL_ADDR_BASE_ADDR_1_DATA  0x28
#define XDMA_CONTROL_BITS_BASE_ADDR_1_DATA  32
#define XDMA_CONTROL_ADDR_FRAME_ID_DATA     0x30
#define XDMA_CONTROL_BITS_FRAME_ID_DATA     32
#define XDMA_CONTROL_ADDR_FRAME_ID_CTRL     0x34

