// ==============================================================
// Vitis HLS - High-Level Synthesis from C, C++ and OpenCL v2021.1 (64-bit)
// Copyright 1986-2021 Xilinx, Inc. All Rights Reserved.
// ==============================================================
/***************************** Include Files *********************************/
#include "xdma.h"

/************************** Function Implementation *************************/
#ifndef __linux__
int XDma_CfgInitialize(XDma *InstancePtr, XDma_Config *ConfigPtr) {
    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(ConfigPtr != NULL);

    InstancePtr->Control_BaseAddress = ConfigPtr->Control_BaseAddress;
    InstancePtr->IsReady = XIL_COMPONENT_IS_READY;

    return XST_SUCCESS;
}
#endif

void XDma_Set_Total_blocks(XDma *InstancePtr, u32 Data) {
    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    XDma_WriteReg(InstancePtr->Control_BaseAddress, XDMA_CONTROL_ADDR_TOTAL_BLOCKS_DATA, Data);
}

u32 XDma_Get_Total_blocks(XDma *InstancePtr) {
    u32 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XDma_ReadReg(InstancePtr->Control_BaseAddress, XDMA_CONTROL_ADDR_TOTAL_BLOCKS_DATA);
    return Data;
}

void XDma_Set_Control_In(XDma *InstancePtr, u32 Data) {
    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    XDma_WriteReg(InstancePtr->Control_BaseAddress, XDMA_CONTROL_ADDR_CONTROL_IN_DATA, Data);
}

u32 XDma_Get_Control_In(XDma *InstancePtr) {
    u32 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XDma_ReadReg(InstancePtr->Control_BaseAddress, XDMA_CONTROL_ADDR_CONTROL_IN_DATA);
    return Data;
}

void XDma_Set_Base_Addr_0(XDma *InstancePtr, u32 Data) {
    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    XDma_WriteReg(InstancePtr->Control_BaseAddress, XDMA_CONTROL_ADDR_BASE_ADDR_0_DATA, Data);
}

u32 XDma_Get_Base_Addr_0(XDma *InstancePtr) {
    u32 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XDma_ReadReg(InstancePtr->Control_BaseAddress, XDMA_CONTROL_ADDR_BASE_ADDR_0_DATA);
    return Data;
}

void XDma_Set_Base_Addr_1(XDma *InstancePtr, u32 Data) {
    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    XDma_WriteReg(InstancePtr->Control_BaseAddress, XDMA_CONTROL_ADDR_BASE_ADDR_1_DATA, Data);
}

u32 XDma_Get_Base_Addr_1(XDma *InstancePtr) {
    u32 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XDma_ReadReg(InstancePtr->Control_BaseAddress, XDMA_CONTROL_ADDR_BASE_ADDR_1_DATA);
    return Data;
}

u32 XDma_Get_Frame_Number(XDma *InstancePtr) {
    u32 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XDma_ReadReg(InstancePtr->Control_BaseAddress, XDMA_CONTROL_ADDR_FRAME_NUMBER_DATA);
    return Data;
}

u32 XDma_Get_Frame_Number_vld(XDma *InstancePtr) {
    u32 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XDma_ReadReg(InstancePtr->Control_BaseAddress, XDMA_CONTROL_ADDR_FRAME_NUMBER_CTRL);
    return Data & 0x1;
}

void XDma_Set_simulate_frq_in(XDma *InstancePtr, u32 Data) {
    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    XDma_WriteReg(InstancePtr->Control_BaseAddress, XDMA_CONTROL_ADDR_SIMULATE_FRQ_IN_DATA, Data);
}

u32 XDma_Get_simulate_frq_in(XDma *InstancePtr) {
    u32 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XDma_ReadReg(InstancePtr->Control_BaseAddress, XDMA_CONTROL_ADDR_SIMULATE_FRQ_IN_DATA);
    return Data;
}

u32 XDma_Get_cfg_word_BaseAddress(XDma *InstancePtr) {
    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    return (InstancePtr->Control_BaseAddress + XDMA_CONTROL_ADDR_CFG_WORD_BASE);
}

u32 XDma_Get_cfg_word_HighAddress(XDma *InstancePtr) {
    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    return (InstancePtr->Control_BaseAddress + XDMA_CONTROL_ADDR_CFG_WORD_HIGH);
}

u32 XDma_Get_cfg_word_TotalBytes(XDma *InstancePtr) {
    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    return (XDMA_CONTROL_ADDR_CFG_WORD_HIGH - XDMA_CONTROL_ADDR_CFG_WORD_BASE + 1);
}

u32 XDma_Get_cfg_word_BitWidth(XDma *InstancePtr) {
    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    return XDMA_CONTROL_WIDTH_CFG_WORD;
}

u32 XDma_Get_cfg_word_Depth(XDma *InstancePtr) {
    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    return XDMA_CONTROL_DEPTH_CFG_WORD;
}

u32 XDma_Write_cfg_word_Words(XDma *InstancePtr, int offset, word_type *data, int length) {
    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr -> IsReady == XIL_COMPONENT_IS_READY);

    int i;

    if ((offset + length)*4 > (XDMA_CONTROL_ADDR_CFG_WORD_HIGH - XDMA_CONTROL_ADDR_CFG_WORD_BASE + 1))
        return 0;

    for (i = 0; i < length; i++) {
        *(int *)(InstancePtr->Control_BaseAddress + XDMA_CONTROL_ADDR_CFG_WORD_BASE + (offset + i)*4) = *(data + i);
    }
    return length;
}

u32 XDma_Read_cfg_word_Words(XDma *InstancePtr, int offset, word_type *data, int length) {
    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr -> IsReady == XIL_COMPONENT_IS_READY);

    int i;

    if ((offset + length)*4 > (XDMA_CONTROL_ADDR_CFG_WORD_HIGH - XDMA_CONTROL_ADDR_CFG_WORD_BASE + 1))
        return 0;

    for (i = 0; i < length; i++) {
        *(data + i) = *(int *)(InstancePtr->Control_BaseAddress + XDMA_CONTROL_ADDR_CFG_WORD_BASE + (offset + i)*4);
    }
    return length;
}

u32 XDma_Write_cfg_word_Bytes(XDma *InstancePtr, int offset, char *data, int length) {
    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr -> IsReady == XIL_COMPONENT_IS_READY);

    int i;

    if ((offset + length) > (XDMA_CONTROL_ADDR_CFG_WORD_HIGH - XDMA_CONTROL_ADDR_CFG_WORD_BASE + 1))
        return 0;

    for (i = 0; i < length; i++) {
        *(char *)(InstancePtr->Control_BaseAddress + XDMA_CONTROL_ADDR_CFG_WORD_BASE + offset + i) = *(data + i);
    }
    return length;
}

u32 XDma_Read_cfg_word_Bytes(XDma *InstancePtr, int offset, char *data, int length) {
    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr -> IsReady == XIL_COMPONENT_IS_READY);

    int i;

    if ((offset + length) > (XDMA_CONTROL_ADDR_CFG_WORD_HIGH - XDMA_CONTROL_ADDR_CFG_WORD_BASE + 1))
        return 0;

    for (i = 0; i < length; i++) {
        *(data + i) = *(char *)(InstancePtr->Control_BaseAddress + XDMA_CONTROL_ADDR_CFG_WORD_BASE + offset + i);
    }
    return length;
}

