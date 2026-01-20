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

u32 XDma_Get_Frame_ID(XDma *InstancePtr) {
    u32 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XDma_ReadReg(InstancePtr->Control_BaseAddress, XDMA_CONTROL_ADDR_FRAME_ID_DATA);
    return Data;
}

u32 XDma_Get_Frame_ID_vld(XDma *InstancePtr) {
    u32 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XDma_ReadReg(InstancePtr->Control_BaseAddress, XDMA_CONTROL_ADDR_FRAME_ID_CTRL);
    return Data & 0x1;
}

