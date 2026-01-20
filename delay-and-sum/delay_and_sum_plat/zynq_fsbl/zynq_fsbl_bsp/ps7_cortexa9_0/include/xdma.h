// ==============================================================
// Vitis HLS - High-Level Synthesis from C, C++ and OpenCL v2021.1 (64-bit)
// Copyright 1986-2021 Xilinx, Inc. All Rights Reserved.
// ==============================================================
#ifndef XDMA_H
#define XDMA_H

#ifdef __cplusplus
extern "C" {
#endif

/***************************** Include Files *********************************/
#ifndef __linux__
#include "xil_types.h"
#include "xil_assert.h"
#include "xstatus.h"
#include "xil_io.h"
#else
#include <stdint.h>
#include <assert.h>
#include <dirent.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <unistd.h>
#include <stddef.h>
#endif
#include "xdma_hw.h"

/**************************** Type Definitions ******************************/
#ifdef __linux__
typedef uint8_t u8;
typedef uint16_t u16;
typedef uint32_t u32;
typedef uint64_t u64;
#else
typedef struct {
    u16 DeviceId;
    u32 Control_BaseAddress;
} XDma_Config;
#endif

typedef struct {
    u64 Control_BaseAddress;
    u32 IsReady;
} XDma;

typedef u32 word_type;

/***************** Macros (Inline Functions) Definitions *********************/
#ifndef __linux__
#define XDma_WriteReg(BaseAddress, RegOffset, Data) \
    Xil_Out32((BaseAddress) + (RegOffset), (u32)(Data))
#define XDma_ReadReg(BaseAddress, RegOffset) \
    Xil_In32((BaseAddress) + (RegOffset))
#else
#define XDma_WriteReg(BaseAddress, RegOffset, Data) \
    *(volatile u32*)((BaseAddress) + (RegOffset)) = (u32)(Data)
#define XDma_ReadReg(BaseAddress, RegOffset) \
    *(volatile u32*)((BaseAddress) + (RegOffset))

#define Xil_AssertVoid(expr)    assert(expr)
#define Xil_AssertNonvoid(expr) assert(expr)

#define XST_SUCCESS             0
#define XST_DEVICE_NOT_FOUND    2
#define XST_OPEN_DEVICE_FAILED  3
#define XIL_COMPONENT_IS_READY  1
#endif

/************************** Function Prototypes *****************************/
#ifndef __linux__
int XDma_Initialize(XDma *InstancePtr, u16 DeviceId);
XDma_Config* XDma_LookupConfig(u16 DeviceId);
int XDma_CfgInitialize(XDma *InstancePtr, XDma_Config *ConfigPtr);
#else
int XDma_Initialize(XDma *InstancePtr, const char* InstanceName);
int XDma_Release(XDma *InstancePtr);
#endif


void XDma_Set_Total_blocks(XDma *InstancePtr, u32 Data);
u32 XDma_Get_Total_blocks(XDma *InstancePtr);
void XDma_Set_Control_In(XDma *InstancePtr, u32 Data);
u32 XDma_Get_Control_In(XDma *InstancePtr);
void XDma_Set_Base_Addr_0(XDma *InstancePtr, u32 Data);
u32 XDma_Get_Base_Addr_0(XDma *InstancePtr);
void XDma_Set_Base_Addr_1(XDma *InstancePtr, u32 Data);
u32 XDma_Get_Base_Addr_1(XDma *InstancePtr);
u32 XDma_Get_Frame_ID(XDma *InstancePtr);
u32 XDma_Get_Frame_ID_vld(XDma *InstancePtr);

#ifdef __cplusplus
}
#endif

#endif
