// ==============================================================
// Vitis HLS - High-Level Synthesis from C, C++ and OpenCL v2021.1 (64-bit)
// Copyright 1986-2021 Xilinx, Inc. All Rights Reserved.
// ==============================================================
#ifndef __linux__

#include "xstatus.h"
#include "xparameters.h"
#include "xdma.h"

extern XDma_Config XDma_ConfigTable[];

XDma_Config *XDma_LookupConfig(u16 DeviceId) {
	XDma_Config *ConfigPtr = NULL;

	int Index;

	for (Index = 0; Index < XPAR_XDMA_NUM_INSTANCES; Index++) {
		if (XDma_ConfigTable[Index].DeviceId == DeviceId) {
			ConfigPtr = &XDma_ConfigTable[Index];
			break;
		}
	}

	return ConfigPtr;
}

int XDma_Initialize(XDma *InstancePtr, u16 DeviceId) {
	XDma_Config *ConfigPtr;

	Xil_AssertNonvoid(InstancePtr != NULL);

	ConfigPtr = XDma_LookupConfig(DeviceId);
	if (ConfigPtr == NULL) {
		InstancePtr->IsReady = 0;
		return (XST_DEVICE_NOT_FOUND);
	}

	return XDma_CfgInitialize(InstancePtr, ConfigPtr);
}

#endif

