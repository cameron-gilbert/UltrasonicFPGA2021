# Email Draft — Cameron → Dr. Nikaein (Request for complete Vivado hardware source)

**Status:** SENT (2026-07-30)

**Subject:** Request for complete Vivado hardware source (custom DMA IP needed to rebuild)

Dear Dr. Nikaein,

Thank you again — receiving the acquisition source was a big step forward, and I've now mapped the full design (the 128-channel demodulator, the PS7 configuration, and the constraints).

I've confirmed the J12 path for the IMU is viable: J12 exposes PS MIO10 and MIO11 (BANK500, 3.3 V), which are exactly the pins Zynq routes to **I²C0**. So the BNO085 can connect over I²C on the PS side, independently of the PL, just as you described.

The one thing blocking me from rebuilding the platform independently is the custom **`xilinx.com:hls:DMA:1.0`** block. The archive I have contains only its `.xci` instance file — there's no packaged IP core, `component.xml`, or HLS source in the folder, so Vivado reports the block design as *locked* and I can't regenerate the bitstream or re-export the `.xsa` (for example, to enable I²C0 in the PS7 config).

To become self-sufficient and avoid round-tripping with you on every hardware change, could you please send the **complete hardware source**, specifically:

1. The full Vivado project **re-archived with IP repositories included** (Project → Archive with the "include IP" / include-all-sources option), **or**
2. The custom DMA IP on its own — either the exported IP core / `ip_repo` folder (with `component.xml`), or the Vitis HLS project (the C++ source + `run_hls.tcl`) so I can regenerate it.

It would also help to know the **Vivado version** you built with, so I can match it locally.

With the complete source I can rebuild the platform, enable I²C0 on MIO10/11, and re-export the `.xsa` myself — which should let me move quickly on the IMU integration ahead of the August 4th report.

Thank you very much for your help.

Best regards,
Cameron
