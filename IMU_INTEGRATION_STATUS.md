# IMU Integration — Status & Continuation Notes

**Goal (due Aug 4):** Fuse BNO085 IMU orientation (quaternion) into the mic-array UDP
packets so the host can motion-compensate the acoustic frames.

**Board:** AX7010 (Alinx) Zynq-7010 `xc7z010clg400`. 102-mic array, 128 physical channels,
RGMII Ethernet, lwIP UDP to PC at 192.168.1.100:5000. Vivado/Vitis 2021.

**Firmware we edit:** `delay-and-sum-v2` (this repo, branch `udp-delay-and-sum`). This is the
ONLY firmware target for the project. Do NOT edit UDP_Sources / UDP_App.

---

## Where we are

### DONE
- **Packet layout for IMU metadata.** `imu_meta_t` overlays the existing `mic_packet_t.reserved[56]`
  — packet size unchanged (70-byte header preserved). Big-endian, Q14 quaternion (BNO085 native →
  lossless). Fields: marker `0x494D` ('IM'), valid, accuracy(0-3), quat i/j/k/real (Q14),
  timestamp_us (Global Timer), seq. Validated with a real quaternion round-trip (error 0.00).
  - Firmware copy: `delay-and-sum-v2-app/src/imu_meta_layout.h`
  - Source of truth: `<workspace>/packet_design/imu_meta_layout.h` (+ `validate_imu_meta.py`) — keep in sync.

- **Firmware plumbing (no PL rebuild needed).**
  - `imu_source.h/.c` — latest-sample store + `imu_stamp_reserved()` (writes big-endian `imu_meta_t`
    into `reserved[56]`) + `imu_source_tick_synthetic()` (rotating-yaw Q14 stand-in).
  - `data_channel.c` — `build_mic_packet()` now calls `imu_stamp_reserved(pkt->reserved)` instead of
    zeroing reserved; `send_frame_packets()` drives the synthetic source once per frame under
    `#if IMU_USE_SYNTHETIC`.
  - `IMU_USE_SYNTHETIC = 1` today → board emits a synthetic rotating quaternion on the EXISTING
    bitstream, so the whole path is demonstrable now. Set to 0 once the BNO085 driver publishes.

- **J12 / I²C path verified (from AX7010 User Manual §7).** J12 is a 12-pin PMOD to PS BANK500 MIO
  (3.3 V): PIN1=MIO11, PIN7=MIO10, PIN2=MIO9, PIN10=MIO12. Zynq **I²C0 muxes onto MIO[10:11]** — both
  on J12 → BNO085 on I²C0 via J12 is **PS-side only, no PL involvement**. Dr. Nikaein already approved
  J12. MIO9-12 are free expansion pins (MIO0/13 are the PS LEDs).

### BLOCKED (hardware rebuild)
- The Queensu acquisition Vivado project can't be rebuilt: the custom **`xilinx.com:hls:DMA:1.0`** IP
  ships as only a `.xci` pointer (no packaged core / HLS source), so the block design is **locked** and
  the bitstream / `.xsa` can't be regenerated (e.g. to enable I²C0 cleanly in the PS7 config).
- **Full hardware source requested from Dr. Nikaein** (email sent; draft in `<workspace>/Paperwork/
  email_draft_nikaein_full_hw_source.md`).

---

## Next steps (in order)

1. **Host-side decode** (repo: `Ultrasonic-Beamforming-real-time-cpp`) — parse `imu_meta_t` from the
   packet's reserved[] so we can SEE the synthetic quaternion arriving → proves the pipe end-to-end.

2. **BNO085 driver — Layer 1: I²C0 controller bring-up (no rebuild).** I²C0 is a hardened PS peripheral
   at base `0xE0004000`, present regardless of the `.xsa`. The BSP has NO `xiicps` (I²C disabled in HW),
   so either drop the Xilinx `xiicps` sources into `src/` with a hand-built `XIicPs_Config
   {BaseAddress=0xE0004000, InputClockHz=…}`, or write a ~150-line polled I²C driver. Steps:
   - MIO mux MIO10/11 → I²C0 via SLCR (unlock `0xF8000008=0xDF0D`; `MIO_PIN_10/11` at `0xF8000700+pin*4`).
   - Enable I²C0 clock in `APER_CLK_CTRL` (SLCR `0x12C`); pulse reset in `I2C_RST_CTRL` (SLCR `0x224`).
   - Configure control reg + SCL divisors (~100/400 kHz). Verify by reading BNO085 product ID.

3. **BNO085 driver — Layer 2: SH-2 / SHTP.** BNO085 speaks SHTP (4-byte header: len_lsb, len_msb,
   channel, seq), not plain registers. Reset + drain advertisement → send Set-Feature to enable Rotation
   Vector → read input reports (channel 3, report ID `0x05`) = Q14 i/j/k/real + accuracy. On each read,
   call `imu_source_update(qi,qj,qk,qr,accuracy, XTime_us)` then set `IMU_USE_SYNTHETIC = 0`.
   Poll in the main loop / slow timer (NOT inside `send_frame_packets` — keep the TX burst tight).
   Option: port the CEVA/Adafruit `sh2.c`+`shtp.c` reference driver with a small I²C/delay/time HAL.

4. Latency: `timestamp_us` is stamped at the I²C read — that's the value the host latency-comp uses.

---

## Artifacts that live OUTSIDE this repo (workspace root — copy separately to laptop)
- `packet_design/` — `imu_meta_layout.h` (source of truth) + `validate_imu_meta.py`.
- `motion_comp_sim/` — MATLAB motion-compensation sim (step1-4) + measured BNO085 results.
- `context/` — project context, emails verbatim.
- `Paperwork/` — email drafts (incl. full-HW-source request).
- `sensor_bench/` — BNO085 bench characterization (98.1 Hz, yaw noise 0.026° RMS).

These are NOT under any git repo yet — decide whether to add a dedicated repo or fold into one of
the existing repos.
