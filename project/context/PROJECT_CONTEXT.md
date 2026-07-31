# Project Context — Motion-Compensated Acoustic Camera (ELEC 497 + ICASSP paper)

Shared context for Cameron's Master's project. Kept here so any tool/assistant has the full picture. Last updated 2026-07-26.

## People
- **Cameron Gilbert** — student, 4+1 accelerated Master's, Queen's University. Built the capstone (the 102-mic acoustic camera). In BC (Victoria) until September 2026; hardware work is remote until then.
- **Professor Gazor (Saeed Gazor)** — primary supervisor; owns the theory; directed MATLAB-first simulation. Handles reimbursement (keep receipts → ERS claim). Eastern time.
- **Dr. Nikaein (Hossein Nikaein)** — co-supervisor; hardware/practical side. Address him as **Dr. Nikaein, never "Dr. Hossein."** At Isfahan University of Technology, Iran (nikaein@iut.ac.ir, UTC+3:30). 2pm Eastern ≈ 9:30pm his time.
- **Zack Zhao** — lab colleague in Kingston who holds the physical array; shipping it to Cameron.

## The project
Extends the capstone acoustic camera (102 mics, AX7010 Zynq board) into a **motion-compensated** system. Problem: when the array rotates/translates, the sound-source image smears because device motion mixes with source motion. Goal: fuse an orientation sensor with successive acoustic frames to stabilize the image into a fixed world frame — separating "the device moved" from "the source moved." Downstream (Gazor's pipeline) format: a sparse grid where each active cell is a source carrying a direction (cell + sub-pixel offset) + its isolated audio (separation assumed already done upstream).

## Current direction (after Gazor's Jul 19 pivot + Jul 23 meeting)
- **Fusion algorithm is set aside** for the ELEC 497 report. Don't build fusion/tracking from scratch — use existing tools later.
- **The deliverable is hardware/sensor integration**: get the sensors (BNO085 IMU, u-blox GNSS) operational and reliably recording data alongside the microphones. Modules will attach to the system's **networking packets** (host-side capture). First step: test the existing hardware end-to-end, then add modules.
- The completed simulation is the validated foundation.

## Phase 1 simulation (DONE, validated) — lives in `motion_comp_sim/`
Rotation-only, single-axis yaw. Four MATLAB steps:
1. `step1_generate_trajectory.m` — known "shaky hand" motion: ±10° yaw, 2 Hz, 100 Hz sampling, 10 s (ground truth).
2. `step2_corrupt_to_sensor.m` — corrupt into realistic sensor readings (jitter, drift, 20 ms latency, quantization). Placeholders until BNO085 is bench-measured.
3. `step3_generate_frames.m` — synthetic beamformed frames of a fixed source seen from the rotating array, using the real 102-mic geometry + delay-and-sum (mirrors `Ultrasonic-Beamforming-real-time-cpp/model/BeamformerWorker.cpp`). 93.75 fps, 101×101 direction-cosine grid.
4. `step4_compensate_and_score.m` — counter-rotate each frame into world coordinates using the sensor readings; score vs truth.

**Key result (world-frame source position error, RMS):** uncompensated 7.09° → sensor-compensated 1.82° → sensor + 20 ms latency compensation 0.47° → perfect-sensor ideal 0.41°.
**Headline:** sensor **latency dominates** the error. Fixing the 20 ms lag recovers nearly the perfect-sensor bound → latency is the first thing to measure on real hardware.

Concept notes: results are in degrees of azimuth (the bearing to a source); the image grid is (vx,vy) direction cosines (2 numbers cover all 3D directions, like lat/long). A single array gives direction only — range/full-3D needs two arrays (Gazor's post-summer two-device vision). Compensation = rotate each frame back by the measured orientation into a fixed world frame (like video stabilization); the "map" is a coordinate frame anchored to the array's start pose, built up by the orientation sensor.

## Hardware
- **AX7010 (Zynq) board + 102-mic array** — the capstone platform. FPGA pipeline sized to **128 channels (powers of two, for efficient high-speed filtering); 102 mics populated.** fs 48 kHz, 512 samples/frame → 93.75 fps. Shipping from Zack (Kingston → Victoria, Purolator pack-and-ship); **arrives ~Jul 27 (delayed)**. Cameron only has the COMPILED platform — needs the FPGA source (HDL + Vivado project) from Nikaein to modify it.
- **BNO085 orientation sensor** (Adafruit 4754) — integrated fusion module, outputs orientation directly (I²C/UART). Delivered to Vancouver, relaying to Victoria. Caveat: BNO08x I²C can violate the protocol; UART-RVC is the fallback (needs a soldered mode pin).
- **u-blox SAM-M8Q GNSS** (SparkFun 15210) — absolute position/velocity. Ordered.
- **MCP2221A** (Adafruit 4471) — USB-to-I²C adapter for solderless bench reading. Ordered.
- **STEMMA QT cables** (Adafruit 5384) ×2 — solderless connections. Ordered.
- Decided against: barometer (optional z-axis, skip unless in stock), ZED-F9R (pricey GPS).
- **Bench setup (no tools):** sensors → QT cable → MCP2221A → USB → laptop. Later (September): sensor → J12 PMOD header on the board (I²C into the PS), which may need a soldered header.

## Conference paper (ICASSP 2027, Toronto)
Separate, longer-horizon deliverable. Target **ICASSP 2027** (Toronto; paper deadline ~Sept 16 2026; conference May 16–21 2027). Secondary venue: **EUSIPCO** (Europe).
- Nikaein's framing: a **platform + experimental** paper — leverage the FPGA acquisition system, characterize experimental capabilities, add a comparative evaluation of beamforming algorithms on real data. NOT a new-algorithm paper. Motion compensation is Cameron's separate ELEC 497 work, not the paper's centerpiece.
- Skeleton (Overleaf, "OurManuscript.tex", IEEEtran conference class): Intro (motivation, contributions) → System Architecture (128-ch array, FPGA acquisition, sync + Ethernet) → Signal Processing (preprocessing, delay-and-sum, optional other beamformers) → Experimental Setup → Experimental Results (localization, beam patterns, latency, beamforming comparison) → Conclusion + Future Work.
- Differentiators vs the reference papers: most of those are **active** 40 kHz ultrasonic sonar (transmit + echo); ours is a **passive** source-imaging array with a much higher channel count; plus the beamforming-comparison and (uniquely) the motion-compensation angle.

### Reference papers in `Paperwork/` (with one-line digests)
Nikaein sent three:
- **Real-Time 3-D Imaging Using an Air-Coupled Ultrasonic Phased-Array** (Allevato et al., TUFFC 2021) — 8×8 40 kHz active array, waveguide for λ/2 spacing, FPGA SoC + GPU processing, ±80° FOV, 0.5–3 m, 29 fps. Closest match / best structural template.
- **Embedded Air-Coupled Ultrasonic 3D Sonar System with GPU Acceleration** (Allevato et al., IEEE 2020) — 36-mic hexagonal active sonar, Jetson Nano GPU beamforming, 30 fps, ±80°, 14° resolution. Math = delay-and-sum in the frequency domain + matched filtering (matched filtering is active-sonar only; not needed for passive).
- **Ultraino: An Open Phased-Array System for Narrowband Airborne Ultrasound** (Marzo et al., TUFFC 2018) — open-hardware 64-ch transmit array (levitation/haptics). Model for "open modular platform" framing.

Gazor added more:
- **Ultrasonic Phased Array Device for Real-Time Acoustic Imaging in Air** (Harput et al., 2008) — FPGA 6-TX/4-RX beamforming, grating-lobe suppression via array geometry.
- **applsci-11-02981 — Generating Airborne Ultrasonic Amplitude Patterns** (Morales/Marzo et al., 2021) — 256-emitter acoustic holography (field-shaping/levitation). Least relevant (transmit-side).
- **1-s2.0-S026322412500171X (Li et al., Measurement 2025)** — adaptive beamformer (SRSF-MPAX) real-time on FPGA, benchmarked vs delay-and-sum (sharper FWHM, better contrast, FPGA resource usage). Strong template for the beamforming-comparison section. (NOT one of Nikaein's three.)

## Deadlines
- **ELEC 497 report — Aug 4, 2026.** IEEE format, 5 two-column pages + refs. Graded 70% Gazor, 30% independent reviewer (must stand alone). Rubric: problem/motivation 20%, lit review 20%, methodology/results 45% (issues encountered count), quality 15%.
- ICASSP 2027 paper — deadline ~Sept 16, 2026.

## Working style (Cameron's preferences)
Plain, concise language; short iterations; never overclaim (describe work as planned unless actually done); numbered steps in sentence form; toolbox-free MATLAB where possible. Emails should be short — he dislikes long ones.

## Open items / next steps
- Array arrives ~Jul 27 → test existing hardware, then integrate sensors host-side.
- Get FPGA source (HDL + Vivado) from Nikaein to modify the acquisition/packet path.
- Bench-characterize the BNO085 (measure real jitter/drift/latency to replace step-2 placeholders).
- Draft ELEC 497 report (figures from `motion_comp_sim/`).
- Contribute to the ICASSP Overleaf; confirm with Nikaein which paper to model most closely.
- Keep receipts (modules, shipping) → ERS reimbursement.
