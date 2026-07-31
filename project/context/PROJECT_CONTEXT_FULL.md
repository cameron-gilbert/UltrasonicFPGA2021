# FULL PROJECT CONTEXT DUMP — Motion-Compensated Acoustic Camera

Complete context so any assistant/tool has everything. Verbatim export of all working memory. Last updated 2026-07-26. Cameron = Pacific time, Gazor = Eastern, Dr. Nikaein = Iran (UTC+3:30).

---

## 1. Cameron (user background)
Cameron Gilbert is in a 4+1 accelerated Master's at Queen's University (ELEC 497 research project, summer 2026). He built the capstone this project extends — a working 102-microphone acoustic camera on an AX7010 (Zynq) board — and knows that software deeply. He is in BC (Victoria) until September 2026, so all hardware work is deferred/remote until fall; the summer was purely MATLAB simulation. Email: cameron.gilbert@baroshift.com.

## 2. Working style (preferences — feedback)
Plain language, concise. Short iterative exchanges over monoliths. Never overclaim — describe work as planned unless actually done. Prefer numbered steps in sentence format. Keep MATLAB code toolbox-free where possible (base MATLAB/Octave) unless the repo already depends on toolboxes. Emails must be short — he strongly dislikes long ones. When he asks a question he may have only read up to that point in a prior answer — don't assume he read a whole long output.

## 3. Project overview
Extends Cameron's capstone acoustic camera (102 mics, AX7010 Zynq) into a motion-compensated system. Problem: when the array rotates/translates, the sound-source image smears and shifts because device motion is entangled with source motion. Goal: fuse orientation-sensor data with successive acoustic frames to stabilize the image into a world-fixed frame — cleanly separating "the device moved" from "the source moved."

Downstream interface (Gazor's pipeline output format): a sparse binary pixel grid per frame — each cell is yes/no "source here"; each active cell carries two sub-pixel offsets (exact position within cell) and that source's isolated audio stream (separation assumed done). Motion compensation must keep those entries pointing at correct world locations while the array moves.

Direction check (Cameron + Claude, Jul 17): the motion-comp module sits AFTER separation/localization, consuming the sparse source list. So (a) simulating a dense beamformed image models a stage Gazor considers already solved — the faithful sim would START from the sparse per-source direction+audio list, and motion comp = rotating each source direction into world coordinates; (b) a SINGLE source can't demonstrate the core goal — with MULTIPLE sources, a rotation common to all = device motion, residual per-source motion = true source motion; that common-rotation estimate from successive frames is the slow/driftless signal the proposal fuses against the IMU. Tension: the proposal says "acoustic IMAGE frames" + image-stabilization analogy, so Gazor may accept an image-level view. (This became moot after the Jul 19 pivot away from fusion — see §6.)

Post-summer vision (Gazor, ~Jul 13 email): a second enhanced device, possibly more mics, for two-device collaborative 3D localization + separation.

## 4. Supervision team
- **Professor Gazor (first name Saeed)** — primary supervisor; owns the theoretical math; directed MATLAB-first simulation before any real-time C/C++; grades 70% of the ELEC 497 report. Pivotal instruction: use an integrated off-the-shelf fusion module (cellphone-style, onboard Kalman filtering), do NOT hand-build EKF/orientation filtering — spend effort on the new research. Handles reimbursement: keep receipts → ERS claim.
- **Dr. Nikaein (first name Hossein)** — co-supervisor; hardware/practical implementation. Refer to him as Dr. Nikaein, NEVER "Dr. Hossein." Independently converged with Cameron on the BNO085. At Isfahan University of Technology, Iran (nikaein@iut.ac.ir, UTC+3:30) — 2pm Eastern ≈ 9:30pm his time; afternoon-Eastern meetings = late-evening-Iran. Slower replies may be the time offset.
- **Zack Zhao** — lab colleague; holds the physical hardware in Kingston; shipping it to Cameron.

Communication norm: share simulation plots/results by email as they come.

## 5. Phase 1 simulation (DONE + verified) — in `motion_comp_sim/`
Rotation-only; single-axis yaw; translation explicitly out of scope.
1. Ground-truth motion — DONE: `step1_generate_trajectory.m` — "shaky hand" ±10° yaw sinusoid, 2 Hz, 100 Hz sample rate, 10 s → `trajectory.mat`.
2. Corrupt to sensor readings — DONE: `step2_corrupt_to_sensor.m` — corrupts the angle stream (sensor is an integrated fusion module outputting orientation directly, not raw gyro) with white jitter (0.3° RMS placeholder), random-walk drift (0.05°/√s placeholder), 20 ms latency, quantization → `sensor_readings.mat`. Placeholders until BNO085 bench-measured.
3. Synthetic acoustic frames — DONE: `step3_generate_frames.m` — narrowband single-tone PSF of a fixed world source (az 0°, 8 kHz) seen from the rotating array; real 102-mic geometry + integer-delay steering mirroring `BeamformerWorker`; 937 frames at 93.75 fps on a 101×101 direction-cosine grid → `frames_step3.mat` + `step3_slosh.mp4`.
4. Correction + scoring — DONE: `step4_compensate_and_score.m` — proper spherical counter-rotation of the direction-cosine image, sensor readings interpolated 100 Hz→93.75 fps, four conditions scored vs truth → `results_step4.mat` + `step4_before_after.mp4`.

Results (RMS world-frame position error / avg-image peak): uncompensated 7.09° / 0.31; sensor 1.82° / 0.71; sensor + 20 ms latency comp 0.47° / 0.89; truth 0.41° / 0.89.
KEY FINDING: latency dominates jitter, proven at the image level — compensating the known 20 ms lag recovers nearly the ideal bound. Peak yaw rate ≈126°/s × 20 ms ≈ 2.5°; step-2 error is a clean 2 Hz sinusoid. Implication: correction needs latency estimation/compensation; latency is the first thing to measure on real hardware.

VERIFIED by Claude Jul 17 against the real capstone source: 102 mics, fs 48 kHz (`BeamformerWorker.h` kFs), 512 samples/frame (`MicrophonePacket.h`), 93.75 fps default (scan spinbox 0.001 s → interval 1), geometry formula matches C++ line 37 — all confirmed. Numbers self-consistent (7.09°≈10/√2; 1.82°≈step-2 error). Three honest report caveats: (1) `c=343` hardcoded but the app derives c from temperature (=343 only at ~19.4°C) — state as "343 m/s (≈20°C)"; (2) narrowband single-tone PSF is an agreed modelling simplification, not 512 simulated waveforms — state it; (3) ~1.7° single-frame spikes are a sub-pixel-extractor artifact on resampled images — disclose. Cameron did NOT write steps 1–4 (Claude Code did, ~1 day) and had not reviewed them before this verification pass. "Phase 1 sim pipeline complete" is fair; "simulation work done" is NOT — the fusion filter was the intended research contribution (later set aside — see §6).

Concept notes (worked through with Cameron): results are in degrees of azimuth = the bearing to a source; the image grid is (vx,vy) direction cosines (2 numbers cover all 3D directions, like lat/long on a globe). Single array = direction only, no range; full 3D needs two arrays triangulating. Compensation = rotate each frame back by the measured orientation into a fixed world frame (like phone video stabilization). The "map" isn't pre-existing — it's a coordinate frame anchored to the array's start pose, and the orientation sensor is what builds/maintains it; without the sensor there is no world frame. Deliberate turning doesn't break it (same operation; sources stay world-fixed); only limit is field of view. Sparse grid = a grid using the same coordinates as the beamforming image but only occupied cells stored = equivalently a list of sources each with position+offset+audio; it's the distilled/post-detection version of the dense beamforming image.

## 6. Gazor pivot (Jul 19 email) — hardware integration is now THE deliverable
1. Ultimate goal: accurate device state — x/y/z position, velocity, acceleration, AND orientation → multiple sensors needed.
2. Fusion/tracking: do NOT implement algorithms — use "well-established software tools and frameworks"; from-scratch is "well beyond the scope of ELEC 497." This killed the build-a-complementary-filter plan.
3. Priority for the report period: "temporarily set aside the advanced tracking and fusion stage and concentrate on completing the sensor integration phase" — all sensors operational and collecting data reliably on the current platform. Sim praised as "a solid foundation."
4. Action items: meeting with Gazor (his part "mostly paper works" = admin/ERS); short meeting with Dr. Nikaein on report material.

## 7. Meeting outcomes (Jul 23, Gazor + Nikaein + Cameron)
- Integration approach: modules (BNO085 IMU, u-blox GNSS) attach to the system's networking packets — data joins the existing Ethernet/UDP packet stream (host-side), NOT a separate FPGA firmware path for now. But FIRST test the previously implemented hardware end-to-end before adding modules.
- Conference paper (separate, longer horizon, target ICASSP 2027): Nikaein reframed it as a PLATFORM + EXPERIMENTAL paper — leverage the FPGA acquisition system, focus on experimental capabilities, add a comparative evaluation of beamforming algorithms on real measured data. Motion compensation is NOT the paper's centerpiece. Skeleton in Overleaf (IEEEtran conference class).
- Channel count (clarified by Cameron): NOT a discrepancy — the FPGA pipeline is sized to 128 channels (powers of two) so filters implement efficiently for max speed; only 102 of 128 are populated with mics. Describe as "128-channel FPGA pipeline, 102 mics populated."
- Target venues: ICASSP 2027 — Toronto; paper deadline ~Sept 16 2026; conference May 16–21 2027 (primary). EUSIPCO — Europe (secondary).

## 8. Conference paper skeleton (Overleaf, ICASSP2027, IEEEtran)
1. Introduction (motivation & related work; our contributions)
2. System Architecture (128-channel ultrasonic array; FPGA-based acquisition; synchronization & Ethernet communication)
3. Signal Processing (data preprocessing; delay-and-sum beamforming; optional additional beamforming algorithms)
4. Experimental Setup
5. Experimental Results (localization performance; beam patterns; processing latency; comparison of beamforming methods)
6. Conclusion & Future Work

Differentiators vs the reference papers: most are ACTIVE 40 kHz ultrasonic sonar (transmit + echo); ours is PASSIVE source imaging with a much higher channel count; plus beamforming comparison and (uniquely) the motion-compensation angle.

### Reference papers in `Paperwork/`
Nikaein sent three:
- **Real-Time 3-D Imaging Using an Air-Coupled Ultrasonic Phased-Array** (Allevato et al., TUFFC 2021) — 8×8 40 kHz active array, waveguide→λ/2 spacing, FPGA SoC + GPU, ±80° FOV, 0.5–3 m, 29 fps. Closest match / best structural template.
- **Embedded Air-Coupled Ultrasonic 3D Sonar System with GPU Acceleration** (Allevato et al., IEEE 2020) — 36-mic hexagonal active sonar, Jetson Nano GPU, 30 fps, ±80°, 14° res. Math = delay-and-sum in frequency domain + matched filtering (matched filtering is active-only; not needed for passive). Eq (3) = steering vector (path difference → phase); eq (2) = beamform (matrix mult) then inverse FFT + magnitude.
- **Ultraino: An Open Phased-Array System for Narrowband Airborne Ultrasound** (Marzo et al., TUFFC 2018) — open-hardware 64-ch TRANSMIT array (levitation/haptics). Model for "open modular platform" framing.

Gazor added more:
- **Ultrasonic Phased Array Device for Real-Time Acoustic Imaging in Air** (Harput et al., 2008) — FPGA 6-TX/4-RX beamforming, grating-lobe suppression via geometry.
- **applsci-11-02981 — Generating Airborne Ultrasonic Amplitude Patterns** (Morales/Marzo et al., 2021) — 256-emitter acoustic holography (field-shaping/levitation). Least relevant (transmit-side). NOTE: the highlights in these PDFs were added by a professor (likely Nikaein) as a reading guide, not part of the originals.
- **1-s2.0-S026322412500171X (Li et al., Measurement 2025)** — adaptive beamformer (SRSF-MPAX) real-time on FPGA, benchmarked vs delay-and-sum (sharper FWHM, better contrast, resource usage). Strong template for the beamforming-comparison section. NOT one of Nikaein's three (Gazor added it).

Cameron has a personal interest in the acoustic holography direction (levitation/haptics) — noted to Gazor as a "later" interest, not current focus.

## 9. Hardware inventory
- **AX7010 (Zynq) board + 102-mic array** — capstone platform. 128-channel FPGA pipeline (powers of two), 102 mics populated. fs 48 kHz, 512 samples/frame → 93.75 fps. Shipping from Zack (Kingston → Victoria, Purolator pack-and-ship, express, declared value + signature). ARRIVES ~Jul 27 (shipping delay). Cameron only has the COMPILED platform — needs FPGA source (HDL + Vivado project) from Nikaein to modify the acquisition/packet path.
- **BNO085 orientation sensor** (Adafruit 4754, ~$25) — integrated fusion module, outputs orientation over I²C/SPI/UART. Delivered to the Vancouver house (Jul 20), relaying to Victoria via sister / Harbour Air. Caveat: BNO08x I²C can violate the protocol; UART-RVC is the fallback (needs a soldered mode pin).
- **u-blox SAM-M8Q GNSS** (SparkFun 15210, ~$45) — absolute position/velocity. Ordered → Victoria.
- **MCP2221A** (Adafruit 4471, ~$7) — USB-to-I²C adapter for solderless bench reading. Ordered.
- **STEMMA QT cables** (Adafruit 5384, 300mm) ×2 — solderless connections. Ordered.
- Already had: F/F jumper wires (Adafruit 266) — for later J12 wiring.
- Decided against: barometer (optional z-axis; BMP390 was backordered 18 wks; skip or substitute in-stock Qwiic baro); ZED-F9R (pricey GPS, ~$300 — use later if dead-reckoning needed).
- Bench setup (no tools): sensors → QT cable → MCP2221A → USB → laptop; needs a USB-C DATA cable. Qwiic boards daisy-chain (two ports each). Later (Sept): sensor → J12 PMOD header (I²C into the PS); J12 believed free (mic array uses J10/J11) — confirm with Zack it's unpopulated; on-board version may need a soldered header.

## 10. Deadlines
- **ELEC 497 report — Aug 4, 2026.** IEEE conference format, 5 two-column pages + refs. Graded 70% Gazor, 30% independent reviewer (must stand alone). Rubric: problem/motivation 20%, lit review 20%, methodology/progress/results 45% ("issues encountered" count), report quality 15%. Figures target ~Jul 26; every result = a potential figure (labelled axes, high-res PNGs, reproducible scripts).
- ICASSP 2027 paper — deadline ~Sept 16, 2026.

## 11. Repo pointers (in `AX7010_Work/`)
- `Ultrasonic-Beamforming-real-time-cpp` — the working capstone Qt app (active git repo, has uncommitted changes; Zack may touch it). Contains `matlab_analysis/beamforming.m` etc.; `model/BeamformerWorker.cpp` (real beamforming, integer-delay steering), `model/BeamformerWorker.h` (kFs=48000), `model/MicrophonePacket.h` (SampleCount=512), `microphoneLocations.csv` (102 mics, ±153 mm, ~318 mm diameter), `gui/mainwindow.cpp` (scan interval default → 93.75 fps). Kept read-only from the sim work.
- `motion_comp_sim/` — the summer simulation (created Jul 17, deliberately separate from the Qt repo). Steps 1–4 scripts, `.mat` files, `fig_*.png` (report figures), movies, README, `meeting_prep_jul23.md`.
- `Paperwork/` — the six reference PDFs + IEEEtran template + email drafts.
- Architecture JPGs in the root (pipeline, ps layout, beamforming, ping-pong buffers).

## 12. Email chain (chronological, who owes what)
Early chain: Gazor set the motion-decoupling direction → Cameron's summer plan proposal → Nikaein endorsed MATLAB-first → Gazor's pivotal "use integrated off-the-shelf module, don't build EKFs" → Cameron shortlist (BNO085; ZED-F9R GPS option) → Nikaein independently picked BNO085, raised phone-IMU idea → Cameron: J12 PMOD free, will confirm with Zack → Nikaein: test via J12 independently.
- ~Jul 8–9: Cameron → Gazor: plan agreed with Zack — Zack holds until Jul 10 then ships device to Cameron in BC; meanwhile order IMU + finish MATLAB.
- ~Jul 13: Gazor → Cameron: "Great, lengthy discussion with him"; receipts → ERS; post-summer second-device / 3D vision. Didn't oppose shipping.
- ~Jul 14–16: Zack has a Toronto emergency; hardware "available"; offered to hand-deliver Wed/Thu (assumed Cameron in Kingston). Cameron didn't reply until later — handoff didn't happen.
- Jul 19: Cameron → both: first results email (rotation-only sim, latency dominates). Gazor → THE PIVOT (§6).
- Jul 20–22: Scheduling. Nikaein confirmed modules (basic u-blox GNSS, not ZED-F9R; include barometer) and availability (~2pm ET / late evening). Gazor: meet Thursday, prefers afternoons, "mostly paper works." Zack agreed to pack-and-ship this week (declined prepaid label, will keep receipt; Cameron reimburses).
- Jul 23: three-way meeting (§7). Nikaein shared 3 papers + skeleton; Gazor added more papers to Overleaf.
- Jul 26: Cameron sent reply to Nikaein (agree on direction/structure; asked for FPGA source; hardware arrives tomorrow; offered Teams). Reply to Gazor pending (thanks + which paper to model + holography interest).

Outstanding commitments: (1) results/docs to supervisors as they come — promised "docs covering my MATLAB work" ahead of hardware; (2) keep Dr. Nikaein informed of BNO085 test results; (3) receipts → ERS claim (modules + Zack's shipping); (4) confirm with Zack J12 is unpopulated; (5) get FPGA source from Nikaein; (6) contribute to ICASSP Overleaf.

## 13. Project proposal (verbatim, the operative scope document)
"This project addresses motion compensation for the Queen's 102-microphone acoustic array. When the array moves or rotates, the resulting spatial audio images conflate device motion with source motion, making reliable localization impossible. The goal is to decouple these by fusing onboard IMU data with successive acoustic image frames, analogous to digital image stabilization in cellphone cameras.

The work will begin with a hardware assessment to determine whether the Zynq-7010 platform can be extended with an IMU module, coordinated with Dr Hossein. Algorithm development will then proceed in MATLAB: a complimentary filter will fuse noisy IMU readings with acoustic frame sequences to estimate and compensate for device motion. If results are promising, real-time C/C++ implementation will follow.

The project will be supervised by Professor Gazor and co-mentored by Dr Hossein."
(Note: proposal writes "Dr Hossein" and "complimentary" [sic]; Cameron's preference is "Dr. Nikaein" and it's a complementary filter. The opening hardware-assessment step is why hardware integration became the deliverable after the pivot.)

## 14. Open items / next steps
- Array arrives ~Jul 27 → test existing hardware end-to-end, then integrate sensors host-side (into network packets).
- Get FPGA source (HDL + Vivado) from Nikaein.
- Bench-characterize the BNO085 (real jitter/drift/latency to replace step-2 placeholders); solderless USB setup.
- Draft ELEC 497 report (figures from `motion_comp_sim/`), due Aug 4.
- Contribute to ICASSP Overleaf; confirm with Nikaein which paper to model.
- Keep receipts → ERS reimbursement.
