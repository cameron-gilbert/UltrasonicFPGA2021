# Meeting prep — Thu Jul 23, 2 PM ET (with Gazor + Nikaein)

## What the meeting is (low stakes)
Gazor said his part is "mostly paperwork." Add scope + report material with Nikaein. No hardware demo expected. Goal: agree what "integration done" means by Aug 4, sort reimbursement, outline the report.

## Lead with this (your genuine progress)
"I finished the rotation-only motion-compensation simulation end to end, and the main result is that sensor latency dominates the error. Hardware is now inbound and I'm set up to start bench characterization."

## The sim in plain language (own it — it's built on YOUR acoustic camera)
Problem: when the array moves, the sound-source image smears because device motion mixes with source motion. Fix: use an orientation sensor to un-rotate each frame back into world coordinates.
Scope this summer: MATLAB, rotation-only (single-axis yaw). Four steps:
1. Made a known "shaky hand" motion (±10° yaw, 2 Hz, 10 s) as ground truth.
2. Corrupted it into realistic sensor readings — noise, slow drift, 20 ms lag — like a real BNO085 output.
3. Generated synthetic acoustic-camera frames of a fixed source seen from the wobbling array, using the REAL 102-mic geometry + beamforming from the capstone.
4. Un-rotated the frames with the sensor readings; scored against the known truth.

## The one result that matters
World-frame source position error (RMS):
- Uncompensated (blob sloshes): 7.09°
- Un-rotate with raw sensor: 1.82°
- Un-rotate + compensate the 20 ms lag: 0.47°
- Perfect-sensor ideal bound: 0.41°
Takeaway: the 20 ms latency is almost the whole error. So latency is the first thing to measure on the real sensor. That's the headline.

Numbers to have handy: 7.09 -> 1.82 -> 0.47 -> 0.41. Lag 20 ms. Motion ±10 deg @ 2 Hz. Array: 102 mics, 48 kHz, ~94 frames/s.
Be honest: it's rotation-only, a foundation. Translation and multi-source are future.

## Status to report
- Array: Zack shipping it to me this week (Purolator pack-and-ship, I cover cost + receipts).
- BNO085 IMU: delivered to Vancouver, relaying to Victoria.
- Modules ordered: u-blox SAM-M8Q GNSS + MCP2221A USB adapter + cables. Bench setup is solderless (USB).
- Plan: characterize the BNO085 (jitter/drift/latency) on the bench, then read IMU + GNSS into the PC alongside the mic stream.

## Questions to ASK (shows you're driving it)
1. Does host-side USB capture count as "integrated" for the report, or do you want it on the board (J12) by Aug 4?
2. Which sensors are must-have for the report vs nice-to-have? (I have BNO085 + basic u-blox GNSS; barometer optional.)
3. For the report, how much weight on the simulation results vs the hardware integration?
4. Any preferred existing fusion/tracking tool for the later stage, so I note it now?
5. Report is due Aug 4 — confirming you both have that date.

## Paperwork (Gazor's agenda)
- Keep all receipts (modules + Zack's shipping) -> ERS claim -> Gazor reimburses. Confirm the process/what he needs from you.

## Posture
- Lead with what's done, be honest about what's not.
- It's fine to say "I want to confirm scope with you."
- Write down decisions: scope for Aug 4, report structure, reimbursement steps.

## Have open during the call
- This sheet.
- fig_step4_averages.png and fig_step4_error_vs_time.png (in motion_comp_sim/) — share-screen if they want to see results.
