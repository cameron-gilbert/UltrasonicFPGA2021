# Project Proposal — Verbatim

## A. Formal proposal abstract (submitted document)
"This project addresses motion compensation for the Queen's 102-microphone acoustic array. When the array moves or rotates, the resulting spatial audio images conflate device motion with source motion, making reliable localization impossible. The goal is to decouple these by fusing onboard IMU data with successive acoustic image frames, analogous to digital image stabilization in cellphone cameras.

The work will begin with a hardware assessment to determine whether the Zynq-7010 platform can be extended with an IMU module, coordinated with Dr Hossein. Algorithm development will then proceed in MATLAB: a complimentary filter will fuse noisy IMU readings with acoustic frame sequences to estimate and compensate for device motion. If results are promising, real-time C/C++ implementation will follow.

The project will be supervised by Professor Gazor and co-mentored by Dr Hossein."

[sic: "Dr Hossein" — Cameron's preference is "Dr. Nikaein"; "complimentary" should be "complementary". This opening hardware-assessment step is why hardware integration became the deliverable after the Jul 19 pivot.]

## B. Summer plan email (sent to both supervisors, the operative agreement)
"Dear Professor Gazor and Dr. Hossein,

My work obligations have finally settled down, and I've restructured my schedule so I can put time into this project daily going forward. Below is the plan I'd like to propose for the summer.

The idea is to combine two sources of information: the gyroscope and accelerometer give fast estimates of the device's motion but drift over time, while comparing successive acoustic images gives slower estimates that don't drift. A filter that blends the two should give a reliable picture of how the device is moving, so its motion can be removed from the image. A nice side effect is that stationary sound sources will appear to move with the device while truly moving sources won't — which helps separate device motion from source motion, as you described.

Proposed steps, consistent with the proposal I submitted:

1. Hardware check (with Dr. Hossein): confirm the current Zynq/ALinx board can take an IMU. From what I've found, it can — an inexpensive plug-in module should work with no changes to the PCB. Some candidate parts: Adafruit ICM-20948 (~$15), SparkFun ICM-20948 (~$20), Seeed Grove BMI088 (~$25, built for high-vibration use like drones), and a higher-grade option if the budget supports a precise reference unit: Analog Devices ADIS16470 (~$425).
2. MATLAB development (the first focus): build a simulation before touching hardware. I would define a known motion for the array, generate artificial noisy sensor readings and artificial acoustic images from that motion, and then develop the filter and test how accurately it recovers the true motion and stabilizes the image. Since the true motion is known exactly, the results can be measured precisely. I'd start with rotation only, since it is the simpler case, and treat translation as a later stage.
3. Coordination with Zack: since he currently has the hardware, it would be helpful to know what he is working on at the moment so we can plan test scenarios and eventual data collection around his work.
4. If the simulation and real-data results look promising: implement the filter in C/C++ for real-time use in the existing PC application.

I'd also like to get the meeting we discussed on the calendar — would any day this week or next work for you both? I'm available mornings and any time after 2 PM Eastern (my only standing conflict is a daily work meeting from 12:45 to 1:45 PM Eastern)."

[Note: the sensor shortlist here predates the pivot to integrated fusion modules — superseded by Gazor's reply (see emails, B2/B4).]
