Subject: RE: Conference paper outline + FPGA source

Dear Dr. Nikaein,

Thanks for the papers and the outline — I've gone through them and the platform + beamforming-comparison direction makes sense to me.

A few thoughts on the papers. Most are active 40 kHz ultrasonic sonar (transmit + echo), while ours is a passive source-imaging array — so our higher channel count and passive operation are natural points to emphasize as differentiators. The Allevato TUFFC papers are a strong structural template for the system and experimental sections, and the Li et al. (Measurement 2025) adaptive-beamforming paper looks like a good model for the beamforming-comparison section — it benchmarks an adaptive method against delay-and-sum on FPGA with resolution/contrast and resource-usage metrics.

On the channel count in the outline: I'd suggest describing the system as a 128-channel FPGA pipeline (sized in powers of two so the filters implement efficiently for maximum speed), with 102 microphones populated — that captures both the architecture and the actual array.

On the acoustic holography paper — I found it genuinely interesting (shaping arbitrary ultrasound fields for levitation and haptics), though I agree it's less central to our imaging focus, so I'll keep it as background.

For next steps, could you share the FPGA source — the HDL and the Vivado project? I currently only have the compiled platform, and I'll need the source to take the design apart and fold the IMU/GNSS data into the acquisition/packet stream. The hardware was delayed slightly in shipping and now arrives tomorrow, so I'd like to get moving on the FPGA side in parallel.

Finally, if there's a particular paper you'd like us to model the write-up on most closely, I'm happy to focus there — and glad to meet over Teams whenever suits you to align on the outline.

Best regards,
Cameron
