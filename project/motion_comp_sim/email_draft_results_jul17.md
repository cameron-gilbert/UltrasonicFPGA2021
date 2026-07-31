Subject: Motion-compensation sim — rotation-only results + next step

Dear Professor Gazor and Dr. Nikaein,

First results from the MATLAB simulation are attached. Rotation-only for now (single-axis yaw), the simpler case.

Setup: a known array yaw, corrupted into realistic fusion-module readings (jitter, drift, 20 ms latency), then used to counter-rotate synthetic acoustic frames back into world coordinates and scored against the known truth.

Main result: latency dominates. Compensating the known 20 ms lag brings the world-frame source error to 0.47°, against a 0.41° ideal bound — so lag is the first thing to measure on the real sensor. Uncompensated is 7.09°.

Next step, the fusion itself: working in your sparse per-source output format (source separation assumed already done, as you described), with multiple sources. Frame to frame, the rotation common to all sources gives a slow but drift-free estimate of device motion; the module's orientation output is fast but drifts and lags. A complementary filter blends the two — the module carries the fast motion between frames, the acoustic estimate anchors its drift and latency. Any residual per-source motion is then real source motion, giving the device/source separation directly.

Does this match your intent, or would you rather I keep it at the image level?

The BNO085 is on order for bench characterization; in the meantime I'll use a phone's IMU streamed through MATLAB Mobile for real motion traces, following Dr. Nikaein's earlier suggestion.

Best regards,
Cameron

---
Attachments: fig_step4_averages.png, fig_step4_error_vs_time.png
Optional: step4_before_after.mp4
