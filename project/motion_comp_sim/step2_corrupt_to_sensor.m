% step2_corrupt_to_sensor.m
% Phase 1, step 2: corrupt the ground-truth yaw into realistic sensor readings.
% The sensor (BNO085-class integrated fusion module) outputs orientation
% directly, so we corrupt the ANGLE stream, not raw gyro rates.
% Sensor error parameters are BENCH-MEASURED (2026-07-29, BNO085 via MCP2221A),
% except latency, which stays a placeholder until measured on the board.
% Toolbox-free; runs in base MATLAB or Octave.
%
% Input:  trajectory.mat (from step 1)
% Output: sensor_readings.mat (t, yaw_sensor_deg, fs, and the parameters)
%         fig_step2_sensor_vs_truth.png, fig_step2_error.png (300 dpi)

clear;
load('trajectory.mat');   % t, yaw_true_deg, fs

% --- sensor error parameters (BNO085, bench-measured 2026-07-29 via MCP2221A) ---
% Source: sensor_bench/bno085_static.csv, 120 s static capture at 98.1 Hz.
jitter_rms_deg = 0.026;   % static angular noise on yaw output [deg RMS] (measured)
drift_rw_deg   = 0.004;   % random-walk drift [deg/sqrt(s)]; measured <0.05 deg over 2 min (near floor)
latency_s      = 0.020;   % reporting latency [s] -- PLACEHOLDER; measure on-board (dominant term)
quant_step_deg = 0.01;    % output quantization step [deg] (~BNO085 Q14 rotation-vector resolution)

randn('state', 42);       % reproducible; legacy syntax works in MATLAB and Octave

n  = numel(t);
dt = 1/fs;

% 1) Latency: the sensor reports the angle from latency_s ago.
lag = round(latency_s*fs);
yaw_lagged = [repmat(yaw_true_deg(1), lag, 1); yaw_true_deg(1:n-lag)];

% 2) Slow random-walk drift.
drift_deg = cumsum(randn(n,1) * drift_rw_deg * sqrt(dt));

% 3) White jitter.
jitter_deg = randn(n,1) * jitter_rms_deg;

% 4) Quantization of the final output.
yaw_sensor_deg = quant_step_deg * round((yaw_lagged + drift_deg + jitter_deg) / quant_step_deg);

% --- score against the known truth ---
err_deg = yaw_sensor_deg - yaw_true_deg;
err_lag_only = yaw_lagged - yaw_true_deg;
fprintf('Total error:            %.2f deg RMS\n', sqrt(mean(err_deg.^2)));
fprintf('Latency alone:          %.2f deg RMS  (dominant term)\n', sqrt(mean(err_lag_only.^2)));
fprintf('Jitter setting:         %.2f deg RMS\n', jitter_rms_deg);
fprintf('Drift at end of run:    %.2f deg\n', drift_deg(end));

save('-v7', 'sensor_readings.mat', 't', 'yaw_sensor_deg', 'fs', ...
     'jitter_rms_deg', 'drift_rw_deg', 'latency_s', 'quant_step_deg');

% --- figures ---
figure;
plot(t, yaw_true_deg, 'b-', 'LineWidth', 1); hold on;
plot(t, yaw_sensor_deg, 'r-', 'LineWidth', 0.5);
grid on; xlim([0 2]);   % zoom to 2 s so the 20 ms lag is visible
xlabel('Time [s]');
ylabel('Yaw angle [deg]');
legend('Ground truth', 'Sensor reading', 'Location', 'northeast');
title('Sensor reading vs ground truth (first 2 s)');
print('-dpng', '-r300', 'fig_step2_sensor_vs_truth.png');

figure;
plot(t, err_deg, 'k-', 'LineWidth', 0.5);
grid on;
xlabel('Time [s]');
ylabel('Error [deg]');
title(sprintf('Sensor error vs time (%.2f deg RMS, dominated by %d ms latency)', ...
      sqrt(mean(err_deg.^2)), round(latency_s*1000)));
print('-dpng', '-r300', 'fig_step2_error.png');

fprintf('Saved sensor_readings.mat and step-2 figures\n');
