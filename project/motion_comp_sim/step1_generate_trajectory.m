% step1_generate_trajectory.m
% Phase 1, step 1: ground-truth array motion (rotation-only, single-axis yaw).
% "Shaky hand" profile: +/-10 deg yaw sinusoid at 2 Hz, sampled at 100 Hz for 10 s.
% Toolbox-free; runs in base MATLAB or Octave.
%
% Output: trajectory.mat  (t, yaw_true_deg, fs, T, A, f)
%         fig_step1_trajectory.png (300 dpi)

clear;

fs = 100;   % sensor sample rate [Hz]
T  = 10;    % duration [s]
A  = 10;    % yaw amplitude [deg]
f  = 2;     % wobble frequency [Hz]

t = (0:1/fs:T-1/fs)';             % time vector [s], 1000 samples
yaw_true_deg = A*sin(2*pi*f*t);   % ground-truth yaw [deg]

peak_rate = A*2*pi*f;             % analytic peak yaw rate [deg/s]
fprintf('Peak yaw rate: %.1f deg/s\n', peak_rate);

save('-v7', 'trajectory.mat', 't', 'yaw_true_deg', 'fs', 'T', 'A', 'f');

figure;
plot(t, yaw_true_deg, 'b-', 'LineWidth', 1);
grid on;
xlabel('Time [s]');
ylabel('Yaw angle [deg]');
title('Ground-truth array yaw: "shaky hand", \pm10\circ at 2 Hz');
print('-dpng', '-r300', 'fig_step1_trajectory.png');

fprintf('Saved trajectory.mat and fig_step1_trajectory.png\n');
