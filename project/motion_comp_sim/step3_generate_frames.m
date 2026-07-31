% step3_generate_frames.m
% Phase 1, step 3: synthetic beamformed acoustic frames of a FIXED world
% source seen from the ROTATING array (yaw from trajectory.mat).
%
% Agreed simplification: do not simulate 102 raw waveforms. Each frame's
% power map is computed analytically for a narrowband source, using the
% REAL array geometry and the same delay-and-sum math as the real-time app:
%   - geometry:   ../Ultrasonic-Beamforming-real-time-cpp/microphoneLocations.csv
%   - algorithm:  model/BeamformerWorker.cpp — steering grid in direction
%                 cosines (vx,vy) on the unit circle, per-mic integer delay
%                 n_m = round(micPos_samples . dir), power of the sum.
% For a tone at f0 the delay-and-sum power over a frame equals (up to edge
% effects) |sum_m exp(j*2*pi*f0/fs*(n_m(steer) - p_m(source)))|^2, where
% p_m(source) is the exact (unrounded) source delay in samples. This gives
% the true PSF of the array — mainlobe width and sidelobes included.
%
% Real-system parameters mirrored here (verified in the capstone repo):
%   fs = 48 kHz                      (BeamformerWorker.h  kFs)
%   512 samples/mic per frame        (MicrophonePacket.h  SampleCount)
%   default scan every frame -> 93.75 acoustic frames/s
%                                    (mainwindow.cpp m_scanIntervalFrames = 1)
% Note the acoustic frame rate (93.75 Hz) and sensor rate (100 Hz) are NOT
% commensurate — step 4 must interpolate sensor readings to frame times.
%
% Input:  trajectory.mat (step 1)
% Output: frames_step3.mat  (frame stack + frame times + params)
%         fig_step3_geometry.png, fig_step3_psf.png,
%         fig_step3_slosh.png, fig_step3_smear.png   (300 dpi)
%         step3_slosh.mp4 / .avi (skipped gracefully if headless rendering fails)

clear;

% --- parameters ---
csvFile        = '../Ultrasonic-Beamforming-real-time-cpp/microphoneLocations.csv';
f0             = 8000;      % source tone [Hz] (well below grating-lobe onset)
c              = 343;       % speed of sound [m/s] (app default, sosSpin)
fsAudio        = 48000;     % audio sample rate [Hz]
samplesPerFrm  = 512;       % samples per mic per acoustic frame
gridRes        = 0.02;      % direction-cosine grid step (real-time default 0.1;
                            % finer here for report figures, same math)
az0_deg        = 0;         % fixed world source azimuth [deg]
el0_deg        = 0;         % fixed world source elevation [deg]

load('trajectory.mat');     % t, yaw_true_deg, fs, T

% --- real mic geometry ---
raw = readcell(csvFile, 'Delimiter', ',');
xMm = []; yMm = [];
for row = 1:size(raw,1)
    xv = str2double(string(raw{row,2}));
    yv = str2double(string(raw{row,3}));
    if ~isnan(xv) && ~isnan(yv)
        xMm(end+1,1) = xv;  yMm(end+1,1) = yv; %#ok<SAGROW>
    end
end
numMics = numel(xMm);
fprintf('Loaded %d microphones from %s\n', numMics, csvFile);
if numMics ~= 102
    warning('Expected 102 mics, got %d — check CSV.', numMics);
end
% positions in samples-of-delay units, matching BeamformerWorker::recomputePendingLocked
sx = xMm/1000 * fsAudio / c;                 % [numMics x 1]
sy = yMm/1000 * fsAudio / c;

% --- acoustic frame times (mid-frame timestamps) ---
frameDt  = samplesPerFrm / fsAudio;          % 10.667 ms -> 93.75 fps
nFrames  = floor(T / frameDt);
tFrame   = ((0:nFrames-1)' + 0.5) * frameDt;
yawFrame = interp1(t, yaw_true_deg, tFrame, 'linear', 'extrap');  % true yaw at frame times

% --- steering grid, precomputed once (grid is fixed in the array frame) ---
vList = -1:gridRes:1;
nGrid = numel(vList);
[Vx, Vy] = ndgrid(vList, vList);             % row ix = vx, col iy = vy (matches app)
valid = (Vx.^2 + Vy.^2) <= 1;
nValid = nnz(valid);
w0 = 2*pi*f0/fsAudio;                        % phase per sample of delay
% integer-rounded steering delays, exactly as the real-time worker
Nsteer = round(Vx(valid)*sx' + Vy(valid)*sy');   % [nValid x numMics]
Esteer = exp(1i*w0*Nsteer);                      % steering phasors, precomputed

fprintf('Grid %dx%d (res %.2f), %d valid points; %d frames at %.2f fps\n', ...
        nGrid, nGrid, gridRes, nValid, nFrames, 1/frameDt);

% --- render frames ---
% Array yaw psi about the +y (up) axis; a world source at azimuth az0 appears
% in the array frame at azimuth az0 - psi (checked: R_y(psi)' * [0;0;1] = [-sin psi; 0; cos psi]).
frames = zeros(nGrid, nGrid, nFrames, 'single');
azApp_deg = az0_deg - yawFrame;              % apparent source azimuth per frame
for k = 1:nFrames
    vxs = cosd(el0_deg) * sind(azApp_deg(k));    % apparent source direction cosines
    vys = sind(el0_deg);
    p   = sx*vxs + sy*vys;                       % exact source delays [samples]
    s   = exp(1i*w0*p);                          % source phasors
    amp = Esteer * conj(s);                      % [nValid x 1] beamformed amplitude
    img = zeros(nGrid, nGrid, 'single');
    img(valid) = single(abs(amp).^2 / numMics^2);   % 1.0 = perfect focus
    frames(:,:,k) = img;
end

% --- extract apparent blob position per frame (peak + parabolic sub-pixel) ---
% Mirrors Gazor's output format: coarse cell + sub-cell offset.
vxHat = zeros(nFrames,1);
vyHat = zeros(nFrames,1);
for k = 1:nFrames
    [~, idx] = max(frames(:,:,k), [], 'all');
    [ix, iy] = ind2sub([nGrid nGrid], idx);
    ix = min(max(ix,2), nGrid-1);  iy = min(max(iy,2), nGrid-1);
    P = frames(:,:,k);
    dx = 0.5*(P(ix-1,iy)-P(ix+1,iy)) / (P(ix-1,iy)-2*P(ix,iy)+P(ix+1,iy));
    dy = 0.5*(P(ix,iy-1)-P(ix,iy+1)) / (P(ix,iy-1)-2*P(ix,iy)+P(ix,iy+1));
    vxHat(k) = vList(ix) + dx*gridRes;
    vyHat(k) = vList(iy) + dy*gridRes;
end
azHat_deg = asind(vxHat / cosd(el0_deg));
trackErr  = azHat_deg - azApp_deg;
fprintf('Blob tracking check: RMS(extracted - expected azimuth) = %.3f deg\n', ...
        sqrt(mean(trackErr.^2)));

save('-v7', 'frames_step3.mat', 'frames', 'tFrame', 'yawFrame', 'vList', ...
     'gridRes', 'f0', 'c', 'fsAudio', 'samplesPerFrm', 'az0_deg', 'el0_deg', ...
     'vxHat', 'vyHat', 'azHat_deg');

% --- figure: array geometry ---
figure;
plot(xMm, yMm, 'ko', 'MarkerSize', 4, 'MarkerFaceColor', [0.3 0.5 0.9]);
axis equal; grid on;
xlabel('x [mm] (right +)'); ylabel('y [mm] (up +)');
title(sprintf('Microphone array geometry (%d elements)', numMics));
print('-dpng', '-r300', 'fig_step3_geometry.png');

% --- figure: single-frame PSF (dB) at yaw = 0 ---
[~, k0] = min(abs(yawFrame));                % frame nearest zero yaw
img0 = frames(:,:,k0);
figure;
imagesc(vList, vList, 10*log10(max(img0', 1e-6)));  % transpose: rows=vy for display
axis xy equal tight; caxis([-30 0]); colorbar;
xlabel('v_x (right +)'); ylabel('v_y (up +)');
title(sprintf('Array PSF: beamformed power [dB], source at az=%g\\circ, f_0=%g kHz', ...
      az0_deg, f0/1000));
print('-dpng', '-r300', 'fig_step3_psf.png');

% --- figure: blob slosh vs rotation ---
figure;
plot(tFrame, azApp_deg, 'b-', 'LineWidth', 1); hold on;
plot(tFrame, azHat_deg, 'r.', 'MarkerSize', 5);
grid on; xlim([0 2]);
xlabel('Time [s]'); ylabel('Apparent source azimuth [deg]');
legend('Expected: az_0 - yaw(t)', 'Extracted from frames', 'Location', 'northeast');
title(sprintf('Blob position tracks array rotation (RMS mismatch %.3f\\circ)', ...
      sqrt(mean(trackErr.^2))));
print('-dpng', '-r300', 'fig_step3_slosh.png');

% --- figure: smear (why motion compensation is needed) ---
figure;
subplot(1,2,1);
imagesc(vList, vList, img0');
axis xy equal tight; xlim([-0.5 0.5]); ylim([-0.5 0.5]);
xlabel('v_x'); ylabel('v_y');
title('Single frame (array still)');
subplot(1,2,2);
imagesc(vList, vList, mean(frames,3)');
axis xy equal tight; xlim([-0.5 0.5]); ylim([-0.5 0.5]);
xlabel('v_x'); ylabel('v_y');
title('10 s average, uncompensated');
print('-dpng', '-r300', 'fig_step3_smear.png');

% --- movie: blob sloshing (best effort in batch mode) ---
try
    vw = VideoWriter('step3_slosh', 'MPEG-4');
    vw.FrameRate = 30;
    open(vw);
    fh = figure('Position', [100 100 560 520]);
    stride = 3;                              % 93.75 fps content -> ~31 fps movie
    for k = 1:stride:nFrames
        imagesc(vList, vList, frames(:,:,k)');
        axis xy equal tight; xlim([-0.5 0.5]); ylim([-0.5 0.5]); caxis([0 1]);
        xlabel('v_x'); ylabel('v_y');
        title(sprintf('t = %.2f s   yaw = %+.1f\\circ', tFrame(k), yawFrame(k)));
        drawnow;
        writeVideo(vw, getframe(fh));
    end
    close(vw);
    fprintf('Saved step3_slosh movie\n');
catch err
    fprintf('Movie skipped (%s)\n', err.message);
end

fprintf('Saved frames_step3.mat and step-3 figures\n');
