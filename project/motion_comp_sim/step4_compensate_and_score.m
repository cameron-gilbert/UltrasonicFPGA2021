% step4_compensate_and_score.m
% Phase 1, step 4: counter-rotate each synthetic acoustic frame back into
% world coordinates using the CORRUPTED sensor readings, and score against
% the known ground truth.
%
% Compensation conditions compared:
%   (a) uncompensated            — frames as rendered (blob sloshes)
%   (b) sensor-compensated       — counter-rotate by the corrupted reading
%   (c) sensor + latency comp.   — same, but sample the sensor stream
%                                  latency_s later (tests the step-1/2 finding
%                                  that the 20 ms lag dominates the error)
%   (d) truth-compensated        — counter-rotate by the exact yaw (ideal
%                                  bound: what perfect sensing would give)
%
% Counter-rotation is a proper spherical rotation of the direction-cosine
% image, not a pixel shift: a world direction w = (vx, vy, vz) appears in the
% array frame at a = R_y(psi)' * w, so the world-frame image is
%   I_world(vx,vy) = I_array(cos(psi)*vx - sin(psi)*vz,  vy)
% sampled bilinearly. Sensor readings (100 Hz) are interpolated to the
% acoustic frame times (93.75 fps) — the rates are not commensurate.
%
% Input:  frames_step3.mat, sensor_readings.mat, trajectory.mat
% Output: results_step4.mat
%         fig_step4_error_vs_time.png, fig_step4_averages.png (300 dpi)
%         step4_before_after.mp4 (best effort in batch mode)

clear;

load('frames_step3.mat');    % frames, tFrame, yawFrame, vList, gridRes, az0_deg, el0_deg
load('sensor_readings.mat'); % t, yaw_sensor_deg, fs, latency_s
truth = load('trajectory.mat');

nGrid   = numel(vList);
nFrames = size(frames, 3);

% --- yaw estimates at acoustic frame times ---
yawSens = interp1(t, yaw_sensor_deg, tFrame, 'linear', 'extrap');            % (b)
yawLat  = interp1(t, yaw_sensor_deg, tFrame + latency_s, 'linear', 'extrap');% (c)
yawTrue = yawFrame;                                                          % (d) exact, from step 3

% --- world-frame grid, precomputed ---
[VxW, VyW] = ndgrid(vList, vList);
validW = (VxW.^2 + VyW.^2) <= 1;
VzW = zeros(nGrid);
VzW(validW) = sqrt(max(0, 1 - VxW(validW).^2 - VyW(validW).^2));  % max(0,·): guard fp residue at the rim

conds = {'uncompensated', 'sensor', 'sensor+latency', 'truth'};
yawByCond = {zeros(nFrames,1), yawSens, yawLat, yawTrue};
nCond = numel(conds);

avgImg = zeros(nGrid, nGrid, nCond, 'single');   % time-averaged world image
azErr  = zeros(nFrames, nCond);                  % extracted world azimuth - az0
sensorCompStack = zeros(nGrid, nGrid, nFrames, 'single');  % kept for the movie

for ci = 1:nCond
    psiAll = yawByCond{ci};
    for k = 1:nFrames
        img = double(frames(:,:,k));
        psi = psiAll(k);
        if psi == 0
            imgW = single(img);
        else
            % world grid point -> where it sits in the array-frame image
            VxA = cosd(psi)*VxW - sind(psi)*VzW;   % vy unchanged (yaw about y)
            F = griddedInterpolant({vList, vList}, img, 'linear', 'none');
            imgW = zeros(nGrid, nGrid);
            imgW(validW) = F(VxA(validW), VyW(validW));
            imgW(~isfinite(imgW)) = 0;
            imgW = single(imgW);
        end
        avgImg(:,:,ci) = avgImg(:,:,ci) + imgW / nFrames;
        [vxh, ~] = peakSubpixel(imgW, vList);
        azErr(k, ci) = asind(vxh / cosd(el0_deg)) - az0_deg;
        if ci == 2
            sensorCompStack(:,:,k) = imgW;
        end
    end
end

% --- metrics ---
% Focus quality: peak of the time-averaged image (single sharp frame peaks at ~1;
% smearing spreads energy and lowers it).
fprintf('\n%-18s %14s %18s\n', 'Condition', 'RMS err [deg]', 'avg-image peak');
rmsErr = zeros(1,nCond); pk = zeros(1,nCond);
for ci = 1:nCond
    rmsErr(ci) = sqrt(mean(azErr(:,ci).^2));
    pk(ci)     = max(avgImg(:,:,ci), [], 'all');
    fprintf('%-18s %14.3f %18.3f\n', conds{ci}, rmsErr(ci), pk(ci));
end

save('-v7', 'results_step4.mat', 'tFrame', 'azErr', 'rmsErr', 'pk', 'conds', ...
     'avgImg', 'vList', 'latency_s', 'az0_deg');

% --- figure: position error vs time ---
figure('Position', [100 100 900 700]);
subplot(2,1,1);
plot(tFrame, azErr(:,1), 'Color', [0.5 0.5 0.5]);
grid on;
ylabel('Error [deg]');
title(sprintf('(a) Uncompensated: blob wanders with the array (%.2f\\circ RMS)', rmsErr(1)));
subplot(2,1,2);
plot(tFrame, azErr(:,2), 'r-', 'LineWidth', 0.75); hold on;
plot(tFrame, azErr(:,3), 'b-', 'LineWidth', 0.75);
plot(tFrame, azErr(:,4), 'k-', 'LineWidth', 0.75);
grid on;
xlabel('Time [s]'); ylabel('Error [deg]');
legend(sprintf('(b) sensor (%.2f\\circ RMS)', rmsErr(2)), ...
       sprintf('(c) sensor + %d ms latency comp. (%.2f\\circ RMS)', round(latency_s*1000), rmsErr(3)), ...
       sprintf('(d) truth (%.2f\\circ RMS)', rmsErr(4)), 'Location', 'northeast');
title('Compensated: world-frame source position error');
print('-dpng', '-r300', 'fig_step4_error_vs_time.png');

% --- figure: time-averaged images, before/after ---
figure('Position', [100 100 1000 900]);
panelTitles = { ...
    sprintf('(a) Uncompensated (peak %.2f)', pk(1)), ...
    sprintf('(b) Sensor-compensated (peak %.2f)', pk(2)), ...
    sprintf('(c) + latency compensation (peak %.2f)', pk(3)), ...
    sprintf('(d) Truth-compensated (peak %.2f)', pk(4))};
for ci = 1:nCond
    subplot(2,2,ci);
    imagesc(vList, vList, avgImg(:,:,ci)');
    axis xy equal tight; xlim([-0.5 0.5]); ylim([-0.5 0.5]); caxis([0 1]);
    xlabel('v_x'); ylabel('v_y');
    title(panelTitles{ci});
end
sgtitle('10 s time-averaged image in world coordinates');
print('-dpng', '-r300', 'fig_step4_averages.png');

% --- movie: before/after side by side (best effort) ---
try
    vw = VideoWriter('step4_before_after', 'MPEG-4');
    vw.FrameRate = 30;
    open(vw);
    fh = figure('Position', [100 100 1000 520]);
    stride = 3;
    for k = 1:stride:nFrames
        subplot(1,2,1);
        imagesc(vList, vList, frames(:,:,k)');
        axis xy equal tight; xlim([-0.5 0.5]); ylim([-0.5 0.5]); caxis([0 1]);
        xlabel('v_x'); ylabel('v_y'); title('Uncompensated (array frame)');
        subplot(1,2,2);
        imagesc(vList, vList, sensorCompStack(:,:,k)');
        axis xy equal tight; xlim([-0.5 0.5]); ylim([-0.5 0.5]); caxis([0 1]);
        xlabel('v_x'); ylabel('v_y'); title('Sensor-compensated (world frame)');
        sgtitle(sprintf('t = %.2f s   yaw = %+.1f\\circ', tFrame(k), yawFrame(k)));
        drawnow;
        writeVideo(vw, getframe(fh));
    end
    close(vw);
    fprintf('Saved step4_before_after movie\n');
catch err
    fprintf('Movie skipped (%s)\n', err.message);
end

fprintf('Saved results_step4.mat and step-4 figures\n');

% --- local: blob peak + parabolic sub-pixel, same as step 3 ---
function [vxh, vyh] = peakSubpixel(P, vList)
nG = numel(vList);
[~, idx] = max(P, [], 'all');
[ix, iy] = ind2sub([nG nG], idx);
ix = min(max(ix,2), nG-1);  iy = min(max(iy,2), nG-1);
gridRes = vList(2) - vList(1);
dx = 0.5*(P(ix-1,iy)-P(ix+1,iy)) / (P(ix-1,iy)-2*P(ix,iy)+P(ix+1,iy));
dy = 0.5*(P(ix,iy-1)-P(ix,iy+1)) / (P(ix,iy-1)-2*P(ix,iy)+P(ix,iy+1));
vxh = vList(ix) + dx*gridRes;
vyh = vList(iy) + dy*gridRes;
end
