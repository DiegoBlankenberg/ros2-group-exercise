%% kalmanFilter_part4_outage.m
% Part 4 Task G: Barometer + GPS altitude filter with artificial GPS outage
% This script removes GPS altitude measurements for a 20-second window in
% the middle of the flight and plots the covariance diagonal terms.

clear; clc; close all;

%% User settings
DATA_FILE = 'solar.mat';
STATIONARY_SECONDS = 10;
OUTAGE_LENGTH_SECONDS = 20;
SIGMA_A = 0.30;
P0 = diag([0.5, 0.1]);

if ~isfile(DATA_FILE)
    error('solar.mat not found. Put solar.mat in the same folder as this script. Current folder: %s', pwd);
end

data = load(DATA_FILE);

%% Load and clean signals
imu_time = double(data.IMU.TimeUS(:)); imu_accZ = double(data.IMU.AccZ(:));
att_time = double(data.ATT.TimeUS(:)); att_roll = double(data.ATT.Roll(:)); att_pitch = double(data.ATT.Pitch(:));
baro_time = double(data.BARO.TimeUS(:)); baro_alt = double(data.BARO.Alt(:));
gps_time = double(data.GPS.TimeUS(:)); gps_alt = double(data.GPS.Alt(:));
pos_time = double(data.POS.TimeUS(:)); pos_rel = double(data.POS.RelHomeAlt(:));

[imu_time, ia] = unique(imu_time); imu_accZ = imu_accZ(ia);
[att_time, ia] = unique(att_time); att_roll = att_roll(ia); att_pitch = att_pitch(ia);
[baro_time, ia] = unique(baro_time); baro_alt = baro_alt(ia);
[gps_time, ia] = unique(gps_time); gps_alt = gps_alt(ia);
[pos_time, ia] = unique(pos_time); pos_rel = pos_rel(ia);

gps_alt = convert_gps_alt_to_metres(gps_alt);

%% Time interval and alignment
all_starts = [imu_time(1), att_time(1), baro_time(1), gps_time(1), pos_time(1)];
all_ends = [imu_time(end), att_time(end), baro_time(end), gps_time(end), pos_time(end)];
t_start = max(all_starts);
t_end = min(all_ends);
alt0_baro = interp1(baro_time, baro_alt, t_start, 'linear', 'extrap');
alt0_gps = interp1(gps_time, gps_alt, t_start, 'linear', 'extrap');
gps_alt_aligned = gps_alt + (alt0_baro - alt0_gps);

%% IMU vertical acceleration
roll_interp = interp1(att_time, att_roll, imu_time, 'linear', 'extrap');
pitch_interp = interp1(att_time, att_pitch, imu_time, 'linear', 'extrap');
g = 9.81;
az_earth = -(imu_accZ .* cosd(roll_interp) .* cosd(pitch_interp)) - g;
idx = (imu_time >= t_start) & (imu_time <= t_end);
kf_time = imu_time(idx);
kf_az = az_earth(idx);

%% Noise estimates
stationary_end = t_start + STATIONARY_SECONDS*1e6;
R_baro = estimate_variance_in_window(baro_time, baro_alt, t_start, stationary_end, 0.0234);
R_gps_alt = estimate_variance_in_window(gps_time, gps_alt_aligned, t_start, stationary_end, 1.0);

%% Define outage window in the middle of the flight
flight_duration_us = t_end - t_start;
outage_start = t_start + 0.5*flight_duration_us - 0.5*OUTAGE_LENGTH_SECONDS*1e6;
outage_end = outage_start + OUTAGE_LENGTH_SECONDS*1e6;

%% Run normal Part 2 filter and outage Part 2 filter
normal = run_filter_part2(kf_time, kf_az, alt0_baro, P0, SIGMA_A, ...
    baro_time, baro_alt, R_baro, gps_time, gps_alt_aligned, R_gps_alt, NaN, NaN);
outage = run_filter_part2(kf_time, kf_az, alt0_baro, P0, SIGMA_A, ...
    baro_time, baro_alt, R_baro, gps_time, gps_alt_aligned, R_gps_alt, outage_start, outage_end);

%% Plot covariance evolution
kf_t_sec = (kf_time - t_start)/1e6;
outage_start_sec = (outage_start - t_start)/1e6;
outage_end_sec = (outage_end - t_start)/1e6;

P11_normal = squeeze(normal.P_hist(1,1,:));
P22_normal = squeeze(normal.P_hist(2,2,:));
P11_outage = squeeze(outage.P_hist(1,1,:));
P22_outage = squeeze(outage.P_hist(2,2,:));

figure('Name','Part 4 Task G - Covariance During GPS Outage');
subplot(2,1,1); hold on;
plot(kf_t_sec, P11_normal, 'b', 'LineWidth', 1.1, 'DisplayName', 'Normal GPS');
plot(kf_t_sec, P11_outage, 'r', 'LineWidth', 1.3, 'DisplayName', 'GPS outage');
xline(outage_start_sec, 'k--', 'Outage starts');
xline(outage_end_sec, 'k--', 'Outage ends');
grid on; xlabel('Time (s)'); ylabel('P(1,1) altitude variance (m^2)');
title('Altitude Variance During GPS Outage'); legend('Location','best');

subplot(2,1,2); hold on;
plot(kf_t_sec, P22_normal, 'b', 'LineWidth', 1.1, 'DisplayName', 'Normal GPS');
plot(kf_t_sec, P22_outage, 'r', 'LineWidth', 1.3, 'DisplayName', 'GPS outage');
xline(outage_start_sec, 'k--', 'Outage starts');
xline(outage_end_sec, 'k--', 'Outage ends');
grid on; xlabel('Time (s)'); ylabel('P(2,2) velocity variance ((m/s)^2)');
title('Velocity Variance During GPS Outage'); legend('Location','best');

saveas(gcf, 'part4_P_evolution_gps_outage.png');
savefig(gcf, 'part4_P_evolution_gps_outage.fig');
fprintf('Saved Task G plot: part4_P_evolution_gps_outage.png\n');
fprintf('Outage window: %.1f s to %.1f s\n', outage_start_sec, outage_end_sec);

%% Local functions
function gps_alt_m = convert_gps_alt_to_metres(gps_alt_raw)
    gps_alt_m = gps_alt_raw;
    if median(abs(gps_alt_raw), 'omitnan') > 1000
        gps_alt_m = gps_alt_raw/100;
    end
end

function R = estimate_variance_in_window(t, y, t0, t1, fallback)
    idx = t >= t0 & t <= t1 & isfinite(y);
    if nnz(idx) >= 3
        R = var(y(idx));
        if ~isfinite(R) || R <= 0
            R = fallback;
        end
    else
        R = fallback;
    end
end

function out = run_filter_part2(kf_time, kf_az, alt0, P0, sigma_a, baro_time, baro_alt, R_baro, gps_time, gps_alt, R_gps_alt, outage_start, outage_end)
    N = numel(kf_time); x = [alt0; 0]; P = P0;
    x_hist = zeros(2,N); P_hist = zeros(2,2,N);
    x_hist(:,1) = x; P_hist(:,:,1) = P;
    baro_idx = 1; gps_idx = 1; H_alt = [1 0];
    use_outage = isfinite(outage_start) && isfinite(outage_end);

    for k = 2:N
        dt = (kf_time(k) - kf_time(k-1))/1e6;
        if dt <= 0 || dt > 1
            x_hist(:,k) = x; P_hist(:,:,k) = P; continue;
        end
        F = [1 dt; 0 1]; B = [0.5*dt^2; dt];
        Q = sigma_a^2 * [dt^4/4 dt^3/2; dt^3/2 dt^2];
        x = F*x + B*kf_az(k); P = F*P*F' + Q;

        while baro_idx <= numel(baro_time) && baro_time(baro_idx) > kf_time(k-1) && baro_time(baro_idx) <= kf_time(k)
            [x, P] = kalman_update(x, P, baro_alt(baro_idx), H_alt, R_baro);
            baro_idx = baro_idx + 1;
        end
        while baro_idx <= numel(baro_time) && baro_time(baro_idx) <= kf_time(k-1)
            baro_idx = baro_idx + 1;
        end

        while gps_idx <= numel(gps_time) && gps_time(gps_idx) > kf_time(k-1) && gps_time(gps_idx) <= kf_time(k)
            in_outage = use_outage && gps_time(gps_idx) >= outage_start && gps_time(gps_idx) <= outage_end;
            if ~in_outage
                [x, P] = kalman_update(x, P, gps_alt(gps_idx), H_alt, R_gps_alt);
            end
            gps_idx = gps_idx + 1;
        end
        while gps_idx <= numel(gps_time) && gps_time(gps_idx) <= kf_time(k-1)
            gps_idx = gps_idx + 1;
        end
        x_hist(:,k) = x; P_hist(:,:,k) = P;
    end
    out.x_hist = x_hist; out.P_hist = P_hist;
end

function [x, P] = kalman_update(x, P, z, H, R)
    if any(~isfinite(z)); return; end
    y = z - H*x; S = H*P*H' + R; K = P*H'/S;
    x = x + K*y; P = (eye(size(P)) - K*H)*P;
end
