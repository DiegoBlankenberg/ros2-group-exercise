%% kalmanFilter_part2.m
% Part 2: Barometer + GPS altitude Kalman filter
% State vector: x = [altitude; vertical_velocity]
% Predict step uses IMU vertical acceleration. Measurement updates use
% barometer altitude and GPS altitude when each sensor sample arrives.

clear; clc; close all;

%% User settings
DATA_FILE = 'solar.mat';
STATIONARY_SECONDS = 10;      % used to estimate measurement noise
SIGMA_A = 0.30;               % accelerometer process noise std dev (m/s^2)
P0 = diag([0.5, 0.1]);        % initial covariance [altitude, velocity]

if ~isfile(DATA_FILE)
    error('solar.mat not found. Put solar.mat in the same folder as this script. Current folder: %s', pwd);
end

data = load(DATA_FILE);

%% Load and clean signals
imu_time = double(data.IMU.TimeUS(:));
imu_accZ = double(data.IMU.AccZ(:));
att_time = double(data.ATT.TimeUS(:));
att_roll = double(data.ATT.Roll(:));
att_pitch = double(data.ATT.Pitch(:));
baro_time = double(data.BARO.TimeUS(:));
baro_alt = double(data.BARO.Alt(:));
gps_time = double(data.GPS.TimeUS(:));
gps_alt = double(data.GPS.Alt(:));
pos_time = double(data.POS.TimeUS(:));
pos_rel = double(data.POS.RelHomeAlt(:));

[imu_time, ia] = unique(imu_time); imu_accZ = imu_accZ(ia);
[att_time, ia] = unique(att_time); att_roll = att_roll(ia); att_pitch = att_pitch(ia);
[baro_time, ia] = unique(baro_time); baro_alt = baro_alt(ia);
[gps_time, ia] = unique(gps_time); gps_alt = gps_alt(ia);
[pos_time, ia] = unique(pos_time); pos_rel = pos_rel(ia);

% Convert GPS altitude if the log appears to store cm rather than m.
gps_alt = convert_gps_alt_to_metres(gps_alt);

%% Common time interval
all_starts = [imu_time(1), att_time(1), baro_time(1), gps_time(1), pos_time(1)];
all_ends = [imu_time(end), att_time(end), baro_time(end), gps_time(end), pos_time(end)];
t_start = max(all_starts);
t_end = min(all_ends);

% Initial altitude reference from barometer.
alt0_baro = interp1(baro_time, baro_alt, t_start, 'linear', 'extrap');
alt0_gps = interp1(gps_time, gps_alt, t_start, 'linear', 'extrap');
gps_offset = alt0_baro - alt0_gps;
gps_alt_aligned = gps_alt + gps_offset;

%% Convert IMU acceleration to approximate earth vertical acceleration
roll_interp = interp1(att_time, att_roll, imu_time, 'linear', 'extrap');
pitch_interp = interp1(att_time, att_pitch, imu_time, 'linear', 'extrap');
g = 9.81;
az_earth = -(imu_accZ .* cosd(roll_interp) .* cosd(pitch_interp)) - g;

idx = (imu_time >= t_start) & (imu_time <= t_end);
kf_time = imu_time(idx);
kf_az = az_earth(idx);
N = numel(kf_time);

%% Estimate measurement noise from the first stationary window
stationary_end = t_start + STATIONARY_SECONDS*1e6;
R_baro = estimate_variance_in_window(baro_time, baro_alt, t_start, stationary_end, 0.0234);
R_gps_alt = estimate_variance_in_window(gps_time, gps_alt_aligned, t_start, stationary_end, 1.0);

fprintf('Part 2 measurement noise values:\n');
fprintf('  R_baro    = %.6f m^2\n', R_baro);
fprintf('  R_gps_alt = %.6f m^2\n', R_gps_alt);

%% Run barometer-only filter for comparison
baro_only = run_filter(kf_time, kf_az, alt0_baro, P0, SIGMA_A, ...
    baro_time, baro_alt, R_baro, [], [], [], []);

%% Run barometer + GPS altitude filter
baro_gps = run_filter(kf_time, kf_az, alt0_baro, P0, SIGMA_A, ...
    baro_time, baro_alt, R_baro, gps_time, gps_alt_aligned, R_gps_alt, []);

%% Prepare plotting vectors
kf_t_sec = (kf_time - t_start)/1e6;
baro_t_sec = (baro_time - t_start)/1e6;
gps_t_sec = (gps_time - t_start)/1e6;
pos_t_sec = (pos_time - t_start)/1e6;
pos_alt_ref = pos_rel + alt0_baro;

%% Plot Part 2 result
figure('Name','Part 2 - Barometer + GPS Altitude Fusion');
hold on;
plot(baro_t_sec, baro_alt, 'Color', [0.7 0.7 0.7], 'DisplayName', 'Raw barometer');
plot(gps_t_sec, gps_alt_aligned, '.', 'MarkerSize', 4, 'DisplayName', 'GPS altitude aligned');
plot(kf_t_sec, baro_only.x_hist(1,:), 'r', 'LineWidth', 1.2, 'DisplayName', 'Baro-only KF');
plot(kf_t_sec, baro_gps.x_hist(1,:), 'b', 'LineWidth', 1.5, 'DisplayName', 'Baro + GPS altitude KF');
plot(pos_t_sec, pos_alt_ref, 'k--', 'LineWidth', 1.2, 'DisplayName', 'ArduPilot EKF reference');
grid on;
xlabel('Time (s)');
ylabel('Altitude (m)');
title('Part 2: Kalman Filter with Barometer and GPS Altitude');
legend('Location','best');
saveas(gcf, 'part2_baro_gps_altitude.png');
savefig(gcf, 'part2_baro_gps_altitude.fig');

fprintf('Saved Part 2 plot: part2_baro_gps_altitude.png\n');

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

function out = run_filter(kf_time, kf_az, alt0, P0, sigma_a, baro_time, baro_alt, R_baro, gps_time, gps_alt, R_gps_alt, gps_vz)
    N = numel(kf_time);
    x = [alt0; 0];
    P = P0;
    x_hist = zeros(2,N);
    P_hist = zeros(2,2,N);
    x_hist(:,1) = x;
    P_hist(:,:,1) = P;
    baro_idx = 1;
    gps_idx = 1;
    H_alt = [1 0];

    for k = 2:N
        dt = (kf_time(k) - kf_time(k-1))/1e6;
        if dt <= 0 || dt > 1
            x_hist(:,k) = x;
            P_hist(:,:,k) = P;
            continue;
        end

        F = [1 dt; 0 1];
        B = [0.5*dt^2; dt];
        Q = sigma_a^2 * [dt^4/4 dt^3/2; dt^3/2 dt^2];
        x = F*x + B*kf_az(k);
        P = F*P*F' + Q;

        while baro_idx <= numel(baro_time) && baro_time(baro_idx) > kf_time(k-1) && baro_time(baro_idx) <= kf_time(k)
            z = baro_alt(baro_idx);
            [x, P] = kalman_update(x, P, z, H_alt, R_baro);
            baro_idx = baro_idx + 1;
        end
        while baro_idx <= numel(baro_time) && baro_time(baro_idx) <= kf_time(k-1)
            baro_idx = baro_idx + 1;
        end

        if ~isempty(gps_time)
            while gps_idx <= numel(gps_time) && gps_time(gps_idx) > kf_time(k-1) && gps_time(gps_idx) <= kf_time(k)
                z = gps_alt(gps_idx);
                [x, P] = kalman_update(x, P, z, H_alt, R_gps_alt);
                gps_idx = gps_idx + 1;
            end
            while gps_idx <= numel(gps_time) && gps_time(gps_idx) <= kf_time(k-1)
                gps_idx = gps_idx + 1;
            end
        end

        x_hist(:,k) = x;
        P_hist(:,:,k) = P;
    end
    out.x_hist = x_hist;
    out.P_hist = P_hist;
end

function [x, P] = kalman_update(x, P, z, H, R)
    if ~isfinite(z)
        return;
    end
    y = z - H*x;
    S = H*P*H' + R;
    K = P*H'/S;
    x = x + K*y;
    P = (eye(size(P)) - K*H)*P;
end
