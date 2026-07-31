%% kalmanFilter_part3.m
% Part 3: Barometer + GPS altitude + GPS vertical velocity Kalman filter
% State vector: x = [altitude; vertical_velocity]
% GPS altitude and GPS vertical velocity arrive at the same timestamp, so
% they are fused together in one vector measurement update.

clear; clc; close all;

%% User settings
DATA_FILE = 'solar.mat';
STATIONARY_SECONDS = 10;
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
gps_time = double(data.GPS.TimeUS(:)); gps_alt = double(data.GPS.Alt(:)); gps_vz = double(data.GPS.VZ(:));
pos_time = double(data.POS.TimeUS(:)); pos_rel = double(data.POS.RelHomeAlt(:));

[imu_time, ia] = unique(imu_time); imu_accZ = imu_accZ(ia);
[att_time, ia] = unique(att_time); att_roll = att_roll(ia); att_pitch = att_pitch(ia);
[baro_time, ia] = unique(baro_time); baro_alt = baro_alt(ia);
[gps_time, ia] = unique(gps_time); gps_alt = gps_alt(ia); gps_vz = gps_vz(ia);
[pos_time, ia] = unique(pos_time); pos_rel = pos_rel(ia);

gps_alt = convert_gps_alt_to_metres(gps_alt);
gps_vz_up = convert_gps_vz_to_up_ms(gps_vz);

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

%% Noise estimates from first stationary window
stationary_end = t_start + STATIONARY_SECONDS*1e6;
R_baro = estimate_variance_in_window(baro_time, baro_alt, t_start, stationary_end, 0.0234);
R_gps_alt = estimate_variance_in_window(gps_time, gps_alt_aligned, t_start, stationary_end, 1.0);
R_gps_vz = estimate_variance_in_window(gps_time, gps_vz_up, t_start, stationary_end, 0.25);

fprintf('Part 3 measurement noise values:\n');
fprintf('  R_baro    = %.6f m^2\n', R_baro);
fprintf('  R_gps_alt = %.6f m^2\n', R_gps_alt);
fprintf('  R_gps_vz  = %.6f (m/s)^2\n', R_gps_vz);

%% Run three filters for comparison
baro_only = run_filter(kf_time, kf_az, alt0_baro, P0, SIGMA_A, ...
    baro_time, baro_alt, R_baro, [], [], [], [], []);
baro_gps_alt = run_filter(kf_time, kf_az, alt0_baro, P0, SIGMA_A, ...
    baro_time, baro_alt, R_baro, gps_time, gps_alt_aligned, R_gps_alt, [], []);
baro_gps_alt_vz = run_filter(kf_time, kf_az, alt0_baro, P0, SIGMA_A, ...
    baro_time, baro_alt, R_baro, gps_time, gps_alt_aligned, R_gps_alt, gps_vz_up, R_gps_vz);

%% Plot
kf_t_sec = (kf_time - t_start)/1e6;
baro_t_sec = (baro_time - t_start)/1e6;
pos_t_sec = (pos_time - t_start)/1e6;
pos_alt_ref = pos_rel + alt0_baro;

figure('Name','Part 3 - GPS Altitude and Velocity Fusion');
subplot(2,1,1); hold on;
plot(baro_t_sec, baro_alt, 'Color', [0.75 0.75 0.75], 'DisplayName', 'Raw barometer');
plot(kf_t_sec, baro_only.x_hist(1,:), 'r', 'LineWidth', 1.1, 'DisplayName', 'Part 1: Baro-only KF');
plot(kf_t_sec, baro_gps_alt.x_hist(1,:), 'b', 'LineWidth', 1.3, 'DisplayName', 'Part 2: Baro + GPS altitude KF');
plot(kf_t_sec, baro_gps_alt_vz.x_hist(1,:), 'm', 'LineWidth', 1.5, 'DisplayName', 'Part 3: Baro + GPS altitude + VZ KF');
plot(pos_t_sec, pos_alt_ref, 'k--', 'LineWidth', 1.2, 'DisplayName', 'ArduPilot EKF reference');
grid on; xlabel('Time (s)'); ylabel('Altitude (m)');
title('Part 3: Altitude Estimates'); legend('Location','best');

subplot(2,1,2); hold on;
plot(kf_t_sec, baro_only.x_hist(2,:), 'r', 'LineWidth', 1.1, 'DisplayName', 'Part 1 velocity');
plot(kf_t_sec, baro_gps_alt.x_hist(2,:), 'b', 'LineWidth', 1.3, 'DisplayName', 'Part 2 velocity');
plot(kf_t_sec, baro_gps_alt_vz.x_hist(2,:), 'm', 'LineWidth', 1.5, 'DisplayName', 'Part 3 velocity');
grid on; xlabel('Time (s)'); ylabel('Vertical velocity (m/s, positive up)');
title('Estimated Vertical Velocity'); legend('Location','best');

saveas(gcf, 'part3_gps_altitude_velocity.png');
savefig(gcf, 'part3_gps_altitude_velocity.fig');
fprintf('Saved Part 3 plot: part3_gps_altitude_velocity.png\n');

%% Local functions
function gps_alt_m = convert_gps_alt_to_metres(gps_alt_raw)
    gps_alt_m = gps_alt_raw;
    if median(abs(gps_alt_raw), 'omitnan') > 1000
        gps_alt_m = gps_alt_raw/100;
    end
end

function vz_up = convert_gps_vz_to_up_ms(vz_raw)
    % ArduPilot GPS.VZ is often positive down and stored in cm/s. The filter
    % velocity state is positive up, so the sign is reversed.
    vz_up = -vz_raw;
    if median(abs(vz_raw), 'omitnan') > 30
        vz_up = -vz_raw/100;
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

function out = run_filter(kf_time, kf_az, alt0, P0, sigma_a, baro_time, baro_alt, R_baro, gps_time, gps_alt, R_gps_alt, gps_vz, R_gps_vz)
    N = numel(kf_time); x = [alt0; 0]; P = P0;
    x_hist = zeros(2,N); P_hist = zeros(2,2,N);
    x_hist(:,1) = x; P_hist(:,:,1) = P;
    baro_idx = 1; gps_idx = 1;
    H_baro = [1 0];

    for k = 2:N
        dt = (kf_time(k) - kf_time(k-1))/1e6;
        if dt <= 0 || dt > 1
            x_hist(:,k) = x; P_hist(:,:,k) = P; continue;
        end
        F = [1 dt; 0 1]; B = [0.5*dt^2; dt];
        Q = sigma_a^2 * [dt^4/4 dt^3/2; dt^3/2 dt^2];
        x = F*x + B*kf_az(k); P = F*P*F' + Q;

        while baro_idx <= numel(baro_time) && baro_time(baro_idx) > kf_time(k-1) && baro_time(baro_idx) <= kf_time(k)
            [x, P] = kalman_update(x, P, baro_alt(baro_idx), H_baro, R_baro);
            baro_idx = baro_idx + 1;
        end
        while baro_idx <= numel(baro_time) && baro_time(baro_idx) <= kf_time(k-1)
            baro_idx = baro_idx + 1;
        end

        if ~isempty(gps_time)
            while gps_idx <= numel(gps_time) && gps_time(gps_idx) > kf_time(k-1) && gps_time(gps_idx) <= kf_time(k)
                if isempty(gps_vz)
                    H = [1 0]; z = gps_alt(gps_idx); R = R_gps_alt;
                else
                    H = [1 0; 0 1];
                    z = [gps_alt(gps_idx); gps_vz(gps_idx)];
                    R = diag([R_gps_alt, R_gps_vz]);
                end
                [x, P] = kalman_update(x, P, z, H, R);
                gps_idx = gps_idx + 1;
            end
            while gps_idx <= numel(gps_time) && gps_time(gps_idx) <= kf_time(k-1)
                gps_idx = gps_idx + 1;
            end
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
