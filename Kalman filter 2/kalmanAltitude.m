%% kalmanAltitude.m
%   - IMU vertical acceleration (input / control)
%   - Barometer altitude (measurement)
%
% State vector: x = [altitude; vertical_velocity]
% The IMU accelerometer is used in the predict step (control input).
% Barometer is used in the update step (measurement).
%
% Requires pymavlink: pip install pymavlink --break-system-packages


clear; clc; close all;

%% Load the .mat log file (exported from bin2mat.py)
data = load('solar.mat');

% Extract IMU data
imu_time = double(data.IMU.TimeUS);
imu_accZ = double(data.IMU.AccZ);

% Extract ATT data
att_time  = double(data.ATT.TimeUS);
att_roll  = double(data.ATT.Roll);
att_pitch = double(data.ATT.Pitch);

% Extract BARO data
baro_time = double(data.BARO.TimeUS);
baro_alt  = double(data.BARO.Alt);

% Extract GPS data
if isfield(data, 'GPS')
    gps_time = double(data.GPS.TimeUS);
    gps_alt  = double(data.GPS.Alt);
else
    gps_time = [];
    gps_alt = [];
end

% Extract POS data (for comparison)
if isfield(data, 'POS')
    pos_time = double(data.POS.TimeUS);
    pos_rel  = double(data.POS.RelHomeAlt);
else
    pos_time = [];
    pos_rel = [];
end

%% ---- Remove duplicate timestamps ----
[imu_time, ia] = unique(imu_time);  imu_accZ = imu_accZ(ia);
[att_time, ia] = unique(att_time);  att_roll = att_roll(ia); att_pitch = att_pitch(ia);
[baro_time, ia] = unique(baro_time); baro_alt = baro_alt(ia);
[gps_time, ia] = unique(gps_time);  gps_alt = gps_alt(ia);
[pos_time, ia] = unique(pos_time);  pos_rel = pos_rel(ia);

%% ---- Compute BARO.Alt variance ----
baro_variance = var(baro_alt);
fprintf('BARO.Alt Variance: %.6f (m^2)\n', baro_variance);

%% ---- Compute vertical acceleration in earth frame ----
% Interpolate roll/pitch to IMU timestamps
roll_interp  = interp1(att_time, att_roll,  imu_time, 'linear', 'extrap');
pitch_interp = interp1(att_time, att_pitch, imu_time, 'linear', 'extrap');

% IMU AccZ is in body frame (pointing down). Rotate to get earth-frame
% vertical acceleration. For a simple tilt correction:
%   a_vertical = AccZ * cos(roll) * cos(pitch) + gravity
% AccZ is negative when hovering (~-9.81), so earth-frame up acceleration:
g = 9.81;
roll_rad  = deg2rad(roll_interp);
pitch_rad = deg2rad(pitch_interp);
az_earth  = -(imu_accZ .* cos(roll_rad) .* cos(pitch_rad)) - g;

%% ---- Align all data to a common reference ----
% Use baro altitude at start as the initial altitude reference
t_start = max([imu_time(1), baro_time(1)]);
t_end   = min([imu_time(end), baro_time(end)]);

% Initial altitude from baro
alt0_baro = interp1(baro_time, baro_alt, t_start, 'linear', 'extrap');
% GPS alignment is not needed for filter, but keep for plotting
if ~isempty(gps_time)
    alt0_gps  = interp1(gps_time,  gps_alt,  t_start, 'linear', 'extrap');
    gps_offset = alt0_baro - alt0_gps;
    gps_alt_aligned = gps_alt + gps_offset;
else
    gps_alt_aligned = [];
end

%% ---- Run Kalman Filter ----
fprintf('Running Kalman filter...\n');

% Use IMU timestamps as the main time base (highest rate)
% Only keep samples in the common time range
idx = (imu_time >= t_start) & (imu_time <= t_end);
kf_time  = imu_time(idx);
kf_az    = az_earth(idx);

N = length(kf_time);
% State: x = [altitude; velocity]
x = [alt0_baro; 0];

% State covariance
P = diag([0.5, 0.1]);
%P = diag([1, 1]);
% Process noise (tuning parameters)
sigma_a = 0.3;  % accelerometer noise (m/s^2)
%sigma_a = 0.5; 
% Measurement noise
R_baro = 0.0234;   % barometer noise variance (m^2)
%R_baro = 0.5;
% Print R values
fprintf('========== Kalman Filter Parameters ==========\n');
fprintf('Measurement Noise (R):\n');
fprintf('  R_baro = %.4f (m^2)\n', R_baro);
fprintf('=============================================\n');

% Measurement matrix (baro measures altitude)
H = [1, 0];

% Pre-allocate output
x_hist = zeros(2, N);
x_hist(:,1) = x;

% Index for baro (for update step)
baro_idx = 1;

for k = 2:N
    % Time step
    dt = (kf_time(k) - kf_time(k-1)) / 1e6;  % seconds
    if dt <= 0 || dt > 1
        x_hist(:,k) = x;
        continue;
    end

    % --- PREDICT (using IMU acceleration as control input) ---
    F = [1, dt;
         0,  1];
    B = [0.5*dt^2;
         dt];
    u = kf_az(k);

    x = F * x + B * u;

    % Process noise covariance
    Q = sigma_a^2 * [dt^4/4, dt^3/2;
                     dt^3/2, dt^2];
    P = F * P * F' + Q;

    % --- UPDATE with barometer (when new sample available) ---
    while baro_idx < length(baro_time) && baro_time(baro_idx) <= kf_time(k)
        baro_idx = baro_idx + 1;
    end
    % Check if a baro sample falls in this interval
    if baro_idx > 1 && baro_time(baro_idx-1) > kf_time(k-1) && baro_time(baro_idx-1) <= kf_time(k)
        z_baro = baro_alt(baro_idx-1);
        y_baro = z_baro - H * x;          % innovation
        S_baro = H * P * H' + R_baro;
        K_baro = P * H' / S_baro;         % Kalman gain
        x = x + K_baro * y_baro;
        P = (eye(2) - K_baro * H) * P;
    end


    x_hist(:,k) = x;
end

%% ---- Convert to seconds for plotting ----
kf_t_sec   = (kf_time - t_start) / 1e6;
baro_t_sec = (baro_time - t_start) / 1e6;
if ~isempty(gps_time)
    gps_t_sec  = (gps_time - t_start) / 1e6;
else
    gps_t_sec = [];
end
pos_t_sec  = (pos_time - t_start) / 1e6;

%% ---- Plot results ----
figure('Name', 'Kalman Filter Altitude Estimation');

% Altitude comparison
subplot(2,1,1);
hold on;
plot(baro_t_sec, baro_alt, 'b', 'LineWidth', 0.8, 'DisplayName', 'Barometer');
if ~isempty(gps_t_sec)
    plot(gps_t_sec, gps_alt_aligned, 'g', 'LineWidth', 0.8, 'DisplayName', 'GPS (aligned)');
end
plot(kf_t_sec, x_hist(1,:), 'r', 'LineWidth', 1.5, 'DisplayName', 'Kalman Filter');
plot(pos_t_sec, pos_rel + alt0_baro, 'y--', 'LineWidth', 1.0, 'DisplayName', 'ArduPilot EKF');
xlabel('Time (s)');
ylabel('Altitude (m)');
title('Altitude Estimation: Kalman Filter vs Sensors');
legend('Location', 'best');
grid on;

% Estimated vertical velocity
subplot(2,1,2);
plot(kf_t_sec, x_hist(2,:), 'm', 'LineWidth', 1.2);
xlabel('Time (s)');
ylabel('Velocity (m/s)');
title('Estimated Vertical Velocity');
grid on;

sgtitle('Simple Kalman Filter — Baro + Accelerometer');
fprintf('Done.\n');
