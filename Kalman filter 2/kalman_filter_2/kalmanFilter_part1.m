%% kalmanFilter_part1.m
% Part 1: Barometer-only Kalman filter for drone altitude estimation
% State vector: x = [altitude; vertical_velocity]
% Measurement: BARO.Alt
% Control input: IMU vertical acceleration corrected using roll and pitch
%
% Place this script in the same folder as solar.mat and run it in MATLAB.

clear; clc; close all;

%% Load data
if ~isfile('solar.mat')
    error('solar.mat not found. Place solar.mat in the same folder as this script.');
end

data = load('solar.mat');

imu_time = double(data.IMU.TimeUS(:));
imu_accZ = double(data.IMU.AccZ(:));

att_time  = double(data.ATT.TimeUS(:));
att_roll  = double(data.ATT.Roll(:));
att_pitch = double(data.ATT.Pitch(:));

baro_time = double(data.BARO.TimeUS(:));
baro_alt  = double(data.BARO.Alt(:));

pos_time = double(data.POS.TimeUS(:));
pos_rel  = double(data.POS.RelHomeAlt(:));

%% Remove duplicate timestamps
[imu_time, ia] = unique(imu_time);   imu_accZ = imu_accZ(ia);
[att_time, ia] = unique(att_time);   att_roll = att_roll(ia); att_pitch = att_pitch(ia);
[baro_time, ia] = unique(baro_time); baro_alt = baro_alt(ia);
[pos_time, ia] = unique(pos_time);   pos_rel = pos_rel(ia);

%% Common time interval
t_start = max([imu_time(1), att_time(1), baro_time(1), pos_time(1)]);
t_end   = min([imu_time(end), att_time(end), baro_time(end), pos_time(end)]);

alt0_baro = interp1(baro_time, baro_alt, t_start, 'linear', 'extrap');

%% Estimate barometer measurement noise from first 10 s stationary window
stationary_start = t_start;
stationary_end = t_start + 10e6;
stationary_idx = baro_time >= stationary_start & baro_time <= stationary_end;

if sum(stationary_idx) >= 3
    R_baro = var(baro_alt(stationary_idx));
else
    warning('Not enough stationary samples found. Using fallback R_baro = 0.0234 m^2.');
    R_baro = 0.0234;
end

%% Compute vertical acceleration in earth frame
roll_interp  = interp1(att_time, att_roll,  imu_time, 'linear', 'extrap');
pitch_interp = interp1(att_time, att_pitch, imu_time, 'linear', 'extrap');

g = 9.81;
roll_rad  = deg2rad(roll_interp);
pitch_rad = deg2rad(pitch_interp);

% IMU AccZ is body-frame. This simple correction gives positive upward acceleration.
az_earth = -(imu_accZ .* cos(roll_rad) .* cos(pitch_rad)) - g;

%% Use IMU timestamps as prediction timeline
idx = imu_time >= t_start & imu_time <= t_end;
kf_time = imu_time(idx);
kf_az = az_earth(idx);
N = length(kf_time);

%% Kalman filter parameters
x = [alt0_baro; 0];       % [altitude; vertical velocity]
P = diag([0.5, 0.1]);     % initial uncertainty
sigma_a = 0.30;           % acceleration process noise standard deviation
H = [1, 0];               % barometer measures altitude only

x_hist = zeros(2, N);
P_hist = zeros(2, 2, N);
x_hist(:,1) = x;
P_hist(:,:,1) = P;

baro_idx = 1;

%% Run Kalman filter
for k = 2:N
    dt = (kf_time(k) - kf_time(k-1)) / 1e6;

    if dt <= 0 || dt > 1
        x_hist(:,k) = x;
        P_hist(:,:,k) = P;
        continue;
    end

    % Predict
    F = [1, dt;
         0,  1];
    B = [0.5*dt^2;
         dt];

    x = F*x + B*kf_az(k);

    Q = sigma_a^2 * [dt^4/4, dt^3/2;
                     dt^3/2, dt^2];
    P = F*P*F' + Q;

    % Update when a new barometer sample occurred in this IMU interval
    while baro_idx <= length(baro_time) && baro_time(baro_idx) <= kf_time(k)
        if baro_time(baro_idx) > kf_time(k-1)
            z = baro_alt(baro_idx);
            y = z - H*x;
            S = H*P*H' + R_baro;
            K = P*H'/S;
            x = x + K*y;
            P = (eye(2) - K*H)*P;
        end
        baro_idx = baro_idx + 1;
    end

    x_hist(:,k) = x;
    P_hist(:,:,k) = P;
end

%% Convert time to seconds and prepare reference
kf_t_sec = (kf_time - t_start)/1e6;
baro_t_sec = (baro_time - t_start)/1e6;
pos_t_sec = (pos_time - t_start)/1e6;
pos_alt_ref = pos_rel + alt0_baro;

pos_interp = interp1(pos_time, pos_alt_ref, kf_time, 'linear', 'extrap');
rmse_baro_only = sqrt(mean((x_hist(1,:) - pos_interp').^2));

%% Plot and save
figure('Name', 'Part 1 Barometer-only Kalman Filter', 'Color', 'w');
plot(baro_t_sec, baro_alt, 'Color', [0.65 0.65 0.65], 'LineWidth', 0.7, 'DisplayName', 'Raw barometer'); hold on;
plot(kf_t_sec, x_hist(1,:), 'r', 'LineWidth', 1.3, 'DisplayName', 'Barometer-only KF');
plot(pos_t_sec, pos_alt_ref, 'k--', 'LineWidth', 1.1, 'DisplayName', 'ArduPilot EKF reference');
grid on;
xlabel('Time (s)');
ylabel('Altitude (m)');
title('Part 1: Barometer-only Kalman Filter Altitude Estimate');
legend('Location', 'best');

saveas(gcf, 'part1_baro_only_altitude.png');

%% Display parameter summary
fprintf('\nPart 1 complete.\n');
fprintf('R_baro from first 10 s stationary window: %.6f m^2\n', R_baro);
fprintf('P0 = diag([0.5, 0.1])\n');
fprintf('sigma_a = %.2f m/s^2\n', sigma_a);
fprintf('RMSE vs ArduPilot EKF reference: %.3f m\n', rmse_baro_only);
fprintf('Saved figure: part1_baro_only_altitude.png\n');
