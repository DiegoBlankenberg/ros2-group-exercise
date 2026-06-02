% Plot Kalman Filter Performance
% Compares filter estimates against raw measurements and theoretical trajectory
% Marks occlusion region

clear; clc; close all;

% ===================== Parameters =====================
dt = 0.033;
g = -9.81;  % Gravitational acceleration (negative = downward)
H = [1 0];
R = 0.1^2;  % Measurement uncertainty
SDg = 0.01 * 9.81;  % Standard deviation of acceleration

% State transition and input matrices
F = [1 dt; 0 1];
B = [0.5*dt^2; dt];

% Process noise covariance
sigma2 = SDg^2;
Q = sigma2 * [dt^4/4, dt^3/2; dt^3/2, dt^2];

% ===================== Load Data =====================
% Parse Data points.txt (European decimal format with commas)
fid = fopen('Data points.txt', 'r');

% Skip header
fgetl(fid);  % 'mass A'
fgetl(fid);  % 't x y'

time = [];
y_meas = [];
k = 1;

while ~feof(fid)
    line = fgetl(fid);
    if ischar(line) && ~isempty(line)
        % Replace commas with periods for European decimal format
        line = strrep(line, ',', '.');
        
        % Split by tabs
        parts = strsplit(line, sprintf('\t'));
        parts = parts(~cellfun(@isempty, parts));
        
        if length(parts) >= 1
            t = str2double(parts{1});
            if ~isnan(t)
                time(k) = t;
                
                if length(parts) >= 3 && ~isempty(parts{3})
                    y = str2double(parts{3});
                else
                    y = NaN;  % Missing measurement (occlusion)
                end
                
                y_meas(k) = y;
                k = k + 1;
            end
        end
    end
end
fclose(fid);

time = time';
y_meas = y_meas';

fprintf('Loaded %d measurements\n', length(y_meas));
fprintf('Valid measurements: %d, Occluded: %d\n', sum(~isnan(y_meas)), sum(isnan(y_meas)));

% Identify occlusion region
occlusion_idx = isnan(y_meas);

% ===================== Run Kalman Filter =====================
x = [0; 0];  % Initial state [position; velocity]
P = zeros(2, 2);  % Initial covariance

% Store results
N = length(time);
x_est = zeros(2, N);  % Estimated state [position; velocity]
P_diag = zeros(2, N);  % Diagonal of covariance
y_est = zeros(1, N);  % Estimated position

for k = 1:N
    % Prediction
    x_pred = F * x + B * g;
    P_pred = F * P * F' + Q;
    
    % Update (if measurement available)
    if ~isnan(y_meas(k))
        % Kalman gain
        S = H * P_pred * H' + R;
        K = P_pred * H' / S;
        
        % Innovation
        y_innov = y_meas(k) - H * x_pred;
        
        % Update
        x = x_pred + K * y_innov;
        P = (eye(2) - K * H) * P_pred;
    else
        % No measurement, just prediction
        x = x_pred;
        P = P_pred;
    end
    
    % Store
    x_est(:, k) = x;
    P_diag(:, k) = diag(P);
    y_est(k) = x(1);
end

% ===================== Theoretical Trajectory =====================
% Pure physics: x(t) = x0 + v0*t + 0.5*g*t^2
% Starting from x0=0, v0=0
y_theory = 0.5 * g * time.^2;

% ===================== Create Plots =====================
figure('Position', [100 100 1200 800]);

% Plot 1: Position trajectory
subplot(2, 2, 1);
hold on;
plot(time, y_est, 'b-', 'LineWidth', 2, 'DisplayName', 'Kalman Filter Estimate');
plot(time(~occlusion_idx), y_meas(~occlusion_idx), 'ko', 'MarkerSize', 6, ...
    'DisplayName', 'Raw Measurements');
plot(time, y_theory, 'r--', 'LineWidth', 1.5, 'DisplayName', 'Theoretical (gravity only)');

% Mark occlusion region
if any(occlusion_idx)
    occlusion_times_unique = unique(time(occlusion_idx));
    for t_occ = occlusion_times_unique'
        plot([t_occ t_occ], [min(y_meas(~isnan(y_meas)))-0.1, max(y_meas(~isnan(y_meas)))+0.1], ...
            'color', [1 0.5 0], 'LineStyle', ':', 'LineWidth', 1.5);
    end
    % Add text for occlusion
    ax = gca;
    yrange = ax.YLim;
    text(mean(occlusion_times_unique), yrange(2)*0.95, 'Occlusion', ...
        'HorizontalAlignment', 'center', 'BackgroundColor', 'yellow', 'FontSize', 10);
end

grid on;
xlabel('Time (s)');
ylabel('Position (m)');
title('Trajectory Estimate vs Raw Measurements vs Theory');
legend('Location', 'best');
hold off;

% Plot 2: Velocity estimate
subplot(2, 2, 2);
v_est = x_est(2, :);
plot(time, v_est, 'b-', 'LineWidth', 2);
grid on;
xlabel('Time (s)');
ylabel('Velocity (m/s)');
title('Estimated Velocity');

% Plot 3: Position estimation error
subplot(2, 2, 3);
error = y_est - y_meas;
plot(time(~occlusion_idx), error(~occlusion_idx), 'ko-', 'MarkerSize', 5);
grid on;
xlabel('Time (s)');
ylabel('Error (m)');
title('Position Estimation Error (Estimate - Measurement)');

% Plot 4: Covariance (uncertainty)
subplot(2, 2, 4);
plot(time, sqrt(P_diag(1, :)), 'b-', 'LineWidth', 2, 'DisplayName', 'Position std dev');
plot(time, sqrt(P_diag(2, :)), 'r-', 'LineWidth', 2, 'DisplayName', 'Velocity std dev');
grid on;
xlabel('Time (s)');
ylabel('Standard Deviation');
title('Filter Uncertainty (std dev)');
legend('Location', 'best');

sgtitle('Kalman Filter Performance Evaluation');

% Save figure
savefig('kalman_filter_performance.fig');
print('kalman_filter_performance.png', '-dpng', '-r150');

fprintf('\nPlots saved:\n');
fprintf('  kalman_filter_performance.fig\n');
fprintf('  kalman_filter_performance.png\n');

% ===================== Statistics =====================
fprintf('\n=== Performance Statistics ===\n');
fprintf('Mean position error: %.6f m\n', mean(error(~isnan(error))));
fprintf('RMS position error: %.6f m\n', sqrt(mean(error(~isnan(error)).^2)));
fprintf('Max position error: %.6f m\n', max(abs(error(~isnan(error)))));
fprintf('Occlusion duration: %.3f s\n', (length(unique(time(isnan(y_meas)))) * dt));
