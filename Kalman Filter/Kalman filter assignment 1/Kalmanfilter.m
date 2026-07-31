% I want to run three iterations of a kalman filter i.e k=0,1,2.
% Only measure position
% delta t=0.033s
% H=[1 0];
% standard deviation of the random acceleration is SDg=1% of 9.81
% Measurement uncertainty is SDm=0.1 m
% Initial state is x0=[0;0]
% P0=zeros(2,2)
% z=[-2.762E-9;-1.614E-2;-4.453E-2] % measurements at k=0,1,2

clear; clc;

% Parameters
dt = 0.033;
H = [1 0];
R = 0.1^2;  % Measurement uncertainty (0.1 m)
SDg = 0.01 * 9.81;  % Standard deviation of acceleration (1% of 9.81)

% Initial conditions
x = [0; 0];  % [position; velocity]
P = zeros(2, 2);  % Initial covariance

% State transition matrix
F = [1 dt; 0 1];

% Input matrix for gravitational acceleration
% For discrete-time system: x = [position; velocity]
% u = -g (negative because downward is negative direction)
B = [0.5*dt^2; dt];
g = -9.81;  % Gravitational acceleration (negative = downward)

% Process noise covariance (constant acceleration model)
% Q = σ² * [[Δt⁴/4, Δt³/2], [Δt³/2, Δt²]]
sigma2 = SDg^2;
Q = sigma2 * [dt^4/4, dt^3/2; dt^3/2, dt^2];

% Measurements (corrected from comments)
z = [-2.762e-4; -1.614e-2; -4.453e-2];

% Store results in table for export to Excel
results = table();

fprintf('\n=== Kalman Filter Iterations ===\n');

for k = 0:2
    fprintf('\n--- Iteration k=%d ---\n', k);
    
    % Prediction step
    % x_minus = F * x + B * u, where u = gravity acceleration
    x_minus = F * x + B * g;
    P_minus = F * P * F' + Q;
    
    fprintf('x⁻ (prediction with gravity) = [%.6e; %.6e]\n', x_minus(1), x_minus(2));
    fprintf('P⁻ (prediction covariance) =\n');
    disp(P_minus);
    
    % Kalman gain
    S = H * P_minus * H' + R;  % Innovation covariance
    K = P_minus * H' / S;
    
    fprintf('K (Kalman gain) = [%.6e; %.6e]\n', K(1), K(2));
    
    % Innovation (measurement residual)
    y = z(k+1) - H * x_minus;
    fprintf('y (innovation) = %.6e\n', y);
    
    % Update step
    x_plus = x_minus + K * y;
    P_plus = (eye(2) - K * H) * P_minus;
    
    fprintf('x⁺ (corrected) = [%.6e; %.6e]\n', x_plus(1), x_plus(2));
    fprintf('P⁺ (corrected covariance) =\n');
    disp(P_plus);
    
    % Store results
    new_row = table(k, x_minus(1), x_minus(2), P_minus(1,1), P_minus(1,2), P_minus(2,1), P_minus(2,2), ...
                    K(1), K(2), x_plus(1), x_plus(2), P_plus(1,1), P_plus(1,2), P_plus(2,1), P_plus(2,2), ...
                    'VariableNames', {'k', 'x_minus_pos', 'x_minus_vel', 'P_minus_11', 'P_minus_12', ...
                                      'P_minus_21', 'P_minus_22', 'K_pos', 'K_vel', 'x_plus_pos', 'x_plus_vel', ...
                                      'P_plus_11', 'P_plus_12', 'P_plus_21', 'P_plus_22'});
    results = [results; new_row];
    
    % Update state for next iteration
    x = x_plus;
    P = P_plus;
end

% Display results table
fprintf('\n\n=== Summary Table ===\n');
disp(results);

% Export to Excel
writetable(results, 'Kalman_Filter_Results.xlsx');
fprintf('\n✓ Results exported to Kalman_Filter_Results.xlsx\n');
