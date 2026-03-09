%% FMCW Radar + Kalman Tracker
clear; clc; close all;

% 1. Radar & Simulation Setup 
fc = 77e9;          % 77 GHz
c = 3e8;            
res = 1;            % 1m resolution
bw = c/(2*res);     % Bandwidth
range_max = 200;
t_sweep = 5.5 * 2 * range_max/c; 
slope = bw/t_sweep;
fs = 150e6;         % Sampling rate
t = linspace(0, t_sweep, t_sweep*fs);
num_chirps = 200;    
v1 = 3000;           
r_start = 100;       % Starting distance

%2. Kalman Filter Initialization
dt = t_sweep;       % Time step
% State Vector: [Position; Velocity]
A = [1 dt; 0 1];    % Transition Matrix
H = [1 0];          % Measurement Matrix 
Q = [0.1 0; 0 0.01];% Process Noise
R_cov = 1.5;        % Measurement Noise Covariance
P = eye(2);         % Initial Error Covariance
x_est = [100; 20];  % Initial Guess [Pos; Vel]

% Storage for plotting
kalman_pos = zeros(1, num_chirps);
measured_pos = zeros(1, num_chirps);
true_pos = zeros(1, num_chirps);

% 3. The Tracking Loop
fprintf('Starting Tracking Loop...\n');
for k = 1:num_chirps
    % Update Real Target Range (True Position)
    r_true = r_start + v1 * (k * dt);
    true_pos(k) = r_true;
    
    % --- Signal Generation (Simulating Radar Physics) ---
    tau = 2 * r_true / c;
    tau_ghost = 2 * 50 / c; % Static Ghost at 50m
    
    % Create Beat Signal with Noise
    tx = cos(2*pi*(fc*t + 0.5*slope*t.^2));
    rx = cos(2*pi*(fc*(t-tau) + 0.5*slope*(t-tau).^2)) + ...
         cos(2*pi*(fc*(t-tau_ghost) + 0.5*slope*(t-tau_ghost).^2)) + ...
         0.5*randn(size(t)); % Added Noise
    
    beat = tx .* rx;
    
    % Step 3: Peak Detection (Digital Signal Processing)
    L = length(beat);
    Y = fft(beat);
    P2 = abs(Y/L);
    P1 = P2(1:L/2+1);
    f = fs*(0:(L/2))/L;
    
    % Find detections
    [~, locs] = findpeaks(P1, f, 'MinPeakHeight', 0.05);
    detected_ranges = (locs * c * t_sweep) / (2 * bw);
    
    % Gating Logic: Pick the detection closest to last estimate
    if ~isempty(detected_ranges)
        [~, idx] = min(abs(detected_ranges - x_est(1)));
        z = detected_ranges(idx);
    else
        z = x_est(1); % Lost track, hold last pos
    end
    measured_pos(k) = z;

    % Step 4: Kalman Filter (State Estimation) 
    % Predict
    x_pred = A * x_est;
    P_pred = A * P * A' + Q;
    
    % Update
    K = P_pred * H' / (H * P_pred * H' + R_cov);
    x_est = x_pred + K * (z - H * x_pred);
    P = (eye(2) - K * H) * P_pred;
    
    kalman_pos(k) = x_est(1);
end

% 5. Final Results Visualization
figure('Color', 'k', 'Name', 'Radar Tracker');
plot(1:num_chirps, true_pos, 'g-', 'LineWidth', 2); hold on;
plot(1:num_chirps, measured_pos, 'rx', 'MarkerSize', 8);
plot(1:num_chirps, kalman_pos, 'b--', 'LineWidth', 2);
title('Radar Target Tracking: Kalman Filter vs Noisy FFT');
xlabel('Chirp Number'); ylabel('Distance (meters)');
legend('True Path', 'Noisy FFT Measurement', 'Kalman Filter Track');
grid on;
