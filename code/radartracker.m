%% FMCW Radar + Kalman Tracker (Live Analysis)
clear; clc; close all;

% 1. Radar & Simulation Setup
fc = 77e9;          % 77 GHz
c = 3e8;            
res = 1.5;          % Range resolution (~1.5m)
bw = c/(2*res);     % Bandwidth (100MHz)
range_max = 200;
t_sweep = 5.5 * 2 * range_max/c; 
slope = bw/t_sweep;
fs = 150e6;         
t = linspace(0, t_sweep, t_sweep*fs);

% SIMULATION PARAMETERS
num_chirps = 150;   
v1 = 1500;           % Velocity (m/s)
r_start = 80;       % Starting distance (m)

% 2. Kalman Filter Initialization
dt = t_sweep;       
A = [1 dt; 0 1];    % Transition Matrix
H = [1 0];          % Measurement Matrix
Q = [0.05 0; 0 0.005]; % Process Noise
R_cov = 2.5;        % Increased Measurement Noise for visual effect
P = eye(2);         
x_est = [r_start; v1]; 

% Storage
kalman_pos = zeros(1, num_chirps);
kalman_vel = zeros(1, num_chirps);
measured_pos = zeros(1, num_chirps);
true_pos = zeros(1, num_chirps);

% Setup Figure
h_fig = figure('Color', 'k', 'Name', 'Radar Intelligence System');

% 3. The Tracking Loop 
for k = 1:num_chirps
    % 3a. Update True Physics
    r_true = r_start + v1 * (k * dt); 
    true_pos(k) = r_true;
    
    % 3b. Signal Generation
    tau = 2 * r_true / c;
    tau_ghost = 2 * 50 / c; % Static Ghost at 50m
    
    tx = cos(2*pi*(fc*t + 0.5*slope*t.^2));
    rx = cos(2*pi*(fc*(t-tau) + 0.5*slope*(t-tau).^2)) + ...
         cos(2*pi*(fc*(t-tau_ghost) + 0.5*slope*(t-tau_ghost).^2)) + ...
         0.1*randn(size(t)); 
    
    beat = tx .* rx;
    
    % 3c. DSP: FFT & Peak Detection
    L = length(beat);
    Y = fft(beat);
    P2 = abs(Y/L);
    P1 = P2(1:L/2+1);
    f = fs*(0:(L/2))/L;
    
    [pks, locs] = findpeaks(P1, f, 'MinPeakHeight', 0.05);
    detected_ranges = (locs * c * t_sweep) / (2 * bw);
    
    % 3d. SMART GATING LOGIC (Fixes the Zero-Drop)
    if ~isempty(detected_ranges)
        % Association: Find detection closest to where the Kalman Filter predicted
        [val, idx] = min(abs(detected_ranges - x_est(1)));
        z_raw = detected_ranges(idx);
        % Add synthetic jitter to demonstrate Kalman smoothing
        z = z_raw + (randn * 1.2); 
    else
        % If no peak found, use the last estimated position (Freeze track)
        z = x_est(1);
    end
    measured_pos(k) = z;

    % 3e. Kalman Update
    x_pred = A * x_est;
    P_pred = A * P * A' + Q;
    K = P_pred * H' / (H * P_pred * H' + R_cov);
    x_est = x_pred + K * (z - H * x_pred);
    P = (eye(2) - K * H) * P_pred;
    
    kalman_pos(k) = x_est(1);
    kalman_vel(k) = x_est(2);

    % 4. HIGH-CONTRAST PLOTTING
    if mod(k, 5) == 0
        % Subplot 1: Raw Signals
        subplot(3,1,1);
        plot(f/1e6, P1, 'Color', [0 0.8 1], 'LineWidth', 1); hold on;
        stem(locs/1e6, pks, 'r', 'LineWidth', 1); hold off;
        title(['SIGNAL DOMAIN: Chirp #', num2str(k)], 'Color', 'w');
        ylabel('Amplitude', 'Color', 'w'); grid on;
        set(gca, 'Color', [0.1 0.1 0.1], 'XColor', 'w', 'YColor', 'w');
        xlim([0 80]); ylim([0 0.6]);

        % Subplot 2: Range Tracking
        subplot(3,1,2);
        plot(1:k, true_pos(1:k), 'g', 'LineWidth', 2); hold on;
        plot(1:k, measured_pos(1:k), 'rx', 'MarkerSize', 3); 
        plot(1:k, kalman_pos(1:k), 'y--', 'LineWidth', 1.5); hold off;
        title('SPATIAL DOMAIN: Range Tracking', 'Color', 'w');
        ylabel('Range (m)', 'Color', 'w'); grid on;
        set(gca, 'Color', [0.1 0.1 0.1], 'XColor', 'w', 'YColor', 'w');
        legend({'True', 'Noisy Sensor', 'Kalman'}, 'TextColor', 'w', 'Location', 'northwest');

        % Subplot 3: Velocity State
        subplot(3,1,3);
        plot(1:k, repmat(v1, 1, k), 'g', 'LineWidth', 1.5); hold on;
        plot(1:k, kalman_vel(1:k), 'c', 'LineWidth', 1.5); hold off;
        title('STATE ESTIMATION: Velocity Vector', 'Color', 'w');
        xlabel('Chirp Number', 'Color', 'w'); ylabel('Vel (m/s)', 'Color', 'w'); grid on;
        set(gca, 'Color', [0.1 0.1 0.1], 'XColor', 'w', 'YColor', 'w');
        drawnow;
    end
end
