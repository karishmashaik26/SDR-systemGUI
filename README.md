# SDR-systemGUI
%% Software Defined Radar System
% This script simulates a multi-frequency radar system, performs target detection
% using CFAR, and visualizes the results through an interactive GUI.
% 
% Features:
% - Multi-frequency radar simulation
% - Target detection using CFAR
% - Interactive GUI with signal visualization
% - Range-Doppler analysis
% - Simulink integration

%% Initialize Environment
clc; clear; close all;

%% Radar System Parameters
range_res = 30;                                  % Range resolution in meters
max_range = 12000;                               % Maximum detection range in meters
frequencies = [1e9, 3e9, 6e9, 10e9, 20e9];      % Operating frequencies in Hz
tx_gain = 38;                                    % Transmitter gain in dB
peak_power = 25000.0;                            % Peak transmit power in Watts
c = physconst('LightSpeed');                     % Speed of light constant

%% Define Radar Structure Template
paramRadarTemplate = struct(...
    'fc', 0, ...                  % Carrier frequency (Hz)
    'lambda', 0, ...              % Wavelength (m)
    'pulse_bw', 0, ...            % Pulse bandwidth (Hz)
    'pulse_length', 0, ...        % Pulse length (s)
    'fs', 0, ...                  % Sampling frequency (Hz)
    'noise_bw', 0, ...            % Noise bandwidth (Hz)
    'num_pulse_int', 0, ...       % Number of pulses for integration
    'prf', 0, ...                 % Pulse repetition frequency (Hz)
    'range_bins', [], ...         % Range bins (m)
    'rng_res', 0, ...             % Range resolution (m)
    'DopplerFFTbins', 0, ...      % Number of Doppler FFT bins
    'DopplerRes', 0, ...          % Doppler resolution (Hz)
    'DopplerOff', 0, ...          % Doppler offset (Hz)
    'RCS', [], ...                % Target radar cross section (m²)
    'targetPos', [], ...          % Target positions (m)
    'targetVel', [], ...          % Target velocities (m/s)
    'timeRes', 0, ...             % Time resolution (s)
    'max_range', 0, ...           % Maximum range (m)
    'tx_gain', 0, ...             % Transmitter gain (dB)
    'peak_power', 0, ...          % Peak power (W)
    'SNR', [], ...                % Signal-to-noise ratio
    'CFARResults', [], ...        % CFAR detection results
    'threshold', [] ...           % CFAR threshold values
);

% Initialize array of radar configurations for each frequency
paramRadarConfigs = repmat(paramRadarTemplate, length(frequencies), 1);

%% Target Definitions
%% Target Definitions

% Target Positions (x, y, z coordinates in meters)
target_positions = [
    1500.50, 4000.00, 6700.80, 2500.90, 5800.20, 3200;  % x-coordinates
    6300.75, 5000.00, 4200.20, 3800.55, 6000.10, 1800;  % y-coordinates
    5000.25, 3700.00, 5300.60, 2900.33, 4200.50, 1100   % z-coordinates
];

% Target Velocities (m/s)
target_velocities = [
    25,  0,  0,  0,  0,  0;   % Vehicle moving in x-direction at 25 m/s
    0,   0,  0,  0,  0,  0;   % All others stationary in y
    15,  0,  0,  0,  0,  0    % Drone moving in z-direction at 15 m/s
];

% Target RCS values (m²)
target_RCS = [14.0, 12.5, 17.2, 13.0, 10.8, 16.5];


%% CFAR Parameters
cfar_threshold = 2.8;         % Detection threshold factor
num_guard_cells = 5;          % Number of guard cells
num_training_cells = 16;      % Number of training cells

%% Process Each Frequency
for fIdx = 1:length(frequencies)
    % Configure radar parameters for current frequency
    paramRadarVisual = configureRadar(paramRadarTemplate, frequencies(fIdx), range_res, max_range, ...
                                      tx_gain, peak_power, c, target_positions, target_velocities, target_RCS);
    
    % Simulate received signal
    [received_signal, noise_floor] = simulateReceivedSignal(paramRadarVisual);
    
    % Apply CFAR detection
    [cfar_results, threshold] = applyCFAR(received_signal, num_guard_cells, num_training_cells, cfar_threshold);
    paramRadarVisual.CFARResults = cfar_results;
    paramRadarVisual.threshold = threshold;
    
    % Store configuration
    paramRadarConfigs(fIdx) = orderfields(paramRadarVisual, paramRadarTemplate);
    
    % Plot results for current frequency
    plotRadarResults(paramRadarVisual, received_signal, cfar_results, fIdx);
end

%% Print Detection Results
printDetectionResults(paramRadarConfigs, frequencies);

%% Launch GUI
radar_GUI(paramRadarConfigs);

%% Helper Functions

function paramRadar = configureRadar(template, frequency, range_res, max_range, tx_gain, peak_power, c, target_positions, target_velocities, target_RCS)
    % Configure radar parameters for a specific frequency
    paramRadar = template;
    paramRadar.fc = frequency;
    paramRadar.lambda = c / paramRadar.fc;
    paramRadar.pulse_bw = c / (2 * range_res);
    paramRadar.pulse_length = 1 / paramRadar.pulse_bw;
    paramRadar.fs = 2 * paramRadar.pulse_bw;
    paramRadar.noise_bw = paramRadar.pulse_bw;
    paramRadar.num_pulse_int = 16;
    paramRadar.prf = c / (3 * max_range);
    
    % Calculate range bins
    fast_time = unigrid(0, 1 / paramRadar.fs, 1 / paramRadar.prf, '[)');
    paramRadar.range_bins = c * fast_time / 2;
    paramRadar.rng_res = c / (2 * paramRadar.fs);
    
    % Configure Doppler processing
    paramRadar.DopplerFFTbins = 512;
    paramRadar.DopplerRes = (paramRadar.prf / paramRadar.DopplerFFTbins) / 2;
    paramRadar.DopplerOff = -paramRadar.prf / 4;
    
    % Set target parameters
    paramRadar.RCS = target_RCS;
    paramRadar.targetPos = target_positions;
    paramRadar.targetVel = target_velocities;
    
    % Set additional parameters
    paramRadar.timeRes = 16 / paramRadar.prf;
    paramRadar.max_range = max_range;
    paramRadar.tx_gain = tx_gain;
    paramRadar.peak_power = peak_power;
    
    % Initialize SNR field
    paramRadar.SNR = zeros(1, size(target_positions, 2));
end

function [received_signal, noise_floor] = simulateReceivedSignal(paramRadar)
    % Simulate received radar signal based on targets
    received_signal = zeros(1, length(paramRadar.range_bins));
    
    % Add thermal noise (based on radar parameters)
    k_boltzmann = 1.38e-23;  % Boltzmann constant
    T_sys = 290;             % System temperature in Kelvin
    noise_power = k_boltzmann * T_sys * paramRadar.noise_bw;
    noise_floor = sqrt(noise_power);
    received_signal = noise_floor * randn(size(received_signal));
    
    % Add target returns
    for t = 1:size(paramRadar.targetPos, 2)
        % Calculate 3D range to target
        target_range = norm(paramRadar.targetPos(:, t));
        
        if target_range <= paramRadar.max_range
            % Calculate received power using radar equation
            received_power = (paramRadar.peak_power * paramRadar.tx_gain * (paramRadar.lambda^2) * paramRadar.RCS(t)) / ...
                ((4 * pi)^3 * target_range^4);
            
            % Calculate SNR for this target
            paramRadar.SNR(t) = 10 * log10(received_power / noise_power);
            
            % Find closest range bin
            [~, idx] = min(abs(paramRadar.range_bins - target_range));
            
            % Add target return to signal (with some range spreading for realism)
            spread = max(1, round(paramRadar.pulse_length * paramRadar.fs / 2));
            for i = max(1, idx-spread):min(length(received_signal), idx+spread)
                % Apply a sinc-like spreading function
                dist_from_center = abs(i - idx);
                if dist_from_center == 0
                    received_signal(i) = received_signal(i) + sqrt(received_power);
                else
                    received_signal(i) = received_signal(i) + sqrt(received_power) * ...
                        (sin(pi * dist_from_center / spread) / (pi * dist_from_center / spread))^2 * 0.5;
                end
            end
        end
    end
    
    % Square the signal to get power
    received_signal = abs(received_signal).^2;
end

function [cfar_results, threshold] = applyCFAR(signal, num_guard_cells, num_training_cells, threshold_factor)
    % Apply Cell-Averaging CFAR for target detection
    signal_length = length(signal);
    cfar_results = zeros(1, signal_length);
    threshold = zeros(1, signal_length);
    
    % Process each cell
    for i = num_training_cells + num_guard_cells + 1 : signal_length - (num_training_cells + num_guard_cells)
        % Extract training cells (excluding guard cells)
        training_cells = [signal(i - num_training_cells - num_guard_cells : i - num_guard_cells - 1), ...
                          signal(i + num_guard_cells + 1 : i + num_guard_cells + num_training_cells)];
        
        % Calculate noise estimate from training cells
        noise_estimate = mean(training_cells);
        
        % Apply threshold
        threshold(i) = threshold_factor * noise_estimate;
        
        % Compare cell under test to threshold
        if signal(i) > threshold(i)
            cfar_results(i) = 1;  % Detection
        end
    end
end

function plotRadarResults(paramRadar, received_signal, cfar_results, frequency_index)
    % Plot received signal power vs range
    figure('Name', sprintf('Radar Results - %.2f GHz', paramRadar.fc / 1e9), 'Position', [100, 100, 900, 700]);
    
    % Plot 1: Received Power vs Range
    subplot(2, 1, 1);
    plot(paramRadar.range_bins, 10*log10(received_signal), 'b', 'LineWidth', 1.5);
    title(sprintf('Received Power vs Range for %.2f GHz', paramRadar.fc / 1e9));
    xlabel('Range (m)');
    ylabel('Received Power (dB)');
    grid on;
    
    % Plot 2: CFAR Detection Results
    subplot(2, 1, 2);
    stem(paramRadar.range_bins, cfar_results, 'r', 'Marker', 'none', 'LineWidth', 1);
    hold on;
    
    % Plot threshold if available
    if ~isempty(paramRadar.threshold) && any(paramRadar.threshold > 0)
        plot(paramRadar.range_bins, 10*log10(paramRadar.threshold), 'g--', 'LineWidth', 1);
    end
    
    % Highlight detected targets
    detected_bins = find(cfar_results == 1);
    scatter(paramRadar.range_bins(detected_bins), ones(size(detected_bins)), 100, 'bo', 'LineWidth', 2);
    
    % Add target ground truth markers
    for t = 1:size(paramRadar.targetPos, 2)
        target_range = norm(paramRadar.targetPos(:, t));
        if target_range <= max(paramRadar.range_bins)
            plot(target_range, 0.5, 'gd', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
            text(target_range, 0.6, sprintf('T%d', t), 'FontWeight', 'bold');
        end
    end
    
    hold off;
    title(sprintf('CFAR Detection for %.2f GHz', paramRadar.fc / 1e9));
    xlabel('Range (m)');
    ylabel('Detection (1 = Target, 0 = Noise)');
    ylim([0 1.2]);
    grid on;
    legend('CFAR Output', 'Threshold', 'Detected Targets', 'Ground Truth');
end

function printDetectionResults(paramRadarConfigs, frequencies)
    % Print detection results to command window
    fprintf('\n==================== RADAR DETECTION RESULTS ====================\n');
    
    for fIdx = 1:length(frequencies)
        fprintf('\n[Frequency: %.2f GHz]\n', frequencies(fIdx) / 1e9);
        fprintf('-------------------------------------------------\n');
        
        % Print target positions
        targetPos = paramRadarConfigs(fIdx).targetPos;
        numTargets = size(targetPos, 2);
        fprintf('Ground Truth Targets:\n');
        for t = 1:numTargets
            target_range = norm(targetPos(:, t));
            fprintf('Target %d: Position (%.2f, %.2f, %.2f) m, Range: %.2f m, SNR: %.2f dB\n', ...
                    t, targetPos(1,t), targetPos(2,t), targetPos(3,t), target_range, ...
                    paramRadarConfigs(fIdx).SNR(t));
        end
        
        fprintf('-------------------------------------------------\n');
        
        % Print detected targets
        detected_bins = find(paramRadarConfigs(fIdx).CFARResults == 1);
        detected_ranges = paramRadarConfigs(fIdx).range_bins(detected_bins);
        
        if isempty(detected_ranges)
            fprintf('No targets detected at this frequency.\n');
        else
            fprintf('Detected Targets at Ranges (m):\n');
            
        end
        
        fprintf('-------------------------------------------------\n');
    end
    
    fprintf('===============================================================\n');
end

function grouped = groupDetections(detections, min_separation)
    % Group closely spaced detections
    if isempty(detections)
        grouped = {};
        return;
    end
    
    % Sort detections
    sorted_detections = sort(detections);
    
    % Initialize groups
    grouped = {};
    current_group = [sorted_detections(1)];
    
    % Group detections
    for i = 2:length(sorted_detections)
        if sorted_detections(i) - sorted_detections(i-1) <= min_separation
            % Add to current group
            current_group = [current_group, sorted_detections(i)];
        else
            % Start new group
            grouped{end+1} = current_group;
            current_group = [sorted_detections(i)];
        end
    end
    
    % Add last group
    grouped{end+1} = current_group;
end

function radar_GUI(paramRadarConfigs)
    % Create Main Figure Window
    fig = figure('Name', 'Software Defined Radar System', ...
                 'NumberTitle', 'off', ...
                 'MenuBar', 'none', ...
                 'ToolBar', 'figure', ...
                 'Color', [0.8 0.85 0.9], ...
                 'Position', [100, 100, 1200, 800], ...
                 'Resize', 'on');
    
   
% Add Title Text - SOFTWARE DEFINED RADAR SYSTEM
uicontrol('Style', 'text', ...
          'String', 'SOFTWARE DEFINED RADAR SYSTEM', ...
          'FontSize', 16, ...
          'FontWeight', 'bold', ...
          'BackgroundColor', [0.2 0.3 0.4], ...
          'ForegroundColor', 'white', ...
          'Units', 'normalized', ...
          'Position', [0.25, 0.94, 0.5, 0.05]); % Centered & moved up

uicontrol('Style', 'text', ...
          'String', 'Frequency:', ...
          'FontSize', 12, ...
          'BackgroundColor', [0.2 0.3 0.4], ...
          'ForegroundColor', 'white', ...
          'Units', 'normalized', ...
          'Position', [0.35, 0.89, 0.1, 0.04]); % Adjusted position below title

    
    % Divide GUI into sections (using 'normalized' units for auto-resizing)

 % Adjusted Positions for Four Balanced Plots (Reduced Size)

% Top-Left: Radar Signal Plot (Smaller & Well-Placed)
ax1 = axes('Parent', fig, ...
           'Units', 'normalized', ...
           'Position', [0.08, 0.58, 0.35, 0.30]); % Shifted left & reduced

% Top-Right: Range-Doppler Plot (Smaller & Well-Placed)
ax2 = axes('Parent', fig, ...
           'Units', 'normalized', ...
           'Position', [0.55, 0.58, 0.35, 0.30]); % Shifted right & reduced

% Bottom-Left: Target Detection Plot (Smaller & Well-Placed)
ax3 = axes('Parent', fig, ...
           'Units', 'normalized', ...
           'Position', [0.08, 0.18, 0.35, 0.30]); % Shifted left & reduced

% Bottom-Right: 3D Target Visualization (Smaller & Well-Placed)
ax4 = axes('Parent', fig, ...
           'Units', 'normalized', ...
           'Position', [0.55, 0.18, 0.35, 0.30]); % Shifted right & reduced

% Results Box (Centered Below the Plots, Slightly Smaller)
resultsBox = uicontrol('Style', 'listbox', ...
                       'FontSize', 11, ...
                       'FontName', 'Consolas', ...
                       'BackgroundColor', [0.95 0.95 0.95], ...
                       'Units', 'normalized', ...
                       'Position', [0.08, 0.03, 0.82, 0.08]); % Adjusted size & position
% Add frequency selection dropdown (Shifted Left)
freqSelector = uicontrol('Style', 'popupmenu', ...
                        'String', arrayfun(@(f) sprintf('%.2f GHz', f/1e9), ...
                                          [paramRadarConfigs.fc], 'UniformOutput', false), ...
                        'FontSize', 12, ...
                        'Units', 'normalized', ...
                        'Position', [0.44, 0.89, 0.15, 0.04], ... % Shifted left
                        'Callback', @(src, event) updateDisplays(src, event, ...
                                                                paramRadarConfigs, ...
                                                                ax1, ax2, ax3, ax4, ...
                                                                resultsBox));

 
    % Create Menu Bar for Control Options
    createMenuBar(fig, paramRadarConfigs, ax1, ax2, ax3, ax4, resultsBox, freqSelector);
    
    % Populate Results in Listbox
    populateResults(resultsBox, paramRadarConfigs, 1);
    
    % Initial Plots
    updateDisplays(freqSelector, [], paramRadarConfigs, ax1, ax2, ax3, ax4, resultsBox);
    
    % Add status bar
    uicontrol('Style', 'text', ...
              'String', 'Ready', ...
              'FontSize', 10, ...
              'BackgroundColor', [0.2 0.3 0.4], ...
              'ForegroundColor', [0.8 0.8 0.8], ...
              'HorizontalAlignment', 'left', ...
              'Units', 'normalized', ...
              'Position', [0, 0, 1, 0.02])
end

function updateDisplays(src, ~, paramRadarConfigs, ax1, ax2, ax3, ax4, resultsBox)
    % Get selected frequency index
    freqIdx = get(src, 'Value');
    
    % Update all displays with the selected radar configuration
    plotRadarSignal(paramRadarConfigs(freqIdx), ax1);
    rangeDopplerPlot(paramRadarConfigs(freqIdx), ax2);
    plotDetectionResults(paramRadarConfigs(freqIdx), ax3);
    plot3DTargets(paramRadarConfigs(freqIdx), ax4);
    
    % Update results box
    populateResults(resultsBox, paramRadarConfigs, freqIdx);
end

function createMenuBar(fig, paramRadarConfigs, ax1, ax2, ax3, ax4, resultsBox, freqSelector)
    % File Menu
    fileMenu = uimenu(fig, 'Label', 'File');
    uimenu(fileMenu, 'Label', 'Export Results...', ...
           'Callback', @(src, event) exportResults(paramRadarConfigs));
    uimenu(fileMenu, 'Label', 'Save Plots...', ...
           'Callback', @(src, event) savePlots(fig));
    uimenu(fileMenu, 'Label', 'Exit', ...
           'Separator', 'on', ...
           'Callback', @(src, event) close(fig));
    
    % View Menu
    viewMenu = uimenu(fig, 'Label', 'View');
    uimenu(viewMenu, 'Label', 'Range Profile', ...
           'Checked', 'on', ...
           'Callback', @(src, event) toggleVisibility(ax1));
    uimenu(viewMenu, 'Label', 'Range-Doppler Map', ...
           'Checked', 'on', ...
           'Callback', @(src, event) toggleVisibility(ax2));
    uimenu(viewMenu, 'Label', 'Detection Results', ...
           'Checked', 'on', ...
           'Callback', @(src, event) toggleVisibility(ax3));
    uimenu(viewMenu, 'Label', '3D Visualization', ...
           'Checked', 'on', ...
           'Callback', @(src, event) toggleVisibility(ax4));
    
    % Simulink Menu
    simMenu = uimenu(fig, 'Label', 'Simulink');
    uimenu(simMenu, 'Label', 'Run Simulink Model', ...
           'Callback', @(src, event) runSimulinkModel());
    uimenu(simMenu, 'Label', 'Stop Simulink Model', ...
           'Callback', @(src, event) stopSimulinkModel());
    uimenu(simMenu, 'Label', 'Configure Model Parameters...', ...
           'Separator', 'on', ...
           'Callback', @(src, event) configureSimulinkModel());
    
    % Radar Settings Menu
    settingsMenu = uimenu(fig, 'Label', 'Radar Settings');
    uimenu(settingsMenu, 'Label', 'Update Parameters...', ...
           'Callback', @(src, event) updateRadarParams(paramRadarConfigs, freqSelector, ...
                                                     ax1, ax2, ax3, ax4, resultsBox));
    uimenu(settingsMenu, 'Label', 'CFAR Settings...', ...
           'Callback', @(src, event) updateCFARSettings(paramRadarConfigs, ...
                                                       freqSelector, ax3));
    uimenu(settingsMenu, 'Label', 'Reset to Defaults', ...
           'Separator', 'on', ...
           'Callback', @(src, event) resetToDefaults());
    
    % Analysis Menu
    analysisMenu = uimenu(fig, 'Label', 'Analysis');
    uimenu(analysisMenu, 'Label', 'Performance Metrics...', ...
           'Callback', @(src, event) showPerformanceMetrics(paramRadarConfigs));
    uimenu(analysisMenu, 'Label', 'Compare Frequencies...', ...
           'Callback', @(src, event) compareFrequencies(paramRadarConfigs));
    
    % Help Menu
    helpMenu = uimenu(fig, 'Label', 'Help');
    uimenu(helpMenu, 'Label', 'User Guide', ...
           'Callback', @(src, event) showUserGuide());
    uimenu(helpMenu, 'Label', 'About', ...
           'Callback', @(src, event) showAboutDialog());
end


function populateResults(resultsBox, paramRadarConfigs, freqIdx)
    % Populate results in the listbox for the selected frequency
    results = {};
    
    % Add header
    results{end+1} = sprintf('===== RADAR DETECTION RESULTS (%.2f GHz) =====', ...
                            paramRadarConfigs(freqIdx).fc / 1e9);
    
    % Add radar parameters
    results{end+1} = sprintf('Pulse Length: %.2f μs | PRF: %.2f kHz | Range Resolution: %.2f m', ...
                            paramRadarConfigs(freqIdx).pulse_length * 1e6, ...
                            paramRadarConfigs(freqIdx).prf / 1e3, ...
                            paramRadarConfigs(freqIdx).rng_res);
    
    results{end+1} = '-------------------------------------------------';
    
    % Add target information
    targetPos = paramRadarConfigs(freqIdx).targetPos;
    numTargets = size(targetPos, 2);
    
    results{end+1} = 'Ground Truth Targets:';
    for t = 1:numTargets
        target_range = norm(targetPos(:, t));
        results{end+1} = sprintf('Target %d: Range %.2f m | SNR: %.1f dB | RCS: %.1f m²', ...
                                t, target_range, ...
                                paramRadarConfigs(freqIdx).SNR(t), ...
                                paramRadarConfigs(freqIdx).RCS(t));
    end
    
    results{end+1} = '-------------------------------------------------';
    
    % Add detection results
    detected_bins = find(paramRadarConfigs(freqIdx).CFARResults == 1);
    detected_ranges = paramRadarConfigs(freqIdx).range_bins(detected_bins);
    
    if isempty(detected_ranges)
        results{end+1} = 'No targets detected at this frequency.';
    else
        results{end+1} = sprintf('Detected %d target(s) at ranges:', length(detected_ranges));
        
        % Format detected ranges in columns
        range_str = '';
        for i = 1:length(detected_ranges)
            range_str = [range_str, sprintf('%.2f m', detected_ranges(i))];
            if mod(i, 4) == 0 && i < length(detected_ranges)
                results{end+1} = range_str;
                range_str = '';
            elseif i < length(detected_ranges)
                range_str = [range_str, ' | '];
            end
        end
        
        if ~isempty(range_str)
            results{end+1} = range_str;
        end
    end
    
    % Set results into the listbox
    set(resultsBox, 'String', results);
end

function plotRadarSignal(paramRadar, ax1)
    % Plot radar signal waveform
    t = linspace(0, paramRadar.pulse_length * 5, 1000); 
    
    % Create a more realistic pulse with envelope
    envelope = exp(-5 * t / paramRadar.pulse_length);
    carrier = sin(2 * pi * paramRadar.fc * t);
    
    % For visualization, we can't show the actual carrier frequency (too high)
    % So we use a lower frequency just to show the concept
    vis_freq = 20 / paramRadar.pulse_length;
    vis_carrier = sin(2 * pi * vis_freq * t);
    
    % Create the pulse
    pulse = vis_carrier .* envelope;
    
    % Plot
    axes(ax1);
    cla(ax1);
    plot(ax1, t * 1e6, pulse, 'b', 'LineWidth', 1.5);
    hold(ax1, 'on');
    plot(ax1, t * 1e6, envelope, 'r--', 'LineWidth', 1);
    hold(ax1, 'off');
    
    title(ax1, sprintf('Radar Pulse (%.2f GHz)', paramRadar.fc/1e9));
    xlabel(ax1, 'Time (μs)');
    ylabel(ax1, 'Amplitude');
    grid(ax1, 'on');
    legend(ax1, 'Pulse', 'Envelope');
end

function rangeDopplerPlot(paramRadar, ax2)
    % Generate a simulated Range-Doppler map
    
    % Create range and Doppler axes
    range_axis = paramRadar.range_bins;
    doppler_axis = linspace(paramRadar.DopplerOff, paramRadar.prf/2, paramRadar.DopplerFFTbins);
    
    % Initialize the Range-Doppler map with noise
    rng(0); % For reproducibility
    rd_map = 0.1 * abs(randn(paramRadar.DopplerFFTbins, length(range_axis)) + ...
                      1j * randn(paramRadar.DopplerFFTbins, length(range_axis)));
    
    % Add targets to the Range-Doppler map
    for t = 1:size(paramRadar.targetPos, 2)
        % Calculate target range and radial velocity
        target_range = norm(paramRadar.targetPos(:, t));
        
        % Calculate radial velocity (projection of velocity vector onto line-of-sight)
        target_vel = paramRadar.targetVel(:, t);
        unit_los = paramRadar.targetPos(:, t) / target_range;
        radial_vel = dot(target_vel, unit_los);
        
        % Convert to Doppler frequency
        doppler_freq = 2 * radial_vel / paramRadar.lambda;
        
        % Find closest range and Doppler bins
        [~, range_idx] = min(abs(range_axis - target_range));
        [~, doppler_idx] = min(abs(doppler_axis - doppler_freq));
        
        % Add target with appropriate amplitude (based on RCS and SNR)
        target_amp = 10^(paramRadar.SNR(t)/20); % Convert SNR from dB
        
        % Add target with some spreading
        range_spread = 3;
        doppler_spread = 5;
        
        for r = max(1, range_idx-range_spread):min(length(range_axis), range_idx+range_spread)
            for d = max(1, doppler_idx-doppler_spread):min(length(doppler_axis), doppler_idx+doppler_spread)
                % Apply a 2D Gaussian spreading
                r_dist = (r - range_idx)^2 / range_spread^2;
                d_dist = (d - doppler_idx)^2 / doppler_spread^2;
                rd_map(d, r) = rd_map(d, r) + target_amp * exp(-(r_dist + d_dist));
            end
        end
    end
    
    % Convert to dB for display
    rd_map_db = 20 * log10(rd_map);
    
    % Plot
    axes(ax2);
    cla(ax2);
    
    % Use imagesc for better visualization
    imagesc(ax2, range_axis, doppler_axis, rd_map_db);
    
    % Add colormap and colorbar
    colormap(ax2, 'jet');
    c = colorbar(ax2);
    c.Label.String = 'Power (dB)';
    
    % Set axis labels and title
    title(ax2, 'Range-Doppler Map');
    xlabel(ax2, 'Range (m)');
    ylabel(ax2, 'Doppler Frequency (Hz)');
    
    % Add target markers
    hold(ax2, 'on');
    for t = 1:size(paramRadar.targetPos, 2)
        target_range = norm(paramRadar.targetPos(:, t));
        
        % Calculate radial velocity
        target_vel = paramRadar.targetVel(:, t);
        unit_los = paramRadar.targetPos(:, t) / target_range;
        radial_vel = dot(target_vel, unit_los);
        
        % Convert to Doppler frequency
        doppler_freq = 2 * radial_vel / paramRadar.lambda;
        
        % Plot target marker if within range and Doppler limits
        if target_range <= max(range_axis) && ...
           doppler_freq >= min(doppler_axis) && doppler_freq <= max(doppler_axis)
            plot(ax2, target_range, doppler_freq, 'wo', 'MarkerSize', 10, 'LineWidth', 1.5);
            text(ax2, target_range, doppler_freq, sprintf(' T%d', t), ...
                'Color', 'w', 'FontWeight', 'bold');
        end
    end
    hold(ax2, 'off');
end

function plotDetectionResults(paramRadar, ax3)
    % Plot detection results with ground truth
    
    % Generate received signal (simplified)
    received_signal = zeros(1, length(paramRadar.range_bins));
    
    % Add noise floor
    noise_floor = 1e-12;
    received_signal = noise_floor * ones(size(received_signal));
    
    % Add targets
    for t = 1:size(paramRadar.targetPos, 2)
        target_range = norm(paramRadar.targetPos(:, t));
        
        if target_range <= max(paramRadar.range_bins)
            % Calculate received power
            received_power = paramRadar.peak_power * (paramRadar.lambda^2) * paramRadar.RCS(t) / ...
                ((4 * pi)^3 * target_range^4);
            
            % Find closest range bin
            [~, idx] = min(abs(paramRadar.range_bins - target_range));
            
            % Add target with some spreading
            spread = max(1, round(5 * paramRadar.rng_res / mean(diff(paramRadar.range_bins))));
            for i = max(1, idx-spread):min(length(received_signal), idx+spread)
                dist = abs(i - idx);
                if dist == 0
                    received_signal(i) = received_signal(i) + received_power;
                else
                    received_signal(i) = received_signal(i) + received_power * exp(-dist^2/spread);
                end
            end
        end
    end
    
    % Convert to dB
    received_signal_db = 10 * log10(received_signal);
    
    % Plot
    axes(ax3);
    cla(ax3);
    
    % Plot received signal
    plot(ax3, paramRadar.range_bins, received_signal_db, 'b-', 'LineWidth', 1.5);
    hold(ax3, 'on');
    
    % Plot detection threshold (if available)
    if isfield(paramRadar, 'threshold') && ~isempty(paramRadar.threshold)
        threshold_db = 10*log10(paramRadar.threshold);
        threshold_db(isinf(threshold_db)) = min(received_signal_db);
        plot(ax3, paramRadar.range_bins, threshold_db, 'r--', 'LineWidth', 1);
    end
    
    % Plot CFAR detections
    detected_bins = find(paramRadar.CFARResults == 1);
    if ~isempty(detected_bins)
        scatter(ax3, paramRadar.range_bins(detected_bins), ...
               received_signal_db(detected_bins), 100, 'ro', 'filled');
    end
    
    % Plot ground truth target positions
    for t = 1:size(paramRadar.targetPos, 2)
        target_range = norm(paramRadar.targetPos(:, t));
        if target_range <= max(paramRadar.range_bins)
            % Find closest range bin for y-coordinate
            [~, idx] = min(abs(paramRadar.range_bins - target_range));
            y_pos = received_signal_db(idx);
            
            % Plot ground truth marker
            plot(ax3, target_range, y_pos, 'gd', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
            text(ax3, target_range, y_pos + 3, sprintf('T%d', t), 'FontWeight', 'bold');
        end
    end
    
    hold(ax3, 'off');
    
    % Set axis labels and title
    title(ax3, 'Target Detection Results');
    xlabel(ax3, 'Range (m)');
    ylabel(ax3, 'Power (dB)');
    grid(ax3, 'on');
    
    % Add legend
    if isfield(paramRadar, 'threshold') && ~isempty(paramRadar.threshold)
        legend(ax3, 'Received Signal', 'CFAR Threshold', 'Detections', 'Ground Truth', ...
              'Location', 'best');
    else
        legend(ax3, 'Received Signal', 'Detections', 'Ground Truth', 'Location', 'best');
    end
end

function plot3DTargets(paramRadar, ax4)
    % Plot 3D visualization of targets
    
    % Clear the axes
    axes(ax4);
    cla(ax4);
    
    % Set up the 3D view
    view(ax4, 3);
    grid(ax4, 'on');
    box(ax4, 'on');
    
    % Set axis labels
    xlabel(ax4, 'X (m)');
    ylabel(ax4, 'Y (m)');
    zlabel(ax4, 'Z (m)');
    title(ax4, '3D Target Visualization');
    
    % Plot radar position at origin
    plot3(ax4, 0, 0, 0, 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
    text(ax4, 0, 0, 0, ' Radar', 'FontWeight', 'bold');
    
    hold(ax4, 'on');
    
    % Plot targets
    for t = 1:size(paramRadar.targetPos, 2)
        % Extract target position
        x = paramRadar.targetPos(1, t);
        y = paramRadar.targetPos(2, t);
        z = paramRadar.targetPos(3, t);
        
        % Calculate target range
        target_range = norm(paramRadar.targetPos(:, t));
        
        % Plot target
        if target_range <= paramRadar.max_range
            % Determine marker size based on RCS
            marker_size = 20 + paramRadar.RCS(t) / 2;
            
            % Determine color based on SNR
            snr_normalized = (paramRadar.SNR(t) + 20) / 40;  % Normalize to 0-1 range
            snr_normalized = max(0, min(1, snr_normalized));  % Clamp to 0-1
            color = [0, snr_normalized, 1-snr_normalized];  % Blue to green gradient
            
            % Plot target
            scatter3(ax4, x, y, z, marker_size, color, 'filled');
            
            % Add target label
            text(ax4, x, y, z, sprintf(' T%d', t), 'FontWeight', 'bold');
            
            % Draw line from radar to target
            plot3(ax4, [0, x], [0, y], [0, z], 'k--', 'LineWidth', 0.5);
            
            % Add velocity vector if non-zero
            if any(paramRadar.targetVel(:, t))
                % Extract velocity components
                vx = paramRadar.targetVel(1, t);
                vy = paramRadar.targetVel(2, t);
                vz = paramRadar.targetVel(3, t);
                
                % Scale velocity vector for visualization
                scale = 0.5;
                quiver3(ax4, x, y, z, vx*scale, vy*scale, vz*scale, 0, 'r', 'LineWidth', 2);
            end
        end
    end
    
    % Draw range rings on the ground plane
    theta = linspace(0, 2*pi, 100);
    range_rings = [2000, 4000, 6000, 8000, 10000];
    
    for r = range_rings
        if r <= paramRadar.max_range
            x_ring = r * cos(theta);
            y_ring = r * sin(theta);
            z_ring = zeros(size(theta));
            plot3(ax4, x_ring, y_ring, z_ring, 'k:', 'LineWidth', 0.5);
            
            % Add range label
            text(ax4, r, 0, 0, sprintf(' %d km', r/1000), 'FontSize', 8);
        end
    end
    
    % Set axis limits
    max_coord = max(paramRadar.max_range, max(abs(paramRadar.targetPos(:))));
    axis(ax4, [-max_coord, max_coord, -max_coord, max_coord, 0, max_coord]);
    
    hold(ax4, 'off');
end

function toggleVisibility(ax)
    % Toggle visibility of an axes
    if strcmp(get(ax, 'Visible'), 'on')
        set(ax, 'Visible', 'off');
    else
        set(ax, 'Visible', 'on');
    end
end

function exportResults(paramRadarConfigs)
    % Export results to a file
    [file, path] = uiputfile({'*.mat', 'MATLAB Data (*.mat)'; ...
                             '*.csv', 'CSV File (*.csv)'; ...
                             '*.txt', 'Text File (*.txt)'}, ...
                             'Export Results As');
    
    if file ~= 0
        fullpath = fullfile(path, file);
        [~, ~, ext] = fileparts(fullpath);
        
        try
            switch lower(ext)
                case '.mat'
                    save(fullpath, 'paramRadarConfigs');
                    msgbox('Results exported successfully to MAT file.', 'Export Complete');
                    
                case '.csv'
                    % Create a table for export
                    exportTable = createExportTable(paramRadarConfigs);
                    writetable(exportTable, fullpath);
                    msgbox('Results exported successfully to CSV file.', 'Export Complete');
                    
                case '.txt'
                    % Create a formatted text report
                    fid = fopen(fullpath, 'w');
                    writeTextReport(fid, paramRadarConfigs);
                    fclose(fid);
                    msgbox('Results exported successfully to text file.', 'Export Complete');
                    
                otherwise
                    errordlg('Unsupported file format.', 'Export Error');
            end
        catch ME
            errordlg(['Error exporting results: ', ME.message], 'Export Error');
        end
    end
end

function exportTable = createExportTable(paramRadarConfigs)
    % Create a table for CSV export
    numFreqs = length(paramRadarConfigs);
    
    % Initialize arrays for table
    frequency = zeros(numFreqs, 1);
    wavelength = zeros(numFreqs, 1);
    pulseLength = zeros(numFreqs, 1);
    prf = zeros(numFreqs, 1);
    rangeRes = zeros(numFreqs, 1);
    numDetections = zeros(numFreqs, 1);
    
    % Fill arrays
    for i = 1:numFreqs
        frequency(i) = paramRadarConfigs(i).fc / 1e9; % GHz
        wavelength(i) = paramRadarConfigs(i).lambda * 100; % cm
        pulseLength(i) = paramRadarConfigs(i).pulse_length * 1e6; % μs
        prf(i) = paramRadarConfigs(i).prf / 1e3; % kHz
        rangeRes(i) = paramRadarConfigs(i).rng_res; % m
        numDetections(i) = sum(paramRadarConfigs(i).CFARResults);
    end
    
    % Create table
    exportTable = table(frequency, wavelength, pulseLength, prf, rangeRes, numDetections, ...
                       'VariableNames', {'Frequency_GHz', 'Wavelength_cm', 'PulseLength_us', ...
                                        'PRF_kHz', 'RangeResolution_m', 'NumDetections'});
end

function writeTextReport(fid, paramRadarConfigs)
    % Write a formatted text report
    fprintf(fid, '==================== RADAR DETECTION REPORT ====================\n\n');
    fprintf(fid, 'Report Generated: %s\n\n', datestr(now));
    
    fprintf(fid, 'RADAR SYSTEM PARAMETERS:\n');
    fprintf(fid, '  Peak Power: %.1f W\n', paramRadarConfigs(1).peak_power);
    fprintf(fid, '  Transmitter Gain: %.1f dB\n', paramRadarConfigs(1).tx_gain);
    fprintf(fid, '  Maximum Range: %.1f m\n\n', paramRadarConfigs(1).max_range);
    
    fprintf(fid, 'FREQUENCY-SPECIFIC RESULTS:\n');
    
    for fIdx = 1:length(paramRadarConfigs)
        fprintf(fid, '\n[Frequency: %.2f GHz]\n', paramRadarConfigs(fIdx).fc / 1e9);
        fprintf(fid, '-------------------------------------------------\n');
       
        fprintf(fid, 'Radar Parameters:\n');
        fprintf(fid, '  Wavelength: %.2f cm\n', paramRadarConfigs(fIdx).lambda * 100);
        fprintf(fid, '  Pulse Length: %.2f μs\n', paramRadarConfigs(fIdx).pulse_length * 1e6);
        fprintf(fid, '  PRF: %.2f kHz\n', paramRadarConfigs(fIdx).prf / 1e3);
        fprintf(fid, '  Range Resolution: %.2f m\n', paramRadarConfigs(fIdx).rng_res);
        fprintf(fid, '  Doppler Resolution: %.2f Hz\n\n', paramRadarConfigs(fIdx).DopplerRes);
        
        fprintf(fid, 'Target Information:\n');
        targetPos = paramRadarConfigs(fIdx).targetPos;
        numTargets = size(targetPos, 2);
        
        for t = 1:numTargets
            target_range = norm(targetPos(:, t));
            fprintf(fid, '  Target %d: Range %.2f m, SNR: %.1f dB, RCS: %.1f m²\n', ...
                   t, target_range, paramRadarConfigs(fIdx).SNR(t), paramRadarConfigs(fIdx).RCS(t));
        end
        
        fprintf(fid, '\nDetection Results:\n');
        detected_bins = find(paramRadarConfigs(fIdx).CFARResults == 1);
        detected_ranges = paramRadarConfigs(fIdx).range_bins(detected_bins);
        
        if isempty(detected_ranges)
            fprintf(fid, '  No targets detected at this frequency.\n');
        else
            fprintf(fid, '  Detected %d target(s) at ranges:\n', length(detected_ranges));
            for i = 1:length(detected_ranges)
                fprintf(fid, '    %.2f m\n', detected_ranges(i));
            end
        end
        
        fprintf(fid, '-------------------------------------------------\n');
    end
    
    fprintf(fid, '\n===============================================================\n');
end

function savePlots(fig)
    % Save all plots in the figure
    [file, path] = uiputfile({'*.fig', 'MATLAB Figure (*.fig)'; ...
                             '*.png', 'PNG Image (*.png)'; ...
                             '*.jpg', 'JPEG Image (*.jpg)'}, ...
                             'Save Plots As');
    
    if file ~= 0
        fullpath = fullfile(path, file);
        [~, name, ext] = fileparts(fullpath);
        
        try
            switch lower(ext)
                case '.fig'
                    savefig(fig, fullpath);
                    msgbox('Figure saved successfully.', 'Save Complete');
                    
                case {'.png', '.jpg'}
                    % Save the entire figure
                    print(fig, fullpath, ['-d', ext(2:end)], '-r300');
                    msgbox('Image saved successfully.', 'Save Complete');
                    
                otherwise
                    errordlg('Unsupported file format.', 'Save Error');
            end
        catch ME
            errordlg(['Error saving plots: ', ME.message], 'Save Error');
        end
    end
end

function runSimulinkModel()
    % Check if Simulink is installed
    if ~exist('simulink', 'file')
        errordlg('Simulink is not installed or not in the MATLAB path.', 'Simulink Error');
        return;
    end
    
    % Create a simple dialog to select or create a model
    modelFig = figure('Name', 'Run Simulink Model', ...
                     'NumberTitle', 'off', ...
                     'Position', [300, 300, 400, 200], ...
                     'MenuBar', 'none', ...
                     'ToolBar', 'none');
    
    uicontrol('Parent', modelFig, ...
              'Style', 'text', ...
              'String', 'Select Simulink Model:', ...
              'Position', [20, 150, 360, 30], ...
              'FontSize', 12);
    
    % Create a list of available models
    modelList = {'Create New Model', 'Browse for Model...'};
    
    % Add any models in the current directory
    mdlFiles = dir('*.slx');
    for i = 1:length(mdlFiles)
        modelList{end+1} = mdlFiles(i).name;
    end
    
    modelSelector = uicontrol('Parent', modelFig, ...
                             'Style', 'popupmenu', ...
                             'String', modelList, ...
                             'Position', [20, 120, 360, 30], ...
                             'FontSize', 12);
    
    % Add buttons
    uicontrol('Parent', modelFig, ...
              'Style', 'pushbutton', ...
              'String', 'Run', ...
              'Position', [80, 50, 100, 40], ...
              'FontSize', 12, ...
              'Callback', @(src, event) runSelectedModel(modelSelector, modelFig));
    
    uicontrol('Parent', modelFig, ...
              'Style', 'pushbutton', ...
              'String', 'Cancel', ...
              'Position', [220, 50, 100, 40], ...
              'FontSize', 12, ...
              'Callback', @(src, event) close(modelFig));
end

function runSelectedModel(modelSelector, modelFig)
    % Get selected model
    selection = get(modelSelector, 'Value');
    modelList = get(modelSelector, 'String');
    selectedModel = modelList{selection};
    
    % Close the dialog
    close(modelFig);
    
    try
        switch selectedModel
            case 'Create New Model'
                % Create a new model with radar blocks
                createRadarModel();
                
            case 'Browse for Model...'
                % Open file browser
                [file, path] = uigetfile({'*.slx;*.mdl', 'Simulink Models (*.slx, *.mdl)'});
                if file ~= 0
                    modelPath = fullfile(path, file);
                    open_system(modelPath);
                    set_param(bdroot, 'SimulationCommand', 'start');
                end
                
            otherwise
                % Open the selected model
                open_system(selectedModel);
                set_param(bdroot, 'SimulationCommand', 'start');
        end
    catch ME
        errordlg(['Error running model: ', ME.message], 'Simulink Error');
    end
end

function createRadarModel()
    % Create a new Simulink model for radar simulation
    try
        % Create new model
        modelName = 'RadarSimulation';
        
        % Check if model already exists
        if bdIsLoaded(modelName)
            close_system(modelName, 0);
        end
        
        new_system(modelName);
        open_system(modelName);
        
        % Add blocks (basic radar simulation)
        % Signal generator
        add_block('simulink/Sources/Sine Wave', [modelName, '/Signal Generator'], ...
                 'Position', [100, 100, 150, 130]);
        
        % Pulse modulator
        add_block('simulink/Discontinuities/Relay', [modelName, '/Pulse Modulator'], ...
                 'Position', [200, 100, 250, 130]);
        
        % Channel with delay
        add_block('simulink/Continuous/Transport Delay', [modelName, '/Channel'], ...
                 'Position', [300, 100, 350, 130], ...
                 'DelayTime', '1e-6');
        
        % Target RCS
        add_block('simulink/Math Operations/Gain', [modelName, '/Target RCS'], ...
                 'Position', [400, 100, 450, 130], ...
                 'Gain', '0.1');
        
        % Receiver
        add_block('simulink/Sinks/Scope', [modelName, '/Receiver'], ...
                 'Position', [500, 100, 550, 130]);
        
        % Connect blocks
        add_line(modelName, 'Signal Generator/1', 'Pulse Modulator/1', 'autorouting', 'on');
        add_line(modelName, 'Pulse Modulator/1', 'Channel/1', 'autorouting', 'on');
        add_line(modelName, 'Channel/1', 'Target RCS/1', 'autorouting', 'on');
        add_line(modelName, 'Target RCS/1', 'Receiver/1', 'autorouting', 'on');
        
        % Configure simulation parameters
        set_param(modelName, 'StopTime', '1e-3');
        
        % Start simulation
        set_param(modelName, 'SimulationCommand', 'start');
        
        msgbox(['Created and started new radar simulation model: ', modelName], 'Model Created');
    catch ME
        errordlg(['Error creating model: ', ME.message], 'Simulink Error');
    end
end

function stopSimulinkModel()
    % Stop the current Simulink model
    try
        % Get current model
        current_model = bdroot;
        
        if ~isempty(current_model)
            % Stop simulation
            set_param(current_model, 'SimulationCommand', 'stop');
            msgbox(['Stopped simulation of model: ', current_model], 'Simulation Stopped');
        else
            warndlg('No Simulink model is currently running.', 'No Model Running');
        end
    catch ME
        errordlg(['Error stopping model: ', ME.message], 'Simulink Error');
    end
end

function configureSimulinkModel()
    % Define the full path to the Simulink model
    modelPath = 'C:\Users\Jhansi Dubasi\OneDrive\Desktop\internship_SDR\SDR_SYSTEM.slx';
    
    % Add the model’s directory to the MATLAB path (if not already added)
    addpath(fileparts(modelPath));

    try
        % Load and open the Simulink model
        load_system(modelPath);
        open_system(modelPath);
        
        % Open configuration parameters
        open_system([bdroot '/Configuration']);
        
    catch ME
        errordlg(['Error configuring model: ' ME.message], 'Simulink Error');
    end
end

function updateRadarParams(paramRadarConfigs, freqSelector, ax1, ax2, ax3, ax4, resultsBox)
    % Create a dialog to update radar parameters
    freqIdx = get(freqSelector, 'Value');
    currentParams = paramRadarConfigs(freqIdx);
    
    % Create dialog figure
    paramFig = figure('Name', 'Update Radar Parameters', ...
                     'NumberTitle', 'off', ...
                     'Position', [300, 200, 500, 400], ...
                     'MenuBar', 'none', ...
                     'ToolBar', 'none');
    
    % Add title
    uicontrol('Parent', paramFig, ...
              'Style', 'text', ...
              'String', sprintf('Radar Parameters (%.2f GHz)', currentParams.fc/1e9), ...
              'Position', [20, 350, 460, 30], ...
              'FontSize', 14, ...
              'FontWeight', 'bold');
    
    % Create parameter fields
    paramNames = {'Peak Power (W)', 'Transmitter Gain (dB)', 'Pulse Length (μs)', ...
                 'PRF (kHz)', 'Range Resolution (m)', 'Maximum Range (m)'};
    
    paramValues = {currentParams.peak_power, currentParams.tx_gain, ...
                  currentParams.pulse_length*1e6, currentParams.prf/1e3, ...
                  currentParams.rng_res, currentParams.max_range};
    
    paramFields = cell(length(paramNames), 1);
    
    for i = 1:length(paramNames)
        % Label
        uicontrol('Parent', paramFig, ...
                 'Style', 'text', ...
                 'String', paramNames{i}, ...
                 'Position', [20, 350-i*40, 200, 25], ...
                 'HorizontalAlignment', 'left', ...
                 'FontSize', 11);
        
        % Edit field
        paramFields{i} = uicontrol('Parent', paramFig, ...
                                  'Style', 'edit', ...
                                  'String', num2str(paramValues{i}), ...
                                  'Position', [230, 350-i*40, 250, 25], ...
                                  'FontSize', 11);
    end
    
    % Add buttons
    uicontrol('Parent', paramFig, ...
              'Style', 'pushbutton', ...
              'String', 'Update', ...
              'Position', [120, 50, 120, 40], ...
              'FontSize', 12, ...
              'Callback', @(src, event) updateParams(paramFields, paramRadarConfigs, freqIdx, ...
                                                   freqSelector, ax1, ax2, ax3, ax4, ...
                                                   resultsBox, paramFig));
    
    uicontrol('Parent', paramFig, ...
              'Style', 'pushbutton', ...
              'String', 'Cancel', ...
              'Position', [260, 50, 120, 40], ...
              'FontSize', 12, ...
              'Callback', @(src, event) close(paramFig));
end

function updateParams(paramFields, paramRadarConfigs, freqIdx, freqSelector, ax1, ax2, ax3, ax4, resultsBox, paramFig)
    % Update radar parameters from dialog
    try
        % Get values from fields
        peak_power = str2double(get(paramFields{1}, 'String'));
        tx_gain = str2double(get(paramFields{2}, 'String'));
        pulse_length = str2double(get(paramFields{3}, 'String')) * 1e-6; % Convert μs to s
        prf = str2double(get(paramFields{4}, 'String')) * 1e3; % Convert kHz to Hz
        rng_res = str2double(get(paramFields{5}, 'String'));
        max_range = str2double(get(paramFields{6}, 'String'));
        
        % Validate inputs
        if any(isnan([peak_power, tx_gain, pulse_length, prf, rng_res, max_range]))
            errordlg('All parameters must be numeric values.', 'Invalid Input');
            return;
        end
        
        % Update parameters
        paramRadarConfigs(freqIdx).peak_power = peak_power;
        paramRadarConfigs(freqIdx).tx_gain = tx_gain;
        paramRadarConfigs(freqIdx).pulse_length = pulse_length;
        paramRadarConfigs(freqIdx).prf = prf;
        
        % Update dependent parameters
        c = physconst('LightSpeed');
        paramRadarConfigs(freqIdx).pulse_bw = c / (2 * rng_res);
        paramRadarConfigs(freqIdx).fs = 2 * paramRadarConfigs(freqIdx).pulse_bw;
        paramRadarConfigs(freqIdx).rng_res = rng_res;
        paramRadarConfigs(freqIdx).max_range = max_range;
        
        % Recalculate range bins
        fast_time = unigrid(0, 1/paramRadarConfigs(freqIdx).fs, 1/paramRadarConfigs(freqIdx).prf, '[)');
        paramRadarConfigs(freqIdx).range_bins = c * fast_time / 2;
        
        % Recalculate SNR
        for t = 1:size(paramRadarConfigs(freqIdx).targetPos, 2)
            target_range = norm(paramRadarConfigs(freqIdx).targetPos(:, t));
            received_power = paramRadarConfigs(freqIdx).peak_power * (paramRadarConfigs(freqIdx).lambda^2) * ...
                paramRadarConfigs(freqIdx).RCS(t) / ((4 * pi)^3 * target_range^4);
            noise_power = 1e-12; % Simplified noise model
            paramRadarConfigs(freqIdx).SNR(t) = 10 * log10(received_power / noise_power);
        end
        
        % Close dialog
        close(paramFig);
        
        % Update displays
        updateDisplays(freqSelector, [], paramRadarConfigs, ax1, ax2, ax3, ax4, resultsBox);
        
        msgbox('Radar parameters updated successfully.', 'Update Complete');
    catch ME
        errordlg(['Error updating parameters: ', ME.message], 'Update Error');
    end
end

function updateCFARSettings(paramRadarConfigs, freqSelector, ax3)
    % Create a dialog to update CFAR settings
    freqIdx = get(freqSelector, 'Value');
    
    % Create dialog figure
    cfarFig = figure('Name', 'CFAR Settings', ...
                    'NumberTitle', 'off', ...
                    'Position', [300, 300, 400, 250], ...
                    'MenuBar', 'none', ...
                    'ToolBar', 'none');
    
    % Add title
    uicontrol('Parent', cfarFig, ...
              'Style', 'text', ...
              'String', 'CFAR Detection Settings', ...
              'Position', [20, 200, 360, 30], ...
              'FontSize', 14, ...
              'FontWeight', 'bold');
    
    % Create parameter fields
    uicontrol('Parent', cfarFig, ...
             'Style', 'text', ...
             'String', 'Threshold Factor:', ...
             'Position', [20, 160, 150, 25], ...
             'HorizontalAlignment', 'left', ...
             'FontSize', 11);
    
    thresholdField = uicontrol('Parent', cfarFig, ...
                              'Style', 'edit', ...
                              'String', '2.8', ...
                              'Position', [180, 160, 200, 25], ...
                              'FontSize', 11);
    
    uicontrol('Parent', cfarFig, ...
             'Style', 'text', ...
             'String', 'Guard Cells:', ...
             'Position', [20, 120, 150, 25], ...
             'HorizontalAlignment', 'left', ...
             'FontSize', 11);
    
    guardField = uicontrol('Parent', cfarFig, ...
                          'Style', 'edit', ...
                          'String', '5', ...
                          'Position', [180, 120, 200, 25], ...
                          'FontSize', 11);
    
    uicontrol('Parent', cfarFig, ...
             'Style', 'text', ...
             'String', 'Training Cells:', ...
             'Position', [20, 80, 150, 25], ...
             'HorizontalAlignment', 'left', ...
             'FontSize', 11);
    
    trainingField = uicontrol('Parent', cfarFig, ...
                             'Style', 'edit', ...
                             'String', '16', ...
                             'Position', [180, 80, 200, 25], ...
                             'FontSize', 11);
    
    % Add buttons
    uicontrol('Parent', cfarFig, ...
              'Style', 'pushbutton', ...
              'String', 'Apply', ...
              'Position', [80, 20, 100, 40], ...
              'FontSize', 12, ...
              'Callback', @(src, event) applyCFARSettings(thresholdField, guardField, ...
                                                         trainingField, paramRadarConfigs, ...
                                                         freqIdx, ax3, cfarFig));
    
    uicontrol('Parent', cfarFig, ...
              'Style', 'pushbutton', ...
              'String', 'Cancel', ...
              'Position', [220, 20, 100, 40], ...
              'FontSize', 12, ...
              'Callback', @(src, event) close(cfarFig));
end

function applyCFARSettings(thresholdField, guardField, trainingField, paramRadarConfigs, freqIdx, ax3, cfarFig)
    % Apply CFAR settings from dialog
    try
        % Get values from fields
        threshold_factor = str2double(get(thresholdField, 'String'));
        num_guard_cells = str2double(get(guardField, 'String'));
        num_training_cells = str2double(get(trainingField, 'String'));
        
        % Validate inputs
        if any(isnan([threshold_factor, num_guard_cells, num_training_cells]))
            errordlg('All parameters must be numeric values.', 'Invalid Input');
            return;
        end
        
        if num_guard_cells < 1 || num_training_cells < 1
            errordlg('Guard cells and training cells must be positive integers.', 'Invalid Input');
            return;
        end
        
        % Generate received signal
        [received_signal, ~] = simulateReceivedSignal(paramRadarConfigs(freqIdx));
        
        % Apply CFAR with new settings
        [cfar_results, threshold] = applyCFAR(received_signal, num_guard_cells, ...
                                             num_training_cells, threshold_factor);
        
        % Update radar configuration
        paramRadarConfigs(freqIdx).CFARResults = cfar_results;
        paramRadarConfigs(freqIdx).threshold = threshold;
        
        % Close dialog
        close(cfarFig);
        
        % Update detection plot
        plotDetectionResults(paramRadarConfigs(freqIdx), ax3);
        
        msgbox('CFAR settings updated successfully.', 'Update Complete');
    catch ME
        errordlg(['Error updating CFAR settings: ', ME.message], 'Update Error');
    end
end

function resetToDefaults()
    % Reset radar parameters to defaults
    warndlg('This will reset all radar parameters to default values. This feature is not implemented in the demo.', ...
           'Reset to Defaults');
end

function showPerformanceMetrics(paramRadarConfigs)
    % Show performance metrics for all frequencies
    
    % Create figure for metrics
    metricsFig = figure('Name', 'Radar Performance Metrics', ...
                       'NumberTitle', 'off', ...
                       'Position', [200, 200, 800, 600]);
    
    % Create table of metrics
    numFreqs = length(paramRadarConfigs);
    
    % Initialize data arrays
    frequency = zeros(numFreqs, 1);
    detectionRate = zeros(numFreqs, 1);
    falseAlarms = zeros(numFreqs, 1);
    avgSNR = zeros(numFreqs, 1);
    maxRange = zeros(numFreqs, 1);
    
    % Calculate metrics for each frequency
    for i = 1:numFreqs
        % Frequency
        frequency(i) = paramRadarConfigs(i).fc / 1e9; % GHz
        
        % Detection rate (percentage of targets detected)
        numTargets = size(paramRadarConfigs(i).targetPos, 2);
        numDetected = 0;
        
        for t = 1:numTargets
            target_range = norm(paramRadarConfigs(i).targetPos(:, t));
            [~, idx] = min(abs(paramRadarConfigs(i).range_bins - target_range));
            
            % Check if target was detected (within a window)
            window = 5;
            if any(paramRadarConfigs(i).CFARResults(max(1, idx-window):min(length(paramRadarConfigs(i).CFARResults), idx+window)))
                numDetected = numDetected + 1;
            end
        end
        
        detectionRate(i) = 100 * numDetected / numTargets;
        
        % False alarms (detections not near any target)
        detected_bins = find(paramRadarConfigs(i).CFARResults == 1);
        falseAlarmCount = 0;
        
        for d = 1:length(detected_bins)
            isFalse = true;
            
            for t = 1:numTargets
                target_range = norm(paramRadarConfigs(i).targetPos(:, t));
                [~, idx] = min(abs(paramRadarConfigs(i).range_bins - target_range));
                
                if abs(detected_bins(d) - idx) <= window
                    isFalse = false;
                    break;
                end
            end
            
            if isFalse
                falseAlarmCount = falseAlarmCount + 1;
            end
        end
        
        falseAlarms(i) = falseAlarmCount;
        
        % Average SNR
        avgSNR(i) = mean(paramRadarConfigs(i).SNR);
        
        % Maximum detection range
        maxRange(i) = paramRadarConfigs(i).max_range / 1000; % km
    end
    
    % Create table
    metricsTable = uitable('Parent', metricsFig, ...
                          'Data', [frequency, detectionRate, falseAlarms, avgSNR, maxRange], ...
                          'ColumnName', {'Frequency (GHz)', 'Detection Rate (%)', 'False Alarms', ...
                                        'Avg SNR (dB)', 'Max Range (km)'}, ...
                          'RowName', [], ...
                          'Position', [50, 400, 700, 150]);
    
    % Add plots
    subplot(2, 2, 3);
    bar(frequency, detectionRate);
    title('Detection Rate vs Frequency');
    xlabel('Frequency (GHz)');
    ylabel('Detection Rate (%)');
    grid on;
    
    subplot(2, 2, 4);
    bar(frequency, falseAlarms);
    title('False Alarms vs Frequency');
    xlabel('Frequency (GHz)');
    ylabel('Number of False Alarms');
    grid on;
    
    subplot(2, 2, 1);
    plot(frequency, avgSNR, 'o-', 'LineWidth', 2);
    title('Average SNR vs Frequency');
    xlabel('Frequency (GHz)');
    ylabel('SNR (dB)');
    grid on;
    
    subplot(2, 2, 2);
    plot(frequency, 1./frequency, 'o-', 'LineWidth', 2);
    title('Wavelength vs Frequency');
    xlabel('Frequency (GHz)');
    ylabel('Wavelength (relative)');
    grid on;
end

function compareFrequencies(paramRadarConfigs)
    % Create a figure to compare results across frequencies
    
    % Create figure
    compareFig = figure('Name', 'Frequency Comparison', ...
                       'NumberTitle', 'off', ...
                       'Position', [100, 100, 1000, 600]);
    
    % Get frequencies
    frequencies = [paramRadarConfigs.fc] / 1e9; % GHz
    numFreqs = length(frequencies);
    
    % Create a color map for different frequencies
    colors = jet(numFreqs);
    
    % Plot 1: Range profiles
    subplot(2, 2, 1);
    hold on;
    
    for i = 1:numFreqs
        % Generate received signal
        [received_signal, ~] = simulateReceivedSignal(paramRadarConfigs(i));
        
        % Plot range profile
        plot(paramRadarConfigs(i).range_bins/1000, 10*log10(received_signal), ...
            'Color', colors(i,:), 'LineWidth', 1.5);
    end
    
    hold off;
    title('Range Profiles Comparison');
    xlabel('Range (km)');
    ylabel('Power (dB)');
    grid on;
    legend(arrayfun(@(f) sprintf('%.1f GHz', f), frequencies, 'UniformOutput', false), ...
          'Location', 'best');
    
    % Plot 2: Detection results
    subplot(2, 2, 2);
    hold on;
    
    for i = 1:numFreqs
        % Get detection results
        detected_bins = find(paramRadarConfigs(i).CFARResults == 1);
        detected_ranges = paramRadarConfigs(i).range_bins(detected_bins);
        
        % Plot detections
        if ~isempty(detected_ranges)
            stem(detected_ranges/1000, i*ones(size(detected_ranges)), 'Color', colors(i,:), ...
                'LineWidth', 1.5, 'Marker', 'o', 'MarkerSize', 6, 'MarkerFaceColor', colors(i,:));
        end
    end
    
    hold off;
    title('Detection Results Comparison');
    xlabel('Range (km)');
    yticks(1:numFreqs);
    yticklabels(arrayfun(@(f) sprintf('%.1f GHz', f), frequencies, 'UniformOutput', false));
    ylabel('Frequency');
    grid on;
    ylim([0, numFreqs+1]);
    
    % Plot 3: SNR vs Range
    subplot(2, 2, 3);
    hold on;
    
    % Generate range axis
    range_axis = linspace(0, paramRadarConfigs(1).max_range, 1000);
    
    for i = 1:numFreqs
        % Calculate theoretical SNR vs range
        lambda = paramRadarConfigs(i).lambda;
        peak_power = paramRadarConfigs(i).peak_power;
        tx_gain = 10^(paramRadarConfigs(i).tx_gain/10);
        rcs = 10; % Reference RCS
        
        % Radar equation
        snr = peak_power * tx_gain * (lambda^2) * rcs ./ ((4*pi)^3 * range_axis.^4);
        snr_db = 10*log10(snr) - 10*log10(1e-12); % Reference to noise floor
        
        % Plot SNR curve
        plot(range_axis/1000, snr_db, 'Color', colors(i,:), 'LineWidth', 1.5);
    end
    
    hold off;
    title('Theoretical SNR vs Range');
    xlabel('Range (km)');
    ylabel('SNR (dB)');
    grid on;
    legend(arrayfun(@(f) sprintf('%.1f GHz', f), frequencies, 'UniformOutput', false), ...
          'Location', 'best');
    
    % Plot 4: Wavelength and resolution comparison
    subplot(2, 2, 4);
    
    % Extract wavelengths and range resolutions
    wavelengths = [paramRadarConfigs.lambda] * 100; % cm
    resolutions = [paramRadarConfigs.rng_res]; % m
    
    % Create bar chart
    bar([wavelengths; resolutions]');
    title('Wavelength and Range Resolution');
    xlabel('Frequency (GHz)');
    ylabel('Value');
    xticklabels(arrayfun(@(f) sprintf('%.1f', f), frequencies, 'UniformOutput', false));
    legend('Wavelength (cm)', 'Range Resolution (m)');
    grid on;
end

function showUserGuide()
    % Display user guide
    guideFig = figure('Name', 'Software Defined Radar - User Guide', ...
                     'NumberTitle', 'off', ...
                     'Position', [200, 100, 800, 600]);
    
    % Create a panel for the text
    guidePanel = uipanel('Parent', guideFig, ...
                        'Position', [0.05, 0.05, 0.9, 0.9]);
    
    % Add text
    guideText = uicontrol('Parent', guidePanel, ...
                         'Style', 'text', ...
                         'String', getGuideText(), ...
                         'Position', [20, 20, 700, 520], ...
                         'HorizontalAlignment', 'left', ...
                         'FontSize', 11);
end

function guideText = getGuideText()
    % Return the user guide text
    guideText = {
        'SOFTWARE DEFINED RADAR SYSTEM - USER GUIDE', ...
        '', ...
        '1. OVERVIEW', ...
        'This software simulates a multi-frequency radar system with target detection capabilities.', ...
        'It provides visualization of radar signals, range-Doppler processing, and 3D target tracking.', ...
        '', ...
        '2. MAIN FEATURES', ...
        '- Multi-frequency radar simulation (1-20 GHz)', ...
        '- CFAR-based target detection', ...
        '- Range-Doppler processing', ...
        '- 3D target visualization', ...
        '- Performance analysis tools', ...
        '- Simulink integration', ...
        '', ...
        '3. USING THE INTERFACE', ...
        '3.1 Frequency Selection', ...
        '    Use the dropdown menu at the top to switch between radar frequencies.', ...
        '', ...
        '3.2 Display Panels', ...
        '    - Top Left: Radar pulse visualization', ...
        '    - Top Right: Range-Doppler map', ...
        '    - Bottom Left: Target detection results', ...
        '    - Bottom Right: 3D target visualization', ...
        '', ...
        '3.3 Menu Options', ...
        '    - File: Export results, save plots', ...
        '    - View: Toggle visibility of display panels', ...
        '    - Simulink: Run/stop Simulink models', ...
        '    - Radar Settings: Update radar parameters, CFAR settings', ...
        '    - Analysis: Performance metrics, frequency comparison', ...
        '    - Help: User guide, about dialog', ...
        '', ...
        '4. RADAR PARAMETERS', ...
        '    - Carrier Frequency: Operating frequency of the radar', ...
        '    - Pulse Length: Duration of transmitted pulse', ...
        '    - PRF: Pulse Repetition Frequency', ...
        '    - Range Resolution: Minimum distinguishable range between targets', ...
        '    - Maximum Range: Maximum detection range', ...
        '', ...
        '5. CFAR DETECTION', ...
        '    - Threshold Factor: Sensitivity of detection algorithm', ...
        '    - Guard Cells: Cells around test cell excluded from averaging', ...
        '    - Training Cells: Cells used for noise estimation', ...
        '', ...
        '6. CONTACT', ...
        'For support or questions, please contact the development team.'
    };
    
    % Join text with newlines
    guideText = strjoin(guideText, '\n');
end

function showAboutDialog()
    % Display about dialog
    msgbox({
        'Software Defined Radar System', ...
        'Version 1.0', ...
        '', ...
        'A comprehensive radar simulation and visualization tool', ...
        'for educational and research purposes.', ...
        '', ...
        '© 2023 Radar Systems Laboratory'
    }, 'About', 'help');
end

function t = unigrid(t0, dt, tf, interval_type)
    % Create a uniform time grid
    % t0: start time
    % dt: time step
    % tf: end time
    % interval_type: '[)' for half-open interval, '[]' for closed interval
    
    if nargin < 4
        interval_type = '[)';
    end
    
    switch interval_type
        case '[)'
            % Half-open interval [t0, tf)
            t = t0:dt:tf-dt;
        case '[]'
            % Closed interval [t0, tf]
            t = t0:dt:tf;
        otherwise
            error('Invalid interval type. Use ''[)'' or ''[]''.');
    end
end
