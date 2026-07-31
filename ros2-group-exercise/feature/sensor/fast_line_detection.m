%% FAST LINE DETECTION PIPELINE
% Optimised for simple real-time line detection
% Uses grayscale, ROI cropping, downscaling, and fast centroid calculation

clear; clc; close all;

%% ============================================================
%  USER SETTINGS
% =============================================================

VIDEO_FILE = 'Video.mp4';

% Processing settings
SCALE       = 0.25;     % Resize ROI to 25% of original size
ROI_TOP     = 0.50;     % Use only bottom 50% of frame
THRESHOLD   = 100;      % Brightness threshold for white tape
MIN_PIXELS  = 30;       % Minimum white pixels required to confirm detection

% Display settings
SHOW_VIDEO  = true;     % Set false for maximum speed
SHOW_EVERY  = 5;        % Display every 5th frame only

% Smoothing
SMOOTH_WINDOW = 15;     % Moving average window for error signal


%% ============================================================
%  LOAD VIDEO
% =============================================================

if ~isfile(VIDEO_FILE)
    error('Video file not found. Make sure "%s" is in the same folder as this script.', VIDEO_FILE);
end

v = VideoReader(VIDEO_FILE);

fps = v.FrameRate;
duration = v.Duration;
estimated_frames = floor(duration * fps);

fprintf('Video loaded successfully.\n');
fprintf('Frame rate: %.2f FPS\n', fps);
fprintf('Duration: %.2f seconds\n', duration);
fprintf('Estimated number of frames: %d\n\n', estimated_frames);


%% ============================================================
%  PREALLOCATE ARRAYS
% =============================================================

err_signal = nan(1, estimated_frames);
cx_signal = nan(1, estimated_frames);
detected_signal = false(1, estimated_frames);
time_signal = nan(1, estimated_frames);

processing_times = nan(1, estimated_frames);


%% ============================================================
%  SETUP DISPLAY FIGURE
% =============================================================

if SHOW_VIDEO
    figure('Name', 'Fast Line Detection', 'NumberTitle', 'off');
end


%% ============================================================
%  PROCESS VIDEO FRAME BY FRAME
% =============================================================

frame_id = 0;

fprintf('Processing video...\n');

total_timer = tic;

while hasFrame(v)

    frame_id = frame_id + 1;

    frame = readFrame(v);

    frame_timer = tic;

    [err, detected, cx, mask, roi_small] = detect_line_fast( ...
        frame, SCALE, ROI_TOP, THRESHOLD, MIN_PIXELS);

    processing_times(frame_id) = toc(frame_timer);

    err_signal(frame_id) = err;
    cx_signal(frame_id) = cx;
    detected_signal(frame_id) = detected;
    time_signal(frame_id) = (frame_id - 1) / fps;

    % Display only every few frames to avoid slowing down processing
    if SHOW_VIDEO && mod(frame_id, SHOW_EVERY) == 0

        subplot(1, 2, 1);
        imshow(roi_small);
        title(sprintf('ROI Frame %d', frame_id));

        subplot(1, 2, 2);
        imshow(mask);
        if detected
            title(sprintf('Mask | Error = %.3f', err));
        else
            title('Mask | Line not detected');
        end

        drawnow limitrate;
    end

    % Prevent array overflow if metadata estimate is slightly wrong
    if frame_id >= estimated_frames
        break;
    end
end

total_time = toc(total_timer);


%% ============================================================
%  TRIM ARRAYS TO ACTUAL NUMBER OF FRAMES
% =============================================================

err_signal = err_signal(1:frame_id);
cx_signal = cx_signal(1:frame_id);
detected_signal = detected_signal(1:frame_id);
time_signal = time_signal(1:frame_id);
processing_times = processing_times(1:frame_id);


%% ============================================================
%  PERFORMANCE SUMMARY
% =============================================================

avg_processing_time = mean(processing_times, 'omitnan');
max_processing_time = max(processing_times);
min_processing_time = min(processing_times);

estimated_processing_fps = 1 / avg_processing_time;
detection_rate = 100 * mean(detected_signal);

fprintf('\n============================================\n');
fprintf('FAST LINE DETECTION RESULTS\n');
fprintf('============================================\n');
fprintf('Frames processed: %d\n', frame_id);
fprintf('Total processing time: %.3f seconds\n', total_time);
fprintf('Average processing time: %.3f ms/frame\n', avg_processing_time * 1000);
fprintf('Minimum processing time: %.3f ms/frame\n', min_processing_time * 1000);
fprintf('Maximum processing time: %.3f ms/frame\n', max_processing_time * 1000);
fprintf('Estimated processing speed: %.1f FPS\n', estimated_processing_fps);
fprintf('Detection rate: %.1f %%\n', detection_rate);
fprintf('============================================\n\n');


%% ============================================================
%  SMOOTH ERROR SIGNAL
% =============================================================

err_smooth = movmean(err_signal, SMOOTH_WINDOW, 'omitnan');


%% ============================================================
%  PLOT ERROR SIGNAL OVER TIME
% =============================================================

figure('Name', 'Line Position Error Metric', 'NumberTitle', 'off');

subplot(3, 1, 1);
plot(time_signal, err_signal, 'b-', 'LineWidth', 1);
hold on;
yline(0, 'k--', 'Centre');
yline(0.3, 'r:', 'Right limit');
yline(-0.3, 'r:', 'Left limit');
grid on;
xlabel('Time (s)');
ylabel('Normalised error');
title('Raw Line Position Error');
ylim([-1 1]);

subplot(3, 1, 2);
plot(time_signal, err_signal, 'b-', 'LineWidth', 0.8);
hold on;
plot(time_signal, err_smooth, 'r-', 'LineWidth', 2);
yline(0, 'k--');
grid on;
xlabel('Time (s)');
ylabel('Normalised error');
title('Raw vs Smoothed Error Signal');
legend('Raw error', 'Smoothed error', 'Centre');
ylim([-1 1]);

subplot(3, 1, 3);
histogram(err_signal, 30);
hold on;
xline(mean(err_signal, 'omitnan'), 'r-', 'LineWidth', 2);
xline(0, 'k--', 'LineWidth', 1.5);
grid on;
xlabel('Normalised error');
ylabel('Frame count');
title(sprintf('Error Distribution | Mean = %.3f | Std = %.3f', ...
    mean(err_signal, 'omitnan'), std(err_signal, 'omitnan')));


%% ============================================================
%  PLOT PROCESSING TIME
% =============================================================

figure('Name', 'Processing Time', 'NumberTitle', 'off');

plot(time_signal, processing_times * 1000, 'b-', 'LineWidth', 1);
hold on;
yline(avg_processing_time * 1000, 'r--', 'Average');
grid on;
xlabel('Time (s)');
ylabel('Processing time (ms/frame)');
title(sprintf('Processing Time per Frame | Average = %.3f ms/frame', ...
    avg_processing_time * 1000));


%% ============================================================
%  PLOT DETECTION STATUS
% =============================================================

figure('Name', 'Detection Status', 'NumberTitle', 'off');

stairs(time_signal, detected_signal, 'LineWidth', 1.5);
grid on;
xlabel('Time (s)');
ylabel('Detected');
title(sprintf('Line Detection Status | Detection rate = %.1f %%', detection_rate));
ylim([-0.1 1.1]);
yticks([0 1]);
yticklabels({'Not detected', 'Detected'});


%% ============================================================
%  SAVE RESULTS
% =============================================================

results.err_signal = err_signal;
results.err_smooth = err_smooth;
results.cx_signal = cx_signal;
results.detected_signal = detected_signal;
results.time_signal = time_signal;
results.processing_times = processing_times;
results.avg_processing_time = avg_processing_time;
results.estimated_processing_fps = estimated_processing_fps;
results.detection_rate = detection_rate;

save('fast_line_detection_results.mat', 'results');

fprintf('Results saved to fast_line_detection_results.mat\n');


%% ============================================================
%  LOCAL FUNCTION
% =============================================================

function [err, detected, cx, mask, roi_small] = detect_line_fast(frame, SCALE, ROI_TOP, THRESHOLD, MIN_PIXELS)
%DETECT_LINE_FAST Fast white tape detection using grayscale thresholding.
%
% Outputs:
%   err       Normalised line error
%             -1 = far left
%              0 = centre
%             +1 = far right
%
%   detected  True if line is detected
%   cx        Detected line centre x-position
%   mask      Binary image of detected line
%   roi_small Resized region of interest

    % Default outputs
    err = 0;
    detected = false;
    cx = NaN;

    %% 1. Crop region of interest first
    [H, ~, ~] = size(frame);

    roi_start = round(H * ROI_TOP);

    if roi_start < 1
        roi_start = 1;
    elseif roi_start > H
        roi_start = round(H / 2);
    end

    roi = frame(roi_start:end, :, :);

    %% 2. Resize ROI
    roi_small = imresize(roi, SCALE);

    %% 3. Convert to grayscale
    gray = rgb2gray(roi_small);

    %% 4. Threshold bright pixels
    mask = gray > THRESHOLD;

    %% 5. Count detected pixels
    pixel_count = sum(mask(:));

    if pixel_count < MIN_PIXELS
        err = 0;
        detected = false;
        cx = NaN;
        return;
    end

    %% 6. Fast centroid calculation using column sums
    [~, w] = size(mask);

    col_sum = sum(mask, 1);
    x_coords = 1:w;

    cx = sum(x_coords .* col_sum) / pixel_count;

    %% 7. Normalised error
    err = (cx - w/2) / (w/2);

    % Limit error between -1 and +1
    err = max(min(err, 1), -1);

    detected = true;
end