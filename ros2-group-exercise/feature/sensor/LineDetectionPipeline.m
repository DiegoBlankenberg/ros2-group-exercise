%% Line Detection Pipeline for Masking Tape on Dark Floor
% MATLAB prototyping for Raspberry Pi deployment
% Experiment with RGB and HSV color spaces
% Extract line position estimate (error signal)
% Video named:Video
%29.48 FPS;10s

classdef LineDetectionPipeline
    properties
        video_path              % Path to video file
        fps                     % Frames per second
        frame_count             % Total frames
        
        % Detection parameters (tunable)
        hsv_h_range            % Hue range [min, max]
        hsv_s_range            % Saturation range [min, max]
        hsv_v_range            % Value range [min, max]
        rgb_threshold          % RGB threshold
        morph_radius           % Morphological operation radius
        
        % Performance tracking
        detection_times        % Time per frame
        line_positions         % Line center x-coordinate per frame
        line_errors           % Error signal (deviation from center)
    end
    
    methods
        function obj = LineDetectionPipeline(video_file)
            % Initialize pipeline
            obj.video_path = video_file;
            
            % Default detection parameters
            % HSV range for masking tape (typically lighter than dark floor)
            obj.hsv_h_range = [0, 180];      % Full hue range
            obj.hsv_s_range = [0, 150];      % Low saturation (grayish)
            obj.hsv_v_range = [100, 255];    % Bright values
            
            obj.rgb_threshold = 100;         % RGB threshold
            obj.morph_radius = 3;
            
            % Load video info
            vid = VideoReader(video_file);
            obj.fps = vid.FrameRate;
            obj.frame_count = floor(vid.Duration * obj.fps);
            
            % Initialize storage
            obj.detection_times = [];
            obj.line_positions = [];
            obj.line_errors = [];
            
            fprintf('✓ Pipeline initialized\n');
            fprintf('  Video: %s\n', video_file);
            fprintf('  FPS: %.1f | Frames: %d\n', obj.fps, obj.frame_count);
        end
        
        function mask = detect_hsv(obj, frame)
            % HSV-based line detection
            % Convert to HSV
            hsv = rgb2hsv(frame);
            
            % Extract HSV channels
            h = hsv(:,:,1) * 180;  % Hue: 0-180
            s = hsv(:,:,2) * 255;  % Saturation: 0-255
            v = hsv(:,:,3) * 255;  % Value: 0-255
            
            % Create mask for light-colored tape on dark floor
            % Tape typically has low saturation (grayish) and high value (bright)
            h_mask = (h >= obj.hsv_h_range(1)) & (h <= obj.hsv_h_range(2));
            s_mask = (s >= obj.hsv_s_range(1)) & (s <= obj.hsv_s_range(2));
            v_mask = (v >= obj.hsv_v_range(1)) & (v <= obj.hsv_v_range(2));
            
            mask = h_mask & s_mask & v_mask;
            
            % Morphological cleanup
            se = strel('disk', obj.morph_radius);
            mask = imclose(mask, se);
            mask = imopen(mask, se);
        end
        
        function mask = detect_rgb(obj, frame)
            % RGB-based line detection (lightweight)
            % Convert to grayscale
            gray = rgb2gray(frame);
            
            % Simple threshold - tape is typically brighter than floor
            mask = gray > obj.rgb_threshold;
            
            % Morphological cleanup
            se = strel('disk', obj.morph_radius);
            mask = imclose(mask, se);
            mask = imopen(mask, se);
        end
        
        function mask = detect_otsu(obj, frame)
            % Otsu thresholding (automatic, lightweight)
            gray = rgb2gray(frame);
            threshold = graythresh(gray);
            mask = gray > (threshold * 255);
            
            % Morphological cleanup
            se = strel('disk', obj.morph_radius);
            mask = imclose(mask, se);
            mask = imopen(mask, se);
        end
        
        function [line_pos, line_error] = extract_line_position(obj, mask)
            % Extract line position from binary mask
            % Returns: line center x-coordinate and error signal
            
            [rows, cols] = size(mask);
            center_x = cols / 2;
            
            % Find pixels belonging to tape
            [~, x_coords] = find(mask);
            
            if isempty(x_coords)
                % No line detected
                line_pos = center_x;
                line_error = 0;
            else
                % Calculate center of mass
                line_pos = mean(x_coords);
                
                % Error signal: deviation from image center (normalized)
                line_error = (line_pos - center_x) / center_x;
            end
        end
        
        function results = process_video(obj, method)
            % Process entire video with specified method
            % method: 'hsv', 'rgb', 'otsu'
            
            if nargin < 2
                method = 'hsv';
            end
            
            fprintf('\n=== Processing Video (%s method) ===\n', upper(method));
            
            vid = VideoReader(obj.video_path);
            frame_idx = 0;
            
            % Preallocate storage
            line_positions = zeros(obj.frame_count, 1);
            line_errors = zeros(obj.frame_count, 1);
            detection_times = zeros(obj.frame_count, 1);
            
            while hasFrame(vid)
                frame_idx = frame_idx + 1;
                frame = readFrame(vid);
                
                % Time the detection
                tic;
                
                % Choose detection method
                switch method
                    case 'hsv'
                        mask = obj.detect_hsv(frame);
                    case 'rgb'
                        mask = obj.detect_rgb(frame);
                    case 'otsu'
                        mask = obj.detect_otsu(frame);
                    otherwise
                        error('Unknown method: %s', method);
                end
                
                % Extract line position
                [line_pos, line_error] = obj.extract_line_position(mask);
                
                elapsed = toc;
                
                % Store results
                line_positions(frame_idx) = line_pos;
                line_errors(frame_idx) = line_error;
                detection_times(frame_idx) = elapsed;
                
                % Progress indicator
                if mod(frame_idx, 30) == 0 || frame_idx == obj.frame_count
                    avg_time = mean(detection_times(1:frame_idx));
                    fprintf('  Frame %d/%d | Avg time: %.4fs (%.1f FPS)\n', ...
                        frame_idx, obj.frame_count, avg_time, 1/avg_time);
                end
            end
            
            % Trim storage to actual frame count
            line_positions = line_positions(1:frame_idx);
            line_errors = line_errors(1:frame_idx);
            detection_times = detection_times(1:frame_idx);
            
            % Store in object
            obj.line_positions = line_positions;
            obj.line_errors = line_errors;
            obj.detection_times = detection_times;
            
            % Prepare results
            results.line_positions = line_positions;
            results.line_errors = line_errors;
            results.detection_times = detection_times;
            results.avg_time = mean(detection_times);
            results.fps = 1 / mean(detection_times);
            results.frames_processed = frame_idx;
            
            % Print summary
            fprintf('\n✓ Processing complete\n');
            fprintf('  Frames processed: %d\n', frame_idx);
            fprintf('  Avg detection time: %.4f ms\n', results.avg_time * 1000);
            fprintf('  Achievable FPS: %.1f\n', results.fps);
        end
        
        function visualize_results(obj)
            % Visualize detection results and line position over time
            
            if isempty(obj.line_positions)
                warning('No results to visualize. Run process_video first.');
                return;
            end
            
            frame_nums = 1:length(obj.line_positions);
            
            % Create figure
            fig = figure('Name', 'Line Detection Results', 'NumberTitle', 'off');
            fig.Position = [100, 100, 1200, 600];
            
            % Subplot 1: Line position over time
            subplot(1, 3, 1);
            plot(frame_nums, obj.line_positions, 'b-', 'LineWidth', 1.5);
            xlabel('Frame Number');
            ylabel('X Position (pixels)');
            title('Line Position Over Time');
            grid on;
            
            % Subplot 2: Error signal over time
            subplot(1, 3, 2);
            plot(frame_nums, obj.line_errors, 'r-', 'LineWidth', 1.5);
            xlabel('Frame Number');
            ylabel('Error Signal (normalized)');
            title('Line Error (deviation from center)');
            grid on;
            axline(gca, [0 0], [1 0], 'Color', 'k', 'LineStyle', '--');
            
            % Subplot 3: Detection time per frame
            subplot(1, 3, 3);
            plot(frame_nums, obj.detection_times * 1000, 'g-', 'LineWidth', 1.5);
            xlabel('Frame Number');
            ylabel('Time (ms)');
            title('Detection Time Per Frame');
            grid on;
            
            % Overall statistics
            fprintf('\n=== Statistics ===\n');
            fprintf('Line Position:\n');
            fprintf('  Mean: %.1f pixels\n', mean(obj.line_positions));
            fprintf('  Std Dev: %.1f pixels\n', std(obj.line_positions));
            fprintf('  Range: [%.1f, %.1f]\n', min(obj.line_positions), max(obj.line_positions));
            
            fprintf('\nError Signal:\n');
            fprintf('  Mean: %.4f\n', mean(obj.line_errors));
            fprintf('  Std Dev: %.4f\n', std(obj.line_errors));
            fprintf('  Range: [%.4f, %.4f]\n', min(obj.line_errors), max(obj.line_errors));
            
            fprintf('\nDetection Performance:\n');
            fprintf('  Min time: %.4f ms\n', min(obj.detection_times) * 1000);
            fprintf('  Max time: %.4f ms\n', max(obj.detection_times) * 1000);
            fprintf('  Avg time: %.4f ms\n', mean(obj.detection_times) * 1000);
            fprintf('  Achievable FPS: %.1f\n', 1 / mean(obj.detection_times));
        end
        
        function compare_methods(obj)
            % Compare different detection methods
            
            fprintf('\n' + "="*50 + "\n");
            fprintf('COMPARING DETECTION METHODS\n');
            fprintf("="*50 + "\n\n");
            
            methods = {'hsv', 'rgb', 'otsu'};
            results_all = cell(length(methods), 1);
            
            for i = 1:length(methods)
                fprintf('Method %d/%d: %s\n', i, length(methods), upper(methods{i}));
                results = obj.process_video(methods{i});
                results_all{i} = results;
                pause(1);  % Brief pause between methods
            end
            
            % Print comparison table
            fprintf('\n=== COMPARISON SUMMARY ===\n');
            fprintf('%-10s | %-12s | %-10s | %-15s\n', 'Method', 'Avg Time (ms)', 'FPS', 'Frames');
            fprintf('%-10s-+-%-12s-+-%-10s-+-%-15s\n', '-----------', '-------------', '----------', '---------------');
            
            for i = 1:length(methods)
                res = results_all{i};
                fprintf('%-10s | %12.4f | %10.1f | %15d\n', ...
                    upper(methods{i}), res.avg_time*1000, res.fps, res.frames_processed);
            end
            
            % Recommend best method
            fps_values = [results_all{:}];
            fps_values = [fps_values.fps];
            [~, best_idx] = max(fps_values);
            
            fprintf('\n✓ Recommended method: %s (%.1f FPS)\n', upper(methods{best_idx}), fps_values(best_idx));
        end
    end
end

%% Main Script
clear all; close all; clc;

% Auto-detect video files in current directory
current_dir = fileparts(mfilename('fullpath'));
video_files = dir(fullfile(current_dir, '*.mp4'));

if isempty(video_files)
    error('No MP4 files found in %s', current_dir);
end

fprintf('Found video files:\n');
for i = 1:length(video_files)
    fprintf('  %d. %s (%.1f MB)\n', i, video_files(i).name, video_files(i).bytes/1e6);
end

% Use most recent video if multiple exist
[~, idx] = max([video_files.datenum]);
video_file = fullfile(current_dir, video_files(idx).name);

fprintf('\n✓ Selected: %s\n', video_files(idx).name);

% Measured performance values
fprintf('\n=== PERFORMANCE SPECS ===\n');
fprintf('Expected FPS: 29.48 FPS\n');
fprintf('Test Duration: 10 seconds\n');
fprintf('Frame Size: 3840x2160 pixels\n');
fprintf('==============================\n\n');

% Create pipeline
pipeline = LineDetectionPipeline(video_file);

% Tuned HSV parameters for masking tape on dark floor
fprintf('\n=== TUNED PARAMETERS ===\n');
pipeline.hsv_h_range = [0, 180];        % Full hue (tape is neutral)
pipeline.hsv_s_range = [0, 100];        % Very low saturation (grayish tape)
pipeline.hsv_v_range = [120, 255];      % Bright values (tape is lighter)
pipeline.rgb_threshold = 120;           % Adjusted for better separation
pipeline.morph_radius = 3;              % Morphological cleanup

fprintf('HSV Hue: [%d, %d]\n', pipeline.hsv_h_range(1), pipeline.hsv_h_range(2));
fprintf('HSV Saturation: [%d, %d]\n', pipeline.hsv_s_range(1), pipeline.hsv_s_range(2));
fprintf('HSV Value: [%d, %d]\n', pipeline.hsv_v_range(1), pipeline.hsv_v_range(2));
fprintf('RGB Threshold: %d\n', pipeline.rgb_threshold);
fprintf('Morph Radius: %d\n', pipeline.morph_radius);

% Option 1: Compare all methods
fprintf('\n\nComparing detection methods...\n');
pipeline.compare_methods();

% Option 2: Process with best method and visualize
fprintf('\n' + "="*50 + "\n");
fprintf('DETAILED ANALYSIS: HSV METHOD\n');
fprintf("="*50 + "\n");

results = pipeline.process_video('hsv');
pipeline.visualize_results();

% Export results
fprintf('\n=== EXPORT RESULTS ===\n');
fprintf('Line positions saved: line_positions.csv\n');
fprintf('Error signals saved: line_errors.csv\n');

% Save to CSV for further analysis
writetable(table(results.line_positions, 'VariableNames', {'LinePosition'}), ...
    'line_positions.csv');
writetable(table(results.line_errors, 'VariableNames', {'ErrorSignal'}), ...
    'line_errors.csv');
