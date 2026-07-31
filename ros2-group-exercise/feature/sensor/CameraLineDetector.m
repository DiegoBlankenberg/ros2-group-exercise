%% Camera-based Line Detection for Raspberry Pi Camera Module V2
% Detects masking tape lines on dark floors for robot guidance
% Prototyping in MATLAB before porting to ROS2 on Raspberry Pi

classdef CameraLineDetector
    % Image processing pipeline for detecting masking tape lines
    % Segments tape from background and provides line coordinates
    
    properties
        cam                 % Camera object
        resolution          % [width, height]
        
        % Image processing parameters (tunable)
        blur_kernel         % Gaussian blur kernel size
        canny_low          % Canny edge detection low threshold
        canny_high         % Canny edge detection high threshold
        morph_kernel       % Morphological operations kernel
        hough_threshold    % Hough transform threshold
        tape_threshold     % Binary threshold for tape segmentation
    end
    
    methods
        function obj = CameraLineDetector(varargin)
            % Initialize camera and detection parameters
            % Usage: detector = CameraLineDetector()
            %        detector = CameraLineDetector('resolution', [640 480])
            
            p = inputParser;
            addParameter(p, 'resolution', [640 480], @(x) numel(x) == 2);
            addParameter(p, 'cameraIndex', 1, @isscalar);
            parse(p, varargin{:});
            
            obj.resolution = p.Results.resolution;
            
            % Initialize detection parameters
            obj.blur_kernel = [5 5];
            obj.canny_low = 50;
            obj.canny_high = 150;
            obj.morph_kernel = strel('rectangle', [5 5]);
            obj.hough_threshold = 0.1;  % Normalized threshold for HoughLines
            obj.tape_threshold = 100;   % Binary threshold value
            
            % Initialize camera
            obj = initializeCamera(obj);
        end
        
        function obj = initializeCamera(obj)
            % Initialize the camera connection
            try
                obj.cam = webcam(1);  % Use first webcam
                preview(obj.cam);
                disp(['Camera initialized: ' num2str(obj.resolution(1)) 'x' num2str(obj.resolution(2))]);
            catch ME
                warning(['Failed to initialize camera: ' ME.message]);
                disp('Proceeding in offline mode (for testing with image files)');
            end
        end
        
        function frame = captureFrame(obj)
            % Capture a frame from the camera
            if isempty(obj.cam)
                frame = [];
                return;
            end
            
            try
                frame = snapshot(obj.cam);
            catch ME
                warning(['Failed to capture frame: ' ME.message]);
                frame = [];
            end
        end
        
        function results = detectLine(obj, frame)
            % Detect masking tape line in the image
            %
            % Pipeline:
            % 1. Convert to grayscale
            % 2. Apply Gaussian blur
            % 3. Threshold to isolate tape
            % 4. Morphological operations
            % 5. Edge detection (Canny)
            % 6. Hough line detection
            %
            % Output: results structure with detected lines and properties
            
            if isempty(frame)
                results = [];
                return;
            end
            
            % Convert to grayscale
            if size(frame, 3) == 3
                gray = rgb2gray(frame);
            else
                gray = frame;
            end
            
            % Apply Gaussian blur to reduce noise
            blurred = imgaussfilt(gray, 1.5);
            
            % Threshold to isolate tape
            % Masking tape is typically lighter than dark floor
            binary = blurred > obj.tape_threshold;
            
            % Morphological operations to clean up binary image
            binary = imclose(binary, obj.morph_kernel);
            binary = imopen(binary, obj.morph_kernel);
            
            % Edge detection (Canny equivalent using edge function)
            edges = edge(binary, 'canny', [obj.canny_low/255, obj.canny_high/255]);
            
            % Hough line detection
            [H, T, R] = hough(edges);
            P = houghpeaks(H, 20, 'threshold', ceil(0.3*max(H(:))));
            lines = houghlines(edges, T, R, P, 'FillGap', 10, 'MinLength', 30);
            
            % Calculate centroid of detected tape
            centroid = calculateCentroid(obj, binary);
            
            % Store results
            results.lines = lines;
            results.binary = binary;
            results.edges = edges;
            results.centroid = centroid;
            results.hough_accumulator = H;
            results.hough_theta = T;
            results.hough_rho = R;
        end
        
        function centroid = calculateCentroid(obj, binary_image)
            % Calculate the centroid of the detected tape region
            
            % Label connected components
            [L, num] = bwlabel(binary_image);
            
            if num == 0
                centroid = [];
                return;
            end
            
            % Find the largest component
            stats = regionprops(L, 'Centroid');
            
            if isempty(stats)
                centroid = [];
            else
                % Get centroid of largest region
                areas = regionprops(L, 'Area');
                [~, idx] = max([areas.Area]);
                centroid = stats(idx).Centroid;  % Returns [x, y]
            end
        end
        
        function annotated = visualizeDetection(obj, frame, detection_results)
            % Visualize detection results on the frame
            
            if isempty(detection_results) || isempty(frame)
                annotated = frame;
                return;
            end
            
            annotated = frame;
            
            % Draw detected lines
            if ~isempty(detection_results.lines)
                for k = 1:length(detection_results.lines)
                    line = detection_results.lines(k);
                    xy = [line.point1; line.point2];
                    annotated = insertShape(annotated, 'line', ...
                        [xy(1,1) xy(1,2) xy(2,1) xy(2,2)], ...
                        'Color', 'green', 'LineWidth', 2);
                end
            end
            
            % Draw centroid
            if ~isempty(detection_results.centroid)
                cx = detection_results.centroid(1);
                cy = detection_results.centroid(2);
                
                annotated = insertShape(annotated, 'circle', [cx cy 5], ...
                    'Color', 'red', 'LineWidth', 2);
                
                label = sprintf('Line at (%.0f, %.0f)', cx, cy);
                annotated = insertText(annotated, [10 30], label, ...
                    'FontSize', 16, 'TextColor', 'red');
            end
        end
        
        function runLiveDetection(obj, display_flag)
            % Run live line detection from camera feed
            % Usage: runLiveDetection(detector, true)
            
            if nargin < 2
                display_flag = true;
            end
            
            if isempty(obj.cam)
                error('Camera not initialized');
            end
            
            disp('Starting live line detection...');
            disp('Press Ctrl+C to stop');
            
            try
                while true
                    frame = captureFrame(obj);
                    
                    if isempty(frame)
                        break;
                    end
                    
                    detection = detectLine(obj, frame);
                    
                    if display_flag && ~isempty(detection)
                        annotated = visualizeDetection(obj, frame, detection);
                        imshow(annotated);
                        title('Line Detection - Press Ctrl+C to stop');
                        drawnow;
                    end
                end
            catch
                disp('Detection stopped');
            end
        end
        
        function release(obj)
            % Release camera resources
            if ~isempty(obj.cam)
                clear obj.cam;
            end
            close all;
            disp('Camera released');
        end
    end
end

%% Main script for testing
clear all; close all; clc;

% Create detector object
detector = CameraLineDetector('resolution', [640 480]);

% Option 1: Run live detection from camera
% detector.runLiveDetection(true);

% Option 2: Process a single image (for testing)
% Load test image
try
    test_image = imread('test_floor_image.jpg');
    
    % Detect line
    results = detector.detectLine(test_image);
    
    % Visualize
    figure('Position', [100 100 1200 400]);
    
    % Original image with annotations
    subplot(1, 3, 1);
    annotated = detector.visualizeDetection(test_image, results);
    imshow(annotated);
    title('Detected Line');
    
    % Binary mask
    subplot(1, 3, 2);
    imshow(results.binary);
    title('Binary Tape Mask');
    
    % Edge detection
    subplot(1, 3, 3);
    imshow(results.edges);
    title('Edge Map');
    
    % Print results
    if ~isempty(results.centroid)
        fprintf('Line centroid: (%.1f, %.1f)\n', results.centroid(1), results.centroid(2));
    else
        fprintf('No line detected\n');
    end
    
    fprintf('Number of detected line segments: %d\n', length(results.lines));
    
catch ME
    disp('No test image found. Use detector.runLiveDetection(true) for camera mode.');
    disp(['Error: ' ME.message]);
end

% Clean up
% detector.release();
