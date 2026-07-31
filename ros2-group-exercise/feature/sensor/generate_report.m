%% Line Detection Pipeline — White tape on black floor
% Run this script from the folder containing your video file.
% All plots saved to ./output/

clear; clc; close all;

% =========================================================
% SETTINGS — edit these if needed
% =========================================================
VIDEO_FILE  = 'Video.mp4';   % your video filename
OUTPUT_DIR  = 'output';
SCALE       = 0.25;          % downscale factor (0.25 = 25% resolution)
THRESHOLD   = 100;           % grayscale threshold for white (0-255)
ROI_TOP     = 0.5;           % ignore top 50% of frame (angled camera)
MIN_AREA    = 30;            % minimum blob area to keep (noise filter)

% Sample frame indices for comparison plots
SAMPLE_IDX = [30, 90, 150, 210, 270];

if ~exist(OUTPUT_DIR, 'dir'); mkdir(OUTPUT_DIR); end

% =========================================================
% 1. LOAD VIDEO
% =========================================================
fprintf('Loading video: %s\n', VIDEO_FILE);
v        = VideoReader(VIDEO_FILE);
fps      = v.FrameRate;
n_frames = floor(v.Duration * fps);
W        = v.Width;
H        = v.Height;
fprintf('  %dx%d  %.2f FPS  %d frames  (%.1f s)\n', W, H, fps, n_frames, v.Duration);

% Clamp sample indices
SAMPLE_IDX = SAMPLE_IDX(SAMPLE_IDX <= n_frames);
N_SAMPLES  = numel(SAMPLE_IDX);

% Read sample frames
sample_frames = cell(1, N_SAMPLES);
for i = 1:N_SAMPLES
    v.CurrentTime = (SAMPLE_IDX(i) - 1) / fps;
    sample_frames{i} = readFrame(v);
end

% =========================================================
% 2. FIGURE 1 — Sample frames
% =========================================================
fig1 = figure('Visible','off','Position',[0 0 1400 300]);
for i = 1:N_SAMPLES
    subplot(1, N_SAMPLES, i);
    imshow(sample_frames{i});
    title(sprintf('Frame %d', SAMPLE_IDX(i)), 'FontSize', 9);
end
sgtitle('Sample Frames from Video');
saveas(fig1, fullfile(OUTPUT_DIR, '1_sample_frames.png'));
close(fig1);
fprintf('Saved: 1_sample_frames.png\n');

% =========================================================
% 3. RGB vs HSV COMPARISON
% =========================================================
fprintf('Running RGB vs HSV comparison...\n');

snr_rgb  = zeros(1, N_SAMPLES);
snr_hsv  = zeros(1, N_SAMPLES);
snr_gray = zeros(1, N_SAMPLES);
cov_rgb  = zeros(1, N_SAMPLES);
cov_hsv  = zeros(1, N_SAMPLES);
cov_gray = zeros(1, N_SAMPLES);
t_rgb    = zeros(1, N_SAMPLES);
t_hsv    = zeros(1, N_SAMPLES);
t_gray   = zeros(1, N_SAMPLES);

for i = 1:N_SAMPLES
    small = imresize(sample_frames{i}, SCALE);
    [h, w, ~] = size(small);
    gray_img = rgb2gray(small);

    % --- Grayscale ---
    t0 = tic;
    mask_gray = gray_img > uint8(THRESHOLD);
    brightness = (double(small(:,:,1)) + double(small(:,:,2)) + double(small(:,:,3))) / 3;
    mask_rgb = brightness > THRESHOLD;
    mask_rgb = bwareaopen(mask_rgb, MIN_AREA);
    mask_rgb = imclose(mask_rgb, strel('disk', 3));
    t_rgb(i) = toc(t0);

    % --- HSV ---
    t0 = tic;
    hsv_img  = rgb2hsv(small);
    mask_hsv = (hsv_img(:,:,2) < 0.25) & (hsv_img(:,:,3) > 0.70);
    mask_hsv = bwareaopen(mask_hsv, MIN_AREA);
    mask_hsv = imclose(mask_hsv, strel('disk', 3));
    t_hsv(i) = toc(t0);

    % --- Metrics ---
    gd = double(gray_img) / 255;

    function_snr = @(mask) compute_snr(gd, mask);

    snr_gray(i) = function_snr(mask_gray);
    snr_rgb(i)  = function_snr(mask_rgb);
    snr_hsv(i)  = function_snr(mask_hsv);
    cov_gray(i) = 100 * sum(mask_gray(:)) / numel(mask_gray);
    cov_rgb(i)  = 100 * sum(mask_rgb(:))  / numel(mask_rgb);
    cov_hsv(i)  = 100 * sum(mask_hsv(:))  / numel(mask_hsv);
end

% Figure 2 — Visual comparison (first sample frame)
s1     = imresize(sample_frames{1}, SCALE);
gray1  = rgb2gray(s1);
hsv1   = rgb2hsv(s1);
br1    = (double(s1(:,:,1))+double(s1(:,:,2))+double(s1(:,:,3)))/3;

mk_gray = imclose(bwareaopen(gray1 > uint8(THRESHOLD), MIN_AREA), strel('disk',3));
mk_rgb  = imclose(bwareaopen(br1 > THRESHOLD,          MIN_AREA), strel('disk',3));
mk_hsv  = imclose(bwareaopen((hsv1(:,:,2)<0.25)&(hsv1(:,:,3)>0.70), MIN_AREA), strel('disk',3));

fig2 = figure('Visible','off','Position',[0 0 1400 420]);
subplot(1,4,1); imshow(s1);            title('Original','FontSize',9);
subplot(1,4,2); imshow(mk_gray);       title('Grayscale mask','FontSize',9);
subplot(1,4,3); imshow(mk_rgb);        title('RGB brightness mask','FontSize',9);
subplot(1,4,4); imshow(mk_hsv);        title('HSV mask (S<0.25, V>0.70)','FontSize',9);
sgtitle(sprintf('Mask Comparison — Frame %d', SAMPLE_IDX(1)));
saveas(fig2, fullfile(OUTPUT_DIR, '2_rgb_vs_hsv_visual.png'));
close(fig2);
fprintf('Saved: 2_rgb_vs_hsv_visual.png\n');

% Figure 3 — Quantitative metrics bar charts
xtlabels = arrayfun(@(x) sprintf('F%d',x), SAMPLE_IDX, 'UniformOutput', false);
colors   = [0.4 0.8 0.4; 0.2 0.5 0.9; 0.9 0.4 0.2];  % gray, rgb, hsv

fig3 = figure('Visible','off','Position',[0 0 1000 800]);

subplot(3,1,1);
b = bar([snr_gray; snr_rgb; snr_hsv]');
for k=1:3; b(k).FaceColor = colors(k,:); end
set(gca,'XTickLabel', xtlabels);
ylabel('SNR'); title('Signal-to-Noise Ratio per Frame');
legend('Grayscale','RGB','HSV','Location','best'); grid on;

subplot(3,1,2);
b2 = bar([cov_gray; cov_rgb; cov_hsv]');
for k=1:3; b2(k).FaceColor = colors(k,:); end
set(gca,'XTickLabel', xtlabels);
ylabel('Coverage (%)'); title('Mask Coverage (% of image pixels)');
legend('Grayscale','RGB','HSV','Location','best'); grid on;

subplot(3,1,3);
b3 = bar([t_gray; t_rgb; t_hsv]' * 1000);
for k=1:3; b3(k).FaceColor = colors(k,:); end
set(gca,'XTickLabel', xtlabels);
ylabel('Time (ms)'); title('Processing Time per Frame');
legend('Grayscale','RGB','HSV','Location','best'); grid on;

sgtitle('Quantitative Comparison: Grayscale vs RGB vs HSV');
saveas(fig3, fullfile(OUTPUT_DIR, '3_quantitative_comparison.png'));
close(fig3);
fprintf('Saved: 3_quantitative_comparison.png\n');

% Print table to console
fprintf('\n--- RGB vs HSV Summary Table ---\n');
fprintf('%-20s %10s %10s %10s\n','Metric','Grayscale','RGB','HSV');
fprintf('%-20s %10.3f %10.3f %10.3f\n','Mean SNR',    mean(snr_gray),mean(snr_rgb),mean(snr_hsv));
fprintf('%-20s %10.3f %10.3f %10.3f\n','Mean Cover%', mean(cov_gray),mean(cov_rgb),mean(cov_hsv));
fprintf('%-20s %10.3f %10.3f %10.3f\n','Mean time(ms)',mean(t_gray)*1e3,mean(t_rgb)*1e3,mean(t_hsv)*1e3);
fprintf('\n');

% =========================================================
% 4. SPEED COMPARISON — full timing over 50 frames
% =========================================================
fprintf('Timing comparison over 50 frames...\n');
N_TIME = min(50, n_frames);
tt_gray = zeros(1,N_TIME);
tt_rgb  = zeros(1,N_TIME);
tt_hsv  = zeros(1,N_TIME);

v.CurrentTime = 0;
for i = 1:N_TIME
    if ~hasFrame(v); break; end
    fr = imresize(readFrame(v), SCALE);

    t0 = tic;
    mg = rgb2gray(fr); mg = imclose(bwareaopen(mg>uint8(THRESHOLD),MIN_AREA),strel('disk',2));
    tt_gray(i) = toc(t0);

    t0 = tic;
    br = (double(fr(:,:,1))+double(fr(:,:,2))+double(fr(:,:,3)))/3;
    mr = imclose(bwareaopen(br>THRESHOLD,MIN_AREA),strel('disk',2));
    tt_rgb(i) = toc(t0);

    t0 = tic;
    hh = rgb2hsv(fr);
    mh = imclose(bwareaopen((hh(:,:,2)<0.25)&(hh(:,:,3)>0.70),MIN_AREA),strel('disk',2));
    tt_hsv(i) = toc(t0);
end

means_ms = [mean(tt_gray) mean(tt_rgb) mean(tt_hsv)] * 1000;
stds_ms  = [std(tt_gray)  std(tt_rgb)  std(tt_hsv)]  * 1000;

fig4 = figure('Visible','off','Position',[0 0 700 450]);
b4 = bar(means_ms, 'FaceColor','flat');
b4.CData = colors;
hold on;
errorbar(1:3, means_ms, stds_ms, 'k.', 'LineWidth', 1.5);
set(gca,'XTickLabel',{'Grayscale','RGB','HSV'});
ylabel('Mean time (ms/frame)');
title(sprintf('Processing Speed at %.0f%% Resolution (N=%d frames)', SCALE*100, N_TIME));
grid on;
for k = 1:3
    text(k, means_ms(k)+stds_ms(k)+0.05, sprintf('%.2f ms\n~%.0f FPS', means_ms(k), 1000/means_ms(k)), ...
        'HorizontalAlignment','center','FontSize',8,'FontWeight','bold');
end
saveas(fig4, fullfile(OUTPUT_DIR, '4_speed_comparison.png'));
close(fig4);
fprintf('Saved: 4_speed_comparison.png\n');
fprintf('  Grayscale: %.2fms (~%.0f FPS) | RGB: %.2fms (~%.0f FPS) | HSV: %.2fms (~%.0f FPS)\n', ...
    means_ms(1),1000/means_ms(1), means_ms(2),1000/means_ms(2), means_ms(3),1000/means_ms(3));

% =========================================================
% 5. FINAL MASK OVERLAY on sample frames
% =========================================================
fig5 = figure('Visible','off','Position',[0 0 1400 500]);
for i = 1:N_SAMPLES
    small = imresize(sample_frames{i}, SCALE);
    gray_s = rgb2gray(small);
    mask_f = gray_s > uint8(THRESHOLD);
    mask_f = bwareaopen(mask_f, MIN_AREA);
    mask_f = imclose(mask_f, strel('disk', 3));

    % Green overlay
    ov = small;
    ch2 = ov(:,:,2);
    ch2(mask_f) = uint8(min(255, double(ch2(mask_f)) + 80));
    ov(:,:,2) = ch2;
    ch1 = ov(:,:,1); ch1(mask_f) = uint8(double(ch1(mask_f)) * 0.3); ov(:,:,1) = ch1;
    ch3 = ov(:,:,3); ch3(mask_f) = uint8(double(ch3(mask_f)) * 0.3); ov(:,:,3) = ch3;

    subplot(2, N_SAMPLES, i);
    imshow(small); title(sprintf('Frame %d', SAMPLE_IDX(i)),'FontSize',8);

    subplot(2, N_SAMPLES, N_SAMPLES+i);
    imshow(ov); title('Detected','FontSize',8);
end
sgtitle('Final Pipeline (Grayscale) — Mask Overlay on Sample Frames');
saveas(fig5, fullfile(OUTPUT_DIR, '5_final_mask_overlay.png'));
close(fig5);
fprintf('Saved: 5_final_mask_overlay.png\n');

% =========================================================
% 6. ERROR METRIC OVER ALL FRAMES
% =========================================================
fprintf('Computing error metric over all %d frames...\n', n_frames);

err_signal  = nan(1, n_frames);
cx_all      = nan(1, n_frames);
cy_all      = nan(1, n_frames);
detected    = false(1, n_frames);
timestamps  = (0:n_frames-1) / fps;

v.CurrentTime = 0;
fi = 0;
while hasFrame(v)
    fi = fi + 1;
    fr = readFrame(v);
    small = imresize(fr, SCALE);
    [fh, fw, ~] = size(small);

    % ROI: bottom half
    roi_row = round(fh * ROI_TOP);
    roi = small(roi_row:end, :, :);

    gray_r = rgb2gray(roi);
    mask_r = gray_r > uint8(THRESHOLD);
    mask_r = bwareaopen(mask_r, MIN_AREA);
    mask_r = imclose(mask_r, strel('disk', 2));

    stats = regionprops(mask_r, 'Area', 'Centroid');
    if ~isempty(stats)
        [~, bi] = max([stats.Area]);
        cx = stats(bi).Centroid(1);
        cy = stats(bi).Centroid(2) + roi_row;
        cx_all(fi)   = cx;
        cy_all(fi)   = cy;
        detected(fi) = true;
        % Normalised error: -1=far left, 0=centre, +1=far right
        err_signal(fi) = (cx - fw/2) / (fw/2);
    end

    if mod(fi, round(n_frames/20)) == 0
        fprintf('  %.0f%%\n', 100*fi/n_frames);
    end
    if fi >= n_frames; break; end
end

t_vec  = timestamps(1:fi);
err_v  = err_signal(1:fi);
det_v  = detected(1:fi);
cx_v   = cx_all(1:fi);
cy_v   = cy_all(1:fi);

% Smooth error
win = round(fps / 2);
err_smooth = movmean(err_v, win, 'omitnan');

% Figure 6a — Error over time
fig6 = figure('Visible','off','Position',[0 0 1100 750]);

subplot(3,1,1);
plot(t_vec, err_v, 'b-', 'LineWidth', 0.8); hold on;
plot(t_vec(~det_v), zeros(1,sum(~det_v)), 'rx', 'MarkerSize',5, 'DisplayName','Not detected');
yline(0,    'k--', 'LineWidth',1.2);
yline( 0.3, 'r:',  'LineWidth',1.0);
yline(-0.3, 'r:',  'LineWidth',1.0);
ylim([-1.1 1.1]);
xlabel('Time (s)'); ylabel('Normalised error');
title('Line Position Error (raw)   −1=far left  |  0=centre  |  +1=far right');
legend('Detected','Not detected','Location','best'); grid on;

subplot(3,1,2);
plot(t_vec, err_v,      'b-', 'LineWidth', 0.7, 'DisplayName','Raw'); hold on;
plot(t_vec, err_smooth, 'r-', 'LineWidth', 2.0, 'DisplayName', sprintf('Smoothed (%d-frame window)',win));
yline(0,'k--','LineWidth',1);
ylim([-1.1 1.1]);
xlabel('Time (s)'); ylabel('Normalised error');
title('Raw vs Smoothed Error Signal');
legend('Location','best'); grid on;

subplot(3,1,3);
histogram(err_v(det_v), 40, 'FaceColor',[0.2 0.5 0.9], 'EdgeColor','none'); hold on;
xline(0,                  'k--', 'LineWidth',1.5);
xline(mean(err_v,'omitnan'),'r-','LineWidth',2);
xlabel('Normalised error'); ylabel('Frame count');
title(sprintf('Error Distribution   mean=%.3f   std=%.3f', ...
    mean(err_v,'omitnan'), std(err_v,'omitnan')));
grid on;

sgtitle('Line Position Error Metric');
saveas(fig6, fullfile(OUTPUT_DIR, '6_error_metric.png'));
close(fig6);
fprintf('Saved: 6_error_metric.png\n');

% Figure 6b — Centroid path
[fh2, fw2, ~] = size(imresize(sample_frames{1}, SCALE));
fig7 = figure('Visible','off','Position',[0 0 700 520]);
scatter(cx_v(det_v), cy_v(det_v), 8, t_vec(det_v), 'filled');
cb = colorbar; cb.Label.String = 'Time (s)';
colormap(jet);
xline(fw2/2, 'k--', 'Centre', 'LineWidth',1.5);
xlim([0 fw2]); ylim([0 fh2]);
set(gca,'YDir','reverse');
xlabel('X pixel (scaled)'); ylabel('Y pixel (scaled)');
title('Line Centroid Path Over Time (colour = time)');
saveas(fig7, fullfile(OUTPUT_DIR, '7_centroid_path.png'));
close(fig7);
fprintf('Saved: 7_centroid_path.png\n');

% =========================================================
% 7. SUMMARY
% =========================================================
fprintf('\n========================================\n');
fprintf('SUMMARY\n');
fprintf('========================================\n');
fprintf('Frames processed     : %d\n', fi);
fprintf('Line detected        : %d / %d  (%.1f%%)\n', sum(det_v), fi, 100*mean(det_v));
fprintf('Mean error           : %.4f\n', mean(err_v,'omitnan'));
fprintf('Std of error         : %.4f\n', std(err_v,'omitnan'));
fprintf('Max left error       : %.4f\n', min(err_v,[],'omitnan'));
fprintf('Max right error      : %.4f\n', max(err_v,[],'omitnan'));
fprintf('----------------------------------------\n');
fprintf('Speed (%.0f%% scale):\n', SCALE*100);
fprintf('  Grayscale : %.2f ms  (~%.0f FPS)\n', means_ms(1), 1000/means_ms(1));
fprintf('  RGB       : %.2f ms  (~%.0f FPS)\n', means_ms(2), 1000/means_ms(2));
fprintf('  HSV       : %.2f ms  (~%.0f FPS)\n', means_ms(3), 1000/means_ms(3));
fprintf('========================================\n');
fprintf('All figures saved to ./%s/\n', OUTPUT_DIR);

% =========================================================
% LOCAL HELPER FUNCTION
% =========================================================
function s = compute_snr(gray_double, mask)
    line_px = gray_double(mask);
    bg_px   = gray_double(~mask);
    if isempty(line_px) || isempty(bg_px) || std(bg_px) < 1e-6
        s = 0;
    else
        s = mean(line_px) / std(bg_px);
    end
end
