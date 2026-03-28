%% plot_comparison.m
% =========================================================
% Run this AFTER both wired and wifi tests are done
% Requires: results_wired.mat and results_wifi.mat
%
% What this produces:
%   1. Printed summary table in the command window
%   2. Figure 1 — Overall latency + stats + packet loss
%   3. Figure 2 — Latency distribution (histogram)
%   4. Figure 3 — Latency over time (every packet)
%   5. Saves all figures as PNG files
%   6. Saves a summary table as a .csv file
% =========================================================

close all;

%% === Load Results ===
if ~isfile("results_wired.mat") || ~isfile("results_wifi.mat")
    error("Missing result files! Make sure both results_wired.mat and results_wifi.mat exist.");
end

w  = load("results_wired.mat");
wf = load("results_wifi.mat");

% Extract data
w_lat    = w.net_latency;
wf_lat   = wf.net_latency;
w_valid  = w.valid_lat;
wf_valid = wf.valid_lat;

%% === Compute Statistics ===
% Wired
w_mean   = mean(w_valid);
w_min    = min(w_valid);
w_max    = max(w_valid);
w_std    = std(w_valid);
w_med    = median(w_valid);
w_p95    = prctile(w_valid, 95);   % 95th percentile
w_loss   = w.loss_pct;
w_sent   = w.net_sent;
w_lost   = w.net_lost;

% WiFi
wf_mean  = mean(wf_valid);
wf_min   = min(wf_valid);
wf_max   = max(wf_valid);
wf_std   = std(wf_valid);
wf_med   = median(wf_valid);
wf_p95   = prctile(wf_valid, 95);
wf_loss  = wf.loss_pct;
wf_sent  = wf.net_sent;
wf_lost  = wf.net_lost;

%% === Print Summary Table ===
fprintf('\n');
fprintf('╔══════════════════════════════════════════════════════╗\n');
fprintf('║         NETWORK PERFORMANCE SUMMARY TABLE            ║\n');
fprintf('╠══════════════════════════════════════════════════════╣\n');
fprintf('║ Metric                  │   Wired   │    WiFi       ║\n');
fprintf('╠══════════════════════════════════════════════════════╣\n');
fprintf('║ Packets Sent            │ %9d │ %9d     ║\n', w_sent,  wf_sent);
fprintf('║ Packets Lost            │ %9d │ %9d     ║\n', w_lost,  wf_lost);
fprintf('║ Packet Loss (%%)         │ %8.2f%% │ %8.2f%%     ║\n', w_loss,  wf_loss);
fprintf('╠══════════════════════════════════════════════════════╣\n');
fprintf('║ Mean Latency (ms)       │ %9.2f │ %9.2f     ║\n', w_mean,  wf_mean);
fprintf('║ Min Latency (ms)        │ %9.2f │ %9.2f     ║\n', w_min,   wf_min);
fprintf('║ Max Latency (ms)        │ %9.2f │ %9.2f     ║\n', w_max,   wf_max);
fprintf('║ Median Latency (ms)     │ %9.2f │ %9.2f     ║\n', w_med,   wf_med);
fprintf('║ Std Deviation (ms)      │ %9.2f │ %9.2f     ║\n', w_std,   wf_std);
fprintf('║ 95th Percentile (ms)    │ %9.2f │ %9.2f     ║\n', w_p95,   wf_p95);
fprintf('╠══════════════════════════════════════════════════════╣\n');

% Difference row
diff_mean = wf_mean - w_mean;
diff_pct  = (diff_mean / w_mean) * 100;
fprintf('║ WiFi vs Wired (mean)    │ +%.2f ms (%.1f%% slower)   ║\n', diff_mean, diff_pct);
fprintf('╚══════════════════════════════════════════════════════╝\n\n');

%% === Save Summary as CSV ===
fid = fopen("network_summary.csv", "w");
fprintf(fid, "Metric,Wired,WiFi\n");
fprintf(fid, "Packets Sent,%d,%d\n",          w_sent,  wf_sent);
fprintf(fid, "Packets Lost,%d,%d\n",          w_lost,  wf_lost);
fprintf(fid, "Packet Loss (%%),%%.2f,%%.2f\n", w_loss,  wf_loss);
fprintf(fid, "Mean Latency (ms),%.2f,%.2f\n", w_mean,  wf_mean);
fprintf(fid, "Min Latency (ms),%.2f,%.2f\n",  w_min,   wf_min);
fprintf(fid, "Max Latency (ms),%.2f,%.2f\n",  w_max,   wf_max);
fprintf(fid, "Median Latency (ms),%.2f,%.2f\n", w_med, wf_med);
fprintf(fid, "Std Deviation (ms),%.2f,%.2f\n", w_std,  wf_std);
fprintf(fid, "95th Percentile (ms),%.2f,%.2f\n", w_p95, wf_p95);
fclose(fid);
fprintf("Summary saved to: network_summary.csv\n\n");

%% =====================================================
%% FIGURE 1 — Overall Stats Comparison
%% =====================================================
f1 = figure("Name", "Overall Comparison", "NumberTitle", "off");
set(f1, "Position", [50 50 1100 750]);

% ── Top: Stats bar chart ──────────────────────────────
subplot(2, 2, [1 2]);
metrics = [w_mean  w_min  w_max  w_std  w_med  w_p95;
           wf_mean wf_min wf_max wf_std wf_med wf_p95];
b = bar(metrics, 0.6);
colors = {'#3399FF','#66CC66','#FF6666','#FFAA33','#AA66CC','#FF99CC'};
for k = 1:6, b(k).FaceColor = colors{k}; end
set(gca, "XTickLabel", {"Wired", "WiFi"});
legend({"Mean","Min","Max","Std Dev","Median","95th pct"}, ...
       "Location", "northeast", "FontSize", 9);
ylabel("Latency (ms)");
title("Latency Statistics — Wired vs WiFi");
grid on;

% Add value labels on bars
for k = 1:6
    for row = 1:2
        xpos = b(k).XEndPoints(row);
        ypos = b(k).YEndPoints(row);
        text(xpos, ypos + 0.3, sprintf("%.1f", metrics(row,k)), ...
             "HorizontalAlignment", "center", "FontSize", 7);
    end
end

% ── Bottom left: Packet loss ──────────────────────────
subplot(2, 2, 3);
bar_data = [w_loss, wf_loss];
bh = bar(bar_data, 0.4);
bh.FaceColor = "flat";
bh.CData = [0 0.45 0.74; 0.85 0.33 0.10];
set(gca, "XTickLabel", {"Wired", "WiFi"});
ylabel("Packet Loss (%)");
title("Packet Loss Comparison");
ylim([0, max(bar_data) * 1.5 + 0.5]);
text(1, w_loss  + 0.05, sprintf("%.2f%%", w_loss),  "HorizontalAlignment", "center", "FontSize", 10);
text(2, wf_loss + 0.05, sprintf("%.2f%%", wf_loss), "HorizontalAlignment", "center", "FontSize", 10);
grid on;

% ── Bottom right: Mean + Std as error bar ────────────
subplot(2, 2, 4);
errorbar([1 2], [w_mean wf_mean], [w_std wf_std], ...
         "o", "MarkerSize", 10, "LineWidth", 2, ...
         "Color", "#333333", "MarkerFaceColor", "#3399FF");
hold on;
plot(1, w_mean,  "o", "MarkerSize", 10, "MarkerFaceColor", "#3399FF",  "MarkerEdgeColor", "k");
plot(2, wf_mean, "o", "MarkerSize", 10, "MarkerFaceColor", "#FF6655",  "MarkerEdgeColor", "k");
set(gca, "XTick", [1 2], "XTickLabel", {"Wired", "WiFi"}, "XLim", [0.5 2.5]);
ylabel("Latency (ms)");
title("Mean ± Std Deviation");
grid on;

sgtitle("Network Performance: Wired vs WiFi (5G Router)", ...
        "FontSize", 14, "FontWeight", "bold");

saveas(f1, "figure1_overall_comparison.png");
fprintf("Saved: figure1_overall_comparison.png\n");

%% =====================================================
%% FIGURE 2 — Latency Distribution (Histogram)
%% =====================================================
f2 = figure("Name", "Latency Distribution", "NumberTitle", "off");
set(f2, "Position", [100 100 1000 500]);

% Find common bin edges for fair comparison
all_vals  = [w_valid, wf_valid];
bin_edges = linspace(min(all_vals)*0.9, max(all_vals)*1.1, 40);

% Wired
subplot(1, 2, 1);
histogram(w_valid, bin_edges, "FaceColor", "#3399FF", "EdgeColor", "white", "FaceAlpha", 0.85);
xline(w_mean, "r--", "LineWidth", 2, "Label", sprintf("Mean: %.1fms", w_mean));
xline(w_med,  "g--", "LineWidth", 1.5, "Label", sprintf("Median: %.1fms", w_med));
xlabel("Latency (ms)");
ylabel("Frequency");
title("Wired — Latency Distribution");
grid on;

% WiFi
subplot(1, 2, 2);
histogram(wf_valid, bin_edges, "FaceColor", "#FF6655", "EdgeColor", "white", "FaceAlpha", 0.85);
xline(wf_mean, "r--", "LineWidth", 2, "Label", sprintf("Mean: %.1fms", wf_mean));
xline(wf_med,  "g--", "LineWidth", 1.5, "Label", sprintf("Median: %.1fms", wf_med));
xlabel("Latency (ms)");
ylabel("Frequency");
title("WiFi — Latency Distribution");
grid on;

sgtitle("Latency Distribution: Wired vs WiFi", "FontSize", 13, "FontWeight", "bold");

saveas(f2, "figure2_latency_distribution.png");
fprintf("Saved: figure2_latency_distribution.png\n");

%% =====================================================
%% FIGURE 3 — Latency Over Time (every packet)
%% =====================================================
f3 = figure("Name", "Latency Over Time", "NumberTitle", "off");
set(f3, "Position", [150 150 1100 600]);

% Wired
subplot(2, 1, 1);
hold on;
plot(w_lat, "b.", "MarkerSize", 4, "DisplayName", "Wired");
yline(w_mean, "b--", "LineWidth", 1.5, ...
      "Label", sprintf("Mean: %.1fms", w_mean), ...
      "LabelHorizontalAlignment", "left");
yline(w_p95,  "r:",  "LineWidth", 1.2, ...
      "Label", sprintf("95th: %.1fms", w_p95), ...
      "LabelHorizontalAlignment", "left");
xlabel("Packet Number");
ylabel("Latency (ms)");
title("Wired — Latency per Packet");
legend("Location", "northeast");
grid on;

% WiFi
subplot(2, 1, 2);
hold on;
plot(wf_lat, "r.", "MarkerSize", 4, "DisplayName", "WiFi");
yline(wf_mean, "r--", "LineWidth", 1.5, ...
      "Label", sprintf("Mean: %.1fms", wf_mean), ...
      "LabelHorizontalAlignment", "left");
yline(wf_p95,  "b:",  "LineWidth", 1.2, ...
      "Label", sprintf("95th: %.1fms", wf_p95), ...
      "LabelHorizontalAlignment", "left");
xlabel("Packet Number");
ylabel("Latency (ms)");
title("WiFi — Latency per Packet");
legend("Location", "northeast");
grid on;

sgtitle("Latency Over Time: Wired vs WiFi", "FontSize", 13, "FontWeight", "bold");

saveas(f3, "figure3_latency_over_time.png");
fprintf("Saved: figure3_latency_over_time.png\n");

fprintf("\nAll done! Files saved:\n");
fprintf("  network_summary.csv\n");
fprintf("  figure1_overall_comparison.png\n");
fprintf("  figure2_latency_distribution.png\n");
fprintf("  figure3_latency_over_time.png\n");