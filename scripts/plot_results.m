% plot_results.m - draws the graphs in docs/ from the CSV files in results/.
%
% Run from the repository root after "make results":
%   MATLAB:  run('scripts/plot_results.m')
%   Octave:  octave-cli scripts/plot_results.m

if exist('OCTAVE_VERSION', 'builtin')
    graphics_toolkit('gnuplot');  % lets Octave save figures without a display
end
if exist('readmatrix') > 0
    load_csv = @(file) readmatrix(file);
else
    load_csv = @(file) dlmread(file, ',', 1, 0);  % skip the header row
end
if ~exist('docs', 'dir')
    mkdir('docs');
end

schemes = {'none', 'arq', 'cc', 'ir'};
names = {'No HARQ', 'ARQ (no combining)', 'HARQ chase combining', 'HARQ incremental redundancy'};
marks = {'-o', '-s', '-^', '-d'};
colors = [0.50 0.50 0.50;   % no HARQ: grey
          0.00 0.45 0.74;   % ARQ: blue
          0.85 0.33 0.10;   % chase combining: orange
          0.47 0.67 0.19];  % incremental redundancy: green
snr_range = [-6 10];

% Columns of scheme_*.csv: 1 snr_db, 2 throughput_kbps, 3 bler_first_tx,
% 4 residual_bler, 5 avg_transmissions, 6 avg_latency_ms
data = cell(size(schemes));
for k = 1:numel(schemes)
    data{k} = load_csv(fullfile('results', ['scheme_' schemes{k} '.csv']));
end

% 1. Throughput vs SNR ------------------------------------------------------
fig = figure('visible', 'off');
hold on; grid on; box on;
for k = 1:numel(schemes)
    plot(data{k}(:, 1), data{k}(:, 2), marks{k}, 'Color', colors(k, :), 'LineWidth', 1.5, 'MarkerSize', 5);
end
xlim(snr_range);
xlabel('SNR, Es/N0 (dB)');
ylabel('Throughput (kbit/s)');
title('Throughput vs SNR (8 HARQ processes, up to 4 transmissions)');
legend(names, 'Location', 'northwest');
print(fig, '-dpng', '-r130', fullfile('docs', 'throughput.png'));
close(fig);

% 2. Block error rate: first transmission vs after HARQ -----------------------
fig = figure('visible', 'off');
first = data{1}(:, 3);  % the first transmission is the same for every scheme
first(first == 0) = NaN;  % zero cannot be drawn on a log axis
semilogy(data{1}(:, 1), first, marks{1}, 'Color', colors(1, :), 'LineWidth', 1.5, 'MarkerSize', 5);
hold on; grid on; box on;
for k = 2:numel(schemes)
    residual = data{k}(:, 4);
    residual(residual == 0) = NaN;
    semilogy(data{k}(:, 1), residual, marks{k}, 'Color', colors(k, :), 'LineWidth', 1.5, 'MarkerSize', 5);
end
xlim(snr_range);
ylim([1e-5 1.5]);
xlabel('SNR, Es/N0 (dB)');
ylabel('Block error rate');
title('Blocks lost after up to 4 transmissions');
legend({'First transmission (= no HARQ)', 'ARQ', 'Chase combining', 'Incremental redundancy'}, ...
       'Location', 'southeast');
print(fig, '-dpng', '-r130', fullfile('docs', 'bler.png'));
close(fig);

% 3. Average transmissions per block -----------------------------------------
fig = figure('visible', 'off');
hold on; grid on; box on;
for k = 2:numel(schemes)
    plot(data{k}(:, 1), data{k}(:, 5), marks{k}, 'Color', colors(k, :), 'LineWidth', 1.5, 'MarkerSize', 5);
end
xlim(snr_range);
ylim([0.9 4.1]);
xlabel('SNR, Es/N0 (dB)');
ylabel('Average transmissions per block');
title('How many times each block is sent');
legend(names(2:end), 'Location', 'southwest');
print(fig, '-dpng', '-r130', fullfile('docs', 'transmissions.png'));
close(fig);

% 4. Uncoded QPSK bit error rate vs theory -------------------------------------
ber = load_csv(fullfile('results', 'qpsk_ber_validation.csv'));  % ebn0_db, measured, theory, bits, errors
fig = figure('visible', 'off');
semilogy(ber(:, 1), ber(:, 3), 'k-', 'LineWidth', 1.5);
hold on; grid on; box on;
semilogy(ber(:, 1), ber(:, 2), 'ro', 'LineWidth', 1.5, 'MarkerSize', 6);
xlabel('Eb/N0 (dB)');
ylabel('Bit error rate');
title('Uncoded QPSK over AWGN: simulation vs theory');
legend({'Theory: 0.5 erfc(sqrt(Eb/N0))', 'This simulator'}, 'Location', 'southwest');
print(fig, '-dpng', '-r130', fullfile('docs', 'qpsk_ber_validation.png'));
close(fig);

% 5. Throughput vs number of HARQ processes ------------------------------------
sweep = load_csv(fullfile('results', 'process_sweep.csv'));  % processes, throughput, busy_fraction, ...
fig = figure('visible', 'off');
bar(1:size(sweep, 1), sweep(:, 2));
box on;
set(gca, 'YGrid', 'on', 'XGrid', 'off', 'XTick', 1:size(sweep, 1), ...
    'XTickLabel', cellfun(@num2str, num2cell(sweep(:, 1)), 'UniformOutput', false));
for k = 1:size(sweep, 1)
    text(k, sweep(k, 2) + 15, sprintf('%.1f kbit/s', sweep(k, 2)), ...
         'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom');
end
xlabel('Number of parallel HARQ processes');
ylabel('Throughput (kbit/s)');
title('Why LTE runs 8 HARQ processes (8 ms round trip, SNR 4 dB)');
ylim([0 900]);
print(fig, '-dpng', '-r130', fullfile('docs', 'process_sweep.png'));
close(fig);

disp('Saved docs/throughput.png, bler.png, transmissions.png, qpsk_ber_validation.png, process_sweep.png');
