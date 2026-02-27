%% PID Position Response Plot
%  Usage:
%  1. Run the STM32 in Debug mode (CLion)
%  2. After homing, hand-turn the motor shaft to trigger recording
%  3. Wait ~2 seconds (2000 samples @ 1kHz)
%  4. Check rec_done == 1 in debugger
%  5. In CLion GDB console, dump the arrays:
%       (gdb) set print elements 0
%       (gdb) set print array-indexes off
%       (gdb) output rec_target
%       (gdb) output rec_actual
%  6. Paste the values below and run this script

%% ====== PASTE YOUR DATA HERE ======
% Replace these example arrays with the real data from GDB.
% GDB prints like: {0, 0, 0, -45, -43, ...}
% Convert to MATLAB: [0, 0, 0, -45, -43, ...]

rec_target = [0];  % ← Paste rec_target data here
rec_actual = [0];  % ← Paste rec_actual data here

%% ====== Plot ======
Fs = 1000;  % Sample rate = 1 kHz (PID frequency)
N  = min(length(rec_target), length(rec_actual));
t  = (0:N-1) / Fs * 1000;  % Time in milliseconds

figure('Name', 'PID Position Response', 'NumberTitle', 'off');
hold on; grid on;

plot(t, rec_target(1:N), 'r--', 'LineWidth', 2, 'DisplayName', 'Target Position');
plot(t, rec_actual(1:N), 'b-',  'LineWidth', 1.5, 'DisplayName', 'Actual Position');

xlabel('Time (ms)', 'FontSize', 12);
ylabel('Encoder Counts', 'FontSize', 12);
title(sprintf('PID Position Response  (Kp=%.1f, Ki=%.2f, Kd=%.1f)', 5.0, 0.01, 0.1), 'FontSize', 14);
legend('Location', 'best', 'FontSize', 11);

% Mark settling region
yline(0, 'k:', 'LineWidth', 0.5);

hold off;

fprintf('Samples: %d\n', N);
fprintf('Duration: %.1f ms\n', N / Fs * 1000);
fprintf('Max overshoot: %d counts\n', max(abs(rec_actual(1:N) - rec_target(1:N))));
