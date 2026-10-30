%HW3
clear; clc; close all;

load('data_5G.mat');

x_fo   = recv_signal_freq_offset;   % 含大频偏的接收信号
y_grid = ref_signal_grid;           % 本地模板 y1,y2,y3 (256×3)
fs     = 7.68e6;                    % 采样率 7.68 MHz

fprintf('含频偏信号长度：%d 采样点\n', length(x_fo));

df_search = -50000 : 2000 : 50000;     
max_peak = 0;
best_df  = 0;
best_r   = zeros(size(x_fo));

fprintf('正在搜索最佳频偏... (共 %d 个候选值)\n', length(df_search));

for df = df_search

    t = (0:length(x_fo)-1)' / fs;
    comp = exp(-1j * 2 * pi * df * t);
    x_comp = x_fo .* comp;
    

    r = abs(xcorr(x_comp, y_grid(:,1)));
    current_max = max(r);
    
    if current_max > max_peak
        max_peak = current_max;
        best_df  = df;
        best_r   = r;
    end
end

fprintf(' 频偏估计结果 ');
fprintf('估计载波频偏 Δf ≈ %.0f Hz\n', best_df);
fprintf('补偿后最大相关峰值 ≈ %.0f\n', max_peak);


figure('Position',[100 100 1200 600]);

subplot(1,2,1);
[r_raw, lags] = xcorr(x_fo, y_grid(:,1));
plot(lags, abs(r_raw), 'Color',[1 0.7 0], 'LineWidth',1.5);
title('补偿前相关结果（recv\_signal\_freq\_offset）');
xlabel('Lag (samples)'); ylabel('相关幅度');
xlim([0 5e5]); ylim([0 800]);
grid on;

subplot(1,2,2);
plot(lags, best_r, 'b', 'LineWidth',2);
title(sprintf('补偿后相关结果（Δf=%.0f Hz）', best_df));
xlabel('Lag (samples)'); ylabel('相关幅度');
xlim([0 5e5]); ylim([0 max_peak*1.1]);
grid on;
hold on;
[~, peak_idx] = max(best_r);
plot(lags(peak_idx), best_r(peak_idx), 'r*', 'MarkerSize',15);
text(lags(peak_idx)+8000, best_r(peak_idx)*0.9, ...
     sprintf('峰值=%.0f', best_r(peak_idx)), 'Color','r','FontSize',12);

figure('Position',[100 800 1100 500]);
max_peaks = zeros(1,3);
for k = 1:3
    y = y_grid(:,k);
    t = (0:length(x_fo)-1)' / fs;
    comp = exp(-1j * 2 * pi * best_df * t);
    x_comp = x_fo .* comp;
    r = abs(xcorr(x_comp, y));
    max_peaks(k) = max(r);
    
    subplot(1,3,k);
    plot(lags, r, 'LineWidth',1.5);
    title(['补偿后 R(x_{fo}, y_' num2str(k) ')']);
    xlabel('Lag (samples)'); ylabel('相关幅度');
    grid on; xlim([0 5e5]);
end
fprintf('补偿后三个模板峰值：y1=%.0f, y2=%.0f, y3=%.0f\n', max_peaks);


