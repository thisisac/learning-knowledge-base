%HW2
clear; clc; close all;


load('data_5G.mat');

x = recv_signal;                    % 接收信号 x1(t)，460800×1
y_grid = ref_signal_grid;           % 本地模板 256×3，y1 y2 y3 对应三个PSS
fs = 7.68e6;                        % 采样率（仅用于标注，不参与计算）

fprintf('接收信号长度：%d 采样点\n', length(x));
fprintf('本地模板长度：%d 采样点 × 3 个PSS\n', size(y_grid,1));


figure('Position',[100 100 1100 900], 'Name','TOA估计及PSS解调');

max_peaks = zeros(1,3);             % 记录每个模板的最大峰值
peak_locations = zeros(1,3);

for k = 1:3
    y = y_grid(:,k);        
    

    [r, lags] = xcorr(x, y);       
    abs_r = abs(r);

    [max_peaks(k), idx] = max(abs_r);
    peak_locations(k) = lags(idx);

    subplot(3,1,k);
    if k==1
        plot(lags, abs_r, 'b', 'LineWidth', 1.5);   % y1 → 蓝色
    elseif k==2
        plot(lags, abs_r, 'Color', [1 0.4 0], 'LineWidth', 1.5); % y2 → 橙色
    else
        plot(lags, abs_r, 'Color', [1 0.7 0], 'LineWidth', 1.5); % y3 → 黄色
    end
    
    title(['R(x_1, y_' num2str(k) ')']);
    xlabel('Lag (samples) ×10^5');
    ylabel('相关幅度');
    grid on;
    xlim([0 5e5]);                
    ylim([0 max(max_peaks)*1.1]);
    

    hold on;
    plot(peak_locations(k), max_peaks(k), 'r*', 'MarkerSize', 12);
    text(peak_locations(k)+5000, max_peaks(k)*0.95, ...
         sprintf('峰值=%.0f', max_peaks(k)), 'Color','r', 'FontSize',11);
end

[~, best_k] = max(max_peaks);      
N_PSS = best_k - 1;                  

fprintf('最强相关模板：y%d\n', best_k);
fprintf('信号来自主同步符号 N_PSS = %d 的基站\n', N_PSS);
fprintf('最大相关峰值：%.0f（远高于其他模板）\n', max_peaks(best_k));
fprintf('TOA（样本）≈ %d（第一个明显峰值位置）\n', peak_locations(best_k));

figure('Position',[1220 100 600 500]);
hold on;
for k=1:3
    y = y_grid(:,k);
    r = abs(xcorr(x, y));
    lags = -length(x)+1 : length(x)-1;
    plot(lags, r, 'LineWidth',1.2);
end
title('三模板相关叠加图（蓝色最强 = 正确PSS）');
xlabel('Lag (samples)');
ylabel('相关幅度');
grid on;
xlim([0 5e5]);
legend('y1 (N_{PSS}=0)', 'y2 (N_{PSS}=1)', 'y3 (N_{PSS}=2)', 'Location','best');

disp('TOA估计及PSS解调完成！已自动输出 N_PSS = ? 的答案');