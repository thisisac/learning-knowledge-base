clear; close all; clc;

%% 1. 读取文件
filename = 'bl.txt';
fid = fopen(filename, 'r');
data = textscan(fid, '%s', 'Delimiter', '\n');
fclose(fid);
lines = data{1};

%% 2. 提取基线名称
baseline_names = {};
for i = 1:length(lines)
    if ~isempty(lines{i}) && ~contains(lines{i}, '---------')
        parts = strsplit(strtrim(lines{i}));
        if length(parts) >= 2
            bl_name = parts{1};
            if ~ismember(bl_name, baseline_names) && contains(bl_name, '_')
                baseline_names{end+1} = bl_name;
            end
        end
    end
end
baseline_names = unique(baseline_names);
fprintf('找到 %d 个基线对:\n', length(baseline_names));
disp(baseline_names');

%% 3. 提取RMS数据
rms_data = struct();
for i = 1:length(baseline_names)
    bl = baseline_names{i};
    rms_data.(bl) = struct();
    
    % 查找该基线的RMS行
    for j = 1:length(lines)
        if contains(lines{j}, [bl ' RMS'])
            parts = strsplit(strtrim(lines{j}));
            % 提取N, E, U, L值
            rms_data.(bl).N = str2double(parts{4});
            rms_data.(bl).N_err = str2double(parts{6});
            rms_data.(bl).E = str2double(parts{8});
            rms_data.(bl).E_err = str2double(parts{10});
            rms_data.(bl).U = str2double(parts{12});
            rms_data.(bl).U_err = str2double(parts{14});
            rms_data.(bl).L = str2double(parts{16});
            rms_data.(bl).L_err = str2double(parts{18});
            break;
        end
    end
end

%% 4. 提取时间序列数据
time_series = struct();
for i = 1:length(baseline_names)
    bl = baseline_names{i};
    time_series.(bl) = struct();
    
    times = [];
    N_vals = []; N_errs = [];
    E_vals = []; E_errs = [];
    U_vals = []; U_errs = [];
    L_vals = []; L_errs = [];
    
    for j = 1:length(lines)
        if contains(lines{j}, bl) && contains(lines{j}, '2025.')
            parts = strsplit(strtrim(lines{j}));
            if length(parts) >= 15
                % 检查是否是时间序列行（包含2025.xxx）
                if ~isempty(str2num(parts{2}(1:4)))
                    times(end+1) = str2double(parts{2});
                    N_vals(end+1) = str2double(parts{4});
                    N_errs(end+1) = str2double(parts{6});
                    E_vals(end+1) = str2double(parts{8});
                    E_errs(end+1) = str2double(parts{10});
                    U_vals(end+1) = str2double(parts{12});
                    U_errs(end+1) = str2double(parts{14});
                    L_vals(end+1) = str2double(parts{16});
                    L_errs(end+1) = str2double(parts{18});
                end
            end
        end
    end
    
    if ~isempty(times)
        time_series.(bl).times = times;
        time_series.(bl).N = N_vals; time_series.(bl).N_err = N_errs;
        time_series.(bl).E = E_vals; time_series.(bl).E_err = E_errs;
        time_series.(bl).U = U_vals; time_series.(bl).U_err = U_errs;
        time_series.(bl).L = L_vals; time_series.(bl).L_err = L_errs;
    end
end

%% 5. 可视化1：RMS误差条形图
figure('Position', [100, 100, 1400, 600]);
subplot(2,2,1);

% 提取所有基线的N方向误差
N_errors = [];
E_errors = [];
U_errors = [];
L_errors = [];
bl_labels = {};

for i = 1:length(baseline_names)
    bl = baseline_names{i};
    if isfield(rms_data, bl)
        N_errors(i) = rms_data.(bl).N_err;
        E_errors(i) = rms_data.(bl).E_err;
        U_errors(i) = rms_data.(bl).U_err;
        L_errors(i) = rms_data.(bl).L_err;
        bl_labels{i} = bl;
    end
end

% N方向误差
bar(N_errors);
set(gca, 'XTick', 1:length(baseline_names), 'XTickLabel', baseline_names, ...
    'XTickLabelRotation', 45);
ylabel('误差 (m)');
title('N方向RMS误差');
grid on;

subplot(2,2,2);
bar(E_errors);
set(gca, 'XTick', 1:length(baseline_names), 'XTickLabel', baseline_names, ...
    'XTickLabelRotation', 45);
ylabel('误差 (m)');
title('E方向RMS误差');
grid on;

subplot(2,2,3);
bar(U_errors);
set(gca, 'XTick', 1:length(baseline_names), 'XTickLabel', baseline_names, ...
    'XTickLabelRotation', 45);
ylabel('误差 (m)');
title('U方向RMS误差');
grid on;

subplot(2,2,4);
bar(L_errors);
set(gca, 'XTick', 1:length(baseline_names), 'XTickLabel', baseline_names, ...
    'XTickLabelRotation', 45);
ylabel('误差 (m)');
title('L方向RMS误差');
grid on;

sgtitle('各基线RMS误差比较');

%% 6. 可视化2：选择几个基线的时间序列
figure('Position', [100, 100, 1200, 800]);

% 选择前4个基线进行时间序列显示
num_to_show = min(4, length(baseline_names));
selected_baselines = baseline_names(1:num_to_show);

for i = 1:num_to_show
    bl = selected_baselines{i};
    if isfield(time_series, bl) && ~isempty(time_series.(bl).times)
        subplot(num_to_show, 1, i);
        
        times = time_series.(bl).times;
        
        % 绘制N, E, U三个方向的时间序列
        errorbar(times, time_series.(bl).N, time_series.(bl).N_err, 'o-', 'LineWidth', 1.5, 'DisplayName', 'N');
        hold on;
        errorbar(times, time_series.(bl).E, time_series.(bl).E_err, 's-', 'LineWidth', 1.5, 'DisplayName', 'E');
        errorbar(times, time_series.(bl).U, time_series.(bl).U_err, '^-', 'LineWidth', 1.5, 'DisplayName', 'U');
        
        xlabel('时间 (年.日)');
        ylabel('位移 (m)');
        title(sprintf('基线 %s 时间序列', bl));
        legend('Location', 'best');
        grid on;
        
        % 设置x轴刻度
        set(gca, 'XTick', times);
        xlim([min(times)-0.001, max(times)+0.001]);
    end
end

%% 7. 可视化3：误差分布直方图
figure('Position', [100, 100, 1200, 600]);

all_N_err = [];
all_E_err = [];
all_U_err = [];

for i = 1:length(baseline_names)
    bl = baseline_names{i};
    if isfield(time_series, bl) && ~isempty(time_series.(bl).times)
        all_N_err = [all_N_err, time_series.(bl).N_err];
        all_E_err = [all_E_err, time_series.(bl).E_err];
        all_U_err = [all_U_err, time_series.(bl).U_err];
    end
end

subplot(1,3,1);
histogram(all_N_err, 15, 'FaceColor', 'blue', 'EdgeColor', 'black');
xlabel('N方向误差 (m)');
ylabel('频数');
title('N方向误差分布');
grid on;

subplot(1,3,2);
histogram(all_E_err, 15, 'FaceColor', 'green', 'EdgeColor', 'black');
xlabel('E方向误差 (m)');
ylabel('频数');
title('E方向误差分布');
grid on;

subplot(1,3,3);
histogram(all_U_err, 15, 'FaceColor', 'red', 'EdgeColor', 'black');
xlabel('U方向误差 (m)');
ylabel('频数');
title('U方向误差分布');
grid on;

sgtitle('各方向误差分布直方图');

%% 8. 统计信息输出
fprintf('\n========== 统计分析结果 ==========\n');

% 计算平均误差
fprintf('平均误差统计:\n');
fprintf('N方向平均误差: %.4f ± %.4f m\n', mean(all_N_err), std(all_N_err));
fprintf('E方向平均误差: %.4f ± %.4f m\n', mean(all_E_err), std(all_E_err));
fprintf('U方向平均误差: %.4f ± %.4f m\n', mean(all_U_err), std(all_U_err));

% 找出误差最大的基线
[max_N_err, idx_N] = max(N_errors);
[max_E_err, idx_E] = max(E_errors);
[max_U_err, idx_U] = max(U_errors);

fprintf('\n最大RMS误差:\n');
fprintf('N方向: %s - %.6f m\n', baseline_names{idx_N}, max_N_err);
fprintf('E方向: %s - %.6f m\n', baseline_names{idx_E}, max_E_err);
fprintf('U方向: %s - %.6f m\n', baseline_names{idx_U}, max_U_err);

% 找出误差最小的基线
[min_N_err, idx_N_min] = min(N_errors);
[min_E_err, idx_E_min] = min(E_errors);
[min_U_err, idx_U_min] = min(U_errors);

fprintf('\n最小RMS误差:\n');
fprintf('N方向: %s - %.6f m\n', baseline_names{idx_N_min}, min_N_err);
fprintf('E方向: %s - %.6f m\n', baseline_names{idx_E_min}, min_E_err);
fprintf('U方向: %s - %.6f m\n', baseline_names{idx_U_min}, min_U_err);

%% 9. 保存结果到CSV文件
% 保存RMS数据
rms_table = table();
for i = 1:length(baseline_names)
    bl = baseline_names{i};
    if isfield(rms_data, bl)
        rms_table.Baseline{i} = bl;
        rms_table.N(i) = rms_data.(bl).N;
        rms_table.N_err(i) = rms_data.(bl).N_err;
        rms_table.E(i) = rms_data.(bl).E;
        rms_table.E_err(i) = rms_data.(bl).E_err;
        rms_table.U(i) = rms_data.(bl).U;
        rms_table.U_err(i) = rms_data.(bl).U_err;
        rms_table.L(i) = rms_data.(bl).L;
        rms_table.L_err(i) = rms_data.(bl).L_err;
    end
end
writetable(rms_table, 'baseline_rms_data.csv');
fprintf('\nRMS数据已保存到 baseline_rms_data.csv\n');