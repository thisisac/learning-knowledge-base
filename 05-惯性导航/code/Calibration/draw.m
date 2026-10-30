%% 误差补偿验证
fprintf('\n=== 误差补偿验证 ===\n');

% 构建完整的加速度计误差矩阵
error_matrix_acc = [scale_matrix, bias_acc; 0, 0, 0, 1];

% 对X轴朝上位置的数据进行补偿验证
test_position = 1;
raw_data_sample = acc_means(test_position, :);
raw_data_homogeneous = [raw_data_sample, 1]';

% 补偿：m_compensated = inv(M) * m_raw
compensated_homogeneous = error_matrix_acc \ raw_data_homogeneous;
compensated_data = compensated_homogeneous(1:3)';

fprintf('位置: %s\n', acc_position_names{test_position});
fprintf('原始测量:  [%.6f, %.6f, %.6f] m/s²\n', raw_data_sample);
fprintf('补偿后:    [%.6f, %.6f, %.6f] m/s²\n', compensated_data);
fprintf('理论值:    [%.6f, %.6f, %.6f] m/s²\n', f_theoretical(test_position, :));

compensation_error = compensated_data - f_theoretical(test_position, :);
fprintf('补偿误差:  [%.6f, %.6f, %.6f] m/s²\n', compensation_error);


%% 加速度计时序图补偿前后对比 - 所有六个位置
fprintf('\n=== 加速度计时序图补偿前后对比 - 所有六个位置 ===\n');

% 定义所有六个位置的文件名
position_files = {
    'x_up.ASC', 'x_down.ASC', ...
    'y_up.ASC', 'y_down.ASC', ...
    'z_up.ASC', 'z_down.ASC'
};

position_names = {
    'X轴朝上', 'X轴朝下', ...
    'Y轴朝上', 'Y轴朝下', ...
    'Z轴朝上', 'Z轴朝下'
};

% 理论重力向量
theoretical_gravity = [
    g, 0, 0;   % X轴朝上
    -g, 0, 0;  % X轴朝下
    0, g, 0;   % Y轴朝上
    0, -g, 0;  % Y轴朝下
    0, 0, g;   % Z轴朝上
    0, 0, -g   % Z轴朝下
];

% 修正的补偿函数 - 使用标定参数
function compensated_data = compensate_acceleration_data(raw_data, scale_matrix, bias_acc)
    num_points = size(raw_data, 1);
    compensated_data = zeros(size(raw_data));
    
    for i = 1:num_points
        % 原始测量值
        m_raw = raw_data(i, :)';
        
        % 补偿公式: m_compensated = inv(scale_matrix) * (m_raw - bias_acc)
        m_compensated = scale_matrix \ (m_raw - bias_acc);
        
        compensated_data(i, :) = m_compensated';
    end
end

% 预分配存储补偿前后数据的cell数组
raw_data_all = cell(6, 1);
compensated_data_all = cell(6, 1);
time_all = cell(6, 1);

% 读取所有位置的数据并进行补偿
for i = 1:6
    fprintf('处理位置 %d: %s\n', i, position_names{i});
    
    % 读取数据
    [rawdata, ~] = Read_IMU_Data(position_files{i}, acc_conversion, gyo_conversion);
    raw_data_all{i} = rawdata;
    
    % 进行补偿
    compensated_data_all{i} = compensate_acceleration_data(rawdata, scale_matrix, bias_acc);
    
    % 创建时间向量
    time_all{i} = (1:size(rawdata, 1)) / sample_rate;
    
    fprintf('  数据点数: %d\n', size(rawdata, 1));
end

%% 绘制所有位置的时序图对比 - 每个位置一个图
for i = 1:6
    figure('Position', [100, 100, 1200, 800]);
    
    % 当前位置的数据
    current_raw = raw_data_all{i};
    current_comp = compensated_data_all{i};
    current_time = time_all{i};
    
    % X轴数据对比
    subplot(3, 1, 1);
    plot(current_time, current_raw(:, 1), 'b-', 'LineWidth', 1.5, 'DisplayName', '补偿前');
    hold on;
    plot(current_time, current_comp(:, 1), 'r-', 'LineWidth', 1.5, 'DisplayName', '补偿后');
    % 添加理论值参考线
    yline(theoretical_gravity(i, 1), 'g--', 'LineWidth', 1, 'DisplayName', '理论值');
    xlabel('时间 (s)');
    ylabel('X轴加速度 (m/s²)');
    title(sprintf('%s - X轴加速度补偿前后对比', position_names{i}));
    legend('Location', 'best');
    grid on;
    
    % Y轴数据对比
    subplot(3, 1, 2);
    plot(current_time, current_raw(:, 2), 'b-', 'LineWidth', 1.5, 'DisplayName', '补偿前');
    hold on;
    plot(current_time, current_comp(:, 2), 'r-', 'LineWidth', 1.5, 'DisplayName', '补偿后');
    % 添加理论值参考线
    yline(theoretical_gravity(i, 2), 'g--', 'LineWidth', 1, 'DisplayName', '理论值');
    xlabel('时间 (s)');
    ylabel('Y轴加速度 (m/s²)');
    title(sprintf('%s - Y轴加速度补偿前后对比', position_names{i}));
    legend('Location', 'best');
    grid on;
    
    % Z轴数据对比
    subplot(3, 1, 3);
    plot(current_time, current_raw(:, 3), 'b-', 'LineWidth', 1.5, 'DisplayName', '补偿前');
    hold on;
    plot(current_time, current_comp(:, 3), 'r-', 'LineWidth', 1.5, 'DisplayName', '补偿后');
    % 添加理论值参考线
    yline(theoretical_gravity(i, 3), 'g--', 'LineWidth', 1, 'DisplayName', '理论值');
    xlabel('时间 (s)');
    ylabel('Z轴加速度 (m/s²)');
    title(sprintf('%s - Z轴加速度补偿前后对比', position_names{i}));
    legend('Location', 'best');
    grid on;
    
    sgtitle(sprintf('加速度计补偿前后时序图 - %s', position_names{i}));
end

%% 绘制所有位置的三轴叠加对比图
figure('Position', [100, 100, 1400, 1200]);

for i = 1:6
    subplot(3, 2, i);
    current_raw = raw_data_all{i};
    current_comp = compensated_data_all{i};
    current_time = time_all{i};
    
    % 绘制补偿前三轴数据
    plot(current_time, current_raw(:, 1), 'b-', 'LineWidth', 1, 'DisplayName', '补偿前X轴');
    hold on;
    plot(current_time, current_raw(:, 2), 'g-', 'LineWidth', 1, 'DisplayName', '补偿前Y轴');
    plot(current_time, current_raw(:, 3), 'r-', 'LineWidth', 1, 'DisplayName', '补偿前Z轴');
    
    % 绘制补偿后三轴数据
    plot(current_time, current_comp(:, 1), 'b--', 'LineWidth', 1.5, 'DisplayName', '补偿后X轴');
    plot(current_time, current_comp(:, 2), 'g--', 'LineWidth', 1.5, 'DisplayName', '补偿后Y轴');
    plot(current_time, current_comp(:, 3), 'r--', 'LineWidth', 1.5, 'DisplayName', '补偿后Z轴');
    
    % 添加理论值参考线
    yline(theoretical_gravity(i, 1), 'b:', 'LineWidth', 0.5, 'DisplayName', 'X理论值');
    yline(theoretical_gravity(i, 2), 'g:', 'LineWidth', 0.5, 'DisplayName', 'Y理论值');
    yline(theoretical_gravity(i, 3), 'r:', 'LineWidth', 0.5, 'DisplayName', 'Z理论值');
    
    xlabel('时间 (s)');
    ylabel('加速度 (m/s²)');
    title(sprintf('%s - 三轴补偿前后对比', position_names{i}));
    legend('Location', 'best');
    grid on;
end

sgtitle('所有位置加速度计三轴补偿前后对比图');

%% 计算并显示所有位置的统计信息
fprintf('\n=== 所有位置补偿效果统计分析 ===\n');

for i = 1:6
    fprintf('\n%s位置统计:\n', position_names{i});
    fprintf('  理论值: [%.6f, %.6f, %.6f] m/s²\n', theoretical_gravity(i, :));
    
    current_raw = raw_data_all{i};
    current_comp = compensated_data_all{i};
    
    % 平均值对比
    raw_mean = mean(current_raw, 1);
    comp_mean = mean(current_comp, 1);
    
    fprintf('  补偿前平均值: [%.6f, %.6f, %.6f] m/s²\n', raw_mean);
    fprintf('  补偿后平均值: [%.6f, %.6f, %.6f] m/s²\n', comp_mean);
    
    % 计算与理论值的误差
    raw_error = raw_mean - theoretical_gravity(i, :);
    comp_error = comp_mean - theoretical_gravity(i, :);
    
    fprintf('  补偿前误差:   [%.6f, %.6f, %.6f] m/s²\n', raw_error);
    fprintf('  补偿后误差:   [%.6f, %.6f, %.6f] m/s²\n', comp_error);
    
    % 标准差对比
    raw_std = std(current_raw, 0, 1);
    comp_std = std(current_comp, 0, 1);
    improvement = (raw_std - comp_std) ./ raw_std * 100;
    
    fprintf('  补偿前标准差: [%.6f, %.6f, %.6f] m/s²\n', raw_std);
    fprintf('  补偿后标准差: [%.6f, %.6f, %.6f] m/s²\n', comp_std);
    fprintf('  标准差改善:   [%.2f%%, %.2f%%, %.2f%%]\n', improvement);
end

%% 绘制误差统计对比图
figure('Position', [100, 100, 1400, 800]);

% 计算所有位置的误差
raw_errors = zeros(6, 3);
comp_errors = zeros(6, 3);

for i = 1:6
    raw_mean = mean(raw_data_all{i}, 1);
    comp_mean = mean(compensated_data_all{i}, 1);
    raw_errors(i, :) = raw_mean - theoretical_gravity(i, :);
    comp_errors(i, :) = comp_mean - theoretical_gravity(i, :);
end

% X轴误差对比
subplot(2, 2, 1);
bar([raw_errors(:, 1), comp_errors(:, 1)]);
set(gca, 'XTickLabel', position_names);
ylabel('X轴误差 (m/s²)');
title('X轴补偿前后误差对比');
legend('补偿前', '补偿后', 'Location', 'best');
grid on;

% Y轴误差对比
subplot(2, 2, 2);
bar([raw_errors(:, 2), comp_errors(:, 2)]);
set(gca, 'XTickLabel', position_names);
ylabel('Y轴误差 (m/s²)');
title('Y轴补偿前后误差对比');
legend('补偿前', '补偿后', 'Location', 'best');
grid on;

% Z轴误差对比
subplot(2, 2, 3);
bar([raw_errors(:, 3), comp_errors(:, 3)]);
set(gca, 'XTickLabel', position_names);
ylabel('Z轴误差 (m/s²)');
title('Z轴补偿前后误差对比');
legend('补偿前', '补偿后', 'Location', 'best');
grid on;

% 总误差对比（绝对值之和）
subplot(2, 2, 4);
total_raw_errors = sum(abs(raw_errors), 2);
total_comp_errors = sum(abs(comp_errors), 2);
bar([total_raw_errors, total_comp_errors]);
set(gca, 'XTickLabel', position_names);
ylabel('总误差绝对值 (m/s²)');
title('总误差绝对值对比');
legend('补偿前', '补偿后', 'Location', 'best');
grid on;

sgtitle('所有位置补偿前后误差统计对比');

fprintf('\n所有位置时序图绘制完成！\n');