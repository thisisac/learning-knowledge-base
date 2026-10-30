clc;
clear;

%% 实验参数设置
% 转台实验室地理位置参数
phi = 30.531651244 * pi / 180; % 纬度（弧度）
g = 9.7936174;                 % 重力加速度 (m/s²)
we = 7.292115e-5;              % 地球自转角速度 (rad/s)

% 星网宇达XW-GI7681（黄色）- 标定用具 - 100Hz
acc_scale = 1.5258789063E-06;
gyo_scale = 1.0850694444E-07;
sample_rate = 100;

% 转换因子
acc_conversion = acc_scale * sample_rate;
gyo_conversion = gyo_scale * sample_rate;

%% 数据读取函数（基于您提供的正确格式）
function [rawdata1, rawdata2] = Read_IMU_Data(filename, acc_conversion, gyo_conversion)
    % 打开文件并读取数据
    fileID = fopen(filename, 'r');
    data = textscan(fileID, '%s', 'Delimiter', '\n'); % 按行读取数据
    fclose(fileID);
    
    % 初始化存储变量
    num_samples = length(data{1});
    rawdata1 = zeros(num_samples, 3); % 加速度数据
    rawdata2 = zeros(num_samples, 3); % 陀螺仪数据
    
    % 遍历每一行数据
    for i = 1:num_samples
        line = data{1}{i}; % 去除校验码部分，星号后面的内容
        line = regexp(line, '^(.*?)(\*.*)?$', 'tokens', 'once');
        line = line{1}; % 提取去掉星号后的数据部分
        
        % 按逗号分割数据并转换为数字
        num = str2double(regexp(line, '[,;]', 'split'));
        
        % 检查数据是否有效
        if numel(num) >= 12
            % 提取加速度数据和陀螺仪数据
            acc_data = num(7:9);  % 加速度数据
            gyro_data = num(10:12); % 陀螺仪数据
            
            % 存储加速度和陀螺仪数据
            rawdata1(i, :) = [acc_data(3) * acc_conversion, -acc_data(2) * acc_conversion, acc_data(1) * acc_conversion];
            rawdata2(i, :) = [gyro_data(3) * gyo_conversion, -gyro_data(2) * gyo_conversion, gyro_data(1) * gyo_conversion];
        end
    end
    
    fprintf('文件 %s: 读取了 %d 行有效数据\n', filename, size(rawdata2,1));
end

%% 读取静态数据并求平均
function [f_avg, w_avg] = Read_Static_Average(filename, acc_conversion, gyo_conversion)
    [all_fdata, all_wdata] = Read_IMU_Data(filename, acc_conversion, gyo_conversion);
    
    % 计算平均值
    f_avg = mean(all_fdata, 1)';
    w_avg = mean(all_wdata, 1)';
end

%% 加速度计六位置标定
fprintf('=== 加速度计六位置标定 ===\n');

% 六个位置的比力向量理论值（导航系）
f_theoretical = [
    g, 0, 0;   % X朝上 -> +X感受重力
    -g, 0, 0;   % X朝下 -> -X感受重力
    0, g, 0;   % Y朝上 -> +Y感受重力  
    0, -g, 0;   % Y朝下 -> -Y感受重力
    0, 0, g;   % Z朝上 -> +Z感受重力
    0, 0, -g    % Z朝下 -> -Z感受重力
];

% 读取六个位置的静态数据
acc_files = {
    'x_up.ASC', 'x_down.ASC', ...
    'y_up.ASC', 'y_down.ASC', ...
    'z_up.ASC', 'z_down.ASC'
};

acc_position_names = {
    'X轴朝上', 'X轴朝下', ...
    'Y轴朝上', 'Y轴朝下', ...
    'Z轴朝上', 'Z轴朝下'
};

L = zeros(3, 6); % 观测矩阵
acc_means = zeros(6, 3);
gyro_static_means = zeros(6, 3);

for i = 1:6
    fprintf('处理位置 %d: %s\n', i, acc_files{i});
    [f_avg, w_avg] = Read_Static_Average(acc_files{i}, acc_conversion, gyo_conversion);
    
    L(:, i) = f_avg;
    acc_means(i, :) = f_avg';
    gyro_static_means(i, :) = w_avg';
    
    fprintf('  加速度平均: [%.6f, %.6f, %.6f] m/s²\n', f_avg);
    fprintf('  陀螺仪平均: [%.8f, %.8f, %.8f] rad/s\n', w_avg);
end

% 构建设计矩阵 A
A = [f_theoretical'; ones(1, 6)];  % 4×6 矩阵

% 最小二乘求解加速度计误差参数矩阵 M
M = L * A' / (A * A');

fprintf('\n加速度计误差参数矩阵 M:\n');
disp(M);

% 提取误差参数
bias_acc = M(:, 4);  % 零偏向量
scale_matrix = M(1:3, 1:3);  % 比例因子和交轴耦合矩阵

scale_acc = diag(scale_matrix) - 1;  % 比例因子误差
cross_acc = scale_matrix - diag(diag(scale_matrix));  % 交轴耦合误差

fprintf('加速度计零偏 (m/s²):\n');
fprintf('  bx = %.6f, by = %.6f, bz = %.6f\n', bias_acc);
fprintf('加速度计比例因子误差:\n');
fprintf('  δx = %.6f, δy = %.6f, δz = %.6f\n', scale_acc);
fprintf('加速度计交轴耦合误差:\n');
fprintf('  m_xy = %.6f, m_xz = %.6f\n', cross_acc(1,2), cross_acc(1,3));
fprintf('  m_yx = %.6f, m_yz = %.6f\n', cross_acc(2,1), cross_acc(2,3));
fprintf('  m_zx = %.6f, m_zy = %.6f\n', cross_acc(3,1), cross_acc(3,2));

%% 陀螺仪零偏标定（静态六位置法）
fprintf('\n=== 陀螺仪零偏标定（静态六位置法）===\n');

gyro_bias_static = zeros(3, 1);

for axis = 1:3
    pos_idx = (axis-1)*2 + 1;  % 朝上位置
    neg_idx = (axis-1)*2 + 2;  % 朝下位置
    
    % 对于Z轴需要考虑地球自转影响
    if axis == 3
        earth_component = we * sin(phi);
        gyro_bias_static(axis) = (gyro_static_means(pos_idx, axis) + ...
                                 gyro_static_means(neg_idx, axis)) / 2 - earth_component;
    else
        gyro_bias_static(axis) = (gyro_static_means(pos_idx, axis) + ...
                                 gyro_static_means(neg_idx, axis)) / 2;
    end
end

fprintf('陀螺仪零偏 (rad/s):\n');
fprintf('  bgx = %.8f\n', 3600*180/3.1415926*gyro_bias_static(1));
fprintf('  bgy = %.8f\n', 3600*180/3.1415926*gyro_bias_static(2));
fprintf('  bgz = %.8f\n', 3600*180/3.1415926*gyro_bias_static(3));

%% 陀螺仪比例因子标定（角位置法）
fprintf('\n=== 陀螺仪比例因子标定（角位置法）===\n');

% 转动参数
alpha = 360 * pi / 180;  % 转动角度 360°（弧度）
t_rotate = 36;           % 转动时间 36秒（转速10°/s）

% 转动数据文件
gyro_files = {
    'x_r.ASC', 'x_l.ASC', ...  % X轴正转、反转
    'y_r.ASC', 'y_l.ASC', ...  % Y轴正转、反转  
    'z_r.ASC', 'z_l.ASC'       % Z轴正转、反转
};

gyro_position_names = {
    'X轴正转', 'X轴反转', ...
    'Y轴正转', 'Y轴反转', ...
    'Z轴正转', 'Z轴反转'
};

% 转动区间（需要根据实际波形调整）
rotation_intervals = [
    1, 5880;   % X轴正转区间
    1, 5600;   % X轴反转区间
    4, 5600;   % Y轴正转区间
    3, 11000;   % Y轴反转区间
    1, 4600;   % Z轴正转区间
    2, 5590    % Z轴反转区间
];

gyro_scale_error = zeros(3, 1);
angle_integrals = zeros(3, 2);

for axis = 1:3
    fprintf('\n--- %s轴陀螺比例因子标定 ---\n', char('x' + axis - 1));
    
    % 正转数据
    pos_idx = (axis-1)*2 + 1;
    [~, pos_wdata] = Read_IMU_Data(gyro_files{pos_idx}, 0, gyo_conversion);
    
    % 反转数据
    neg_idx = (axis-1)*2 + 2;
    [~, neg_wdata] = Read_IMU_Data(gyro_files{neg_idx}, 0, gyo_conversion);
    
    % 提取转动区间的数据并积分
    pos_start = rotation_intervals(pos_idx, 1);
    pos_end = rotation_intervals(pos_idx, 2);
    neg_start = rotation_intervals(neg_idx, 1);
    neg_end = rotation_intervals(neg_idx, 2);
    
    if pos_end > size(pos_wdata, 1)
        pos_end = size(pos_wdata, 1);
    end
    if neg_end > size(neg_wdata, 1)
        neg_end = size(neg_wdata, 1);
    end
    
    delta_t = 1/sample_rate;
    
    pos_integral = sum(pos_wdata(pos_start:pos_end, axis)) * delta_t;
    neg_integral = sum(neg_wdata(neg_start:neg_end, axis)) * delta_t;
    
    angle_integrals(axis, :) = [pos_integral, neg_integral];
    
    fprintf('  正转角度积分: %.6f rad\n', pos_integral);
    fprintf('  反转角度积分: %.6f rad\n', neg_integral);
    
    % 计算比例因子误差
    scale_gyro = -((pos_integral - neg_integral) / (2 * alpha) + 1);
    gyro_scale_error(axis) = scale_gyro;
    
    fprintf('  比例因子误差: %.6f\n', scale_gyro);
end

fprintf('\n陀螺仪标定结果汇总:\n');
for axis = 1:3
    fprintf('%s轴: 零偏 = %.8f rad/s, 比例因子误差 = %.6f\n', ...
            char('x' + axis - 1), gyro_bias_static(axis), gyro_scale_error(axis));
end

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