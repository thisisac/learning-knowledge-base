%% 行人航迹推算（PDR）完整实现
% 作者：刘峻琰，武汉大学
% 日期：2025年12月2日
% 课程：室内定位技术

clear all; close all; clc;

%% 1. 数据读取和预处理
fprintf('=== 步骤1: 数据读取和预处理 ===\n');

% 读取传感器数据
sensor_data = load('sensors_matrix.txt');
time_sensor = sensor_data(:, 1);  % 时间 (s)
gyro_x = sensor_data(:, 2);       % 陀螺X (deg/s)
gyro_y = sensor_data(:, 3);       % 陀螺Y (deg/s)
gyro_z = sensor_data(:, 4);       % 陀螺Z (deg/s)
acc_x = sensor_data(:, 5);        % 加速度计X (m/s^2)
acc_y = sensor_data(:, 6);        % 加速度计Y (m/s^2)
acc_z = sensor_data(:, 7);        % 加速度计Z (m/s^2)
mag_x = sensor_data(:, 8);        % 磁强计X (mGauss)
mag_y = sensor_data(:, 9);        % 磁强计Y (mGauss)
mag_z = sensor_data(:, 10);       % 磁强计Z (mGauss)

% 读取参考位置数据（用于验证）
pos_ref = load('pos_reference.txt');
time_ref = pos_ref(:, 1);         % 时间 (s)
lat_ref = pos_ref(:, 2);          % 纬度
lon_ref = pos_ref(:, 3);          % 经度

% 数据检查
fprintf('传感器数据点数: %d\n', length(time_sensor));
fprintf('参考位置点数: %d\n', length(time_ref));
fprintf('采样间隔: %.3f 秒\n', time_sensor(2) - time_sensor(1));

% 转换为国际单位制
gyro_x = deg2rad(gyro_x);  % 转换为 rad/s
gyro_y = deg2rad(gyro_y);
gyro_z = deg2rad(gyro_z);

% 磁强计转换为Tesla (1 Gauss = 1e-4 Tesla, 但保留原单位用于计算)
mag_x = mag_x * 1e-7;  % 转换为 Tesla (1 mGauss = 1e-7 Tesla)
mag_y = mag_y * 1e-7;
mag_z = mag_z * 1e-7;

% 重力加速度 (m/s^2)
g = 9.80665;

%% 2. 绘制传感器数据
fprintf('\n=== 步骤2: 绘制传感器数据 ===\n');

figure('Position', [100, 100, 1200, 800]);

% 陀螺数据
subplot(3, 3, 1);
plot(time_sensor, rad2deg(gyro_x), 'r'); hold on;
plot(time_sensor, rad2deg(gyro_y), 'g');
plot(time_sensor, rad2deg(gyro_z), 'b');
xlabel('时间 (s)'); ylabel('角速度 (deg/s)');
title('陀螺数据');
legend('X', 'Y', 'Z');
grid on;

% 加速度计数据
subplot(3, 3, 2);
plot(time_sensor, acc_x, 'r'); hold on;
plot(time_sensor, acc_y, 'g');
plot(time_sensor, acc_z, 'b');
plot(time_sensor, sqrt(acc_x.^2 + acc_y.^2 + acc_z.^2), 'k--', 'LineWidth', 1.5);
xlabel('时间 (s)'); ylabel('加速度 (m/s^2)');
title('加速度计数据');
legend('X', 'Y', 'Z', '模长');
grid on;

% 磁强计数据
subplot(3, 3, 3);
plot(time_sensor, mag_x*1e7, 'r'); hold on;
plot(time_sensor, mag_y*1e7, 'g');
plot(time_sensor, mag_z*1e7, 'b');
plot(time_sensor, sqrt(mag_x.^2 + mag_y.^2 + mag_z.^2)*1e7, 'k--', 'LineWidth', 1.5);
xlabel('时间 (s)'); ylabel('磁场强度 (mGauss)');
title('磁强计数据');
legend('X', 'Y', 'Z', '模长');
grid on;

%% 3. 计算水平姿态角
fprintf('\n=== 步骤3: 计算水平姿态角 ===\n');

% 使用加速度计计算横滚角(Roll)和俯仰角(Pitch)
roll_raw = atan2(-acc_y, -acc_z);  % 横滚角
pitch_raw = atan2(acc_x, sqrt(acc_y.^2 + acc_z.^2));  % 俯仰角

% 转换为角度用于显示
roll_raw_deg = rad2deg(roll_raw);
pitch_raw_deg = rad2deg(pitch_raw);

% 绘制原始水平姿态角
subplot(3, 3, 4);
plot(time_sensor, roll_raw_deg, 'b'); hold on;
plot(time_sensor, pitch_raw_deg, 'r');
xlabel('时间 (s)'); ylabel('角度 (deg)');
title('水平姿态角（原始）');
legend('Roll', 'Pitch');
grid on;

%% 4. 平滑水平姿态角
fprintf('\n=== 步骤4: 平滑水平姿态角 ===\n');

% 1秒窗口的滑动平均（采样频率20Hz，窗口大小20个点）
window_size = 20;  % 1秒窗口
roll_smooth = movmean(roll_raw, window_size);
pitch_smooth = movmean(pitch_raw, window_size);

% 转换为角度用于显示
roll_smooth_deg = rad2deg(roll_smooth);
pitch_smooth_deg = rad2deg(pitch_smooth);

% 绘制平滑后的水平姿态角
subplot(3, 3, 5);
plot(time_sensor, roll_smooth_deg, 'b'); hold on;
plot(time_sensor, pitch_smooth_deg, 'r');
xlabel('时间 (s)'); ylabel('角度 (deg)');
title('水平姿态角（平滑后）');
legend('Roll', 'Pitch');
grid on;

% 对比原始和平滑
subplot(3, 3, 6);
plot(time_sensor, roll_raw_deg, 'b:'); hold on;
plot(time_sensor, roll_smooth_deg, 'b-', 'LineWidth', 1.5);
plot(time_sensor, pitch_raw_deg, 'r:');
plot(time_sensor, pitch_smooth_deg, 'r-', 'LineWidth', 1.5);
xlabel('时间 (s)'); ylabel('角度 (deg)');
title('水平姿态角对比');
legend('Roll原始', 'Roll平滑', 'Pitch原始', 'Pitch平滑');
grid on;

%% 5. 使用磁强计计算航向角
fprintf('\n=== 步骤5: 使用磁强计计算航向角 ===\n');

% 磁偏角（给定值）
declination = deg2rad(14.11);  % 转换为弧度

% 初始化磁强计航向数组
heading_mag = zeros(length(time_sensor), 1);

for i = 1:length(time_sensor)
    % 获取当前时刻的水平姿态角
    r = roll_smooth(i);
    theta = pitch_smooth(i);
    
    % 调平磁强计数据：从b系转换到n1系
    % 公式：m_x^{n1} = m_x^b * cosθ + m_y^b * sin r * sinθ + m_z^b * cos r * sinθ
    %        m_y^{n1} = m_y^b * cos r - m_z^b * sin r
    m_x_n1 = mag_x(i) * cos(theta) + ...
             mag_y(i) * sin(r) * sin(theta) + ...
             mag_z(i) * cos(r) * sin(theta);
    
    m_y_n1 = mag_y(i) * cos(r) - mag_z(i) * sin(r);
    
    % 计算磁北航向（注意：atan2(y, x)）
    psi_m_prime = -atan2(m_y_n1, m_x_n1);
    
    % 计算真北航向：加上磁偏角
    heading_mag(i) = psi_m_prime + declination;
    
    % 确保航向在[-pi, pi]范围内
    if heading_mag(i) > pi
        heading_mag(i) = heading_mag(i) - 2*pi;
    elseif heading_mag(i) < -pi
        heading_mag(i) = heading_mag(i) + 2*pi;
    end
end

% 转换为角度用于显示
heading_mag_deg = rad2deg(heading_mag);

%% 6. 使用陀螺计算航向角
fprintf('\n=== 步骤6: 使用陀螺计算航向角 ===\n');

% 初始化陀螺航向数组
heading_gyro = zeros(length(time_sensor), 1);
heading_gyro(1) = deg2rad(-90);  % 初始航向：-90度

% 计算采样间隔
dt = mean(diff(time_sensor));

for i = 2:length(time_sensor)
    % 获取当前时刻的水平姿态角
    r = roll_smooth(i);
    theta = pitch_smooth(i);
    
    % 计算垂向角速度：ω_D = [-sinθ, sin r * cosθ, cos r * cosθ] * ω^b
    % 公式来自第20页
    omega_D = -sin(theta) * gyro_x(i) + ...
              sin(r) * cos(theta) * gyro_y(i) + ...
              cos(r) * cos(theta) * gyro_z(i);
    
    % 积分得到航向变化
    heading_gyro(i) = heading_gyro(i-1) + omega_D * dt;
    
    % 确保航向在[-pi, pi]范围内
    if heading_gyro(i) > pi
        heading_gyro(i) = heading_gyro(i) - 2*pi;
    elseif heading_gyro(i) < -pi
        heading_gyro(i) = heading_gyro(i) + 2*pi;
    end
end

% 转换为角度用于显示
heading_gyro_deg = rad2deg(heading_gyro);

% 绘制航向角对比
subplot(3, 3, 7);
plot(time_sensor, heading_mag_deg, 'b'); hold on;
plot(time_sensor, heading_gyro_deg, 'r');
xlabel('时间 (s)'); ylabel('航向角 (deg)');
title('航向角对比');
legend('磁强计航向', '陀螺航向');
grid on;

%% 7. 脚步探测
fprintf('\n=== 步骤7: 脚步探测 ===\n');

% 计算加速度计模长
acc_norm = sqrt(acc_x.^2 + acc_y.^2 + acc_z.^2);

% 寻找波谷（脚步）
threshold = 9.2;  % 阈值 9.2 m/s^2
min_peak_distance = 10;  % 最小脚步间隔（0.5秒，10个采样点）

% 找到所有低于阈值的点
below_threshold = acc_norm < threshold;

% 初始化脚步检测数组
step_indices = [];
step_times = [];
step_count = 0;

% 简单的波谷检测算法
for i = 2:length(acc_norm)-1
    if below_threshold(i) && ...
       acc_norm(i) < acc_norm(i-1) && ...
       acc_norm(i) < acc_norm(i+1)
       
        % 检查是否与上一个脚步太近
        if isempty(step_indices) || (i - step_indices(end)) > min_peak_distance
            step_indices = [step_indices; i];
            step_times = [step_times; time_sensor(i)];
            step_count = step_count + 1;
        end
    end
end

fprintf('检测到的脚步数: %d\n', step_count);

% 绘制脚步检测结果
subplot(3, 3, 8);
plot(time_sensor, acc_norm, 'b'); hold on;
plot(step_times, acc_norm(step_indices), 'ro', 'MarkerSize', 8, 'LineWidth', 2);
plot([time_sensor(1), time_sensor(end)], [threshold, threshold], 'r--', 'LineWidth', 1.5);
xlabel('时间 (s)'); ylabel('加速度模长 (m/s^2)');
title('脚步检测');
legend('加速度模长', '脚步', '阈值');
grid on;

%% 8. PDR推算
fprintf('\n=== 步骤8: PDR推算 ===\n');

% 步长（常值）
step_length = 0.7;  % 米

% 初始化PDR位置数组
% 使用磁强计航向
pdr_mag_north = zeros(step_count, 1);
pdr_mag_east = zeros(step_count, 1);

% 使用陀螺航向
pdr_gyro_north = zeros(step_count, 1);
pdr_gyro_east = zeros(step_count, 1);

% 初始位置设为[0, 0]
current_north_mag = 0;
current_east_mag = 0;

current_north_gyro = 0;
current_east_gyro = 0;

% 记录每一步的航向
step_headings_mag = zeros(step_count, 1);
step_headings_gyro = zeros(step_count, 1);

for step = 1:step_count
    % 获取当前脚步对应的索引
    idx = step_indices(step);
    
    % 获取当前脚步的航向（使用上一步的航向）
    if step == 1
        % 第一步使用初始航向
        heading_step_mag = heading_mag(idx);
        heading_step_gyro = heading_gyro(idx);
    else
        % 使用上一步的航向
        heading_step_mag = heading_mag(step_indices(step-1));
        heading_step_gyro = heading_gyro(step_indices(step-1));
    end
    
    % 保存航向
    step_headings_mag(step) = heading_step_mag;
    step_headings_gyro(step) = heading_step_gyro;
    
    % 更新位置
    % 北向：N_t = N_{t-1} + step_length * cos(ψ)
    % 东向：E_t = E_{t-1} + step_length * sin(ψ)
    
    % 磁强计航向的PDR
    current_north_mag = current_north_mag + step_length * cos(heading_step_mag);
    current_east_mag = current_east_mag + step_length * sin(heading_step_mag);
    
    pdr_mag_north(step) = current_north_mag;
    pdr_mag_east(step) = current_east_mag;
    
    % 陀螺航向的PDR
    current_north_gyro = current_north_gyro + step_length * cos(heading_step_gyro);
    current_east_gyro = current_east_gyro + step_length * sin(heading_step_gyro);
    
    pdr_gyro_north(step) = current_north_gyro;
    pdr_gyro_east(step) = current_east_gyro;
end

% 绘制PDR结果
subplot(3, 3, 9);
plot(pdr_mag_east, pdr_mag_north, 'b-o', 'LineWidth', 1.5, 'MarkerSize', 4); hold on;
plot(pdr_gyro_east, pdr_gyro_north, 'r-s', 'LineWidth', 1.5, 'MarkerSize', 4);
xlabel('东向 (m)'); ylabel('北向 (m)');
title('PDR轨迹');
legend('磁强计航向PDR', '陀螺航向PDR', 'Location', 'best');
grid on; axis equal;

%% 9. 陀螺零偏补偿
fprintf('\n=== 步骤9: 陀螺零偏补偿 ===\n');

% 尝试对Z轴陀螺加入常值零偏补偿
% 由于手机端平，Z轴接近垂向
gyro_z_calibrated = gyro_z - deg2rad(0.5);  % 假设零偏为0.5 deg/s

% 重新计算陀螺航向
heading_gyro_calibrated = zeros(length(time_sensor), 1);
heading_gyro_calibrated(1) = deg2rad(-90);  % 初始航向

for i = 2:length(time_sensor)
    % 获取当前时刻的水平姿态角
    r = roll_smooth(i);
    theta = pitch_smooth(i);
    
    % 使用校准后的陀螺数据
    omega_D_calibrated = -sin(theta) * gyro_x(i) + ...
                         sin(r) * cos(theta) * gyro_y(i) + ...
                         cos(r) * cos(theta) * gyro_z_calibrated(i);
    
    % 积分得到航向变化
    heading_gyro_calibrated(i) = heading_gyro_calibrated(i-1) + omega_D_calibrated * dt;
    
    % 确保航向在[-pi, pi]范围内
    if heading_gyro_calibrated(i) > pi
        heading_gyro_calibrated(i) = heading_gyro_calibrated(i) - 2*pi;
    elseif heading_gyro_calibrated(i) < -pi
        heading_gyro_calibrated(i) = heading_gyro_calibrated(i) + 2*pi;
    end
end

% 使用校准后的陀螺航向进行PDR
pdr_gyro_calibrated_north = zeros(step_count, 1);
pdr_gyro_calibrated_east = zeros(step_count, 1);

current_north_calibrated = 0;
current_east_calibrated = 0;

for step = 1:step_count
    idx = step_indices(step);
    
    if step == 1
        heading_step_calibrated = heading_gyro_calibrated(idx);
    else
        heading_step_calibrated = heading_gyro_calibrated(step_indices(step-1));
    end
    
    current_north_calibrated = current_north_calibrated + step_length * cos(heading_step_calibrated);
    current_east_calibrated = current_east_calibrated + step_length * sin(heading_step_calibrated);
    
    pdr_gyro_calibrated_north(step) = current_north_calibrated;
    pdr_gyro_calibrated_east(step) = current_east_calibrated;
end

%% 10. 创建单独的轨迹图
figure('Position', [100, 100, 800, 600]);

% 绘制所有PDR轨迹对比
subplot(2, 2, 1);
plot(pdr_mag_east, pdr_mag_north, 'b-o', 'LineWidth', 1.5, 'MarkerSize', 4); hold on;
plot(pdr_gyro_east, pdr_gyro_north, 'r-s', 'LineWidth', 1.5, 'MarkerSize', 4);
plot(pdr_gyro_calibrated_east, pdr_gyro_calibrated_north, 'g-d', 'LineWidth', 1.5, 'MarkerSize', 4);
xlabel('东向 (m)'); ylabel('北向 (m)');
title('PDR轨迹对比');
legend('磁强计航向', '陀螺航向', '陀螺校准后', 'Location', 'best');
grid on; axis equal;

% 绘制航向角对比图
subplot(2, 2, 2);
plot(time_sensor, heading_mag_deg, 'b', 'LineWidth', 1.5); hold on;
plot(time_sensor, rad2deg(heading_gyro), 'r', 'LineWidth', 1.5);
plot(time_sensor, rad2deg(heading_gyro_calibrated), 'g', 'LineWidth', 1.5);
xlabel('时间 (s)'); ylabel('航向角 (deg)');
title('航向角对比');
legend('磁强计', '陀螺', '陀螺校准后');
grid on;

% 绘制水平姿态角
subplot(2, 2, 3);
plot(time_sensor, roll_smooth_deg, 'b', 'LineWidth', 1.5); hold on;
plot(time_sensor, pitch_smooth_deg, 'r', 'LineWidth', 1.5);
xlabel('时间 (s)'); ylabel('角度 (deg)');
title('水平姿态角（平滑后）');
legend('Roll', 'Pitch');
grid on;

% 绘制脚步检测
subplot(2, 2, 4);
plot(time_sensor, acc_norm, 'b', 'LineWidth', 1.5); hold on;
plot(step_times, acc_norm(step_indices), 'ro', 'MarkerSize', 8, 'LineWidth', 2);
plot([time_sensor(1), time_sensor(end)], [threshold, threshold], 'r--', 'LineWidth', 1.5);
xlabel('时间 (s)'); ylabel('加速度模长 (m/s^2)');
title(sprintf('脚步检测（共 %d 步）', step_count));
legend('加速度模长', '脚步', '阈值');
grid on;

%% 11. 计算轨迹误差统计
fprintf('\n=== 步骤10: 轨迹误差分析 ===\n');

% 由于没有真实轨迹，我们只能比较不同方法之间的差异
% 计算轨迹长度
traj_length_mag = sum(sqrt(diff(pdr_mag_east).^2 + diff(pdr_mag_north).^2));
traj_length_gyro = sum(sqrt(diff(pdr_gyro_east).^2 + diff(pdr_gyro_north).^2));
traj_length_calibrated = sum(sqrt(diff(pdr_gyro_calibrated_east).^2 + diff(pdr_gyro_calibrated_north).^2));

fprintf('磁强计PDR轨迹长度: %.2f m\n', traj_length_mag);
fprintf('陀螺PDR轨迹长度: %.2f m\n', traj_length_gyro);
fprintf('陀螺校准后PDR轨迹长度: %.2f m\n', traj_length_calibrated);
fprintf('理论轨迹长度: %.2f m (步数×步长)\n', step_count * step_length);

% 计算终点位置
fprintf('\n终点位置对比:\n');
fprintf('磁强计PDR终点: (北向=%.2f m, 东向=%.2f m)\n', pdr_mag_north(end), pdr_mag_east(end));
fprintf('陀螺PDR终点: (北向=%.2f m, 东向=%.2f m)\n', pdr_gyro_north(end), pdr_gyro_east(end));
fprintf('陀螺校准后PDR终点: (北向=%.2f m, 东向=%.2f m)\n', pdr_gyro_calibrated_north(end), pdr_gyro_calibrated_east(end));

%% 12. 保存结果
save('pdr_results.mat', 'time_sensor', 'step_times', 'step_count', ...
     'pdr_mag_north', 'pdr_mag_east', ...
     'pdr_gyro_north', 'pdr_gyro_east', ...
     'pdr_gyro_calibrated_north', 'pdr_gyro_calibrated_east', ...
     'heading_mag_deg', 'heading_gyro_deg', 'roll_smooth_deg', 'pitch_smooth_deg');

fprintf('\n=== PDR实现完成 ===\n');