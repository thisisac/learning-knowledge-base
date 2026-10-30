%% 室内几何定位系统 - 基于距离交会
% 作者：李由，武汉大学
% 日期：2025/9/26

clear all; close all; clc;

%% 1. 读取基站坐标
fprintf('=== 读取基站坐标 ===\n');
base_stations = readBaseStations('base-station-location.txt');
fprintf('成功读取 %d 个基站坐标\n', size(base_stations, 1));

%% 2. 读取测距数据和真实位置
fprintf('\n=== 读取测距数据 ===\n');

% 读取Tag2数据
[tag2_time, tag2_ranges] = readRangeData('range_tag_2.txt');
[tag2_true_time, tag2_true_pos] = readTruePosition('true_position_tag_2.txt');

% 读取Tag3数据  
[tag3_time, tag3_ranges] = readRangeData('range_tag_3.txt');
[tag3_true_time, tag3_true_pos] = readTruePosition('true_position_tag_3.txt');

fprintf('Tag2: %d 个测距时刻\n', length(tag2_time));
fprintf('Tag3: %d 个测距时刻\n', length(tag3_time));

%% 3. 参数设置
% 测距精度（米）
range_accuracy = 0.3;
% 权重矩阵（假设所有测量精度相同）
R = diag(ones(size(base_stations, 1), 1) * range_accuracy^2);
W = inv(R);

% 最大迭代次数
max_iter = 100;
% 收敛阈值
convergence_threshold = 1e-6;

%% 4. 对Tag2进行定位计算
fprintf('\n=== 对Tag2进行定位计算 ===\n');
tag2_estimated_3d = zeros(length(tag2_time), 3);
tag2_estimated_2d = zeros(length(tag2_time), 3); % 第三维用固定高度
tag2_errors_3d = zeros(length(tag2_time), 1);
tag2_errors_2d = zeros(length(tag2_time), 1);

% 获取Tag2的平均高度用于二维定位
tag2_mean_height = mean(tag2_true_pos(:, 3));

for i = 1:length(tag2_time)
    ranges = tag2_ranges(i, :);
    
    % 三维定位
    [pos_3d, converged] = trilateration3D(base_stations, ranges, max_iter, convergence_threshold, W);
    tag2_estimated_3d(i, :) = pos_3d;
    
    % 二维定位（固定高度）
    [pos_2d, converged] = trilateration2D(base_stations, ranges, tag2_mean_height, max_iter, convergence_threshold, W);
    tag2_estimated_2d(i, :) = [pos_2d, tag2_mean_height];
    
    % 计算误差（如果有真实位置）
    if i <= size(tag2_true_pos, 1)
        true_pos = tag2_true_pos(i, :);
        tag2_errors_3d(i) = norm(pos_3d - true_pos);
        tag2_errors_2d(i) = norm([pos_2d, tag2_mean_height] - true_pos);
    end
    
    if mod(i, 10) == 0
        fprintf('处理Tag2第 %d/%d 个时刻...\n', i, length(tag2_time));
    end
end

% 计算统计指标
tag2_rmse_3d = sqrt(mean(tag2_errors_3d(1:size(tag2_true_pos,1)).^2));
tag2_rmse_2d = sqrt(mean(tag2_errors_2d(1:size(tag2_true_pos,1)).^2));
tag2_mean_error_3d = mean(tag2_errors_3d(1:size(tag2_true_pos,1)));
tag2_mean_error_2d = mean(tag2_errors_2d(1:size(tag2_true_pos,1)));

fprintf('Tag2 三维定位 - RMSE: %.4f 米, 平均误差: %.4f 米\n', tag2_rmse_3d, tag2_mean_error_3d);
fprintf('Tag2 二维定位 - RMSE: %.4f 米, 平均误差: %.4f 米\n', tag2_rmse_2d, tag2_mean_error_2d);

%% 5. 对Tag3进行定位计算
fprintf('\n=== 对Tag3进行定位计算 ===\n');
tag3_estimated_3d = zeros(length(tag3_time), 3);
tag3_estimated_2d = zeros(length(tag3_time), 3);
tag3_errors_3d = zeros(length(tag3_time), 1);
tag3_errors_2d = zeros(length(tag3_time), 1);

% 获取Tag3的平均高度用于二维定位
tag3_mean_height = mean(tag3_true_pos(:, 3));

for i = 1:length(tag3_time)
    ranges = tag3_ranges(i, :);
    
    % 三维定位
    [pos_3d, converged] = trilateration3D(base_stations, ranges, max_iter, convergence_threshold, W);
    tag3_estimated_3d(i, :) = pos_3d;
    
    % 二维定位（固定高度）
    [pos_2d, converged] = trilateration2D(base_stations, ranges, tag3_mean_height, max_iter, convergence_threshold, W);
    tag3_estimated_2d(i, :) = [pos_2d, tag3_mean_height];
    
    % 计算误差
    if i <= size(tag3_true_pos, 1)
        true_pos = tag3_true_pos(i, :);
        tag3_errors_3d(i) = norm(pos_3d - true_pos);
        tag3_errors_2d(i) = norm([pos_2d, tag3_mean_height] - true_pos);
    end
    
    if mod(i, 10) == 0
        fprintf('处理Tag3第 %d/%d 个时刻...\n', i, length(tag3_time));
    end
end

% 计算统计指标
tag3_rmse_3d = sqrt(mean(tag3_errors_3d(1:size(tag3_true_pos,1)).^2));
tag3_rmse_2d = sqrt(mean(tag3_errors_2d(1:size(tag3_true_pos,1)).^2));
tag3_mean_error_3d = mean(tag3_errors_3d(1:size(tag3_true_pos,1)));
tag3_mean_error_2d = mean(tag3_errors_2d(1:size(tag3_true_pos,1)));

fprintf('Tag3 三维定位 - RMSE: %.4f 米, 平均误差: %.4f 米\n', tag3_rmse_3d, tag3_mean_error_3d);
fprintf('Tag3 二维定位 - RMSE: %.4f 米, 平均误差: %.4f 米\n', tag3_rmse_2d, tag3_mean_error_2d);

%% 6. 可视化结果
fprintf('\n=== 生成可视化结果 ===\n');

% 图1: Tag2 定位结果对比
figure('Position', [100, 100, 1200, 800]);

% 三维轨迹
subplot(2,3,1);
plot3(tag2_estimated_3d(:,1), tag2_estimated_3d(:,2), tag2_estimated_3d(:,3), 'b-', 'LineWidth', 1.5);
hold on;
plot3(tag2_true_pos(:,1), tag2_true_pos(:,2), tag2_true_pos(:,3), 'r--', 'LineWidth', 1.5);
plot3(base_stations(:,1), base_stations(:,2), base_stations(:,3), 'k^', 'MarkerSize', 10, 'MarkerFaceColor', 'k');
grid on; box on;
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
title('Tag2 三维定位轨迹');
legend('估计位置', '真实位置', '基站', 'Location', 'best');
view(3);

% 二维轨迹（XY平面）
subplot(2,3,2);
plot(tag2_estimated_2d(:,1), tag2_estimated_2d(:,2), 'b-', 'LineWidth', 1.5);
hold on;
plot(tag2_true_pos(:,1), tag2_true_pos(:,2), 'r--', 'LineWidth', 1.5);
plot(base_stations(:,1), base_stations(:,2), 'k^', 'MarkerSize', 10, 'MarkerFaceColor', 'k');
grid on; box on;
xlabel('X (m)'); ylabel('Y (m)');
title('Tag2 二维定位轨迹 (XY平面)');
legend('估计位置', '真实位置', '基站', 'Location', 'best');
axis equal;

% 误差随时间变化
subplot(2,3,3);
time_indices = 1:min(length(tag2_time), size(tag2_true_pos,1));
plot(time_indices, tag2_errors_3d(time_indices), 'b-', 'LineWidth', 1.5);
hold on;
plot(time_indices, tag2_errors_2d(time_indices), 'r-', 'LineWidth', 1.5);
grid on; box on;
xlabel('时间索引'); ylabel('定位误差 (m)');
title('Tag2 定位误差');
legend(['3D (平均: ', sprintf('%.3f', tag2_mean_error_3d), ' m)'], ...
       ['2D (平均: ', sprintf('%.3f', tag2_mean_error_2d), ' m)'], 'Location', 'best');

% 误差分布直方图
subplot(2,3,4);
histogram(tag2_errors_3d(1:size(tag2_true_pos,1)), 20, 'FaceColor', 'b', 'EdgeColor', 'k');
hold on;
histogram(tag2_errors_2d(1:size(tag2_true_pos,1)), 20, 'FaceColor', 'r', 'EdgeColor', 'k', 'FaceAlpha', 0.5);
grid on; box on;
xlabel('定位误差 (m)'); ylabel('频次');
title('Tag2 误差分布');

% 高程对比
subplot(2,3,5);
plot(time_indices, tag2_estimated_3d(time_indices,3), 'b-', 'LineWidth', 1.5);
hold on;
plot(time_indices, tag2_true_pos(time_indices,3), 'r--', 'LineWidth', 1.5);
plot(time_indices, ones(size(time_indices))*tag2_mean_height, 'g-.', 'LineWidth', 1.5);
grid on; box on;
xlabel('时间索引'); ylabel('高程 (m)');
title('Tag2 高程估计');
legend('3D估计高程', '真实高程', '2D固定高程', 'Location', 'best');

% 误差椭圆示例（最后一个时刻）
subplot(2,3,6);
plotErrorEllipse(tag2_estimated_3d(end,:), base_stations, tag2_ranges(end,:), W);
title('Tag2 最后时刻误差椭圆');

%% 图2: Tag3 定位结果对比
figure('Position', [100, 100, 1200, 800]);

% 三维轨迹
subplot(2,3,1);
plot3(tag3_estimated_3d(:,1), tag3_estimated_3d(:,2), tag3_estimated_3d(:,3), 'b-', 'LineWidth', 1.5);
hold on;
plot3(tag3_true_pos(:,1), tag3_true_pos(:,2), tag3_true_pos(:,3), 'r--', 'LineWidth', 1.5);
plot3(base_stations(:,1), base_stations(:,2), base_stations(:,3), 'k^', 'MarkerSize', 10, 'MarkerFaceColor', 'k');
grid on; box on;
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
title('Tag3 三维定位轨迹');
legend('估计位置', '真实位置', '基站', 'Location', 'best');
view(3);

% 二维轨迹（XY平面）
subplot(2,3,2);
plot(tag3_estimated_2d(:,1), tag3_estimated_2d(:,2), 'b-', 'LineWidth', 1.5);
hold on;
plot(tag3_true_pos(:,1), tag3_true_pos(:,2), 'r--', 'LineWidth', 1.5);
plot(base_stations(:,1), base_stations(:,2), 'k^', 'MarkerSize', 10, 'MarkerFaceColor', 'k');
grid on; box on;
xlabel('X (m)'); ylabel('Y (m)');
title('Tag3 二维定位轨迹 (XY平面)');
legend('估计位置', '真实位置', '基站', 'Location', 'best');
axis equal;

% 误差随时间变化
subplot(2,3,3);
time_indices = 1:min(length(tag3_time), size(tag3_true_pos,1));
plot(time_indices, tag3_errors_3d(time_indices), 'b-', 'LineWidth', 1.5);
hold on;
plot(time_indices, tag3_errors_2d(time_indices), 'r-', 'LineWidth', 1.5);
grid on; box on;
xlabel('时间索引'); ylabel('定位误差 (m)');
title('Tag3 定位误差');
legend(['3D (平均: ', sprintf('%.3f', tag3_mean_error_3d), ' m)'], ...
       ['2D (平均: ', sprintf('%.3f', tag3_mean_error_2d), ' m)'], 'Location', 'best');

% 误差分布直方图
subplot(2,3,4);
histogram(tag3_errors_3d(1:size(tag3_true_pos,1)), 20, 'FaceColor', 'b', 'EdgeColor', 'k');
hold on;
histogram(tag3_errors_2d(1:size(tag3_true_pos,1)), 20, 'FaceColor', 'r', 'EdgeColor', 'k', 'FaceAlpha', 0.5);
grid on; box on;
xlabel('定位误差 (m)'); ylabel('频次');
title('Tag3 误差分布');

% 高程对比
subplot(2,3,5);
plot(time_indices, tag3_estimated_3d(time_indices,3), 'b-', 'LineWidth', 1.5);
hold on;
plot(time_indices, tag3_true_pos(time_indices,3), 'r--', 'LineWidth', 1.5);
plot(time_indices, ones(size(time_indices))*tag3_mean_height, 'g-.', 'LineWidth', 1.5);
grid on; box on;
xlabel('时间索引'); ylabel('高程 (m)');
title('Tag3 高程估计');
legend('3D估计高程', '真实高程', '2D固定高程', 'Location', 'best');

% 误差椭圆示例（最后一个时刻）
subplot(2,3,6);
plotErrorEllipse(tag3_estimated_3d(end,:), base_stations, tag3_ranges(end,:), W);
title('Tag3 最后时刻误差椭圆');

%% 图3: 综合对比
figure('Position', [100, 100, 1400, 500]);

% 累积分布函数(CDF)对比
subplot(1,3,1);
[F3d, X3d] = ecdf(tag2_errors_3d(1:size(tag2_true_pos,1)));
[F2d, X2d] = ecdf(tag2_errors_2d(1:size(tag2_true_pos,1)));
plot(X3d, F3d, 'b-', 'LineWidth', 2);
hold on;
plot(X2d, F2d, 'r-', 'LineWidth', 2);
grid on; box on;
xlabel('定位误差 (m)'); ylabel('累积概率');
title('Tag2 定位误差CDF');
legend('三维定位', '二维定位', 'Location', 'southeast');

% 性能统计
subplot(1,3,2);
methods = {'3D Tag2', '2D Tag2', '3D Tag3', '2D Tag3'};
rmse_values = [tag2_rmse_3d, tag2_rmse_2d, tag3_rmse_3d, tag3_rmse_2d];
bar(rmse_values, 'FaceColor', [0.2 0.6 0.8]);
grid on; box on;
ylabel('RMSE (m)');
title('不同方法RMSE对比');
set(gca, 'XTickLabel', methods);

% 基站几何分布可视化
subplot(1,3,3);
scatter3(base_stations(:,1), base_stations(:,2), base_stations(:,3), 100, 'r', 'filled');
hold on;
text(base_stations(:,1), base_stations(:,2), base_stations(:,3), ...
     cellstr(num2str((1:size(base_stations,1))')), ...
     'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'right', 'FontSize', 10);
grid on; box on;
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
title('基站几何分布');
axis equal;
view(45, 30);

fprintf('\n=== 定位计算完成 ===\n');
fprintf('总结:\n');
fprintf('  Tag2 - 三维定位: RMSE = %.4f m, 平均误差 = %.4f m\n', tag2_rmse_3d, tag2_mean_error_3d);
fprintf('  Tag2 - 二维定位: RMSE = %.4f m, 平均误差 = %.4f m\n', tag2_rmse_2d, tag2_mean_error_2d);
fprintf('  Tag3 - 三维定位: RMSE = %.4f m, 平均误差 = %.4f m\n', tag3_rmse_3d, tag3_mean_error_3d);
fprintf('  Tag3 - 二维定位: RMSE = %.4f m, 平均误差 = %.4f m\n', tag3_rmse_2d, tag3_mean_error_2d);

%% 辅助函数定义

function stations = readBaseStations(filename)
    % 读取基站坐标文件
    fid = fopen(filename, 'r');
    if fid == -1
        error('无法打开基站坐标文件: %s', filename);
    end
    
    data = textscan(fid, '%f%f%f%f');
    fclose(fid);
    
    stations = [data{2}, data{3}, data{4}];
end

function [time, ranges] = readRangeData(filename)
    % 读取测距数据文件
    fid = fopen(filename, 'r');
    if fid == -1
        error('无法打开测距数据文件: %s', filename);
    end
    
    % 读取所有数据
    data = textscan(fid, '%f%f%f%f%f%f%f%f%f');
    fclose(fid);
    
    time = data{1};
    ranges = [data{2}, data{3}, data{4}, data{5}, data{6}, data{7}, data{8}, data{9}];
end

function [time, positions] = readTruePosition(filename)
    % 读取真实位置文件
    fid = fopen(filename, 'r');
    if fid == -1
        error('无法打开真实位置文件: %s', filename);
    end
    
    % 读取所有数据
    data = textscan(fid, '%f%f%f%f');
    fclose(fid);
    
    time = data{1};
    positions = [data{2}, data{3}, data{4}];
end

function [position, converged] = trilateration3D(stations, ranges, max_iter, threshold, W)
    % 三维距离交会定位算法
    % stations: 基站坐标 (n×3)
    % ranges: 到各基站的距离 (1×n)
    % max_iter: 最大迭代次数
    % threshold: 收敛阈值
    % W: 权重矩阵
    
    n_stations = size(stations, 1);
    
    % 初始位置估计：使用所有基站坐标的加权质心
    position = mean(stations, 1);
    
    converged = false;
    
    for iter = 1:max_iter
        % 计算预测距离
        predicted_ranges = sqrt(sum((stations - position).^2, 2));
        
        % 计算残差
        residuals = ranges' - predicted_ranges;
        
        % 构建设计矩阵 H
        H = zeros(n_stations, 3);
        for i = 1:n_stations
            if predicted_ranges(i) > 0
                H(i, :) = -(stations(i, :) - position) / predicted_ranges(i);
            else
                H(i, :) = [0, 0, 0];
            end
        end
        
        % 最小二乘解
        delta = (H' * W * H) \ (H' * W * residuals);
        
        % 更新位置
        position = position + delta';
        
        % 检查收敛
        if norm(delta) < threshold
            converged = true;
            break;
        end
    end
end

function [position_2d, converged] = trilateration2D(stations, ranges, fixed_height, max_iter, threshold, W)
    % 二维距离交会定位算法（固定高度）
    % stations: 基站坐标 (n×3)
    % ranges: 到各基站的距离 (1×n)
    % fixed_height: 固定高度
    % max_iter: 最大迭代次数
    % threshold: 收敛阈值
    % W: 权重矩阵
    
    n_stations = size(stations, 1);
    
    % 初始位置估计：使用所有基站坐标的加权质心（仅XY）
    position_2d = mean(stations(:, 1:2), 1);
    
    converged = false;
    
    for iter = 1:max_iter
        % 计算预测距离（考虑固定高度）
        delta_z = fixed_height - stations(:, 3);
        horizontal_dist_sq = sum((stations(:, 1:2) - position_2d).^2, 2);
        predicted_ranges = sqrt(horizontal_dist_sq + delta_z.^2);
        
        % 计算残差
        residuals = ranges' - predicted_ranges;
        
        % 构建设计矩阵 H (仅XY方向)
        H = zeros(n_stations, 2);
        for i = 1:n_stations
            if predicted_ranges(i) > 0
                H(i, :) = -(stations(i, 1:2) - position_2d) / predicted_ranges(i);
            else
                H(i, :) = [0, 0];
            end
        end
        
        % 最小二乘解
        delta = (H' * W * H) \ (H' * W * residuals);
        
        % 更新位置
        position_2d = position_2d + delta';
        
        % 检查收敛
        if norm(delta) < threshold
            converged = true;
            break;
        end
    end
end

function plotErrorEllipse(position, stations, ranges, W)
    % 绘制误差椭圆
    % position: 估计位置
    % stations: 基站坐标
    % ranges: 距离测量值
    % W: 权重矩阵
    
    n_stations = size(stations, 1);
    
    % 计算预测距离
    predicted_ranges = sqrt(sum((stations - position).^2, 2));
    
    % 构建设计矩阵 H
    H = zeros(n_stations, 3);
    for i = 1:n_stations
        if predicted_ranges(i) > 0
            H(i, :) = -(stations(i, :) - position) / predicted_ranges(i);
        else
            H(i, :) = [0, 0, 0];
        end
    end
    
    % 仅取XY部分进行二维误差椭圆计算
    H_2d = H(:, 1:2);
    
    % 计算协方差矩阵
    R_inv = inv(W); % 测量噪声协方差矩阵的逆
    P = inv(H_2d' * inv(R_inv) * H_2d);
    
    % 提取方差和协方差
    sigma_E2 = P(1, 1);
    sigma_N2 = P(2, 2);
    sigma_EN = P(1, 2);
    
    % 计算误差椭圆参数
    a1 = sqrt(0.5*(sigma_E2 + sigma_N2) + sqrt(0.25*(sigma_E2 - sigma_N2)^2 + sigma_EN^2));
    a2 = sqrt(0.5*(sigma_E2 + sigma_N2) - sqrt(0.25*(sigma_E2 - sigma_N2)^2 + sigma_EN^2));
    
    % 计算方向角（弧度）
    if sigma_N2 ~= sigma_E2
        psi = 0.5 * atan2(2*sigma_EN, sigma_N2 - sigma_E2);
    else
        psi = pi/4 * sign(sigma_EN);
    end
    
    % 生成椭圆点
    theta = linspace(0, 2*pi, 100);
    ellipse_x = a1 * cos(theta) * cos(psi) - a2 * sin(theta) * sin(psi);
    ellipse_y = a1 * cos(theta) * sin(psi) + a2 * sin(theta) * cos(psi);
    
    % 绘制椭圆（以估计位置为中心）
    plot(position(1) + ellipse_x, position(2) + ellipse_y, 'b-', 'LineWidth', 1.5);
    hold on;
    
    % 绘制估计位置
    plot(position(1), position(2), 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
    
    % 绘制主轴
    plot([position(1), position(1) + a1*cos(psi)], ...
         [position(2), position(2) + a1*sin(psi)], 'r-', 'LineWidth', 2);
    plot([position(1), position(1) + a2*cos(psi+pi/2)], ...
         [position(2), position(2) + a2*sin(psi+pi/2)], 'g-', 'LineWidth', 2);
    
    grid on; box on;
    xlabel('X (m)'); ylabel('Y (m)');
    axis equal;
    
    % 添加图例和标题
    legend('误差椭圆', '估计位置', '长轴', '短轴', 'Location', 'best');
    
    % 显示椭圆参数
    text(0.02, 0.98, sprintf('长轴: %.3f m\n短轴: %.3f m\n方向角: %.1f°', ...
        a1, a2, rad2deg(psi)), ...
        'Units', 'normalized', 'VerticalAlignment', 'top', ...
        'BackgroundColor', 'w', 'EdgeColor', 'k');
end