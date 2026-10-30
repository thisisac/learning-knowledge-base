%% 室内定位指纹匹配算法实现
% 作者：刘峻琰
% 日期：2024年12月2日

clear; clc; close all;

%% 1. 读取数据文件
fprintf('正在读取数据...\n');

% 读取参考点坐标文件
rp_data = dlmread('rp_loc.txt');
rp_ids = rp_data(:, 1);  % 参考点ID
rp_coords = rp_data(:, 2:3);  % 参考点坐标(x,y)
num_rps = length(rp_ids);

% 读取测试数据文件
test_data = dlmread('test1_with_ref.data');
num_tests = size(test_data, 1);
test_rss = test_data(:, 2:31);  % 30个AP的RSS值
test_time = test_data(:, 1);    % 时间戳
test_ref_coords = test_data(:, 32:33);  % 参考坐标(用于验证)

% 读取指纹数据库文件
database = [];
fid = fopen('database.data', 'r');
while ~feof(fid)
    line = fgetl(fid);
    if isempty(line) || ~ischar(line)
        continue;
    end
    
    % 解析每行数据
    values = sscanf(line, '%f');
    if length(values) >= 31
        rp_id = values(1);
        rss_values = values(2:31)';
        database = [database; rp_id, rss_values];
    end
end
fclose(fid);

fprintf('数据读取完成！\n');
fprintf('参考点数量: %d\n', num_rps);
fprintf('测试点数量: %d\n', num_tests);
fprintf('指纹数据库大小: %d x %d\n', size(database, 1), size(database, 2));

%% 2. 预处理信号强度数据
fprintf('\n正在预处理信号强度数据...\n');

% 将-110替换为NaN（表示无信号）
test_rss(test_rss == -110) = NaN;
database_rss = database(:, 2:end);
database_rss(database_rss == -110) = NaN;

% 创建指纹数据库结构
fingerprint_db = struct();
unique_rp_ids = unique(database(:, 1));

for i = 1:length(unique_rp_ids)
    rp_id = unique_rp_ids(i);
    
    % 找到该参考点的所有指纹
    idx = database(:, 1) == rp_id;
    fingerprints = database_rss(idx, :);
    
    % 计算平均指纹
    avg_fingerprint = nanmean(fingerprints, 1);
    
    % 找到对应的坐标
    coord_idx = find(rp_ids == rp_id, 1);
    if ~isempty(coord_idx)
        fingerprint_db(i).rp_id = rp_id;
        fingerprint_db(i).coord = rp_coords(coord_idx, :);
        fingerprint_db(i).fingerprints = fingerprints;
        fingerprint_db(i).avg_fingerprint = avg_fingerprint;
        fingerprint_db(i).std_fingerprint = nanstd(fingerprints, 0, 1);
    end
end

num_fingerprints = length(fingerprint_db);
fprintf('有效的指纹参考点数量: %d\n', num_fingerprints);

%% 3. 实现不同的指纹匹配算法

% 3.1 K最近邻算法(KNN)
function estimated_coord = knn_localization(test_rss, fingerprint_db, K)
    % 计算测试点与所有指纹的距离
    distances = zeros(length(fingerprint_db), 1);
    
    for i = 1:length(fingerprint_db)
        % 使用欧氏距离（仅考虑有效信号）
        valid_idx = ~isnan(test_rss) & ~isnan(fingerprint_db(i).avg_fingerprint);
        
        if sum(valid_idx) > 0
            diff = test_rss(valid_idx) - fingerprint_db(i).avg_fingerprint(valid_idx);
            distances(i) = sqrt(sum(diff.^2));
        else
            distances(i) = Inf;
        end
    end
    
    % 找到K个最近邻
    [sorted_dist, sorted_idx] = sort(distances);
    k_nearest = sorted_idx(1:min(K, length(sorted_idx)));
    
    % 计算加权平均坐标（权重为距离的倒数）
    weights = 1 ./ (sorted_dist(1:length(k_nearest)) + eps);
    weights = weights / sum(weights);
    
    estimated_coord = zeros(1, 2);
    for j = 1:length(k_nearest)
        idx = k_nearest(j);
        estimated_coord = estimated_coord + weights(j) * fingerprint_db(idx).coord;
    end
end

% 3.2 加权K最近邻算法(WKNN)
function estimated_coord = wknn_localization(test_rss, fingerprint_db, K)
    % 计算测试点与所有指纹的距离
    distances = zeros(length(fingerprint_db), 1);
    
    for i = 1:length(fingerprint_db)
        % 使用欧氏距离
        valid_idx = ~isnan(test_rss) & ~isnan(fingerprint_db(i).avg_fingerprint);
        
        if sum(valid_idx) > 0
            diff = test_rss(valid_idx) - fingerprint_db(i).avg_fingerprint(valid_idx);
            distances(i) = sqrt(sum(diff.^2));
        else
            distances(i) = Inf;
        end
    end
    
    % 找到K个最近邻
    [sorted_dist, sorted_idx] = sort(distances);
    k_nearest = sorted_idx(1:min(K, length(sorted_idx)));
    
    % 使用距离的倒数作为权重
    weights = 1 ./ (sorted_dist(1:length(k_nearest)).^2 + eps);
    weights = weights / sum(weights);
    
    estimated_coord = zeros(1, 2);
    for j = 1:length(k_nearest)
        idx = k_nearest(j);
        estimated_coord = estimated_coord + weights(j) * fingerprint_db(idx).coord;
    end
end

% 3.3 概率定位算法
function estimated_coord = probabilistic_localization(test_rss, fingerprint_db)
    % 使用概率模型进行定位
    probabilities = zeros(length(fingerprint_db), 1);
    
    for i = 1:length(fingerprint_db)
        % 假设每个AP的RSS服从高斯分布
        prob = 1;
        
        for ap = 1:length(test_rss)
            if ~isnan(test_rss(ap)) && ~isnan(fingerprint_db(i).avg_fingerprint(ap))
                mu = fingerprint_db(i).avg_fingerprint(ap);
                sigma = max(fingerprint_db(i).std_fingerprint(ap), 1);  % 防止标准差为0
                
                % 计算高斯概率密度
                prob = prob * (1/(sigma*sqrt(2*pi))) * exp(-0.5*((test_rss(ap)-mu)/sigma)^2);
            end
        end
        
        probabilities(i) = prob;
    end
    
    % 归一化概率
    probabilities = probabilities / sum(probabilities);
    
    % 使用最大后验概率或加权平均
    [~, max_idx] = max(probabilities);
    estimated_coord = fingerprint_db(max_idx).coord;
    
    % 或者使用加权平均
    % estimated_coord = zeros(1, 2);
    % for i = 1:length(fingerprint_db)
    %     estimated_coord = estimated_coord + probabilities(i) * fingerprint_db(i).coord;
    % end
end

%% 4. 执行定位测试
fprintf('\n执行指纹定位测试...\n');

% 设置参数
K_values = [1, 3, 5];  % 测试不同的K值
methods = {'KNN', 'WKNN', 'Probabilistic'};
results = struct();

for method_idx = 1:length(methods)
    method_name = methods{method_idx};
    fprintf('\n使用方法: %s\n', method_name);
    
    for k_idx = 1:length(K_values)
        K = K_values(k_idx);
        fprintf('  K = %d: ', K);
        
        estimated_coords = zeros(num_tests, 2);
        errors = zeros(num_tests, 1);
        
        for test_idx = 1:num_tests
            test_point_rss = test_rss(test_idx, :);
            
            % 根据方法选择定位函数
            switch method_name
                case 'KNN'
                    estimated_coords(test_idx, :) = knn_localization(test_point_rss, fingerprint_db, K);
                case 'WKNN'
                    estimated_coords(test_idx, :) = wknn_localization(test_point_rss, fingerprint_db, K);
                case 'Probabilistic'
                    estimated_coords(test_idx, :) = probabilistic_localization(test_point_rss, fingerprint_db);
            end
            
            % 计算定位误差
            if ~isnan(test_ref_coords(test_idx, 1))
                errors(test_idx) = norm(estimated_coords(test_idx, :) - test_ref_coords(test_idx, :));
            else
                errors(test_idx) = NaN;
            end
        end
        
        % 计算统计信息
        valid_errors = errors(~isnan(errors));
        mean_error = mean(valid_errors);
        median_error = median(valid_errors);
        std_error = std(valid_errors);
        rmse = sqrt(mean(valid_errors.^2));
        
        fprintf('平均误差 = %.2f m, RMSE = %.2f m, 标准差 = %.2f m\n', ...
                mean_error, rmse, std_error);
        
        % 保存结果
        field_name = sprintf('%s_K%d', method_name, K);
        results.(field_name).estimated_coords = estimated_coords;
        results.(field_name).errors = errors;
        results.(field_name).mean_error = mean_error;
        results.(field_name).rmse = rmse;
        results.(field_name).std_error = std_error;
    end
end

%% 5. 可视化结果
fprintf('\n生成可视化结果...\n');

% 5.1 绘制参考点分布
figure('Position', [100, 100, 1200, 800]);

subplot(2, 3, 1);
scatter(rp_coords(:, 1), rp_coords(:, 2), 50, 'b', 'filled');
hold on;
scatter(test_ref_coords(:, 1), test_ref_coords(:, 2), 30, 'r', 'filled');
xlabel('X坐标 (m)');
ylabel('Y坐标 (m)');
title('参考点和测试点分布');
legend('参考点', '测试点', 'Location', 'best');
grid on;
axis equal;

% 5.2 绘制信号强度热图（第一个AP）
subplot(2, 3, 2);
% 创建网格用于插值
[xq, yq] = meshgrid(linspace(min(rp_coords(:,1))-1, max(rp_coords(:,1))+1, 50), ...
                    linspace(min(rp_coords(:,2))-1, max(rp_coords(:,2))+1, 50));
                
% 提取第一个AP的信号强度
ap1_rss = zeros(num_fingerprints, 1);
ap1_coords = zeros(num_fingerprints, 2);
for i = 1:num_fingerprints
    ap1_rss(i) = fingerprint_db(i).avg_fingerprint(1);
    ap1_coords(i, :) = fingerprint_db(i).coord;
end

% 插值
F = scatteredInterpolant(ap1_coords(:,1), ap1_coords(:,2), ap1_rss, 'natural');
vq = F(xq, yq);

% 绘制热图
imagesc([min(xq(:)), max(xq(:))], [min(yq(:)), max(yq(:))], vq);
hold on;
scatter(ap1_coords(:,1), ap1_coords(:,2), 30, 'k', 'filled');
xlabel('X坐标 (m)');
ylabel('Y坐标 (m)');
title('AP1信号强度热图 (dBm)');
colorbar;
axis equal tight;

% 5.3 绘制定位误差累积分布函数(CDF)
subplot(2, 3, 3);
hold on;
colors = {'r', 'g', 'b', 'm', 'c', 'y'};
color_idx = 1;

for method_idx = 1:length(methods)
    for k_idx = 1:length(K_values)
        K = K_values(k_idx);
        method_name = methods{method_idx};
        field_name = sprintf('%s_K%d', method_name, K);
        
        if isfield(results, field_name)
            errors = results.(field_name).errors;
            valid_errors = errors(~isnan(errors));
            
            % 计算CDF
            [f, x] = ecdf(valid_errors);
            plot(x, f, 'Color', colors{color_idx}, 'LineWidth', 2, ...
                 'DisplayName', sprintf('%s (K=%d)', method_name, K));
            color_idx = mod(color_idx, length(colors)) + 1;
        end
    end
end

xlabel('定位误差 (m)');
ylabel('累积概率');
title('定位误差CDF曲线');
legend('Location', 'southeast');
grid on;

% 5.4 绘制轨迹对比图
subplot(2, 3, 4);
plot(test_ref_coords(:,1), test_ref_coords(:,2), 'r-', 'LineWidth', 2, 'DisplayName', '真实轨迹');
hold on;

% 选择一个方法绘制估计轨迹
best_method = 'WKNN_K3';
if isfield(results, best_method)
    est_coords = results.(best_method).estimated_coords;
    plot(est_coords(:,1), est_coords(:,2), 'b--', 'LineWidth', 1.5, 'DisplayName', '估计轨迹');
end

xlabel('X坐标 (m)');
ylabel('Y坐标 (m)');
title('真实轨迹 vs 估计轨迹');
legend('Location', 'best');
grid on;
axis equal;

% 5.5 绘制误差随时间变化
subplot(2, 3, 5);
hold on;

for method_idx = 1:length(methods)
    for k_idx = 1:length(K_values)
        K = K_values(k_idx);
        method_name = methods{method_idx};
        field_name = sprintf('%s_K%d', method_name, K);
        
        if isfield(results, field_name)
            errors = results.(field_name).errors;
            plot(test_time, errors, 'DisplayName', sprintf('%s (K=%d)', method_name, K));
        end
    end
end

xlabel('时间 (s)');
ylabel('定位误差 (m)');
title('定位误差随时间变化');
legend('Location', 'best');
grid on;

% 5.6 绘制不同方法的性能比较
subplot(2, 3, 6);
methods_to_plot = {};
mean_errors = [];
for method_idx = 1:length(methods)
    for k_idx = 1:length(K_values)
        K = K_values(k_idx);
        method_name = methods{method_idx};
        field_name = sprintf('%s_K%d', method_name, K);
        
        if isfield(results, field_name)
            methods_to_plot{end+1} = sprintf('%s\nK=%d', method_name, K);
            mean_errors(end+1) = results.(field_name).mean_error;
        end
    end
end

bar(mean_errors);
set(gca, 'XTickLabel', methods_to_plot);
ylabel('平均定位误差 (m)');
title('不同定位方法的性能比较');
grid on;

%% 6. 输出详细统计报告
fprintf('\n========== 定位性能统计报告 ==========\n');
fprintf('总测试点数: %d\n', num_tests);
fprintf('有效测试点数: %d\n', sum(~isnan(test_ref_coords(:,1))));

fprintf('\n各方法定位性能对比:\n');
fprintf('%-15s %-10s %-10s %-10s\n', '方法', '平均误差(m)', 'RMSE(m)', '标准差(m)');
fprintf('%-15s %-10s %-10s %-10s\n', '--------', '----------', '-------', '----------');

best_method = '';
best_error = Inf;

for method_idx = 1:length(methods)
    for k_idx = 1:length(K_values)
        K = K_values(k_idx);
        method_name = methods{method_idx};
        field_name = sprintf('%s_K%d', method_name, K);
        
        if isfield(results, field_name)
            mean_err = results.(field_name).mean_error;
            rmse_err = results.(field_name).rmse;
            std_err = results.(field_name).std_error;
            
            fprintf('%-15s %-10.2f %-10.2f %-10.2f\n', ...
                   sprintf('%s (K=%d)', method_name, K), mean_err, rmse_err, std_err);
            
            if mean_err < best_error
                best_error = mean_err;
                best_method = field_name;
            end
        end
    end
end

fprintf('\n最佳定位方法: %s (平均误差 = %.2f m)\n', best_method, best_error);

%% 7. 保存结果到文件
fprintf('\n保存结果到文件...\n');

% 保存最佳方法的定位结果
if ~isempty(best_method)
    best_results = results.(best_method);
    
    % 创建输出文件
    output_file = 'localization_results.txt';
    fid = fopen(output_file, 'w');
    
    fprintf(fid, '时间(s) 真实X(m) 真实Y(m) 估计X(m) 估计Y(m) 误差(m)\n');
    
    for i = 1:num_tests
        if ~isnan(test_ref_coords(i, 1))
            fprintf(fid, '%.3f %.3f %.3f %.3f %.3f %.3f\n', ...
                    test_time(i), ...
                    test_ref_coords(i, 1), test_ref_coords(i, 2), ...
                    best_results.estimated_coords(i, 1), best_results.estimated_coords(i, 2), ...
                    best_results.errors(i));
        end
    end
    
    fclose(fid);
    fprintf('定位结果已保存到: %s\n', output_file);
end

% 保存指纹数据库
fingerprint_file = 'fingerprint_database.mat';
save(fingerprint_file, 'fingerprint_db', 'rp_coords', 'rp_ids');
fprintf('指纹数据库已保存到: %s\n', fingerprint_file);

fprintf('\n室内定位指纹匹配完成！\n');