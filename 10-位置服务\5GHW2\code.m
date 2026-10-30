clear; clc; close all;
warning off;

%% ===================== 1. 数据加载 =====================
fprintf('=== 开始加载数据 ===\n');

load('train_data.mat');
if ~exist('train_data', 'var')
    vars = whos;
    train_data = eval(vars(1).name);
end
fprintf('训练集：%d个参考点\n', length(train_data));

load('test_data_no_coor.mat');
if ~exist('test_data', 'var')
    vars = whos;
    test_data = eval(vars(1).name);
end
fprintf('测试集：%d个测试点\n', length(test_data));

%% ===================== 2. 特征提取与预处理（核心优化） =====================
fprintf('\n=== 开始特征提取与预处理 ===\n');

% 初始化
X_train = [];
Y_train = [];
test_feats = cell(length(test_data), 1); % 保存每个测试点的原始特征

% --- 处理训练集 ---
for i = 1:length(train_data)
    % 提取特征
    csi = double(train_data(i).csi{1,1});
    edp = double(train_data(i).csi_EDP{1,1});
    coor = train_data(i).coor;
    
    % 降维
    csi_beam_mean = zeros(size(csi,1), 8);
    for beam = 1:8
        csi_beam_mean(:, beam) = mean(abs(csi(:, (beam-1)*60+1:beam*60)), 2);
    end

    feat = [csi_beam_mean, edp];
    
    % 3σ
    feat_mean = mean(feat);
    feat_std = std(feat);
    valid_idx = all(abs(feat - feat_mean) < 3*feat_std, 2);
    feat_filtered = feat(valid_idx, :);
    
    % 拼接到训练集
    X_train = [X_train; feat_filtered];
    Y_train = [Y_train; repmat(coor, size(feat_filtered,1), 1)];
    
    fprintf('参考点%d: 过滤后剩余%d个样本\n', i, size(feat_filtered,1));
end

% --- 处理测试集 ---
for i = 1:length(test_data)
    csi = double(test_data(i).csi{1,1});
    edp = double(test_data(i).csi_EDP{1,1});
    
    % 同样的特征降维
    csi_beam_mean = zeros(size(csi,1), 8);
    for beam = 1:8
        csi_beam_mean(:, beam) = mean(abs(csi(:, (beam-1)*60+1:beam*60)), 2);
    end
    feat = [csi_beam_mean, edp];
    
    % 同样的3σ过滤
    feat_mean = mean(feat);
    feat_std = std(feat);
    valid_idx = all(abs(feat - feat_mean) < 3*feat_std, 2);
    test_feats{i} = feat(valid_idx, :);
    
    fprintf('测试点%d: 过滤后剩余%d个样本\n', i, size(test_feats{i},1));
end

fprintf('\n预处理完成！训练集总样本数：%d，特征维度：%d\n', size(X_train,1), size(X_train,2));

%% ===================== 3. 标准化 =====================
[X_train_norm, mu, sigma] = zscore(X_train);
sigma(sigma == 0) = 1;

%% ===================== 4. 优化的WKNN定位算法 =====================
fprintf('\n=== 开始定位计算 ===\n');

K = min(5, size(X_train_norm,1)); 
fprintf('使用K值：%d\n', K);

num_test_points = length(test_data);
test_preds = cell(num_test_points, 1); 
Final_Coordinates = zeros(num_test_points, 2); 

for i = 1:num_test_points

    X_test_norm = (test_feats{i} - mu) ./ sigma;
    num_samples = size(X_test_norm, 1);
    preds = zeros(num_samples, 2);
    
    for j = 1:num_samples

        weights = [ones(1,8), 2*ones(1,8)];
        dists = sqrt(sum(weights .* (X_train_norm - X_test_norm(j,:)).^2, 2));

        [sorted_dists, idx] = sort(dists);
        nearest_coords = Y_train(idx(1:K), :);
        nearest_dists = sorted_dists(1:K);

        w = exp(-nearest_dists.^2 / (2*median(nearest_dists)^2));
        w = w / sum(w);
        
        preds(j,:) = sum(nearest_coords .* w, 1);
    end
    
    test_preds{i} = preds;
  
    Final_Coordinates(i,:) = median(preds, 1);
    
    fprintf('测试点%d 预测完成，坐标：[%.2f, %.2f] m\n', i, Final_Coordinates(i,1), Final_Coordinates(i,2));
end

%% ===================== 5. 结果保存与可视化 =====================
fprintf('\n=== 结果保存与可视化 ===\n');

save('predicted_coordinates.mat', 'Final_Coordinates');
fprintf('6个测试点的最终坐标已保存到 predicted_coordinates.mat\n');


figure('Color','white','Position',[100,100,900,700]);
hold on; grid on; axis equal;
xlim([-0.5, 3.5]); ylim([-0.5, 8.5]); 

scatter(Y_train(:,1), Y_train(:,2), 150, [0.2 0.4 0.8], 'filled', 'DisplayName','训练参考点');

for i = 1:num_test_points
    scatter(test_preds{i}(:,1), test_preds{i}(:,2), 8, [1 0.5 0.5], 'filled');
end

scatter(Final_Coordinates(:,1), Final_Coordinates(:,2), 200, [0.8 0.2 0.2], 'filled', 'DisplayName','测试点预测结果');
for i = 1:num_test_points
    text(Final_Coordinates(i,1)+0.1, Final_Coordinates(i,2)+0.1, num2str(i), ...
        'FontSize',12, 'FontWeight','bold', 'Color','black');
end

title('优化结果 (WKNN, K=5)');
xlabel('X坐标 (m)');
ylabel('Y坐标 (m)');
legend('Location','northeast');

fprintf('\n=== 6个点 ===\n');
fprintf('测试点\tX坐标(m)\tY坐标(m)\n');
for i = 1:num_test_points
    fprintf('%d\t%.2f\t\t%.2f\n', i, Final_Coordinates(i,1), Final_Coordinates(i,2));
end
