function PlotNavResults(folder, ref_file, modes)
    param = Param();
    ref_all = load(ref_file);
    cfg = ProcessConfig_LJY(); 

    figure('Name', '位置误差对比 (车体坐标系)');
    labels = {'前向 (Forward) / m', '右向 (Lateral) / m', '下向 (Down) / m'};
    colors = lines(length(modes));
    
    for m = 1:length(modes)
        res_file = fullfile(folder, ['NavResult_' modes{m} '.nav']);
        if ~exist(res_file, 'file'), continue; end
        res = load(res_file);
        
        % 1. 时间对齐
        [~, idx_ref, idx_res] = intersect(ref_all(:,2), res(:,2));
        ref = ref_all(idx_ref, :);
        data = res(idx_res, :);
        time = data(:,2) - data(1,2);
        
        % 2. 计算位置误差投影
        err_v = zeros(size(data,1), 3);
        for i = 1:size(data, 1)
            % 使用你项目中已有的 getRmRn 函数，避免字段不存在的问题
            [Rm, Rn] = getRmRn(ref(i,3)*param.D2R, param);
            
            % 导航系(NED)下的位置误差 (m)
            dn = (data(i,3) - ref(i,3)) * param.D2R * (Rm + ref(i,5));
            de = (data(i,4) - ref(i,4)) * param.D2R * (Rn + ref(i,5)) * cos(ref(i,3)*param.D2R);
            dd = data(i,5) - ref(i,5);
            err_n = [dn; de; dd];
            
            % 3. 投影到车体系 (v-frame)
            % 姿态矩阵 C_n^b
            Cnb = euler2dcm(data(i, 9:11)*param.D2R)'; 
            % err_v = C_b^v * C_n^b * err_n
            err_v(i, :) = (cfg.cbv * Cnb * err_n)';
        end
        
        % 4. 分轴绘图
        for j = 1:3
            subplot(3,1,j); hold on;
            plot(time, err_v(:,j), 'Color', colors(m,:), 'LineWidth', 1.2);
            ylabel(labels{j}); grid on;
        end
    end
    
    % 图形美化
    subplot(3,1,1); legend(modes, 'Interpreter', 'none', 'Location', 'northeastoutside');
    title('车载运动约束对定位精度的提升分析 (车体坐标系)');
    xlabel('时间 (s)');
end