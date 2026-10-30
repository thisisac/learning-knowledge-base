function draw2(presult, ref_data, diff, nav)
    analyze_reference_data(nav);
    analyze_ins_results(presult);
    plot_comparison_results(presult, ref_data, diff);
end

%% 参考数据分析
function analyze_reference_data(nav)
    % 参考数据轨迹图
    figure('Name', '参考数据轨迹分析', 'Position', [100, 100, 1200, 800]);
    
    % 平面轨迹
    subplot(2,3,1);
    plot(nav.lon_deg, nav.lat_deg, 'b-', 'LineWidth', 2);
    xlabel('经度 (deg)'); ylabel('纬度 (deg)'); title('参考平面轨迹');
    grid on; axis equal;
    
    % 高程变化
    subplot(2,3,2);
    plot(nav.sow - nav.sow(1), nav.h_m, 'r-', 'LineWidth', 2);
    xlabel('时间 (s)'); ylabel('高程 (m)'); title('参考高程变化');
    grid on;
    
    % 速度分量
    subplot(2,3,3);
    plot(nav.sow - nav.sow(1), nav.vN, 'r-', 'DisplayName', 'vN'); hold on;
    plot(nav.sow - nav.sow(1), nav.vE, 'g-', 'DisplayName', 'vE');
    plot(nav.sow - nav.sow(1), nav.vD, 'b-', 'DisplayName', 'vD');
    xlabel('时间 (s)'); ylabel('速度 (m/s)'); title('参考速度分量');
    legend; grid on;
    
    % 姿态角
    subplot(2,3,4);
    plot(nav.sow - nav.sow(1), nav.roll_deg, 'r-', 'DisplayName', '横滚'); hold on;
    plot(nav.sow - nav.sow(1), nav.pitch_deg, 'g-', 'DisplayName', '俯仰');
    plot(nav.sow - nav.sow(1), nav.heading_deg, 'b-', 'DisplayName', '航向');
    xlabel('时间 (s)'); ylabel('角度 (deg)'); title('参考姿态角');
    legend; grid on;
    
    % 速度大小
    subplot(2,3,5);
    speed = sqrt(nav.vN.^2 + nav.vE.^2 + nav.vD.^2);
    plot(nav.sow - nav.sow(1), speed, 'm-', 'LineWidth', 2);
    xlabel('时间 (s)'); ylabel('速度大小 (m/s)'); title('参考合速度');
    grid on;
    
    % 3D轨迹
    subplot(2,3,6);
    plot3(nav.lon_deg, nav.lat_deg, nav.h_m, 'k-', 'LineWidth', 2);
    xlabel('经度'); ylabel('纬度'); zlabel('高程 (m)'); title('参考3D轨迹');
    grid on; rotate3d on;
end

%% INS解算结果分析
function analyze_ins_results(presult)

    
    t_ins = presult(:,1) - presult(1,1);
    
    
    % INS解算结果图
    figure('Name', 'INS解算结果分析', 'Position', [200, 100, 1200, 800]);
    
    % 平面轨迹
    subplot(2,3,1);
    plot(presult(:,3), presult(:,2), 'r-', 'LineWidth', 2);
    xlabel('经度 (deg)'); ylabel('纬度 (deg)'); title('INS解算平面轨迹');
    grid on; axis equal;
    
    % 高程变化
    subplot(2,3,2);
    plot(t_ins, presult(:,4), 'r-', 'LineWidth', 2);
    xlabel('时间 (s)'); ylabel('高程 (m)'); title('INS解算高程变化');
    grid on;
    
    % 速度分量
    subplot(2,3,3);
    plot(t_ins, presult(:,5), 'r-', 'DisplayName', 'vN'); hold on;
    plot(t_ins, presult(:,6), 'g-', 'DisplayName', 'vE');
    plot(t_ins, presult(:,7), 'b-', 'DisplayName', 'vD');
    xlabel('时间 (s)'); ylabel('速度 (m/s)'); title('INS解算速度分量');
    legend; grid on;
    
    % 姿态角
    subplot(2,3,4);
    plot(t_ins, presult(:,8), 'r-', 'DisplayName', '横滚'); hold on;
    plot(t_ins, presult(:,9), 'g-', 'DisplayName', '俯仰');
    plot(t_ins, presult(:,10), 'b-', 'DisplayName', '航向');
    xlabel('时间 (s)'); ylabel('角度 (deg)'); title('INS解算姿态角');
    legend; grid on;
    
    % 速度大小
    subplot(2,3,5);
    speed_ins = sqrt(presult(:,5).^2 + presult(:,6).^2 + presult(:,7).^2);
    plot(t_ins, speed_ins, 'm-', 'LineWidth', 2);
    xlabel('时间 (s)'); ylabel('速度大小 (m/s)'); title('INS解算合速度');
    grid on;
    
    % 3D轨迹
    subplot(2,3,6);
    plot3(presult(:,3), presult(:,2), presult(:,4), 'r-', 'LineWidth', 2);
    xlabel('经度'); ylabel('纬度'); zlabel('高程 (m)'); title('INS解算3D轨迹');
    grid on; rotate3d on;
end

%% 对比分析绘图
function plot_comparison_results(presult, ref_data, diff)

    
    N = size(diff, 1);
    t = presult(:,1) - presult(1,1);
    
    % 数据提取
    latErr   = diff(:, 2);   lonErr = diff(:, 3);   hErr = diff(:, 4);
    vNErr    = diff(:, 5);   vEErr  = diff(:, 6);   vDErr= diff(:, 7);
    rollErr  = diff(:, 8);   pitchErr=diff(:, 9);   yawErr= diff(:,10);

    refLat   = ref_data(:, 2);  refLon = ref_data(:, 3);
    estLat   = presult(:, 2);   estLon = presult(:, 3);
    refH     = ref_data(:, 4);  estH   = presult(:, 4);

    xmin = min([min(estLon), min(refLon)]); xmax = max([max(estLon), max(refLon)]);
    ymin = min([min(estLat), min(refLat)]); ymax = max([max(estLat), max(refLat)]);

    % 风格参数
    lw = 1.8; fs = 12; axlw = 1.0;

    % 图1: 位置误差对比
    figure('Name','位置误差对比','Color','w', 'Position', [100, 100, 1000, 600]);
    
    subplot(2,2,1);
    plot(t, latErr, '-', 'Color',[0.00 0.45 0.74], 'LineWidth',lw);
    xlabel('时间 (s)'); ylabel('误差 (deg)'); title('纬度误差');
    grid on;
    
    subplot(2,2,2);
    plot(t, lonErr, '-', 'Color',[0.85 0.33 0.10], 'LineWidth',lw);
    xlabel('时间 (s)'); ylabel('误差 (deg)'); title('经度误差');
    grid on;
    
    subplot(2,2,3);
    plot(t, hErr, '-', 'Color',[0.00 0.62 0.45], 'LineWidth',lw);
    xlabel('时间 (s)'); ylabel('误差 (m)'); title('高程误差');
    grid on;
    
    subplot(2,2,4);
    plot(t, sqrt(latErr.^2 + lonErr.^2) * 111000, '-', 'Color',[0.49 0.18 0.56], 'LineWidth',lw);
    xlabel('时间 (s)'); ylabel('误差 (m)'); title('平面位置误差(米)');
    grid on;

    % 图2: 速度误差对比
    figure('Name','速度误差对比','Color','w', 'Position', [200, 200, 1000, 600]);
    
    subplot(2,2,1);
    plot(t, vNErr, '-', 'Color',[0.85 0.33 0.10], 'LineWidth',lw);
    xlabel('时间 (s)'); ylabel('误差 (m/s)'); title('北向速度误差');
    grid on;
    
    subplot(2,2,2);
    plot(t, vEErr, '-', 'Color',[0.00 0.45 0.74], 'LineWidth',lw);
    xlabel('时间 (s)'); ylabel('误差 (m/s)'); title('东向速度误差');
    grid on;
    
    subplot(2,2,3);
    plot(t, vDErr, '-', 'Color',[0.49 0.18 0.56], 'LineWidth',lw);
    xlabel('时间 (s)'); ylabel('误差 (m/s)'); title('地向速度误差');
    grid on;
    
    subplot(2,2,4);
    plot(t, sqrt(vNErr.^2 + vEErr.^2 + vDErr.^2), '-', 'Color',[0.00 0.62 0.45], 'LineWidth',lw);
    xlabel('时间 (s)'); ylabel('误差 (m/s)'); title('速度矢量误差');
    grid on;

    % 图3: 姿态误差对比
    figure('Name','姿态误差对比','Color','w', 'Position', [300, 300, 1000, 600]);
    
    subplot(2,2,1);
    plot(t, rollErr, '-', 'Color',[0.85 0.33 0.10], 'LineWidth',lw);
    xlabel('时间 (s)'); ylabel('误差 (deg)'); title('横滚误差');
    grid on;
    
    subplot(2,2,2);
    plot(t, pitchErr, '-', 'Color',[0.00 0.45 0.74], 'LineWidth',lw);
    xlabel('时间 (s)'); ylabel('误差 (deg)'); title('俯仰误差');
    grid on;
    
    subplot(2,2,3);
    plot(t, yawErr, '-', 'Color',[0.00 0.62 0.45], 'LineWidth',lw);
    xlabel('时间 (s)'); ylabel('误差 (deg)'); title('航向误差');
    grid on;
    
    subplot(2,2,4);
    plot(t, sqrt(rollErr.^2 + pitchErr.^2 + yawErr.^2), '-', 'Color',[0.49 0.18 0.56], 'LineWidth',lw);
    xlabel('时间 (s)'); ylabel('误差 (deg)'); title('姿态矢量误差');
    grid on;

    % 图4: 轨迹对比
    figure('Name','轨迹对比','Color','w', 'Position', [400, 400, 1200, 400]);
    
    subplot(1,3,1);
    plot(refLon, refLat, 'b-', 'LineWidth', 2, 'DisplayName','参考轨迹'); hold on;
    plot(estLon, estLat, 'r-', 'LineWidth', 1, 'DisplayName','INS轨迹');
    xlabel('经度 (deg)'); ylabel('纬度 (deg)'); title('平面轨迹对比');
    legend; grid on; axis equal;
    xlim([xmin, xmax]); ylim([ymin, ymax]);
    
    subplot(1,3,2);
    plot(t, refH, 'b-', 'LineWidth', 2, 'DisplayName','参考高程'); hold on;
    plot(t, estH, 'r-', 'LineWidth', 1, 'DisplayName','INS高程');
    xlabel('时间 (s)'); ylabel('高程 (m)'); title('高程对比');
    legend; grid on;
    
    subplot(1,3,3);
    plot3(refLon, refLat, refH, 'b-', 'LineWidth', 2, 'DisplayName','参考轨迹'); hold on;
    plot3(estLon, estLat, estH, 'r-', 'LineWidth', 1, 'DisplayName','INS轨迹');
    xlabel('经度'); ylabel('纬度'); zlabel('高程 (m)'); title('3D轨迹对比');
    legend; grid on; rotate3d on;
end
