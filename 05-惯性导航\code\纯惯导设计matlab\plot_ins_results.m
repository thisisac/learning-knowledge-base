function plot_ins_results(presult, mInsNavEpoch, diff)
%PLOT_INS_RESULTS  INS结果与参考的对比图（只画线 + 基本图例）
% 输入:
%   presult        [N x 10]  INS解算数组 [t, lat_deg, lon_deg, h_m, vN, vE, vD, roll_deg, pitch_deg, yaw_deg]
%   mInsNavEpoch   [N x 10]  参考结果数组（PureINS.bin）
%   diff           [N x 10]  差值矩阵（presult - mInsNavEpoch, 第1列保留时间戳）

    N = size(diff, 1);
    t = presult(:,1);
    if numel(t) ~= N || any(~isfinite(t))
        t = (0:N-1).' * 0.005; % 回退：200 Hz
    else
        t = t - t(1); % 从0开始
    end

    % 数据提取
    latErr   = diff(:, 2);   lonErr = diff(:, 3);   hErr = diff(:, 4);
    vNErr    = diff(:, 5);   vEErr  = diff(:, 6);   vDErr= diff(:, 7);
    rollErr  = diff(:, 8);   pitchErr=diff(:, 9);   yawErr= diff(:,10);

    refLat   = mInsNavEpoch(:, 2);  refLon = mInsNavEpoch(:, 3);
    estLat   = presult(:, 2);       estLon = presult(:, 3);
    refH     = mInsNavEpoch(:, 4);  estH   = presult(:, 4);

    xmin = min([min(estLon), min(refLon)]); xmax = max([max(estLon), max(refLon)]);
    ymin = min([min(estLat), min(refLat)]); ymax = max([max(estLat), max(refLat)]);

    % 风格参数
    lw = 1.8; fs = 12; axlw = 1.0;

    % 图1: 纬/经度误差（deg）
    figure('Name','Position Error (B/L)','Color','w'); ax = axes; hold(ax,'on');
    plot(ax, t, latErr, '-', 'Color',[0.00 0.45 0.74], 'LineWidth',lw, 'DisplayName','lat (deg)');
    plot(ax, t, lonErr, '-', 'Color',[0.85 0.33 0.10], 'LineWidth',lw, 'DisplayName','lon (deg)');
    applyAxesStyle(ax, fs, axlw);
    xlabel(ax,'Time (s)'); ylabel(ax,'Error (deg)'); title(ax,'Position Error: Latitude / Longitude');
    legend(ax, 'Location','northeast','Box','off');

    % 图2: 高程误差（m）
    figure('Name','Height Error','Color','w'); ax = axes; hold(ax,'on');
    plot(ax, t, hErr, '-', 'Color',[0.00 0.62 0.45], 'LineWidth',lw, 'DisplayName','height (m)');
    applyAxesStyle(ax, fs, axlw);
    xlabel(ax,'Time (s)'); ylabel(ax,'Error (m)'); title(ax,'Position Error: Height');
    legend(ax, 'Location','northeast','Box','off');

    % 图3: 速度误差 N/E/D（m/s）
    figure('Name','Velocity Error NED','Color','w'); ax = axes; hold(ax,'on');
    plot(ax, t, vNErr, '-', 'Color',[0.85 0.33 0.10], 'LineWidth',lw, 'DisplayName','v_N (m/s)');
    plot(ax, t, vEErr, '-', 'Color',[0.00 0.45 0.74], 'LineWidth',lw, 'DisplayName','v_E (m/s)');
    plot(ax, t, vDErr, '-', 'Color',[0.49 0.18 0.56], 'LineWidth',lw, 'DisplayName','v_D (m/s)');
    applyAxesStyle(ax, fs, axlw);
    xlabel(ax,'Time (s)'); ylabel(ax,'Error (m/s)'); title(ax,'Velocity Error: N / E / D');
    legend(ax, 'Location','northeast','Box','off');

    % 图4: 姿态误差（deg）
    figure('Name','Attitude Error','Color','w'); ax = axes; hold(ax,'on');
    plot(ax, t, rollErr,  '-', 'Color',[0.85 0.33 0.10], 'LineWidth',lw, 'DisplayName','roll (deg)');
    plot(ax, t, pitchErr, '-', 'Color',[0.00 0.45 0.74], 'LineWidth',lw, 'DisplayName','pitch (deg)');
    plot(ax, t, yawErr,   '-', 'Color',[0.00 0.62 0.45], 'LineWidth',lw, 'DisplayName','yaw (deg)');
    applyAxesStyle(ax, fs, axlw);
    xlabel(ax,'Time (s)'); ylabel(ax,'Error (deg)'); title(ax,'Attitude Error: Roll / Pitch / Yaw');
    legend(ax, 'Location','northeast','Box','off');

    % 图5: 平面轨迹（deg）
    figure('Name','Planar Trajectory','Color','w'); ax = axes; hold(ax,'on');
    plot(ax, refLon, refLat, '-', 'Color',[0.30 0.30 0.30], 'LineWidth',2.2, 'DisplayName','Reference');
    plot(ax, estLon, estLat, '-', 'Color',[0.85 0.33 0.10], 'LineWidth',1.6, 'DisplayName','INS result');
    applyAxesStyle(ax, fs, axlw);
    xtickformat(ax,'%.5f'); ytickformat(ax,'%.5f');
    xlim(ax,[xmin, xmax]); ylim(ax,[ymin, ymax]); axis(ax,'equal');
    xlabel(ax,'Longitude (deg)'); ylabel(ax,'Latitude (deg)'); title(ax,'Planar Trajectory (B/L)');
    legend(ax, 'Location','best','Box','off');

    % 图6: 高程变化（m）
    figure('Name','Elevation Change','Color','w'); ax = axes; hold(ax,'on');
    plot(ax, t, refH, '-', 'Color',[0.30 0.30 0.30], 'LineWidth',2.2, 'DisplayName','Reference');
    plot(ax, t, estH, '-', 'Color',[0.85 0.33 0.10], 'LineWidth',1.6, 'DisplayName','INS result');
    applyAxesStyle(ax, fs, axlw);
    xlabel(ax,'Time (s)'); ylabel(ax,'Height (m)'); title(ax,'Elevation Change');
    legend(ax, 'Location','best','Box','off');
end

% 统一坐标轴风格
function applyAxesStyle(ax, fs, axlw)
    grid(ax,'on'); ax.GridAlpha = 0.25;
    ax.MinorGridLineStyle = ':'; grid(ax,'minor'); ax.MinorGridAlpha = 0.15;
    ax.Box = 'on'; ax.LineWidth = axlw; ax.FontSize = fs; ax.TickDir = 'out';
    axis(ax,'tight');
end