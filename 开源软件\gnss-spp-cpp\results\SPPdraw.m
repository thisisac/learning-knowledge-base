clc; clear; close all;

%% 1. 读取文件数据
fid = fopen('RESsocket.txt', 'r');
header = fgetl(fid);
data = textscan(fid, ['%f %f %f %f' repmat(' %f', 1, 37)]);
fclose(fid);

% 提取数据列
week     = data{1};
sow      = data{2};
gps_sat  = data{3};
bds_sat  = data{4};
est_x    = data{5};
est_y    = data{6};
est_z    = data{7};
ref_x    = data{8};
ref_y    = data{9};
ref_z    = data{10};
vel_x    = data{11};
vel_y    = data{12};
vel_z    = data{13};
pdop     = data{14};
pos_sigma= data{15};
vel_sigma= data{16};
clk_gps  = data{17};
clk_bds  = data{18};
std_x    = data{19};
std_y    = data{20};
std_z    = data{21};
std_e    = data{22};
std_n    = data{23};
std_u    = data{24};

% 计算时间向量（单位：小时）
hours = (sow - sow(1)) / 3600;

%% 2. ENU定位误差分析
% 2.1 计算参考点经纬度
ref_llh = ecef2llh([ref_x(1), ref_y(1), ref_z(1)]);
lat0 = ref_llh(1);
lon0 = ref_llh(2);
lat0_rad = deg2rad(lat0);
lon0_rad = deg2rad(lon0);

% 2.2 计算ECEF误差
dx = est_x - ref_x;
dy = est_y - ref_y;
dz = est_z - ref_z;

% 2.3 转换为ENU误差
enu_errors = zeros(length(dx), 3);
R = [-sin(lon0_rad),           cos(lon0_rad),           0;
     -sin(lat0_rad)*cos(lon0_rad), -sin(lat0_rad)*sin(lon0_rad), cos(lat0_rad);
      cos(lat0_rad)*cos(lon0_rad),  cos(lat0_rad)*sin(lon0_rad), sin(lat0_rad)];

for i = 1:length(dx)
    enu_errors(i, :) = R * [dx(i); dy(i); dz(i)];
end

east_errors  = enu_errors(:,1);
north_errors = enu_errors(:,2);
up_errors    = enu_errors(:,3);

% 2.4 误差统计
east_rms  = rms(east_errors);
north_rms = rms(north_errors);
up_rms    = rms(up_errors);

east_std  = std(east_errors);
north_std = std(north_errors);
up_std    = std(up_errors);

%% 3. 图像可视化（单图分开绘制）

% 3.1 ENU误差时间序列
figure('Name', 'ENU定位误差时间序列');
plot(hours, east_errors, 'b', hours, north_errors, 'g', hours, up_errors, 'r', 'LineWidth', 1.5);
legend('东方向', '北方向', '天方向');
xlabel('时间 (小时)'); ylabel('误差 (m)');
title('ENU定位误差时间序列'); grid on;

% 3.2 平面误差散点图（East-North）
figure('Name', '平面定位误差分布 (EN)');
scatter(east_errors, north_errors, 50, hours, 'filled');
xlabel('东方向 (m)'); ylabel('北方向 (m)');
title('平面定位误差分布 (EN)');
grid on; axis equal; colorbar;

% 3.3 三维误差分布
figure('Name', '三维定位误差分布 (ENU)');
scatter3(east_errors, north_errors, up_errors, 50, hours, 'filled');
xlabel('East (m)'); ylabel('North (m)'); zlabel('Up (m)');
title('三维定位误差分布'); grid on; colorbar; rotate3d on;

% 3.4 PDOP时间序列
figure('Name', 'PDOP值时间序列');
plot(hours, pdop, 'LineWidth', 2, 'Color', [0.5 0 0.5]);
xlabel('时间 (小时)'); ylabel('PDOP'); title('PDOP值时间序列'); grid on;

% 3.5 位置标准差时间序列
figure('Name', '位置标准差时间序列');
plot(hours, pos_sigma, 'LineWidth', 2, 'Color', [0 0.6 0]);
xlabel('时间 (小时)'); ylabel('位置标准差 (m)'); title('位置标准差时间序列'); grid on;

% 3.6 速度分量
figure('Name', '速度分量时间序列');
plot(hours, vel_x, 'r', hours, vel_y, 'g', hours, vel_z, 'b', 'LineWidth', 1.5);
xlabel('时间 (小时)'); ylabel('速度 (m/s)');
legend('X方向', 'Y方向', 'Z方向'); title('速度分量时间序列'); grid on;

% 3.7 钟差
figure('Name', '接收机钟差时间序列');
yyaxis left; plot(hours, clk_gps, 'b', 'LineWidth', 1.5); ylabel('GPS钟差 (ns)');
yyaxis right; plot(hours, clk_bds, 'r', 'LineWidth', 1.5); ylabel('BDS钟差 (ns)');
xlabel('时间 (小时)'); title('接收机钟差时间序列'); legend('GPS', 'BDS'); grid on;

% 3.8 卫星数（添加总数）
figure('Name', '可见卫星数时间序列');
plot(hours, gps_sat, 'b', hours, bds_sat, 'r', hours, gps_sat + bds_sat, 'k--', 'LineWidth', 1.5);
xlabel('时间 (小时)'); ylabel('卫星数');
legend('GPS', 'BDS', '总数'); title('可见卫星数时间序列'); grid on;

% 3.9 ENU误差统计柱状图
figure('Name', '定位精度统计');
bar([east_std, north_std, up_std; east_rms, north_rms, up_rms]');
set(gca, 'XTickLabel', {'东方向', '北方向', '天方向'});
ylabel('误差 (m)'); legend('标准差', 'RMS'); title('定位误差统计'); grid on;

%% 4. 参考坐标标准差分析图
figure('Name', '参考坐标精度分析');
subplot(1,2,1);
plot(hours, std_e, 'r', hours, std_n, 'g', hours, std_u, 'b', 'LineWidth', 1.5);
xlabel('时间 (小时)'); ylabel('标准差 (m)');
legend('East', 'North', 'Up'); title('ENU方向标准差时间序列'); grid on;

subplot(1,2,2);
boxplot([east_errors, north_errors, up_errors], 'Labels', {'East', 'North', 'Up'});
ylabel('误差 (m)'); title('ENU误差分布箱线图'); grid on;

%% 5. 输出统计结果
fprintf('===== 定位误差统计 =====\n');
fprintf('East 方向: 均值 = %.4fm, 标准差 = %.4fm, RMS = %.4fm\n', mean(east_errors), east_std, east_rms);
fprintf('North方向: 均值 = %.4fm, 标准差 = %.4fm, RMS = %.4fm\n', mean(north_errors), north_std, north_rms);
fprintf('Up   方向: 均值 = %.4fm, 标准差 = %.4fm, RMS = %.4fm\n\n', mean(up_errors), up_std, up_rms);

fprintf('===== 系统性能指标 =====\n');
fprintf('平均 PDOP: %.4f\n', mean(pdop));
fprintf('平均位置标准差: %.4fm\n', mean(pos_sigma));
fprintf('平均速度标准差: %.4fm/s\n\n', mean(vel_sigma));

fprintf('===== 卫星使用情况 =====\n');
fprintf('平均 GPS 卫星数: %.2f\n', mean(gps_sat));
fprintf('平均 BDS 卫星数: %.2f\n', mean(bds_sat));
fprintf('总卫星数范围: %d ~ %d 颗\n', min(gps_sat + bds_sat), max(gps_sat + bds_sat));

%% 附加函数：ECEF 转 LLH
function llh = ecef2llh(ecef)
    a = 6378137.0;
    f = 1 / 298.257223563;
    b = a * (1 - f);
    e = sqrt((a^2 - b^2) / a^2);
    x = ecef(1); y = ecef(2); z = ecef(3);
    lon = atan2d(y, x); p = sqrt(x^2 + y^2);
    lat0 = atan2d(z, p*(1-e^2)); tol = 1e-12;
    for i = 1:10
        N = a / sqrt(1 - e^2 * sind(lat0)^2);
        h = p / cosd(lat0) - N;
        lat = atan2d(z, p * (1 - e^2 * N / (N + h)));
        if abs(lat - lat0) < tol, break; end
        lat0 = lat;
    end
    N = a / sqrt(1 - e^2 * sind(lat)^2);
    h = p / cosd(lat) - N;
    llh = [lat, lon, h];
end
