%% KFGINS 导航结果误差分析脚本 (含粗差剔除)
% 功能：对比 NavResult 和 Ref_NavResult，计算误差，剔除姿态突变点并绘图
clear; clc; close all;

%% 1. 加载数据
file_test = 'NavResult_GNSSVEL.nav';
file_ref  = 'Ref_NavResult_GNSSVEL.nav';

if ~isfile(file_test) || ~isfile(file_ref)
    error('错误：未找到数据文件，请确保文件名正确并位于当前路径下。');
end

fprintf('正在加载数据...\n');
data_test = load(file_test);
data_ref  = load(file_ref);

%% 2. 数据对齐 (基于第2列 GPS时间)
[common_time, i_test, i_ref] = intersect(data_test(:,2), data_ref(:,2));

t = common_time; 
% 提取测试数据
pos_test = data_test(i_test, 3:5);   % Lat, Lon, H
vel_test = data_test(i_test, 6:8);   % Vn, Ve, Vd
att_test = data_test(i_test, 9:11);  % Roll, Pitch, Yaw

% 提取参考数据
pos_ref = data_ref(i_ref, 3:5);
vel_ref = data_ref(i_ref, 6:8);
att_ref = data_ref(i_ref, 9:11);

fprintf('共对齐 %d 个历元数据。\n', length(t));

%% 3. 计算误差

% --- 3.1 位置误差 (米) ---
Re = 6378137.0; f = 1/298.257223563; e2 = f*(2-f); D2R = pi/180;
lat_ref_rad = pos_ref(:,1) * D2R;
Rm = Re * (1 - e2) ./ (1 - e2 * sin(lat_ref_rad).^2).^1.5;
Rn = Re ./ sqrt(1 - e2 * sin(lat_ref_rad).^2);

dN = (pos_test(:,1) - pos_ref(:,1)) .* D2R .* (Rm + pos_ref(:,3)); 
dE = (pos_test(:,2) - pos_ref(:,2)) .* D2R .* (Rn + pos_ref(:,3)) .* cos(lat_ref_rad);
dU = pos_test(:,3) - pos_ref(:,3);

pos_err = [dN, dE, dU];

% --- 3.2 速度误差 (m/s) ---
vel_err = vel_test - vel_ref;

% --- 3.3 姿态误差初步计算 (degrees) ---
att_err = att_test - att_ref;
% 修正航向角 360 度跳变 (必须在粗差剔除前做)
att_err(:,3) = att_err(:,3) - 360 * fix((att_err(:,3) + 180)/360);

% --- 3.4 姿态粗差剔除 (NEW) ---
% 策略：如果当前历元误差与上一历元误差之差超过阈值，视为粗差剔除
% 你可以在这里调整阈值 (单位: 度)
step_threshold = 2.0; 

% 计算相邻历元的变化量 (第一行补0)
diff_att_err = [0 0 0; abs(diff(att_err))];

% 标记坏点 (变化量 > 阈值)
is_outlier = diff_att_err > step_threshold;

% 创建用于分析的“干净”数据
att_err_clean = att_err;
att_err_clean(is_outlier) = NaN; % 设置为 NaN，绘图和统计时自动忽略

% 打印剔除情况
fprintf('\n--- 粗差剔除统计 (阈值: %.1f度) ---\n', step_threshold);
fprintf('Roll 剔除点数: %d\n', sum(is_outlier(:,1)));
fprintf('Pitch剔除点数: %d\n', sum(is_outlier(:,2)));
fprintf('Yaw  剔除点数: %d\n', sum(is_outlier(:,3)));

%% 4. 统计分析 (RMS)
% 注意：使用 'omitnan' 忽略被剔除的点
rms_pos = rms(pos_err, 'omitnan');
rms_vel = rms(vel_err, 'omitnan');
rms_att = rms(att_err_clean, 'omitnan'); % 使用剔除后的数据

fprintf('\n------------------------------------------------------\n');
fprintf('                误差统计分析结果 (RMS)\n');
fprintf('------------------------------------------------------\n');
fprintf('位置误差 (m)   : North: %.4f, East: %.4f, Up: %.4f\n', rms_pos(1), rms_pos(2), rms_pos(3));
fprintf('速度误差 (m/s) : North: %.4f, East: %.4f, Down: %.4f\n', rms_vel(1), rms_vel(2), rms_vel(3));
fprintf('姿态误差 (deg) : Roll : %.4f, Pitch: %.4f, Yaw: %.4f\n', rms_att(1), rms_att(2), rms_att(3));
fprintf('------------------------------------------------------\n');

%% 5. 绘图

t_plot = t - t(1);

% 位置
figure('Name', 'Position Error', 'Color', 'w');
subplot(3,1,1); plot(t_plot, pos_err(:,1), 'r'); ylabel('North (m)'); title('Position Error'); grid on;
subplot(3,1,2); plot(t_plot, pos_err(:,2), 'g'); ylabel('East (m)'); grid on;
subplot(3,1,3); plot(t_plot, pos_err(:,3), 'b'); ylabel('Up (m)'); xlabel('Time (s)'); grid on;

% 速度
figure('Name', 'Velocity Error', 'Color', 'w');
subplot(3,1,1); plot(t_plot, vel_err(:,1), 'r'); ylabel('Vn (m/s)'); title('Velocity Error'); grid on;
subplot(3,1,2); plot(t_plot, vel_err(:,2), 'g'); ylabel('Ve (m/s)'); grid on;
subplot(3,1,3); plot(t_plot, vel_err(:,3), 'b'); ylabel('Vd (m/s)'); xlabel('Time (s)'); grid on;

% 姿态 (绘制剔除后的数据)
figure('Name', 'Attitude Error (Outliers Removed)', 'Color', 'w');
subplot(3,1,1); plot(t_plot, att_err_clean(:,1), 'r'); ylabel('Roll (deg)'); title(['Attitude Error (Step Threshold: ' num2str(step_threshold) ')']); grid on;
subplot(3,1,2); plot(t_plot, att_err_clean(:,2), 'g'); ylabel('Pitch (deg)'); grid on;
subplot(3,1,3); 
plot(t_plot, att_err_clean(:,3), 'b'); 
ylabel('Yaw (deg)'); 
xlabel('Time (s)'); 
ylim([-0.5 0.5]);   % <<< 新增：限制航向角纵坐标范围
grid on;
