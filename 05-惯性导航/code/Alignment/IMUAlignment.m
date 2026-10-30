clc;
clear;

% 读取数据并解析
filename = 'A01_DJ_New.ASC';
fileID = fopen(filename, 'r');
data = textscan(fileID, '%s', 'Delimiter', '\n'); % 按行读取数据
fclose(fileID);

% 初始化存储变量
rawdata1 = zeros(length(data{1}), 3);
rawdata2 = zeros(length(data{1}), 3);

% 加速计和陀螺仪的缩放因子
accf = 2e-8 * 200;
gyo = 1e-9 * 200;

% 解析每一行数据
for i = 1:length(data{1})
    line = data{1}{i};
    
    % 去除校验码部分，星号后面的内容
    line = regexp(line, '^(.*?)(\*.*)?$', 'tokens', 'once');
    line = line{1}; % 提取去掉星号后的数据部分
    
    % 按逗号分割数据并转换为数字
    num = str2double(regexp(line, '[,;]', 'split'));
    
    % 检查数据是否有效
    if numel(num) >= 12
        % 提取加速度数据和陀螺仪数据
        acc_data  = num(9:11);  % 加速度数据
        gyro_data = num(12:14); % 陀螺仪数据
        
        % 存储加速度和陀螺仪数据
        rawdata1(i, :) = [-acc_data(3) * accf, acc_data(2) * accf, -acc_data(1) * accf];
        rawdata2(i, :) = [-gyro_data(3) * gyo, gyro_data(2) * gyo, -gyro_data(1) * gyo];
    end
end

% 去掉未使用的部分
rawdata1(any(isnan(rawdata1), 2), :) = []; % 删除包含NaN的行
rawdata2(any(isnan(rawdata2), 2), :) = []; % 删除包含NaN的行



% 计算姿态矩阵和欧拉角
function OLangel = Calangel(Cbn)
    % pitch：俯仰角（绕y轴）
    pitch = atan2(-Cbn(3, 1), sqrt(Cbn(3, 2)^2 + Cbn(3, 3)^2));
    % roll：横滚角（绕x轴）
    roll = atan2(Cbn(3, 2), Cbn(3, 3));
    % yaw：偏航角（绕z轴）
    yaw = atan2(Cbn(2, 1), Cbn(1, 1));
    
    % 将弧度转为角度
    rad2deg = 180 / pi;
    pitch = rad2deg * pitch;
    roll = rad2deg * roll;
    yaw = rad2deg * yaw;
    OLangel = [pitch, roll, yaw];
end

% 计算加速度和陀螺仪数据的平均
% function Cbn = com_Cbn(gn, wn, vn, gb, wb)
%     % 归一化
%     wg = gb / norm(gb);
%     vb = vec_cross(gb, wb);
%     vvwgw = vec_cross(vec_cross(gb, wb), gb);
%     ww = vb / norm(vb);
%     wgw = vvwgw / norm(vvwgw);
% 
%     % 计算姿态矩阵
%     v = [gn', wn', vn'];
%     w = [wg', ww', wgw'];
%     Cbn = inv(v') * w';
% end

function Cbn = CalCbn(g, w, gw, gb, wb)
    % 归一化

    Bgw = cross(gb, wb);  % 计算叉乘
    Bggw = cross(cross(gb, wb), gb);  % 计算叉乘
    
    % 归一化结果
    B2g = gb / norm(gb);  % 归一化重力方向
    B2gw = Bgw / norm(Bgw);
    B2ggw = Bggw / norm(Bggw);
    
    % 构造姿态矩阵
    v = [g, w, gw];  % 3x3 矩阵
    w = [B2g', B2gw', B2ggw'];  % 3x3 矩阵
    
    % 姿态矩阵：直接进行矩阵乘法
    Cbn = v * w';  % 计算姿态矩阵
end


% 函数：叉乘
function result = vec_cross(v, v1)
    vx = v(1);
    vy = v(2);
    vz = v(3);
    v2 = [0, -vz, vy;
          vz, 0, -vx;
         -vy, vx, 0];

    result = v2 * v1;
end


% 计算姿态角
% 函数：计算姿态矩阵（com_Cbn）和欧拉角（com_OL）

% 定义常数
g = 9.7936174;  % 重力加速度（m/s^2）
we = 7.292115e-5; % 地球自转角速度（rad/s）
phi = 30.531651244 * pi / 180; % 纬度（弧度）

% 计算地球的重力和自转角速度
gn = [0; 0; g];  % 重力方向向量
wien = [we * cos(phi); 0; -we * sin(phi)];  % 地球自转的角速度向量

ll2 = vec_cross(gn, wien);
ll3 = vec_cross(vec_cross(gn, wien), gn);

% 归一化
g = gn / norm(gn);  % 重力方向单位向量
gw = ll2 / norm(ll2);  % 自转方向单位向量
ggw = ll3 / norm(ll3);  % 自转影响方向单位向量
% 1. **整段静态数据平均后计算一次姿态角**
% 计算整个数据的平均值
ave_data1 = mean(rawdata1, 1);
ave_data2 = mean(rawdata2, 1);

% 计算每个数据的姿态角
Cbn = CalCbn(g, gw, ggw, ave_data1, ave_data2);
ave_OL = Calangel(Cbn);

disp('整段静态数据平均后的姿态角:');
disp(ave_OL);

% 2. **每秒平均值计算姿态角**
% 每秒数据平均计算
% 计算每秒的姿态角
sec_len = 200;  % 每秒2000条数据
num_of_seconds = floor(length(rawdata1) / sec_len);
sec_OL = [];
for i = 1:num_of_seconds
    start_index = (i - 1) * sec_len + 1;
    end_index = min(i * sec_len, length(rawdata1));
    sec_fdata = rawdata1(start_index:end_index, :);
    sec_wdata = rawdata2(start_index:end_index, :);
    
    % 计算每秒的加速度和陀螺仪的平均值
    sec_avg_fdata = mean(sec_fdata, 1);
    sec_avg_wdata = mean(sec_wdata, 1);
    
    % 计算每秒的姿态角
    Cbn_sec = CalCbn(g, gw, ggw, sec_avg_fdata, sec_avg_wdata);
    sec_OL = [sec_OL; Calangel(Cbn_sec)];
end

% 归算Roll角度到-180到180度之间
sec_OL(:, 2) = mod(sec_OL(:, 2) + 180, 360) - 180;  % 将Roll角度调整为-180到180之间

% 绘制每秒的姿态角-时间图
time_sec = (1:length(sec_OL))';
figure;
subplot(3,1,1);
plot(time_sec, sec_OL(:,1), 'b');
xlabel('Time (s)');
ylabel('Pitch Angle (deg)');
title('Pitch Angle vs Time (Every Second)');

subplot(3,1,2);
plot(time_sec, sec_OL(:,2), 'r');  % 使用调整后的roll角度
xlabel('Time (s)');
ylabel('Roll Angle (deg)');
title('Roll Angle vs Time (Every Second)');

subplot(3,1,3);
plot(time_sec, -180-sec_OL(:,3), 'g');
xlabel('Time (s)');
ylabel('Yaw Angle (deg)');
title('Yaw Angle vs Time (Every Second)');

% 3. **每历元计算姿态角**
% 计算每个历元的姿态角
every_OL = [];
for i = 1:length(rawdata1)
    Cbn_epoch = CalCbn(g, gw, ggw, rawdata1(i,:), rawdata2(i,:));
    every_OL = [every_OL; Calangel(Cbn_epoch)];
end

% 绘制每历元的姿态角-时间图
time_epoch = (1:length(every_OL))';
figure;
subplot(3,1,1);
plot(time_epoch, every_OL(:,1), 'b');
xlabel('Time (Epoch)');
ylabel('Pitch Angle (deg)');
title('Pitch Angle vs Time (Every Epoch)');

subplot(3,1,2);
plot(time_epoch, every_OL(:,2), 'r');
xlabel('Time (Epoch)');
ylabel('Roll Angle (deg)');
title('Roll Angle vs Time (Every Epoch)');

subplot(3,1,3);
plot(time_epoch, -180-every_OL(:,3), 'g');
xlabel('Time (Epoch)');
ylabel('Yaw Angle (deg)');
title('Yaw Angle vs Time (Every Epoch)');
