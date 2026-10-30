clear; close all; clc;
imu_file  = 'IMU.nav';   
ref_file  = 'ref.nav';  
ARMb = [-0.64; -0.628; 2.122];      % 杆臂，在 b 系下 
inst_angles_deg = [0, -0.424, -1.2];% (roll,pitch,
deg2rad = pi/180;

imu = load(imu_file);   
ref = load(ref_file);

% IMU
week_imu = imu(:,1);
sec_imu  = imu(:,2);
lat_imu  = imu(:,3);
lon_imu  = imu(:,4);
hgt_imu  = imu(:,5);
Vn_imu   = imu(:,6);
Ve_imu   = imu(:,7);
Vd_imu   = imu(:,8);
roll_imu = imu(:,9);
pitch_imu= imu(:,10);
yaw_imu  = imu(:,11);
wx_deg   = imu(:,12);%b相对于n投影到b
wy_deg   = imu(:,13);
wz_deg   = imu(:,14);

% Reference
week_ref = ref(:,1);
sec_ref  = ref(:,2);
lat_ref  = ref(:,3);
lon_ref  = ref(:,4);
hgt_ref  = ref(:,5);
Vn_ref   = ref(:,6);
Ve_ref   = ref(:,7);
Vd_ref   = ref(:,8);
roll_ref = ref(:,9);
pitch_ref= ref(:,10);
yaw_ref  = ref(:,11);

N = size(imu,1);
if size(ref,1) ~= N
    warning('IMU 和 ref 行数不一致 (%d vs %d)，将按最小行数处理', N, size(ref,1));
    N = min(N, size(ref,1));
end

% WGS84
a = 6378137.0;
f = 1/298.257223563;
e2 = f*(2-f);

lla2ecef = @(lat,lon,h) lla2ecef_func(lat,lon,h,a,e2);
ecef2lla = @(x,y,z) ecef2lla_func(x,y,z,a,e2);
eul2dcm_func = @(roll,pitch,heading) eul2dcm_impl(roll,pitch,heading);

p_imu_e = zeros(N,3);
p_ref_e = zeros(N,3);
p_v_e   = zeros(N,3);
v_v_n   = zeros(N,3);
v_ref_n = zeros(N,3);
euler_v_deg = zeros(N,3);  
euler_ref_deg = zeros(N,3);

% 角速度
omega_b = deg2rad * [wx_deg(1:N), wy_deg(1:N), wz_deg(1:N)]; % Nx3

% 安装角
roll_inst = inst_angles_deg(1) * deg2rad;
pitch_inst= inst_angles_deg(2) * deg2rad;
yaw_inst  = inst_angles_deg(3) * deg2rad;
% 计算 Cvb 

C_v_b_inst = eul2dcm_func(roll_inst, pitch_inst, yaw_inst); % b -> v
C_b_v_inst = C_v_b_inst'; %v -> b

for i=1:N
    lat = lat_imu(i) * deg2rad;
    lon = lon_imu(i) * deg2rad;
    h   = hgt_imu(i);
    % IMU 在 ECEF
    p_imu_e(i,:) = lla2ecef(lat,lon,h);
    % 参考点在 ECEF
    p_ref_e(i,:) = lla2ecef(lat_ref(i)*deg2rad, lon_ref(i)*deg2rad, hgt_ref(i));
    
    % C_n_b
    C_n_b = eul2dcm_func( roll_imu(i)*deg2rad, pitch_imu(i)*deg2rad, yaw_imu(i)*deg2rad );
    
    % ecef 2 n 
    C_n_e = n_from_e(lat, lon);
    C_e_n = C_n_e';        

    p_v_e(i,:) = p_imu_e(i,:).' + (C_e_n * (C_n_b * ARMb)) ;

    v_imu_n = [Vn_imu(i); Ve_imu(i); Vd_imu(i)];
    omega_b_i = omega_b(i,:).'; 
    v_rel_b = cross(omega_b_i, ARMb); 
    v_rel_n = C_n_b * v_rel_b;       
    v_v_n(i,:) = (v_imu_n + v_rel_n).';
 
    C_n_v = C_n_b * C_b_v_inst; % v -> n
    % 由 DCM 得到欧拉角
    [roll_v, pitch_v, yaw_v] = dcm2eul_impl(C_n_v);
    euler_v_deg(i,:) = [roll_v, pitch_v, yaw_v] * (180/pi);
    
    % 参考的姿态直接读取
    euler_ref_deg(i,:) = [roll_ref(i), pitch_ref(i), yaw_ref(i)];
    
    % ref在 n 系
    v_ref_n(i,:) = [Vn_ref(i), Ve_ref(i), Vd_ref(i)];
end

%% ------------- 误差计算：在 ENU下比较 ----------------

pos_err_n = zeros(N,3); % N,E,D 
for i=1:N
    dp_e = p_v_e(i,:).' - p_ref_e(i,:).';
    lat = lat_ref(i)*deg2rad;
    lon = lon_ref(i)*deg2rad;
    C_n_e_ref = n_from_e(lat, lon); % ECEF -> NED
    pos_err_n(i,:) = (C_n_e_ref * dp_e).'; % NED
end
% 为了绘图更直观，把 NED 的第三分量转换为 Up（-D）
pos_err_enu = [ pos_err_n(:,2), pos_err_n(:,1), -pos_err_n(:,3) ]; % [E, N, U]

% 速度误差（直接 n 系）
vel_err_n = v_v_n - v_ref_n; % N,E,D
vel_err_enu = [ vel_err_n(:,2), vel_err_n(:,1), -vel_err_n(:,3) ];

% 姿态误差
att_err = wrapTo180(euler_v_deg - euler_ref_deg); % deg

%% ------------- 统计量 ----------------
rmse_pos = sqrt(mean(pos_err_enu.^2));
rmse_vel = sqrt(mean(vel_err_enu.^2));
rmse_att = sqrt(mean(att_err.^2));

fprintf('位置误差 RMSE (E,N,U) [m]: %.4f, %.4f, %.4f\n', rmse_pos(1), rmse_pos(2), rmse_pos(3));
fprintf('速度误差 RMSE (E,N,U) [m/s]: %.4f, %.4f, %.4f\n', rmse_vel(1), rmse_vel(2), rmse_vel(3));
fprintf('姿态误差 RMSE (roll,pitch,yaw) [deg]: %.4f, %.4f, %.4f\n', rmse_att(1), rmse_att(2), rmse_att(3));

%% ------------- 绘图：位置误差时序（E,N,U） ----------------
t = sec_ref(1:N) - sec_ref(1);
figure; 
subplot(3,1,1); plot(t, pos_err_enu(:,1)); grid on; ylabel('E error (m)'); title('Position error in ENU (E,N,U)');
subplot(3,1,2); plot(t, pos_err_enu(:,2)); grid on; ylabel('N error (m)');
subplot(3,1,3); plot(t, pos_err_enu(:,3)); grid on; ylabel('U error (m)'); xlabel('time (s)');


%% ------------- 速度误差与姿态误差时序 ----------------
figure;
subplot(3,1,1); plot(t, vel_err_enu(:,1)); grid on; ylabel('E vel err (m/s)'); title('Velocity error in ENU');
subplot(3,1,2); plot(t, vel_err_enu(:,2)); grid on; ylabel('N vel err (m/s)');
subplot(3,1,3); plot(t, vel_err_enu(:,3)); grid on; ylabel('U vel err (m/s)'); xlabel('time (s)');

figure;
subplot(3,1,1); plot(t, att_err(:,1)); grid on; ylabel('roll err (deg)'); title('Attitude error (deg)');
subplot(3,1,2); plot(t, att_err(:,2)); grid on; ylabel('pitch err (deg)');
subplot(3,1,3); plot(t, att_err(:,3)); grid on; ylabel('yaw err (deg)'); xlabel('time (s)');


function ecef = lla2ecef_func(lat, lon, h, a, e2)
    % 输入 lat, lon 为弧度
    Nphi = a ./ sqrt(1 - e2 .* sin(lat).^2);
    x = (Nphi + h) .* cos(lat) .* cos(lon);
    y = (Nphi + h) .* cos(lat) .* sin(lon);
    z = (Nphi .* (1 - e2) + h) .* sin(lat);
    ecef = [x, y, z];
end

function [lat, lon, h] = ecef2lla_func(x, y, z, a, e2)

    lon = atan2(y, x);
    p = sqrt(x.^2 + y.^2);
    lat = atan2(z, p*(1 - e2)); % 初始值
    for k=1:5
        N = a ./ sqrt(1 - e2 .* sin(lat).^2);
        h = p ./ cos(lat) - N;
        lat = atan2(z, p*(1 - e2*(N./(N + h))));
    end
end

function C = eul2dcm_impl(roll, pitch, heading)

    cr = cos(roll); cp = cos(pitch); ch = cos(heading);
    sr = sin(roll); sp = sin(pitch); sh = sin(heading);
    C = zeros(3,3);
    C(1,1) = cp * ch ;
    C(1,2) = -cr*sh + sr*sp*ch;
    C(1,3) = sr*sh + cr*sp*ch ;
    C(2,1) = cp * sh;
    C(2,2) = cr*ch + sr*sp*sh;
    C(2,3) = -sr * ch + cr * sp * sh;
    C(3,1) = - sp;
    C(3,2) = sr * cp;
    C(3,3) = cr * cp;
end

function [roll,pitch,heading] = dcm2eul_impl(dc)

    pitch = atan(-dc(3,1)/sqrt(dc(3,2)^2 + dc(3,3)^2));
    if dc(3,1) <= -0.999
        roll = NaN;
        heading = atan2((dc(2,3)-dc(1,2)),(dc(1,3)+dc(2,2)));
    elseif dc(3,1) >= 0.999
        roll = NaN;
        heading = pi + atan2((dc(2,3)+dc(1,2)),(dc(1,3)-dc(2,2)));
    else
        roll = atan2(dc(3,2), dc(3,3));
        heading = atan2(dc(2,1), dc(1,1));
    end
end

function C_n_e = n_from_e(lat, lon)

    sl = sin(lat); cl = cos(lat);
    so = sin(lon); co = cos(lon);
    C_n_e = [ -sl*co, -sl*so,  cl;
              -so,     co,     0;
              -cl*co, -cl*so, -sl ];
end

function ang = wrapTo180(ang)
    ang = mod(ang + 180, 360) - 180;
end
