% 添加函数路径
addpath(genpath('function'));  % 假设你的函数都在 function 文件夹下

%% 1. 加载配置
param = Param();
cfg = ProcessConfig_LJY();       

% 根据模式设置输出后缀，便于对比三种情况
mode_suffix = '';
if cfg.useodonhc
    if cfg.usetype == 1
        mode_suffix = '_ODO_only';
    elseif cfg.usetype == 2
        mode_suffix = '_NHC_only';
    elseif cfg.usetype == 3
        mode_suffix = '_ODO_NHC';
    end
end

%% 2. 数据加载
% IMU 数据（7列：时间, Δθx/y/z rad, Δvx/y/z m/s）
imu = LoadIMUIncreBin(cfg.imufilepath);  % 自行实现或用 fread
% 返回 [N×7] double

% GNSS 数据从 .pos 文件解析
gnss = load_pos_file(cfg.gnssfilepath, cfg.starttime, cfg.endtime, param);

% 如果没有速度，强制关闭速度更新
if all(isnan(gnss(:,5:7)))  % vn,ve,vd 全 NaN
    cfg.usegnssvel = false;
    disp('No velocity in .pos file, GNSS velocity update disabled.');
end

% ODO 数据（时间, speed m/s）
if cfg.useodonhc
    odo = load(cfg.odofilepath);  % [N×2] double
end

%% 3. 时间范围裁剪
imu = imu(imu(:,1) >= cfg.starttime & imu(:,1) <= cfg.endtime, :);
gnss = gnss(gnss(:,1) >= cfg.starttime & gnss(:,1) <= cfg.endtime, :);
if cfg.useodonhc
    odo = odo(odo(:,1) >= cfg.starttime & odo(:,1) <= cfg.endtime, :);
end

%% 4. 输出文件准备
navpath = fullfile(cfg.outputfolder, ['NavResult' mode_suffix '.nav']);
imuerrpath = fullfile(cfg.outputfolder, ['ImuError' mode_suffix '.txt']);
stdpath = fullfile(cfg.outputfolder, ['NavSTD' mode_suffix '.txt']);

navfp = fopen(navpath, 'wt');
imuerrfp = fopen(imuerrpath, 'wt');
stdfp = fopen(stdpath, 'wt');

%% 5. 初始化
[kf, navstate] = Initialize(cfg);

% 初始化索引
imu_idx = 2;          % 从第二条 IMU 开始
gnss_idx = 1;
odo_idx = 1;

lastimu = imu(1,:)';
thisimu = imu(1,:)';
imudt = 0;            % 初始 dt=0

% ODO 更新时间点（等间隔）
if cfg.useodonhc
    update_interval = 1 / cfg.odoupdaterate;
    next_odo_time = cfg.starttime + update_interval - mod(cfg.starttime, update_interval);
end

disp('Start GNSS/INS/ODO/NHC Processing...');

%% 6. 主循环
for i = 2:size(imu,1)
    
    lastimu = thisimu;
    thisimu = imu(i,:)';
    imudt = thisimu(1) - lastimu(1);
    
    if imudt <= 0
        continue;  % 时间异常
    end
    
    %% IMU 误差补偿（使用当前估计的偏置和比例因子）
    thisimu(2:4) = (thisimu(2:4) - navstate.gyrbias * imudt) ./ (1 + navstate.gyrscale);
    thisimu(5:7) = (thisimu(5:7) - navstate.accbias * imudt) ./ (1 + navstate.accscale);
    
    %% INS 机械编排 + 误差传播（预测步）
    navstate = InsMech(navstate, lastimu, thisimu);

    % 零速检测 (简易实现：使用配置中的阈值)
    imu_window = imu(max(1, i-cfg.zupt_window_size+1):i, :);
    acc_std = std(imu_window(:, 5:7) / imudt);
    gyr_std = std(imu_window(:, 2:4) / imudt) * param.R2D;
    
    is_static = all(acc_std < cfg.zupt_accel_threshold) && ...
                all(gyr_std < cfg.zupt_gyro_threshold);

    if cfg.usezupt && is_static
        kf = ZUPTUpdate(navstate, kf, cfg);
        [kf, navstate] = ErrorFeedback(kf, navstate);
    end
    
    if cfg.usezaru && is_static
        kf = ZARUUpdate(navstate, thisimu, kf, cfg, imudt);
        [kf, navstate] = ErrorFeedback(kf, navstate);
    end

    kf = InsPropagate(navstate, thisimu, imudt, kf, cfg.corrtime);
    
    %% GNSS 更新（位置 + 速度，可选）
    while gnss_idx <= size(gnss,1) && gnss(gnss_idx,1) <= thisimu(1)
        thisgnss = gnss(gnss_idx,:)';
        kf = GNSSUpdate(navstate, thisgnss, kf, cfg.antlever, cfg.usegnssvel, lastimu, imudt);
        [kf, navstate] = ErrorFeedback(kf, navstate);
        gnss_idx = gnss_idx + 1;
    end
    
    %% ODO/NHC 更新
    if cfg.useodonhc
        % 推进 ODO 索引到当前时间
        while odo_idx <= size(odo,1) && odo(odo_idx,1) < thisimu(1)
            odo_idx = odo_idx + 1;
        end
        
        % 到达更新时刻
        if thisimu(1) >= next_odo_time && odo_idx <= size(odo,1)
            % 简单平均或最近值（根据 GetOdoVel 实现）
            [odovel, valid] = GetOdoVel(odo(max(1,odo_idx-5):odo_idx,:), thisimu(1));
            
            if valid
                odonhc_vel = [odovel; 0; 0];  % 前向 + NHC 约束
                
                % 调用 ODO/NHC 更新
                kf = ODONHCUpdate(navstate, odonhc_vel, kf, cfg, thisimu, imudt);
                [kf, navstate] = ErrorFeedback(kf, navstate);
            end
            
            next_odo_time = next_odo_time + update_interval;
        end
    end
    
    %% 保存结果
    % 导航结果（11列，与 REF_NAV.nav 格式一致）
    fprintf(navfp, '%2d %12.6f %12.8f %12.8f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f\n', ...
        0, navstate.time, ...  % 周数假设0
        navstate.pos(1)*param.R2D, navstate.pos(2)*param.R2D, navstate.pos(3), ...
        navstate.vel(1), navstate.vel(2), navstate.vel(3), ...
        navstate.att(1)*param.R2D, navstate.att(2)*param.R2D, navstate.att(3)*param.R2D);
    
    % IMU 误差（单位转换）
    imuerr = [navstate.time; ...
              navstate.gyrbias*param.R2D*3600; ...
              navstate.accbias*1e5; ...
              navstate.gyrscale*1e6; ...
              navstate.accscale*1e6];
    if kf.RANK >= 22
        imuerr = [imuerr; kf.x(22)*1e6];  % ODO scale ppm
    end
    fprintf(imuerrfp, '%12.6f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f\n', imuerr);
    
    % 状态 STD
    stdvec = [navstate.time; sqrt(diag(kf.P))];
    stdvec(8:10) = stdvec(8:10) * param.R2D;          % att rad → deg
    stdvec(11:13) = stdvec(11:13) * param.R2D * 3600; % gyrbias rad/s → deg/h
    stdvec(14:16) = stdvec(14:16) * 1e5;              % accbias m/s² → mGal
    stdvec(17:22) = stdvec(17:22) * 1e6;              % scale ppm
    fprintf(stdfp, '%12.6f ', stdvec);
    fprintf(stdfp, '\n');
    
    % 进度显示
    if mod(i, 500) == 0
        fprintf('Processing %.1f%%\n', i/size(imu,1)*100);
    end
end

fclose('all');
disp('Processing Finished! Results saved in:');
disp(cfg.outputfolder);

% 可选：调用绘图函数对比 REF_NAV.nav
% plot_nav_result(navpath, 'REF_NAV.nav');
