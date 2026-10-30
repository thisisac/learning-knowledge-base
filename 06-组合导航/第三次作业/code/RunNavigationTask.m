function RunNavigationTask(cfg, mode_suffix)
    param = Param();
    
    % 加载数据
    imu = LoadIMUIncreBin(cfg.imufilepath);
    gnss = load_pos_file(cfg.gnssfilepath, cfg.starttime, cfg.endtime, param);
    if cfg.useodonhc, odo = load(cfg.odofilepath); end
    
    % 时间裁剪
    imu = imu(imu(:,1) >= cfg.starttime & imu(:,1) <= cfg.endtime, :);
    gnss = gnss(gnss(:,1) >= cfg.starttime & gnss(:,1) <= cfg.endtime, :);
    
    % 输出文件准备
    navpath = fullfile(cfg.outputfolder, ['NavResult_' mode_suffix '.nav']);
    navfp = fopen(navpath, 'wt');
    
    % 初始化
    [kf, navstate] = Initialize(cfg);
    imu_idx = 2; gnss_idx = 1; odo_idx = 1;
    thisimu = imu(1,:)';
    if cfg.useodonhc, next_odo_time = cfg.starttime + 1/cfg.odoupdaterate; end

    % 主循环
    for i = 2:size(imu,1)
        lastimu = thisimu;
        thisimu = imu(i,:)';
        imudt = thisimu(1) - lastimu(1);
        if imudt <= 0, continue; end
        
        % 补偿 + 机械编排
        thisimu(2:4) = (thisimu(2:4) - navstate.gyrbias * imudt) ./ (1 + navstate.gyrscale);
        thisimu(5:7) = (thisimu(5:7) - navstate.accbias * imudt) ./ (1 + navstate.accscale);
        navstate = InsMech(navstate, lastimu, thisimu);
        
        % 零速/零航向率更新
        if (cfg.usezupt || cfg.usezaru)
            imu_window = imu(max(1, i-cfg.zupt_window_size+1):i, :);
            is_static = all(std(imu_window(:, 5:7)/imudt) < cfg.zupt_accel_threshold) && ...
                        all(std(imu_window(:, 2:4)/imudt)*param.R2D < cfg.zupt_gyro_threshold);
            if is_static
                if cfg.usezupt, kf = ZUPTUpdate(navstate, kf, cfg); end
                if cfg.usezaru, kf = ZARUUpdate(navstate, thisimu, kf, cfg, imudt); end
                [kf, navstate] = ErrorFeedback(kf, navstate);
            end
        end
        
        % 误差传播
        kf = InsPropagate(navstate, thisimu, imudt, kf, cfg.corrtime);
        
        % GNSS 更新
        while gnss_idx <= size(gnss,1) && gnss(gnss_idx,1) <= thisimu(1)
            kf = GNSSUpdate(navstate, gnss(gnss_idx,:)', kf, cfg.antlever, cfg.usegnssvel, lastimu, imudt);
            [kf, navstate] = ErrorFeedback(kf, navstate);
            gnss_idx = gnss_idx + 1;
        end
        
        % ODO/NHC 更新
        if cfg.useodonhc && thisimu(1) >= next_odo_time
            while odo_idx <= size(odo,1) && odo(odo_idx,1) < thisimu(1), odo_idx = odo_idx + 1; end
            [odovel, valid] = GetOdoVel(odo(max(1,odo_idx-5):min(size(odo,1),odo_idx+5),:), thisimu(1));
            if valid
                kf = ODONHCUpdate(navstate, [odovel;0;0], kf, cfg, thisimu, imudt);
                [kf, navstate] = ErrorFeedback(kf, navstate);
            end
            next_odo_time = next_odo_time + 1/cfg.odoupdaterate;
        end
        
        % 保存结果
        fprintf(navfp, '%2d %12.6f %12.8f %12.8f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f\n', ...
            0, navstate.time, navstate.pos(1)*param.R2D, navstate.pos(2)*param.R2D, navstate.pos(3), ...
            navstate.vel(1), navstate.vel(2), navstate.vel(3), ...
            navstate.att(1)*param.R2D, navstate.att(2)*param.R2D, navstate.att(3)*param.R2D);
    end
    fclose(navfp);
end