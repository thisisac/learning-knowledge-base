function imu = read_imu(fname)

    fid = fopen(fname, 'r');
    if fid == -1, error('Cannot open: %s', fname); end

    t_week = []; t_sow = [];
    accZ = []; accY = []; accX = [];
    gyrZ = []; gyrY = []; gyrX = [];

    while true
        line = fgetl(fid);
        if ~ischar(line), break; end
        if isempty(line), continue; end
        if ~startsWith(line, '#RAWIMUA'), continue; end

        % 分号后为 IMU 数字段
        p = strfind(line, ';');
        if isempty(p), continue; end
        seg = line(p(1)+1:end);
        star = strfind(seg, '*');
        if ~isempty(star), seg = seg(1:star(1)-1); end

        tok = split(strtrim(seg), ',');
        if numel(tok) < 9, continue; end

        week = str2double(tok{1});
        sow  = str2double(tok{2});
        % tok{3} 为 status，忽略
        zAcc = str2double(tok{4});
        yAcc = str2double(tok{5});
        xAcc = str2double(tok{6});
        zGyr = str2double(tok{7});
        yGyr = str2double(tok{8});
        xGyr = str2double(tok{9});

        t_week(end+1,1) = week; 
        t_sow (end+1,1) = sow;  
        accZ  (end+1,1) = zAcc; 
        accY  (end+1,1) = yAcc; 
        accX  (end+1,1) = xAcc; 
        gyrZ  (end+1,1) = zGyr; 
        gyrY  (end+1,1) = yGyr; 
        gyrX  (end+1,1) = xGyr; 
    end
    fclose(fid);

    % 采样率与dt
    dt = 0.01;
    fq = 100.0;

    % 加速度:  fq/655360 * 9.80665
    % 陀螺仪:  fq/160849.543863
    acc_scale  = fq/655360;
    gyro_scale = fq/160849.543863;
    deg2rad = pi/180;

    az = -accZ * acc_scale ;   % N
    ax = -accX * acc_scale ;   % E
    ay =  accY * acc_scale ;   % D
    gz = -gyrZ * gyro_scale * deg2rad ;  
    gx = -gyrX * gyro_scale * deg2rad ; 
    gy =  gyrY * gyro_scale * deg2rad ; 

    dVel   = [ax, ay, az] .* dt;     % m/s
    dTheta = [gx, gy, gz] .* dt;     % rad

    imu = [t_sow, dTheta, dVel];
end

