function main()
    % 文件名
    imu_file = '20251031.ASC';
    nav_file = '2023302141044.nav';

    % 读取数据
    imu = read_imu(imu_file);     % [t_sow, dThx, dThy, dThz, dVx, dVy, dVz]
    nav = read_nav(nav_file);     % 结构体，含 nav.sow、lat/long/ect

    fprintf('IMU样本数: %d, 列: %d\n', size(imu,1), size(imu,2));
    fprintf('NAV样本数: %d, 列: %d\n', size(nav.mat,1), size(nav.mat,2));

    % 初始条件（取参考第一历元）
    deg2rad = pi/180;
    lat0   = nav.lat_deg(1)   * deg2rad;
    lon0   = nav.lon_deg(1)   * deg2rad;
    h0     = nav.h_m(1);
    vN0    = nav.vN(1);
    vE0    = nav.vE(1);
    vD0    = nav.vD(1);
    roll0  = nav.roll_deg(1)  * deg2rad;
    pitch0 = nav.pitch_deg(1) * deg2rad;
    yaw0   = wrapToPiDeg(nav.heading_deg(1)) * deg2rad; % 转到[-180,180)再转弧度
    t0     = nav.sow(1); % 与 imu 第一列同为 sow

    % INS 解算
    ins = ins_from_imu(imu, lat0, lon0, h0, vN0, vE0, vD0, roll0, pitch0, yaw0, t0);
    presult = ins.data; 

    ref10 = [nav.sow, nav.lat_deg, nav.lon_deg, nav.h_m, ...
             nav.vN, nav.vE, nav.vD, nav.roll_deg, nav.pitch_deg, nav.heading_deg];

    % 时间同步
    t_ins = round(presult(:,1)*1000);
    t_ref = round(ref10(:,1)*1000);
    [~, ia, ib] = intersect(t_ins, t_ref, 'stable');

    presult_a = presult(ia, :);
    ref_a     = ref10(ib, :);

    ref_a(:,10) = arrayfun(@(x) wrapToPiDeg(x), ref_a(:,10));
    
    % 计算差值
    diff = presult_a - ref_a;
    diff(:,1) = presult_a(:,1); % 恢复时间戳

    % 绘图
    % plot_ins_results(presult_a, ref_a, diff);
    draw2(presult_a, ref_a, diff, nav);
end

function ang = wrapToPiDeg(angDeg)
    ang = mod(angDeg + 180, 360) - 180;
end