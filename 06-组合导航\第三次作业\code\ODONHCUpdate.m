function kf = ODONHCUpdate(navstate, odovel_meas, kf, cfg, thisimu, dt)
    Cnb = navstate.cbn;
    Cbn = Cnb';
    Cbv = cfg.cbv;
    v_n = navstate.vel;
    
    % 车体系速度预测 (v = C_b^v * C_n^b * v_n)
    v_v = Cbv * Cbn * v_n; 
    
    Z = []; H = []; R_diag = [];

    % 1. ODO 更新 (Task 2.b)
    if cfg.usetype == 1 || cfg.usetype == 3
        Z = [Z; odovel_meas * (1 + navstate.odoscale) - v_v(1)];
        h_odo = zeros(1, kf.RANK);
        h_odo(4:6) = Cbv(1,:) * Cbn; % 对速度误差偏导
        h_odo(7:9) = -Cbv(1,:) * Cbn * skew(v_n); % 对姿态误差偏导 (关键修复)
        h_odo(22) = -odovel_meas; % 对比例因子偏导
        H = [H; h_odo];
        R_diag = [R_diag; cfg.odo_noise^2];
    end

    % 2. NHC 更新 (Task 2.a)
    if cfg.usetype == 2 || cfg.usetype == 3
        Z = [Z; [0; 0] - v_v(2:3)];
        h_nhc = zeros(2, kf.RANK);
        h_nhc(:, 4:6) = Cbv(2:3, :) * Cbn;
        h_nhc(:, 7:9) = -Cbv(2:3, :) * Cbn * skew(v_n);
        H = [H; h_nhc];
        R_diag = [R_diag; cfg.nhc_noise(1)^2; cfg.nhc_noise(2)^2];
    end

    % Kalman 更新
    R = diag(R_diag);
    K = kf.P * H' / (H * kf.P * H' + R);
    kf.x = kf.x + K * (Z - H * kf.x);
    kf.P = (eye(kf.RANK) - K*H) * kf.P * (eye(kf.RANK) - K*H)' + K*R*K';
end