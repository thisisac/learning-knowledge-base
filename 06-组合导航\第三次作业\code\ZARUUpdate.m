function kf = ZARUUpdate(navstate, kf, cfg, thisimu, dt)
    % 零航向率更新
    gyro_b = thisimu(2:4)/dt;
    Z = [0;0;0] - (gyro_b - navstate.gyrbias);
    H = zeros(3, kf.RANK);
    H(:, 10:12) = eye(3); 
    R = diag(cfg.zaru_noise.^2);
    K = kf.P * H' / (H * kf.P * H' + R);
    kf.x = kf.x + K * Z;
    kf.P = (eye(kf.RANK) - K*H) * kf.P * (eye(kf.RANK) - K*H)' + K*R*K';
end