function kf = ZUPTUpdate(navstate, kf, cfg)
    % 零速更新
    Z = [0;0;0] - navstate.vel; 
    H = zeros(3, kf.RANK);
    H(:, 4:6) = eye(3);
    R = diag(cfg.zupt_noise.^2);
    K = kf.P * H' / (H * kf.P * H' + R);
    kf.x = kf.x + K * Z;
    kf.P = (eye(kf.RANK) - K*H) * kf.P * (eye(kf.RANK) - K*H)' + K*R*K';
end