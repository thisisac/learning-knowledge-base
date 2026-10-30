%要改
function [kf, navstate] = ErrorFeedback(kf, navstate)

    % position and velocity 位置误差反馈
    DR = diag([navstate.Rm + navstate.pos(3), (navstate.Rn + navstate.pos(3))*cos(navstate.pos(1)), -1]);
    DR_inv = inv(DR);
    % DR矩阵
    navstate.pos = navstate.pos - DR_inv * kf.x(1:3, 1);
    % 速度误差反馈
    navstate.vel = navstate.vel - kf.x(4:6, 1);

    % attitude 姿态误差反馈
    qpn = rotvec2quat(kf.x(7:9, 1));
    navstate.qbn = quatProd(qpn, navstate.qbn);
    navstate.cbn = quat2dcm(navstate.qbn);
    navstate.att = dcm2euler(navstate.cbn);

    % imu error IMU误差反馈（直接加上估计值）
    navstate.gyrbias  = navstate.gyrbias  + kf.x(10:12, 1);   % 校正陀螺零偏
    navstate.accbias  = navstate.accbias  + kf.x(13:15, 1);   % 校正加表零偏
    navstate.gyrscale = navstate.gyrscale + kf.x(16:18, 1);   % 校正陀螺比例因子
    navstate.accscale = navstate.accscale + kf.x(19:21, 1);   % 校正加表比例因子

    param = Param(); % 加载地球参数

    if kf.RANK >= 22
        % 检查字段是否存在，不存在则初始化为0
        if ~isfield(navstate, 'odoscale')
            navstate.odoscale = 0; 
        end
        navstate.odoscale = navstate.odoscale + kf.x(22);
    end

    % update some parameters 更新位置相关的参数

    % 用新位置重新算曲率半径Rm
    [navstate.Rm, navstate.Rn] = getRmRn(navstate.pos(1), param);
    % 用新位置算重力
    navstate.gravity = getGravity(navstate.pos);

    % reset state vector
    kf.x = zeros(kf.RANK, 1);
end