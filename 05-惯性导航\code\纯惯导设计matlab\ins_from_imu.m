function ins = ins_from_imu(imu, lat0, lon0, h0, vN0, vE0, vD0, roll0, pitch0, yaw0, t0)
%INS_FROM_IMU 惯导(NED)机理解算，四元数(标量在前)
% 输入:
%   imu [N x 7]: [t, dTheta_x, dTheta_y, dTheta_z, dVel_x, dVel_y, dVel_z]
%   lat0, lon0, h0: 初始位置(rad, rad, m)
%   vN0, vE0, vD0 : 初始速度(m/s)
%   roll0, pitch0, yaw0: 初始姿态(rad)
%   t0: 起始时间戳(s)，仅解算 imu(:,1) > t0
% 输出:
%   ins.data  [M x 10]: [t, lat_deg, lon_deg, h_m, vN, vE, vD, roll_deg, pitch_deg, yaw_deg]
%   ins.table table

    % 常量
    we = 7.292115e-5; a = 6378137.0; e = 0.08181919104; e2 = e*e; rad2deg = 180/pi;
    enableZUPT = false; zuptThresh = 0.1; fs = 200;

    % 起始索引与预分配
    startIdx = find(imu(:,1) > t0, 1, 'first');
    if isempty(startIdx), ins.data = zeros(0,10); ins.table = array2table(ins.data); return; end
    startIdx = max(2, startIdx);
    M = size(imu,1) - startIdx + 1;
    res = zeros(M, 10);

    % 初始状态
    q_prev = eulerToQuat(roll0, pitch0, yaw0);
    lat_p = lat0; lon_p = lon0; h_p = h0; vN_p = vN0; vE_p = vE0; vD_p = vD0;
    lat_pp = lat0; h_pp = h0; vN_pp = vN0; vE_pp = vE0; vD_pp = vD0;

    I3 = eye(3);
    out = 1;
    for i = startIdx:size(imu,1)
        dt = imu(i,1) - imu(i-1,1);
        dTh   = imu(i,  2:4)'; dTh_1 = imu(i-1,2:4)';
        dVel  = imu(i,  5:7)'; dVel_1= imu(i-1,5:7)';

        % 姿态更新
        Mvec   = dTh + (1/12)*cross(dTh_1, dTh);
        q_bk   = rvToQuat(Mvec);
        [w_ie_p, w_en_p] = earthRates(lat_p, h_p, vE_p, vN_p, we, a, e2);
        Nvec   = (w_ie_p + w_en_p) * dt;
        q_nkm1_nk = rvToQuat(Nvec); q_nkm1_nk(2:4) = -q_nkm1_nk(2:4);
        q_cur  = quatMultiply(q_nkm1_nk, quatMultiply(q_prev, q_bk));

        % 速度更新
        lat_h = lat_p + 0.5*(lat_p - lat_pp);
        h_h   = h_p   + 0.5*(h_p   - h_pp);
        vN_h  = vN_p  + 0.5*(vN_p  - vN_pp);
        vE_h  = vE_p  + 0.5*(vE_p  - vE_pp);
        vD_h  = vD_p  + 0.5*(vD_p  - vD_pp);
        [w_ie_h, w_en_h] = earthRates(lat_h, h_h, vE_h, vN_h, we, a, e2);
        g_n_h = gravityN(lat_h, h_h);
        dV_gcor = (g_n_h - cross(2*w_ie_h + w_en_h, [vN_h; vE_h; vD_h])) * dt;

        C_bn_prev = quatToDcm(q_prev);
        N_h = (w_ie_h + w_en_h) * dt;
        dV_fb_b = dVel + 0.5*cross(dTh, dVel) + (1/12)*(cross(dTh_1, dVel) + cross(dVel_1, dTh));
        dV_fb_n = (I3 - 0.5*skew(N_h)) * (C_bn_prev * dV_fb_b);
        vN = vN_p + dV_fb_n(1) + dV_gcor(1);
        vE = vE_p + dV_fb_n(2) + dV_gcor(2);
        vD = vD_p + dV_fb_n(3) + dV_gcor(3);

        % ZUPT（可选）
        if enableZUPT
            acc_est = dVel * fs; g_now = gravityN(lat_p, h_p);
            if abs(acc_est(1)) < zuptThresh && abs(acc_est(2)) < zuptThresh && abs(acc_est(3) + g_now(3)) < zuptThresh
                vN = 0; vE = 0; vD = 0;
            end
        end

        % 位置更新
        h   = h_p - 0.5*(vD_p + vD)*dt; hAve = 0.5*(h + h_p);
        RM_p = a*(1 - e2) / sqrt((1 - e2*sin(lat_p)^2)^3);
        lat = lat_p + (vN + vN_p) / (2*(RM_p + hAve)) * dt;
        RN_h = a / sqrt(1 - e2*sin(lat_h)^2);
        latAve = 0.5*(lat + lat_p);
        lon = lon_p + (vE + vE_p) / (2*(RN_h + hAve) * cos(latAve)) * dt;

        % 输出
        [roll, pitch, yaw] = quatToEuler(q_cur);
        res(out,:) = [imu(i,1), lat*rad2deg, lon*rad2deg, h, vN, vE, vD, roll*rad2deg, pitch*rad2deg, yaw*rad2deg];
        out = out + 1;

        % 滚动
        lat_pp = lat_p; h_pp = h_p; vN_pp = vN_p; vE_pp = vE_p; vD_pp = vD_p;
        lat_p = lat; lon_p = lon; h_p = h; vN_p = vN; vE_p = vE; vD_p = vD;
        q_prev = q_cur;
    end

    % 打包
    ins.data = res;
    ins.table = array2table(res, 'VariableNames', ...
        {'t','lat_deg','lon_deg','h_m','vN','vE','vD','roll_deg','pitch_deg','yaw_deg'});
end

% ======== 简单辅助函数 ========

function [w_ie, w_en] = earthRates(lat, h, vE, vN, we, a, e2)
    s = sin(lat);
    RN = a / sqrt(1 - e2*s*s);
    RM = a*(1 - e2) / sqrt((1 - e2*s*s)^3);
    w_ie = [we*cos(lat); 0; -we*sin(lat)];
    w_en = [vE/(RN + h); -vN/(RM + h); -vE*tan(lat)/(RN + h)];
end

function g_n = gravityN(lat, h)
    s = sin(lat);
    g0 = 9.7803267715 * (1 + 0.0052790414*s^2 + 0.0000232718*s^4);
    g  = g0 - (3.087691089e-6 - 4.397731e-9*s^2)*h + 0.721e-12*h*h;
    g_n = [0; 0; g];
end

function S = skew(v)
    S = [ 0   -v(3)  v(2);
          v(3)  0   -v(1);
         -v(2) v(1)   0 ];
end

function C = quatToDcm(q)
    q0=q(1); q1=q(2); q2=q(3); q3=q(4);
    C = zeros(3,3);
    C(1,1)= q0*q0+q1*q1-q2*q2-q3*q3;
    C(1,2)= 2*(q1*q2 - q0*q3);
    C(1,3)= 2*(q1*q3 + q0*q2);
    C(2,1)= 2*(q1*q2 + q0*q3);
    C(2,2)= q0*q0-q1*q1+q2*q2-q3*q3;
    C(2,3)= 2*(q2*q3 - q0*q1);
    C(3,1)= 2*(q1*q3 - q0*q2);
    C(3,2)= 2*(q2*q3 + q0*q1);
    C(3,3)= q0*q0-q1*q1-q2*q2+q3*q3;
end

function [r,p,y] = quatToEuler(q)
    C = quatToDcm(q);
    p = asin(-C(3,1));
    r = atan2(C(3,2), C(3,3));
    y = atan2(C(2,1), C(1,1));
end


function q = eulerToQuat(roll, pitch, yaw)
    sr=sin(roll/2); cr=cos(roll/2);
    sp=sin(pitch/2); cp=cos(pitch/2);
    sy=sin(yaw/2); cy=cos(yaw/2);
    q = [cr*cp*cy + sr*sp*sy;
         sr*cp*cy - cr*sp*sy;
         cr*sp*cy + sr*cp*sy;
         cr*cp*sy - sr*sp*cy];
    q = q/norm(q);
end

function q = rvToQuat(rv)
    n = norm(rv); q = zeros(4,1);
    q(1) = cos(0.5*n);
    if n > 0, q(2:4) = (sin(0.5*n)/n)*rv; end
    q = q/norm(q);
end

function r = quatMultiply(p, q)
    r = zeros(4,1);
    r(1)=p(1)*q(1)-p(2)*q(2)-p(3)*q(3)-p(4)*q(4);
    r(2)=p(1)*q(2)+p(2)*q(1)+p(3)*q(4)-p(4)*q(3);
    r(3)=p(1)*q(3)-p(2)*q(4)+p(3)*q(1)+p(4)*q(2);
    r(4)=p(1)*q(4)+p(2)*q(3)-p(3)*q(2)+p(4)*q(1);
    r = r/norm(r);
end