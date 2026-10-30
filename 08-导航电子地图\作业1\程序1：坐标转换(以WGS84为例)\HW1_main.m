%刘峻琰2023302141249第一次导航电子地图作业_HW1
a = 6378137;
e2 = 0.00669437999013;
X = -1338215.7700;
Y = 5277079.9330;
Z = 3313658.6930;

% 1) XYZ → BLH
[B, L, H] = xyz2blh(X, Y, Z, a, e2);
fprintf('1)  (B, L, H): B=%.8f°   L=%.8f°   H=%.3f m\n', rad2deg(B), rad2deg(L), H);

% 2) 椭球面投影点
[X0, Y0, Z0] = blh2xyz(B, L, 0, a, e2);
fprintf('2)  XYZ: %.3f   %.3f   %.3f m\n', X0, Y0, Z0);

% 3) 正反验算
[Bc, Lc, Hc] = xyz2blh(X0, Y0, Z0, a, e2);
fprintf('3) 较差: dB=%.3f"   dL=%.3f"   dH=%.3f mm\n', rad2deg(Bc-B)*3600, rad2deg(Lc-L)*3600, Hc*1000);



function [B, L, H] = xyz2blh(X, Y, Z, a, e2)
    L = atan2(Y, X);
    p = sqrt(X^2 + Y^2);
    B = atan(Z / p);               
    for i = 1:20
        N = a / sqrt(1 - e2*sin(B)^2);
        B_new = atan( (Z + N*e2*sin(B)) / p );
        if abs(B_new - B) < 1e-12, break; end
        B = B_new;
    end
    N = a / sqrt(1 - e2*sin(B)^2);
    H = p / cos(B) - N;
end

function [X, Y, Z] = blh2xyz(B, L, H, a, e2)
    N = a / sqrt(1 - e2*sin(B)^2);
    X = (N + H) * cos(B) * cos(L);
    Y = (N + H) * cos(B) * sin(L);
    Z = (N*(1-e2) + H) * sin(B);
end