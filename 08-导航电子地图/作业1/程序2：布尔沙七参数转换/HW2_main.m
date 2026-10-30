%刘峻琰2023302141249第一次导航电子地图作业_HW2
XA = [-2358683.2015; -2476191.1223; -2366980.0034; -2479709.4985];
YA = [ 5153007.9355;  5220545.6468;  5241954.8314;  5074389.1912];
ZA = [ 2916777.1882;  2692588.9666;  2748152.9313;  2953715.9067];

XB = [-2358682.9205; -2476190.8413; -2366979.7223; -2479709.2174];
YB = [ 5153007.9970;  5220545.7090;  5241954.8932;  5074389.2528];
ZB = [ 2916777.2489;  2692589.0279;  2748152.9923;  2953715.9674];

L = [XB-XA; YB-YA; ZB-ZA];   % 12×1

A = zeros(12,7);
for i = 1:4
    idx = 3*(i-1);
    % X行
    A(idx+1,1) = 1;
    A(idx+1,5) = -ZA(i);   % ey
    A(idx+1,6) = YA(i);    % ez
    A(idx+1,7) = XA(i);    % k
    % Y行
    A(idx+2,2) = 1;
    A(idx+2,4) = ZA(i);    % ex
    A(idx+2,6) = -XA(i);   % ez
    A(idx+2,7) = YA(i);
    % Z行
    A(idx+3,3) = 1;
    A(idx+3,4) = -YA(i);   % ex
    A(idx+3,5) = XA(i);    % ey
    A(idx+3,7) = ZA(i);
end

params = (A'*A) \ (A'*L);   
dX = params(1); dY = params(2); dZ = params(3);
ex = params(4); ey = params(5); ez = params(6); k = params(7);

fprintf('七参数（正算）------------\n');
fprintf('deltaX = %.6f m\ndeltaY = %.6f m\ndeltaZ = %.6f m\n', dX, dY, dZ);
fprintf('epx = %.10f"   epy = %.10f"   epz = %.10f"\n', rad2deg(ex)*3600, rad2deg(ey)*3600, rad2deg(ez)*3600);
fprintf('k  = %.10f   \n\n', k);

i = 2;
XA_i = XA(i); YA_i = YA(i); ZA_i = ZA(i);
XBc = XA_i + dX - ey*ZA_i + ez*YA_i + k*XA_i;
YBc = YA_i + dY + ex*ZA_i - ez*XA_i + k*YA_i;
ZBc = ZA_i + dZ - ex*YA_i + ey*XA_i + k*ZA_i;

fprintf('较差-------------\n');
fprintf('dX = %.3f mm\n dY = %.3f mm\n dZ = %.3f mm\n', ...
        (XBc - XB(i))*1000, (YBc - YB(i))*1000, (ZBc - ZB(i))*1000);

residual = A*params - L;
fprintf('\n残差 = %.3f m (约0.15m)，\n', max(abs(residual)));
