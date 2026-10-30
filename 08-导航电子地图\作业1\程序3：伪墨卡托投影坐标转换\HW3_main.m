%刘峻琰2023302141249第一次导航电子地图作业_HW3
a = 6378137;
B = 33.12340;
L = 114.52770;

[x, y] = bl2webmercator(B, L, a);
fprintf('(x, y): x=%.3f m   y=%.3f m\n', x, y);

[Bc, Lc] = webmercator2bl(x, y, a);
fprintf('较差: dB=%.8f°   dL=%.8f°\n', Bc-B, Lc-L);

function [x, y] = bl2webmercator(B_deg, L_deg, a)
    B = deg2rad(B_deg); L = deg2rad(L_deg);
    x = a * L;
    y = a * log(tan(pi/4 + B/2));
end

function [B_deg, L_deg] = webmercator2bl(x, y, a)
    L_deg = rad2deg(x / a);
    B = 2*atan(exp(y/a)) - pi/2;
    B_deg = rad2deg(B);
end