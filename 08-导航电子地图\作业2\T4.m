clear; clc;


A = [84500.0, 41200.0]; 
B = [85800.0, 42500.0]; 
C = [86800.0, 43600.0];
P = [85620.3, 42351.7];   % 事故点
M_A = 11.5;               % km

AB_vec = B - A; AB_len = norm(AB_vec);
BC_vec = C - B; BC_len = norm(BC_vec);

% 投影函数
function [closest, t, dist] = projectToSegment(p, a, b)
    ab = b - a; ap = p - a;
    proj = dot(ap, ab) / dot(ab, ab);
    t = max(0, min(1, proj));
    closest = a + t * ab;
    dist = norm(p - closest);
end

% 事故点投影
[clAB, tAB, dAB] = projectToSegment(P, A, B);
[clBC, tBC, dBC] = projectToSegment(P, B, C);

if dAB <= dBC
    closest = clAB; seg = 'AB'; distA = tAB * AB_len;
else
    closest = clBC; seg = 'BC'; distA = AB_len + tBC * BC_len;
end
M_P = M_A + distA / 1000;

% 养护点 12.8 km
M_maint = 12.8;
dist_m = (M_maint - M_A) * 1000;
if dist_m <= AB_len
    t_m = dist_m / AB_len; coord_m = A + t_m * AB_vec; seg_m = 'AB';
else
    t_m = (dist_m - AB_len) / BC_len; coord_m = B + t_m * BC_vec; seg_m = 'BC';
end

% 输出
fprintf('AB长度: %.3f m \nBC长度: %.3f m \n', AB_len, BC_len);
fprintf('【事故点】里程 M = %.6f km (%s 投影坐标 %.2f, %.2f)\n', M_P, seg, closest(1), closest(2));
fprintf('【养护点】里程 %.2f km 坐标: (%.2f, %.2f) (%s)\n', M_maint, coord_m(1), coord_m(2), seg_m);