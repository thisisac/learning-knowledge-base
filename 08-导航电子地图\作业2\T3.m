clear; clc;

node_names = {'u0', 'u1', 'u2', 'u3', 'u4', 'u5', 'u6', 'u7'};
n = 8; 
INF = 1e9;

adj = INF * ones(n);
for i = 1:n, adj(i,i) = 0; end

edges = [
    0 1 2;  0 2 1;  0 3 8; 
    1 3 6;  1 4 1; 
    2 3 7;  2 6 9; 
    3 4 5;  3 5 1;  3 6 2; 
    4 5 3;  4 7 9; 
    5 6 4;  5 7 6; 
    6 7 3
];

for i = 1:size(edges, 1)
    u = edges(i,1) + 1; v = edges(i,2) + 1; w = edges(i,3);
    adj(u,v) = w; adj(v,u) = w;
end

dist = INF * ones(1, n); 
dist(1) = 0;             
prev = zeros(1, n);      
visited = false(1, n);   

fprintf('%-4s | %-4s | %-20s | %-30s\n', '步数', '选择', 'S 集合', 'dist 向量 (u0~u7)');
fprintf('--------------------------------------------------------------------------\n');

for i = 1:n
    temp_dist = dist;
    temp_dist(visited) = INF;
    [min_val, u] = min(temp_dist);
    
    if isinf(min_val), break; end
    visited(u) = true;
    
    % 打印当前状态
    S_str = ['{', strjoin(node_names(visited), ','), '}'];
    dist_str = ['[', regexprep(num2str(dist), '\s+', ','), ']'];
    dist_str = strrep(dist_str, '1000000000', 'inf'); 
    fprintf('第%-2d步 | %-4s | %-20s | %-30s\n', i-1, node_names{u}, S_str, dist_str);

    for v = 1:n
        if adj(u,v) < INF && ~visited(v)
            if dist(u) + adj(u,v) < dist(v)
                dist(v) = dist(u) + adj(u,v);
                prev(v) = u;
            end
        end
    end
end

% 路径回溯
target = 8; 
path_idx = []; curr = target;
while curr ~= 0
    path_idx = [curr path_idx];
    curr = prev(curr);
end

fprintf('--------------------------------------------------------------------------\n');
fprintf('最终最短路径: %s\n', strjoin(node_names(path_idx), ' -> '));
fprintf('最短距离长度: %d\n', dist(target));