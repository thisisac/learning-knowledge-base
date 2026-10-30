%
% fname = 文件名
% n_col = 每个历元包含n_col个double数据
% lhs   = 数据体矩阵（N行x n_col列）
function lhs = read_data_bin(fname, n_col)

if nargin ~= 2
    disp('Needs more parameter(s)!')
    lhs = 0;
    return;
end
    
f_id = fopen(fname, 'rb');
if f_id == -1
    disp('Cannot open the file!');
    lhs = 0;
    return;
end

[lhs, eof] = fread(f_id, [n_col,inf], 'double');
lhs = lhs';
fclose(f_id);
