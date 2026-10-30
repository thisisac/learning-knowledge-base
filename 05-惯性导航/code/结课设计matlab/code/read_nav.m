function nav = read_nav(fname)
    fid = fopen(fname, 'r');
    if fid == -1
        error('read_nav_txt:CannotOpen', '无法打开文件: %s', fname);
    end
    C = textscan(fid, '%[^\r\n]');  % 先整行读入
    fclose(fid);

    lines = C{1};
    rows = zeros(0,11);
    rowsCapacity = 10000; 
    rowsBuf = zeros(rowsCapacity, 11);
    n = 0;

    for k = 1:numel(lines)
        s = strtrim(lines{k});
        if isempty(s)
            continue;
        end
        % 仅接受以数字或+-号开头的行
        c1 = s(1);
        if ~(isstrprop(c1,'digit') || c1=='+' || c1=='-')
            continue;
        end
        vals = sscanf(s, '%f');
        if numel(vals) < 11
            continue;
        end
        if n >= rowsCapacity
            rowsCapacity = rowsCapacity + 10000;
            rowsBuf(rowsCapacity,11) = 0; 
        end
        n = n + 1;
        rowsBuf(n, :) = vals(1:11).';
    end

    if n == 0
        nav = struct('mat', zeros(0,11), 'table', table());
        return;
    end

    M = rowsBuf(1:n, :);

    % 字段
    nav.mat = M;
    nav.week = M(:,1);
    nav.sow  = M(:,2);
    nav.lat_deg = M(:,3);
    nav.lon_deg = M(:,4);
    nav.h_m     = M(:,5);
    nav.vN      = M(:,6);
    nav.vE      = M(:,7);
    nav.vD      = M(:,8);
    nav.roll_deg  = M(:,9);
    nav.pitch_deg = M(:,10);
    nav.heading_deg = M(:,11);
    nav.t = nav.week*604800 + nav.sow;

    nav.table = table( ...
        nav.sow, nav.lat_deg, nav.lon_deg, nav.h_m, ...
        nav.vN, nav.vE, nav.vD, nav.roll_deg, nav.pitch_deg, nav.heading_deg, ...
        'VariableNames', {'sow','lat_deg','lon_deg','h_m','vN','vE','vD','roll_deg','pitch_deg','heading_deg'});
end