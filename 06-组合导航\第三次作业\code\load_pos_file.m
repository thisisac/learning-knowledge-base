function gnss_data = load_pos_file(pos_filepath, start_time, end_time, param)
    % 功能：读取包含速度和标准差信息的 RTKLIB .pos 文件
    % 输出 gnss_data 格式 (13列):
    % [sow, lat(rad), lon(rad), alt, vn, ve, vd, sdn, sde, sdu, sdvn, sdve, sdvd]
    
    fid = fopen(pos_filepath, 'r');
    if fid == -1, error('无法打开文件: %s', pos_filepath); end

    % 跳过文件头（以%或#开头的行）
    frewind(fid);
    pos = 0;
    while ~feof(fid)
        line = fgetl(fid);
        if ~startsWith(line, '%') && ~startsWith(line, '#')
            fseek(fid, pos, 'bof'); % 回退到非注释行开头
            break;
        end
        pos = ftell(fid);
    end

    % 使用 textscan 一次性读取，根据你提供的示例行：
    % 1:Date 2:Time 3:Lat 4:Lon 5:Alt 6:Q 7:ns 
    % 8:sdn 9:sde 10:sdu 11:sdne 12:sdeu 13:sdun 
    % 14:age 15:ratio 16:vn 17:ve 18:vu 19:sdvn 20:sdve 21:sdvu ...
    formatSpec = '%s %s %f %f %f %d %d %f %f %f %f %f %f %f %f %f %f %f %f %f %f %*[^\n]';
    C = textscan(fid, formatSpec);
    fclose(fid);

    if isempty(C{1}), gnss_data = []; return; end

    % 1. 时间转换 (Date + Time -> SOW)
    time_str = strcat(C{1}, {' '}, C{2});
    dt = datetime(time_str, 'InputFormat', 'yyyy/MM/dd HH:mm:ss.SSS');
    % 计算周内秒 (sow)
    sow = mod((datenum(dt) - datenum(1980,1,6))*86400, 604800);

    % 2. 提取数据并进行单位转换
    lat_rad = C{3} * param.D2R;
    lon_rad = C{4} * param.D2R;
    alt = C{5};
    Q = C{6};
    
    % 速度转换 (RTKLib输出为 vn, ve, vu -> 转换为 vn, ve, vd)
    vn = C{16};
    ve = C{17};
    vd = -C{18}; % Up -> Down
    
    % 标准差 (位置和速度)
    sdn = C{8}; sde = C{9}; sdu = C{10};
    sdvn = C{19}; sdve = C{20}; sdvd = C{21}; % 这里假设vu的标准差等于vd的标准差

    % 3. 封装并根据时间、质量过滤
    raw_data = [sow, lat_rad, lon_rad, alt, vn, ve, vd, sdn, sde, sdu, sdvn, sdve, sdvd];
    
    % 过滤 Q=1(固定) 或 Q=2(浮点)，且在时间范围内
    mask = (Q == 1 | Q == 2) & (sow >= start_time & sow <= end_time);
    gnss_data = raw_data(mask, :);

    if isempty(gnss_data)
        warning('未找到符合时间段或质量要求的GNSS数据');
    else
        disp(['成功加载 ' num2str(size(gnss_data,1)) ' 帧有效GNSS数据']);
    end
end