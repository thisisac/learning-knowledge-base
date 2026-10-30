
% 主函数（与Python代码的main部分对应）
function main()
    % 数据的读取
    imu_file = 'IMU.bin';
    pureins_file = 'PureINS.bin';
    
    % 调用读取函数
    mImuDataEpoch = read_data_bin(imu_file, 7);
    mInsNavEpoch = read_data_bin(pureins_file, 10);
    
    % 输出大小确认读取成功
    fprintf('IMU数据大小: %d x %d\n', size(mImuDataEpoch));
    fprintf('PureINS数据大小: %d x %d\n', size(mInsNavEpoch));
    
    % 设定初始条件（与Python代码完全一致）
    deg2rad = pi / 180;
    rad2deg = 180 / pi;
    
    lat0 = 23.1373950708 * deg2rad;
    lon0 = 113.3713651222 * deg2rad;
    h0 = 2.175;
    vN0 = 0.0;
    vE0 = 0.0;
    vD0 = 0.0;
    roll0 = 0.0107951084511778 * deg2rad;
    pitch0 = -2.14251290749072 * deg2rad;
    yaw0 = -75.7498049314083 * deg2rad;
    time0 = 91620.000000000;

    
    % 调用INS解算函数
    presult = ins_from_imu(mImuDataEpoch, lat0, lon0, h0, vN0, vE0, vD0, ...
                          roll0, pitch0, yaw0, time0);
    
    % 将table转换为数组以便计算差值
    presult_array = table2array(presult.table);
    
    % 计算差值（与Python代码完全一致）
    diff = presult_array - mInsNavEpoch;
    for i = 1:size(diff, 1)
        diff(i, 1) = presult_array(i, 1);  % 恢复时间戳
    end
    
    % 调用绘图函数
    plot_ins_results(presult_array, mInsNavEpoch, diff);
    
    fprintf('处理完成\n');
end
