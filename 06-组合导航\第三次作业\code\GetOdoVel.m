% average odometer data to decrease quantization noise
% 对一段ODO数据平均，获取某一时刻ODO速度，可减小量化噪声
% 相当于匀变速假设，有一定精度损失
function [odovel, valid] = GetOdoVel(odoraw, time)
% GetOdoVel - 通过前后平均ODO速度，插值估算某一时刻的里程计速度

    odovel = 0;           % 默认速度为0
    valid = false;        % 默认无效

    if (size(odoraw, 1) < 5)
        disp('WARN: too little data to get odovel!');
    else

        firstvel = 0;
        secondvel = 0;
        firsttime = 0;
        secondtime = 0;
        firstindex = 0;
        secondindex = 0;
    
        % 遍历ODO数据
        for i = 1:size(odoraw, 1)
            if odoraw(i, 1) <= time
                % 前组：累加速度和时间，求平均
                firstvel = (firstvel * firstindex + odoraw(i, 2)) / (firstindex + 1);
                firsttime = (firsttime * firstindex + odoraw(i, 1)) / (firstindex + 1);
                firstindex = firstindex + 1;
            else
                % 后组：同理累加速度和时间，求平均
                secondvel = (secondvel * secondindex + odoraw(i, 2)) / (secondindex + 1);
                secondtime = (secondtime * secondindex + odoraw(i, 1)) / (secondindex + 1);
                secondindex = secondindex + 1;
            end
        end
        % 用前后两组平均速度做线性插值，得到查询时刻的估计速度
        % 假设匀变速运动
        odovel = firstvel + (time - firsttime) / (secondtime - firsttime) * (secondvel - firstvel);
        valid = true;
    end
end