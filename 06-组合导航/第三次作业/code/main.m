%% main_task.m
clc; clear;
test_modes = {'NHC_Only', 'ODO_Only', 'ODO_NHC', 'Full'};

for m = 1:length(test_modes)
    cfg = ProcessConfig_LJY();
    mode = test_modes{m};
    
    switch mode
        case 'NHC_Only', cfg.usetype = 2; cfg.usezupt = false; cfg.usezaru = false;
        case 'ODO_Only', cfg.usetype = 1; cfg.usezupt = false; cfg.usezaru = false;
        case 'ODO_NHC',  cfg.usetype = 3; cfg.usezupt = false; cfg.usezaru = false;
        case 'Full',     cfg.usetype = 3; cfg.usezupt = true;  cfg.usezaru = true;
    end
    
    % 执行封装后的 RunNavigationTask (此函数需基于你原来的 kf_gins 逻辑)
    RunNavigationTask(cfg, mode); 
end

% 自动画图分析
PlotNavResults('data', 'data/REF_NAV.nav', test_modes);