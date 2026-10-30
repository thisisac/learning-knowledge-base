function cfg = ProcessConfig_50Hz()

    param = Param();

    %% filepath
    cfg.imufilepath   = 'dataset_LJY/IMU_incre.bin';        
    cfg.gnssfilepath  = 'dataset_LJY/gnss_RTD_vel.txt';    
    cfg.refnavpath    = 'dataset_LJY/ref_out.nav';         
    cfg.odofilepath = '';
    cfg.outputfolder  = 'zuoye2';

    %% configure    
    cfg.usegnssvel = true;  
    cfg.useodonhc  = false;
    cfg.odoupdaterate = 1; % [Hz]

    %% initial information
    cfg.starttime = 106790;     
    cfg.endtime   = inf;

    cfg.initpos = [39.98383597; 116.31027922; 43.157 ];    
    cfg.initvel = [0; 0; 0];
    cfg.initatt = [ -0.189; -0.398; 355.608 ];    

    cfg.initposstd = [0.5; 0.5; 1.0];     % [m]
    cfg.initvelstd = [0.1; 0.1; 0.1];     % [m/s]
    cfg.initattstd = [0.5; 0.5; 1.0];     % [deg]

    cfg.initgyrbias = [-1800; 900; 1560];
    cfg.initaccbias = [-11670; 4280; -580];
    cfg.initgyrscale = [-6500; 600; 7500];
    cfg.initaccscale = [2800; 1100; 9160];

    cfg.initgyrbiasstd  = [50; 50; 50];       % [deg/h]
    cfg.initaccbiasstd  = [250; 250; 250];    % [mGal]
    cfg.initgyrscalestd = [1000; 1000; 1000]; % [ppm]
    cfg.initaccscalestd = [1000; 1000; 1000]; % [ppm]

    cfg.gyrarw       = 0.24;    % [deg/sqrt(h)]
    cfg.accvrw       = 0.24;    % [m/s/sqrt(h)]
    cfg.gyrbiasstd   = 50;      % [deg/h]
    cfg.accbiasstd   = 250;     % [mGal]
    cfg.gyrscalestd  = 1000;    % [ppm]
    cfg.accscalestd  = 1000;    % [ppm]
    cfg.corrtime     = 1;       % [h]

    %% install parameters 安装参数
    cfg.antlever     = [0; 0; 0];    
    cfg.odolever = [0; 0; 0]; %[m]
    cfg.installangle = [0; 0; 0];     

    %% ODO/NHC measurement noise 观测噪声
    cfg.odonhc_measnoise = [0.1; 0.1; 0.1]; % [m/s]

    %% convert unit to standard unit (单位转换)
    cfg.initpos(1) = cfg.initpos(1) * param.D2R;
    cfg.initpos(2) = cfg.initpos(2) * param.D2R;
    cfg.initatt = cfg.initatt * param.D2R;

    cfg.initattstd   = cfg.initattstd * param.D2R;

    cfg.initgyrbias     = cfg.initgyrbias * param.D2R / 3600;
    cfg.initaccbias     = cfg.initaccbias * 1e-5;
    cfg.initgyrscale    = cfg.initgyrscale * 1e-6;
    cfg.initaccscale    = cfg.initaccscale * 1e-6;
    cfg.initgyrbiasstd  = cfg.initgyrbiasstd * param.D2R / 3600;
    cfg.initaccbiasstd  = cfg.initaccbiasstd * 1e-5;
    cfg.initgyrscalestd = cfg.initgyrscalestd * 1e-6;
    cfg.initaccscalestd = cfg.initaccscalestd * 1e-6;

    cfg.gyrarw      = cfg.gyrarw * param.D2R / 60;
    cfg.accvrw      = cfg.accvrw / 60;
    cfg.gyrbiasstd  = cfg.gyrbiasstd * param.D2R / 3600;
    cfg.accbiasstd  = cfg.accbiasstd * 1e-5;
    cfg.gyrscalestd = cfg.gyrscalestd * 1e-6;
    cfg.accscalestd = cfg.accscalestd * 1e-6;
    cfg.corrtime = cfg.corrtime * 3600;

    cfg.installangle = cfg.installangle * param.D2R;
    cfg.cbv          = euler2dcm(cfg.installangle);

end
