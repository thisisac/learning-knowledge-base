function cfg = ProcessConfig_LJY()

    param = Param();

    %% filepath
    cfg.imufilepath = 'data/IMU.bin';           % IMU binary file (7 cols: time, gyro increments rad, acc increments m/s)
    cfg.gnssfilepath = 'data/MOV.pos';  % Processed GNSS (RTK/SPP) file with pos/vel
    cfg.odofilepath = 'data/ODO.txt';           % ODO text file (2 cols: time, speed m/s)
    cfg.outputfolder = 'data';           % Output folder to avoid overwriting data

    %% configure
    cfg.usegnssvel = true;                         % Use GNSS velocity (from RTK)
    cfg.useodonhc = true;                          % Enable ODO/NHC update
    cfg.usetype = 3;                               % 1=ODO only, 2=NHC only, 3=ODO+NHC
    cfg.usezupt = true;                           % Enable Zero Velocity Update (ZUPT)
    cfg.usezaru = true;                           % Enable Zero Angular Rate Update (ZARU)
    cfg.odoupdaterate = 1;                         % [Hz] ODO update rate (match 1Hz ODO sampling)

    %% initial information
    cfg.starttime = 529000;                        % Start time (GPS sow)需要相应进行更改
    cfg.endtime = 529060;

    cfg.initpos = [28.1421701503; 112.9585396051; 38.591]; % [deg, deg, m]
    cfg.initvel = [-0.273; -3.942; -0.020];        % [m/s] N E D
    cfg.initatt = [1.40822092; -0.95903515; 267.77618701]; % [deg] roll pitch yaw

    cfg.initposstd = [0.05; 0.05; 0.1];            % [m]
    cfg.initvelstd = [0.05; 0.05; 0.05];           % [m/s]
    cfg.initattstd = [0.1; 0.1; 0.5];              % [deg]

    cfg.initgyrbias = [100; -300; -100];           % [deg/h]
    cfg.initaccbias = [-1500; -600; 0];            % [mGal]
    cfg.initgyrscale = [1000; 6000; -4000];        % [ppm]
    cfg.initaccscale = [2600; 8400; 300];          % [ppm]

    cfg.initgyrbiasstd = [48; 48; 48];             % [deg/h] (use bias instability)
    cfg.initaccbiasstd = [50; 50; 50];             % [mGal]
    cfg.initgyrscalestd = [500; 500; 500];         % [ppm]
    cfg.initaccscalestd = [500; 500; 500];         % [ppm]
    cfg.initodoscalestd = 0.01; % 1%
    
    cfg.gyrarw = 0.2;                              % [deg/sqrt(hr)] ARW
    cfg.accvrw = 0.2;                              % [m/s/sqrt(hr)] VRW
    cfg.gyrbiasstd = 48;                           % [deg/h] bias instability
    cfg.accbiasstd = 50;                           % [mGal]
    cfg.gyrscalestd = 500;                         % [ppm]
    cfg.accscalestd = 500;                         % [ppm]
    cfg.corrtime = 1;                              % [h]

    %% install parameters
    cfg.antlever = [0.505; -0.145; -1.105];        % [m] GNSS antenna lever arm (forward-right-down in body frame)
    cfg.odolever = [0.605; -1.025; 0.705];         % [m] ODO lever arm (forward-right-down in body frame)
    cfg.installangle = [0.0; -0.532; 1.38];        % [deg] IMU to vehicle installation angles (roll, pitch, yaw)

    %% ODO/NHC measurement noise
    cfg.odonhc_measnoise = [0.1; 0.1; 0.1]; % [m/s] ODO forward, NHC right/down STD

    %% ODO scale factor (modeled as per assignment)
    cfg.initodoscalefactor = 0;                    % Initial ODO scale factor error (dimensionless)
    cfg.initodoscalefactorstd = 0.01;              % Initial STD (e.g., 1%)
    cfg.odoscalestd = 1e-6;                        % Process noise for ODO scale

    %% ZUPT/ZARU parameters (for assignment 3)
    cfg.zupt_noise = [0.01; 0.01; 0.01];           % [m/s] ZUPT measurement noise
    cfg.zaru_noise = 0.01;                         % [rad/s] ZARU measurement noise
    cfg.zupt_window_size = 10;                     % Sliding window size for zero-speed detection
    cfg.zupt_accel_threshold = 0.1;                % Acc variance threshold [(m/s²)²]
    cfg.zupt_gyro_threshold = 0.05;                % Gyro variance threshold [(deg/s)²]

    %% GNSS measurement noise (for GNSSUpdate)
    cfg.gnss_pos_std = [0.1; 0.1; 0.2];            % [m] GNSS position STD (adjust based on RTK quality)
    cfg.gnss_vel_std = [0.05; 0.05; 0.1];          % [m/s] GNSS velocity STD

    %% convert unit to standard unit
    cfg.initpos(1) = cfg.initpos(1) * param.D2R;
    cfg.initpos(2) = cfg.initpos(2) * param.D2R;
    cfg.initatt = cfg.initatt * param.D2R;

    cfg.initattstd = cfg.initattstd * param.D2R;

    cfg.initgyrbias = cfg.initgyrbias * param.D2R / 3600;
    cfg.initaccbias = cfg.initaccbias * 1e-5;
    cfg.initgyrscale = cfg.initgyrscale * 1e-6;
    cfg.initaccscale = cfg.initaccscale * 1e-6;
    cfg.initgyrbiasstd = cfg.initgyrbiasstd * param.D2R / 3600;
    cfg.initaccbiasstd = cfg.initaccbiasstd * 1e-5;
    cfg.initgyrscalestd = cfg.initgyrscalestd * 1e-6;
    cfg.initaccscalestd = cfg.initaccscalestd * 1e-6;

    cfg.gyrarw = cfg.gyrarw * param.D2R / 60;
    cfg.accvrw = cfg.accvrw / 60;
    cfg.gyrbiasstd = cfg.gyrbiasstd * param.D2R / 3600;
    cfg.accbiasstd = cfg.accbiasstd * 1e-5;
    cfg.gyrscalestd = cfg.gyrscalestd * 1e-6;
    cfg.accscalestd = cfg.accscalestd * 1e-6;
    cfg.corrtime = cfg.corrtime * 3600;

    cfg.installangle = cfg.installangle * param.D2R;
    cfg.cbv = euler2dcm(cfg.installangle);         % C_b^v rotation matrix (IMU body to vehicle)

    % Additional conversions if needed
    cfg.zupt_gyro_threshold = cfg.zupt_gyro_threshold * param.D2R;  % Convert deg/s to rad/s

end