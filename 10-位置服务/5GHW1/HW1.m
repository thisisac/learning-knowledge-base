%HW1
clear; clc; close all;

load('data_5G.mat');             

whos -file data_5G.mat

signal_var_name = 'recv_signal';   

if ~exist(signal_var_name, 'var')
    error('变量 "%s" 不存在，请根据 whos 输出修改 signal_var_name', signal_var_name);
end

rx_signal = eval(signal_var_name);  
fs = 7.68e6;                     

if size(rx_signal, 1) == 1
    rx_signal = rx_signal(:);
end

N = length(rx_signal);           
t = (0:N-1)' / fs;                  

fprintf('信号长度: %d 采样点\n', N);
fprintf('信号时长: %.6f 秒\n', N/fs);

Y = fft(rx_signal);
Y_mag = abs(Y / N);                 
Y_mag_shift = fftshift(Y_mag);     

f = (-N/2:N/2-1)' * (fs / N);      

figure('Name', 'FFT幅度谱');
plot(f, Y_mag_shift);
xlabel('频率 (Hz)');
ylabel('幅度');
title('接收信号 FFT 幅度谱');
grid on;
xlim([-fs/2, fs/2]);

%  Welch 功率谱密度估计 

[Pxx, f_welch] = pwelch(rx_signal, hamming(1024), 512, 4096, fs);

figure('Name', 'Welch功率谱');
plot(f_welch, 10*log10(Pxx));     
xlabel('频率 (Hz)');
ylabel('功率谱密度 (dB/Hz)');
title('接收信号功率谱密度 (Welch 法)');
grid on;
xlim([-fs/2, fs/2]);

figure('Name', 'Welch功率谱 (正频率)');
plot(f_welch, 10*log10(Pxx));
xlabel('频率 (Hz)');
ylabel('功率谱密度 (dB/Hz)');
title('接收信号功率谱密度 (正频率部分)');
grid on;
xlim([0, fs/2]);

figure('Name', '时域波形');
plot(t, real(rx_signal));
xlabel('时间 (s)');
ylabel('幅度');
title('接收信号时域波形 (实部)');
grid on;
xlim([0, min(0.001, t(end))]);      % 仅显示前 1 ms 以观察细节
