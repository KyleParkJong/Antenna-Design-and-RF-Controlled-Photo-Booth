clc;
clear all;
close all;

fs = 60e6;   % sampling freq.
freq = [1e9-10e6 : 10e6 : 3.8e9];   % 반송파 주파수 범위
S21 = zeros(length(freq), 1);       

% Sine wave %
sw = dsp.SineWave;  % sine wave 생성
sw_setting(sw, 30e6, length(freq));  % freq : 3MHz 

% Transmit settings %   
tx = sdrtx('Pluto');
tx.BasebandSampleRate = fs;
tx.Gain = 0;
txdata = sw();

% Receive settings %
rx = sdrrx('Pluto'); 
rx.BasebandSampleRate = fs; 
rx.OutputDataType = 'double'; 
rx.SamplesPerFrame = length(freq);    % txdata.SamplesPerFrame의 2배
rx.GainSource = 'Manual'; 
rx.Gain = 0;

i = 1;

pause(3)    % 안테나 위치 조절를 위한 시간

for fc = freq
    
    % 반송파 주파수 설정 %
    tx.CenterFrequency = fc;
    rx.CenterFrequency = fc;

    % 신호 송수신 %
    transmitRepeat(tx,txdata);
    rxdata = rx();

    % 송수신 신호의 전력 계산 %
    tx_power = bandpower(real(txdata));
    rx_power = bandpower(real(rxdata));
    
    % 전력비를 통해 S21 파라미터 계산 %
    S21(i) = 10 * log10( rx_power/tx_power );

    i = i + 1;

    release(rx)
    release(tx)
    
end

% PLOT %
[MAX, index] = max(S21);
str1 = sprintf('<Peak>\n%d MHz\n%.2f dB', freq(index)/1e6, MAX);
str2 = sprintf('%d MHz', freq(index)/1e6);
dim = [.2 .5 .3 .3];

figure()
plot(freq, S21)
title('S21 Parameter', 'FontWeight','bold', 'FontSize', 13)
ylim([-50 0])
xlim([1e9 3.8e9])
xline(freq(index), '--r', {str2})
xlabel('Frequency', 'FontWeight','bold')
ylabel('dB', 'FontWeight','bold')
annotation('textbox',dim,'String',str1,'FitBoxToText','on');
grid on

release(sw);
release(rx)
release(tx)

% sine wave 설정을 위한 함수 선언 %
function sw_setting(sw, freq, samplesPerFrame)
    sw.Amplitude = 0.5;
    sw.Frequency = freq;
    sw.ComplexOutput = true;
    sw.SampleRate = 1e9;  % sampling freq.
    sw.SamplesPerFrame = samplesPerFrame; 
end