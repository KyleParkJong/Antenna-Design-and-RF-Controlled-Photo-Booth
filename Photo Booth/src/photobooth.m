clear all;
close all;
clc;

fs = 10e6;
fc = 830e6; % Carrier Freq.

% State (flag) %
IDLE = true; CAPTURE = false; UP = false; DOWN = false;

% Freq. %
idle = 100e3; selfi = 110e3; up = 120e3; down = 130e3;

% No. of captures %
i = 1;

% WebCam settings %
webcamlist
mycam = webcam('HD WEB CAMERA');
preview(mycam)

% Arduino settings %
a = arduino('/dev/cu.usbmodem11101', 'Uno', 'Libraries', 'Servo');


%% Servo settings %%
s = servo(a, 'D11', 'MinPulseDuration', 700*10^-6, 'MaxPulseDuration', 2300*10^-6);
theta = 3;      % 0~7 중앙값
writePosition(s, 0.5);

%% Receive settings %%
rx = sdrrx('Pluto'); 
rx.CenterFrequency = fc; 
rx.BasebandSampleRate = fs; 
rx.OutputDataType = 'double'; 
rx.SamplesPerFrame = 1000*2;    % txdata.SamplesPerFrame x 2
rx.GainSource = 'Manual'; 
rx.Gain = 25; 

while (i ~= 5)  % 사진 4번 찍으면 종료
    rxdata = capture(rx, rx.SamplesPerFrame,'Samples');
    %rxdata = rx();
    
    tiledlayout(2, 1)
    nexttile
    plot(real(rxdata))

    % FFT %
    N = rx.SamplesPerFrame;    % samples per frame
    n = -N/2 : N/2-1;
    f = fs * n/N;
    Ft = fftshift(fft(rxdata));

    nexttile
    stem(f, 2*abs(Ft)/N)
    xlim([0 140e3])

    len = length(Ft);
    Ftt = Ft((len/2 + 2 : len), 1);
    [MAX, index] = max(Ftt);
    index = index + len/2 + 1;
    freq = f(index);     % rxdata의 주파수
    fprintf("rxdata freq : %d\n", freq)
    pause(1)
    
    switch freq
        case selfi
            image = snapshot(mycam);
            file_name = sprintf('Image%d.png', i);
            imwrite(image,file_name,'png');
            i = i + 1;
            %pause(2);    
            %imshow(my_image);
        case up
            for angle = 0.5+0.07*(theta-3) : 0.01 : 0.5+0.07*(theta-2)  % 약 10 degrees up 
                writePosition(s, angle);
            end
            theta = theta+1;
        case down
            for angle = 0.5+0.07*(theta-3) : -0.01 : 0.5+0.07*(theta-4)  % 10 degrees down
                writePosition(s, angle);
            end
            theta = theta-1;
        otherwise
            continue;
    end
    
end

% backgroud generate %
s0 = [70 2040];   bg1 = repmat(permute(uint8([13 97 62]),[1 3 2]),s0); % uint8
s0 = [1080 60];   bg2_1 = repmat(permute(uint8([13 97 62]),[1 3 2]),s0);
s0 = [40 2040];   bg3 = repmat(permute(uint8([13 97 62]),[1 3 2]),s0);
s0 = [1080 2040]; bg_last = repmat(permute(uint8([13 97 62]),[1 3 2]),s0);

% Image read %
img1 = imread('Image1.png'); img2 = imread('Image2.png'); img3 = imread('Image3.png'); img4 = imread('Image4.png');

% Image connect %
tmp1 = horzcat(bg2_1, img1, bg2_1);
tmp2 = horzcat(bg2_1, img2, bg2_1);
tmp3 = horzcat(bg2_1, img3, bg2_1);
tmp4 = horzcat(bg2_1, img4, bg2_1);

result_img = vertcat(bg1, tmp1, bg3, tmp2, bg3, tmp3, bg3, tmp4, bg_last);

% Image save %
ImName = "Life4cut_ori.png";
ImFileOut = fullfile(pwd, ImName);
imwrite(result_img,ImFileOut);

%% 이미지 흑백으로 변환 및 흑백 대비 조정 %%
img1_gray = imadjust(im2gray(img1),[0.1 0.8],[]); img2_gray = imadjust(im2gray(img2),[0.1 0.8],[]); 
img3_gray = imadjust(im2gray(img3),[0.1 0.8],[]); img4_gray = imadjust(im2gray(img4),[0.1 0.8],[]);
bg1_gray = im2gray(bg1); bg2_1_gray =  im2gray(bg2_1); bg3_gray = im2gray(bg3); bg_last_gray = im2gray(bg_last);  

% Image connect %
tmp1 = horzcat(bg2_1_gray, img1_gray, bg2_1_gray);
tmp2 = horzcat(bg2_1_gray, img2_gray, bg2_1_gray);
tmp3 = horzcat(bg2_1_gray, img3_gray, bg2_1_gray);
tmp4 = horzcat(bg2_1_gray, img4_gray, bg2_1_gray);

result_img_gray = vertcat(bg1_gray, tmp1, bg3_gray, tmp2, bg3_gray, tmp3, bg3_gray, tmp4, bg_last_gray);

% Image save %
ImName = "Life4cut_gray.png";
ImFileOut = fullfile(pwd, ImName);
imwrite(result_img_gray,ImFileOut);


%% Lightening %%
img_array = {img1, img2, img3, img4};
img_imp = {0,0,0,0};
for ii = 1 : 4
    tmp1 = imcomplement(img_array{ii});
    tmp2 = imreducehaze(tmp1, 'Method','approx','ContrastEnhancement','boost');
    img_imp{ii} = imcomplement(tmp2);
end

% Image connect %
tmp1 = horzcat(bg2_1, img_imp{1}, bg2_1);
tmp2 = horzcat(bg2_1, img_imp{2}, bg2_1);
tmp3 = horzcat(bg2_1, img_imp{3}, bg2_1);
tmp4 = horzcat(bg2_1, img_imp{4}, bg2_1);
result_img_imp = vertcat(bg1, tmp1, bg3, tmp2, bg3, tmp3, bg3, tmp4, bg_last);

% Image save %
ImName = "Life4cut_imp.png";
ImFileOut = fullfile(pwd, ImName);
imwrite(result_img_imp,ImFileOut);

release(rx);
clear s;
clear a;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%% Transmit Image %%%%%%%%%%%%%%
%% Image Transmission and Reception Using WLAN Toolbox and One PlutoSDR 
% This example shows how to transmit and receive WLAN packets on a single
% PlutoSDR device, using the Communications Toolbox(TM) Support Package for
% ADALM-PLUTO Radio and the WLAN Toolbox(TM).  An image file is encoded and
% packed into WLAN packets for transmission, and subsequently decoded on
% reception.

% Copyright 2017-2018 The MathWorks, Inc.

%% Required Hardware and Software
% To run this example, you need the following software:
%
% * <matlab:web(['https://www.mathworks.com/products/communications/'],'-browser') Communications Toolbox(TM)>
%
% You also need the following SDR device and the corresponding support
% package Add-On:
%
% * ADALM-PLUTO radio and the corresponding software
% <matlab:web(['https://www.mathworks.com/hardware-support/adalm-pluto-radio.html'],'-browser')
% Communications Toolbox Support Package for ADALM-PLUTO Radio>
%
%
% For a full list of Communications Toolbox supported SDR platforms,
% refer to Supported Hardware section of
% <matlab:web(['https://www.mathworks.com/discovery/sdr.html'],'-browser')
% Software Defined Radio (SDR) discovery page>.

%% Introduction
% You can use WLAN Toolbox to generate standard-compliant MAC frames and
% waveforms. These baseband waveforms can be upconverted for RF
% transmission using SDR hardware such as PlutoSDR. The PlutoSDR
% <matlab:plutoradiodoc('sdrpluto_repeatedwaveformtx') Repeated Waveform
% Transmitter> functionality allows a waveform to be transmitted over the
% air and is received using the same SDR hardware. The received waveform is
% captured and downsampled to baseband using a PlutoSDR and is decoded to
% recover the transmitted information as shown in the following figure.
%
% <<SDRWLAN80211aTransceiver_published.png>>
%
% This example imports and segments an image file into multiple MAC service
% data units (MSDUs). Each MSDU is passed to the
% <matlab:doc('wlanMACFrame') wlanMACFrame> function to create a MAC
% protocol data unit (MPDU). This function also consumes
% <matlab:doc('wlanMACFrameConfig') wlanMACFrameConfig> object as an input,
% which can be used to sequentially number the MPDUs through the
% |SequenceNumber| property. The MPDUs are passed to the PHY layer as PHY
% Layer Service Data Units (PSDUs). Each PSDU data is packed into a single
% NonHT, 802.11a(TM) [ <#24 1> ] WLAN packet using WLAN Toolbox. This
% example creates a WLAN baseband waveform using the
% <matlab:doc('wlanWaveformGenerator') wlanWaveformGenerator> function.
% This function consumes multiple PSDUs and processes each to form a series
% of PLCP Protocol Data Units (PPDUs). The multiple PPDUs are upconverted
% and the RF waveform is sent over the air using PlutoSDR as shown in the
% following figure.
%
% <<plutoradioWLANTransmitReceiveExampleTransmit.png>>
%
% This example then captures the transmitted waveform using the same
% PlutoSDR.  The RF transmission is demodulated to baseband and the
% received MPDUs are decoded using the <matlab:doc('wlanMPDUDecode')
% wlanMPDUDecode> function. The extracted MSDUs are ordered using the
% |SequenceNumber| property in the recovered MAC frame configuration
% object. The information bits in the multiple received MSDUs are combined
% to recover the transmitted image. The receiver processing is illustrated
% in the following diagram.
%
% <<plutoradioWLANTransmitReceiveExampleReceive.png>>

%% Example Setup
% Before you run this example, perform the following steps:
%
% # Configure your host computer to work with the Support Package for
% ADALM-PLUTO Radio. See <matlab:plutoradiodoc('sdrpluto_spsetup') Getting
% Started> for help.
% # Make sure that WLAN Toolbox is installed. You must have a WLAN
% Toolbox license to run this example.
%
% When you run this example, the first thing the script does is to check
% for WLAN Toolbox.

% Check that WLAN Toolbox is installed, and that there is a valid
% license
if isempty(ver('wlan')) % Check for WLAN Toolbox install
    error('Please install WLAN Toolbox to run this example.');
elseif ~license('test', 'WLAN_System_Toolbox') % Check that a valid license is present
    error( ...
        'A valid license for WLAN Toolbox is required to run this example.');
end
 
%%
% The script then configures all the scopes and figures that will be
% displayed throughout the example.

% Setup handle for image plot
if ~exist('imFig', 'var') || ~ishandle(imFig)
    imFig = figure;
    imFig.NumberTitle = 'off';
    imFig.Name = 'Image Plot';
    imFig.Visible = 'off';
else   
    clf(imFig); % Clear figure
    imFig.Visible = 'off';
end

% Setup Spectrum viewer
spectrumScope = dsp.SpectrumAnalyzer( ...
    'SpectrumType', 'Power density', ...
    'SpectralAverages', 10, ...
    'YLimits', [-130 -50], ...
    'Title', 'Received Baseband WLAN Signal Spectrum', ...
    'YLabel', 'Power spectral density', ...
    'Position', [69 376 800 450]);

% Setup the constellation diagram viewer for equalized WLAN symbols
constellation = comm.ConstellationDiagram(...
    'Title', 'Equalized WLAN Symbols', ...
    'ShowReferenceConstellation', false, ...
    'Position', [878 376 460 460]);
                            
%%
% An <matlab:plutoradiodoc('commsdrtxpluto') SDR Transmitter> System object
% is used with the PlutoSDR to transmit baseband data to the SDR hardware.
%

%  Initialize SDR device
deviceNameSDR = 'Pluto'; % Set SDR Device
radio = sdrdev(deviceNameSDR);           % Create SDR device object

%%
% The following sections explain the design and architecture of this
% example, and what you can expect to see as the code is executed.

%% Transmitter Design
% The general structure of the WLAN transmitter can be described as follows:
%
% # Import an image file and convert it to a stream of decimal bytes.
% # Generate a baseband WLAN signal using WLAN Toolbox, pack the data
% stream into multiple 802.11a packets.
% # Prepare the baseband signal for transmission using the SDR hardware.
% # Send the baseband data to the SDR hardware for upsampling and
% continuous transmission at the desired center frequency.

%%
% The transmitter gain parameter is used to impair the quality of the
% received waveform, you can change this parameter to reduce transmission
% quality, and impair the signal. These are suggested values, depending on
% your antenna configuration, you may have to tweak these values. The
% suggested values are:
%
% # Set to 0 for increased gain (0dB)
% # Set to -10 for default gain (-10dB)
% # Set to -20 for reduced gain (-20dB)

txGain = 0;   % gain 조절 (거리 조절 대신. 거리가 늘수록 게인 작아짐) 

%%
% *Prepare Image File*
% 
% The example reads data from the image file, scales it for transmission,
% and converts it to a stream of decimal bytes. The scaling of the image
% reduces the quality of the image by decreasing the size of the binary
% data stream.
%
% The size of the transmitted image directly impacts the number of WLAN
% packets which are required for the transmission of the image data. A
% scaling factor is used to scale the original size of the image. The
% number of WLAN packets that are generated for transmission is dependent
% on the following:
%
% # The image scaling that you set when importing the image file.
% # The length of the data carried in a packet. This is specified by the
% |msduLength| variable. 
% # The MCS value of the transmitted packet.
%
% The combination of scaling factor |scale| of 0.2, and MSDU length
% |msduLength| of 2304 as shown below, requires the transmission of 11 WLAN
% radio packets. Increasing the scaling factor or decreasing the MSDU
% length will result in the transmission of more packets.

% Input an image file and convert to binary stream
fileTx = 'Life4cut_ori.png';            % Image file name
fData = imread(fileTx);            % Read image data from file
scale = 0.2; % 0.2                       % Image scaling factor
origSize = size(fData);            % Original input image size
scaledSize = max(floor(scale.*origSize(1:2)),1); % Calculate new image size
heightIx = min(round(((1:scaledSize(1))-0.5)./scale+0.5),origSize(1));
widthIx = min(round(((1:scaledSize(2))-0.5)./scale+0.5),origSize(2));
fData = fData(heightIx,widthIx,:); % Resize image
imsize = size(fData);              % Store new image size
txImage = fData(:);

%%
% The example displays the image file that is to be transmitted. When the
% image file is successfully received and decoded, the example displays the
% image.

% Plot transmit image
figure(imFig);
imFig.Visible = 'on';
 
    imshow(fData);
    title('Transmitted Image');


pause(1); % Pause to plot Tx image

%%
% *Fragment transmit data*
%
% The example uses the data stream that is created from the input image
% file |txImage|. The data stream is split into smaller transmit units
% (MSDUs) of size |msduLength|. An MPDU is created for each transmit unit
% using the <matlab:doc('wlanMACFrame') wlanMACFrame> function. Each call
% to this function creates an MPDU corresponding to the given MSDU and the
% frame configuration object. The frame configuration object can be created
% using <matlab:doc('wlanMACFrameConfig') wlanMACFrameConfig> which can be
% used to configure the sequence number of the MPDU. All the MPDUs are then
% sequentially passed to the physical layer for transmission.
%
% In this example the |msduLength| field is set to 2304 bytes. This is to
% ensure that the maximum MSDU size specified in the standard [ <#24 1> ]
% is not exceeded. The data in the last MPDU is appended with zeros, this
% is to make all MPDUs the same size.

msduLength = 2304; % MSDU length in bytes
numMSDUs = ceil(length(txImage)/msduLength);
padZeros = msduLength-mod(length(txImage),msduLength);
txData = [txImage; zeros(padZeros,1)];
txDataBits = double(reshape(de2bi(txData, 8)', [], 1));

% Divide input data stream into fragments
bitsPerOctet = 8;
data = zeros(0, 1);

for ind=0:numMSDUs-1
    
    % Extract image data (in octets) for each MPDU
    frameBody = txData(ind*msduLength+1:msduLength*(ind+1),:);
    
    % Create MAC frame configuration object and configure sequence number
    cfgMAC = wlanMACFrameConfig('FrameType', 'Data', 'SequenceNumber', ind);
    
    % Generate MPDU
    [mpdu, lengthMPDU]= wlanMACFrame(frameBody, cfgMAC);
    
    % Convert MPDU bytes to a bit stream
    psdu = reshape(de2bi(hex2dec(mpdu), 8)', [], 1);
    
    % Concatenate PSDUs for waveform generation
    data = [data; psdu]; %#ok<AGROW>
    
end

%% 
% *Generate IEEE 802.11a Baseband WLAN Signal*
%
% The non-HT waveform is synthesized using
% <matlab:doc('wlanWaveformGenerator') wlanWaveformGenerator> with a non-HT
% format configuration object. The object is created using the
% <matlab:doc('wlanNonHTConfig') wlanNonHTConfig> function. The properties
% of the object contain the configuration. In this example an object is
% configured for a 20 MHz bandwidth, 1 transmit antenna and 64QAM rate 2/3
% (MCS 6).

nonHTcfg = wlanNonHTConfig;         % Create packet configuration
nonHTcfg.MCS = 6;                   % Modulation: 64QAM Rate: 2/3   ->  2^6 QAM
nonHTcfg.NumTransmitAntennas = 1;   % Number of transmit antenna
chanBW = nonHTcfg.ChannelBandwidth;
nonHTcfg.PSDULength = lengthMPDU;   % Set the PSDU length

% The sdrTransmitter uses the |transmitRepeat| functionality to transmit
% the baseband WLAN waveform in a loop from the DDR memory on the PlutoSDR.
% The transmitted RF signal is oversampled and transmitted at 30 MHz.  The
% 802.11a signal is transmitted on channel 5, which corresponds to a center
% frequency of 2.432 GHz as defined in section 17.4.6.3 of [1].

sdrTransmitter = sdrtx(deviceNameSDR); % Transmitter properties
sdrTransmitter.RadioID = 'usb:0';

% Resample the transmit waveform at 30 MHz
fs = wlanSampleRate(nonHTcfg); % Transmit sample rate in MHz
osf = 1.5;                     % Oversampling factor

sdrTransmitter.BasebandSampleRate = fs*osf; 
sdrTransmitter.CenterFrequency = 830e6;  % Channel 5, 830MHz 로 변경해서 전송 (pluto 맞춤)
sdrTransmitter.ShowAdvancedProperties = true;
sdrTransmitter.Gain = txGain;

% Initialize the scrambler with a random integer for each packet
scramblerInitialization = randi([1 127],numMSDUs,1);

% Generate baseband NonHT packets separated by idle time 
txWaveform = wlanWaveformGenerator(data,nonHTcfg, ...
    'NumPackets',numMSDUs,'IdleTime',20e-6, ...
    'ScramblerInitialization',scramblerInitialization);

% Resample transmit waveform 
txWaveform  = resample(txWaveform,fs*osf,fs);

fprintf('\nGenerating WLAN transmit waveform:\n')

% Scale the normalized signal to avoid saturation of RF stages
powerScaleFactor = 0.8;
txWaveform = txWaveform.*(1/max(abs(txWaveform))*powerScaleFactor);

% Transmit RF waveform
sdrTransmitter.transmitRepeat(txWaveform);

%%
% *Repeated transmission using SDR Hardware*
%
% The |transmitRepeat| function transfers the baseband WLAN packets with
% idle time to the PlutoSDR, and stores the signal samples in hardware
% memory. The example then transmits the waveform continuously over the air
% until the release method of the transmit object is called. Messages are
% displayed in the command window to confirm that transmission has started
% successfully.