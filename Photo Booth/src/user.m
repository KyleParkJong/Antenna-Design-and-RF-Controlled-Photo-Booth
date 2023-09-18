clear all;
close all;
clc;

% Arduino Open %
clear a;
a = arduino('COM6','Nano33BLE')

% State (flag) %
IDLE = true; CAPTURE = false; UP = false; DOWN = false;
% Freq. %
idle = 100e3; selfi = 110e3; up = 120e3; down = 130e3;
% No. of Captures %
i = 1;
% Sine wave %
fs = 10e6;   % sampling freq.
sw = dsp.SineWave;
sw_setting(sw, idle, 1000);

% Transmit %
fc = 830e6; % Carrier Freq.
tx = sdrtx('Pluto');
tx.CenterFrequency = fc;
tx.BasebandSampleRate = fs;
tx.Gain = 0;
txdata = sw();    

while (i ~= 5)
    transmitRepeat(tx,txdata);
    
    % Arduino Button input %
    x = readDigitalPin(a, 'D2');
    y = readDigitalPin(a, 'D6');
    z = readDigitalPin(a, 'D9');
    
    % State setting %
    if x
        CAPTURE = true;
        IDLE = false;
    elseif y
        UP = true;
        IDLE = false;
    elseif z
        DOWN = true;
        IDLE = false;
    end
    
    if CAPTURE && ~IDLE
        release(sw);
        sw = dsp.SineWave;
        sw_setting(sw, selfi, 1000);
        txdata = sw();
        transmitRepeat(tx,txdata);
        pause(1);
        CAPTURE = false;
        IDLE = true;
        i = i + 1;
    end
    
    if UP && ~IDLE
        release(sw);
        sw = dsp.SineWave;
        sw_setting(sw, up, 1000);
        txdata = sw();
        transmitRepeat(tx,txdata);
        pause(1);
        UP = false; 
        IDLE = true;
    end
    
    if DOWN && ~IDLE
        release(sw);
        sw = dsp.SineWave;
        sw_setting(sw, down, 1000);
        txdata = sw();
        transmitRepeat(tx,txdata);
        pause(1);
        DOWN = false;
        IDLE = true;
    end
    
    % FFT %
    N = tx.SamplesPerFrame;    % samples per frame
    n = -N/2 : N/2-1;
    f = fs * n/N;
    Ft = fftshift(fft(txdata));

    [MAX, index] = max(Ft);
    freq = f(index);   % txdata freq.
    fprintf("txdata freq : %d\n", freq)
    
    release(sw);
    sw_setting(sw, idle, 1000);
    txdata = sw();
end
release(tx);

pause(6);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

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

% Plot transmit image
figure(imFig);
imFig.Visible = 'on';
% subplot(211); 
%     imshow(fData);
%     title('Transmitted Image');
% subplot(212);
%     title('Received image will appear here...');
%     set(gca,'Visible','off');
%     set(findall(gca, 'type', 'text'), 'visible', 'on');

% pause(1); % Pause to plot Tx image

msduLength = 2304; % MSDU length in bytes
numMSDUs = ceil(82944/msduLength);
padZeros = msduLength-mod(82944,msduLength);

% Divide input data stream into fragments
bitsPerOctet = 8;
data = zeros(0, 1);

for ind=0:numMSDUs-1
    
    % Extract image data (in octets) for each MPDU
    %frameBody = txData(ind*msduLength+1:msduLength*(ind+1),:);
    
    % Create MAC frame configuration object and configure sequence number
    cfgMAC = wlanMACFrameConfig('FrameType', 'Data', 'SequenceNumber', ind);
    
    % Generate MPDU
    %[mpdu, lengthMPDU]= wlanMACFrame(frameBody, cfgMAC);
    
    % Convert MPDU bytes to a bit stream
   %psdu = reshape(de2bi(hex2dec(mpdu), 8)', [], 1);
    
    % Concatenate PSDUs for waveform generation
    %data = [data; psdu]; %#ok<AGROW>
    
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
nonHTcfg.PSDULength = 2332;   % Set the PSDU length

% The sdrTransmitter uses the |transmitRepeat| functionality to transmit
% the baseband WLAN waveform in a loop from the DDR memory on the PlutoSDR.
% The transmitted RF signal is oversampled and transmitted at 30 MHz.  The
% 802.11a signal is transmitted on channel 5, which corresponds to a center
% frequency of 2.432 GHz as defined in section 17.4.6.3 of [1].

%sdrTransmitter = sdrtx(deviceNameSDR); % Transmitter properties
%sdrTransmitter.RadioID = 'usb:0';

% Resample the transmit waveform at 30 MHz
fs = wlanSampleRate(nonHTcfg); % Transmit sample rate in MHz
osf = 1.5;                     % Oversampling factor

%sdrTransmitter.BasebandSampleRate = fs*osf; 
%sdrTransmitter.CenterFrequency = 830e6;  % Channel 5, 830MHz 로 변경해서 전송 (pluto 맞춤)
%sdrTransmitter.ShowAdvancedProperties = true;
%sdrTransmitter.Gain = txGain;

% Initialize the scrambler with a random integer for each packet
scramblerInitialization = randi([1 127],numMSDUs,1);

% Generate baseband NonHT packets separated by idle time 
%txWaveform = wlanWaveformGenerator(data,nonHTcfg, ...
 %   'NumPackets',numMSDUs,'IdleTime',20e-6, ...
  %  'ScramblerInitialization',scramblerInitialization);

% Resample transmit waveform 
%txWaveform  = resample(txWaveform,fs*osf,fs);

%fprintf('\nGenerating WLAN transmit waveform:\n')

% Scale the normalized signal to avoid saturation of RF stages
powerScaleFactor = 0.8;
%txWaveform = txWaveform.*(1/max(abs(txWaveform))*powerScaleFactor);

% Transmit RF waveform
%sdrTransmitter.transmitRepeat(txWaveform);

%%
% *Repeated transmission using SDR Hardware*
%
% The |transmitRepeat| function transfers the baseband WLAN packets with
% idle time to the PlutoSDR, and stores the signal samples in hardware
% memory. The example then transmits the waveform continuously over the air
% until the release method of the transmit object is called. Messages are
% displayed in the command window to confirm that transmission has started
% successfully.

%% Receiver Design
%
% The general structure of the WLAN receiver can be described as follows:
%
% # Capture multiple packets of the transmitted WLAN signal using
% SDR hardware.
% # Detect a packet
% # Coarse carrier frequency offset is estimated and corrected
% # Fine timing synchronization is established. The L-STF, L-LTF and L-SIG
% samples are provided for fine timing to allow to adjust the packet
% detection at the start or end of the L-STF 
% # Fine carrier frequency offset is estimated and corrected
% # Perform a channel estimation for the received signal using the L-LTF
% # Detect the format of the packet
% # Decode the L-SIG field to recover the MCS value and the length of the
% data portion
% # Decode the data field to obtain the transmitted data within each
% packet
% # Decode the received PSDU and check if the frame check sequence (FCS)
% passed for the PSDU.
% # Order the decoded MSDUs based on the |SequenceNumber| property in the
% recovered MAC frame configuration object.
% # Combine the decoded MSDUs from all the transmitted packets to form the
% received image
%
% This example plots the power spectral density (PSD) of the captured
% waveform, and shows visualizations of the equalized data symbols, and
% the received image.
% 
%%
% *Receiver Setup*
% 
% The sdrReceiver is controlled using the properties defined in the
% |sdrReceiver| object. The sample rate of the receiver is 30 MHz, which is
% 1.5 times the baseband sample rate of 20 MHz.

%%
% An <matlab:plutoradiodoc('commsdrrxpluto') SDR Receiver> System object is
% used with the PlutoSDR to receive baseband data from the SDR hardware.
% Resample the transmit waveform at 30 MHz
fs = wlanSampleRate(nonHTcfg); % Transmit sample rate in MHz
osf = 1.5;                     % Oversampling factor
BasebandSampleRate = fs*osf; 

sdrReceiver = sdrrx(deviceNameSDR);
sdrReceiver.RadioID = 'usb:0';
sdrReceiver.BasebandSampleRate = BasebandSampleRate;
sdrReceiver.CenterFrequency = 830e6;
sdrReceiver.GainSource = 'Manual';
sdrReceiver.Gain = 20;
sdrReceiver.OutputDataType = 'double';

% Configure the capture length equivalent to twice the length of the
% transmitted signal, this is to ensure that PSDUs are received in order.
% On reception the duplicate MAC fragments are removed.
captureLength = 2*466560;
spectrumScope.SampleRate = sdrReceiver.BasebandSampleRate;

% Get the required field indices within a PSDU 
indLSTF = wlanFieldIndices(nonHTcfg,'L-STF'); 
indLLTF = wlanFieldIndices(nonHTcfg,'L-LTF'); 
indLSIG = wlanFieldIndices(nonHTcfg,'L-SIG');
Ns = indLSIG(2)-indLSIG(1)+1; % Number of samples in an OFDM symbol

%%
% *Capture Receive Packets* 
%%
% The transmitted waveform is captured using the PlutoSDR.
fprintf('\nStarting a new RF capture.\n')
    
burstCaptures = capture(sdrReceiver, captureLength, 'Samples');

%%
% *Receiver Processing* 
%%
% The example uses a while loop to capture and decode packets. The WLAN
% waveform is continually transmitted over the air in a loop, the first
% packet that is captured by the sdrReceiver is not guaranteed to be the
% first packet that was transmitted. This means that the packets may be
% decoded out of sequence. To enable the received packets to be recombined
% in the correct order, their sequence number must be determined. The
% decoded PSDU bits for each packet are passed to the
% <matlab:doc('wlanMPDUDecode') wlanMPDUDecode> function. This function
% decodes the MPDU and outputs the MSDU as well as the recovered MAC frame
% configuration object <matlab:doc('wlanMACFrameConfig')
% wlanMACFrameConfig>. The |SequenceNumber| property in the recovered MAC
% frame configuration object can be used for ordering the MSDUs in the
% transmitted sequence. The while loop finishes receive processing when a
% duplicate frame is detected, which is finally removed during receiver
% processing. In case of a missing frame the quality of the image is
% degraded.
%
% When the WLAN packet has successfully decoded, the detected sequence
% number is displayed in the command window for each received packet. The
% equalized data symbol constellation is shown for each received packet.

% Show power spectral density of the received waveform
spectrumScope(burstCaptures);

% Downsample the received signal
rxWaveform = resample(burstCaptures,fs,fs*osf);
rxWaveformLen = size(rxWaveform,1);
searchOffset = 0; % Offset from start of the waveform in samples

% Minimum packet length is 10 OFDM symbols
lstfLen = double(indLSTF(2)); % Number of samples in L-STF
minPktLen = lstfLen*5;
pktInd = 1;
sr = wlanSampleRate(nonHTcfg); % Sampling rate
fineTimingOffset = [];
packetSeq = [];
displayFlag = 0; % Flag to display the decoded information

% Perform EVM calculation
evmCalculator = comm.EVM('AveragingDimensions',[1 2 3]);
evmCalculator.MaximumEVMOutputPort = true;

% Receiver processing
while (searchOffset + minPktLen) <= rxWaveformLen    
    % Packet detect
    pktOffset = wlanPacketDetect(rxWaveform, chanBW, searchOffset, 0.8);
 
    % Adjust packet offset
    pktOffset = searchOffset+pktOffset;
    if isempty(pktOffset) || (pktOffset+double(indLSIG(2))>rxWaveformLen)
        if pktInd==1
            disp('** No packet detected **');
        end
        break;
    end

    % Extract non-HT fields and perform coarse frequency offset correction
    % to allow for reliable symbol timing
    nonHT = rxWaveform(pktOffset+(indLSTF(1):indLSIG(2)),:);
    coarseFreqOffset = wlanCoarseCFOEstimate(nonHT,chanBW); 
    nonHT = helperFrequencyOffset(nonHT,fs,-coarseFreqOffset);

    % Symbol timing synchronization
    fineTimingOffset = wlanSymbolTimingEstimate(nonHT,chanBW);
    
    % Adjust packet offset
    pktOffset = pktOffset+fineTimingOffset;

    % Timing synchronization complete: Packet detected and synchronized
    % Extract the non-HT preamble field after synchronization and
    % perform frequency correction
    if (pktOffset<0) || ((pktOffset+minPktLen)>rxWaveformLen) 
        searchOffset = pktOffset+1.5*lstfLen; 
        continue; 
    end
    fprintf('\nPacket-%d detected at index %d\n',pktInd,pktOffset+1);
  
    % Extract first 7 OFDM symbols worth of data for format detection and
    % L-SIG decoding
    nonHT = rxWaveform(pktOffset+(1:7*Ns),:);
    nonHT = helperFrequencyOffset(nonHT,fs,-coarseFreqOffset);

    % Perform fine frequency offset correction on the synchronized and
    % coarse corrected preamble fields
    lltf = nonHT(indLLTF(1):indLLTF(2),:);           % Extract L-LTF
    fineFreqOffset = wlanFineCFOEstimate(lltf,chanBW);
    nonHT = helperFrequencyOffset(nonHT,fs,-fineFreqOffset);
    cfoCorrection = coarseFreqOffset+fineFreqOffset; % Total CFO

    % Channel estimation using L-LTF
    lltf = nonHT(indLLTF(1):indLLTF(2),:);
    demodLLTF = wlanLLTFDemodulate(lltf,chanBW);
    chanEstLLTF = wlanLLTFChannelEstimate(demodLLTF,chanBW);

    % Noise estimation
    noiseVarNonHT = helperNoiseEstimate(demodLLTF);

    % Packet format detection using the 3 OFDM symbols immediately
    % following the L-LTF
    format = wlanFormatDetect(nonHT(indLLTF(2)+(1:3*Ns),:), ...
        chanEstLLTF,noiseVarNonHT,chanBW);
    disp(['  ' format ' format detected']);
    if ~strcmp(format,'Non-HT')
        fprintf('  A format other than Non-HT has been detected\n');
        searchOffset = pktOffset+1.5*lstfLen;
        continue;
    end
    
    % Recover L-SIG field bits
    [recLSIGBits,failCheck] = wlanLSIGRecover( ...
           nonHT(indLSIG(1):indLSIG(2),:), ...
           chanEstLLTF,noiseVarNonHT,chanBW);

    if failCheck
        fprintf('  L-SIG check fail \n');
        searchOffset = pktOffset+1.5*lstfLen;
        continue; 
    else
        fprintf('  L-SIG check pass \n');
    end

    % Retrieve packet parameters based on decoded L-SIG
    [lsigMCS,lsigLen,rxSamples] = helperInterpretLSIG(recLSIGBits,sr);

    if (rxSamples+pktOffset)>length(rxWaveform)
        disp('** Not enough samples to decode packet **');
        break;
    end
    
    % Apply CFO correction to the entire packet
    rxWaveform(pktOffset+(1:rxSamples),:) = helperFrequencyOffset(...
        rxWaveform(pktOffset+(1:rxSamples),:),fs,-cfoCorrection);

    % Create a receive Non-HT config object
    rxNonHTcfg = wlanNonHTConfig;
    rxNonHTcfg.MCS = lsigMCS;
    rxNonHTcfg.PSDULength = lsigLen;

    % Get the data field indices within a PPDU 
    indNonHTData = wlanFieldIndices(rxNonHTcfg,'NonHT-Data');

    % Recover PSDU bits using transmitted packet parameters and channel
    % estimates from L-LTF
    [rxPSDU,eqSym] = wlanNonHTDataRecover(rxWaveform(pktOffset+...
           (indNonHTData(1):indNonHTData(2)),:), ...
           chanEstLLTF,noiseVarNonHT,rxNonHTcfg);

    constellation(reshape(eqSym,[],1)); % Current constellation
    pause(0); % Allow constellation to repaint
    release(constellation); % Release previous constellation plot

    refSym = wlanClosestReferenceSymbol(eqSym,rxNonHTcfg);
    [evm.RMS,evm.Peak] = evmCalculator(refSym,eqSym);

    % Decode the MPDU and extract MSDU
    [cfgMACRx, msduList{pktInd}, status] = wlanMPDUDecode(rxPSDU, rxNonHTcfg); %#ok<*SAGROW>

    if strcmp(status, 'Success')
        disp('  MAC FCS check pass');
        
        % Store sequencing information
        packetSeq(pktInd) = cfgMACRx.SequenceNumber;
        
        % Convert MSDU to a binary data stream
        rxBit{pktInd} = reshape(de2bi(hex2dec(cell2mat(msduList{pktInd})), 8)', [], 1);
        
    else % Decoding failed
        if strcmp(status, 'FCSFailed')
            % FCS failed
            disp('  MAC FCS check fail');
        else
            % FCS passed but encountered other decoding failures
            disp('  MAC FCS check pass');
        end
        
        % Since there are no retransmissions modeled in this example, we'll
        % extract the image data (MSDU) and sequence number from the MPDU,
        % even though FCS check fails.
        
        % Remove header and FCS. Extract the MSDU.
        macHeaderBitsLength = 24*bitsPerOctet;
        fcsBitsLength = 4*bitsPerOctet;
        msduList{pktInd} = rxPSDU(macHeaderBitsLength+1 : end-fcsBitsLength);
        
        % Extract and store sequence number
        sequenceNumStartIndex = 23*bitsPerOctet+1;
        sequenceNumEndIndex = 25*bitsPerOctet - 4;
        packetSeq(pktInd) = bi2de(rxPSDU(sequenceNumStartIndex:sequenceNumEndIndex)');
        
        % MSDU binary data stream
        rxBit{pktInd} = double(msduList{pktInd});
    end
    
    % Display decoded information
    if displayFlag
        fprintf('  Estimated CFO: %5.1f Hz\n\n',cfoCorrection); %#ok<UNRCH>

        disp('  Decoded L-SIG contents: ');
        fprintf('                            MCS: %d\n',lsigMCS);
        fprintf('                         Length: %d\n',lsigLen);
        fprintf('    Number of samples in packet: %d\n\n',rxSamples);

        fprintf('  EVM:\n');
        fprintf('    EVM peak: %0.3f%%  EVM RMS: %0.3f%%\n\n', ...
        evm.Peak,evm.RMS);

        fprintf('  Decoded MAC Sequence Control field contents:\n');
        fprintf('    Sequence number:%d\n',packetSeq(pktInd));
    end

    % Update search index
    searchOffset = pktOffset+double(indNonHTData(2));

    
    pktInd = pktInd+1;
    % Finish processing when a duplicate packet is detected. The
    % recovered data includes bits from duplicate frame
    if length(unique(packetSeq))<length(packetSeq)
        break
    end  
end

% Release the state of sdrTransmitter and sdrReceiver object
release(sdrReceiver);

%%
% *Reconstruct Image*
%%
% The image is reconstructed from the received MAC frames.
if ~(isempty(fineTimingOffset)||isempty(pktOffset))&& ...
        (numMSDUs==(numel(packetSeq)-1))
    % Remove the duplicate captured MAC fragment
    rxBitMatrix = cell2mat(rxBit); 
    rxData = rxBitMatrix(1:end,1:numel(packetSeq)-1);

    startSeq = find(packetSeq==0);
    rxData = circshift(rxData,[0 -(startSeq(1)-1)]);% Order MAC fragments

%     % Perform bit error rate (BER) calculation
%     bitErrorRate = comm.ErrorRate;
%     err = bitErrorRate(double(rxData(:)), ...
%                     txDataBits(1:length(reshape(rxData,[],1))));
%     fprintf('  \nBit Error Rate (BER):\n');
%     fprintf('          Bit Error Rate (BER) = %0.5f.\n',err(1));
%     fprintf('          Number of bit errors = %d.\n', err(2));
%     fprintf('    Number of transmitted bits = %d.\n\n',length(txDataBits));

    % Recreate image from received data
    imsize = [288 96 3];
    fprintf('\nConstructing image from received data.\n');
    
    decdata = bi2de(reshape(rxData(1:82944*bitsPerOctet), 8, [])');   
  
    receivedImage = uint8(reshape(decdata,imsize));
    % Plot received image
%     if exist('imFig', 'var') && ishandle(imFig) % If Tx figure is open
%         figure(imFig); subplot(212); 
%     else
%         figure; subplot(212);
%     end
    imshow(receivedImage);
    title(sprintf('Received Image'));
end

%% Things to Try
% You can modify the sdrTransmitter |txGain| gain to observe the difference in
% the EVM and BER after signal reception and processing. You should also be
% able to see any errors in the displayed, received image. Try changing the
% scaling factor |scale| to 0.5. This should improve the quality of the
% received image by generating more transmit bits. This should also
% increase the number of transmitted PPDUs.

%% Troubleshooting
%
% General tips for troubleshooting SDR hardware and the Communications
% Toolbox Support Package for ADALM-PLUTO Radio can be found in
% <matlab:plutoradiodoc('sdrpluto_troubleshoot') Common Problems and Fixes>.

%% Selected Bibliography
% # IEEE Std 802.11(TM)-2012 IEEE Standard for Information technology -
% Telecommunications and information exchange between systems - Local and
% metropolitan area networks - Specific requirements - Part 11: Wireless
% LAN Medium Access Control (MAC) and Physical Layer (PHY) Specifications.
%imshow(receivedImage);
displayEndOfDemoMessage(mfilename)

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function sw_setting(sw, freq, samplesPerFrame)
    fs = 10e6;
    sw.Amplitude = 0.5;
    sw.Frequency = freq;
    sw.ComplexOutput = true;
    sw.SampleRate = fs;
    sw.SamplesPerFrame = samplesPerFrame; % to meet waveform size requirements
end