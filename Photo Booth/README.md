# Photo Booth: RF controlled Camera
> Term project in teams of two (Kyle Jonghyuk Park, Joongseon Park)

# 1. Objective
- Wireless Camera Control via RF signal
    * Control the camera's angle adjustment and camera shooting commands with buttons
- Transmit images using digital modulation
- Utilize image processing techniques

# 2. Flow & Block Diagram
<img src="/Photo Booth/images/flow chart.jpg" width="28%" height="28%" title="flow" alt="flow"></img>
<img src="/Photo Booth/images/block diagram.jpg" width="58%" height="58%" title="flow" alt="flow"></img>

## (1) Controlling Camera
Depending on the input of the pushbutton, it generates sine waves of different frequencies and sends them to Photo Booth via Adalm Pluto. 

- This process is repeated until the camera shoot button is pressed 4 times, causing the Photo Booth to take 4 pictures

### _User_
- When the button is not pressed, it sends a fixed 100 KHz sine wave constantly
- When a button is pressed, it reads the digital input of the connected Arduino and sends a sine wave of the frequency assigned to each button


### _Photo Booth_
- Fourier transform the sine wave signal sent by the user and find the frequency with the largest amplitude 
- Perform different actions depending on the frequency of transmitted signal 
    * Camera shoot
    * Servo motor control (10 degrees each)

## (2) Image Processing
- After taking four photos, the contrast and light levels of the images will be adjusted
- Four images are stitched together with the pre-made background, and save it into .png file

<img src="/Photo Booth/images/version1.jpg" width="21%" height="21%" title="flow" alt="flow"></img>
<img src="/Photo Booth/images/version2.jpg" width="20%" height="20%" title="flow" alt="flow"></img>
<img src="/Photo Booth/images/version3.jpg" width="20%" height="20%" title="flow" alt="flow"></img>

* From left, original, enhanced, and black-and-white 

## (3) Image Transmission via Digital Modulation (64-QAM)

### _Photo Booth_ sending 
<img src="/Photo Booth/images/img tran1.jpg" width="50%" height="50%" title="flow" alt="flow"></img>
<img src="/Photo Booth/images/img tran11.jpg" width="21%" height="21%" title="flow" alt="flow"></img>

### _User_ receiving
<img src="/Photo Booth/images/img tran2.jpg" width="50%" height="50%" title="flow" alt="flow"></img>
<img src="/Photo Booth/images/img tran22.jpg" width="21%" height="21%" title="flow" alt="flow"></img>

- Referenced the Matlab documentation below
- "Image Transmission and Reception Using 802.11 Waveform and SDR", https://www.mathworks.com/help/supportpkg/plutoradio/ug/transmission-and-reception-of-an-image-using-wlan-system-toolbox-and-a-single-pluto-radio.html

# Overall hardware configuration
<img src="/Photo Booth/images/hw1.jpg" width="20%" height="20%" title="flow" alt="flow"></img>
<img src="/Photo Booth/images/hw2.jpg" width="20%" height="20%" title="flow" alt="flow"></img>
<img src="/Photo Booth/images/hw3.jpg" width="20%" height="20%" title="flow" alt="flow"></img>

- Arduino, Adalm Pluto, USB Webcam, Servo motor, Buttons
