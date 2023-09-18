# Patch Antenna Design

# 1. Introdudction
> (1) Design and simulate Patch Antenna with AWS design software tool
> (2) Measure S21 paramters of designed patch antenna with Pluto

## Before
- Used the calculator provided by the site below to find the width and length of the patch antenna.
    * Frequency : 3.5 GHz (Carrier Freq.)
    * Dielectric constant : 3.5
    * Dielectric Height : 0.8 mm

https://www.everythingrf.com/rf-calculators/microstrip-patch-antenna-calculator

# 2. Design Flow 
<img src="/Antenna Design/images/design flow.png" width="75%" height="75%" title="flow" alt="flow"></img>

## (1) Set Units, Frequency

<img src="/Antenna Design/images/flow1-1.png" width="35%" height="35%" title="flow" alt="flow"></img>
<img src="/Antenna Design/images/flow1-2.png" width="35%" height="35%" title="flow" alt="flow"></img>

- 'Option' > 'Project Options'
- Set units
- Set frequency domain to simulate near the carrier frequency(3.5 GHz) of the patch antenna

## (2) Set PCB
<img src="/Antenna Design/images/flow2-1.png" width="35%" height="35%" title="flow" alt="flow"></img>
<img src="/Antenna Design/images/flow2-2.png" width="35%" height="35%" title="flow" alt="flow"></img>
<img src="/Antenna Design/images/flow2-2.png" width="35%" height="35%" title="flow" alt="flow"></img>

- 'Project' > 'EM Structures' > 'New EM Structure'
- Set the material, information, thickness, and layers of the patch antenna substrate to be designed
- Set the length of the simulation grid

## (3) Draw Layout
<img src="/Antenna Design/images/flow3.jpg" width="35%" height="35%" title="flow" alt="flow"></img>

- Draw a layout based on the numbers calculated by the Microstrip Calculator

## (4) Simulations

### (4-1) S11 Parameters
<img src="/Antenna Design/images/flow4-1.jpg" width="60%" height="60%" title="flow" alt="flow"></img>

- Plot S11 parameters
- Simulation results show that the carrier frequency of __3.57 GHz__ has the lowest return loss with a return voltage ratio of -35.12 dB

### (4-2) Radiation Pattern
<img src="/Antenna Design/images/flow4-2.jpg" width="60%" height="60%" title="flow" alt="flow"></img>

- By checking the antenna radiation pattern, you can see that the radiation intensity is measured uniformly in the direction of the patch antenna


## (5) Generating Gerber file
<img src="/Antenna Design/images/flow5.jpg" width="40%" height="40%" title="flow" alt="flow"></img>

- 'Circuit Schematics' > 'new Schematic' > 'View layout'
- Gerber file : a file used for PCB fabrication, 3D schematic representation of a PCB


# Individual Project
# 1. Simulation
<img src="/Antenna Design/images/flow4-3-1.jpg" width="60%" height="60%" title="flow" alt="flow"></img>

- 'enclosure' > 'Dieletric Layers'
- By setting air between the antennas, I increased the number of layers to 5
- Adjusted the distance between the antennas while adjusting the thickness of the Layer 3

<img src="/Antenna Design/images/flow4-3-2.jpg" width="60%" height="60%" title="flow" alt="flow"></img>

- Changed the shape property

<img src="/Antenna Design/images/flow4-3-3.jpg" width="60%" height="60%" title="flow" alt="flow"></img>

- EM structure after layer setup is complete
- Set the top and bottom to face parallel

## (1) d = 5cm, S21 Parameter plot & Radiation Pattern
<img src="/Antenna Design/images/flow4-3-4.jpg" width="60%" height="60%" title="flow" alt="flow"></img>
<img src="/Antenna Design/images/flow4-3-5.jpg" width="60%" height="60%" title="flow" alt="flow"></img>

## (2) d = 15cm, S21 Parameter plot & Radiation Pattern
<img src="/Antenna Design/images/flow4-3-6.jpg" width="60%" height="60%" title="flow" alt="flow"></img>
<img src="/Antenna Design/images/flow4-3-7.jpg" width="60%" height="60%" title="flow" alt="flow"></img>

## (3) d = 25cm, S21 Parameter plot & Radiation Pattern
<img src="/Antenna Design/images/flow4-3-8.jpg" width="60%" height="60%" title="flow" alt="flow"></img>
<img src="/Antenna Design/images/flow4-3-9.jpg" width="60%" height="60%" title="flow" alt="flow"></img>

## Result
- S21 parameter dB shows a peak at the carrier frequency
- As the distance between the patch antennas increases, the antenna radiation intensity worsens

# 2. S21 Parameter Measure of Designed Patch Antenna with Pluto

## Patch antenna connected to Adalm Pluto
<img src="/Antenna Design/images/indiv1.jpg" width="60%" height="60%" title="flow" alt="flow"></img>


# Case 1 : Placing two Patch antennas parallel to each other
<img src="/Antenna Design/images/case1.jpg" width="60%" height="60%" title="flow" alt="flow"></img>

## (1) S21 Parameter plot
### A. d = 5cm
<img src="/Antenna Design/images/indiv2.jpg" width="40%" height="40%" title="flow" alt="flow"></img>
<img src="/Antenna Design/images/indiv3.jpg" width="40%" height="40%" title="flow" alt="flow"></img>


### B. d = 15cm
<img src="/Antenna Design/images/indiv4.jpg" width="40%" height="40%" title="flow" alt="flow"></img>
<img src="/Antenna Design/images/indiv5.jpg" width="40%" height="40%" title="flow" alt="flow"></img>

### C. d = 25cm
<img src="/Antenna Design/images/indiv6.jpg" width="40%" height="40%" title="flow" alt="flow"></img>
<img src="/Antenna Design/images/indiv7.jpg" width="40%" height="40%" title="flow" alt="flow"></img>




