# Patch Antenna Design
> Designd Patch Antenna and simulated it's S11 parameter with AWS design software tool

## Before
- Used the calculator provided by the site below to find the width and length of the patch antenna.
    * Frequency : 3.5 GHz (Carrier Freq.)
    * Dielectric constant : 3.5
    * Dielectric Height : 0.8 mm

https://www.everythingrf.com/rf-calculators/microstrip-patch-antenna-calculator

# Design Flow 
<img src="/Antenna Design/images/design flow.png" width="75%" height="75%" title="flow" alt="flow"></img>

## (1) Set Units, Frequency
<img src="/Antenna Design/images/flow1-1.JPG" width="35%" height="35%" title="flow" alt="flow"></img>
<img src="/Antenna Design/images/flow1-2.JPG" width="35%" height="35%" title="flow" alt="flow"></img>

- 'Option' > 'Project Options'
- Set units
- Set frequency domain to simulate near the carrier frequency(3.5 GHz) of the patch antenna

## (2) Set PCB
<img src="/Antenna Design/images/flow2-1.JPG" width="35%" height="35%" title="flow" alt="flow"></img>
<img src="/Antenna Design/images/flow2-2.JPG" width="35%" height="35%" title="flow" alt="flow"></img>
<img src="/Antenna Design/images/flow2-3.JPG" width="35%" height="35%" title="flow" alt="flow"></img>

- 'Project' > 'EM Structures' > 'New EM Structure'
- Set the material, information, thickness, and layers of the patch antenna substrate to be designed
- Set the length of the simulation grid

## (3) Draw Layout
<img src="/Antenna Design/images/flow3.jpg" width="35%" height="35%" title="flow" alt="flow"></img>

- Draw a layout based on the numbers calculated by the Microstrip Calculator

## (4) Simulations

### (4-1) S11 Parameter
<img src="/Antenna Design/images/flow4-1.jpg" width="60%" height="60%" title="flow" alt="flow"></img>

- Plot S11 parameter
- Simulation results show that the carrier frequency of __3.57 GHz__ has the lowest return loss with a return voltage ratio of -35.12 dB

### (4-2) Radiation Pattern
<img src="/Antenna Design/images/flow4-2.jpg" width="60%" height="60%" title="flow" alt="flow"></img>

- By checking the antenna radiation pattern, you can see that the radiation intensity is measured uniformly in the direction of the patch antenna


## (5) Generating Gerber file
<img src="/Antenna Design/images/flow5.jpg" width="40%" height="40%" title="flow" alt="flow"></img>

- 'Circuit Schematics' > 'new Schematic' > 'View layout'
- Gerber file : a file used for PCB fabrication, 3D schematic representation of a PCB