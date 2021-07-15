# ni-dsa-sync

## Overview
<p>This repository contains example programs that demonstrate the synchronization of measurements between NI's DSA and other modular instruments. C source code and associated build files are included, and they can be deployed to NI Linux Real-Time Systems.
  
Three synchronization types are demonstrated:
  * Reference clock 
  * Sample clock 
  * Channel expansion

In all examples, a DSA and multifunction IO (MIO) device were used to retrieve voltage measurements. The code can be modified to accomodate different device types. 
 
NOTE: to measure the phase-skew between the two measurements, the FFTW3 library is utilized (visit FFTW.org to learn more).</p>

## Links:
* To learn more about the NI Linux Real-Time OS, visit this [link](https://www.ni.com/en-us/shop/linux.html "link").
* To learn more about device synchronization, vis this [link](https://www.ni.com/en-us/support/documentation/supplemental/10/synchronization-explained.html).
* To learn about the basics of synchronizing DSA devices, visit this [link](https://www.ni.com/en-us/support/documentation/supplemental/10/dynamic-signal-acquisition--dsa--synchronization-basics.html "link").
* To learn how to cross-compile code and deploy to the NI Linux RTOS using Microsoft VSCode, visit this [link](https://forums.ni.com/t5/NI-Linux-Real-Time-Documents/NI-Linux-Real-Time-Cross-Compiling-Using-the-NI-Linux-Real-Time/ta-p/4026449 "link").
