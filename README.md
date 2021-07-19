# ni-dsa-sync

## Overview
<p>This repository contains example programs that demonstrate the synchronization of measurements between NI's DSA and multifunction IO (MIO) devices. C source code and associated build files are included, and they can be deployed to NI Linux Real-Time Systems.
  
Three synchronization types are demonstrated:
  * Reference clock 
  * Sample clock 
  * Channel expansion

In all examples, a DSA and a MIO device were used to retrieve voltage measurements. The code can be modified to accomodate different device types. 
 
NOTE: to measure the phase shift between the two measurements, the FFTW3 library is utilized (visit FFTW.org to learn more).</p>

## Code Output
<p>The programs allow users to store voltage measurements and Discrete Fourier Transform (DFT) data to separate .CSV files. The console output displays the number of samples acquired for each device, the detected signal frequency in Hz, the phase shift in degrees, and the phase shift in seconds.</p>
  
![Console output](https://github.com/edavis0/ni-dsa-sync/blob/main/ConsoleOutImage.png)
  
## Building
To compile the source code on your host machine, you must install the GNU C/C++ Compile Tools for [x64 Linux][1] or [ARMv7 Linux][2]. Make sure to include the CMakeLists.txt file and .vscode directories when building the binary (included in "samplebuildfiles").

Note: It is highly recommended that you first learn how to cross-compile code and deploy to the NI Linux RTOS using Microsoft VSCode by visiting this [NI Forum Post](https://forums.ni.com/t5/NI-Linux-Real-Time-Documents/NI-Linux-Real-Time-Cross-Compiling-Using-the-NI-Linux-Real-Time/ta-p/4026449 "NI forum post").
  
## Reference Material
* To learn more about the NI Linux Real-Time OS, visit this [link](https://www.ni.com/en-us/shop/linux.html "link").
* To learn more about device synchronization, visit this [link](https://www.ni.com/en-us/support/documentation/supplemental/10/synchronization-explained.html).
* To learn about the basics of synchronizing DSA devices, visit this [link](https://www.ni.com/en-us/support/documentation/supplemental/10/dynamic-signal-acquisition--dsa--synchronization-basics.html "link").

[1]: https://www.ni.com/en-us/support/downloads/software-products/download.gnu-c---c---compile-tools-x64.html#338442 "x64 Linux" 
[2]: https://www.ni.com/en-us/support/downloads/software-products/download.gnu-c---c---compile-tools-for-armv7.html#338448 "ARMv7 Linux" 
