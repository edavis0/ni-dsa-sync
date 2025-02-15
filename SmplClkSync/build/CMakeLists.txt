set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR x86_64)
set(toolchain_path C:/build/18.0/x64/sysroots)
set(CMAKE_C_COMPILER ${toolchain_path}/i686-nilrtsdk-mingw32/usr/bin/x86_64-nilrt-linux/x86_64-nilrt-linux-gcc.exe)
set(CMAKE_CXX_COMPILER ${toolchain_path}/i686-nilrtsdk-mingw32/usr/bin/x86_64-nilrt-linux/x86_64-nilrt-linux-g++.exe)
set(CMAKE_SYSROOT ${toolchain_path}/core2-64-nilrt-linux)
set(CMAKE_<LANG>_STANDARD_INCLUDE_DIRECTORIES ${toolchain_path}/core2-64-nilrt-linux/usr/include/c++/6.3.0 ${toolchain_path}
    /core2-64-nilrt-linux/usr/include/c++/6.3.0/x86_64-nilrt-linux)
set(CMAKE_<LANG>_FLAGS "-Wall -fmessage-length=0")
set(CMAKE_<LANG>_FLAGS_DEBUG "-O0 -g3")
set(CMAKE_<LANG>_FLAGS_RELEASE "-O3")
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)

# project specific information
cmake_minimum_required(VERSION 3.7.2)
project(SmplClkSync)
set(EXECUTABLE_OUTPUT_PATH bin)
set(CMAKE_BUILD_TYPE Debug)

include_directories(${toolchain_path}/core2-64-nilrt-linux/usr/include)
set(HEADER_DIR "C:/Program\ Files\ (x86)/National\ Instruments/NI-DAQ/DAQmx\ ANSI\ C\ Dev/include")
set(DAQMXLIBPATH "C:/Program\ Files\ (x86)/National\ Instruments/Shared/ExternalCompilerSupport/C/lib64/gcc")
add_executable(SmplClkSync ../src/SmplClkSync.c ${HEADER_DIR}/NIDAQMX.h)
target_include_directories(SmplClkSync PUBLIC ${HEADER_DIR})

# Adding libraries for FFTW functionality
add_library(libm SHARED IMPORTED)
set_property(TARGET libm PROPERTY IMPORTED_LOCATION ${toolchain_path}/core2-64-nilrt-linux/usr/lib/libm.so)
add_library(libfftw3 SHARED IMPORTED)
set_property(TARGET libfftw3 PROPERTY IMPORTED_LOCATION ${toolchain_path}/core2-64-nilrt-linux/usr/lib/libfftw3.so.3.5.7)

target_link_libraries(SmplClkSync PUBLIC libm libfftw3 ${DAQMXLIBPATH}/libnidaqmx.so)