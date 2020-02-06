# SUGAR_VNHC_LIB
This library enables VNHC control of the SUGAR Acrobot system. 

This library contains two sections:
* The Arduino library, consisting of symbolic links to the actual code.
* The pure C++ code, which can be compiled and tested on any computer.

The pure C++ code is found in the `include/` and `src/` folders, 
while the folder labelled `SUGAR_VNHC_Library` has symbolic links to the relevant files that the
Arduino can use to compile the code.

If you want to compile and test the code on a computer, perform the following commands from the root folder of this library:
```
mkdir build/
cd build
../scripts/run_cmake_config.sh ..
cmake .
ninja
```
Optionally, you can test the code with `ninja test`, which runs the gtest suite.

If you want to include the library in your Arduino, copy this whole folder into your Arduino libraries folder (in Windows this is usually under `Documents\Arduino\libraries\`). Then, in your Arduino IDE you can load in the folder labelled `SUGAR_VNHC_Library`. Simply include the file `VNHC.h`.
