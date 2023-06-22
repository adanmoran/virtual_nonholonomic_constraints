# SUGAR_VNHC_LIB
This library enables VNHC control of the SUGAR Acrobot system. 

## Using the Library in Arduino
To use this library in Windows on an Arduino, you must first copy this entire folder to
your Arduino libraries folder. This is usually located in `Documents\Arduino\libraries\`. 

IMPORTANT: You must then run the batch script in the `scripts` folder to copy the relevant cpp files to
this folder's root directory. This will allow the Arduino compiler to actually compile the library for use.

Simply put
```c++
#include "SUGAR_VNHC.h"
```
at the top of your arduino code to include this library and use all of its components.

## Editing the library 
If you want to compile and test the code, especially on a linux machine, 
perform the following commands from the root folder of this library:
```
mkdir build/
cd build
../scripts/run_cmake_config.sh ..
cmake .
ninja
```
Optionally, you can test the code with `ninja test`, which runs the gtest suite.

