# Set the cmake version and define the project
cmake_minimum_required(VERSION 3.5)

project(phaselib)

# Add the --std=c++CXX flag to the compilation of all targets
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Download external sources related to this project
include(${CMAKE_SOURCE_DIR}/cmake/DownloadGTest.cmake)
include(CTest)

# Add compilation targets:

##################
# Subdirectories #
##################
add_subdirectory(src)

# vim: set ts=4 sw=4 tw=80 et :
