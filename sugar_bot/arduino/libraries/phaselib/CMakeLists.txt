cmake_minimum_required(VERSION 3.5)

project(phaselib)

include(${CMAKE_SOURCE_DIR}/cmake/DownloadGTest.cmake)

include(CTest)

##################
# Subdirectories #
##################
add_subdirectory(src)

# vim: set ts=4 sw=4 et :
