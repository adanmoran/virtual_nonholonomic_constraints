#######################
# Phase Space Library #
#######################
add_library(PhaseLib 
    AcrobotDynamics.cpp
    Phase.cpp
)
target_include_directories(PhaseLib PUBLIC
    ${CMAKE_SOURCE_DIR}/include/
)

########
# Test #
########
add_subdirectory(test)

# vim:set et sts=0 sw=4 ts=4 tw=80 :
