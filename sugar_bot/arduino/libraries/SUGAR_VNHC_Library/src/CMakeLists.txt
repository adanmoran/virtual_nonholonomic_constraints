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

add_library(VNHCLib
    VNHC.cpp
)
target_include_directories(VNHCLib PUBLIC
    ${CMAKE_SOURCE_DIR}/includ/
)
target_link_libraries(VNHCLib
    PhaseLib
)

########
# Test #
########
add_subdirectory(test)

# vim:set et sts=0 sw=4 ts=4 tw=80 :
