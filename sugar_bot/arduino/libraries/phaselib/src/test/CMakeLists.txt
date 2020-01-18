####################
# Phase Space Test #
####################

add_executable(AcrobotDynamicsTest AcrobotDynamicsTest.cpp)
target_link_libraries(AcrobotDynamicsTest
	PhaseLib
	gtest
)

add_executable(PhaseSpaceTest PhaseSpaceTest.cpp)
target_link_libraries(PhaseSpaceTest
	PhaseLib
    gtest
)

add_test(NAME acrobot_inv_inertia_test COMMAND AcrobotDynamicsTest)
add_test(NAME phase_space_test COMMAND PhaseSpaceTest)

# vim:set noet sts=0 sw=4 ts=4 tw=80 :
