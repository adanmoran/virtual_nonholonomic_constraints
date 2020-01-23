####################
# Phase Space Test #
####################

add_executable(AcrobotDynamicsTest AcrobotDynamicsTest.cpp)
target_link_libraries(AcrobotDynamicsTest
	PhaseLib
	gtest
)

add_executable(PhaseTest PhaseTest.cpp)
target_link_libraries(PhaseTest
	PhaseLib
    gtest
)

add_test(NAME acrobot_inv_inertia_test COMMAND AcrobotDynamicsTest)
add_test(NAME phase_test COMMAND PhaseTest)

# vim:set noet sts=0 sw=4 ts=4 tw=80 :
