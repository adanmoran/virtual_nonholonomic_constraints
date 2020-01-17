####################
# Phase Space Test #
####################

add_executable(AcrobotInverseInertiaTest AcrobotInverseInertiaTest.cpp)
target_link_libraries(AcrobotInverseInertiaTest
	PhaseLib
	gtest
)

add_executable(PhaseSpaceTest PhaseSpaceTest.cpp)
target_link_libraries(PhaseSpaceTest
	PhaseLib
    gtest
)

add_test(NAME acrobot_inv_inertia_test COMMAND AcrobotInverseInertiaTest)
add_test(NAME phase_space_test COMMAND PhaseSpaceTest)

# vim:set noet sts=0 sw=4 ts=4 tw=80 :
