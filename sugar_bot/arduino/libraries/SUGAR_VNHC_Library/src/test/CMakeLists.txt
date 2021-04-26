####################
# Phase Space Test #
####################

add_executable(AcrobotDynamicsTest AcrobotDynamicsTest.cpp)
target_include_directories(AcrobotDynamicsTest PUBLIC
	${CMAKE_SOURCE_DIR}/include/
)
target_link_libraries(AcrobotDynamicsTest
	PhaseLib
	gtest
)

add_executable(PhaseTest PhaseTest.cpp)
target_include_directories(PhaseTest PUBLIC
	${CMAKE_SOURCE_DIR}/include/
)
target_link_libraries(PhaseTest
	PhaseLib
    gtest
)

add_executable(VNHCTest VNHCTest.cpp)
target_include_directories(VNHCTest PUBLIC
	${CMAKE_SOURCE_DIR}/include/
)
target_link_libraries(VNHCTest
	VNHCLib
    gtest
)

add_executable(SupervisorTest SupervisorTest.cpp)
target_include_directories(SupervisorTest PUBLIC
	${CMAKE_SOURCE_DIR}/include/
)
target_link_libraries(SupervisorTest
	VNHCLib
    gtest
)

add_test(NAME acrobot_dynamics_test COMMAND AcrobotDynamicsTest)
add_test(NAME phase_test COMMAND PhaseTest)
add_test(NAME vnhc_test COMMAND VNHCTest)
add_test(NAME supervisor_test COMMAND SupervisorTest)

# vim:set noet sts=0 sw=4 ts=4 tw=80 :
