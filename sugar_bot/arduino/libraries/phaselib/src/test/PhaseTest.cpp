/*******************************************************************************
* File:	        PhaseSpaceTest.cpp
*		 
* Author:       Adan Moran-MacDonald
* Created:      13/Jan/20
*
* Systems Control Group
* Department of Electrical and Computer Engineering
* University of Toronto
*
* Last Modified: 14/Jan/20
* Last Editor:   Adan Moran-MacDonald
* Description:   Test Suite for the phaselib library
*******************************************************************************/

// Our required headers
#include "Phase.h"
#include "StateSpace.h"
#include "AcrobotDynamics.h"
// C++ libraries
#include <vector>
#include <iostream>
#define _USE_MATH_DEFINES
#include <math.h>
#undef _USE_MATH_DEFINES
// Test libraries
#include "gtest/gtest.h"

using namespace SUGAR;

class PhaseTest : public ::testing::Test
{
public:
	PhaseTest()
	: simple1_(1,1),
	  simple2_(30,1),
	  xingboBot_(
        0.2112, 0.1979,   // m
        0.148, 0.145,     // d
        0.073, 0.083,     // l
        0.00075, 0.00129) // J
	{}

protected:
	void SetUp() override 
	{
		qpb2000_.psi = M_PI/2;
		q0pb200_.alpha = M_PI/2;
		q0010_.dpsi = 1;
		q0001_.dalpha = 1;
	}
	void TearDown() override {}

	// Some simple acrobots for basic tests
	AcrobotInertia simple1_;
	AcrobotInertia simple2_;
	// Set up the xingbo-bot for testing
	AcrobotInertia xingboBot_;

	// Set up a configuration for updating
	Configuration q0_; // all zero configuration
	Configuration qpb2000_; // qu = pi/2
	Configuration q0pb200_;  // qa = pi/2
	Configuration q0010_; // qud = 1 (bottom of swing)
	Configuration q0001_; // qad = 1 
};

TEST_F(PhaseTest, UPDATE_USES_M)   //name of suite, name of test
{
}

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

/* vim:set noet sts=0 sw=4 ts=4 tw=80 : */
