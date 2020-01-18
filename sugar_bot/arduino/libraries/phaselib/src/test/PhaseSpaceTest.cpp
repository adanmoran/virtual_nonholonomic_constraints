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

#include "Phase.h"
#include "gtest/gtest.h"

using namespace SUGAR;

class PhaseTest : public ::testing::Test
{
public:
	PhaseTest()
	{}

protected:
	void SetUp() override {}
	void TearDown() override {}
};

TEST_F(PhaseTest, name)   //name of suite, name of test
{
}

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

/* vim:set noet sts=0 sw=4 ts=4 tw=80 : */
