/*******************************************************************************
* File:	        VNHCTest.cpp
*		 
* Author:       Adan Moran-MacDonald
* Created:      04/Feb/20
*
* Systems Control Group
* Department of Electrical and Computer Engineering
* University of Toronto
*
* Last Modified: 04/Feb/20
* Last Editor:   Adan Moran-MacDonald
* Description:   Test suite for VNHC.h
*******************************************************************************/

// Includes for our code
#include "StateSpace.h"
#include "AcrobotDynamics.h"
#include "Phase.h"
#include "VNHC.h"
// Includes for std library
#include <iostream>
// Includes for gtest
#include "gtest/gtest.h"

using namespace SUGAR;

// Testing class for abstract code functionality: we want to implement a
// non-realistic VNHC to make sure everything works fine with the abstract
// class. For this, we use qa = 0 as our VNHC
class ZeroVNHC : public AcrobotVNHC
{
public:
    auto qa(UnactuatedPhase p)  const -> double 
    { return 0.0;}
    auto dqu(UnactuatedPhase p) const -> double
    { return 0.0;}
    auto dpu(UnactuatedPhase p) const -> double
    { return 0.0;}
};

class VNHCTest : public ::testing::Test
{
public:
    VNHCTest()
    : xingbot_(
        0.2112, 0.1979,   // m
        0.148, 0.145,     // d
        0.073, 0.083,     // l
        0.00075, 0.00129, // J
        9.81) // g
    {}
protected:
    void SetUp() override 
    {}

    void TearDown() override
    {}
private:

    // Dynamics for Xingbo's bot
    Acrobot xingbot_;

    // Define a Zero VNHC for this acrobot for testing abstract VNHC class

}; // class VNHCTest

TEST_F(VNHCTest, DQA_IS_ALWAYS_1)
{
    ASSERT_TRUE(false);
}

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

/* vim: set tw=80 ts=4 sw=4 sts=0 et ffs=unix : */
