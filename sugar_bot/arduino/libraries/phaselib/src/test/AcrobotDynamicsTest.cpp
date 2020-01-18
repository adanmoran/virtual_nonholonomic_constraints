/*******************************************************************************
* File:	        AcrobotDynamicsTest.cpp
*		 
* Author:       Adan Moran-MacDonald
* Created:      17/Jan/20
*
* Systems Control Group
* Department of Electrical and Computer Engineering
* University of Toronto
*
* Last Modified: 17/Jan/20
* Last Editor:   Adan Moran-MacDonald
* Description:   Test suite to ensure the Acrobot inverse inertia object
* behaves properly.
*******************************************************************************/

// Includes to test
#include "AcrobotDynamics.h"
// C++ libraries
#include <vector>
#include <iostream>
#define _USE_MATH_DEFINES
#include <math.h>
#undef _USE_MATH_DEFINES
// Test suite
#include "gtest/gtest.h"

using namespace SUGAR;

class AcrobotDynamicsTest : public ::testing::Test
{
public:
    AcrobotDynamicsTest()
    :   simple1_(1,1),
        simple2_(30,1),
        simple3_(0.2,0.4),
        simple4_(0.2,0.2),
        complex1_(1,1,1,1,1,1,0,0)
    {
        // Compute the step size for qa
        const int N = 9;
        for(int i = -N; i <= N; ++i)
        {
            qaVec_.push_back(i*M_PI/N);
        }
    }

protected:
    void SetUp() override
    {}

    void TearDown() override
    {}

    // Define simple acrobots
    AcrobotInverseInertia simple1_;
    AcrobotInverseInertia simple2_;
    AcrobotInverseInertia simple3_;
    AcrobotInverseInertia simple4_;
    // Define complex acrobots
    AcrobotInverseInertia complex1_;

    // Define a list of values of qa for which we'll test the matrices
    std::vector<double> qaVec_;
};

// Test to make sure that the outputs of simple acrobots match what MATLAB gives
// us for the value of Minv at each qa
TEST_F(AcrobotDynamicsTest, SIMPLE_IS_CORRECT)
{
    // Loop through and test the simple acrobot dynamics at each qa
    for(auto&& qa : qaVec_)
    {
        //TODO: expect that the acrobot dynamics are equal to the KNOWN matlab
        //outputs for the simple acrobot
    }
}

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

/* vim: set tw=80 ts=4 sw=4 sts=0 et ffs=unix : */
