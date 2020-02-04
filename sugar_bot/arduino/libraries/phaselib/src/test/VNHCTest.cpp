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

class VNHCTest : public ::testing::Test
{
public:
    VNHCTest(){}
protected:
    void SetUp() override 
    {}

    void TearDown() override
    {}
private:

}; // class VNHCTest

TEST_F(VNHCTest,TEST_NAME)
{
    
}

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

/* vim: set tw=80 ts=4 sw=4 sts=0 et ffs=unix : */
