/*******************************************************************************
* File:	        AcrobotInverseInertiaTest.cpp
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

#include "Phase.h"
#include "gtest/gtest.h"

class AcrobotInverseInertiaTest
{
public:
    AcrobotInverseInertiaTest()
    {

    }

private:
    AcrobotInverseInertia Minv_;
};

TEST(AcrobotInverseInertiaTest,TEST_NAME)
{
    
}

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

/* vim: set tw=80 ts=4 sw=4 sts=0 et ffs=unix : */
