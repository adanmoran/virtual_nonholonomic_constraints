/*******************************************************************************
* File:	        SupervisorTest.cpp
*		 
* Author:       Adan Moran-MacDonald
* Created:      26/Apr/21
*
* Systems Control Group
* Department of Electrical and Computer Engineering
* University of Toronto
*
* Last Modified: 26/Apr/21
* Last Editor:   Adan Moran-MacDonald
* Description:   Test suite for Supervisor.h
*******************************************************************************/

// Includes for our code
#include "StateSpace.h"
#include "AcrobotDynamics.h"
#include "Phase.h"
#include "VNHC.h"
#include "Supervisor.h"
// Include predicates
#include "test/Predicates.h"
// Includes for M_PI, cosine, etc
#define _USE_MATH_DEFINES
#include <math.h>
#undef _USE_MATH_DEFINES
// Includes for std library
#include <iostream>
// Includes for gtest
#include "gtest/gtest.h"

using namespace SUGAR;
using SUGAR::predicates::Within;

class SupervisorTest : public ::testing::Test
{
public:
    SupervisorTest()
    : xingbot_(
        mt_, ml_,
        dt_, dl_,
        lt_, ll_,
        Jt_, Jl_,
        g_),
      in_(xingbot_,qmax_,I_),
      diss_(xingbot_,qmax_,-I_),
      sup_(in_,diss_) // Todo: implement this correctly
    {}
protected:
    void SetUp() override 
    {

    }

    void TearDown() override
    {}
protected:
    // constants for Xingbo's acrobot
    double mt_ = 0.2112;  double ml_ = 0.1979;
    double dt_ = 0.148;   double dl_ = 0.145;
    double lt_ = 0.073;   double ll_ = 0.083;
    double Jt_ = 0.00075; double Jl_ = 0.00129;
    double g_ = 9.81;

    // Constant control input for the VNHC
    double I_ = 10;
    
    // error margin for predicates
    double eps_ = 10e-15;

    // Dynamics for Xingbo's bot
    Acrobot xingbot_;

    // Define a pointer to the VNHC
    AcrobotVNHC* pVNHC_;

    // Define an unactuatedPhase for testing the objects
    UnactuatedPhase qpu_;

    // Constants for our VNHCs
    double qmax_ = 1;
    // Define our injection and dissipation VNHCs
    ArctanVNHC in_;
    ArctanVNHC diss_;

    // Define our supervisor, which takes in the two VNHCs
    Supervisor sup_;
    // A default unactuated phase for testing
    UnactuatedPhase qpu;
}; // class VNHCTest

// Make sure if abs(value - des) <= hys, that the supervisor returns qa = 0
TEST_F(SupervisorTest, ZERO_WITHIN_HYS)
{
    double des = M_PI/2;
    double hys = 0.2;
    // check that if value is in [des-hys, des+hys] that we get qa = 0
    EXPECT_EQ(sup_.stabilize(qpu_,des-hys,des,hys),0);
    EXPECT_EQ(sup_.stabilize(qpu_,des,des,hys),0);
    EXPECT_EQ(sup_.stabilize(qpu_,des+hys,des,hys),0);

    // Check that if value is in [-des-hys,-des+hys] that we get qa = 0
    EXPECT_EQ(sup_.stabilize(qpu_,-des-hys,des,hys),0);
    EXPECT_EQ(sup_.stabilize(qpu_,-des,des,hys),0);
    EXPECT_EQ(sup_.stabilize(qpu_,-des+hys,des,hys),0);

    // If the desried value is negative, it should be the same as it being
    // positive.
    EXPECT_EQ(sup_.stabilize(qpu_,des-hys,-des,hys),0);
    EXPECT_EQ(sup_.stabilize(qpu_,des,-des,hys),0);
    EXPECT_EQ(sup_.stabilize(qpu_,des+hys,-des,hys),0);

    // If the hysteresis is negative, it should be the same as it being
    // positive.
    EXPECT_EQ(sup_.stabilize(qpu_,des-hys,des,-hys),0);
    EXPECT_EQ(sup_.stabilize(qpu_,des,des,-hys),0);
    EXPECT_EQ(sup_.stabilize(qpu_,des+hys,-des,-hys),0);

    // Changing qpu should do nothing
    qpu_.qu = M_PI/2;
    qpu_.pu = 100;
    EXPECT_EQ(sup_.stabilize(qpu_,des-hys,des,hys),0);
    EXPECT_EQ(sup_.stabilize(qpu_,des,des,hys),0);
    EXPECT_EQ(sup_.stabilize(qpu_,des+hys,des,hys),0);
}

// If the current value is above the desired value, the supervisor should return
// the injection VNHC
TEST_F(SupervisorTest, INJECTION)
{
    double des = M_PI/2;

    // If our actual value is below the desired value, then we should return the
    // injection VNHC
    EXPECT_EQ(sup_.stabilize(qpu_, des/2,des,0),in_.qa(qpu_));
    EXPECT_EQ(sup_.stabilize(qpu_, des-0.1,des,0),in_.qa(qpu_));
    EXPECT_EQ(sup_.stabilize(qpu_, 0,des,0),in_.qa(qpu_));

    // Changing qpu should do nothing
    qpu_.qu = M_PI/4;
    qpu_.pu = 100;
    EXPECT_EQ(sup_.stabilize(qpu_,des/2,des,0),in_.qa(qpu_));
    EXPECT_EQ(sup_.stabilize(qpu_,des-0.1,des,0),in_.qa(qpu_));
    EXPECT_EQ(sup_.stabilize(qpu_,0,des,0),in_.qa(qpu_));

    // Changing the sign of value should not change the result
    EXPECT_EQ(sup_.stabilize(qpu_,-des/2,des,0),in_.qa(qpu_));
    EXPECT_EQ(sup_.stabilize(qpu_,-des+0.1,des,0),in_.qa(qpu_));
}

// If the current value is below the desired value, the supervisor should return
// the dissipation VNHC
TEST_F(SupervisorTest, DISSIPATION)
{
    double des = M_PI/2;

    // If our actual value is below the desired value, then we should return the
    // injection VNHC
    EXPECT_EQ(sup_.stabilize(qpu_, des*2,des,0),diss_.qa(qpu_));
    EXPECT_EQ(sup_.stabilize(qpu_, des+0.1,des,0),diss_.qa(qpu_));

    // Changing qpu should not change the result
    qpu_.qu = M_PI/4;
    qpu_.pu = 100;
    EXPECT_EQ(sup_.stabilize(qpu_,des*2,des,0),diss_.qa(qpu_));
    EXPECT_EQ(sup_.stabilize(qpu_,des+0.1,des,0),diss_.qa(qpu_));

    // Changing the sign of value should not change the result
    EXPECT_EQ(sup_.stabilize(qpu_, -des*2,des,0),diss_.qa(qpu_));
    EXPECT_EQ(sup_.stabilize(qpu_, -des-0.1,des,0),diss_.qa(qpu_));
}

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

/* vim: set tw=80 ts=4 sw=4 sts=0 et ffs=unix : */
