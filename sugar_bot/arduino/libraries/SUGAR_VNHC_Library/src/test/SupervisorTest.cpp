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
// Includes for M_PI, cosine, etc
#define _USE_MATH_DEFINES
#include <math.h>
#undef _USE_MATH_DEFINES
// Includes for std library
#include <iostream>
// Includes for gtest
#include "gtest/gtest.h"

using namespace SUGAR;

////////////////
// Supervisor //
////////////////
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
      sup_(in_,diss_)
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

    // Define an unactuatedPhase for testing the objects
    UnactuatedPhase qpu_;

    // Constants for our VNHCs
    double qmax_ = 1;
    // Define our injection and dissipation VNHCs
    ArctanVNHC in_;
    ArctanVNHC diss_;

    // Define our supervisors, which take in the two VNHCs
    Supervisor sup_;
}; // class SupervisorTest

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

/*****************************************************************************/


////////////////////////////
// Oscillation Supervisor //
////////////////////////////
class OscillationSupervisorTest : public ::testing::Test
{
public:
    OscillationSupervisorTest()
    : bot_(1,1,1,1,1,1,1,0,0),
      in_(bot_,qmax_,I_),
      diss_(bot_,qmax_,-I_),
      sup_(in_,diss_),
      osup_(sup_)
    {}
protected:
    void SetUp() override 
    {

    }

    void TearDown() override
    {}

    // The acrobot
    Acrobot bot_;
    // Constants for our VNHCs
    double qmax_ = 1;
    double I_ = 1;

    // Define our injection and dissipation VNHCs
    ArctanVNHC in_;
    ArctanVNHC diss_;

    // Define our supervisors, which take in the two VNHCs
    Supervisor sup_;
    OscillationSupervisor osup_;

    // Define an unactuatedPhase for testing the objects
    UnactuatedPhase qpu_;

    // The desired angle and hysteresis
    double qudes_ = M_PI/4;
    double hys_ = 0.1;
}; // class OscillationSupervisorTest

 
/**
 * The oscillation supervisor should return 0 if qu is within range of qudes.
 */
TEST_F(OscillationSupervisorTest, OSCILLATION_ZEROS_OUT)
{
    qpu_.qu = qudes_ - hys_;
    EXPECT_EQ(osup_.stabilize(qpu_,qudes_,hys_), 0);

    qpu_.qu = qudes_;
    EXPECT_EQ(osup_.stabilize(qpu_,qudes_,hys_), 0);

    qpu_.qu = qudes_ + hys_;
    EXPECT_EQ(osup_.stabilize(qpu_,qudes_,hys_), 0);

    // Changing qu's sign shouldn't matter.
    qpu_.qu = -qudes_ + hys_;
    EXPECT_EQ(osup_.stabilize(qpu_,qudes_,hys_), 0);

    qpu_.qu = -qudes_;
    EXPECT_EQ(osup_.stabilize(qpu_,qudes_,hys_), 0);

    qpu_.qu = -qudes_ - hys_;
    EXPECT_EQ(osup_.stabilize(qpu_,qudes_,hys_), 0);

    // Changing the sign of qudes_ shouldn't matter.
     qpu_.qu = qudes_ - hys_;
    EXPECT_EQ(osup_.stabilize(qpu_,-qudes_,hys_), 0);

    qpu_.qu = qudes_;
    EXPECT_EQ(osup_.stabilize(qpu_,-qudes_,hys_), 0);

    qpu_.qu = qudes_ + hys_;
    EXPECT_EQ(osup_.stabilize(qpu_,-qudes_,hys_), 0);

    // Changing pu here shouldn't matter.
    qpu_.pu = 1;
    qpu_.qu = qudes_;
    EXPECT_EQ(osup_.stabilize(qpu_,qudes_,hys_), 0);

    qpu_.pu = -1;
    qpu_.qu = qudes_;
    EXPECT_EQ(osup_.stabilize(qpu_,qudes_,hys_), 0);
}

/**
 * Test the injection mechanism, which should occur whenever qu < qudes_.
 */
TEST_F(OscillationSupervisorTest, OSCILLATION_INJECTION)
{

    // Initialize the acrobot at (qu,pu)=(qudes_/2,0). This should inject energy.
    qpu_.qu = qudes_/2;
    EXPECT_EQ(osup_.stabilize(qpu_,qudes_,hys_),in_.qa(qpu_));
    // Do the same for (qu,pu) = (-qudes_/2,0)
     qpu_.qu = -qudes_/2;
    EXPECT_EQ(osup_.stabilize(qpu_,qudes_,hys_),in_.qa(qpu_));
    // Do the same for (qu,pu) = (0,0)
     qpu_.qu = 0;
    EXPECT_EQ(osup_.stabilize(qpu_,qudes_,hys_),in_.qa(qpu_));
    // Do the same for (qu,pu) < (qudes_-hys_,0)
     qpu_.qu = qudes_ - hys_ - 0.001;
    EXPECT_EQ(osup_.stabilize(qpu_,qudes_,hys_),in_.qa(qpu_));
}

TEST_F(OscillationSupervisorTest, OSCILLATION_DISSIPATION)
{
    // Initialize the acrobot at (qu,pu)=(qudes_*2,0). This should dissipate energy.
    qpu_.qu = qudes_*2;
    EXPECT_EQ(osup_.stabilize(qpu_,qudes_,hys_),in_.qa(qpu_));
    // Do the same for (qu,pu) = (-qudes_*2,0)
     qpu_.qu = -qudes_*2;
    EXPECT_EQ(osup_.stabilize(qpu_,qudes_,hys_),in_.qa(qpu_));
    // Do the same for (qu,pu) = (pi,0)
     qpu_.qu = M_PI;
    EXPECT_EQ(osup_.stabilize(qpu_,qudes_,hys_),in_.qa(qpu_));
    // Do the same for (qu,pu) > (qudes_+hys_,0)
     qpu_.qu = qudes_ + hys_ + 0.001;
    EXPECT_EQ(osup_.stabilize(qpu_,qudes_,hys_),in_.qa(qpu_));

}

/**
 * When pu changes sign, we should toggle from injection to dissipation.
 */
TEST_F(OscillationSupervisorTest, TOGGLE_IN_DISS)
{
    // We start with injection at (qu,pu) = (0,0)
    EXPECT_EQ(osup_.stabilize(qpu_,qudes_,hys_),in_.qa(qpu_));

    // Now if the next (qu,pu) has positive pu, we should remain injecting when
    // qu < qudes_.
    qpu_.qu = qudes_-2*hys_;
    qpu_.pu = 1;
    EXPECT_EQ(osup_.stabilize(qpu_,qudes_,hys_),in_.qa(qpu_));

    // Pu changes sign, but we but we haven't hit the hys_teresis range.
    // Should still be injecting.
    qpu_.qu = qudes_-1.5*hys_;
    qpu_.pu = -0.5;
    EXPECT_EQ(osup_.stabilize(qpu_,qudes_,hys_),in_.qa(qpu_));

    // Swing to the other side, change sign, but not in range. Should be
    // injecting.
    qpu_.qu = -qudes_+1.5*hys_;
    qpu_.pu = 0.5;
    EXPECT_EQ(osup_.stabilize(qpu_,qudes_,hys_),in_.qa(qpu_));

    // Now we go to the front, past qudes_. Should be injecting because pu hasn't changed sign.
     qpu_.qu = qudes_+2*hys_;
    qpu_.pu = 0.5;
    EXPECT_EQ(osup_.stabilize(qpu_,qudes_,hys_),in_.qa(qpu_));

    // Same spot but pu changes sign. Should now be dissipating.
    qpu_.qu = qudes_+2*hys_;
    qpu_.pu = -0.5;
    EXPECT_EQ(osup_.stabilize(qpu_,qudes_,hys_),diss_.qa(qpu_));

    // We swing down to the bottom, should still be dissipating.
    qpu_.qu = 0;
    qpu_.pu = -10;
    EXPECT_EQ(osup_.stabilize(qpu_,qudes_,hys_),diss_.qa(qpu_));

    // We swing up to the other side, should still be dissipating even though we
    // switched signs of pu.
    qpu_.qu = -qudes_-2*hys_;
    qpu_.pu = 1;
    EXPECT_EQ(osup_.stabilize(qpu_,qudes_,hys_),diss_.qa(qpu_));

    // Pendulum oscillates back to the front below qudes_. 
    // still dissipating because pu hasn't changed sign.
    qpu_.qu = qudes_-2*hys_;
    qpu_.pu = 1;
    EXPECT_EQ(osup_.stabilize(qpu_,qudes_,hys_),diss_.qa(qpu_));

    // Now pu changes sign and we're not in qudes_ range. Should be injecting.
    qpu_.qu = qudes_-2*hys_;
    qpu_.pu = -0.1;
    EXPECT_EQ(osup_.stabilize(qpu_,qudes_,hys_),in_.qa(qpu_));
}

/**
 * If we hit a peak in the range qu in [qudes-hys,qudes+hys] then we should
 * maintain qa = 0 throughout the rest of a cycle.
 */
TEST_F(OscillationSupervisorTest, OSCILLATION_STAYS_STABLE)
{
    // Start in range, qa = 0
    qpu_.qu = qudes_;
    qpu_.pu = -0.0001;
    EXPECT_EQ(osup_.stabilize(qpu_,qudes_,hys_),0);

    // Bottom of the swing, qa = 0
    qpu_.qu = 0; 
    qpu_.pu = -10;
    EXPECT_EQ(osup_.stabilize(qpu_,qudes_,hys_),0);

    // Swing to the back, go too low, switch to injection.
    qpu_.qu = -qudes_+2*hys_; 
    qpu_.pu = 1;
    EXPECT_EQ(osup_.stabilize(qpu_,qudes_,hys_),in_.qa(qpu_));

    // Bottom of the swing, should still be injecting.
    qpu_.qu = 0;
    qpu_.qu = 10;
    EXPECT_EQ(osup_.stabilize(qpu_,qudes_,hys_),in_.qa(qpu_));

    // Swing to the front and go within range, qa = 0.
    qpu_.qu = qudes_+hys_;
    qpu_.pu = -1;
    EXPECT_EQ(osup_.stabilize(qpu_,qudes_,hys_),0);

    // Swing to the bottom, still qa = 0.
    qpu_.qu = 0; 
    qpu_.pu = -10;
    EXPECT_EQ(osup_.stabilize(qpu_,qudes_,hys_),0);

    // Swing to the back and go too high somehow, switch to dissipation.
    qpu_.qu = -qudes_-2*hys_; 
    qpu_.pu = 0;
    EXPECT_EQ(osup_.stabilize(qpu_,qudes_,hys_),diss_.qa(qpu_));

    // We move slightly and now have positive velocity.
    // Still dissipating.
    qpu_.qu = -qudes_-1.5*hys_; 
    qpu_.pu = 0.1;
    EXPECT_EQ(osup_.stabilize(qpu_,qudes_,hys_),diss_.qa(qpu_));

    // We are now in range, qa should be zero.
    qpu_.qu = -qudes_-hys_; 
    qpu_.pu = 1;
    EXPECT_EQ(osup_.stabilize(qpu_,qudes_,hys_),0);

    // Bottom of the swing, qa should remain zero.
    qpu_.qu = 0;
    qpu_.pu = 5;
    EXPECT_EQ(osup_.stabilize(qpu_,qudes_,hys_),0);

    // Top of the swing, in range, qa = 0
    qpu_.qu = qudes_-hys_;
    qpu_.pu = -1;
    EXPECT_EQ(osup_.stabilize(qpu_,qudes_,hys_),0);

    // Bottom of the swing, qa = 0.
    qpu_.qu = 0; 
    qpu_.pu = -10;
    EXPECT_EQ(osup_.stabilize(qpu_,qudes_,hys_),0);
}


/*****************************************************************************/
/////////////////////////
// Rotation Supervisor //
/////////////////////////



/*****************************************************************************/
int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

/* vim: set tw=80 ts=4 sw=4 sts=0 et ffs=unix : */
