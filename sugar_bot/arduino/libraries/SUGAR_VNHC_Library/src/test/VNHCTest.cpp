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

// Testing class for abstract code functionality: we want to implement a
// non-realistic VNHC to make sure everything works fine with the abstract
// class. For this, we use qa = 0 as our VNHC
class ZeroVNHC : public AcrobotVNHC
{
public:
    ZeroVNHC(const Acrobot& acrobot)
        : AcrobotVNHC(acrobot)
    {}
    auto qa(const UnactuatedPhase& p)  const -> double 
    { return 0.0;}
    auto dqu(const UnactuatedPhase& p) const -> double
    { return 0.0;}
    auto dpu(const UnactuatedPhase& p) const -> double
    { return 0.0;}
};

class VNHCTest : public ::testing::Test
{
public:
    VNHCTest()
    : xingbot_(
        mt_, ml_,
        dt_, dl_,
        lt_, ll_,
        Jt_, Jl_,
        g_),
      zero_(xingbot_),
      tanhVNHC_(xingbot_, qmax_),
      sinuVNHC_(xingbot_, qmax_)
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
    
    // error margin for predicates
    double eps_ = 10e-15;

    // Dynamics for Xingbo's bot
    Acrobot xingbot_;

    // Define a Zero VNHC for this acrobot for testing abstract VNHC class
    ZeroVNHC zero_;

    // Define a pointer to the VNHC
    AcrobotVNHC* pVNHC_;

    // Define an unactuatedPhase for testing the objects
    UnactuatedPhase qpu_;

    // Constants for our VNHCs
    double qmax_ = 1;
    // Define our concrete VNHCs
    TanhVNHC tanhVNHC_;
    SinuVNHC sinuVNHC_;
}; // class VNHCTest

// Test that dh/dqa is always 1 and dh/dpa = 0 for all inputs
TEST_F(VNHCTest, DQA_DPA_CONSTANT)
{
    // Test the constant functions for the abstract VNHC, which we use zero_ to
    // define concretely
    pVNHC_ = &zero_;

    // Now iterate over a bunch of qu and pu values between -5 and 5
    for(int i = -10; i <= 10; ++i)
    {
        qpu_.qu = i/2.0;

        for(int j = -10; j <= 10; ++j)
        {
            qpu_.pu = j/2.0;

            ASSERT_DOUBLE_EQ(pVNHC_->dqa(qpu_), 1.0);
            ASSERT_DOUBLE_EQ(pVNHC_->dpa(qpu_), 0.0);
        }
    }

}

// Test that pa as computed is always correct, for each VNHC at specified test
// values of qa = f(qu,pu).
TEST_F(VNHCTest, PA_MATCHES_SOLVABILITY_EQUATIONS)
{

    // For the zero constraint, the equation for pa under qa = 0 is given by
    // pa = pu * (ml*ll^2 + dt*ml*ll + Jl)/(ml*dt^2 + 2*ml*dt*ll
    // + ml*ll^2 + mt*lt^2 + Jl + Jt)
    double mlll2 = ml_*ll_*ll_;
    double mtlt2 = mt_*lt_*lt_;
    double mldt2 = ml_*dt_*dt_;
    double dtmlll = dt_*ml_*ll_;
    double zeroPaConstant = (mlll2 + dtmlll + Jl_)/(mldt2 + 2*dtmlll + mlll2 + mtlt2 + Jl_ + Jt_);

    // Iterate over our test cases which are
    // qu = i*pi/10 for k in -10...10
    // pu = -10 -8 ... 8 100
    for (int i = -10; i <= 10; ++i)
    {
        qpu_.qu = i*M_PI/10.0;
        qpu_.qu = i/2.0;
        for(int j = -5; j <= 5; ++j)
        {
            qpu_.pu = j*10/5.0;

            // Test the zero vnhc, whch is pa = pu*zeroPaConstant
            // (ie it does not depend on qa)
            pVNHC_ = &zero_;
            ASSERT_DOUBLE_EQ(pVNHC_->pa(qpu_),
                qpu_.pu * zeroPaConstant
            );
        }
    }
}

// Test to make sure the pa of concrete VNHCs is valid
TEST_F(VNHCTest, CONCRETE_PA_MATCHES_SOLVABILITY_EQUATIONS)
{
    // Test the tanh VNHC
    pVNHC_ = &tanhVNHC_;

    //1. At pu = 0, we get pa = -4.617524208065135e-04*sin(qu)
    qpu_.pu = 0;
    for(int i = -10; i <= 10; ++i)
    {
        auto qu = i*M_PI/10;
        qpu_.qu = qu; 
        auto expected_pa = -4.617524208065135e-04*sin(qu);
        ASSERT_PRED3(Within, pVNHC_->pa(qpu_), expected_pa, eps_);
    }

    // Now let's test some other random values. For instance, qu = 0 and pu =
    // 1 has solution pa = 0.356320517129854
    qpu_.qu = 0; qpu_.pu = 1;
    auto pa_01 = 0.356320517129854;
    ASSERT_PRED3(Within, pVNHC_->pa(qpu_), pa_01, eps_);

    // qu = pi/2 and pu = -10 has solution -3.452229826810668
    qpu_.qu = M_PI/2; qpu_.pu = -10;
    auto pa_pb2_n10 = -3.45222982681066;
    ASSERT_PRED3(Within, pVNHC_->pa(qpu_), pa_pb2_n10, eps_);

    // Test the sin(theta) VNHC, which has very complicated pa
    // Instead of iterating, we simply note down the results for some common
    // cases.
    pVNHC_ = &sinuVNHC_;

    // 1. We cannot enforce qu = pu = 0 because of irregularity, so choose any
    // qu > 0. This gives qa = sin(0) = 0. At qu = pi/2, we get
    // pa = -264076723158814266129/(285950556250000000000000*pi)
    qpu_.qu = M_PI/2; qpu_.pu = 0;
    double pa_at_qu_is_pb2 = -264076723158814266129.0/(285950556250000000000000.0*M_PI);
    EXPECT_PRED3(Within, pVNHC_->pa(qpu_), pa_at_qu_is_pb2, eps_);

    // 2. For pu > 0 and qu = 0, we get qa = sin(pi/2) = 1. At pu = 1, we get 
    // pa = pi*(24310036*cos(qa) + 26533331)/(97240144*cos(qa) + 177272390)
    qpu_.qu = 0; qpu_.pu = 1;
    double pa_at_pu_is_1 = (24310036*cos(1) + 26533331)/(48620072*cos(1) + 88636195);
    EXPECT_PRED3(Within, pVNHC_->pa(qpu_), pa_at_pu_is_1, eps_);
    // 3. For qu = -1, pu = 0 we get -pa from (1). For qu = 0, pu = -1 we get -pa from (2)
    qpu_.qu = -M_PI/2; qpu_.pu = 0;
    auto pa_nqu = -pa_at_qu_is_pb2;
    EXPECT_PRED3(Within, pVNHC_->pa(qpu_), pa_nqu, eps_);

    qpu_.qu = 0; qpu_.pu = -1;
    auto pa_npu = -pa_at_pu_is_1;
    EXPECT_PRED3(Within, pVNHC_->pa(qpu_), pa_npu, eps_);

    // If we do any more, we get really gross answers, so I'll just estimate
    // them to within a certain tolerance. 
    // For instance, we should get that for qu = sqrt(3), pu = 1, we have qa =
    // sin(pi/6) = 1/2 which yields pa ~ 0.348085485082814. Let's set our check
    // to be only within this number of decimals, 10e-14.
    qpu_.qu = sqrt(3); qpu_.pu = 1;
    auto pa_pb6 = 0.348085485082814; 
    EXPECT_PRED3(Within, pVNHC_->pa(qpu_), pa_pb6, eps_);
}

// Test for the TanhVNHC, to make sure its outputs are valid
TEST_F(VNHCTest, TANH_OUTPUTS_VALID)
{
    for (int i = -10; i <= 10; ++i)
    {
        qpu_.qu = i*M_PI/10.0;
        qpu_.qu = i/2.0;
        for(int j = -5; j <= 5; ++j)
        {
            qpu_.pu = j*10/5.0;

            // qa is only a function of pu
            ASSERT_DOUBLE_EQ(tanhVNHC_.qa(qpu_), tanh(qpu_.pu));

            // dqu is 0
            ASSERT_DOUBLE_EQ(tanhVNHC_.dqu(qpu_), 0);

            // dpu is the derivative, which is dependent only on pu
            auto tanhpu = tanh(qpu_.pu);
            auto tanhpu2 = tanhpu*tanhpu;
            ASSERT_DOUBLE_EQ(tanhVNHC_.dpu(qpu_), tanhpu2-1);
        }
    }
}

// Test for the SinuVNHC, to make sure its outputs are valid
TEST_F(VNHCTest, SINU_OUTPUTS_VALID)
{
    for (int i = -10; i <= 10; ++i)
    {
        qpu_.qu = i*M_PI/10.0;
        qpu_.qu = i/2.0;
        for(int j = -5; j <= 5; ++j)
        {
            qpu_.pu = j*10/5.0;

            // This is not valid at qu = pu = 0
            if(qpu_.qu != 0 && qpu_.pu != 0)
            {
                auto theta = atan2(qpu_.pu, qpu_.qu);

                // qa is only a function of pu
                ASSERT_PRED3(Within, sinuVNHC_.qa(qpu_), qmax_*sin(theta), eps_);

                // dqu is -df = -cos(theta)*theta' = pu*cos(theta)/(qu^2 + pu^2)
                auto ct = cos(theta);
                auto norm_qpu = qpu_.qu*qpu_.qu + qpu_.pu*qpu_.pu;
                auto dthetaq = qpu_.pu / norm_qpu;
                ASSERT_PRED3(Within, sinuVNHC_.dqu(qpu_), qmax_*ct*dthetaq, eps_);

                // dpu is -df = -c(theta)*theta' = -qu*cos(theta)/(qu^2 + pu^2)
                auto dthetap = -qpu_.qu/norm_qpu;
                ASSERT_PRED3(Within, sinuVNHC_.dpu(qpu_), qmax_*ct*dthetap, eps_);
            }
        }
    }
}
// Test for TanhVNHC to test different q_max values
TEST_F(VNHCTest, Tanh_QMAX)
{
    // Positive values should work fine
    TanhVNHC tanh_0(xingbot_, 0);
    TanhVNHC tanh_1(xingbot_, 1);
    TanhVNHC tanh_pi(xingbot_, M_PI);
    // Negative values are allowed to dissipate energy 
    TanhVNHC tanh_npi(xingbot_, -M_PI);
    // Values outside of [-M_PI, M_PI] should be capped to within that range.
    TanhVNHC tanh_above_pi(xingbot_,  10);
    TanhVNHC tanh_below_npi(xingbot_, -5);

    for (int i = -10; i <= 10; ++i)
    {
        qpu_.qu = i*M_PI/10.0;
        qpu_.qu = i/2.0;
        for(int j = -5; j <= 5; ++j)
        {
            qpu_.pu = j*10/5.0;

            // This is not valid at qu = pu = 0
            if(qpu_.qu != 0 && qpu_.pu != 0)
            {
                // Test the positive (energy_gain) vnhcs
                EXPECT_PRED3(Within, tanh_0.qa(qpu_), 0, eps_);
                EXPECT_PRED3(Within, tanh_1.qa(qpu_), tanh(qpu_.pu), eps_);
                EXPECT_PRED3(Within, tanh_pi.qa(qpu_), M_PI*tanh(qpu_.pu), eps_);
                // Test the energy dissipation VNHC
                EXPECT_PRED3(Within, tanh_npi.qa(qpu_), -M_PI*tanh(qpu_.pu), eps_);
                // Test the outside of range vnhc, which should be capped
                EXPECT_PRED3(Within, tanh_above_pi.qa(qpu_), M_PI*tanh(qpu_.pu), eps_);
                ASSERT_PRED3(Within, tanh_below_npi.qa(qpu_), -M_PI*tanh(qpu_.pu), eps_);
            }
        }
    }
}
// Test for SinuVNHC to test different q_max values
TEST_F(VNHCTest, SINU_QMAX)
{
    // Positive values should work fine
    SinuVNHC sinu_0(xingbot_, 0);
    SinuVNHC sinu_1(xingbot_, 1);
    SinuVNHC sinu_pi(xingbot_, M_PI);
    // Negative values are allowed to dissipate energy 
    SinuVNHC sinu_npi(xingbot_, -M_PI);
    // Values outside of [-M_PI, M_PI] should be capped to within that range.
    SinuVNHC sinu_above_pi(xingbot_,  10);
    SinuVNHC sinu_below_npi(xingbot_, -5);

    for (int i = -10; i <= 10; ++i)
    {
        qpu_.qu = i*M_PI/10.0;
        qpu_.qu = i/2.0;
        for(int j = -5; j <= 5; ++j)
        {
            qpu_.pu = j*10/5.0;

            // This is not valid at qu = pu = 0
            if(qpu_.qu != 0 && qpu_.pu != 0)
            {
                auto theta = atan2(qpu_.pu, qpu_.qu);

                // Test the positive (energy_gain) vnhcs
                EXPECT_PRED3(Within, sinu_0.qa(qpu_), 0, eps_);
                EXPECT_PRED3(Within, sinu_1.qa(qpu_), sin(theta), eps_);
                EXPECT_PRED3(Within, sinu_pi.qa(qpu_), M_PI*sin(theta), eps_);
                // Test the energy dissipation VNHC
                EXPECT_PRED3(Within, sinu_npi.qa(qpu_), -M_PI*sin(theta), eps_);
                // Test the outside of range vnhc, which should be capped
                EXPECT_PRED3(Within, sinu_above_pi.qa(qpu_), M_PI*sin(theta), eps_);
                ASSERT_PRED3(Within, sinu_below_npi.qa(qpu_), -M_PI*sin(theta), eps_);
            }
        }
    }
}

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

/* vim: set tw=80 ts=4 sw=4 sts=0 et ffs=unix : */
