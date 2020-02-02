/*******************************************************************************
* File:	        AcrobotInertiaTest.cpp
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
#include "StateSpace.h"
// C++ libraries
#include <vector>
#include <iostream>
#define _USE_MATH_DEFINES
#include <math.h>
#undef _USE_MATH_DEFINES
// Test suite
#include "gtest/gtest.h"

using namespace SUGAR;

///////////////////////
// Custom Predicates //
///////////////////////

/**
* @brief: Predicate to decide if a and b are within tolerance of each other
*
* @param: double a
*       : double b
*       : double tolerance
*
* @return: bool
*/
bool Within(double a, double b, double tolerance)
{
    return std::abs(a-b) <= tolerance;
}

///////////////////
// Matrix2 Tests //
///////////////////

// Test a variety of 2x2 matrices
TEST(Matrix2Tests, MATRIX2_GETTERS_VALID)
{
    // Generate a bunch of matrix2 objects
    Matrix2 zero(0,0,0,0); // 0-matrix
    Matrix2 eye(1,0,0,1); // eye(2)
    Matrix2 ottf(1,2,3,4); // [1 2; 3 4]
    Matrix2 R90(0,1,1,0);  // 90-deg rotation matrix
    Matrix2 negatives(-99.99,-30.30,-15.51,-0.005);
    Matrix2 mixed(-1000,2,-0.99,42.42);

    // Iterate through the matrices and assert each element is what it should be
    // Zero matrix
    EXPECT_EQ(zero.at(1,1),0);
    EXPECT_EQ(zero.at(1,2),0);
    EXPECT_EQ(zero.at(2,1),0);
    EXPECT_EQ(zero.at(2,2),0);
    // eye matrix
    EXPECT_EQ(eye.at(1,1),1);
    EXPECT_EQ(eye.at(1,2),0);
    EXPECT_EQ(eye.at(2,1),0);
    EXPECT_EQ(eye.at(2,2),1);
    // ottf matrix
    EXPECT_EQ(ottf.at(1,1),1);
    EXPECT_EQ(ottf.at(1,2),2);
    EXPECT_EQ(ottf.at(2,1),3);
    EXPECT_EQ(ottf.at(2,2),4);
    // R90 matrix
    EXPECT_EQ(R90.at(1,1),0);
    EXPECT_EQ(R90.at(1,2),1);
    EXPECT_EQ(R90.at(2,1),1);
    EXPECT_EQ(R90.at(2,2),0);
    // negatives matrix
    EXPECT_EQ(negatives.at(1,1),-99.99);
    EXPECT_EQ(negatives.at(1,2),-30.30);
    EXPECT_EQ(negatives.at(2,1),-15.51);
    EXPECT_EQ(negatives.at(2,2),-0.005);
    // mixed matrix
    EXPECT_EQ(mixed.at(1,1),-1000);
    EXPECT_EQ(mixed.at(1,2),2);
    EXPECT_EQ(mixed.at(2,1),-0.99);
    EXPECT_EQ(mixed.at(2,2),42.42);
}

TEST(Matrix2Tests, MATRIX_GETTERS_INVALID_INPUT)
{
    // If you give an out of range input, the .at() function should return NaN
    Matrix2 m(0,0,0,0);
    
    // Both row and column are 0 should be nan
    EXPECT_TRUE(std::isnan(m.at(0,0)));

    // Test rows out of range (0, 3, 4, ...)
    EXPECT_TRUE(std::isnan(m.at(0,1)));
    EXPECT_TRUE(std::isnan(m.at(0,2)));
    EXPECT_TRUE(std::isnan(m.at(3,1)));
    EXPECT_TRUE(std::isnan(m.at(3,2)));
    EXPECT_TRUE(std::isnan(m.at(4,1)));
    EXPECT_TRUE(std::isnan(m.at(4,2)));
    EXPECT_TRUE(std::isnan(m.at(10,1)));
    EXPECT_TRUE(std::isnan(m.at(100,1)));
    EXPECT_TRUE(std::isnan(m.at(1000,2)));

    // Test columns out of range
    EXPECT_TRUE(std::isnan(m.at(1,0)));
    EXPECT_TRUE(std::isnan(m.at(2,0)));
    EXPECT_TRUE(std::isnan(m.at(1,3)));
    EXPECT_TRUE(std::isnan(m.at(2,3)));
    EXPECT_TRUE(std::isnan(m.at(1,4)));
    EXPECT_TRUE(std::isnan(m.at(2,4)));
    EXPECT_TRUE(std::isnan(m.at(1,10)));
    EXPECT_TRUE(std::isnan(m.at(1,100)));
    EXPECT_TRUE(std::isnan(m.at(2,1000)));

    // Test both out of range
    EXPECT_TRUE(std::isnan(m.at(3,4)));
    EXPECT_TRUE(std::isnan(m.at(4,3)));
    EXPECT_TRUE(std::isnan(m.at(10,5)));
    EXPECT_TRUE(std::isnan(m.at(5,7)));
    EXPECT_TRUE(std::isnan(m.at(3,3)));

    // Now just make sure that if they are in range, we don't get Nan
    EXPECT_FALSE(std::isnan(m.at(1,1)));
    EXPECT_FALSE(std::isnan(m.at(1,2)));
    EXPECT_FALSE(std::isnan(m.at(2,1)));
    EXPECT_FALSE(std::isnan(m.at(2,2)));
}

///////////////////
// Inertia Tests //
///////////////////
class AcrobotInertiaTest : public ::testing::Test
{
public:
    AcrobotInertiaTest()
    :   eps_(10e-13),
        simple1_(1,1),
        simple2_(30,1),
        simple3_(0.2,0.4),
        simple4_(0.2,0.2),
        complex1_(1,1,1,1,1,1,0,0),
        xingboBot_(
            0.2112, 0.1979,   // m
            0.148, 0.145,     // d
            0.073, 0.083,     // l
            0.00075, 0.00129) // J

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

    // Allowable error for tests to compare doubles
    double eps_;

    // Define simple acrobots
    AcrobotInertia simple1_;
    AcrobotInertia simple2_;
    AcrobotInertia simple3_;
    AcrobotInertia simple4_;
    // Define complex acrobots based on ones we can compute
    AcrobotInertia complex1_;
    AcrobotInertia xingboBot_;

    std::vector<AcrobotInertia> simpleAcrobots_ = 
    { simple1_, simple2_, simple3_, simple4_};

    // Define a list of values of qa for which we'll test the matrices generated
    // by the AcrobotInertia
    std::vector<double> qaVec_;

    // Get the mass matrices of all the simple inertia objects, in order, at the
    // given qa, with the rest of the config set to 0
    auto simpleMAtQa(double qa) -> std::vector<Matrix2>
    {
        Configuration config_at_qa;
        config_at_qa.alpha = qa;

        // Get the 2x2 inverse inertia at each qa
        auto Ms1 = simple1_.at(config_at_qa);
        auto Ms2 = simple2_.at(config_at_qa);
        auto Ms3 = simple3_.at(config_at_qa);
        auto Ms4 = simple4_.at(config_at_qa);

        // Now collect these into a vector
        std::vector<decltype(Ms1)> MVec = {
            Ms1, Ms2, Ms3, Ms4
        };

        return MVec;
    }


    // Get the mass matrices of all the simple inverse inertia objects, in
    // order, at the given qa, with the rest of the configuration set to 0.
    auto simpleMinvAtQa(double qa) -> std::vector<Matrix2>
    {
        Configuration config_at_qa;
        config_at_qa.alpha = qa;

        // Get the 2x2 inverse inertia at each qa
        auto Ms1 = simple1_.inverseAt(config_at_qa);
        auto Ms2 = simple2_.inverseAt(config_at_qa);
        auto Ms3 = simple3_.inverseAt(config_at_qa);
        auto Ms4 = simple4_.inverseAt(config_at_qa);

        // Now collect these into a vector
        std::vector<decltype(Ms1)> MinvVec = {
            Ms1, Ms2, Ms3, Ms4
        };

        return MinvVec;
    }
};

// Test to make sure that, for simple acrobots, we have
// mt = ml, dt = dl = lt = ll, and Jt = Jl = 0
TEST_F(AcrobotInertiaTest, SIMPLE_MASSES_AND_LENGTHS_MATCH)
{
    for(auto&& simple : simpleAcrobots_)
    {
        EXPECT_DOUBLE_EQ(simple.mt(), simple.ml());
        EXPECT_DOUBLE_EQ(simple.dt(), simple.dl());
        EXPECT_DOUBLE_EQ(simple.lt(), simple.ll());
        EXPECT_DOUBLE_EQ(simple.dt(), simple.lt());
        EXPECT_DOUBLE_EQ(simple.Jt(), 0);
        EXPECT_DOUBLE_EQ(simple.Jl(), 0);
    }
}

TEST_F(AcrobotInertiaTest, SIMPLE_M_IS_CORRECT)
{
    // Loop through and test the simple acrobot dynamics at each qa
    for(auto&& qa : qaVec_)
    {

        auto MVec = simpleMAtQa(qa);

        // Now check that the simple configurations at each element are
        // approximately the ones we expect. We will need acrobot information,
        // so recall that the masses and acrobots line up in the vector
        // locations.
        for(int i = 0; i < simpleAcrobots_.size(); ++i)
        {
            // Validate the simple elements:
            // (1,1) = 3ml^2 + 2ml^2cos(qa)
            // (1,2) = (2,1) = ml^2 + ml^2cos(qa)
            // (2,2) = ml^2

            // Get the mass and length. Since these are simple acrobots, use mt and lt.
            auto m = simpleAcrobots_[i].mt();
            auto l = simpleAcrobots_[i].lt();

            // compute cos(qa) as we use it in multiple places
            auto cqa = cos(qa);
            auto ml2 = m*l*l;

            // Compute the elements and assert that the element MVec[i]
            // aligns with the computed elements within tolerance
            auto a11 = ml2*(3 + 2*cqa);
            auto a12 = ml2*(1+cqa);
            auto a22 = ml2;
            EXPECT_PRED3(Within, MVec[i].at(1,1), a11, eps_);
            EXPECT_PRED3(Within, MVec[i].at(1,2), a12, eps_);
            EXPECT_PRED3(Within, MVec[i].at(2,1), a12, eps_);
            ASSERT_PRED3(Within, MVec[i].at(2,2), a22, eps_);
        }
    }
}

// Test to make sure that the outputs of simple acrobots match what MATLAB gives
// us for the value of Minv at each qa
TEST_F(AcrobotInertiaTest, SIMPLE_MINV_IS_CORRECT)
{
    // Loop through and test the simple acrobot dynamics at each qa
    for(auto&& qa : qaVec_)
    {

        auto MinvVec = simpleMinvAtQa(qa);

        // Now check that the simple configurations at each element are
        // approximately the ones we expect. We will need acrobot information,
        // so recall that the masses and acrobots line up in the vector
        // locations.
        for(int i = 0; i < simpleAcrobots_.size(); ++i)
        {
            // Validate the simple elements:
            // multiply all by 1/(ml^2(2-cos(qa)^2))
            // (1,1) = 1 
            // (1,2) = (2,1) = -(1+cos(qa))
            // (2,2) = 3 + 2*cos(qa)

            // Get the mass and length. Since these are simple acrobots, use mt and lt.
            auto m = simpleAcrobots_[i].mt();
            auto l = simpleAcrobots_[i].lt();

            // compute cos(qa) as we use it in multiple places
            auto cqa = cos(qa);
            // Compute the denominator of the inverse inertia matrix
            auto den = m*l*l*(2 - (cqa*cqa));

            // Now get the elements
            auto a11 = 1.0/den;
            EXPECT_PRED3(Within, MinvVec[i].at(1,1), a11, eps_);

            auto a12 = -(1 + cqa)/den;
            EXPECT_PRED3(Within, MinvVec[i].at(1,2), a12, eps_);
            EXPECT_PRED3(Within, MinvVec[i].at(2,1), a12, eps_);

            auto a22 = (3 + (2*cqa))/den;
            ASSERT_PRED3(Within, MinvVec[i].at(2,2), a22, eps_);
        }
        
    }
}

// Test to make sure changing qa does not affect the inertia matrix
TEST_F(AcrobotInertiaTest, SIMPLE_M_IS_FUNC_OF_QA)
{
    // For each qa, we want M(q) to be a function of only qa
    // We use pi/3 since it's a strange number and cannot be faked easily
    double qa = 7*M_PI/9;

    Configuration q;
    q.alpha = qa;

    //compute the actual components of M(qa) as before 
    auto cqa = cos(qa);
    auto m = simple3_.mt();
    auto l = simple3_.lt();
    auto ml2 = m*l*l;
    auto a11 = ml2*(3 + 2*cqa);
    auto a12 = ml2*(1+cqa);
    auto a22 = ml2;

    // Iterate through the qu values to make sure this is only a function of qa
    for(auto && qu : qaVec_)
    {
        q.psi = qu;
        // Test just one to make sure
        auto mat = simple3_.at(q);

         // Get the mass and length. Since these are simple acrobots, use mt and lt.
        auto m = simple3_.mt();
        auto l = simple3_.lt();

        // Expect that the mass matrix at this config is within tolerance
        // of the constant mass matrix elements above
        EXPECT_PRED3(Within, mat.at(1,1), a11, eps_);
        EXPECT_PRED3(Within, mat.at(1,2), a12, eps_);
        EXPECT_PRED3(Within, mat.at(2,1), a12, eps_);
        ASSERT_PRED3(Within, mat.at(2,2), a22, eps_);
    }
}

// Test to make sure that changing qu does not affect the value of the inverse
// inertia matrix (i.e. Minv(q) = Minv(qa)) for simple acrobots.
TEST_F(AcrobotInertiaTest, SIMPLE_MINV__IS_FUNC_OF_QA)
{
    // For each qa, we want Minv(q) to be a function of only qa
    // We use pi/3 since it's a strange number and cannot be faked easily
    double qa = 7*M_PI/9;

    Configuration q;
    q.alpha = qa;
    // Iterate through the qu values to make sure this is only a function of qa
    for(auto && qu : qaVec_)
    {
        q.psi = qu;
        // Test just one to make sure
        auto mat = simple3_.inverseAt(q);

         // Get the mass and length. Since these are simple acrobots, use mt and lt.
        auto m = simple3_.mt();
        auto l = simple3_.lt();

        // compute cos(qa) as we use it in multiple places
        auto cqa = cos(qa);
        // Compute the denominator of the inverse inertia matrix
        auto den = m*l*l*(2 - (cqa*cqa));

        // Now get the elements
        auto a11 = 1.0/den;
        EXPECT_PRED3(Within, mat.at(1,1), a11, eps_);

        auto a12 = -(1 + cqa)/den;
        EXPECT_PRED3(Within, mat.at(1,2), a12, eps_);
        EXPECT_PRED3(Within, mat.at(2,1), a12, eps_);

        auto a22 = (3 + (2*cqa))/den;
        ASSERT_PRED3(Within, mat.at(2,2), a22, eps_);
    }
}

// Test to make sure Xingbo's acrobot's inertia matrix matches the one MATLAB
// returns to us
TEST_F(AcrobotInertiaTest, XINGBO_M_CORRECT)
{
    for(auto&& qa : qaVec_)
    {
        Configuration q;
        q.alpha = qa;
        auto M = xingboBot_.at(q);

        // Get the actual content of the Xingbo mass matrix
        double cqa = cos(qa);
        
        double a11 = (6077509.0*cqa)/(1.25e9) + 17727239.0/(2.0e9);
        double a22 = 26533331.0/(1.0e10);
        double a12 = (6077509.0*cqa)/(2.5e9) + a22;
    }
}

// Test to make sure that the outputs of complex acrobots matches what we expect
// them to be for certain acrobots
TEST_F(AcrobotInertiaTest, XINGBO_MINV_IS_CORRECT)
{
    // Test Xingbo's acrobot, which has non-zero Jl and Jt
    for(auto&& qa : qaVec_)
    {
        Configuration q;
        q.alpha = qa;
        auto Minv = xingboBot_.inverseAt(q);

        // Get the ACTUAL contents of the Xingbo matrix
        auto cqa = cos(qa);
        auto cqa2 = cqa*cqa;
        double det = (102987240409999.0/(6.25e18))
                     - (36936115645081.0/(6.25e18))*cqa2;

        double a11 = 26533331.0/(1.0e10);
        double a12 = -6077509.0*cqa/(2.5e9) - a11;
        double a22 = 6077509.0*cqa/(1.25e9) + 17727239.0/(2.0e9);
        a11 = a11/det;
        a12 = a12/det;
        a22 = a22/det;

        // Now make sure they match
        ASSERT_PRED3(Within, Minv.at(1,1),a11,eps_);
        ASSERT_PRED3(Within, Minv.at(1,2),a12,eps_);
        ASSERT_PRED3(Within, Minv.at(2,1),a12,eps_);
        ASSERT_PRED3(Within, Minv.at(2,2),a22,eps_);
    }
}

// Test to make sure the matrix is pd at each q, and symmetric
TEST_F(AcrobotInertiaTest, M_IS_SYMMETRIC_PD)
{
    for(auto&& qa : qaVec_)
    {
        auto MVec = simpleMAtQa(qa);

        for(auto&& M : MVec)
        {
            // Check symmetry
            ASSERT_DOUBLE_EQ(M.at(1,2), M.at(2,1));

            // Check PD
            auto determinant = M.at(1,1)*M.at(2,2) - M.at(1,2)*M.at(2,1);
            EXPECT_GT(M.at(1,1), 0);
            ASSERT_GT(determinant,0);
        }
    }

}

// Test to make sure every matrix is positive definite at each q, and symmetric
TEST_F(AcrobotInertiaTest, MINV_IS_SYMMETRIC_PD)
{
    // Make sure that each matrix has positive (1,1) element, positive
    // determinant, and that (1,2) = (2,1)
    for(auto&& qa : qaVec_)
    {
        auto MinvVec = simpleMinvAtQa(qa);

        for(auto&& Minv : MinvVec)
        {
            // Check symmetry
            ASSERT_DOUBLE_EQ(Minv.at(1,2), Minv.at(2,1));

            // Check PD
            auto determinant = Minv.at(1,1)*Minv.at(2,2) - Minv.at(1,2)*Minv.at(2,1);
            EXPECT_GT(Minv.at(1,1), 0);
            EXPECT_GT(determinant,0);
        }
    }
}

/////////////////////
// Potential Tests //
/////////////////////
class AcrobotPotentialTest : public ::testing::Test
{
public:
    AcrobotPotentialTest()
    : simpleP_(simplem_, simplel_, g_),
      xingboP_(
        0.2112, 0.1979, //mt, ml,
        0.148, // dt
        0.073, 0.083, //lt, ll,
        g_)
    {}

protected:
    void SetUp() override
    {
    }

    void TearDown() override
    {}

    // Simple potential functions to test
    double simplem_ = 30.0;
    double simplel_ = 1.0;
    const double g_ = 9.81;

    // Define the acrobots. This must come after the constants.
    AcrobotPotential simpleP_;

    // Complex potential functions
    AcrobotPotential xingboP_;



}; // AcrobotPotentialTest

// Test for when qa = 0, qu varies i.e. the acrobot is a pendulum
TEST_F(AcrobotPotentialTest, SIMPLE_POTENTIAL_QU_VARYING)
{
    // We expect the mass to be 2*m at length = 3/2*l
    auto M = 2*simplem_;
    auto l = 3.0/2.0*simplel_;

    Configuration q;
    q.alpha = 0;

    for(int i = -9; i <= 9; ++i)
    {
        q.psi = M_PI*i/9.0;
        ASSERT_DOUBLE_EQ(simpleP_.at(q), M*g_*l*(1 - cos(q.psi)));
    }
}

// TEST for when qu = 0 and qa varies i.e. the first link is fied and the
// acrobot is a shorter pendulum
TEST_F(AcrobotPotentialTest, SIMPLE_POTENTIAL_QA_VARYING)
{
    auto m = simplem_;
    auto l = simplel_;

    Configuration q;
    q.psi= 0;

    for(int i = -9; i <= 9; ++i)
    {
        q.alpha = M_PI*i/9.0;
        ASSERT_DOUBLE_EQ(simpleP_.at(q), m*g_*l*(1 - cos(q.alpha)));
    }
}

// Test for generic qu and qa variations of  asimple acrobot
TEST_F(AcrobotPotentialTest, SIMPLE_POTENTIAL_CORRECT)
{
    auto m = simplem_;
    auto l = simplel_;

    Configuration q;

    for(int i = -9; i <= 9; ++i)
    {
        q.psi = M_PI*i/9.0;
        for(int j = -9; j <= 9; ++j)
        {
            q.alpha = M_PI*j/9.0;
            auto qu = q.psi;
            auto qa = q.alpha;
            ASSERT_DOUBLE_EQ(simpleP_.at(q), g_*((m*l*(1-cos(qu+qa)) + 2*m*l*(1-cos(qu)))));
        }
    }
}

// Test for the xingbo-bot
TEST_F(AcrobotPotentialTest, XINGBO_POTENTIAL_CORRECT)
{
    Configuration q;

    for(int i = -9; i <= 9; ++i)
    {
        q.psi = M_PI*i/9.0;
        for(int j = -9; j <= 9; ++j)
        {
            q.alpha = M_PI*j/9.0;
            auto qu = q.psi;
            auto qa = q.alpha;
            auto correct_P = 23988393.0/40000000.0 - 109643427.0/250000000.0*cos(qu) - 161136117.0/1000000000.0*cos(qu+qa);
            ASSERT_PRED3(Within, xingboP_.at(q), correct_P, 10e-14);
        }
    }
}

// Test to make sure the derivative is correct for simple systems
TEST_F(AcrobotPotentialTest, DV_DQU_SIMPLE)
{
    auto m = simplem_;
    auto l = simplel_;
    auto g = g_;

    Configuration q;

    for(int i = -9; i <= 9; ++i)
    {
        q.psi = M_PI*i/9.0;
        for(int j = -9; j <= 9; ++j)
        {
            q.alpha = M_PI*j/9.0;
            auto qu = q.psi;
            auto qa = q.alpha;
            auto correct_DP = 2*m*g*l*sin(qu) + m*g*l*sin(qu + qa);
            ASSERT_PRED3(Within, simpleP_.dqu(q), correct_DP, 10e-14);
        }
    }
}

// Test to make sure the derivative is ccorrect for xingbo's acrobot
TEST_F(AcrobotPotentialTest, DV_DQU_XINGBO)
{
    Configuration q;

    for(int i = -9; i <= 9; ++i)
    {
        q.psi = M_PI*i/9.0;
        for(int j = -9; j <= 9; ++j)
        {
            q.alpha = M_PI*j/9.0;
            auto qu = q.psi;
            auto qa = q.alpha;
            auto correct_DP = 109643427.0/250000000.0*sin(qu) + 161136117.0/1000000000.0*sin(qu + qa);
            ASSERT_PRED3(Within, xingboP_.dqu(q), correct_DP, 10e-14);
        }
    }

}

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

/* vim: set tw=80 ts=4 sw=4 sts=0 et ffs=unix : */
