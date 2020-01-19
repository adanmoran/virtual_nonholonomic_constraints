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

///////////////////////////
// Inverse Inertia Tests //
///////////////////////////
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

    // Define a list of values of qa for which we'll test the matrices generated
    // by the AcrobotInverseInertia
    std::vector<double> qaVec_;
};

// Test to make sure that the outputs of simple acrobots match what MATLAB gives
// us for the value of Minv at each qa
TEST_F(AcrobotDynamicsTest, SIMPLE_MINV_IS_CORRECT)
{
    Configuration config_at_qa;
    // Loop through and test the simple acrobot dynamics at each qa
    for(auto&& qa : qaVec_)
    {
        // Update the qa value.
        config_at_qa.alpha = qa;

        // Get the 2x2 inverse inertia at each qa
        auto Ms1 = simple1_.at(config_at_qa);
        auto Ms2 = simple2_.at(config_at_qa);
        auto Ms3 = simple3_.at(config_at_qa);
        auto Ms4 = simple4_.at(config_at_qa);

        // Now collect these into a vector
        std::vector<decltype(Ms1)> MinvVec = {
            Ms1, Ms2, Ms3, Ms4
        };

        // Now check that the simple configurations at each element are
        // approximately the ones we expect
        for(auto&& Minv : MinvVec)
        {
            // TODO: validate the simple elements:
            // multiply all by 1/(ml^2(2-cos(qa)^2))
            // (1,1) = 1 
            // (1,2) = (2,1) = -(1+cos(qa))
            // (2,2) = 3 + 2*cos(qa)
        }
        
    }
}

// Test to make sure that changing qu does not affect the value of the inverse
// inertia matrix (i.e. Minv(q) = Minv(qa)) for simple acrobots.
TEST_F(AcrobotDynamicsTest, SIMPLE_IS_MINV_QA)
{}

// Test to make sure that the outputs of complex acrobots matches what the
// equatiosn should be
TEST_F(AcrobotDynamicsTest, COMPLEX_MINV_IS_CORRECT)
{}

// Test to make sure that the outputs of complex acrobots do not depend on qu
TEST_F(AcrobotDynamicsTest, COMPLEX_IS_MINV_QA)
{}

// Test to make sure every matrix is positive definite at each q, and symmetric
TEST_F(AcrobotDynamicsTest, MINV_IS_SYMMETRIC_PD)
{}

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

/* vim: set tw=80 ts=4 sw=4 sts=0 et ffs=unix : */
