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

// Our required headers
#include "Phase.h"
#include "StateSpace.h"
#include "AcrobotDynamics.h"
// C++ libraries
#include <vector>
#include <iostream>
#define _USE_MATH_DEFINES
#include <math.h>
#undef _USE_MATH_DEFINES
// Test libraries
#include "gtest/gtest.h"

using namespace SUGAR;

class PhaseTest : public ::testing::Test
{
public:
	PhaseTest()
	: simple1_(1,1),
	  simple2_(30,2),
	  xingboBot_(
        0.2112, 0.1979,   // m
        0.148, 0.145,     // d
        0.073, 0.083,     // l
        0.00075, 0.00129), // J
	  ps1_(),
	  ps2_(),
	  px_()
	{}

protected:
	void SetUp() override 
	{
		qpb2000_.psi = M_PI/2;
		q0pb200_.alpha = M_PI/2;
		q0010_.dpsi = 1;
		q0001_.dalpha = 1;
		qpb4npb8n2n1_.psi = M_PI/4;
		qpb4npb8n2n1_.alpha = -M_PI/8;
		qpb4npb8n2n1_.dpsi = -2;
		qpb4npb8n2n1_.dalpha = -1;
	}
	void TearDown() override {}

	// Get a vector of States where qd = 0
	auto qd0() -> std::vector<State>
	{
		std::vector<State> qd_0(3);
		qd_0.push_back(q0_);
		qd_0.push_back(qpb2000_);
		qd_0.push_back(q0pb200_);
		return qd_0;
	}

	auto qs() -> std::vector<State>
	{
		auto qVec = qd0();
		qVec.push_back(q0010_);
		qVec.push_back(q0001_);
		qVec.push_back(qpb4npb8n2n1_);
		return qVec;
	}

	// Some simple acrobots for basic tests
	AcrobotInertia simple1_;
	AcrobotInertia simple2_;
	// Set up the xingbo-bot for testing
	AcrobotInertia xingboBot_;

	// Set up a State for updating
	State q0_; // all zero configuration
	State qpb2000_; // qu = pi/2
	State q0pb200_;  // qa = pi/2
	State q0010_; // qud = 1 (bottom of swing)
	State q0001_; // qad = 1 
	State qpb4npb8n2n1_; // qu = pi/4, qa = pi/8, qud = -2, qad = -1

	// Set up a phase object for each test acrobot
	Phase ps1_; // Phase for simple2
	Phase ps2_; // Phase for Simple1
	Phase px_; // Phase for xingbo acrobot
};

// Test to make sure that when you call update that psi = qu, alpha = qa
TEST_F(PhaseTest, Q_IS_THE_SAME)
{
	auto qVals = qs();
	for(int i = 0; i < qVals.size(); ++i)
	{
		auto q = qVals[i];
		ps1_ = Phase(simple1_, q);
		ASSERT_DOUBLE_EQ(ps1_.qu, q.psi);
		ASSERT_DOUBLE_EQ(ps1_.qa, q.alpha);
		ps2_ = Phase(simple2_, q);
		ASSERT_DOUBLE_EQ(ps2_.qu, q.psi);
		ASSERT_DOUBLE_EQ(ps2_.qa, q.alpha);
		px_ = Phase(xingboBot_, q);
		ASSERT_DOUBLE_EQ(px_.qu, q.psi);
		ASSERT_DOUBLE_EQ(px_.qa, q.alpha);
	}
}

// Test to ensure a non-zero velocity results in the correct output for simple
// acrobots
TEST_F(PhaseTest, SIMPLE_ACROBOTS)
{
	// First we test simple1, which has a mass matrix of
	// [ 3 + 2cqa, 1+cqa; 1+cqa, 1]
	auto qa = M_PI/8;
	auto cqa = cos(qa);
	// M(q)qd = [3+2cqa; 1+cqa]
	ps1_ = Phase(simple1_, q0010_);
	ASSERT_DOUBLE_EQ(ps1_.pu, 5);
	ASSERT_DOUBLE_EQ(ps1_.pa, 2);
	// M(q)qd = [1+cqa;1]
	ps1_ = Phase(simple1_, q0001_);
	ASSERT_DOUBLE_EQ(ps1_.pu,2);
	ASSERT_DOUBLE_EQ(ps1_.pa,1);
	// M(q)qd = [(3+2cqa)qud + (1+cqa)qad; (1+cqa)qud + qad];
	ps1_ = Phase(simple1_, qpb4npb8n2n1_);
	ASSERT_DOUBLE_EQ(ps1_.pu, -2*(3+2*cqa) - (1+cqa));
	ASSERT_DOUBLE_EQ(ps1_.pa, -2*(1+cqa) - 1);

	// Now test simple2, which has a mass matrix given by
	// ml^2 [ 3+2*cqa, 1+cqa; 1+cqa, 1]
	auto m = simple2_.mt();
	auto l = simple2_.lt();
	auto ml2 = m*l*l;
	// M(q)qd = ml2*[3+2cqa; 1+cqa]
	ps2_ = Phase(simple2_, q0010_);
	ASSERT_DOUBLE_EQ(ps2_.pu, ml2*5);
	ASSERT_DOUBLE_EQ(ps2_.pa, ml2*2);
	// M(q)qd = ml2*[1+cqa;1]
	ps2_ = Phase(simple2_, q0001_);
	ASSERT_DOUBLE_EQ(ps2_.pu,ml2*2);
	ASSERT_DOUBLE_EQ(ps2_.pa,ml2*1);
	// M(q)qd = ml2[(3+2cqa)qud + (1+cqa)qad; (1+cqa)qud + qad];
	ps2_ = Phase(simple2_, qpb4npb8n2n1_);
	ASSERT_DOUBLE_EQ(ps2_.pu, ml2*(-2*(3+2*cqa) - (1+cqa)));
	ASSERT_DOUBLE_EQ(ps2_.pa, ml2*(-2*(1+cqa) - 1));
}

// Test to ensure a non-zero velocity results in the correct output for xingbo's
// acrobot
TEST_F(PhaseTest, XINGBO_BOT)
{
	// Parameter in front of cos(qa) in M(1,1)
	double a11_1 = 6077509.0/(1.25e9);
	// Parameter added to a11_1*cqa in M(1,1)
	double a11_2 = 17727239.0/(2.0e9);
	// Parameter in front of cos(qa) in M(1,2), M(2,1)
	double a12_1 = 6077509.0/(2.5e9);
	// Parameter added to a12_1*cqa in M(1,2), M(2,1)
	double a12_2 = 26533331/(1.0e10);
	// Parameter in a22
	double a22 = a12_2;
	// Phase of Xingbo is given as follows
	auto xpu = [=](double qa, double qud, double qad) {
		return (a11_1*cos(qa) + a11_2)*qud + (a12_1*cos(qa) + a12_2)*qad;
	};
	auto xpa = [=](double qa, double qud, double qad) {
		return (a12_1*cos(qa) + a12_2)*qud + a22*qad;
	};

	// For qud = 1
	px_ = Phase(xingboBot_, q0010_);
	ASSERT_DOUBLE_EQ(px_.pu, 137256267/(10e9));
	ASSERT_DOUBLE_EQ(px_.pa, 50843367/(10e9));
	// For qad = 1
	px_ = Phase(xingboBot_, q0001_);
	ASSERT_DOUBLE_EQ(px_.pu, 50843367/(10e9));
	ASSERT_DOUBLE_EQ(px_.pa, 26533331/(10e9));
	// For mixed qa, qd
	px_ = Phase(xingboBot_, qpb4npb8n2n1_);
	ASSERT_DOUBLE_EQ(px_.pu, xpu(-M_PI/8, -2, -1));
	ASSERT_DOUBLE_EQ(px_.pa, xpa(-M_PI/8, -2, -1));
}

// Test to make sure unactuated phase method returns the unactuated phase
TEST_F(PhaseTest, UNACTUATED_PHASE_IS_CORRECT)
{
	auto qVals = qs();
	for(int i = 0; i < qVals.size(); ++i)
	{
		auto q = qVals[i];
		ps1_ = Phase(simple1_, q);

		auto ps1u = ps1_.unactuatedPhase();
		ASSERT_DOUBLE_EQ(ps1u.qu, ps1_.qu);
		ASSERT_DOUBLE_EQ(ps1u.pu, ps1_.pu);

		ps2_ = Phase(simple2_,q);
		auto ps2u = ps2_.unactuatedPhase();
		ASSERT_DOUBLE_EQ(ps2u.qu, ps2_.qu);
		ASSERT_DOUBLE_EQ(ps2u.pu, ps2_.pu);

		px_ = Phase(xingboBot_, q);
		auto pxu = px_.unactuatedPhase();
		ASSERT_DOUBLE_EQ(pxu.qu, px_.qu);
		ASSERT_DOUBLE_EQ(pxu.pu, px_.pu);
	}
}

TEST_F(PhaseTest, QD_0_IMLIES_P_0)
{
	ps1_ = Phase(simple1_, q0_);
	EXPECT_DOUBLE_EQ(ps1_.pu, 0);
	ASSERT_DOUBLE_EQ(ps1_.pa, 0);

	ps2_ = Phase(simple2_, qpb2000_);
	EXPECT_DOUBLE_EQ(ps2_.pu, 0);
	ASSERT_DOUBLE_EQ(ps2_.pa, 0);

	px_ = Phase(xingboBot_, q0pb200_);
	EXPECT_DOUBLE_EQ(px_.pu, 0);
	ASSERT_DOUBLE_EQ(px_.pa, 0);
}

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

/* vim:set noet sts=0 sw=4 ts=4 tw=80 : */
