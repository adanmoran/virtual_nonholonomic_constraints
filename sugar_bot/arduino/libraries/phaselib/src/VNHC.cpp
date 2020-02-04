/*******************************************************************************
* File:	        VNHC.cpp
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
* Description:   Implements VNHC.h, the virutal nonholonomic constraint code
*******************************************************************************/

#include "VNHC.h"

namespace SUGAR
{
/////////////////
// AcrobotVNHC //
/////////////////

AcrobotVNHC::AcrobotVNHC(const AcrobotInertia& M, const AcrobotPotential& V)
: M_(M),
  V_(V)
{}

AcrobotVNHC::~AcrobotVNHC()
{}

auto AcrobotVNHC::pa(const UnactuatedPhase& qpu) -> double
{
    // TODO: Compute pa with the equation from VNHC research
    return 0.0;
}

auto AcrobotVNHC::dqa(const UnactuatedPhase& qpu) -> double
{
    // for qa = f(qu,pu), this is always true
    return 1;
}

//-------------------//
// Private Functions //
//-------------------//
auto AcrobotVNHC::M() const -> const AcrobotInertia&
{
    return M_;
}

auto AcrobotVNHC::V() const -> const AcrobotPotential&
{
    return V_;
}

//////////////
// TanhVNHC //
//////////////
// TODO: Fill in the TanhVNHC code and test it
//////////////////
// SinThetaVNHC //
//////////////////

}; // namespace SUGAR
/* vim: set tw=80 ts=4 sw=4 sts=0 et ffs=unix : */
