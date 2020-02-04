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

AcrobotVNHC::AcrobotVNHC(const Acrobot& acrobot)
: acrobot_(acrobot)
{}

AcrobotVNHC::~AcrobotVNHC()
{}

auto AcrobotVNHC::pa(const UnactuatedPhase& qpu) const -> double
{
    // TODO: Compute pa with the equation from VNHC research
    return 0.0;
}

auto AcrobotVNHC::dqa(const UnactuatedPhase& qpu) const -> double
{
    // for qa = f(qu,pu), this is always true
    return 1;
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
