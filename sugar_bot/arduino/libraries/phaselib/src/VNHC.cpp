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

auto AcrobotVNHC::dpa(const UnactuatedPhase& qpu) const -> double
{
    // for qa = f(qu,pu), this is always true
    return 0;
}

//////////////
// TanhVNHC //
//////////////

TanhVNHC::TanhVNHC(const Acrobot& acrobot)
: AcrobotVNHC(acrobot)
{}

TanhVNHC::~TanhVNHC()
{}

auto TanhVNHC::qa(const UnactuatedPhase& qpu) const -> double
{
    // TODO: This should return tanh(pu)
    return 0;
}

auto TanhVNHC::dqu(const UnactuatedPhase& qpu) const -> double
{
    // This should return -d/dqu tanh(pu) = 0
    return 0.0;
}

auto TanhVNHC::dpu(const UnactuatedPhase& qpu) const -> double
{
    // TODO: This should return the derivative dh/dpu = - d/dpu tanh(pu)
    return 0;
}

//////////////
// SinuVNHC //
//////////////
SinuVNHC::SinuVNHC(double qmax, const Acrobot& acrobot)
: AcrobotVNHC(acrobot),
  qmax_(qmax)
{}

SinuVNHC::~SinuVNHC()
{}

auto SinuVNHC::qa(const UnactuatedPhase& qpu) const -> double
{
    // TODO: This should return qmax*Sin(atan2(pu,qu))
    return 0;
}

auto SinuVNHC::dqu(const UnactuatedPhase& qpu) const -> double
{
    //TODO: This should return -d/dqu sin(atan2(pu,qu)) =
    //qbar*pu*qu/((qu^2+pu^2)^(3/2))
    return 0.0;
}

auto SinuVNHC::dpu(const UnactuatedPhase& qpu) const -> double
{
    // TODO: This should return the derivative dh/dpu =
    // = - qmax*qu^2/((qu^2 + pu^2)^(3/2))
    return 0;
}


}; // namespace SUGAR
/* vim: set tw=80 ts=4 sw=4 sts=0 et ffs=unix : */
