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

#include "include/VNHC.h"
// For tanh, cosh, sin, cos, etc
#include <math.h>

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
    // Compute pa with the equation from VNHC research.
    // This equation is of the form
    // pa = (dhq*Minv*e1)^{-1} * (dhpu*dP/dqu - dhq*Minv*e1*pu)
    
    // The acrobot inertia matrix depends on qa, so take qa = f(qu,pu) and
    // use that in the configuration state.
    Configuration q;
    q.psi = qpu.qu; q.alpha = qa(qpu);
    auto Minv = acrobot_.M().inverseAt(q);

    // Now compute dhq*Minv, which is a row vector with two components
    auto dhqu = dqu(qpu);
    auto dhqa = dqa(qpu);
    auto dhqMinv1 = dhqu*Minv.at(1,1) + dhqa*Minv.at(2,1); // dhq*Minv*e1
    auto dhqMinv2 = dhqu*Minv.at(1,2) + dhqa*Minv.at(2,2); // dhq*Minv*e2

    // Next, we can compute dhpu*dP/dqu
    auto dPqu = acrobot_.V().dqu(q);
    auto dhpu_dPqu = dpu(qpu)*dPqu;

    // Finally, we compute the whole thing.
    return (dhpu_dPqu - qpu.pu*dhqMinv1)/dhqMinv2;
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
    // This should return tanh(pu)
    return tanh(qpu.pu);
}

auto TanhVNHC::dqu(const UnactuatedPhase& qpu) const -> double
{
    // This should return -d/dqu tanh(pu) = 0
    return 0.0;
}

auto TanhVNHC::dpu(const UnactuatedPhase& qpu) const -> double
{
    // TODO: This should return the derivative dh/dpu = - d/dpu tanh(pu) =
    // tanh(pu)^2 - 1
    auto tanhpu = qa(qpu);
    auto tanhpu2 = tanhpu*tanhpu;

    return tanhpu2 - 1;
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
    // This should return qmax*sin(theta). For simplicity, we won't solve for
    // theta, we'll just use the expanded form. This is because sqrt is faster
    // than atan2 and sine (usually).
    return qmax_*qpu.pu/norm(qpu);
}

auto SinuVNHC::dqu(const UnactuatedPhase& qpu) const -> double
{
    //This should return -d/dqu sin(theta) =
    // -cos(theta) * d/dqu theta
    //
    // For simplicity, we will solve this without plugging in for theta. We
    // should get qmax*qu*pu / ((qu^2 + pu^2)^(3/2))
    auto qpu_norm = norm(qpu);
    auto qpu_norm3 = qpu_norm*qpu_norm*qpu_norm;
    return qmax_*qpu.qu*qpu.pu/qpu_norm3;
}

auto SinuVNHC::dpu(const UnactuatedPhase& qpu) const -> double
{
    // This should return the derivative dh/dpu =
    // = -cos(theta)*d/dpu theta
    // For simplicity, we will solve this without plugging in for theta. We
    // should get -qmax*qu^2 / ((qu^2 + pu^2)^(3/2))
    auto qpu_norm = norm(qpu);
    auto qpu_norm3 = qpu_norm*qpu_norm*qpu_norm;
    return -qmax_*qpu.qu*qpu.qu/qpu_norm3;
}

//-------------------//
// Private Functions //
//-------------------//
auto SinuVNHC::norm(const UnactuatedPhase& qpu) const -> double
{
    return sqrt(qpu.qu*qpu.qu + qpu.pu*qpu.pu);
}

}; // namespace SUGAR
/* vim: set tw=80 ts=4 sw=4 sts=0 et ffs=unix : */
