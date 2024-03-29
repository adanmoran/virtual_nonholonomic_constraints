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
: acrobot_(acrobot),
  qmax_(M_PI)
{}

AcrobotVNHC::AcrobotVNHC(const Acrobot& acrobot, ActuatorLimit qmax)
: acrobot_(acrobot)
{
    // Limit ourselves to have an actuator limit within -PI to PI, since this is
    // an acrobot after all.
    if(qmax < 0)
    {
        qmax = 0;
    }
    else if(qmax > M_PI)
    {
        qmax = M_PI;
    }
    qmax_ = qmax;
}

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

TanhVNHC::TanhVNHC(const Acrobot& acrobot, ActuatorLimit qmax)
: TanhVNHC(acrobot, qmax, ScalingFactor(1.0))
{}

TanhVNHC::TanhVNHC(const Acrobot& acrobot, ActuatorLimit qmax, ScalingFactor scale)
: AcrobotVNHC(acrobot, qmax),
  scale_(scale)
{}

TanhVNHC::~TanhVNHC()
{}

auto TanhVNHC::qa(const UnactuatedPhase& qpu) const -> double
{
    // This should return qmax*tanh(scale*pu)
    return qmax()*tanh(scale_*qpu.pu);
}

auto TanhVNHC::dqu(const UnactuatedPhase& qpu) const -> double
{
    // This should return -d/dqu tanh(pu) = 0
    return 0.0;
}

auto TanhVNHC::dpu(const UnactuatedPhase& qpu) const -> double
{
    // This returns the derivative dh/dpu = -qmax* d/dpu tanh(scale*pu) =
    // qmax_*scale_*(tanh(scale_*pu)^2 - 1)
    auto tanhpu = tanh(scale_*qpu.pu);
    auto tanhpu2 = tanhpu*tanhpu;

    return qmax()*scale_*(tanhpu2 - 1);
}

//////////////
// SinuVNHC //
//////////////
SinuVNHC::SinuVNHC(const Acrobot& acrobot, ActuatorLimit qmax)
: AcrobotVNHC(acrobot, qmax)
{}

SinuVNHC::~SinuVNHC()
{}

auto SinuVNHC::qa(const UnactuatedPhase& qpu) const -> double
{
    // This should return qmax*sin(theta). For simplicity, we won't solve for
    // theta, we'll just use the expanded form. This is because sqrt is faster
    // than atan2 and sine (usually).
    return qmax()*qpu.pu/norm(qpu);
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
    return qmax()*qpu.qu*qpu.pu/qpu_norm3;
}

auto SinuVNHC::dpu(const UnactuatedPhase& qpu) const -> double
{
    // This should return the derivative dh/dpu =
    // = -cos(theta)*d/dpu theta
    // For simplicity, we will solve this without plugging in for theta. We
    // should get -qmax*qu^2 / ((qu^2 + pu^2)^(3/2))
    auto qpu_norm = norm(qpu);
    auto qpu_norm3 = qpu_norm*qpu_norm*qpu_norm;
    return -qmax()*qpu.qu*qpu.qu/qpu_norm3;
}

////////////////
// ArctanVNHC //
////////////////
ArctanVNHC::ArctanVNHC(const Acrobot& acrobot, ActuatorLimit qmax)
: ArctanVNHC(acrobot, qmax, ScalingFactor(1.0))
{}

ArctanVNHC::ArctanVNHC(const Acrobot& acrobot, ActuatorLimit qmax, ScalingFactor scale)
: AcrobotVNHC(acrobot,qmax),
  scale_(scale)
{}

ArctanVNHC::~ArctanVNHC()
{}

auto ArctanVNHC::qa(const UnactuatedPhase& qpu) const -> double
{
    // This should return qmax*Arctan(scale*pu)
    return qmax()*pib2inv_*atan(scale_*qpu.pu);
}

auto ArctanVNHC::dqu(const UnactuatedPhase& qpu) const -> double
{
    // This should return -d/dqu Arctan(pu) = 0
    return 0.0;
}

auto ArctanVNHC::dpu(const UnactuatedPhase& qpu) const -> double
{
    // This returns the derivative dh/dpu = -qmax* d/dpu Arctan(scale*pu) =
    // -qmax*scale/(1 + scale^2*pu^2)
    return -qmax()*pib2inv_*scale_/(1 + scale_*scale_*qpu.pu*qpu.pu);
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
