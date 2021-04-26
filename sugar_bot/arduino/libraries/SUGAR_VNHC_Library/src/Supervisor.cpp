/*******************************************************************************
* File:	        Supervisor.cpp
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
* Description:   Implementation of Supervisor.h
*******************************************************************************/

#include "include/Supervisor.h"
#include <math.h>

namespace SUGAR
{

Supervisor::Supervisor(const AcrobotVNHC& in, const AcrobotVNHC& diss)
: in_(in),
  diss_(diss)
{}

Supervisor::~Supervisor()
{}

auto Supervisor::stabilize(const UnactuatedPhase& qpu, 
                           double value, 
                           double des, 
                           double hys) const -> double
{
    // Make sure everything is positive.
    des = fabs(des);
    value = fabs(value);
    hys = fabs(hys);

    // injection
    if (value < des - hys)
    {
        return in_.qa(qpu);
    }
    // dissipation
    else if (value > des + hys)
    {
        return diss_.qa(qpu);
    }
    // stability
    return 0;
}

///////////////////////////
// OscillationSupervisor //
///////////////////////////

OscillationSupervisor::OscillationSupervisor(const Supervisor& sup)
: sup_(sup)
{}

auto OscillationSupervisor::stabilize(const UnactuatedPhase& qpu,
                                      double qudes,
                                      double hys) const -> double
{
    // Make sure qudes and hys are positive.
    qudes = fabs(qudes);
    hys = fabs(hys);

    // Extract qu. We don't care about its sign.
    double qu = fabs(qpu.qu);

    // If we are within range of qudes, then we update our "peak"
    // (even though this is not a peak) so that we maintain our qa = 0.
    // This means that dissipation only occurs when we are outside the range;
    // as soon as we hit the range again, we go back to qa = 0.
    // TODO: If this doesn't work in experiments, remove it.
    if (fabs(qudes - qu) <= hys)
    {
        prevPeak_ = qpu;
    }

    // Check if pu has changed sign since last time. If so we reached a peak,
    // which we take as this qu.

    // NOTE: if pu = 0 we save this as a peak, but the next position we measure
    // where pu != 0 will also be taken as a peak. If you have fast enough
    // measurements this should not be a problem.
    double pu = qpu.pu;
    if (pu == 0 || (sign(pu) != sign(prevPeak_.pu)))
    {
        prevPeak_ = qpu;
    }
    
    // Return the appropriate injection mechanism for this value qpu, based on the previous peak.
    return sup_.stabilize(qpu,prevPeak_.qu,qudes,hys);
}

}; // namespace SUGAR
/* vim: set tw=80 ts=4 sw=4 sts=0 et ffs=unix : */
