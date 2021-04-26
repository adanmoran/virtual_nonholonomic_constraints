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

}; // namespace SUGAR
/* vim: set tw=80 ts=4 sw=4 sts=0 et ffs=unix : */
