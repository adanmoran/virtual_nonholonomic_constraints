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

auto Supervisor::stabilize(double value, double des, double hys) const -> double
{
    if(fabs(fabs(value)-des) <= hys)
    {
        return 0;
    }

    return 1.0;
}

}; // namespace SUGAR
/* vim: set tw=80 ts=4 sw=4 sts=0 et ffs=unix : */
