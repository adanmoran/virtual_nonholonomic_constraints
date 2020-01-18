/*******************************************************************************
* File:	        Phase.cpp
*		 
* Author:       Adan Moran-MacDonald
* Created:      14/Jan/20
*
* Systems Control Group
* Department of Electrical and Computer Engineering
* University of Toronto
*
* Last Modified: 14/Jan/20
* Last Editor:   Adan Moran-MacDonald
* Description:   Source file for the Phase library
*******************************************************************************/

#include "Phase.h"

namespace SUGAR
{

//////////////////
// Constructors //
//////////////////

Phase::Phase(const AcrobotInverseInertia& Minv)
: qu(0)		    ,
  qa(0)		    ,
  pu(0)		    ,
  pa(0)		    ,
  E(0) 		    ,
  Minv_(Minv) 	
{}

Phase::Phase(const AcrobotInverseInertia& Minv, const Configuration& configuration)
: Phase(Minv)
{
    updateFromConfiguration(configuration);
}

//////////////////////
// Public Functions //
//////////////////////

auto Phase::updateFromConfiguration(const Configuration& configuration) -> bool
{
    // Todo: Update the configuration

    //  Update the conjugate of momenta
    //  TODO: p = Minv(q)*[dpsi; dalpha]

    return true;
}

}; // namespace SUGAR
/* vim: set tw=80 ts=4 sw=4 sts=0 et ffs=unix : */
