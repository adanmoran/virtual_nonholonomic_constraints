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

Phase::Phase(const AcrobotInertia& M)
: qu(0)		    ,
  qa(0)		    ,
  pu(0)		    ,
  pa(0)		    ,
  E(0) 		    ,
  M_(M) 	
{}

Phase::Phase(const AcrobotInertia& M, const Configuration& configuration)
: Phase(M)
{
    fromConfiguration(configuration);
}

//////////////////////
// Public Functions //
//////////////////////

auto Phase::fromConfiguration(const Configuration& configuration) -> bool
{
    // Todo: Update the configuration
    qu = configuration.psi;
    qa = configuration.alpha;

    //  Update the conjugate of momenta
    //  p = M(q)*[dpsi; dalpha]
    auto M = M_.at(configuration);
    pu = M.at(1,1)*configuration.dpsi + M.at(1,2)*configuration.dalpha;
    pa = M.at(2,1)*configuration.dpsi + M.at(2,2)*configuration.dalpha;

    return true;
}

}; // namespace SUGAR
/* vim: set tw=80 ts=4 sw=4 sts=0 et ffs=unix : */
