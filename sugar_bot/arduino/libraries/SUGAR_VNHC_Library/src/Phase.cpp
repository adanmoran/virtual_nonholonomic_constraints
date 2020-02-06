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

#include "include/Phase.h"

namespace SUGAR
{

//////////////////
// Constructors //
//////////////////

Phase::Phase()
: qu(0)		    ,
  qa(0)		    ,
  pu(0)		    ,
  pa(0)		    ,
  E(0) 		    
{}

Phase::Phase(const AcrobotInertia& M, const State& state)
: Phase()
{
    // Set the State
    qu = state.psi;
    qa = state.alpha;

    //  Set the conjugate of momenta
    //  p = M(q)*[dpsi; dalpha]
    auto Mq = M.at(state);
    pu = Mq.at(1,1)*state.dpsi + Mq.at(1,2)*state.dalpha;
    pa = Mq.at(2,1)*state.dpsi + Mq.at(2,2)*state.dalpha;

    // Set the energy
    E = state.E; 
}

////////////////////
// Public Methods //
////////////////////

auto Phase::unactuatedPhase() -> UnactuatedPhase
{
	UnactuatedPhase qpu; 
	qpu.qu = qu; qpu.pu = pu;
    return qpu;
}

}; // namespace SUGAR
/* vim: set tw=80 ts=4 sw=4 sts=0 et ffs=unix : */
