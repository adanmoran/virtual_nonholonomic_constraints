/*******************************************************************************
* File:	        Phase.h
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
* Description:   VNHC code for the SUGAR_bot. Note thatArduino does not use full
* C++ functionality. We will do some OOP, but will break a few rules. Here's
* what we'll do: classes have public local variables because it is less overhead
* than calling functions on legacy arduino products as compared to using
* getters/setters.
*******************************************************************************/

#ifndef __PHASE_H__
#define __PHASE_H__

#include "AcrobotDynamics.h"

namespace SUGAR
{

/**
 * Forward declare the UnactuatedPhase object
 */
class UnactuatedPhase;
/**
 * @brief: Stores the Acrobot phase for the Hamiltonian framework. 
 * It takes an inverse inertia matrix, so that it can
 * compute its own updates.
 */
class Phase
{
public:
    Phase ();
    Phase (const AcrobotInertia& M, const State& state);

    /**
    * @brief: Returns the unactuated phase components for use in standard VNHCs
    *
    * @return: UnactuatedPhase
    */
    auto unactuatedPhase() -> UnactuatedPhase;

    double qu; // State unactuated variable = psi
    double qa; // State actuated variable   = alpha
    double pu; // Phase unactuated momentum         = e1' * M(q) * p
    double pa; // Phase actuated momentum           = e2' * M(q) * p
    double E;  // State energy as given by State
}; // class Phase

/**
 * @brief: Stores the unactuated variables of a phase object
 */
struct UnactuatedPhase
{
    double qu = 0.0;
    double pu = 0.0;
};

}; // namespace SUGAR
#endif

/* vim: set tw=80 ts=4 sw=4 sts=0 et ffs=unix : */
