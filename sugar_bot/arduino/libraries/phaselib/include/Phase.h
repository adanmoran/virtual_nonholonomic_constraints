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
 * @brief: Stores the Acrobot phase for the Hamiltonian framework. 
 * It takes an inverse inertia matrix, so that it can
 * compute its own updates.
 */
class Phase
{
public:
    Phase (const AcrobotInertia& M);
    Phase (const AcrobotInertia& M, const Configuration& configuration);

    /**
    * @brief: Updates the phase by computing p = M(q) q_dot at the given
    * configuration.
    *
    * @param: Configuration configuration
    *
    * @return: bool
    */
    auto updateFromConfiguration(const Configuration& configuration) -> bool;

    double qu; // Configuration unactuated variable = psi
    double qa; // Configuration actuated variable   = alpha
    double pu; // Phase unactuated momentum         = e1' * M(q) * p
    double pa; // Phase actuated momentum           = e2' * M(q) * p
    double E;  // Configuration energy as given by configuration

private:
    const AcrobotInertia& M_; // The inverse inertia matrix of the acrobot
}; // class Phase


}; // namespace SUGAR
#endif

/* vim: set tw=80 ts=4 sw=4 sts=0 et ffs=unix : */
