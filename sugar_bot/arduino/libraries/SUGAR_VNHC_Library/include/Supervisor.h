/*******************************************************************************
* File:	        Supervisor.h
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
* Description:   A simple supervisor for switching between injection and
* dissipation VNHCs.
*******************************************************************************/
#ifndef __SUPERVISOR_H__
#define __SUPERVISOR_H__

#include "VNHC.h"

namespace SUGAR
{

class Supervisor
{
public:
    /**
     * @brief: Constructor which produces a supervisor that toggles between the
     * given injection and dissipation VNHCs. These do not have to be the same.
     */
	Supervisor(const AcrobotVNHC& in, const AcrobotVNHC& diss); 
	virtual ~Supervisor ();

    /**
     * @brief: Function to stabilize a given nonnegative value.
     *
     * @param: double the current value of the acrobot that you are trying to
     * stabilize (examples are qu, pu, or E)
     * @param: double the desired value of the acrobot which you want the first
     * input to reach. This must be >= 0.
     * @param: double the hysteresis; the supervisor will set qa = 0 whenever
     * abs(abs(value) - value_des) <= hys.
     *
     * @return: double the value of qa associated with the correct VNHC
     */
    auto stabilize(double val, double des, double hys) const -> double;

private:
    // The injection and dissipation VNHCs
    const AcrobotVNHC& in_;
    const AcrobotVNHC& diss_;

}; // class Supervisor

} // namespace SUGAR

#endif // __SUPERVISOR_H__
/* vim: set tw=80 ts=4 sw=4 sts=0 et ffs=unix : */
