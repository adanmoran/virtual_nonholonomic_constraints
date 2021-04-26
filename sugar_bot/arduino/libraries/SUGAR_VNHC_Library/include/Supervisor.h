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
     * @brief: Function to stabilize a given nonnegative value to a desired
     * nonnegative value, within some hysteresis.
     *
     * @param: const UnactuatedPhase&. The current unactuated phase for the
     * acrobot.
     * @param: double the current value of the acrobot that you are trying to
     * stabilize (examples are qu, pu, or E)
     * @param: double the desired value of the acrobot which you want the first
     * input to reach. This must be >= 0.
     * @param: double the hysteresis. This must be >= 0. the supervisor will set
     * qa = 0 whenever abs(abs(value) - value_des) <= hys.
     *
     * @return: double the value of qa associated with the correct VNHC
     */
    auto stabilize( const UnactuatedPhase& qpu, 
                    double value, 
                    double des, 
                    double hys) const -> double;

private:
    // The injection and dissipation VNHCs
    const AcrobotVNHC& in_;
    const AcrobotVNHC& diss_;

}; // class Supervisor

/*****************************************************************************/

class OscillationSupervisor
{
public:
    /**
     * @brief: Create a meta-supervisor which tracks the value qu = qudes using the given supervisor.
     */
    OscillationSupervisor(const Supervisor& sup);

    /**
      * @brief: Set qa to stabilize a desired qu in ]-pi,pi[. This updates an
      * internal state which keeps track of the injection/dissipation process;
      * if this fails, simply input a (qu,pu)=(0,0) into the unactuated phase to
      * reset.
      * @param: const UnactuatedPhase&. The current phase of the acrobot.
      * @param: double. The desired angle qu, in ]-pi,pi[. If it is out of
      * range, we set the desired angle to -pi+hys or pi-hys depending on the
      * sign of the input.
      * @param: double. The hysteresis.
      *
      * @return: double. The value of qa.
      */
    auto stabilize(const UnactuatedPhase& qpu,
                   double qudes,
                   double hys) const -> double;

private:
    // The sign function
    auto sign(double x) const -> double
    {
        return (x > 0) - (x < 0);
    }
    // The supervisor we are using
    const Supervisor& sup_;

    // The last peak value of qpu that we reached.
    // this is modified by stabilize(), which is a const-correct function, so we
    // define this as mutable.
    mutable  UnactuatedPhase prevPeak_;

}; // class OscillationSupervisor

} // namespace SUGAR

#endif // __SUPERVISOR_H__
/* vim: set tw=80 ts=4 sw=4 sts=0 et ffs=unix : */
