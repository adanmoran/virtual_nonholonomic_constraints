/*******************************************************************************
* File:	        vnhc.h
*		 
* Author:       Adan Moran-MacDonald
* Created:      24/Jan/20
*
* Systems Control Group
* Department of Electrical and Computer Engineering
* University of Toronto
*
* Last Modified: 24/Jan/20
* Last Editor:   Adan Moran-MacDonald
* Description:   A class for running a vnhc
*******************************************************************************/
#ifndef VNHC_H
#define VNHC_H

#include "AcrobotDynamics.h"
#include "Phase.h"

namespace SUGAR
{

/**
 * @brief: Define an object which defines an actuator limit for VNHCs
 */
using ActuatorLimit = double;

/**
 * @brief: Define an object which defines a scaling factor  for VNHCs
 */
using ScalingFactor = double;

/**
 * @brief: An abstract function which allows for virtual nonholonomic
 * constraints for the acrobot of the type qa = f(qu,pu)
 */
class AcrobotVNHC
{
public:
    /**
     * @brief: consructor which generates a VNHC for the given acrobot with
     * actuator limit qa in [-pi, pi].
     */
	AcrobotVNHC(const Acrobot& acrobot);

    /**
     * @brief: Constructor which generates a VNHC for the given acrobot. It take
     * in an actuator limit, which is limited to be within the range [0, pi].
     * The actuator will be able to go within [-qmax, qmax]. 
     *
     * @param: const Acrobot&
     * @param: ActuatorLimit
     */
    AcrobotVNHC(const Acrobot& acrobot, ActuatorLimit qmax);
	virtual ~AcrobotVNHC();

	/**
	 * @brief Abstract function which holds the VNHC, qa = f(qu,pu).
	 * This is to be defined properly by any classes which implement
	 * this.
	 *
	 * @param: const UnactuatedPhase& the input to f(qu,pu)
	 * @return: double the value qa
	 */
	virtual auto qa(const UnactuatedPhase& qpu) const -> double = 0;

    /**
    * @brief: Computes the actuated momentum on the constraint manifold. This is
    * concrete because it will never change, as it is fully defined by the other
    * virtual functions.
    *
    * @param: const UnactuatedPhase& qpu
    * @return: double pa the actuated momentum
    */
    auto pa(const UnactuatedPhase& qpu) const -> double;

    /**
    * @brief: Abstract function which, upon implementation, must compute the
    * value dh/dqu = -df/dqu.
    *
    * @param: const UnactuatedPhase&
    *
    * @return: double dh/dqu
    */
    virtual auto dqu(const UnactuatedPhase& qpu) const -> double = 0;

    /**
    * @brief: Computes the value dh/dqa, which for this type of constraint is
    * always equal to 1 for all inputs.
    *
    * @param: const UnactuatedPhase& qpu
    *
    * @return: double dh/dqa
    */
    auto dqa(const UnactuatedPhase& qpu) const -> double;

    /**
    * @brief: Abstract function which, upon implementation, must compute the
    * value dh/dpu = -df/dpu.
    *
    * @param: const UnactuatedPhase& qpu
    *
    * @return: double dh/dpu
    */
    virtual auto dpu(const UnactuatedPhase& qpu) const -> double = 0;

    /**
     * @brief: Computes the value dh/dpa, which for this type of constraint is
     * always equal to 0 for all inputs.
     *
     * @param: const UnactuatedPhase& qpu
     * 
     * @return: double dh/dpa
     */
    auto dpa(const UnactuatedPhase& qpu) const -> double;

protected:

    /**
    * @brief: Get the acrobot used by this vnhc
    *
    * @return: const Acrobot&
    */
    inline auto acrobot() const -> const Acrobot& { return acrobot_; }

    /**
     * @brief: Get the actuator limit for this vnhc
     *
     * @return: ActuatorLimit
     */
    inline auto qmax() const -> ActuatorLimit { return qmax_; }

private:
    const Acrobot& acrobot_;

    ActuatorLimit qmax_;

}; // class AcrobotVNHC

/**
 * @brief: A class for the acrobot VNHC of the type qa = tanh(pu), which is a
 * regular VNHC that successfully injects energy into the acrobot in simulation.
 */
class TanhVNHC : public AcrobotVNHC
{
public:
    /**
     * @brief: Constructs a VNHC of the type qa = qmax*tanh(pu). Note that qmax
     * must be wtihin [0, pi]. If not, it is truncated to be within that
     * value.
     *
     * @param: const Acrobot&
     * @param: ActuatorLimit
     */
    TanhVNHC(const Acrobot& acrobot, ActuatorLimit qmax);

    /**
     * @brief: Constructs a VNHC of the type qa = qmax*tanh(scale*pu). Note that
     * qmax must be within [0, pi]. If not, it is truncated to be within that
     * value.
     *
     * @param: const Acrobot&
     * @param: ActuatorLimit
     * @param: ScalingFactor
     */
    TanhVNHC(const Acrobot& acrobot, ActuatorLimit qmax, ScalingFactor scale);

    ~TanhVNHC();

    /**
     * @brief: Implementation of AcrobotVNHC's qa function, which gives qa =
     * tanh(pu)
     *
     * @param: const UnactuatedPhase& 
     * @return: double
     */
    auto qa(const UnactuatedPhase& qpu) const -> double;

    /**
    * @brief: Implementation of AcrobotVNHC's dqu function, whch gives 
    * dh/dqu = 0 since d/dqu tanh(pu) = 0.
    *
    * @param: const UnactuatedPhase&
    * @return: double
    */
    auto dqu(const UnactuatedPhase& qpu) const -> double;

    /**
    * @brief: Implementation of AcrobotVNHC's dpu function, which gives dh/dpu =
    * -d/dpu tanh(pu)
    *
    * @param: const UnactuatedPhase& qpu
    * @return: double 
    */
    auto dpu(const UnactuatedPhase& qpu) const -> double;

private:
    // Store the scaling factor
    ScalingFactor scale_;

}; // class TanhVNHC

/**
 * @brief: A class for the acrobot VNHC of the type qa = sin(atan2(pu,qu)),
 * which is NOT a regular VNHC yet it still injects energy into the acrobot in
 * simulation (and it does it quite well).
 */
class SinuVNHC : public AcrobotVNHC
{
public:
    /**
     * @brief: Construct a VNHC of the type qmax*sin(atan2(pu,qu)). The actuator
     * limit qmax must be within [0, pi]. If not, it will be truncated to
     * within that range.
     *
     * @param: const Acrobot&
     * @param: ActuatorLimit
     */
    SinuVNHC(const Acrobot& acrobot, ActuatorLimit qmax);
    ~SinuVNHC();

    /**
     * @brief: Implementation of AcrobotVNHC's qa function, which gives qa =
     * sin(atan2(pu,qu))
     *
     * @param: const UnactuatedPhase& 
     * @return: double
     */
    auto qa(const UnactuatedPhase& qpu) const -> double;

    /**
    * @brief: Implementation of AcrobotVNHC's dqu function, whch gives 
    * dh/dqu = - d/dqu sin(atan2(pu,qu))
    *
    * @param: const UnactuatedPhase&
    * @return: double
    */
    auto dqu(const UnactuatedPhase& qpu) const -> double;

    /**
    * @brief: Implementation of AcrobotVNHC's dpu function, which gives dh/dpu =
    * -d/dpu sin(atan2(pu,qu))
    *
    * @param: const UnactuatedPhase& qpu
    * @return: double 
    */
    auto dpu(const UnactuatedPhase& qpu) const -> double;

private:
    /**
    * @brief: Returns the norm of the unactuated phase, sqrt(qu^2_pu^2)
    *
    * @param: const UnactuatedPhase& qpu
    *
    * @return: double
    */
    auto norm(const UnactuatedPhase& qpu) const -> double;

}; // class SinuVNHC

} // namespace SUGAR
#endif /* VNHC_H */
/* vim: set tw=80 ts=4 sw=4 sts=0 et ffs=unix : */
