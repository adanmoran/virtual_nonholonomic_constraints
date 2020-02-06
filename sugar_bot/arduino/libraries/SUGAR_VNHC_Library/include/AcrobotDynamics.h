/*******************************************************************************
* File:	        AcrobotDynamics.h
*		 
* Author:       Adan Moran-MacDonald
* Created:      17/Jan/20
*
* Systems Control Group
* Department of Electrical and Computer Engineering
* University of Toronto
*
* Last Modified: 17/Jan/20
* Last Editor:   Adan Moran-MacDonald
* Description:   A collection of objects which help with acrobot dynamics
* computations
*******************************************************************************/

#ifndef __ACROBOTDYNAMICS_H__
#define __ACROBOTDYNAMICS_H__

#include "StateSpace.h"
#include <math.h> // this is allowed in Arduino for NAN

namespace SUGAR
{

/**
 * class Matrix
 *
 * @brief: Stores a read_only 2x2 square matrix  of doubles
 *
 * Adan: I don't feel like making this a template properly, so to save time I'm
 * just going to write this very simply
 */
class Matrix2
{
public:
    /**
     * @brief: Constructor for a 2x2 matrix
     *
     * @param: double a11 = element (1,1)
     * @param: double a12 = element (1,2)
     * @param: double a21 = element (2,1)
     * @param: double a22 = element (2,2)
     */
    Matrix2(double a11, double a12, double a21, double a22);

    /**
    * @brief: Get the element at row i, column j for {i,j} in {1,2}.
    *
    * @param: uint8_t i the row of the element, in {1,2}
    *       : uint8_t j the coumn of the element, in {1,2}
    *
    * @return: double The element value at (i,j). Returns NAN if (i,j) out of
    * range.
    */
    auto at(unsigned int i, unsigned int j) const -> double;

private:
    double matrix_[2][2] = {{0,0},{0,0}};
}; // class Matrix


/**
 * class AcrobotInertia
 *
 * @brief: Stores the inverse inertia matrix for an acrobot.
 */
class AcrobotInertia
{
public:
    /**
     * @brief: Constructor for a simple acrobot with masses at the end of
     * equally-sized links.
     *
     * @param: double m = mass of each link, kg
     * @param: double l = length of each link, m
     */
    AcrobotInertia(double m, double l);

    /**
     * @brief: Constructor for a general acrobot
     *
     * @param: double mt = mass of torso link, kg
     * @param: double ml = mass of leg link, kg
     * @param: double dt = length of torso link, m
     * @param: double dl = length of leg link, m
     * @param: double lt = distance from hand to mt, m
     * @param: double ll = distance from hip to ml, m
     * @param: double Jt = absolute moment of inertia of torso link, kg*m^2
     * @param: double Jl = absolute moment of inertia of leg link, kg*m^2
     */
    AcrobotInertia(
            double mt, double ml,
            double dt, double dl,
            double lt, double ll,
            double Jt, double Jl);

    /**
     * @brief: Compute the inertia matrix at a given configuration
     *
     * @param: Configuration configuration
     *
     * @return: Matrix2 a 2x2 Matrix object with the value of the inertia matrix at
     * that configuration.
     */
    auto at(const Configuration& configuration) const -> Matrix2;

    /**
    * @brief: Compute the inverse inertia matrix at a given configruation
    *
    * @param: const Configuration& configuration
    *
    * @return: auto
    */
    auto inverseAt(const Configuration& configuration) const -> Matrix2;

    /**
     * @brief: Get the torso mass in kg
     *
     * @return: double
     */
    inline auto mt() const -> double { return mt_; }
    /**
     * @brief: Get the leg mass in kg
     *
     * @return: double
     */
    inline auto ml() const -> double { return ml_; }
    /**
     * @brief: Get the torso length in m
     *
     * @return: double
     */
    inline auto dt() const -> double { return dt_; }
    /**
     * @brief: Get the leg length in m
     *
     * @return: double
     */
    inline auto dl() const -> double { return dl_; }
    /**
     * @brief: Get the distance from hands to torso COM in m
     *
     * @return: double
     */
    inline auto lt() const -> double { return lt_; }
    /**
     * @brief: Get the distance from hips to leg COM in m
     *
     * @return: double
     */
    inline auto ll() const -> double { return ll_; }
     /**
     * @brief: Get the absolute moment of inertia of the torso in kg*m^2
     *
     * @return: double
     */
    inline auto Jt() const -> double { return Jt_; }
     /**
     * @brief: Get the absolute moment of inertia of the legs in kg*m^2
     *
     * @return: double
     */
    inline auto Jl() const -> double { return Jl_; }
          
private:
    // Masses in kg
    double mt_;
    double ml_;
    // Lengths of links in m
    double dt_;
    double dl_;
    // Distance of masses from base of links, in m
    double lt_;
    double ll_;
    // Moments of inertia in kg*m^2
    double Jt_;
    double Jl_;

    // Constants for the computation of the inverse inertia matrix
    double JlJt_; // Jl * Jt
    double Jlmldt2_; // Jl * ml * dt^2
    double Jtmlll2_; // Jt * ml * ll^2
    double Jlmtlt2_; // Jl * mt * lt^2
    double ll2lt2mlmt_; // (ll*lt)^2 * ml * mt
    double mlll2_; // ml * ll^2
    double mldt2_; // ml * dt^2
    double mtlt2_; // mt * lt^2
    double mldtll_; // ml * dt * ll
    double ml2dt2ll2_; // (mldtll_)^2
    double mlll2pJl_; // ml*ll^2 + Jl
    
}; // class AcrobotInertia

/**
 * class AcrobotPotential 
 *
 * @brief: A class which stores the acrobot's potential energy and can compute
 * it at a given State or Phase. It can also give the derivative of the inertia
 * with respect to its unactuated variable
 */
class AcrobotPotential
{
public:

    /**
     * @brief: Constructs an acrobot's potential function for a simple acrobot
     *
     * @param: double m = mass of each link, kg
     * @param: double l = length of each link, m
     * @param: double g = gravitational acceleration, m/(s^2)
     */
    AcrobotPotential(double m, double l, double g);
    /**
     * @brief: Constructs an acrobot's potential function
     *
     * @param: double mt = mass of torso link, kg
     * @param: double ml = mass of leg link, kg
     * @param: double dt = length of torso link, m
     * @param: double lt = distance from hand to mt, m
     * @param: double ll = distance from hip to ml, m
     * @param: double g = gravitational acceleration m/(s^2)
     */
    AcrobotPotential(
            double mt, double ml, 
            double dt,            
            double lt, double ll,
            double g
    );

    /**
    * @brief: compute the potential at the given state value
    *
    * @param: const Configuration& configuration
    *
    * @return: double The potential V(q)
    */
    auto at(const Configuration& configuration) const -> double;

    /**
    * @brief: Computes the derivative of the potential function with respect to
    * the unactuated variable qu, at the configuration
    *
    * @param: const Configuration& configuration
    *
    * @return: double The partial derivative d/dqu V(q)
    */
    auto dqu(const Configuration& configuration) const -> double;

    /**
     * @brief: returns the force of gravity used by this potential function
     *
     * @return: double g
     */
    auto g() const -> double { return g_; }

private:

    // Masses in kg
    double mt_;
    double ml_;
    // Lengths of links in m
    double dt_;
    // Distance of masses from base of links, in m
    double lt_;
    double ll_;
    // Gravitational acceleration, in m/(s^2)
    double g_;

    // Constants for the computation of the potential
    double gmlll_; //g*(ml_*ll_)
    double gmldtpmtlt_; //g*(ml_*dt_ + mt_*lt_)

}; // class AcrobotPotential

/**
 * @brief: A class which holds all the acrobot dynamics for a system. It stores
 * the inertia matrix along with the potential.
 */
class Acrobot
{
public:
    /**
     * @brief: Consructor. Given masses and gravity, this generates an acrobot
     * with the correct inertia and potential.
     */
    Acrobot(
        double mt, double ml,
        double dt, double dl,
        double lt, double ll,
        double Jt, double Jl,
        double g);

    /**
     * @brief: Constructor which holds the inputted mass and potential objects.
     */
    Acrobot(const AcrobotInertia& M, const AcrobotPotential V);

    /**
     * @brief: Get the acrobot inertia matrix
     *
     * @return: const AcrobotInertia&
     */
    auto M() const -> const AcrobotInertia& { return M_; }
    
    /**
     * @brief: Get the acrobot potential object
     *
     * @return: const AcrobotPotential&
     */
    auto V() const -> const AcrobotPotential& { return V_; }

private:
    AcrobotInertia M_;
    AcrobotPotential V_;

}; // class Acrobot

}; // namespace SUGAR

#endif

/* vim: set tw=80 ts=4 sw=4 sts=0 et ffs=unix : */
