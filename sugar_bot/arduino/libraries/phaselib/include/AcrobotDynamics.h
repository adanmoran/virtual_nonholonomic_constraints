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
    auto at(unsigned int i, unsigned int j) -> double;

private:
    double matrix_[2][2] = {{0,0},{0,0}};
}; // class Matrix

/**
 * class AcrobotInverseInertia
 *
 * @brief: Stores the inertia matrix for an acrobot in the most general form. It
 * is given the masses, lengths, etc. and provides functions to compute this
 * inertia matrix at a given configuration.
 */
class AcrobotInverseInertia
{
public:
    /**
     * @brief: Constructor for a simple acrobot with masses at the end of
     * equally-sized links.
     *
     * @param: double m = mass of each link, kg
     * @param: double l = length of each link, m
     */
    AcrobotInverseInertia(double m, double l);

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
    AcrobotInverseInertia(
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
    auto at(const Configuration& configuration) -> Matrix2;
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
}; // class AcrobotInverseInertia

}; // namespace SUGAR

#endif

/* vim: set tw=80 ts=4 sw=4 sts=0 et ffs=unix : */
