/*******************************************************************************
* File:	        Predicates.h
*		 
* Author:       Adan Moran-MacDonald
* Created:      06/Feb/20
*
* Systems Control Group
* Department of Electrical and Computer Engineering
* University of Toronto
*
* Last Modified: 06/Feb/20
* Last Editor:   Adan Moran-MacDonald
* Description:   Contains custom predicates for gtest
*******************************************************************************/

#ifndef __PREDICATES_H__
#define __PREDICATES_H__

// For abs
#include <cmath>

namespace SUGAR
{
namespace predicates
{

/**
* @brief: Predicate to decide if a and b are within tolerance of each other
*
* @param: double a
*       : double b
*       : double tolerance
*
* @return: bool
*/
bool Within(double a, double b, double tolerance)
{
    return std::abs(a-b) <= tolerance;
}

}; // namespace predicates
}; // namespace SUGAR

#endif

/* vim: set tw=80 ts=4 sw=4 sts=0 et ffs=unix : */
