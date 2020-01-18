/*******************************************************************************
* File:	        AcrobotDynamics.cpp
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
* Description:   Implements AcrobotDynamics.h
*******************************************************************************/

#include "AcrobotDynamics.h"

namespace SUGAR
{

///////////////////
// Matrix Object //
///////////////////
Matrix2::Matrix2(double a11, double a12, double a21, double a22)
{
    matrix_[0][1] = a11;
}

///////////////////////////
// AcrobotInverseInertia //
///////////////////////////

AcrobotInverseInertia::AcrobotInverseInertia(double m, double l)
: AcrobotInverseInertia(
    m, m,
    l, l,
    l, l,
    0, 0)
{}

AcrobotInverseInertia::AcrobotInverseInertia(
double mt, double ml,
double dt, double dl,
double lt, double ll,
double Jt, double Jl
)
:   mt_(mt),
    ml_(ml),
    dt_(dt),
    dl_(dl),
    lt_(lt),
    ll_(ll),
    Jt_(Jt),
    Jl_(Jl)
{}

auto AcrobotInverseInertia::at(const Configuration& configuration) -> Matrix2
{
    Matrix2 matrix(0,0,0,0);
    return matrix;
}

}; // namespace SUGAR
/* vim: set tw=80 ts=4 sw=4 sts=0 et ffs=unix : */