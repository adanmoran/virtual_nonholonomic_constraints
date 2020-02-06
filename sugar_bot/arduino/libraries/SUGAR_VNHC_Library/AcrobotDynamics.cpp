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

#include "include/AcrobotDynamics.h"

namespace SUGAR
{

///////////////////
// Matrix Object //
///////////////////
Matrix2::Matrix2(double a11, double a12, double a21, double a22)
{
    matrix_[0][0] = a11;
    matrix_[0][1] = a12;
    matrix_[1][0] = a21;
    matrix_[1][1] = a22;
}

auto Matrix2::at(unsigned int i, unsigned int j) const -> double
{
    if(i <= 0 || i > 2 || j <= 0 || j > 2)
    {
        return NAN;
    }
    // Arrays are 0-based, while our row/col is 1-based
    return matrix_[i-1][j-1];
}

////////////////////
// AcrobotInertia //
////////////////////

AcrobotInertia::AcrobotInertia(double m, double l)
: AcrobotInertia(
    m, m,
    l, l,
    l, l,
    0, 0)
{}

AcrobotInertia::AcrobotInertia(
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
    Jl_(Jl),
    JlJt_(Jl*Jt),
    Jlmldt2_(Jl*ml*dt*dt),
    Jtmlll2_(Jt*ml*ll*ll),
    Jlmtlt2_(Jl*mt*lt*lt),
    ll2lt2mlmt_(ll*ll*lt*lt*ml*mt),
    mlll2_(ml*ll*ll),
    mldt2_(ml*dt*dt),
    mtlt2_(mt*lt*lt),
    mldtll_(ml*dt*ll),
    ml2dt2ll2_(mldtll_*mldtll_),
    mlll2pJl_(mlll2_ + Jl_)
{}

auto AcrobotInertia::at(const Configuration& configuration) const -> Matrix2
{
    double cqa = cos(configuration.alpha);

    // Compute the elements of the matrix at this qa
    // (1,1) = ml*dt^2 + 2*ml*dt*ll*cos(qa) + ml*ll^2 + mt*lt^2 + Jl + Jt
    // (1,2) = (2,1) = ml*ll^2 + ml*dt*ll*cos(qa) + Jl
    // (2,2) = ml*ll^2 + Jl
    double mldtllcqa = mldtll_*cqa;
    
    double a11 = mldt2_ + 2*mldtllcqa + mtlt2_ + mlll2pJl_ + Jt_;
    double a12 = mldtllcqa + mlll2pJl_;
    double a22 = mlll2pJl_;

    return Matrix2(a11,a12,a12,a22);
}

auto AcrobotInertia::inverseAt(const Configuration& configuration) const -> Matrix2
{
    // Get the configuration value for qa, which is all we need
    double qa = configuration.alpha;
    // Now compute cos(qa) to avoid doing it multiple times
    double cqa = cos(qa);
    double cqa2 = cqa*cqa;

    // Next, compute the elements of the matrix at this qa.
    //
    // den = Jl*Jt + (dt*ml*ll)^2 + Jl*ml*dt^2 + Jt*ml*ll^2 + Jl*mt*lt^2 + (ll*lt)^2*ml*mt - (dt*ml*ll*cos(qa))^2
    //
    double den = JlJt_ + ml2dt2ll2_ + Jlmldt2_ + Jtmlll2_ + Jlmtlt2_ + ll2lt2mlmt_ - ml2dt2ll2_*cqa2;

    // Next, multiply the following by 1/den:
    // (1,1) = ml*ll^2 + Jl 
    // (1,2) = (2,1) = -(ml*ll^2) - (dt*ml*ll*cos(qa)) - Jl
    // (2,2) = ml*dt^2 + 2*dt*ml*ll*cos(qa) + ml*ll^2 + mt*lt^2 + Jl + Jt
    double mldtllcqa = mldtll_*cqa;
    double a11 = (mlll2_ + Jl_)/den;
    double a12 = -( mlll2_ + mldtllcqa + Jl_)/den;
    double a22 = (mldt2_ + 2*mldtllcqa + mlll2_ + mtlt2_ + Jl_ + Jt_)/den;

    return Matrix2(a11,a12,a12,a22);
}

//////////////////////
// AcrobotPotential //
//////////////////////

AcrobotPotential::AcrobotPotential(double m, double l, double g)
: AcrobotPotential(
    m,m,
    l,
    l,l,
    g)
{}

AcrobotPotential::AcrobotPotential(
double mt, double ml,
double dt,
double lt, double ll,
double g
)
:   mt_(mt), ml_(ml),
    dt_(dt),
    lt_(lt), ll_(ll),
    g_(g),
    gmlll_(g*ml*ll),
    gmldtpmtlt_(g*(ml*dt + mt*lt))
{}

auto AcrobotPotential::at(const Configuration& configuration) const -> double
{
    auto qu = configuration.psi;
    return gmldtpmtlt_ * (1 - cos(qu)) + gmlll_ * (1 - cos(qu + configuration.alpha));
}

auto AcrobotPotential::dqu(const Configuration& configuration) const -> double
{
    auto qu = configuration.psi;
    return gmldtpmtlt_*sin(qu) + gmlll_*sin(qu + configuration.alpha);
}

/////////////
// Acrobot //
/////////////

Acrobot::Acrobot(
double mt, double ml,
double dt, double dl,
double lt, double ll,
double Jt, double Jl,
double g)
:   M_( mt, ml,
        dt, dl,
        lt, ll,
        Jt, Jl),
    V_( mt, ml,
        dt, 
        lt, ll,
        g)
{}

Acrobot::Acrobot(const AcrobotInertia& M, const AcrobotPotential V)
: M_(M),
  V_(V)
{}

}; // namespace SUGAR
/* vim: set tw=80 ts=4 sw=4 sts=0 et ffs=unix : */