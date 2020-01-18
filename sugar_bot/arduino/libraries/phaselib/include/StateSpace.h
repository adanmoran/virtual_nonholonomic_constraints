/*******************************************************************************
* File:	        StateSpace.h
*		 
* Author:       Xingbo Wang
* Created:      14/Jan/20
*
* Systems Control Group
* Department of Electrical and Computer Engineering
* University of Toronto
*
* Last Modified: 14/Jan/20
* Last Editor:   Adan Moran-MacDonald
* Description:   For use with the SUGAR system. Code for the acrobot arduino
*                nano.
*******************************************************************************/

#ifndef __STATE_SPACE_H__
#define __STATE_SPACE_H__

// Legacy code by Xingbo -----------------------------

// Structure which holds the configuration object
typedef struct Configurations { double psi, alpha, dpsi, dalpha, E; }
Configuration; 

// Global configuration object
// NOTE: This cannot be in the library or it causes linking issues, put it in
// the Arduino code itself

//Configuration configuration = (Configuration){ 
//    .psi = 0, .alpha = 0, .dpsi = 0, .dalpha = 0, .E = 0 
//};

// Structure for the compensator
typedef struct Compensators { double s, xi, rho; } Compensator; 

// Global compensator object
// NOTE: This cannot be in the library or it causes linking issues, put it in
// the Arduino code itself
// Compensator compensator = (Compensator){ .s = 0, .xi = 0, .rho = 0 };

// Automaton function: VHCState is a function pointer to a function of
// Configuration and Compensator
typedef void (*VHCState)(Configuration, Compensator);

#endif

/* vim:set et sts=0 sw=4 ts=4 tw=80 : */
