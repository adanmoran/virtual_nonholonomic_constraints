/**
 * AUTHOR: Xingbo Wang, MASc
 *
 * Systems Control Group
 * Department of Electrical and Computer Engineering
 * University of Toronto
 *
 * For use with the SUGAR system
 * Code for the acrobot arduino nano
 */

#ifndef __SUGAR_BOT_H__
#define __SUGAR_BOT_H__

typedef struct Configurations {
  double psi, alpha, dpsi, dalpha, E;
} Configuration;
Configuration configuration = (Configuration){ .psi = 0, .alpha = 0, .dpsi = 0, .dalpha = 0, .E = 0 };

typedef struct Compensators {
  double s, xi, rho;
} Compensator;
Compensator compensator = (Compensator){ .s = 0, .xi = 0, .rho = 0 };

typedef void (*VHCState)(Configuration, Compensator);

#endif
