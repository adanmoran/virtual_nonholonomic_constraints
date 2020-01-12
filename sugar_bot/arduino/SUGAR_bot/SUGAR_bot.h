/**
 * AUTHOR: Xingbo Wang, MASc and Adan Moran-MacDonald, MASc
 *
 * Systems Control Group
 * Department of Electrical and Computer Engineering
 * University of Toronto
 *
 * For use with the SUGAR system
 * Code for the acrobot arduino nano
 * 
 * Last Modified: 12 January 2020
 * Last Editor: Adan Moran-MacDonald
 */

#ifndef __SUGAR_BOT_H__
#define __SUGAR_BOT_H__

// Legacy code by Xingbo -----------------------------

// Structure which holds the configuration object
typedef struct Configurations {
  double psi, alpha, dpsi, dalpha, E;
} Configuration;
Configuration configuration = (Configuration){ .psi = 0, .alpha = 0, .dpsi = 0, .dalpha = 0, .E = 0 };

// Structure for the compensator
typedef struct Compensators {
  double s, xi, rho;
} Compensator;
Compensator compensator = (Compensator){ .s = 0, .xi = 0, .rho = 0 };

// Automaton function: VHCState is a function pointer to a function of Configuration and Compensator
typedef void (*VHCState)(Configuration, Compensator);

// VNHC Code by Adan -----------------------------------
// We use classes instead of structs because
// a) it avoids typedefs
// b) we can generate constructors
// c) while structs allow functions, it makes more sense for this to be in a
//
// I may break some rules of OOP here: classes could have public local variables because it is less overhead
// on legacy arduino products to access the variable directly, rather than using getters and setters.

// Store an nxn inverse-inertia matrix for the acrobot, where you pass in the masses, lengths, etc.
// TODO: write a function which computes Minv(q) and returns a 2x2 or a 1x4 array 
class AcrobotInverseInertia
{
public:

private:

};

// Store the phase. We will give it an inverse inertia matrix so that it can compute its own updates.
// TODO: write an update function, which computes the phase given a configuration.
class Phase
{
public:
  Phase(const AcrobotInverseInertia& Minv)
  : qu(0),
    qa(0),
    pu(0),
    pa(0),
    E(0),
    Minv_(Minv)
  {}

  Phase(const AcrobotInverseInertia& Minv, const Configuration& configuration)
  : Phase(Minv)
  {
    updateFromConfiguration(configuration);
  }

  /**
   * Update the phase of the acrobot at the given configuration state
   */
  auto updateFromConfiguration(const Configuration& configuration) -> void
  {
    // Update the configuration
    qu = configuration.psi;
    qa = configuration.alpha;
    E  = configuration.E;
    
    // Update the conjugate of momenta
    //TODO: p = Minv(q)*[dpsi; dalpha]
  }
  
  double qu; // Configuration unactuated variable = psi
  double qa; // Configuration actuated variable   = alpha
  double pu; // Phase unactuated momentum         = e1' * Minv(q) * p
  double pa; // Phase actuated momentum           = e2' * Minv(q) * p
  double E;  // Configuration energy as given by configuration
  
private:
  const AcrobotInverseInertia& Minv_; // The inverse inertia matrix of the acrobot
};

#endif
