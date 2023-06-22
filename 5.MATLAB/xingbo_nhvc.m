%% Nonholonomic Constraints (Xingbo Thesis)
% This script will model the nonholonomic constraints from Xingbo Wang's
% thesis using the approach from Grizzle,Mohammadi (2019).
% We will convert the model of a bar phase acrobot into Hamiltonian form,
% and put a constraint on the state and momentum of the unactuated
% variable.

clear all;
close all;
clc;
fprintf('---------------------------------\n');
fprintf(' Virtual Nonholonomic Constraint \n');
fprintf('---------------------------------\n');
%% Step 1: Set up the symbolic variables
fprintf('Initializing Symbolic Expressions.\n');
% Time variable
t = sym('t', 'real');

% State variable with q1 = psi, q2 = alpha
syms q1(t) q2(t);
% Ensure they are treated as real-valued functions. For some reason, making
% the 'real' assumption only sets conj(q) = q, but does not do anything to
% set real(q) = q and imag(q) = 0. We need to manually set the assumption
% ourselves.
assume(q1(t),'real'); assumeAlso(q2(t),'real');
assumeAlso(real(q1(t))==q1(t)); assumeAlso(imag(q1(t)) == 0);
assumeAlso(real(q2(t))==q2(t)); assumeAlso(imag(q2(t)) == 0);
% Stack them as a vector for later.
q = [q1;q2];

% State velocities q1d = psi_dot, q2d = alpha_dot
syms q1d(t) q2d(t);
% Ensure they are treated as R-functions
assumeAlso(q1d(t), 'real'); assumeAlso(q2d(t),'real');
assumeAlso(real(q1d(t))==q1d(t)); assumeAlso(imag(q1d(t)) == 0);
assumeAlso(real(q2d(t))==q2d(t)); assumeAlso(imag(q2d(t)) == 0);
% Make an assumption that these are the derivative of q
assumeAlso(diff(q1(t),t)==q1d(t)); assumeAlso(diff(q2(t),t) == q2d(t));
% Stack them into a vector for later
qd = [q1d;q2d];

% Other relevant variables
mt = 1;%sym('m_t','real'); % mass of torso link
ml = 1;%sym('m_l','real'); % mass of leg link
Jt = 0;%sym('J_t','real'); % Moment of inertia of the torso link
Jl = 0;%sym('J_l','real'); % Moment of inertia of the leg link
lt = 0.5;%sym('l_t','real'); % Distance of the hand to the torso center of mass (COM)
ll = 0.5;%sym('l_l','real'); % Distance of the hip to the leg COM
Rt = 1;%sym('R_t','real'); % Length of the torso link
Rl = 1;%sym('R_l','real'); % Length of the leg link
g = 9.8;%sym('g',   'real'); % Gravitational constant

%% Step 2: Define the Model
fprintf('Setting up equations of motion in Lagrangian Form.\n');

% Mass/Inertia matrix and its inverse
M = [ml*Rt^2 + 2*ml*cos(q2)*Rt*ll + ml*ll^2 + mt*lt^2 + Jl + Jt,  ...
                                    ml*ll^2 + ml*Rt*ll*cos(q2) + Jl; ...
        ml*ll^2 + ml*Rt*ll*cos(q2) + Jl,   ml*ll^2 + Jl];
    
Minv = inv(M);
    
% Potential function
V = g*(ml*ll*(1 - cos(q1 + q2)) + (ml*Rt + mt*lt)*(1 - cos(q1)));

% Gradient of V(q)
% We cannot take derivatives with respect to functions of time, so I'll sub
% in constant coordinate functions to take the derivative with respect to
% those, then sub time back in.
syms q1Const q2Const
VofQ = subs(subs(V,q1,q1Const),q2,q2Const);
GVofQq = [diff(VofQ,q1Const); diff(VofQ,q2Const)]; % Gradient for const q
GVq = subs(subs(GVofQq, q1Const, q1),q2Const, q2);

% Coriolis matrix
C = [-2*ml*Rt*ll*sin(q2)*q2d,   -ml*Rt*ll*sin(q2)*q2d; ...
        ml*Rt*ll*sin(q2)*q1d,         0             ];
       
% Control matrix
B = [0;1];
% Left-annihilator of control matrix
Bperp = [1, 0];

% Control input
tau = sym('tau');

%% Constraints in Hamiltonian Form
fprintf('Converting Lagrangian Form to Hamiltonian Form.\n');
% Symbolic version of conjugate of momenta
syms p1(t) p2(t)
% Assume these are R-valued functions
assumeAlso(p1(t),'real'); assumeAlso(p2(t),'real');
assumeAlso(real(p1(t))==p1(t)); assumeAlso(imag(p1(t))==0);
assumeAlso(real(p2(t))==p2(t)); assumeAlso(imag(p2(t))==0);
% Stack them into a vector for later
p = [p1;p2];

% Now make assumptions to set the derivatives of p, which we'll store as
% symbolic variables until we substitute in the above later
syms p1d(t) p2d(t);
assumeAlso(p1d(t),'real'); assumeAlso(p2d(t),'real');
assumeAlso(real(p1d(t))==p1d(t)); assumeAlso(imag(p1d(t))==0);
assumeAlso(real(p2d(t))==p2d(t)); assumeAlso(imag(p2d(t))==0);
assumeAlso(diff(p1(t),t)==p1d(t)); assumeAlso(diff(p2(t),t)==p2d(t));
% Now stack them as a vector for later
pd = [p1d;p2d];

% Set symbolic values for the "constant" version of p1(t), p2(t).
syms p1Const p2Const;

% Now generate the Hamiltonian
fprintf('Creating the EOM from the Hamiltonian.\n');
qdConcrete = Minv*p;
% We know dM/dq1 = 0, so when getting pd we only need to compute dM/dq2
dMinvdq2 = diff_q(Minv,q2);
pdConcrete = -[0 ; 1/2 * p' * dMinvdq2 * p] - GVq + B*tau;

% Next, set the constraint
fprintf('Defining the Constraint of alpha with respect to psi.\n');

% The radial constraint depends on rho
% Temporarily set h1 and h2 to 1
h1 = 0.7; h2 = 0.1;
rho = h1^2*q1^2 + h2^2*p1^2;

% The angular constraint depends on xi
xi = atan2(h2*p1, h1*q1);

% The constraint is a combination of constraints valid only on 
% 0 <= xi <= pi/2.
rho_0 = 1;%sym('rho_0','real'); % Shape parameter of radial constraint
alpha_max = pi/4;%sym('alpha_max','real'); % Maximum of angular constraint
constraint = tanh(rho/rho_0)^2 * alpha_max * ...
    exp(1 - 1/(1 - (4*xi/pi - 1)^2));

fprintf('Determining if the constraint is solvable.\n');
% Is this constraint solvable in p2? Check that
% e1^T * Minv*p - df/(dq1) * e2^T*Minv*p + df/(dp1)*dV/(dq1) = 0
% is solvable in p2(t) on the constraint q2 = constraint(q1,p1)
% First, solve for df/dq1 and df/dp1 and dV/dp1
dfq1 = diff_q(constraint,q1(t));
dfp1 = diff_q(constraint,p1(t));
dVq1 = diff_q(constraint,q1(t));
% Now find out if this constraint satisfies the equation
satisfaction_eqn = [1 0]*Minv*p - dfq1*[0 1]*Minv*p + dfp1*dVq1;
satisfaction_eqn = simplify(subs(satisfaction_eqn,q2(t),constraint));
% Solve for p2(t) on the zero-dynamics manifold q2 = f(q1,p1)
p2OnC0NonZero = solve_for(satisfaction_eqn == 0, p2(t));
if isempty(p2OnC0NonZero)
    fprintf('This constraint is not solvable.');
    return;
end

% Second: Check if the constraint is of relative degree 2 by using 
% dqf(q1,q2)*Minv*B nonzero.
relativeDegreeConstant = simplify(subs([-dfq1 1]*Minv*B,q2(t),constraint));
if relativeDegreeConstant == 0
    fprintf('This constraint is not of relative degree 2.\n');
    return;
end

fprintf('This constraint is relative degree 2 and is solvable! p2(t) is \n');
p2OnC0NonZero

%% Find the EOM
fprintf('Determining the constrained Equations of Motion.\n');
% Determine the equations of motion from the hamiltonian; we need to set
% q2 = constraint in the equations of motion for q1dot,p1dot. 
% We also need p2(t) in closed form.

% pd is a function of q1, q2, p1, and p2. No derivatives.
pdConstraintNonZero = subs(pdConcrete, p2(t), p2OnC0NonZero);
pdConstraintNonZero = subs(pdConstraintNonZero, q2(t), constraint(t));

% qd is a function of p2, and Minv is a function of q2
qdConstraintNonZero = subs(qdConcrete, p2(t), p2OnC0NonZero);
qdConstraintNonZero = subs(qdConstraintNonZero, q2(t), constraint(t));

% Print the EOM
fprintf('Equations of Motion in 0 < xi <= pi/2 are:\n');
fprintf('pd: ');
pdConstraintNonZero
fprintf('qd: ');
% TODO: For some reason qdConcrete depends on p2(t). Should
% p2(t) be put back into qdConcrete as well?
% I'll do that for now, but this is a question for Manfredi
% THIS IS WRONG: We shouldn't sub in pd for p2(t), obviously...
%qdConstraintNonZero = simplify(subs(qdConstraintNonZero,p2(t), pdConstraintNonZero(2)));
qdConstraintNonZero

% Now determine the EOM in areas outside 0 < xi <= pi/2, where q2 = q2d = 0
% In this case, we have p2(t) solved by q2d = [0 1]*Minv*p = 0
c0_satisfaction_eqn = [0 1]*Minv*p;
c0_satisfaction_eqn = simplify(subs(c0_satisfaction_eqn, q2(t), 0));

% P2 on the constraint=0 zero-dynamics manifold
p2OnC0 = subs(solve(...
              subs(c0_satisfaction_eqn, p2(t),p2Const) == 0,p2Const),...
                                                     p2Const, p2(t));
p2OnC0 = simplify(subs(p2OnC0, q2(t), 0));
if isempty(p2OnC0)
    fprintf('The zero-constraint cannot be solved for p2(t).')
    return;
end
% Start with p
pdConstraintZero = subs(pdConcrete, q2(t), 0);
pdConstraintZero = subs(pdConstraintZero, q2d(t), 0);
pdConstraintZero = subs(pdConstraintZero, p2(t), p2OnC0);
pdConstraintZero = simplify(pdConstraintZero);
% Now do q
qdConstraintZero = subs(qdConcrete, q2(t), 0);
qdConstraintZero = subs(qdConstraintZero, q2d(t), 0);
qdConstraintZero = subs(qdConstraintZero, p2(t), p2OnC0);
qdConstraintZero = simplify(qdConstraintZero);

% Print the EOM with the constraint set to 0
fprintf('Equations of Motion for xi not in [0, pi/2] are:\n');
fprintf('pd: ');
pdConstraintZero
fprintf('qd: ');
qdConstraintZero

%% Compute the streamslice
fprintf('Creating a streamslice grid of the vector field.\n');
gridPartitionQ = -3.1:0.1:3.1;
gridPartitionP = -5:0.1:5;
[gridQ, gridP] = meshgrid(gridPartitionQ, gridPartitionP);
% Determine which points in the grid are in 0 < xi < pi/2
gridXi = atan2(h2.*gridP,h1.*gridQ);
nonZeroLocs = (gridXi > 0) & (gridXi < pi/2);
zeroLocs = ~nonZeroLocs;

% Initialize the vector field
gridQdot = zeros(size(gridQ));
gridPdot = zeros(size(gridQ));

% Get the vector field for the nonzero constraint
qdotNZ = qdConstraintNonZero(t);
pdotNZ = pdConstraintNonZero(t);
% We only care about q1 and p1, not their time-dependent values
qdotNZ(q1Const,p1Const) = subs(qdotNZ(1),[q1(t),p1(t)],[q1Const,p1Const]);
pdotNZ(q1Const,p1Const) = subs(pdotNZ(1),[q1(t),p1(t)],[q1Const,p1Const]);

% Get the vector field for the zero-constraint
qdotZ = qdConstraintZero(t);
pdotZ = pdConstraintZero(t);
% Again, we only care about q1 and p1
qdotZ(q1Const,p1Const) = subs(qdotZ(1), [q1(t),p1(t)],[q1Const,p1Const]);
pdotZ(q1Const,p1Const) = subs(pdotZ(1), [q1(t),p1(t)],[q1Const,p1Const]);

% Now sub in the gradient on the grid
gridQdot(nonZeroLocs) = qdotNZ(gridQ(nonZeroLocs), gridP(nonZeroLocs));
gridPdot(nonZeroLocs) = pdotNZ(gridQ(nonZeroLocs), gridP(nonZeroLocs));
gridQdot(zeroLocs) = qdotZ(gridQ(zeroLocs), gridP(zeroLocs));
gridPdot(zeroLocs) = pdotZ(gridQ(zeroLocs), gridP(zeroLocs));

% Now perform a streamslice
streamslice(gridQ,gridP,gridQdot,gridPdot);