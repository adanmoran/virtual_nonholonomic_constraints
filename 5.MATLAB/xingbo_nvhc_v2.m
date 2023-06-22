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

% Symbolic variables for state
syms q1 q2;
q = [q1;q2];
assume(q,'real');

% Symbolic variables for velocity
syms q1d q2d;
assumeAlso(q1d, 'real');
assumeAlso(q2d, 'real');

% Symbolic variables for momentum
syms p1 p2;
p = [p1;p2];
assumeAlso(p,'real');

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
fprintf('Setting up motion model.\n');

% Mass/Inertia matrix and its inverse
M = [ml*Rt^2 + 2*ml*cos(q2)*Rt*ll + ml*ll^2 + mt*lt^2 + Jl + Jt,  ...
                                    ml*ll^2 + ml*Rt*ll*cos(q2) + Jl; ...
        ml*ll^2 + ml*Rt*ll*cos(q2) + Jl,   ml*ll^2 + Jl];
    
Minv = inv(M);
    
% Potential function
V = g*(ml*ll*(1 - cos(q1 + q2)) + (ml*Rt + mt*lt)*(1 - cos(q1)));

% Gradient of V(q)
GVq = [diff(V,q1); diff(V,q2)];

% Coriolis matrix
C = [-2*ml*Rt*ll*sin(q2)*q2d,   -ml*Rt*ll*sin(q2)*q2d; ...
        ml*Rt*ll*sin(q2)*q1d,         0             ];
       
% Control matrix
B = [0;1];
% Left-annihilator of control matrix
Bperp = [1, 0];

% Control input
tau = sym('tau');

% Energy
E = 1/2 * p' * Minv * p + V;

%% Constraints in Hamiltonian Form
fprintf('Creating EOM for the model.\n');

% EOM from the hamiltonian
qd = Minv*p;
% We know dM/dq1 = 0, so when getting pd we only need to compute dM/dq2
dMinvdq2 = diff(Minv,q2);
pd = -[0 ; 1/2 * p' * dMinvdq2 * p] - GVq + B*tau;

% Next, set the constraint
fprintf('Defining the Constraint of alpha with respect to psi.\n');

% The radial constraint depends on rho
% Temporarily set h1 and h2 to 1
h1 = 0.7; h2 = 0.1;
rho = h1^2*q1^2 + h2^2*p1^2;

% The angular constraint depends on xi
xi = atan2(h2*p1, h1*q1);

% The constraint is a combination of constraints valid only on 
% 0 < xi < pi/2.
rho_0 = 1;%sym('rho_0','real'); % Shape parameter of radial constraint
alpha_max = pi/4;%sym('alpha_max','real'); % Maximum of angular constraint

radConstraint = tanh(rho/rho_0)^2;
xiConstraint = alpha_max*exp(1 - 1/(1 - (4*xi/pi - 1)^2));
xiConstraintPW = piecewise(0 < xi < pi/2, xiConstraint,0);
% If you want to use the constraint on the whole domain, use
% xiConstraintPW. Otherwise, use xiConstraint.
% TODO: figure out how to use the xiConstraintPW using pw2fun
constraint = radConstraint * xiConstraint;

% Display the constraint
figure;
fsurf(constraint, [0.0001 pi]);
title('Constraint on 0 < xi < pi/2');
xlabel('q');
ylabel('p');
zlabel('f(q,p)');

% Take the derivatives of the constraint
dhq1 = simplify(diff(constraint,q1));
dhp1 = simplify(diff(constraint,p1));

fprintf('Determining if the constraint is regular.\n');
% A constraint h(q,p) = q2 - f(q1,p1) is regular if dhq*Minv*B is full
% rank.
% Since our constraint is just f(q1,p1), we need to correctly differentiate
% the h(q,p) term by putting a 1 and a -1 term where necessary.
% 
% The constraint is zero if xi is out of a certain range, so check that the
% constraint h(q,p) = q2 = 0 is regular (when [0, 1]*Minv*B is full rank).
dhq = [-dhq1, 1];
if rank(dhq * subs(Minv,q2,constraint) * B) < 1 || ...
   rank(double([0, 1]*subs(Minv,q2,0)*B)) < 1
    fprintf('This constraint is not regular.\n');
    return;
end
fprintf('----> Regular!\n');

fprintf('Determining if the constraint is solvable.\n');
% Is this constraint solvable in p2? Use the criterion, which is that
% ([0 1] Minv - dhq1 * [1 0] * Minv) * [0;1]) is full rank.
% Find the "G(qu,pu)" matrix on the constraint manifold
GNZbar = ([0 1] - dhq1*[1 0])*Minv;
GNZbar = simplify(subs(GNZbar,q2,constraint));
% Now determine if G * [0;1] is full rank (ie nonzero)
GNZ = GNZbar * [0;1];
if rank(GNZ) < 1
    fprintf('This constraint is not solvable.\n');
    return;
end
% Next, determine if we can solve for p2(t) when the constraint is 0.
GZbar = simplify(subs([0, 1]*Minv, q2, 0));
GZ = GZbar * [0;1];
if rank(GZ) < 1
    fprintf('The q2=0 constraint is not solvable.\n');
end
fprintf('----> Solvable!\n');
% Since this is solvable, let's solve for p2 on the constraint manifold
% This is given by pa = -(G*[0;1])^{-1} * (G*[1;0] + dhp1*dVq1)
fprintf('Getting the equation for pa(t) on the constraint manifold.\n');
g = GNZbar * [1;0] * p1;
dVq1 = GVq(1);
p2NZ = subs(-GNZ\(g + dhp1*dVq1), q2, constraint);

% Do the same for p2 on the constraint manifold, when q2(t) = 0. Since
% dhp1 = 0, we just get p2 = -(GZ*[0;1])^{-1} * GZbar*[1;0]
gZ = GZbar * [1;0] * p1;
p2Z = -GZ\gZ;

%% Find the EOM
fprintf('Determining the constrained Equations of Motion.\n');

% Get the constrained EOM for when the constraint is nonzero
qdNZ = subs(qd, [q2; p2], [constraint; p2NZ]);
pdNZ = subs(pd, [q2; p2], [constraint; p2NZ]);

% Get the constrained energy for the holonomic constraint
E_h = subs(E, [q2; p2], [0; p2Z]);

% Get the constrained EOM for when the constraint is 0
qdZ = subs(qd, [q2; p2], [0; p2Z]);
pdZ = subs(pd, [q2; p2], [0; p2Z]);

% Print the EOM
fprintf('Equations of Motion in 0 < xi <= pi/2 are:\n');
fprintf('qd: ');
qdNZ
fprintf('pd: ');
pdNZ

fprintf('Equations of motion in xi < 0 or xi > pi/2 are:\n');
fprintf('qd: ');
qdZ
fprintf('pd: ');
pdZ

%% Compute the streamslice
fprintf('Creating a streamslice grid of the vector field.\n');
gridPartitionQ = -3.2:0.1:3.2; % qu is in ]-pi,pi
gridPartitionP = -20:20;
[gridQ, gridP] = meshgrid(gridPartitionQ, gridPartitionP);
% Determine which points in the grid are in 0 < xi < pi/2
gridXi = atan2(h2.*gridP,h1.*gridQ);
nonZeroLocs = (gridXi > 0) & (gridXi < pi/2);
zeroLocs = ~nonZeroLocs;

% Initialize the vector field
gridQdot = zeros(size(gridQ));
gridPdot = zeros(size(gridQ));

% Get the vector field for the nonzero constraint at q1, p1
qdotNZ(q1,p1) = qdNZ(1);
pdotNZ(q1,p1) = pdNZ(1);

% Get the vector field for the zero-constraint
qdotZ(q1,p1) = qdZ(1);
pdotZ(q1,p1) = pdZ(1);

% Now sub in the gradient on the grid
gridQdot(nonZeroLocs) = qdotNZ(gridQ(nonZeroLocs), gridP(nonZeroLocs));
gridPdot(nonZeroLocs) = pdotNZ(gridQ(nonZeroLocs), gridP(nonZeroLocs));
gridQdot(zeroLocs) = qdotZ(gridQ(zeroLocs), gridP(zeroLocs));
gridPdot(zeroLocs) = pdotZ(gridQ(zeroLocs), gridP(zeroLocs));

% Now perform a streamslice
figure;
streamslice(gridQ,gridP,gridQdot,gridPdot);
title('Streamlines for acrobot under q_a = f(q_u,p_u)');
xlabel('q_u');
ylabel('p_u');

%% Check the Expansion Characteristics
% Get the dynamics of the constrained and unconstrained regions
dynH = simplify([qdZ(1);pdZ(1)]);
dynNH = simplify([qdNZ(1);pdNZ(1)]);

% TODO Convert the constraint dynamics to a piecewise function handle for
% expansion_vnhc to work
% dynNHpw = pw2fun(dynNH,0 < xi < pi/2);

% TODO remove this in favour of dynNHpw. 
% Convert dynNH and dynH into functions
dynHfun = sym2fun(dynH);
dynNHfun = sym2fun(dynNH);
% Get a function representing the nonholonomic region, using the holonomic
% dynamics on the boundary.
nhFun = @(q,p) safeNHDynamics(q,p,dynHfun,dynNHfun);

% Now pass this into expansion_vnhc to compute the ode at each step for a
% bunch of starting p-values
% I have determined that it takes at most 1 second for the NH constraint
% used by Xingbo to converge, and that at p > 13.3 it starts performing
% rotations rather than oscillations
expansion_vnhc(dynHfun,nhFun,0.01:0.01:13.3,'duration',2,'options',...
    odeset('RelTol',10^-8,'AbsTol',10^-8));

% Now display the level sets of H(q,p) as well as the solutions to the the
% dynNH ODE, to see how the orbits of the nonholonomic constraint travel
% through level sets of H.
figure;
hold on
grid on;
for p = [9 9.5 10]
sol = ode45(@(t,x)nhFun(x(1),x(2)),[0,2],[0;p],...
            odeset('Events',@pZeroEvents,'RelTol',10^-8,'AbsTol',10^-8));
plot(sol.y(1,:),sol.y(2,:),'--');
end
fc = fcontour(@(q,p)hamiltonian_vhc(q,p,dynHfun),[0 2 0 11]);
fc.LevelList = linspace(40,52,13);
xlim([0 1.75]); ylim([0 10.5]);
xlabel('q');
ylabel('p');
title('Orbits of NH System vs. level sets of H');
legend('','','Orbits of dynNH','Level sets of H');

%% Conclusion
% Define a variable that can be used by other scripts to guarantee this has
% only been run once. Think of it as the top of what would be in a C++ .h
% file.
XINGBO_NVHC_V2 = 1;
