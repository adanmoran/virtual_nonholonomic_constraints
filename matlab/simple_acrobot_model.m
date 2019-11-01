%% Simple Acrobot Modeling
% This script generates the model for the acrobot under very simplified
% assumptions: both links are length l, both masses are point masses at the
% tips of mass m. This makes the moments of inertia of both masses equal to
% ml^2.
%
% The goal of this script is two-fold. First, we will generate the model
% symbolically from scratch (i.e. compute KE and PE and get the energy
% function / Lagrangian). Then, we will use the model Xingbo made and input
% the parameters here, thereby showing whether his model is valid or not
% for this case.

% Create the symbols for the dynamics
qu = sym('qu','real'); % angle of the arms with respect to vertical
qa = sym('qa','real'); % angle of the legs with respect to the arms
qud = sym('qud','real'); % angular velocity of the arms
qad = sym('qad','real'); % angular velocity of the legs
q = [qu; qa]; % state positions
qd = [qud; qad]; % state velocities
l = sym('l','positive'); % length of the rods
m = sym('m','positive'); % masses
g = sym('g','positive'); % gravity

% Find the state of each point mass
x1 = l*[sin(qu); -cos(qu)]; % position of arm mass
x1d = diff(x1,qu)*qud; % velocity of the arm mass
x2 = x1 + l*[sin(qu + qa); -cos(qu + qa)]; % position of leg mass
x2d = diff(x2,qu)*qud + diff(x2,qa)*qad; % velocity of the leg mass
    
% Get the kinetic and potential energies
K = simplify(1/2*m*(x1d(1)^2 + x1d(2)^2) + 1/2*m*(x2d(1)^2 + x2d(2)^2));
P = simplify(m*g*l*(1 - cos(qu)) + ...
    m*g*(l*(1 - cos(qu)) + l*(1 - cos(qu + qa))));

% Now compute Xingbo's kinetic and potential energies
mt = m; ml = m;
dt = l; dl = l;
lt = l; ll = l;
Jt = m*l^2; Jl = m*l^2;
M = [ml*dt^2 + 2*ml*cos(qa)*dt*ll + ml*ll^2 + mt*lt^2 + Jl + Jt,  ...
                                    ml*ll^2 + ml*dt*ll*cos(qa) + Jl; ...
        ml*ll^2 + ml*dt*ll*cos(qa) + Jl,   ml*ll^2 + Jl];

XingboK = simplify(1/2*qd'*M*qd);
V = g*(ml*ll*(1 - cos(qu + qa)) + (ml*dt + mt*lt)*(1 - cos(qu)));

% Are they the same?
sameKinetic = isAlways(K == XingboK);
samePotential = isAlways(P == V);
if sameKinetic && samePotential 
    fprintf('They are the same!');
else
    fprintf('Xingbos and our model are NOT the same. They differ in: \n');
    if ~sameKinetic
        fprintf('* kinetic energy\n');
    end
    if ~samePotential
        fprintf('* potential energy\n');
    end
end
 