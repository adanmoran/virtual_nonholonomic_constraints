%% Variable Length Pendulum
% This script simulates the variable length pendulum, so we can try to
% create a VNHC which will inject energy into the system. This VNHC will
% eventually be used to generate a VNHC for the Acrobot.
clear all
close all
clc

% Set up the variables for a VLP with angle q, momentum p,
% length l, change in length l_dot, acceleration l_ddot. 
% Time is labelled by t. We assume mass is 1 and gravity is 9.8 m/s^2.
syms q p l l_dot l_ddot t ;
assume(q,'real');
assumeAlso(p,'real');

m = 1;
g = 9.8;
% Get the kinetic and potential energy, where the 0 potential is at the
% height of the rotation point for the pendulum.
KE1 = 1/(2*m*l^2)*p^2;
KE2 = 1/2*m*l_dot^2;
KE = KE1 + KE2;
V = -m*g*l*cos(q);

% The Hamiltonian (which is NOT equal to the energy)
H = KE1 - KE2 + V;
% The equations of motion of the pendulum
qd = diff(H,p);
pd = -diff(H,q);
eom = [qd;pd];

% The energy of the pendulum
E = KE + V;

% Get the derivative of E with respect to time.
E_dot = [diff(E,q) diff(E,p) diff(E,l) diff(E,l_dot)]*[eom;l_dot;l_ddot];

% Suppose the pendulum has maximum length d, and minimum length d/2. Then
% we create a constraint l(q) = (lmax - lmin)e^{-q} + lmin, which has
% negative first derivative (wrt time) and positive second derivative (wrt
% time). This implies that E_dot under this known l should be
% positive on {q >= 0, p >= 0}
d = 1;
lmax = d;
lmin = d/2;
lofq = (lmax - lmin)*exp(-q) + lmin;
% Get its derivatives to sub into E and E_dot
lofq_dot = diff(lofq,q)*qd;
lofq_ddot = [diff(lofq_dot,q) diff(lofq_dot,p) diff(lofq_dot,l)]* ...
            [eom;lofq_dot];
        
% Convert these symbolic equations to functions
lofqFun = matlabFunction(lofq,'vars',[q,p]);
lofq_dotFun = matlabFunction(subs(lofq_dot,l,lofq),'vars',[q,p]);
lofq_ddotFun = matlabFunction(subs(lofq_ddot,l,lofq),'vars',[q,p]);
% Find meshes of l(q) and E(q,p,l(q))
[qMesh,pMesh] = ndgrid(linspace(0,pi,1000),linspace(0,5,1000));
lMesh = lofqFun(qMesh,pMesh);
ldMesh = lofq_dotFun(qMesh,pMesh);
lddMesh = lofq_ddotFun(qMesh,pMesh);

E_with_lofqdot = subs(E,l_dot,lofq_dot);
E_with_lofq = subs(E_with_lofqdot,l,lofq);
E_with_lofqFun = matlabFunction(E_with_lofq,'vars',[q,p]);
E_with_lofqMesh = E_with_lofqFun(qMesh,pMesh);
% Display them
lmeshfig = figure;
mesh(qMesh,pMesh,lMesh);
xlabel('q');
ylabel('p');
zlabel('l(q)');
title('VHC l(q) = (lmax-lmin)e^{-q} + lmin');

Elofqmeshfig = figure;
mesh(qMesh,pMesh,E_with_lofqMesh);
xlabel('q');
ylabel('p');
zlabel('l(q)');
title('Energy of the VLP under l(q) = a*e^{-q}+b');

% Find the dynamics under the input l(q)
eom_with_lofq = subs(eom,l,lofq);
eomFun = matlabFunction(eom_with_lofq,'vars',[q,p]);
% Now get the solution along some orbits starting at (p,0);
orbitsfig = figure;
hold on;
grid on;
pRange = 1:0.5:3;
solutions = cell(size(pRange));
for i = 1:length(pRange)
    pk = pRange(i);
    sol = ode45(@(t,x)eomFun(x(1),x(2)),[0,2],[0;pk],...
    odeset('Events',@pZeroEvents,'RelTol',10^-13,'AbsTol',10^-13));
    plot(sol.y(1,:),sol.y(2,:),'--');
    solutions{i} = sol;
end
contourRegion = [0 2.2 0 3];
fc = fcontour(E_with_lofqFun, contourRegion);
fc.LevelList = E_with_lofqFun(zeros(size(pRange)),pRange);

% Now plot E_dot along solutions to one of the orbits to see what happens.
E_dotlofq = subs(E_dot,l_ddot,lofq_ddot);
E_dotlofq = subs(E_dotlofq,l_dot,lofq_dot);
E_dotlofq = subs(E_dotlofq,l,lofq);
E_dotlofqFun = matlabFunction(E_dotlofq,'vars',[q,p]);

for i = 1:length(solutions)
    sol = solutions{i};
    figure;
    E_dotAlongOrbit = E_dotlofqFun(sol.y(1,:),sol.y(2,:));
    plot(sol.x, E_dotAlongOrbit);
    xlabel('t');
    ylabel('E_dot(t)');
    title(sprintf('E_dot under l(q) starting at (0,%.2f)',sol.y(2,1)));
    grid on;
end

% Try l(theta) being a smoothed version of the following:
% l(x) =     {1          if x < -pi/2 or x > pi/2,
%            -x/pi + 1/2 if -pi/2 <= x <= 0,
%             x/pi + 1/2 if 0 <= x <= pi/2 }
% Smoothing it into a gaussian becomes
% l(x) = -1/2*exp(- x^2 / (pi/4)^2 ) + 1

% Define theta, the angle from the q-axis in phase space
theta = atan2(p,q);

% Define the smoothed l(theta) constraint
ltheta = -1/2*exp(- (theta^2)/(pi/4)^2) + 1;
lthetad = [diff(ltheta,q) diff(ltheta,p)]*eom;
lthetad = subs(lthetad,l,ltheta);
lthetadd = [diff(lthetad,q) diff(lthetad,p)]*eom;

% Plot the l(theta) constraint
lthetaFun = matlabFunction(ltheta,'vars',[q p]);
lthetameshfig = figure;
[qGrid, pGrid] = ndgrid(linspace(-pi,pi,1000),linspace(-5,5,1000));
lthetaMesh = lthetaFun(qGrid,pGrid);
mesh(qGrid,pGrid,lthetaMesh);
xlabel('q');
ylabel('p');
zlabel('l(theta)');
title('Mesh of l(theta) a negative gaussian');

% Compute E_dot under l(theta) and see what it does
E_dotltheta = subs(E_dot,l_ddot,lthetadd);
E_dotltheta = subs(E_dotltheta,l_dot,lthetad);
E_dotltheta = subs(E_dotltheta,l,ltheta);
E_dotlthetaFun = matlabFunction(E_dotltheta,'vars',[q,p]);

E_dotlthetameshfig = figure;
E_dotlthetamesh = E_dotlthetaFun(qMesh,pMesh);
mesh(qMesh,pMesh,E_dotlthetamesh);
xlabel('q');
ylabel('p');
zlabel('E_dot');
title('Mesh of E_dot under l(theta)');

eom_theta = subs(eom,l,ltheta);
eom_thetaFun = matlabFunction(eom_theta,'vars',[q p]);
hold on;
grid on;
ltheta_solutions = cell(size(pRange));
for i = 1:length(pRange)
    pk = pRange(i);
    sol = ode45(@(t,x)eom_thetaFun(x(1),x(2)),[0,2],[0;pk],...
    odeset('Events',@pZeroEvents,'RelTol',10^-13,'AbsTol',10^-13));
    figure;
    E_dotAlongOrbit = E_dotlthetaFun(sol.y(1,:),sol.y(2,:));
    plot(sol.x, E_dotAlongOrbit);
    xlabel('t');
    ylabel('E_dot(t)');
    title(sprintf('E_dot under l(theta) starting at (0,%.2f)',sol.y(2,1)));
    grid on;
end