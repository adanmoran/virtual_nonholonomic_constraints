%% Net Energy Gain Control of the Variable-Length Pendulum
% 26 June 2019
% 
% The variable-length pendulum can gain energy by varying the length
% properly. We have discovered that a feedback control law l(q) =
% delta*exp(-lambda*q) + a on the {q>=0,p>=0} region results in a gain in
% energy through the whole region (i.e. E_dot >= 0). However, once this
% l(q) controller is fixed, there is no other controller we can find which
% will guarantee continuity of l, hybrid invariance, and E_dot >= 0 on the
% rest of the phase space.
%
% As such, we will do the following:
% a) Apply l(q) = delta*exp(-lamba*q) + a on the {q >= 0} region
% b) Apply l(q) = delta*exp(-lambda*|q|) + a on the {p >= 0} region
% c) Apply l(q,p) = {alternating between l_min and l_max based on angle},
% which was shown to be the time-optimal way to inject energy into a
% standing swing.
%
% (b) is guaranteed to have E_dot < 0 for some q < 0,
% while (a) will have negative E_dot when p < 0. The hope is that the
% energy lost in these regions is less than that gained in the {q>=0, p>=0}
% region, resulting in a net gain in mechanical energy.
%
% RESULTS:
% Neither (a) nor (b) inject energy into the system overall.
close all;
clc;

% Choose which results to plot.
results.run_a = false;
results.run_b = false;
results.run_c = true;

% Define the energy of the VLP
m = sym('m','positive'); % mass
g = sym('g','positive'); % gravitational force
lmax = sym('lmax','positive'); % maximum length of the vlp
lmin = sym('lmin','positive'); % minimum length of the vlp
q = sym('q','real'); % angle of the vlp
p = sym('p','real'); % conjugate of momenta for q
l = sym('l','positive'); % length of the vlp, which will become a controlled value
ld = sym('ld','real'); % derivative of l(t)
lambda = sym('lambda','positive'); % Rate of decay of the controller. Should be << 1.
T = sym('T','positive'); % Domain of transition for l(q,p) in (c). Should be small.

KE1 = 1/(2*m*l^2)*p^2;
KE2 = 1/2*m*ld^2;
V = - m*g*l*cos(q);
E = KE1 + KE2 + V;

% Get the equations of motion
qd = p/(m*l^2);
pd = -m*g*l*sin(q);
eom = [qd;pd];

% Physical parameters to be used for all controller tests
params.symbolic = [m g lmax lmin lambda T];
params.concrete = [1 9.8 2 1 0.1 0.1];
% Do the same thing in dictionary form, for use with l(theta)
params.dict.m = params.concrete(1);
params.dict.g = params.concrete(2);
params.dict.lmax = params.concrete(3);
params.dict.lmin = params.concrete(4);
params.dict.lambda = params.concrete(5);
params.dict.T = params.concrete(6);

% Initial conditions (0,p0) for each test
p0s = [0.01 0.1 1 5];

% Get the controller l(q)
qmax = atan(1/lambda); % E_dot is guaranteed to be >= 0 before this q
delta = lmax + (lmax*exp(-lambda*qmax) - lmin)/(1 - exp(-lambda*qmax));
a = (lmin - lmax*exp(-lambda*qmax))/(1 - exp(-lambda*qmax));

lq = delta*exp(-lambda*abs(q)) + a;

% Get the base options for plotting odes
baseOptions = odeset('RelTol',10^-8,'AbsTol',10^-8, 'OutputFcn',@odephas2);

% (a) Create a full phase-space controller which keeps ldot = 0 on q <= 0.
if results.run_a
    la = piecewise((0 <= q) & ( q <= qmax), lq, q > qmax, lmin, lmax);
    qda = subs(qd,l,la);
    lad = piecewise((0 <= q) & ( q <= qmax), diff(lq,q)*qda, 0); % l_dot

    % Get the energy for this controller
    syms ELa(q,p);
    ELa(q,p) = subs(E,[l ld], [la lad]);
    ELa(q,p) = subs(ELa, params.symbolic, params.concrete);
    ELaFun = @(q1,p1) double(ELa(q1,p1));

    % Get the eom for this controller
    syms eomLa(q,p);
    eomLa(q,p) = subs(eom,l,la);
    eomLa(q,p) = subs(eomLa, params.symbolic, params.concrete);
    eomLaFun = @(q1,p1) double(eomLa(q1,p1));

    % Plot it in ode45 for various initial conditions and run it for 10 seconds
    for p0 = p0s
        figure;
        sol = ode45(@(t,x)eomLaFun(x(1),x(2)), [0, 10], [0; p0],baseOptions);
        xlabel('q');
        ylabel('p');
        title(sprintf('Orbits for p0 = %.1f',p0));
        figure;
        plot(sol.x,ELaFun(sol.y(1,:),sol.y(2,:)));
        xlabel('t');
        ylabel('E');
        title(sprintf('Energy for l(q) on R1,R2 at p0 = %.1f',p0));
    end
end

% (b) Create a full phase-space controller which keeps ldot = 0 on p <= 0. 
% This requires knowledge of when the q-axis has been crossed, and keeping
% l constant until p becomes > 0 again.
if results.run_b
    % Define l(q), the energy, and the EOM for this setup.
    lb = lq;
    qdb = subs(qd,l,lb);
    lbd = diff(lb,q)*qdb;
    % Get the energy for l on R1, R4
    ELb = subs(E,[l ld],[lb lbd]);
    ELb = subs(ELb, params.symbolic, params.concrete);
    ELbFun = matlabFunction(ELb,'vars',[q p]);
    % Equations of motion
    eomLb = subs(eom,l,lb);
    eomLb = subs(eomLb, params.symbolic, params.concrete);
    eomLbFun = matlabFunction(eomLb,'vars',[q p]);

    % First, we use l(q) on q > 0, p > 0. We do this by starting on (0,p)
    % and flowing until the event p = 0 is triggered. Then we hold onto the
    % length at that position (q,0) until we hit p = 0 again (corresponding
    % to reaching R4) and then continue with l(q) on q < 0, p > 0.
    options = odeset(baseOptions,'Events',@pZeroEvents);
    for p0 = p0s
        figure;
        hold on;
        grid on;
        % Traverse through region 1
        solR1 = ode45(@(t,x)eomLbFun(x(1),x(2)), [0, 10], [0; p0],options);
        % Find the point q that was reached
        q0 = solR1.ye(1,1);
        % Fix the length to that value for R2/R3.
        lq0 = subs(lb,q,q0);
        lq0 = double(subs(lq0, params.symbolic, params.concrete));
        % Find the energy for R2/R3
        ELq0 = subs(E,[l ld],[lq0 0]);
        ELq0 = subs(ELq0, params.symbolic, params.concrete);
        ELq0Fun = matlabFunction(ELq0,'vars',[q p]);
        % Find the EOM for R2/R3
        eomLq0 = subs(eom,l,lq0);
        eomLq0 = subs(eomLq0, params.symbolic, params.concrete);
        eomLq0Fun = matlabFunction(eomLq0,'vars',[q p]);
        % Get the motion along R2/R3
        solR23 = ode45(@(t,x)eomLq0Fun(x(1),x(2)), [0,10], [q0;0],options);
        % Since we started at (q0,0), we get an extra trigger of the event
        % and need to take the second one.
        q1 = solR23.ye(1,2);

        % Now get the motion along R4 and back through R1.
        solR4 = ode45(@(t,x)eomLbFun(x(1),x(2)), [0,10], [q1;0], options);
        xlabel('q');
        ylabel('p');
        title(sprintf('Orbits with l(q) on R1/4 for p0 = %.1f',p0));
        % All the orbits should be plotted now, so display the energy.
        figure;
        hold on;
        grid on;
        plot(solR1.x,...
                ELbFun(solR1.y(1,:),solR1.y(2,:)), 'b', ...
             solR1.x(end) + solR23.x, ...
                ELq0Fun(solR23.y(1,:),solR23.y(2,:)), 'r', ...
             solR1.x(end) + solR23.x(end) + solR4.x, ...
                ELbFun(solR4.y(1,:), solR4.y(2,:)),'b');
        xlabel('t');
        ylabel('E');
        title(sprintf('Energy for l(q) on R1/4 for p0 = %.1f',p0));
    end
end

% (c) Create a constraint which depends on the angle theta = atan2(p,q),
% which alternates between lmin and lmax. However, it must be continuous;
% at each transition we create a small domain T where the continuous
% transition happens, which will be a sinusoid.
if results.run_c
    % Define l(q,p) as a helper function 
    lt = @(t) ltheta(t,params.dict); % make lt accessible to plot.
    lc = @(q,p) ltheta(atan2(p,q),params.dict);
    lcp = @(q,p) lthetap(atan2(p,q),params.dict); %l'(theta)
    
    % Get the symbolic function of l(theta)
    syms th;
    theta = atan2(p,q);
    [lcpw] = lthetapw(theta,params.dict);
    % Get the Equations of Motion symbolically
    syms eomLcS(q,p)
    eomLcS(q,p) = subs(eom, l, lcpw);
    eomLcS(q,p) = subs(eomLcS, params.symbolic, params.concrete);
    eomLcSFun = @(q1,p1) double(eomLcS(q1,p1));
    % Now get them in functional form. we cancel out l in the eom before
    % doing all this so we can substitute in l(theta) as a function
    eomLcFFun = subs(eom.*[l^2;1/l],params.symbolic,params.concrete);
    eomLcFFun = matlabFunction(eomLcFFun,'vars',[q p]);
    eomLcFFun = @(q,p)eomLcFFun(q,p) .* [ 1./(lc(q,p)).^2; lc(q,p)];
    
    % Compute l(theta)_dot as a symbolic function
    lcpwTh = subs(lcpw,theta,th);
    lcpwprime = diff(lcpwTh,th);
    lcpwprime = subs(lcpwprime, th, theta);
    lcpwd = lcpwprime * [diff(theta,q) diff(theta,p)] * eomLcS;
    % Now get it as a matlab function handle
    thetad = matlabFunction([diff(theta,q);diff(theta,p)],'vars',[q p]);
    lcd = @(q,p) lcp(q,p)*(thetad(q,p)')*eomLcFFun(q,p);
    
    % Get the energy of the system to see how that evolves over time,
    % especially during the transitions, as a symbolic function
    syms ELcS(q,p)
    ELcS(q,p) = subs(E,[l ld],[lcpw, lcpwd]);
    ELcS(q,p) = subs(ELcS, params.symbolic, params.concrete);
    ELcSFun = @(q1,p1) double(ELcS(q1,p1));
    % Now get it as a function handle
    ELcKE1 = subs(KE1,params.symbolic,params.concrete);
    ELcKE1 = matlabFunction(ELcKE1*l^2,'vars',[q p]);
    ELcKE2 = subs(KE2,params.symbolic,params.concrete);
    ELcKE2 = matlabFunction(ELcKE2/(ld^2),'vars',[q p]);
    ELcV = subs(V,params.symbolic,params.concrete);
    ELcV = matlabFunction(ELcV/l,'vars',[q p]);
    ELcFFun = @(q,p)(ELcKE1(q,p)./(lc(q,p).^2)) + ...
                    (ELcKE2(q,p).*lcd(q,p).^2) + ...
                    ELcV(q,p).*lc(q,p);
                
    
    % Plot it in ode45 for various initial conditions and run it for 3 seconds
    for p0 = p0s
        figure;
        sol = ode45(@(t,x)eomLcFFun(x(1),x(2)), [0, 3], [0; p0],baseOptions);
        xlabel('q');
        ylabel('p');
        title(sprintf('Orbits for p0 = %.1f',p0));
        figure;
        plot(sol.x,ELcSFun(sol.y(1,:),sol.y(2,:)));
        xlabel('t');
        ylabel('E');
        title(sprintf('Energy for l(q) on R1,R2 at p0 = %.1f',p0));
    end
    
    % Display meshes
    N = 100; % the grid must be square for our FFunctions to work.
    %^ also note that N > 100 makes E go to 10^13 at the transitions, which
    %does not let you see the rest of the shape.
    [qM,pM] = ndgrid(linspace(-pi,pi,N),linspace(-5,5,N));
    % Display mesh of E under l(theta)
    figure;
    mesh(qM,pM,ELcFFun(qM,pM));
    xlabel('q');
    ylabel('p');
    zlabel('E');
    title('Energy under l(theta)');
    % Display mesh of l(theta)
    figure;
    mesh(qM,pM,lc(qM,pM));
    xlabel('p');
    ylabel('q');
    zlabel('l(theta)');
    title('Value of l(atan2(p,q))');
    
    % Define VLP_CONTROLLER so other functions can use this controller.
    VLP_CONTROLLER = lcpw;
end

% This function takes in the symbol for theta, along with
% the parameters dictionary with lmax, lmin, and T
% defined, and the equations of motion [qd;pd]. 
% It returns a piecewise symbolic function representing the
% variable length pendulum's length depending on atan2(p,q)
function [ltpw] = lthetapw(theta, paramsDict)
    % Get the concrete values for ranges of angles
    lmax = paramsDict.lmax;
    lmin = paramsDict.lmin;
    delta = (lmax - lmin)/2;
    lavg = (lmax + lmin)/2;
    T = paramsDict.T;
    w = pi/T;
    
    % Get the symbolic version of the l(theta) function
    ltpw = piecewise(...
        ...% [-pi, -pi+T/2[
        -pi <= theta & theta < (-pi + T/2), ...
            -delta.*sin(w.*(theta + pi)) + lavg, ...
        ...%[-pi+T/2,pi/2-T/2]
        (-pi+T/2) <= theta & theta <= (-pi/2-T/2), lmin, ...
        ...% ]-pi/2 - T/2, -pi/2 + T/2[
        (-pi/2-T/2) < theta & theta < (-pi/2 + T/2), ...
             delta.*sin(w.*(theta + pi/2)) + lavg, ...
        ...% [-pi/2 + T/2, -T/2]
        (-pi/2+T/2) <= theta & theta <= -T/2, lmax, ...
        ...% ]-T/2, T/2[
        -T/2 <= theta & theta <= T/2, ...
            -delta.*sin(w.*theta) + lavg, ...
        ...% [T/2, pi/2-T/2]
        T/2 <= theta & theta <= (pi/2 - T/2), lmin, ...
        ...% ]pi/2-T/2, pi/2+T/2[
        (pi/2-T/2) < theta & theta < (pi/2+T/2), ...
            delta.*sin(w.*(theta - pi/2)) + lavg, ...
        ...% [pi/2+T/2, pi-T/2]
        (pi/2+T/2) <= theta & theta <= (pi - T/2), lmax, ...
        ...% ]pi-T/2,pi]
        (pi - T/2) < theta & theta <= pi, ...
            -delta.*sin(w.*(theta - pi)) + lavg);
end

% This function takes in theta in [-pi,pi] and the parameters structure
% with lmax, lmin, and T defined, and returns the time-optimal function for
% making a variable length pendulum swing depending on the current angle in
% phase space. This function smooths out the transitions so they can be
% used as VNHCs.
function lval = ltheta(theta, paramsDict)
    % Set up parameters from the input params dictionary
    lmax = paramsDict.lmax;
    lmin = paramsDict.lmin;
    delta = (lmax - lmin)/2;
    lavg = (lmax + lmin)/2;
    T = paramsDict.T;
    w = pi/T;
    % Set up the output
    lval = zeros(size(theta));
    % l(theta) values within [-pi, -pi+T/2[ are transitionary going down
    Rminuspi = -pi <= theta & theta < (-pi + T/2);
    lval(Rminuspi) = -delta.*sin(w.*(theta(Rminuspi)+pi)) + lavg;
    % l(theta) values within [-pi+T/2,-pi/2-T/2]lmin
    Rlmin_minus = (-pi+T/2) <= theta & theta <= (-pi/2-T/2);
    lval(Rlmin_minus) = lmin;
    % l(theta) values within ]-pi/2-T/2, -pi/2+T/2[ are transitions up
    Rminuspi2 = (-pi/2-T/2) < theta & theta < (-pi/2 + T/2);
    lval(Rminuspi2) = delta.*sin(w.*(theta(Rminuspi2)+pi/2)) + lavg;
    % l(theta) values within [-pi/2+T/2, -T/2] are lmax
    Rlmax_minus = (-pi/2+T/2) <= theta & theta <= -T/2;
    lval(Rlmax_minus) = lmax;
    % l(theta) values within ]-T/2,T/2[ are transitions down
    Rzero = -T/2 < theta & theta < T/2;
    lval(Rzero) = -delta.*sin(w.*theta(Rzero)) + lavg;
    % l(theta) values within [T/2,pi/2-T/2] are lmin
    Rlmin_plus = T/2 <= theta & theta <= (pi/2 - T/2);
    lval(Rlmin_plus) = lmin;
    % l(theta) values within ]pi/2-T/2,pi/2+T/2[ are transitions up
    Rpluspi2 = (pi/2-T/2) < theta & theta < (pi/2+T/2);
    lval(Rpluspi2) = delta.*sin(w.*(theta(Rpluspi2)-pi/2)) + lavg;
    % l(theta) values within [pi/2+T/2,pi-T/2] are lmax
    Rlmax_plus = (pi/2+T/2) <= theta & theta <= (pi - T/2);
    lval(Rlmax_plus) = lmax;
    % l(theta) values within ]pi-T/2,pi] are transitions down
    Rpluspi = (pi - T/2) < theta & theta <= pi;
    lval(Rpluspi) = -delta.*sin(w.*(theta(Rpluspi) - pi)) + lavg;
end

function ldval = lthetap(theta,paramsDict)
    % Set up parameters from the input params dictionary
    lmax = paramsDict.lmax;
    lmin = paramsDict.lmin;
    delta = (lmax - lmin)/2;
    T = paramsDict.T;
    w = pi/T;
    % Set up the output
    ldval = zeros(size(theta));
    % l(theta) values within [-pi, -pi+T/2[ are transitionary going down
    Rminuspi = -pi <= theta & theta < (-pi + T/2);
    ldval(Rminuspi) = -delta.*w.*cos(w.*(theta(Rminuspi)+pi));
    % l(theta) values within [-pi+T/2,-pi/2-T/2]lmin
    Rlmin_minus = (-pi+T/2) <= theta & theta <= (-pi/2-T/2);
    ldval(Rlmin_minus) = 0;
    % l(theta) values within ]-pi/2-T/2, -pi/2+T/2[ are transitions up
    Rminuspi2 = (-pi/2-T/2) < theta & theta < (-pi/2 + T/2);
    ldval(Rminuspi2) = delta.*w.*cos(w.*(theta(Rminuspi2)+pi/2));
    % l(theta) values within [-pi/2+T/2, -T/2] are lmax
    Rlmax_minus = (-pi/2+T/2) <= theta & theta <= -T/2;
    ldval(Rlmax_minus) = 0;
    % l(theta) values within ]-T/2,T/2[ are transitions down
    Rzero = -T/2 < theta & theta < T/2;
    ldval(Rzero) = -delta.*w.*cos(w.*theta(Rzero));
    % l(theta) values within [T/2,pi/2-T/2] are lmin
    Rlmin_plus = T/2 <= theta & theta <= (pi/2 - T/2);
    ldval(Rlmin_plus) = 0;
    % l(theta) values within ]pi/2-T/2,pi/2+T/2[ are transitions up
    Rpluspi2 = (pi/2-T/2) < theta & theta < (pi/2+T/2);
    ldval(Rpluspi2) = delta.*w.*cos(w.*(theta(Rpluspi2)-pi/2));
    % l(theta) values within [pi/2+T/2,pi-T/2] are lmax
    Rlmax_plus = (pi/2+T/2) <= theta & theta <= (pi - T/2);
    ldval(Rlmax_plus) = 0;
    % l(theta) values within ]pi-T/2,pi] are transitions down
    Rpluspi = (pi - T/2) < theta & theta <= pi;
    ldval(Rpluspi) = -delta.*w.*cos(w.*(theta(Rpluspi) - pi));
end