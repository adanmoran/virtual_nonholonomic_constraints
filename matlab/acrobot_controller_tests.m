%% Acrobot Controller Testing
% We finished getting insights from the Variable-Length Pendulum, so now
% it's time to do the same with the 
%
% This script will generate the acrobot model and apply nonholonomic
% constraints on the actuated controller:
% 
% a) Use a smooth input, such as sin(2*theta) where theta is the angle of
% the unactuated state in phase space
%
% b) Similarly to the variable-length pendulum give a "kick"-type
% transition at each axis of the phase podtrait. For example, kick forward
% (and keep legs bent) to go up, lengthen to go down.
%
% This script will also attempt to find an average energy Eavg for
% whichever controller works, which is strictly increasing (if possible).

clear all
close all
clc;
fprintf('-----------------\n');
fprintf(' Acrobot Control \n');
fprintf('-----------------\n');

% Script parameters
script.smooth_control.run = true; % Run the smooth controller sin(theta)
script.kick_control.run = true; % Run the kick controller
script.simplify_acrobot = true; % Make both links the same
% Find qa(theta) to generate a VLP controller in the effective pendulum
% which represents the acrobot.
script.controller.as_vlp = false; 

if script.controller.as_vlp && ~script.simplify_acrobot
    error('Acrobot must be simplified to generate VLP-style controller.');
end

% Build the acrobot model
fprintf('Initializing Symbols.\n');
% Symbols for phase of the acrobot
qu = sym('qu','real'); % Unactuated limb angle relative to vedtical
pu = sym('pu','real'); % Conjugate of momenta for qu
qa = sym('qa','real'); % Actuated limb angle relative to qu
pa = sym('pa','real'); % Conjugate of momenta for qa
q = [qu;qa];
p = [pu;pa];
% Symbols for dynamics of the acrobot, with their corresponding real values
mt = sym('m_t','positive'); % mass of torso link
ml = sym('m_l','positive'); % mass of leg link
Jt = sym('J_t','positive'); % Moment of inedtia of the torso link
Jl = sym('J_l','positive'); % Moment of inedtia of the leg link
lt = sym('l_t','positive'); % Distance of the hand to the torso center of mass (COM)
ll = sym('l_l','positive'); % Distance of the hip to the leg COM
dt = sym('R_t','positive'); % Length of the torso link
dl = sym('R_l','positive'); % Length of the leg link
g =  sym('g'  ,'positive'); % Gravitational constant

% Symbols for control
tau = sym('tau'); % Control input

% If we are using simplified dynamics, do that now. This models the links
% being of the same length, with point masses at the tips of the links.
% Thus, the moments of inertia are m*d^2.
if script.simplify_acrobot
    m = sym('m','positive'); % mass of both links
    d = sym('d', 'positive'); % length of both links
    mt = m; ml = m;
    Jt = 0; Jl = 0; %m*d^2; Jl = m*d^2;
    ll = d; lt = d; dt = d; dl = d;
    dynamics.symbolic = [m d g];
    dynamics.concrete = [30 1 9.8]; % A Gymnast
    %dynamics.concrete = [0.2 0.4 9.8]; % A small test robot
else
    % Convedt the dynamics into parameter arrays for substitution
    dynamics.symbolic = [mt, ml, Jt, Jl, lt ,  ll ,  dt, dl, g  ];
    dynamics.concrete = [1, 1 , 1 , 1 , 2,  1,  4 , 2 , 9.8];
end
% Define the maximum range over which qa can vary.
qa_max = pi/8;
% Define the time frame for which we simulate
T = 10;

% Get the motion model
fprintf('Getting EOM for the acrobot.\n');

% Mass/Inertia matrix
M = [ml*dt^2 + 2*ml*cos(qa)*dt*ll + ml*ll^2 + mt*lt^2 + Jl + Jt,  ...
                                    ml*ll^2 + ml*dt*ll*cos(qa) + Jl; ...
        ml*ll^2 + ml*dt*ll*cos(qa) + Jl,   ml*ll^2 + Jl];

% Potential function
V = g*(ml*ll*(1 - cos(qu + qa)) + (ml*dt + mt*lt)*(1 - cos(qu)));

% Gradient of V(q)
GVq = [diff(V,qu); diff(V,qa)];
% Control matrix
B = [0;1];
% Left-annihilator of control matrix
Bperp = [1, 0];

% Energy of the acrobot
E = 1/2 * (p' * (M \ p)) + V;

% Equations of motion
qd = M \ p;
dMinvdqa = diff(inv(M),qa);
pd = -[0 ; 1/2 * p' * dMinvdqa * p] - GVq + B*tau;
eom = [qd; pd];
% Equations of motion specifically for the pair (qu,pu)
eomu = simplify([qd(1);pd(1)]);

% Initial conditions (q0,0) for each test
q0s = [pi/16 pi/8  pi/4 pi/2 3*pi/2 pi-eps];
q0sToUse = 2*qa_max/(3+2*cos(qa_max));

% Base options for plotting the odes
baseOptions = odeset('RelTol',10^-8,'AbsTol',10^-8, 'OutputFcn',@odephas2);

% a) Define a smooth controller for the acrobot
if script.smooth_control.run
    fprintf('Creating the smooth controller.\n');
    % Define the controller which makes the acrobot gain energy smoothly.
    theta = atan2(pu,qu);
    qa_avg = 0;
    if script.controller.as_vlp
        l_max = 3/2*d;
        l_min = d*sqrt(5/4+cos(qa_max));
        delta_l = (l_max - l_min)/2;
        lAVG = (l_max + l_min)/2;
        % Make the effective VLP have l(theta) = -delta*sin(2theta)+lavg
        qa_a = acos(((-delta_l * sin(2*theta) + lAVG)/d)^2-5/4);
    else
        qa_a = qa_max * sin(theta) + qa_avg;
    end
    % For some reason we get qa_a contains an abs(qu + pu*1i). This can be
    % converted to a real value by sqrt(qu^2 + pu^2), since sin(theta) =
    % pu normalized
    qa_a = subs(qa_a, abs(qu + pu*1i), sqrt(qu^2 + pu^2));
    
    % Compute pa under this control input using the equations
    % developed when coming up with nonholonomic constraints
    % First, we need the equation h(q) = qa - f(qu,pu) = 0
    h = qa - qa_a;
    % Now get the derivative of h with respect to q
    dhq = [diff(h,qu) diff(h,qa)];
    % We also need the derivative of qa_a with respect to p, since it is a
    % nonholonomic constraint. Label qa_a = f(qu,pu) to say df/dpu = dfp
    dfp = diff(qa_a,pu);
    % Now we can directly compute pa_a. We will assume the VNHC is solvable
    % without checking it, because the structure of the constraint implies
    % it should be.
    pa_a = -(dhq * (M \ [0;1])) \ (dfp*GVq(1) + dhq * (M \ [1;0])*pu);
    pa_a = subs(pa_a,qa,qa_a);
    
    % Get the equations of motion under this control
    eomqa_a = subs(eomu,pa,pa_a);
    eomqa_a = subs(eomqa_a, qa, qa_a);
    eomqa_a = subs(eomqa_a, dynamics.symbolic, dynamics.concrete);
    eomqa_aFun = matlabFunction(eomqa_a, 'vars', [qu pu]);
    eomqa_aFun = @(q,p)sign(eomqa_aFun(q,p)).*abs(eomqa_aFun(q,p));
    
    % Get the actual energy function for this controller
    E_a = subs(E, [qa pa], [qa_a pa_a]);
    E_a = subs(E_a, dynamics.symbolic, dynamics.concrete);
    E_aFun = matlabFunction(E_a,'vars',[qu pu]);
    
    % Let us define a function for computing pa when qa is constant - i.e.
    % h(q) = qa - const.
    dhq_const = [0 1];
    pa_const = -(dhq_const* (M \ [0;1])) \ (dhq_const * (M \ [1;0])*pu);
    
    % Define a bunch of possible average-energy functions to see if any of
    % them are strictly increasing over time
    
    % First, a function for a variable length pendulum under the assumption
    % that the acrobot is simple. This is only well-defined if
    % script.simplify_acrobot is enabled.
    if script.simplify_acrobot
        leff = d*sqrt(5/4 + cos(qa));
        lavg = (subs(leff,qa,qa_max) + subs(leff,qa,0))/2;
        Evlpu = pu^2/(2*m*lavg^2) + m*g*lavg*(1 - cos(qu));
    end
    
    % Another option is to use a lyapunov function derived directly from
    % setting qa_dot = 0, qa = const and obtaining a 1-DOF energy function
    if script.simplify_acrobot
        qud = sym('qud','real'); % velocity of the arm
        qad = sym('qad','real'); % velocity of the leg
        qd = [qud; qad];
        % Compute the kinetic energy
        KE1dof = simplify(1/2 * qd' * M * qd);
        KE1dof = subs(KE1dof, qad, 0);
        
        % Compute the "inertia" for the 1DOF system from pud = dL/dqud =
        % (inertia)*qud.
        m1dof = simplify(diff(KE1dof, qud)/qud);
        % Convert to hamiltonian energy form
        E1dof = 1/(2*m1dof) * pu^2 + V;
        
        % Now sub in a midpoint value for qa in E1dof. There is no pa in
        % this version. This midpoint value is the qa which 
        % generates the average value for leff along theta.
        t = sym('t'); % angle symbol
        % First, get the average length of the effective VLP as a function
        % of the angle
        if script.controller.as_vlp
            leff1dof = -delta_l*sin(2*t)+lAVG;
        else
            leff1dof = d*sqrt(5/4 + cos(qa_max*sin(t)));
        end
        leff1dofa = subs(leff1dof,dynamics.symbolic,dynamics.concrete);
        leff1dofaFun = matlabFunction(leff1dofa,'vars',t);
        % Now get its average
        leffAvg = 1/(2*pi)*integral(leff1dofaFun,-pi,pi);
        % -> As it turns out, leffAvg = lavg from above!
        
        % Now get qa that generates this by inverting the equation
        if script.controller.as_vlp
            % If the controller is modelling the smooth VLP one, then at
            % theta = 0 we get l(theta) = lAVG => qa(0) is our "average"
            % angle.
            qa1dof = double(subs(acos(((lAVG)/d)^2-5/4), ...
                    dynamics.symbolic, dynamics.concrete));
        else
            qa1dof = acos(leffAvg^2/d - 5/4);
        end
        qa1dof = double(subs(qa1dof,dynamics.symbolic,dynamics.concrete));
        E1dofa = subs(E1dof,qa,qa1dof);
    end
    
    %% TODO: This is the wrong "effective" VLP. Please convert this into the correct one.
    % Another option is to find the "effective" variable-length pendulum,
    % with q = q(qu,qa) and p = p(qu,qa,pu,pa), l = l(qa). Then we
    % substitute qa = f(qu,pu) and solve, then use the Eavg being the
    % for this effective vlp (this is different than the previous one
    % because it does not use q = qu and p = pu). Again, this is only
    % well-defined if script.simplify_acrobot is enabled.
    if script.simplify_acrobot
        % The length is given above by leff. 
        % The q-variable for the effective
        % pendulum depends on an additional term "gamma" which we input
        % here.
        qeff = atan2((sin(qu) + sin(qu + qa)), ...
                    -(cos(qu) + 1/2*cos(qu + qa)));
        %% To get peff, we need to differentiate qeff and
        % multiply by the total mass (it's 2*m since the acrobot is
        % simplified).
        peff = m * (diff(qeff,qu)*qd(1) + diff(qeff,qa)*qd(2));
        % Get the derivative of leff 
        leffd = diff(leff,qa)*qd(2);
        % Now that we have both qeff and peff, we can compute Eeff by
        % taking the equations for a variable-length pendulum (which also
        % involves taking l_dot.
        KEeff = 1/(2*m*leff^2)*peff^2 + 1/2*m*leffd^2;
        Veff = m*g*leff*(1 - cos(qeff));
        Eeffsym = KEeff + Veff;
        % Now that we have Eeff, we can simply substitute in different
        % values to test.
        % For instance, let's start with Eeffective at qa = const.
        % Something fails at doing qa_max/2 for some reason, as does it at
        % pi/4, pi/16, pi. Use 0.9999*(input) for these.
        qeff1 = qa_max;
        peff1 = subs(pa_const, qa, qeff1);
        Eeff1 = subs(Eeffsym, pa, peff1);
        Eeff1 = subs(Eeff1, qa, qeff1);
        
        % Instead of choosing a random constant term, let's input l(q) =
        % lavg and remove leffd, since it is just l'(qa)qa_d and qa_d = 0.
        Eeff2 = 1/(2*m*lavg^2)*peff^2 + m*g*lavg*(1 - cos(qeff));
        % Now we can find q such that l(q) = lavg and use that as our
        % input.
        qlavg = qa1dof;
        peff2 = subs(pa_const, qa, qlavg);
        Eeff2 = subs(Eeff2, pa, peff2);
        Eeff2 = subs(Eeff2,qa, qlavg);
    end
    
    % Another option is to use the energy of the acrobot at the "midpoint"
    % length for one direction. For example, for p > 0 we might have the
    % "midpoint" be +qa_max/2, while for p < 0 we might have the "midpoint"
    % be -qa_max/2, or some other value qa_mid. 
    % That is, use the constraint h(q) = qa -/+ qa_mid,
    % which leads to dhq = [0 1] and dfp = 0.
    qa_mid = 0;%qa_max + qa_avg;
    pa_mid = subs(pa_const,qa,qa_mid);
    Emid = subs(E,pa,pa_mid);
    Emid = subs(Emid, qa, qa_mid);
    
    qa_mid2 = -qa_max + qa_avg;
    pa_mid2 = subs(pa_const,qa,qa_mid2);
    Emid2 = subs(E, pa, pa_mid2);
    Emid2 = subs(Emid2, qa, qa_mid2);
    
    % Select the actual Eavg function
    Eavg = Emid;
    Eavg = subs(Eavg,dynamics.symbolic,dynamics.concrete);
    EavgFun = matlabFunction(Eavg, 'vars', [qu pu]);
    
    % Now just take the derivative and use that to plot. We want this to be
    % >= 0 everywhere.
    Eavgd = [diff(Eavg,qu) diff(Eavg,pu)]*eomqa_a;
    Eavgd = simplify(subs(Eavgd, dynamics.symbolic, dynamics.concrete));
    EavgdFun = matlabFunction(Eavgd, 'vars', [qu pu]);
    
    % Plot simulations in ode45
    % There was something weird going on here... we can't use small p0,
    % it seems to crash because the integration gets close enough to [0;0]
    % that it results in a division by zero. Inherently, that means we've
    % lost enough energy that the system stops because we hit [0;0].
    % Alternatively, it's just that the simulation is crashing from the
    % division by zero which occurs BEFORE you hit [0;0], and the energy
    % would increase again from there. It's unclear.
    N = 1000;
    [qM,pM] = ndgrid(linspace(-pi,pi,N),linspace(-20,20,N));
    for q0 = q0sToUse
        % Plot the orbit
        figure;
        sol = ode45(@(t,x)eomqa_aFun(x(1),x(2)), [0, T], [q0;0],baseOptions);
        xlabel('q');
        ylabel('p');
        title(sprintf('Orbits for q0 = %.1f',q0));
        % Plot the actual mechanical energy
        figure;
        plot(sol.x, E_aFun(sol.y(1,:), sol.y(2,:)));
        xlabel('time (s)');
        ylabel('E');
        title(sprintf('True Mechanical Energy for q0 = %.1f',q0));
        % Plot the chosen "average energy" function's derivative
        figure;
        plot(sol.x, EavgdFun(sol.y(1,:), sol.y(2,:)));
        xlabel('time (s)');
        ylabel('Edot');
        title(strcat('$\dot{E_{avg}}$ for $q0 = $',...
            sprintf('%.1f',q0)),'Interpreter','latex');
        % Plot the sections of the orbit where the "average energy"
        % function is increasing
        figure;
        Eavgd_ge0 = EavgdFun(sol.y(1,:),sol.y(2,:)) >= 0;
        plot(sol.y(1,Eavgd_ge0),sol.y(2,Eavgd_ge0));
        xlabel('q');
        ylabel('p');
        title(strcat('Parts of Orbit where $\dot{E_{avg}} \geq 0$ ',...
            'for $q0 = $',sprintf('%.1f',q0)),'Interpreter','latex');
        figure;
        mesh(qM,pM,EavgdFun(qM,pM) >= 0);
        xlabel('q');
        ylabel('p');
        title(strcat('Phase Space where $\dot{E_{avg}} \ge 0$ ', ...
            ' for $q_0 = $',sprintf('%.1f',q0)),'Interpreter','latex');
        colorbar;
        view(2); % set a 2d view
        
        % Finally, plot what the effective length of the pendulum does along
        % solutions to the ODE.
        figure;
        thetas = atan2(sol.y(2,:),sol.y(1,:));
        plot(thetas,leff1dofaFun(thetas),[-pi,pi],[leffAvg,leffAvg]);
        xlabel('\theta');
        ylabel('l(\theta)');
        title('Effective length of COM along trajectories');
        legend({'l(\theta)','l_{avg}'},'Location','SouthWest');
    end
    
    %% Compute the equations of motion in polar coordinates
    fprintf('Converting everything to polar coordinates.\n');
    syms r t
    % First get qu_dot and pu_dot as functions of r and theta
    eomqp_in_rt = simplify(subs(eomqa_a,[qu pu],[r*cos(t) r*sin(t)]));
    % Now get dr = derivative of r as a function of time in (r,t) coords
    rqp = sqrt(qu^2+pu^2);
    drqp = subs([diff(rqp,qu) diff(rqp,pu)],[qu pu],[r*cos(t) r*sin(t)]);
    drqp = simplify(drqp);
    dr = simplify(drqp*eomqp_in_rt);
    drf = matlabFunction(dr, 'vars', [t r]);
    % Get dt = derivative of theta as a fn of time in (r,t) coords
    tqp = atan2(pu,qu);
    dtqp = subs([diff(tqp,qu) diff(tqp,pu)],[qu pu],[r*cos(t) r*sin(t)]);
    dtqp = simplify(dtqp);
    dt = simplify(dtqp*eomqp_in_rt);
    dtf = matlabFunction(dt,'vars',[t r]);
    
    % Plot the mesh theta_dot >= 0 to see where theta is increasing or
    % decreasing (which we need to know for averaging theory)
    [tM, rM] = ndgrid(linspace(-pi,pi,N),linspace(0,4*pi,N));
    figure;
    mesh(tM,rM, dtf(tM,rM)>=0);
    xlabel('\theta');
    ylabel('r');
    title('$\dot{\theta} \geq 0$','interpreter','latex');
    colorbar;
    % Now on the same display, show what values r is allowed to take
    hold on;
    % r must be below pi/|cos(theta)|
    thetas = -pi:1/N:pi;
    rmaxplot = plot(thetas, pi./abs(cos(thetas)));
    % for the denominator of theta_dot to be positive, we require r > r0
    rmin = 2*qa_max/(3+2*cos(qa_max));
    r0plot = plot([-pi pi], [rmin, rmin]);
    % Show the plot as a 2d figure
    axis([min(min(tM)) max(max(tM)) min(min(rM)) max(max(rM))]);
    view(2);
    legend([rmaxplot r0plot],{'$\frac{\pi}{|cos(\theta)|}$','$r_0$'},...
        'interpreter','latex');
    hold off;
    
    % Plot orbits on the (t,r) plane instead of the (q,p) plane
    tsol = atan2(sol.y(2,:),sol.y(1,:));
    rsol = sqrt(sol.y(1,:).^2 + sol.y(2,:).^2);
    figure;
    plot(tsol,rsol);
    xlabel('\theta');
    ylabel('r');
    title(strcat('Orbit for q0 = ',sprintf('%.2f',q0),' in polar form.'));
    
    % Now plot orbits of dr/dtheta as a function of theta instead of time,
    % which solves the integral integral{dr/dt * dt, dt = 0 to t}. Thus,
    % the average integral{dr/dt * dt, dt = 0 to 2*pi} is the orbit value
    % at theta = 2*pi.
    % r' = drdt(r',theta) is a scalar ode that represents how r changes
    % with respect to theta.
    drdt = simplify(dr/dt);
    rp = matlabFunction(drdt,'vars',[t r]);
    figure;
    % Plot theta (our new time) over 3 rotations for an r0 value of q0,
    % which corresponds to (q,p) = (q0,0). Since theta(t) goes negative in
    % our actual orbits, we plot over clockwise rotations by starting at 0
    % and going to -6*pi.
    drdtoptions = odeset('AbsTol',10^-8,'RelTol',10^-8,...
        'OutputFcn',@odeplot);
    soldrdt = ode45(@(t,x)rp(t,x), [0,-6*pi],0.4, drdtoptions);
    xlabel('\theta');
    ylabel('$r^`(\theta)$','interpreter','latex');
    title(strcat('$r^`(\theta)$ where $\theta$',...
        '(t) travels left since $\dot{\theta}<0$'),'interpreter','latex');
    
    % Plot the mechanical energy as a function of theta(t)
    figure;
    plot(tsol,E_aFun(sol.y(1,:),sol.y(2,:)));
    xlabel('\theta(t)');
    ylabel('E(q(t),p(t))');
    title('Energy as a function of theta over time');
    
    % Results: Using this smooth controller works, though it is unclear
    % why at the moment. I was just messing around with it until something
    % happened, and this one worked just fine. Switching it to -sin(theta)
    % also removes energy from the system. I think this is the smoothed
    % version of some "kick" method which is more time-optimal, similarly
    % to what we had in the VLP.
    
    % TODO: We may perhaps want to restrict the controller to one
    % direction. For instance, we can make the controller be
    % qa = sin(theta) if pu > 0, 0 if pu <= 0. This might give us a bit
    % more insight, since we can see how the system reacts on a half-orbit.

end