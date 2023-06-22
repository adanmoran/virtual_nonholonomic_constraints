%% Expansion Characteristics of Virtual Nonholonomic Constraints
% This script simulates the expansion characteristics for a virtual
% nonholonomic constraint. The dynamics are assumed to be for a single
% coordinate along two different constrained regions in the (q,p) plane.
% The region {(q,p) | q in ]0,pi[, p > 0} corresponds to the area where the
% VNHC is applied, while the rest of the phase space is associated with a
% virtual holonomic constraint.
% 
% This function generates takes in a set of increasing points on the
% (0,p)-axis, {p1,...,pN}. These share a VHC-hamiltonian H(q,p) with the 
% points {q1,...qN}
% i.e. H(0,pj) = H(qj,0) for j in 1,...,N
%
% After finding the points {q1,...,qN} using the VHC dynamics, the
% trajectories for {p1,...,pN} are simulated using the VNHC dynamics on the
% top-right quadrant. This leads to new points {q_bar1,...,q_barN}. We know
% that H(0,pj) - H(q_barj,0) = c(pj). Intuitively, if c(pj) > 0 then
% q_barj > qj, so the VNHC injects energy along orbits starting at pj.
% Alternatively, if c(pj) < 0 then q_barj < qj, so the VNHC reduces energy 
% along that orbit.
%
% If H(0,pj) - H(q_barj,0) = e is contant, then there is a first integral
% of the dynamics J(q,p) so that GradJ(0,p) = GradH(0,p) and 
% GradJ(q,0) = GradH(q,0) for all q and p.
%
% Inputs:
% * dynH = the VHC dynamics, used for computing H(q,p). This is either a) a
% symbolic equation, with the symbols {p,q} in the equation [phi(q)p;Pi(q)]
% (in that order when calling symvar) or b) a function handle, with the
% format @(q,p) [phi(q)p;Pi(q)].
% * dynNH = the VNHC dynamics, used for simulating the orbits starting at
% (0,pj) and ending at (q_barj,0). This must be either a) a symbolic
% equation, with the symbols {p,q} in the equation 
% [phi_1(q,p)p + phi_2(q,p);Pi(q,p)] or b) a function handle with the
% format @(q,p) [phi_1(q,p)p + phi2(q,p); Pi(q,p)]. Note that Pi is
% in general different from that of dynH.
% * pN = a vector of points [p1;...;pN] to simulate. All values must be
% strictly positive.
% * duration = (optional) maximum duration for which to run ode45 when
% simulating each trajectory. Default is Inf, which may cause a hang if
% the trajectory never hits the q-axis.
% * options = (optional) an initial set of options that can be passed in to
% odeset for plotting, etc. Default is to have no output function.
%
% Outputs:
% * p0 = pN, sorted in increasing order.
% * q_bar = the points which intersect the q-axis after following the flow
% starting from (0,pj).
% * c = the values c(pj) given by H(q_bar(j),0) - H(0,p0(j)). 
function [c, q_bar, p0] = expansion_vnhc(dynH,dynNH, pN, varargin)
    % Validate the inputs
    parser = inputParser;
    addRequired(parser,'dynH');
    addRequired(parser,'dynNH');
    addRequired(parser,'pN',@isvector);
    validPosNum = @(x) validateattributes(x,{'numeric'},...
                        {'scalar','positive'});
    addOptional(parser,'duration',Inf,validPosNum);
    defaultOptions = odeset('OutputFcn',[]);
    addOptional(parser,'options',defaultOptions);
    parse(parser,dynH,dynNH,pN,varargin{:});
    
    % Set p0 to the vector form of pN, sorted and filtered
    p0 = sort(unique(pN));
    % Find N, the number of points provided
    N = numel(p0);
    % Make sure p0 is a column vector
    p0 = reshape(p0,N,1);
    
    % Compute H(0,pj) for each j
    Hs = hamiltonian_vhc(zeros(N,1),p0,dynH);
    
    % Get the VNHC dynamics as a function of two variables
    if isa(dynNH,'function_handle')
        f = dynNH;
    else
        f = sym2fun(dynNH);
    end
    
    % Set the options for ode45 to stop when the q-axis is touched. We also
    % display the simulations
    baseOptions = parser.Results.options;
    options = odeset(baseOptions,'Events',@pZeroEvents);
    
    % Simulate each p0. We use arrayfun to get
    % the entire ODE solution directly.
    tf = parser.Results.duration;
    simSols = arrayfun(@(p) ode45(@(t,x)f(x(1),x(2)), [0 tf], [0;p],...
                                  options), p0);
    % Get q_bar directly from the simSols, since it stores the events list
    % corresponding to when p = 0. We only take the first one that appears,
    % since after this first contact we no longer use the nonholonomic
    % dynamics.
    q_bar = zeros(N,1);
    for i = 1:N
        sol = simSols(i);
        q_bar(i) = sol.ye(1,1);
    end
    
    % Compute H(q_bar,0) for each q_bar.
    Hqbar = hamiltonian_vhc(q_bar,zeros(size(q_bar)),dynH);
    
    % Subtract H(0,pj) from H(q_bar,0) to get the difference, and plot with
    % respect to pj to get c(pj).
    c = Hqbar - Hs;
    figure;
    plot(p0,c);
    grid on;
    xlabel('p');
    ylabel('$H(0,p) - H(q_bar,0)$','Interpreter','latex');
    title('Expansion Characteristics');
end

