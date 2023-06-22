%% VNHC Expansion with Lyapunov Functions
% Our goal is to see if there exists a W(q,p) = 1/2p^2 + mu/2q^2 so that
% a) For qk_hat where W(qk_hat,0) = W(0,pk), we have H(qk_hat,0) > H(0,pk)
% where H is the hamiltonian for a VHC
% b) Along solutions of dynamics for Xingbo's VNHC, we get that
% W_dot >= 0
%
% These two facts together imply that the energy of the acrobot is
% increasing along nonholonomic dynamics, since they drive the acrobot from
% a state (0,pk) to a state qk LARGER THAN (qk_hat,0), which means H(qk,0)
% > H(qk_hat,0) > H(0,pk) = H(q_{k-1},0) => qk > q_{k-1}.
%
% This script finds valid mu so that a) is satisfied and displays the
% figures. We can then check that visually to see if b) is satisfied.
%
% Other W(q,p) I've tried that did not work:
% * 1/2 p^2 + mu/2 *sin^2(q) for mu large enough (and locally only)
% This has 1/2 p0^2 = mu/2*sin^2(q0) iff mu = p0^2/sin^2(q0)
% and qk = positive version of asin(sqrt(pk^2/mu)).
% 1/2 p^2 + mu/2 * mu/2 sin(q) for 


% Check that stuff from xingbo_nvhc_v2.m has been created. If not, run that
% script.
if ~exist('XINGBO_NVHC_V2','var')
    xingbo_nvhc_v2
end

% Define a bunch of data structures which will contain information about
% how W changes in different ranges of p
expansion1.pRange = 0.1:0.1:2; % range of p over which to solve for W
expansion1.p0 = 1; % p used to find mu
expansion1.contourArea = [0 0.29 0 2]; % area over which to plot fcontour
expansion1.bestq0 = 0.143; % best value of q0 I've found
% if true q0 should be taken as input. If false use bestq0
expansion1.askForq0 = false;

% Find where H(qk_hat,0) > H(0,pk) for expansion1. We know 0.143 is the
% best so far, but there might be a better result in a small range.
qk_hat1 = 0.14:0.0005:0.145;
lyapStructs1 = findPkGrowing(expansion1,qk_hat1,dynHfun,nhFun);

% Do the same for the next range
expansion2.pRange = 2.1:0.1:4;
expansion2.p0 = 3; 
expansion2.contourArea = [0 0.6 0 4];
expansion2.askForq0 = true;

qk_hat2 = 0.43:0.005:0.46;
lyapStructs2 = findPkGrowing(expansion2,qk_hat2, dynHfun,nhFun);

% Test the
expansion3.pRange = 8.1:0.1:10;
expansion3.p0 = 9;
expansion3.contourArea = [0 1.7 0 10];
expansion3.askForq0 = true;

qk_hat3 = 1.37:0.005:1.41;
lyapStructs3 = findPkGrowing(expansion3, qk_hat3, dynHfun, nhFun);
%% Helper Functions

% Take in a set of possible qk_hat values and find the ones which produce a
% pk_growing set.

function growingStructs = findPkGrowing(expansionStruct, qk_hat, dynHfun, nhFun)
growingStructs = cell(1);
j = 1;
for i = 1:numel(qk_hat)
    expStruct = expansionStruct;
    expStruct.bestq0 = qk_hat(i);
    expStruct.askForq0 = false;
    lyapStruct = findW({expStruct},dynHfun,nhFun);
    expSt = lyapStruct{1};
    if numel(expSt.pk_growing) == 0
        close(expSt.f);
    else
        growingStructs{j} = expSt;
        j = j + 1;
    end
end
end

% Take in a cell array and compute W(q,p) for each element of the array.
% TODO: Use a parser to add options with "q0" as a choice, rather than
% using the property of the struct? also remove the for loop, this will
% just be used for a single expansion struct.
function newLyapStruct = findW(lyapunovStructs, dynHfun, nhFun)
    % Try to find a W(q,p) = mu/2 q^2 + 1/2 p^2 so that W(qk_hat,0) =
    % W(0,pk) and H(qk_hat,0) >= H(0,pk) for each struct in the lyapunov
    % structs.
    for i = 1:numel(lyapunovStructs)
        % Get the current data structure where we will store everything
        expansionStruct = lyapunovStructs{i};

        % Start a new figure for this W vs. ODE
        f = figure;
        hold on
        grid on;
        pRange = expansionStruct.pRange;
        for p = pRange
        sol = ode45(@(t,x)nhFun(x(1),x(2)),[0,2],[0;p],...
        odeset('Events',@pZeroEvents,'RelTol',10^-13,'AbsTol',10^-13));
        plot(sol.y(1,:),sol.y(2,:),'--');
        end
        % Ask for the value of q0.
        if expansionStruct.askForq0
            q0 = input(sprintf(strcat('What is the value where W(0,%0.2f)',...
                ' should cross the q-axis? \n'),expansionStruct.p0));
        else
            q0 = expansionStruct.bestq0;
        end
        % Determine mu, which is easy to compute from W(0,p0) = W(q0,0)
        mu = (expansionStruct.p0/q0)^2;
        % Now get the function for W(q,p)
        syms q1 p1
        W = 1/2 * mu * q1^2 + 1/2 * p1^2;
        Wf = sym2fun(W);
        % Display now the contour of W on this mu.
        fc = fcontour(Wf, expansionStruct.contourArea);
        fc.LevelList = Wf(zeros(size(pRange)), pRange);
        xlabel('q');
        ylabel('p');
        title('Orbits of NVHC vs. W(0,pk)');
        hold off;
        % Now get the difference between H(qk_hat,0) and H(0,pk), where
        % qk_hat = pk/sqrt(mu)
        qk_hat = pRange' ./ sqrt(mu);
        W_H_diff = hamiltonian_vhc(qk_hat,zeros(size(qk_hat)),dynHfun) - ...
                   hamiltonian_vhc(zeros(size(pRange')),pRange',dynHfun);
        pk_growing = transpose(pRange(W_H_diff > 0));

        % Update the data
        expansionStruct.f = f;
        expansionStruct.q0 = q0; expansionStruct.mu = mu;
        expansionStruct.W = W; expansionStruct.Wf = Wf;
        expansionStruct.qk_hat = qk_hat; expansionStruct.W_H_diff = W_H_diff;
        expansionStruct.pk_growing = pk_growing;
        % Save this updated structure into the Lyapunov data
        lyapunovStructs{i} = expansionStruct;
    end
    newLyapStruct = lyapunovStructs;
end

