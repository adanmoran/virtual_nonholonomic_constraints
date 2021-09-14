%% Energy Level Events
% Triggers an event in ode45 when the inputted energy E reaches a specific
% value.
% You use this in ode45 by setting the events function handle to
% @(t,qp) energyEvent(t,qp,E,Eval).
%
% Inputs:
% * t = current time value of simulation
% * qp = current value of the phase on the (qu,pu)-plane
% * f = function handle, to trigger when f(q,p) = fval
% * fval = value at which to trigger the event.
% * d = direction: +1 for increasing, -1 for decreasing, 0 for either.

function [value,isterminal,direction] = regulationEvent(t,qp,f,fval,d)
    % Trigger when f(q,p) = fval. The value increases as f -> fval from
    % below.
    value = f(qp(1),qp(2))-fval;
    % Stop the integration
    isterminal = 1;
    % Track only values where the energy is increasing past Eval, not those
    % where you come back down. If you want it to track only decreasing,
    % set direction to -1. If you want both increasing and decreasing, set
    % it to 0.
    direction = 1;
end

