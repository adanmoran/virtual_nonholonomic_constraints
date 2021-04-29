%% Energy Level Events
% Triggers an event in ode45 when the inputted energy E reaches a specific
% value.
% You use this in ode45 by setting the events function handle to
% @(t,qp) energyEvent(t,qp,E,Eval).
%
% Inputs:
% * t = current time value of simulation
% * qp = current value of the phase on the (qu,pu)-plane
% * E = function handle to the energy of the system
% * Eval = energy value at which to trigger the event.

function [value,isterminal,direction] = energyInEvent(t,qp,E,Eval)
    % Trigger when E(q,p) = Eval. The value increases as E -> Eval from
    % below.
    value = E(qp(1),qp(2))-Eval;
    % Don't stop the integration, just record the event
    isterminal = 0;
    % Track only values where the energy is increasing past Eval, not those
    % where you come back down. If you want it to track only decreasing,
    % set direction to -1. If you want both increasing and decreasing, set
    % it to 0.
    direction = 1;
end

