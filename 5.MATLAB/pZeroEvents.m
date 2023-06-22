%% Events for (q,p) = (q,0)
% Events handler for odeXY() to stop execution of integration when the ode
% hits the p=0 axis.
function [position,isterminal,direction] = pZeroEvents(t,qp)
position = qp(2); % We want p = 0
isterminal = 1;  % Halt integration 
direction = 0;   % any time p hits zero we stop
end