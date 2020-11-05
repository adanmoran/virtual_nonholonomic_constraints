%% VLP Optimal Controller $l^\star(\theta)$
% This function defines the time-optimal controller for the VLP, based on
% the results from the paper "Pumping a Swing by Standing and Squatting".
%
% Inputs:
% * theta = atan2(p,q)
% * lu = lower actuator limit for l(theta). Domain is ]0,lb[
% * lb = upper actuator limit for l(theta). Domain is ]lu,inf[
%
% Output:
% * l^\star(theta), the time-optimal controller.
function l = vlp_l_optimal(theta,lu,lb)
% If theta is within ]-pi,-pi/2] or [0,pi/2], we stand.
if theta <= -pi/2 || (0 <= theta && theta <= pi/2)
    l = lu;
% Otherwise, if theta is in ]-pi/2,0[ or ]pi/2,pi], we squat.
else 
    l = lb;
end
end

